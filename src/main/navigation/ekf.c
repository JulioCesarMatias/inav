/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

// Extended Kalman Filter ported from ArduPilot to INAV by Julio Cesar Matias
// EKF based on https://github.com/priseborough/InertialNav

#include "navigation/ekf.h"

#include "build/debug.h"
#include "common/matrix.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "drivers/time.h"
#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
#include "flight/imu.h"
#include "io/gps.h"
#include "io/gps_private.h"
#include "navigation/navigation.h"
#include "navigation/navigation_pos_estimator_private.h"
#include "platform.h"
#include "scheduler/scheduler.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"

typedef struct
{
    fpQuaternion_t quat;        // 0, 1, 2, 3
    fpVector3_t velocity;       // 4, 5, 6
    fpVector3_t position;       // 7, 8, 9
    fpVector3_t gyro_bias;      // 10, 11, 12
    float accel_zbias1;         // 13
    fpVector2_t wind_vel;       // 14, 15
    fpVector3_t earth_magfield; // 16, 17, 18
    fpVector3_t body_magfield;  // 19, 20, 21
    float accel_zbias2;         // 22
    fpVector3_t vel1;           // 23, 24, 25
    float posD1;                // 26
    fpVector3_t vel2;           // 27, 28, 29
    float posD2;                // 30
} state_elements_t;

typedef union
{
    Vector31 statesArray;
    state_elements_t stateStruct;
} ekfStates_U;

// states held by magnetomter fusion across time steps
// magnetometer X,Y,Z measurements are fused across three time steps
// to level computational load as this is an expensive operation
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float magN;
    float magE;
    float magD;
    float magXbias;
    float magYbias;
    float magZbias;
    uint8_t obsIndex;
    fpMatrix3_t DCM;
    fpVector3_t MagPred;
    float R_MAG;
    float SH_MAG[9];
} mag_state_t;

typedef struct
{
    bool bad_xmag : 1;
    bool bad_ymag : 1;
    bool bad_zmag : 1;
    bool bad_airspeed : 1;
    bool bad_sideslip : 1;
} fault_status_t;

PG_REGISTER_WITH_RESET_TEMPLATE(ekfConfig_t, ekfConfig, PG_EKF_CONFIG, 0);

PG_RESET_TEMPLATE(ekfConfig_t, ekfConfig,
                  .ekfEnabled = SETTING_EKF_ENABLED_DEFAULT, );

state_elements_t storedStates[50];     // state vectors stored for the last 50 time steps
state_elements_t statesAtVelTime;      // states at the effective time of velNED measurements
state_elements_t statesAtPosTime;      // states at the effective time of posNE measurements
state_elements_t statesAtHgtTime;      // states at the effective time of hgtMeas measurement
state_elements_t statesAtMagMeasTime;  // filter states at the effective time of compass measurements
state_elements_t statesAtViasMeasTime; // filter states at the effective measurement time
ekfStates_U ekfStates;                 // union of fusion states
mag_state_t mag_state;                 // EKF magnetometer fusion states
navEstimatedPosVel_t ekfPosVel;        // EKF pos and vel to use in navigation modes
fault_status_t faultStatus;            // EKF fusions status
ekfParam_t ekfParam;                   // EKF params to copy diferents default values for Multirotor and Fixed-Wing

fpQuaternion_t correctedDelAngQuat; // quaternion representation of correctedDelAng

Matrix22 P;          // covariance matrix
Matrix22 nextP;      // predicted covariance matrix before addition of process noise to diagonals
Matrix22 KHP;        // intermediate result used for covariance updates
Matrix22 KH;         // intermediate result used for covariance updates
fpMatrix3_t prevTnb; // previous nav to body transformation used for INS earth rotation compensation

Vector6 varInnovVelPos;         // innovation variance output for a group of measurements
Vector6 innovVelPos;            // innovation output for a group of measurements
Vector8 SG;                     // intermediate variables used to calculate predicted covariance matrix
Vector8 SPP;                    // intermediate variables used to calculate predicted covariance matrix
Vector11 SQ;                    // intermediate variables used to calculate predicted covariance  matrix
Vector15 SF;                    // intermediate variables used to calculate predicted covariance matrix
Vector22 processNoise;          // process noise added to diagonals of predicted covariance matrix
Vector31 Kfusion;               // kalman gain vector
fpVector2_t gpsPosNE;           // north, east position measurements (m)
fpVector3_t summedDelAng;       // corrected & summed delta angles about the xyz body axes (rad)
fpVector3_t summedDelVel;       // corrected & summed delta velocities along the XYZ body axes (m/s)
fpVector3_t correctedDelAng;    // delta angles about the xyz body axes corrected for errors (rad)
fpVector3_t correctedDelVel12;  // delta velocities along the XYZ body axes for weighted average of IMU1 and IMU2 corrected for errors (m/s)
fpVector3_t correctedDelVel1;   // delta velocities along the XYZ body axes for IMU1 corrected for errors (m/s)
fpVector3_t correctedDelVel2;   // delta velocities along the XYZ body axes for IMU2 corrected for errors (m/s)
fpVector3_t delAngBiasAtArming; // value of the gyro delta angle bias at arming
fpVector3_t dAngIMU;            // delta angle vector in XYZ body axes measured by the IMU (rad)
fpVector3_t lastAngRate;        // angular rate from previous IMU sample used for trapezoidal integrator
fpVector3_t lastGyroBias;       // previous gyro bias vector used by filter divergence check
fpVector3_t dVelIMU1;           // delta velocity vector in XYZ body axes measured by IMU1 (m/s)
fpVector3_t dVelIMU2;           // delta velocity vector in XYZ body axes measured by IMU2 (m/s)
fpVector3_t lastAccel1;         // acceleration from previous IMU1 sample used for trapezoidal integrator
fpVector3_t lastAccel2;         // acceleration from previous IMU2 sample used for trapezoidal integrator
fpVector3_t velNED;             // north, east, down velocity measurements (m/s)
fpVector3_t velDotNED;          // rate of change of velocity in NED frame
fpVector3_t velDotNEDfilt;      // low pass filtered velDotNED
fpVector3_t earthRateNED;       // earths angular rate vector in NED (rad/s)
fpVector3_t innovMag;           // innovation output from fusion of X,Y,Z compass measurements
fpVector3_t varInnovMag;        // innovation variance output from fusion of X,Y,Z compass measurements
fpVector3_t magData;            // magnetometer flux readings in X,Y,Z body axes
fpVector3_t magBias;            // magnetometer bias vector in XYZ body axes
fpVector3_t magTestRatio;       // sum of squares of magnetometer innovations divided by fail threshold

bool ekf_started;              // boolean true when EKF have been initialised
bool statesInitialised;        // boolean true when filter states have been initialised
bool vehicleArmed;             // boolean to force position and velocity measurements to zero for pre-arm or bench testing
bool prevVehicleArmed;         // value of vehicleArmed from last update
bool covPredStep;              // boolean set to true when a covariance prediction step has been performed
bool yawAligned;               // true when the yaw angle has been aligned
bool newDataMag;               // true when new magnetometer data has arrived
bool newDataHgt;               // true when new height data has arrived
bool newDataGps;               // true when new GPS data has arrived
bool newDataIAS;               // true when new airspeed data has arrived
bool fuseVelData;              // this boolean causes the velNED measurements to be fused
bool fusePosData;              // this boolean causes the posNE measurements to be fused
bool fuseHgtData;              // this boolean causes the hgtMeas measurements to be fused
bool magHealth;                // boolean true if magnetometer has passed innovation  consistency check
bool velHealth;                // boolean true if velocity measurements have passed innovation consistency check
bool posHealth;                // boolean true if position measurements have passed innovation consistency check
bool hgtHealth;                // boolean true if height measurements have passed innovation consistency check
bool iasHealth;                // boolean true if true airspeed has passed innovation consistency check
bool velTimeout;               // boolean true if velocity measurements have failed innovation consistency check and timed out
bool posTimeout;               // boolean true if position measurements have failed innovation consistency check and timed out
bool hgtTimeout;               // boolean true if height measurements have failed innovation consistency check and timed out
bool magTimeout;               // boolean true if magnetometer measurements have failed for too long and have timed out
bool iasTimeout;               // boolean true if true airspeed measurements have failed for too long and have timed out
bool badIMUdata;               // boolean true if the bad IMU data is detected
bool badMag;                   // boolean true if the magnetometer is declared to be producing bad data
bool onGround;                 // boolean true when the flight vehicle is on the ground (not flying)
bool prevOnGround;             // value of onGround from previous update
bool earthRateNEDReset = true; // true for initial earth-rate calc
bool inhibitWindStates = true; // true when wind states and covariances are to remain constant
bool inhibitMagStates = true;  // true when magnetic field states and covariances are to remain constant
bool firstArmComplete;         // true when first transition out of disarmed has been performed after start up
bool manoeuvring;              // boolean true when the flight vehicle is performing horizontal changes in velocity
bool highYawRate;              // true when the vehicle is doing rapid yaw rotation where gyro scel factor errors could cause loss of heading reference
bool iasDataWaiting;           // true when new airspeed data is waiting to be fused

uint8_t msecMagDelay = 40;  // magnetometer measurement delay (msec)
uint8_t msecHgtDelay = 60;  // effective average delay of height measurements rel to (msec)
uint8_t msecIASDelay = 240; // airspeed measurement delay (msec)
uint8_t msecMagAvg = 100;   // average number of msec between magnetometer measurements
uint8_t msecHgtAvg = 100;   // average number of msec between height measurements
uint8_t msecGpsAvg = 100;   // average number of msec between GPS measurements
uint8_t msecBetaAvg = 100;  // average number of msec between synthetic sideslip measurements
uint8_t magUpdateCount;     // count of the number of minor state corrections using Magnetometer data
uint8_t magUpdateCountMax;  // limit on the number of minor state corrections using Magnetometer data
uint8_t gpsUpdateCount;     // count of the number of minor state corrections using GPS data
uint8_t gpsUpdateCountMax;  // limit on the number of minor state corrections using GPS data
uint8_t hgtUpdateCount;     // count of the number of minor state corrections using Baro data
uint8_t hgtUpdateCountMax;  // limit on the number of minor state corrections using Baro data
uint8_t storeIndex;         // state vector storage index

float dt;                          // time lapsed since the last covariance prediction (sec)
float dtIMU;                       // time lapsed since the last IMU measurement (sec)
float accNavMag;                   // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
float accNavMagHoriz;              // magnitude of navigation accel in horizontal plane (m/s^2)
float hgtMeas;                     // height measurement relative to reference point  (m)
float hgtRate;                     // state for rate of change of height filter
float IMU1_weighting = 0.5f;       // Weighting applied to use of IMU1. Varies between 0 and 1.
float gyroBiasNoiseScaler = 2.0f;  // scale factor applied to gyro bias state process noise when on ground
float gpsNoiseScaler = 1.0f;       // Used to scale the GPS measurement noise and consistency gates to compensate for operation with small satellite counts
float magVarRateScale = 0.05f;     // scale factor applied to magnetometer variance due to angular rate
float gpsNEVelVarAccScale = 0.05f; // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
float gpsDVelVarAccScale = 0.07f;  // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
float gpsPosVarAccScale = 0.05f;   // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
float covTimeStepMax = 0.02f;      // maximum time allowed between covariance predictions
float covDelAngMax = 0.05f;        // maximum delta angle between covariance predictions
float velTestRatio;                // sum of squares of GPS velocity innovation divided by fail threshold
float posTestRatio;                // sum of squares of GPS position innovation divided by fail threshold
float hgtTestRatio;                // sum of squares of baro height innovation divided by fail threshold
float iasTestRatio;                // sum of squares of true airspeed innovation divided by fail threshold
float gpsIncrStateDelta[10];       // vector of corrections to attitude, velocity and position to be applied over the period between the current and next GPS measurement
float hgtIncrStateDelta[10];       // vector of corrections to attitude, velocity and position to be applied over the period between the current and next height measurement
float magIncrStateDelta[10];       // vector of corrections to attitude, velocity and position to be applied over the period between the current and next magnetometer measurement
float magUpdateCountMaxInv;        // floating point inverse of magFilterCountMax
float gpsUpdateCountMaxInv;        // floating point inverse of gpsFilterCountMax
float hgtUpdateCountMaxInv;        // floating point inverse of hgtFilterCountMax
float scaledDeltaGyrBiasLgth;      // scaled delta gyro bias vector length used to test for filter divergence
float ekfMagDeclination;           // declination calced based on the gps coordinates
float yawRateFilt;                 // filtered yaw rate used to determine when the vehicle is doing rapid yaw rotation where gyro scel factor errors could cause loss of heading reference
float hgtInnovFiltState;           // state used for fitering of the height innovations used for pre-flight checks
float innovVias;                   // innovation output from fusion of airspeed measurements
float varInnovVias;                // innovation variance output from fusion of airspeed measurements
float ViasMeas;                    // true airspeed measurement (m/s)

timeMs_t statetimeStamp[50];          // time stamp for each state vector stored
timeMs_t ekfStartTime_ms;             // ekf start time
timeMs_t imuSampleTime_ms;            // time that the last IMU value was taken
timeMs_t lastStateStoreTime_ms;       // time of last state vector storage
timeMs_t lastHgtTime_ms;              // time of last height update (msec) used to calculate timeout
timeMs_t lastVelPassTime_ms;          // time stamp when GPS velocity measurement last failed covaraiance consistency check (msec)
timeMs_t lastPosPassTime_ms;          // time stamp when GPS position measurement last failed covaraiance consistency check (msec)
timeMs_t lastHgtPassTime_ms;          // time stamp when height measurement last failed covaraiance consistency check (msec)
timeMs_t lastHealthyMagTime_ms;       // time the magnetometer was last declared healthy
timeMs_t lastFixTime_ms;              // time of last GPS fix used to determine if new data has arrived
timeMs_t secondLastFixTime_ms;        // time of second last GPS fix used to determine  how long since last update
timeMs_t lastDivergeTime_ms;          // time in msec divergence of filter last detected
timeMs_t airborneDetectTime_ms;       // last time flight movement was detected
timeMs_t magFailTimeLimit_ms = 10000; // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
timeMs_t hgtRetryTime_ms = 10000;     // height retry time with vertical velocity measurement (msec)
timeMs_t iasRetryTime = 5000;         // True airspeed timeout and retry interval (msec)
timeMs_t lastAirspeedUpdate_ms;       // last time airspeed was updated
timeMs_t lastIASPassTime;             // time stamp when airspeed measurement last passed innovation consistency check (msec)
timeMs_t BETAmsecPrev;                // time stamp of last synthetic sideslip fusion step

timeUs_t lastMagUpdate_us;   // last time compass was updated
timeUs_t lastHgtMeasTime_us; // time of last height measurement used to determine if new data has arrived

// Forces EKF to disable if the following settings are enabled. The reason for this procedure is because this EKF still does not fuse these sensors.
bool forceEKFDisable(void)
{
    return sensors(SENSOR_RANGEFINDER) || sensors(SENSOR_OPFLOW) || STATE(ROVER) || STATE(BOAT);
}

// This function is used to force the PID Task to run at max 400Hz (F4) or 1Khz (H7) when the EKF is active.
bool ekf_getLoopTime(uint16_t *newLoopTime)
{
    if (ekfConfig()->ekfEnabled && !forceEKFDisable())
    {
#if defined(STM32H7)
        *newLoopTime = MAX(1000, gyroConfig()->looptime);
#else
        *newLoopTime = MAX(2500, gyroConfig()->looptime);
#endif

        return true;
    }

    return false;
}

void ekf_setMagDeclination(float declination)
{
    ekfMagDeclination = DEGREES_TO_RADIANS(declination);
}

// return true if we should use the magnetometer sensor
bool ekf_useCompass(void)
{
    return sensors(SENSOR_MAG) && compassIsCalibrationComplete();
}

// return true if we should use the barometer sensor
bool ekf_useBarometer(void)
{
    return sensors(SENSOR_BARO) && baro.calibrationFinished;
}

// return true if we should use the airspeed sensor
bool ekf_useAirspeed(void)
{
    return sensors(SENSOR_PITOT) && pitot.calibrationFinished;
}

// 120ms for UBLOX7, UBLOXM8, UBLOXM9 and UBLOXM10
// 110ms for GPS MSP
timeMs_t ekf_getGPSDelay(void)
{
    return gpsConfig()->provider == GPS_MSP ? 110 : 120;
}

// wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
float wrap_PI(float angle_in_radians)
{
    if (angle_in_radians > 10 * M_PIf || angle_in_radians < -10 * M_PIf)
    {
        // for very large numbers use modulus
        angle_in_radians = fmodf(angle_in_radians, 2 * M_PIf);
    }

    while (angle_in_radians > M_PIf)
    {
        angle_in_radians -= 2 * M_PIf;
    }

    while (angle_in_radians < -M_PIf)
    {
        angle_in_radians += 2 * M_PIf;
    }

    return angle_in_radians;
}

// align the NE earth magnetic field states with the published declination
void ekf_alignMagStateDeclination(void)
{
    // get the magnetic declination
    const float magDecAng = ekf_useCompass() ? ekfMagDeclination : 0.0f;

    // rotate the NE values so that the declination matches the published value
    const fpVector3_t initMagNED = ekfStates.stateStruct.earth_magfield;
    float magLengthNE = calc_length_pythagorean_2D(initMagNED.x, initMagNED.y);
    ekfStates.stateStruct.earth_magfield.x = magLengthNE * cos_approx(magDecAng);
    ekfStates.stateStruct.earth_magfield.y = magLengthNE * sin_approx(magDecAng);
}

// return the distance in meters in North/East plane as a N/E vector
fpVector2_t ekf_LocationDiff(const gpsOrigin_t loc1, const gpsLocation_t loc2)
{
    fpVector2_t loc_diff;

    float scale = cos_approx(DEGREES_TO_RADIANS(loc1.lat * 1.0e-7f));

    loc_diff.x = (loc2.lat - loc1.lat) * 0.011131884502145034f;
    loc_diff.y = (loc2.lon - loc1.lon) * 0.011131884502145034f * constrainf(scale, 0.01f, 1.0f);

    return loc_diff;
}

/*
return the filter fault status as a bitmasked integer
 0 = quaternions are NaN
 1 = velocities are NaN
 2 = badly conditioned X magnetometer fusion
 3 = badly conditioned Y magnetometer fusion
 5 = badly conditioned Z magnetometer fusion
 6 = badly conditioned airspeed fusion
 7 = badly conditioned synthetic sideslip fusion
 7 = filter is not initialised
*/
uint16_t ekf_getFilterFaults(void)
{
    return ((isnan(ekfStates.stateStruct.quat.q0) || isnan(ekfStates.stateStruct.quat.q1) || isnan(ekfStates.stateStruct.quat.q2) || isnan(ekfStates.stateStruct.quat.q3)) << 0 |
            (isnan(ekfStates.stateStruct.velocity.x) || isnan(ekfStates.stateStruct.velocity.y) || isnan(ekfStates.stateStruct.velocity.z)) << 1 |
            faultStatus.bad_xmag << 2 |
            faultStatus.bad_ymag << 3 |
            faultStatus.bad_zmag << 4 |
            faultStatus.bad_airspeed << 5 |
            faultStatus.bad_sideslip << 6 |
            !statesInitialised << 7);
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void ekf_getVariances(float *velVar, float *posVar, float *hgtVar, float *magVar, float *tasVar)
{
    *velVar = sqrtf(velTestRatio);
    *posVar = sqrtf(posTestRatio);
    *hgtVar = sqrtf(hgtTestRatio);
    *magVar = calc_length_pythagorean_3D(sqrtf(magTestRatio.x), sqrtf(magTestRatio.y), sqrtf(magTestRatio.z));
    *tasVar = sqrtf(iasTestRatio);
}

// zero specified range of rows in the state covariance matrix
void ekf_zeroRows(Matrix22 covMat, uint8_t first, uint8_t last)
{
    for (uint8_t row = first; row <= last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0]) * 22);
    }
}

// zero specified range of columns in the state covariance matrix
void ekf_zeroCols(Matrix22 covMat, uint8_t first, uint8_t last)
{
    for (uint8_t row = 0; row <= 21; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0]) * (1 + last - first));
    }
}

// Check basic filter health metrics and return a consolidated health status
bool ekf_healthy(void)
{
    const uint16_t faultInt = ekf_getFilterFaults();

    if (faultInt > 0)
    {
        return false;
    }

    if (velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1)
    {
        // all three metrics being above 1 means the filter is extremely unhealthy.
        return false;
    }

    // barometer and position innovations must be within limits when on-ground
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (!vehicleArmed && (!hgtHealth || fabsf(hgtInnovFiltState) > 1.0f || horizErrSq > 2.0f))
    {
        return false;
    }

    // all OK
    return true;
}

bool ekf_HealthyToUse(void)
{
    // check if the EKF is configured to use and if the healthy is ok
    return ekf_started && ekfConfig()->ekfEnabled && ekf_healthy();
}

// calculate the NED earth spin vector in rad/sec
void ekf_calcEarthRateNED(fpVector3_t *omega, float latitude)
{
    float lat_rad = DEGREES_TO_RADIANS(latitude);
    omega->x = EARTH_RATE * cos_approx(lat_rad);
    omega->y = 0.0f;
    omega->z = EARTH_RATE * sin_approx(lat_rad);
}

// recall state vector stored at closest time to the one specified by msec
void ekf_RecallStates(state_elements_t *statesForFusion, timeMs_t msec)
{
    timeMs_t timeDelta;
    timeMs_t bestTimeDelta = 200;
    uint8_t bestStoreIndex = 0;

    for (uint8_t i = 0; i <= 49; i++)
    {
        timeDelta = msec - statetimeStamp[i];
        if (timeDelta < bestTimeDelta)
        {
            bestStoreIndex = i;
            bestTimeDelta = timeDelta;
        }
    }

    if (bestTimeDelta < 200) // only output stored state if < 200 msec retrieval error
    {
        *statesForFusion = storedStates[bestStoreIndex];
    }
    else // otherwise output current state
    {
        *statesForFusion = ekfStates.stateStruct;
    }
}

// update IMU delta angle and delta velocity measurements
void ekf_readIMUData(void)
{
    fpVector3_t angRate; // angular rate vector in XYZ body axes measured by the IMU (rad/s)
    fpVector3_t accel1;  // acceleration vector in XYZ body axes measured by IMU1 (m/s^2)
    fpVector3_t accel2;  // acceleration vector in XYZ body axes measured by IMU2 (m/s^2)

    // the imu sample time is sued as a common time reference throughout the filter
    imuSampleTime_ms = millis();

    gyroGetMeasuredRotationRate(&angRate); // Calculate gyro rate in body frame in rad/s
    accGetMeasuredAcceleration(&accel1);   // Calculate accel in body frame in cm/s

    // convert the accel in body frame in cm/s to m/s
    accel1.x = CENTIMETERS_TO_METERS(accel1.x);
    accel1.y = CENTIMETERS_TO_METERS(accel1.y);
    accel1.z = CENTIMETERS_TO_METERS(accel1.z);

    accel2 = accel1;

    // trapezoidal integration
    // blended gyro
    dAngIMU.x = (angRate.x + lastAngRate.x) * dtIMU * 0.5f;
    dAngIMU.y = (angRate.y + lastAngRate.y) * dtIMU * 0.5f;
    dAngIMU.z = (angRate.z + lastAngRate.z) * dtIMU * 0.5f;
    lastAngRate.x = angRate.x;
    lastAngRate.y = angRate.y;
    lastAngRate.z = angRate.z;

    // accel IMU1
    dVelIMU1.x = (accel1.x + lastAccel1.x) * dtIMU * 0.5f;
    dVelIMU1.y = (accel1.y + lastAccel1.y) * dtIMU * 0.5f;
    dVelIMU1.z = (accel1.z + lastAccel1.z) * dtIMU * 0.5f;
    lastAccel1.x = accel1.x;
    lastAccel1.y = accel1.y;
    lastAccel1.z = accel1.z;

    // accel IMU2
    dVelIMU2.x = (accel2.x + lastAccel2.x) * dtIMU * 0.5f;
    dVelIMU2.y = (accel2.y + lastAccel2.y) * dtIMU * 0.5f;
    dVelIMU2.z = (accel2.z + lastAccel2.z) * dtIMU * 0.5f;
    lastAccel2.x = accel2.x;
    lastAccel2.y = accel2.y;
    lastAccel2.z = accel2.z;
}

// check for new magnetometer data and update store measurements if available
void ekf_readMagData(void)
{
    // check to see if magnetometer measurement has changed so we know if a new measurement has arrived
    if (ekf_useCompass() && (compassLastUpdate() != lastMagUpdate_us))
    {
        // store time of last measurement update
        lastMagUpdate_us = compassLastUpdate();

        // read compass and data assign to bias and uncorrected measurement is miliGauss
        magBias.x = -(compassConfig()->magZero.raw[X] / 1024 * compassConfig()->magGain[X]) * 0.001f;
        magBias.y = -(compassConfig()->magZero.raw[Y] / 1024 * compassConfig()->magGain[Y]) * 0.001f;
        magBias.z = -(compassConfig()->magZero.raw[Z] / 1024 * compassConfig()->magGain[Z]) * 0.001f;
        magData.x = mag.magADC[X] * 0.001f;
        magData.y = mag.magADC[Y] * 0.001f;
        magData.z = mag.magADC[Z] * 0.001f;

        // get states stored at time closest to measurement time after allowance for measurement delay
        ekf_RecallStates(&statesAtMagMeasTime, imuSampleTime_ms - msecMagDelay);

        // let other processes know that new compass data has arrived
        newDataMag = true;
    }
    else
    {
        newDataMag = false;
    }
}

// check for new altitude measurement data and update stored measurement if available
void ekf_readHgtData(void)
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    if (ekf_useBarometer() && (posEstimator.baro.lastUpdateTime != lastHgtMeasTime_us))
    {
        // time stamp used to check for new measurement
        lastHgtMeasTime_us = posEstimator.baro.lastUpdateTime;

        // time stamp used to check for timeout
        lastHgtTime_ms = imuSampleTime_ms;

        // get measurement and set flag to let other functions know new data has arrived
        hgtMeas = CENTIMETERS_TO_METERS(posEstimator.baro.rawAlt - posEstimator.baro.altOffSet);

        newDataHgt = true;

        // get states that wer stored at the time closest to the measurement time, taking measurement delay into account
        ekf_RecallStates(&statesAtHgtTime, US2MS(lastHgtMeasTime_us) - msecHgtDelay);
    }
    else
    {
        newDataHgt = false;
    }
}

// check for new valid GPS data and update stored measurement if available
void ekf_readGpsData(void)
{
    if (gpsSol.fixType < GPS_FIX_3D)
    {
        return;
    }

    // check to see if GPS measurement has changed so we know if a new measurement has arrived
    if (gpsState.lastMessageMs != lastFixTime_ms)
    {
        // store fix time from previous read
        secondLastFixTime_ms = lastFixTime_ms;

        // get current fix time
        lastFixTime_ms = gpsState.lastMessageMs;

        // set flag that lets other functions know that new GPS data has arrived
        newDataGps = true;

        // get state vectors that were stored at the time that is closest to when the the GPS measurement time after accounting for measurement delays
        ekf_RecallStates(&statesAtVelTime, imuSampleTime_ms - ekf_getGPSDelay());
        ekf_RecallStates(&statesAtPosTime, imuSampleTime_ms - ekf_getGPSDelay());

        // read the NED velocity from the GPS
        velNED.x = CENTIMETERS_TO_METERS(gpsSol.velNED[X]);
        velNED.y = CENTIMETERS_TO_METERS(gpsSol.velNED[Y]);
        velNED.z = CENTIMETERS_TO_METERS(gpsSol.velNED[Z]);

        // check if we have enough GPS satellites and increase the gps noise scaler if we don't
        if (gpsSol.numSat >= 6)
        {
            gpsNoiseScaler = 1.0f;
        }
        else if (gpsSol.numSat == 5)
        {
            gpsNoiseScaler = 1.4f;
        }
        else if (gpsSol.numSat <= 4)
        {
            gpsNoiseScaler = 2.0f;
        }

        // Set the EKF origin and magnetic field declination if not previously set and GPS checks have passed
        if (posControl.gpsOrigin.valid && earthRateNEDReset)
        {
            // define Earth rotation vector in the NED navigation frame
            ekf_calcEarthRateNED(&earthRateNED, posControl.gpsOrigin.lat * 1.0e-7f);
            // align the NE earth magnetic field states with the published declination
            ekf_alignMagStateDeclination();
            earthRateNEDReset = false;
        }

        // Convert to local coordinates if we have an origin.
        if (posControl.gpsOrigin.valid)
        {
            gpsPosNE = ekf_LocationDiff(posControl.gpsOrigin, gpsSol.llh);
        }
    }
}

// check for new airspeed data and update stored measurements if available
void ekf_readAirSpdData(void)
{
    // if airspeed reading is valid and is set by the user to be used and has been updated then we take a new reading
    if (ekf_useAirspeed() && pitot.lastSeenHealthyMs != lastAirspeedUpdate_ms)
    {
        lastAirspeedUpdate_ms = pitot.lastSeenHealthyMs;
        ViasMeas = CENTIMETERS_TO_METERS(getAirspeedEstimate());
        newDataIAS = true;
        ekf_RecallStates(&statesAtViasMeasTime, lastAirspeedUpdate_ms - msecIASDelay);
    }
    else
    {
        newDataIAS = false;
    }
}

// resets position states to last GPS measurement or to zero if in vehicleArmed
void ekf_ResetPosition(void)
{
    if (!vehicleArmed)
    {
        ekfStates.stateStruct.position.x = 0.0f;
        ekfStates.stateStruct.position.y = 0.0f;
    }
    else if (gpsSol.fixType >= GPS_FIX_3D)
    {
        // write to state vector and compensate for GPS latency
        ekfStates.stateStruct.position.x = gpsPosNE.x + 0.001f * velNED.x * (float)ekf_getGPSDelay();
        ekfStates.stateStruct.position.y = gpsPosNE.y + 0.001f * velNED.y * (float)ekf_getGPSDelay();
        // the estimated states at the last GPS measurement are set equal to the GPS measurement to prevent transients on the first fusion
        statesAtPosTime.position.x = gpsPosNE.x;
        statesAtPosTime.position.y = gpsPosNE.y;
    }
    // stored horizontal position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i = 0; i <= 49; i++)
    {
        storedStates[i].position.v[X] = ekfStates.stateStruct.position.v[X];
        storedStates[i].position.v[Y] = ekfStates.stateStruct.position.v[Y];
    }
}

// resets velocity states to last GPS measurement or to zero if in vehicleArmed
void ekf_ResetVelocity(void)
{
    if (!vehicleArmed)
    {
        vectorZero(&ekfStates.stateStruct.velocity);
        vectorZero(&ekfStates.stateStruct.vel1);
        vectorZero(&ekfStates.stateStruct.vel2);
    }
    else if (gpsSol.fixType >= GPS_FIX_3D)
    {
        // reset filter velocity states
        ekfStates.stateStruct.velocity.x = velNED.x;
        ekfStates.stateStruct.velocity.y = velNED.y;
        ekfStates.stateStruct.vel1.x = velNED.x;
        ekfStates.stateStruct.vel1.y = velNED.y;
        ekfStates.stateStruct.vel2.x = velNED.x;
        ekfStates.stateStruct.vel2.y = velNED.y;
        // reset stored velocity states to prevent subsequent GPS measurements from being rejected
        for (uint8_t i = 0; i <= 49; i++)
        {
            storedStates[i].velocity.x = velNED.x;
            storedStates[i].velocity.y = velNED.y;
        }
    }
}

// reset the vertical position state using the last height measurement
void ekf_ResetHeight(void)
{
    // read the altimeter
    ekf_readHgtData();
    // write to the state vector
    ekfStates.stateStruct.position.z = hgtMeas; // down position from blended accel data
    ekfStates.stateStruct.posD1 = hgtMeas;      // down position from IMU1 accel data
    ekfStates.stateStruct.posD2 = hgtMeas;      // down position from IMU2 accel data
    // Reset the vertical velocity state using GPS vertical velocity if we are airborne (use arm status as a surrogate)
    if (vehicleArmed && gpsSol.fixType >= GPS_FIX_3D)
    {
        ekfStates.stateStruct.velocity.z = velNED.z;
    }
    // reset stored vertical position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i = 0; i <= 49; i++)
    {
        storedStates[i].position.z = hgtMeas;
    }
}

// store states in a history array along with time stamp
void ekf_StoreStates(void)
{
    // Don't need to store states more often than every 10 msec
    if (imuSampleTime_ms - lastStateStoreTime_ms >= 10)
    {
        lastStateStoreTime_ms = imuSampleTime_ms;
        if (storeIndex > 49)
        {
            storeIndex = 0;
        }
        storedStates[storeIndex] = ekfStates.stateStruct;
        statetimeStamp[storeIndex] = lastStateStoreTime_ms;
        storeIndex = storeIndex + 1;
    }
}

// reset the stored state history and store the current state
void ekf_StoreStatesReset(void)
{
    // clear stored state history
    memset(&storedStates[0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
    // store current state vector in first column
    storeIndex = 0;
    storedStates[storeIndex] = ekfStates.stateStruct;
    statetimeStamp[storeIndex] = imuSampleTime_ms;
    storeIndex = storeIndex + 1;
}

// zero stored variables - this needs to be called before a full filter initialisation
void ekf_InitialiseVariables(void)
{
    // initialise time stamps
    imuSampleTime_ms = millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    lastDivergeTime_ms = imuSampleTime_ms;
    lastHgtTime_ms = imuSampleTime_ms;
    lastVelPassTime_ms = imuSampleTime_ms;
    lastPosPassTime_ms = imuSampleTime_ms;
    lastHgtPassTime_ms = imuSampleTime_ms;
    lastStateStoreTime_ms = imuSampleTime_ms;
    lastFixTime_ms = imuSampleTime_ms;
    secondLastFixTime_ms = imuSampleTime_ms;
    lastMagUpdate_us = MS2US(imuSampleTime_ms);
    lastHgtMeasTime_us = MS2US(imuSampleTime_ms);
    lastAirspeedUpdate_ms = imuSampleTime_ms;
    lastIASPassTime = imuSampleTime_ms;
    BETAmsecPrev = imuSampleTime_ms;

    if (STATE(FIXED_WING_LEGACY))
    {
        ekfParam.accBiasUncertainty = FW_INIT_ACCEL_BIAS_UNCERTAINTY;
        ekfParam.accNoise = FW_ACC_PNOISE_DEFAULT;
        ekfParam.gyroNoise = FW_GYRO_PNOISE_DEFAULT;
        ekfParam.accelBiasProcessNoise = FW_ABIAS_PNOISE_DEFAULT;
        ekfParam.gyroBiasProcessNoise = FW_GBIAS_PNOISE_DEFAULT;
        ekfParam.magNoise = FW_MAG_NOISE_DEFAULT;
        ekfParam.magEarthProcessNoise = FW_MAGE_PNOISE_DEFAULT;
        ekfParam.magBodyProcessNoise = FW_MAGB_PNOISE_DEFAULT;
        ekfParam.magInnovGate = FW_MAG_GATE_DEFAULT;
        ekfParam.hgtInnovGate = FW_HGT_GATE_DEFAULT;
        ekfParam.baroAltNoise = FW_ALT_NOISE_DEFAULT;
        ekfParam.gpsHorizVelNoise = FW_VELNE_NOISE_DEFAULT;
        ekfParam.gpsVertVelNoise = FW_VELD_NOISE_DEFAULT;
        ekfParam.gpsHorizPosNoise = FW_POSNE_NOISE_DEFAULT;
        ekfParam.gpsPosInnovGate = FW_POS_GATE_DEFAULT;
        ekfParam.gpsGlitchAccelMax = FW_GLITCH_ACCEL_DEFAULT;
        ekfParam.gpsGlitchRadiusMax = FW_GLITCH_RADIUS_DEFAULT;
        ekfParam.gpsVelInnovGate = FW_VEL_GATE_DEFAULT;
        ekfParam.iasNoise = 1.4f;
        ekfParam.iasInnovGate = 10.0f;
        ekfParam.windVelProcessNoise = 0.1f;
        ekfParam.wndVarHgtRateScale = 0.5f;
    }
    else
    {
        ekfParam.accBiasUncertainty = MC_INIT_ACCEL_BIAS_UNCERTAINTY;
        ekfParam.accNoise = MC_ACC_PNOISE_DEFAULT;
        ekfParam.gyroNoise = MC_GYRO_PNOISE_DEFAULT;
        ekfParam.accelBiasProcessNoise = MC_ABIAS_PNOISE_DEFAULT;
        ekfParam.gyroBiasProcessNoise = MC_GBIAS_PNOISE_DEFAULT;
        ekfParam.magNoise = MC_MAG_NOISE_DEFAULT;
        ekfParam.magEarthProcessNoise = MC_MAGE_PNOISE_DEFAULT;
        ekfParam.magBodyProcessNoise = MC_MAGB_PNOISE_DEFAULT;
        ekfParam.magInnovGate = MC_MAG_GATE_DEFAULT;
        ekfParam.hgtInnovGate = MC_HGT_GATE_DEFAULT;
        ekfParam.baroAltNoise = MC_ALT_NOISE_DEFAULT;
        ekfParam.gpsHorizVelNoise = MC_VELNE_NOISE_DEFAULT;
        ekfParam.gpsVertVelNoise = MC_VELD_NOISE_DEFAULT;
        ekfParam.gpsHorizPosNoise = MC_POSNE_NOISE_DEFAULT;
        ekfParam.gpsPosInnovGate = MC_POS_GATE_DEFAULT;
        ekfParam.gpsGlitchAccelMax = MC_GLITCH_ACCEL_DEFAULT;
        ekfParam.gpsGlitchRadiusMax = MC_GLITCH_RADIUS_DEFAULT;
        ekfParam.gpsVelInnovGate = MC_VEL_GATE_DEFAULT;
        ekfParam.iasNoise = 1.4f;
        ekfParam.iasInnovGate = 10.0f;
        ekfParam.windVelProcessNoise = 0.1f;
        ekfParam.wndVarHgtRateScale = 0.5f;
    }

    badIMUdata = false;
    velTimeout = false;
    posTimeout = false;
    hgtTimeout = false;
    magTimeout = false;
    firstArmComplete = false;
    highYawRate = false;

    storeIndex = 0;
    dtIMU = 0.0f;
    dt = 0.0f;
    hgtMeas = 0.0f;
    yawRateFilt = 0.0f;
    hgtInnovFiltState = 0.0f;

    vectorZero(&lastGyroBias);
    vectorZero(&lastAngRate);
    vectorZero(&lastAccel1);
    vectorZero(&lastAccel2);
    vectorZero(&velDotNEDfilt);
    vectorZero(&summedDelAng);
    vectorZero(&summedDelVel);
    vectorZero(&velNED);
    vectorZero(&delAngBiasAtArming);
    gpsPosNE.x = 0.0f;
    gpsPosNE.y = 0.0f;
    mag_state.q0 = 1.0f;
    identityMatrix(&mag_state.DCM);
    zeroMatrix(&prevTnb);

    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    memset(&storedStates[0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
    memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
    memset(&hgtIncrStateDelta[0], 0, sizeof(hgtIncrStateDelta));
    memset(&magIncrStateDelta[0], 0, sizeof(magIncrStateDelta));
    memset(&faultStatus, 0, sizeof(faultStatus));
}

// initialise the covariance matrix
void ekf_CovarianceInit(void)
{
    // zero the matrix
    for (uint8_t i = 1; i <= 21; i++)
    {
        for (uint8_t j = 0; j <= 21; j++)
        {
            P[i][j] = 0.0f;
        }
    }

    // quaternions - TODO better maths for initial quaternion covariances that uses roll, pitch and yaw
    P[0][0] = 1.0e-9f;
    P[1][1] = 0.25f * sq(DEGREES_TO_RADIANS(10.0f));
    P[2][2] = 0.25f * sq(DEGREES_TO_RADIANS(10.0f));
    P[3][3] = 0.25f * sq(DEGREES_TO_RADIANS(10.0f));

    // velocities
    P[4][4] = sq(0.7f);
    P[5][5] = P[4][4];
    P[6][6] = sq(0.7f);

    // positions
    P[7][7] = sq(15.0f);
    P[8][8] = P[7][7];
    P[9][9] = sq(ekfParam.baroAltNoise);

    // delta angle biases
    P[10][10] = sq(DEGREES_TO_RADIANS(INIT_GYRO_BIAS_UNCERTAINTY * dtIMU));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];

    // Z delta velocity bias
    P[13][13] = sq(ekfParam.accBiasUncertainty * dtIMU);

    // wind velocities
    P[14][14] = 0.0f;
    P[15][15] = P[14][14];

    // earth magnetic field
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];

    // body magnetic field
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
}

// initialise the earth magnetic field states using declination, suppled
// roll/pitch and magnetometer measurements and return initial attitude
// quaternion if no magnetometer data, do not update amgentic field states and
// assume zero yaw angle
fpQuaternion_t ekf_calcQuatAndFieldStates(float roll, float pitch)
{
    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    fpMatrix3_t Tbn;
    fpVector3_t initMagNED;
    fpQuaternion_t initQuat;

    if (ekf_useCompass())
    {
        // calculate rotation matrix from body to NED frame
        matrixFromEuler(&Tbn, roll, pitch, 0.0f);

        // read the magnetometer data
        ekf_readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = multiplyMatrixByVector(Tbn, magData);

        // calculate heading of mag field rel to body heading
        float magHeading = atan2_approx(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = ekf_useCompass() ? ekfMagDeclination : 0.0f;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAligned = true;

        // calculate initial filter quaternion states using yaw from magnetometer if mag heading healthy otherwise use existing heading
        if (!badMag)
        {
            quaternionFromEuler(&initQuat, roll, pitch, yaw);
        }
        else
        {
            initQuat = ekfStates.stateStruct.quat;
        }

        // calculate initial Tbn matrix and rotate Mag measurements into NED to set initial NED magnetic field states
        quaternionToRotationMatrix(initQuat, &Tbn);
        ekfStates.stateStruct.earth_magfield = multiplyMatrixByVector(Tbn, magData);

        // align the NE earth magnetic field states with the published declination
        ekf_alignMagStateDeclination();

        // clear bad magnetometer status
        badMag = false;
    }
    else
    {
        quaternionFromEuler(&initQuat, roll, pitch, 0.0f);
        yawAligned = false;
    }

    // return attitude quaternion
    return initQuat;
}

// this function is used to initialise the EKF filter whilst moving, using the AHRS.
bool ekf_InitialiseFilterDynamic(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (STATE(FIXED_WING_LEGACY) && gpsSol.fixType < GPS_FIX_3D)
    {
        statesInitialised = false;
        return false;
    }

    // Wait a while for the sensors and DCM AHRS to stabilize
    if (millis() - ekfStartTime_ms < 3000U)
    {
        return false;
    }

    // Set re-used variables to zero
    ekf_InitialiseVariables();

    // get initial time deltat between IMU measurements (sec)
    dtIMU = US2S(getLooptime());

    // set number of updates over which gps and baro measurements are applied to the velocity and position states
    magUpdateCountMaxInv = (dtIMU * 1000.0f) / msecMagAvg;
    magUpdateCountMax = (uint8_t)(1.0f / magUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMU * 1000.0f) / msecHgtAvg;
    hgtUpdateCountMax = (uint8_t)(1.0f / hgtUpdateCountMaxInv);
    gpsUpdateCountMaxInv = (dtIMU * 1000.0f) / msecGpsAvg;
    gpsUpdateCountMax = (uint8_t)(1.0f / gpsUpdateCountMaxInv);

    // initialise IMU pre-processing states
    ekf_readIMUData();

    // read the magnetometer data
    ekf_readMagData();

    // calculate initial orientation and earth magnetic field states
    ekfStates.stateStruct.quat = ekf_calcQuatAndFieldStates(DECIDEGREES_TO_RADIANS(attitude.values.roll), DECIDEGREES_TO_RADIANS(attitude.values.pitch));

    // write to state vector
    vectorZero(&ekfStates.stateStruct.gyro_bias);
    ekfStates.stateStruct.accel_zbias1 = 0.0f;
    ekfStates.stateStruct.accel_zbias2 = 0.0f;
    ekfStates.stateStruct.wind_vel.x = 0.0f;
    ekfStates.stateStruct.wind_vel.y = 0.0f;
    ekfStates.stateStruct.body_magfield = magBias;

    // read the GPS and set the position and velocity states
    ekf_readGpsData();
    ekf_ResetVelocity();
    ekf_ResetPosition();

    // read the barometer and set the height state
    ekf_readHgtData();
    ekf_ResetHeight();

    // set stored states to current state
    ekf_StoreStatesReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // define Earth rotation vector in the NED navigation frame
    ekf_calcEarthRateNED(&earthRateNED, posControl.gpsOrigin.lat * 1.0e-7f);

    // initialise the covariance matrix
    ekf_CovarianceInit();

    return true;
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void ekf_ForceSymmetry(void)
{
    for (uint8_t i = 1; i <= 21; i++)
    {
        for (uint8_t j = 0; j <= i - 1; j++)
        {
            float temp = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to prevent ill-conditioning
void ekf_ConstrainVariances(void)
{
    for (uint8_t i = 0; i <= 3; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0f); // quaternions

    for (uint8_t i = 4; i <= 6; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e3f); // velocities

    for (uint8_t i = 7; i <= 9; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e6f); // positions

    for (uint8_t i = 10; i <= 12; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, sq(0.175f * dtIMU)); // delta angle biases

    P[13][13] = constrainf(P[13][13], 0.0f, sq(10.0f * dtIMU)); // delta velocity bias

    for (uint8_t i = 14; i <= 15; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e3f); // earth magnetic field

    for (uint8_t i = 16; i <= 21; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0f); // body magnetic field
}

// constrain states to prevent ill-conditioning
void ekf_ConstrainStates(void)
{
    // quaternions are limited between +-1
    for (uint8_t i = 0; i <= 3; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -1.0f, 1.0f);

    // velocity limit 500 m/sec
    for (uint8_t i = 4; i <= 6; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -5.0e2f, 5.0e2f);

    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i = 7; i <= 8; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -1.0e6f, 1.0e6f);

    // height limit covers home alt on everest through to home alt at SL and ballon drop
    ekfStates.statesArray[9] = constrainf(ekfStates.statesArray[9], -4.0e4f, 1.0e4f);

    // gyro bias limit ~6 deg/sec (this needs to be set based on manufacturers specs)
    for (uint8_t i = 10; i <= 12; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -0.1f * dtIMU, 0.1f * dtIMU);

    // when the vehicle arms we adjust the limits so that in flight the bias can change by the same amount in either direction
    float delAngBiasLim = EKF_MAX_GYRO_BIAS * dtIMU;
    ekfStates.stateStruct.gyro_bias.x = constrainf(ekfStates.stateStruct.gyro_bias.x, (delAngBiasAtArming.x - delAngBiasLim), (delAngBiasAtArming.x + delAngBiasLim));
    ekfStates.stateStruct.gyro_bias.y = constrainf(ekfStates.stateStruct.gyro_bias.y, (delAngBiasAtArming.y - delAngBiasLim), (delAngBiasAtArming.y + delAngBiasLim));
    ekfStates.stateStruct.gyro_bias.z = constrainf(ekfStates.stateStruct.gyro_bias.z, (delAngBiasAtArming.z - delAngBiasLim), (delAngBiasAtArming.z + delAngBiasLim));

    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    ekfStates.statesArray[13] = constrainf(ekfStates.statesArray[13], -1.0f * dtIMU, 1.0f * dtIMU);
    ekfStates.statesArray[22] = constrainf(ekfStates.statesArray[22], -1.0f * dtIMU, 1.0f * dtIMU);

    // wind velocity limit 100 m/s - TODO apply circular limit
    for (uint8_t i = 14; i <= 15; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -100.0f, 100.0f);

    // earth magnetic field limit
    for (uint8_t i = 16; i <= 18; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -1.0f, 1.0f);

    // body magnetic field limit
    for (uint8_t i = 19; i <= 21; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -0.5f, 0.5f);
}

// copy covariances across from covariance prediction calculation and fix numerical errors
void ekf_CopyAndFixCovariances(void)
{
    // copy predicted variances
    for (uint8_t i = 0; i <= 21; i++)
    {
        P[i][i] = nextP[i][i];
    }

    // copy predicted covariances and force symmetry
    for (uint8_t i = 1; i <= 21; i++)
    {
        for (uint8_t j = 0; j <= i - 1; j++)
        {
            P[i][j] = 0.5f * (nextP[i][j] + nextP[j][i]);
            P[j][i] = P[i][j];
        }
    }
}

// fuse magnetometer measurements and apply innovation consistency checks fuse each axis on consecutive time steps to spread computional load
void ekf_FuseMagnetometer(void)
{
    // declarations
    float *q0 = &mag_state.q0;
    float *q1 = &mag_state.q1;
    float *q2 = &mag_state.q2;
    float *q3 = &mag_state.q3;
    float *magN = &mag_state.magN;
    float *magE = &mag_state.magE;
    float *magD = &mag_state.magD;
    float *magXbias = &mag_state.magXbias;
    float *magYbias = &mag_state.magYbias;
    float *magZbias = &mag_state.magZbias;
    uint8_t *obsIndex = &mag_state.obsIndex;
    fpMatrix3_t *DCM = &mag_state.DCM;
    fpVector3_t *MagPred = &mag_state.MagPred;
    float *R_MAG = &mag_state.R_MAG;
    float *SH_MAG = &mag_state.SH_MAG[0];
    Vector22 H_MAG;
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains
    if (*obsIndex == 0)
    {
        // copy required states to local variable names
        *q0 = statesAtMagMeasTime.quat.q0;
        *q1 = statesAtMagMeasTime.quat.q1;
        *q2 = statesAtMagMeasTime.quat.q2;
        *q3 = statesAtMagMeasTime.quat.q3;
        *magN = statesAtMagMeasTime.earth_magfield.x;
        *magE = statesAtMagMeasTime.earth_magfield.y;
        *magD = statesAtMagMeasTime.earth_magfield.z;
        *magXbias = statesAtMagMeasTime.body_magfield.x;
        *magYbias = statesAtMagMeasTime.body_magfield.y;
        *magZbias = statesAtMagMeasTime.body_magfield.z;

        // rotate predicted earth components into body axes and calculate predicted measurements
        DCM->m[0][0] = *q0 * *q0 + *q1 * *q1 - *q2 * *q2 - *q3 * *q3;
        DCM->m[0][1] = 2 * (*q1 * *q2 + *q0 * *q3);
        DCM->m[0][2] = 2 * (*q1 * *q3 - *q0 * *q2);
        DCM->m[1][0] = 2 * (*q1 * *q2 - *q0 * *q3);
        DCM->m[1][1] = *q0 * *q0 - *q1 * *q1 + *q2 * *q2 - *q3 * *q3;
        DCM->m[1][2] = 2 * (*q2 * *q3 + *q0 * *q1);
        DCM->m[2][0] = 2 * (*q1 * *q3 + *q0 * *q2);
        DCM->m[2][1] = 2 * (*q2 * *q3 - *q0 * *q1);
        DCM->m[2][2] = *q0 * *q0 - *q1 * *q1 - *q2 * *q2 + *q3 * *q3;
        MagPred->v[X] = DCM->m[0][0] * *magN + DCM->m[0][1] * *magE + DCM->m[0][2] * *magD + *magXbias;
        MagPred->v[Y] = DCM->m[1][0] * *magN + DCM->m[1][1] * *magE + DCM->m[1][2] * *magD + *magYbias;
        MagPred->v[Z] = DCM->m[2][0] * *magN + DCM->m[2][1] * *magE + DCM->m[2][2] * *magD + *magZbias;

        // scale magnetometer observation error with total angular rate
        *R_MAG = sq(constrainf(ekfParam.magNoise, 0.01f, 0.5f)) + sq(magVarRateScale * calc_length_pythagorean_3D(dAngIMU.x, dAngIMU.y, dAngIMU.z) / dtIMU);

        // calculate observation jacobians
        SH_MAG[0] = 2 * *magD * *q3 + 2 * *magE * *q2 + 2 * *magN * *q1;
        SH_MAG[1] = 2 * *magD * *q0 - 2 * *magE * *q1 + 2 * *magN * *q2;
        SH_MAG[2] = 2 * *magD * *q1 + 2 * *magE * *q0 - 2 * *magN * *q3;
        SH_MAG[3] = sq(*q3);
        SH_MAG[4] = sq(*q2);
        SH_MAG[5] = sq(*q1);
        SH_MAG[6] = sq(*q0);
        SH_MAG[7] = 2 * *magN * *q0;
        SH_MAG[8] = 2 * *magE * *q3;
        for (uint8_t i = 0; i <= 21; i++)
            H_MAG[i] = 0;
        H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2;
        H_MAG[1] = SH_MAG[0];
        H_MAG[2] = 2 * *magE * *q1 - 2 * *magD * *q0 - 2 * *magN * *q2;
        H_MAG[3] = SH_MAG[2];
        H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
        H_MAG[17] = 2 * *q0 * *q3 + 2 * *q1 * *q2;
        H_MAG[18] = 2 * *q1 * *q3 - 2 * *q0 * *q2;
        H_MAG[19] = 1;

        // calculate Kalman gain
        float temp =
            (P[19][19] + *R_MAG + P[1][19] * SH_MAG[0] +
             P[3][19] * SH_MAG[2] -
             P[16][19] * (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) -
             (2 * *magD * *q0 - 2 * *magE * *q1 + 2 * *magN * *q2) *
                 (P[19][2] + P[1][2] * SH_MAG[0] + P[3][2] * SH_MAG[2] -
                  P[16][2] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][2] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][2] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][2] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                             2 * *magN * *q2) +
                  P[0][2] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2) *
                 (P[19][0] + P[1][0] * SH_MAG[0] + P[3][0] * SH_MAG[2] -
                  P[16][0] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][0] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][0] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][0] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                             2 * *magN * *q2) +
                  P[0][0] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             SH_MAG[0] *
                 (P[19][1] + P[1][1] * SH_MAG[0] + P[3][1] * SH_MAG[2] -
                  P[16][1] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][1] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][1] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][1] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                             2 * *magN * *q2) +
                  P[0][1] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             SH_MAG[2] *
                 (P[19][3] + P[1][3] * SH_MAG[0] + P[3][3] * SH_MAG[2] -
                  P[16][3] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][3] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][3] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][3] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                             2 * *magN * *q2) +
                  P[0][3] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) *
                 (P[19][16] + P[1][16] * SH_MAG[0] + P[3][16] * SH_MAG[2] -
                  P[16][16] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][16] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][16] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][16] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                              2 * *magN * *q2) +
                  P[0][16] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             P[17][19] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
             P[18][19] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
             P[2][19] *
                 (2 * *magD * *q0 - 2 * *magE * *q1 + 2 * *magN * *q2) +
             (2 * *q0 * *q3 + 2 * *q1 * *q2) *
                 (P[19][17] + P[1][17] * SH_MAG[0] + P[3][17] * SH_MAG[2] -
                  P[16][17] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][17] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][17] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][17] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                              2 * *magN * *q2) +
                  P[0][17] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             (2 * *q0 * *q2 - 2 * *q1 * *q3) *
                 (P[19][18] + P[1][18] * SH_MAG[0] + P[3][18] * SH_MAG[2] -
                  P[16][18] *
                      (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) +
                  P[17][18] * (2 * *q0 * *q3 + 2 * *q1 * *q2) -
                  P[18][18] * (2 * *q0 * *q2 - 2 * *q1 * *q3) -
                  P[2][18] * (2 * *magD * *q0 - 2 * *magE * *q1 +
                              2 * *magN * *q2) +
                  P[0][18] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             P[0][19] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2));
        if (temp >= *R_MAG)
        {
            SK_MX[0] = 1.0f / temp;
            faultStatus.bad_xmag = false;
        }
        else
        {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            ekf_CovarianceInit();
            *obsIndex = 1;
            faultStatus.bad_xmag = true;
            return;
        }
        SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
        SK_MX[2] = 2 * *magD * *q0 - 2 * *magE * *q1 + 2 * *magN * *q2;
        SK_MX[3] = SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2;
        SK_MX[4] = 2 * *q0 * *q2 - 2 * *q1 * *q3;
        SK_MX[5] = 2 * *q0 * *q3 + 2 * *q1 * *q2;
        Kfusion[0] = SK_MX[0] * (P[0][19] + P[0][1] * SH_MAG[0] +
                                 P[0][3] * SH_MAG[2] + P[0][0] * SK_MX[3] -
                                 P[0][2] * SK_MX[2] - P[0][16] * SK_MX[1] +
                                 P[0][17] * SK_MX[5] - P[0][18] * SK_MX[4]);
        Kfusion[1] = SK_MX[0] * (P[1][19] + P[1][1] * SH_MAG[0] +
                                 P[1][3] * SH_MAG[2] + P[1][0] * SK_MX[3] -
                                 P[1][2] * SK_MX[2] - P[1][16] * SK_MX[1] +
                                 P[1][17] * SK_MX[5] - P[1][18] * SK_MX[4]);
        Kfusion[2] = SK_MX[0] * (P[2][19] + P[2][1] * SH_MAG[0] +
                                 P[2][3] * SH_MAG[2] + P[2][0] * SK_MX[3] -
                                 P[2][2] * SK_MX[2] - P[2][16] * SK_MX[1] +
                                 P[2][17] * SK_MX[5] - P[2][18] * SK_MX[4]);
        Kfusion[3] = SK_MX[0] * (P[3][19] + P[3][1] * SH_MAG[0] +
                                 P[3][3] * SH_MAG[2] + P[3][0] * SK_MX[3] -
                                 P[3][2] * SK_MX[2] - P[3][16] * SK_MX[1] +
                                 P[3][17] * SK_MX[5] - P[3][18] * SK_MX[4]);
        Kfusion[4] = SK_MX[0] * (P[4][19] + P[4][1] * SH_MAG[0] +
                                 P[4][3] * SH_MAG[2] + P[4][0] * SK_MX[3] -
                                 P[4][2] * SK_MX[2] - P[4][16] * SK_MX[1] +
                                 P[4][17] * SK_MX[5] - P[4][18] * SK_MX[4]);
        Kfusion[5] = SK_MX[0] * (P[5][19] + P[5][1] * SH_MAG[0] +
                                 P[5][3] * SH_MAG[2] + P[5][0] * SK_MX[3] -
                                 P[5][2] * SK_MX[2] - P[5][16] * SK_MX[1] +
                                 P[5][17] * SK_MX[5] - P[5][18] * SK_MX[4]);
        Kfusion[6] = SK_MX[0] * (P[6][19] + P[6][1] * SH_MAG[0] +
                                 P[6][3] * SH_MAG[2] + P[6][0] * SK_MX[3] -
                                 P[6][2] * SK_MX[2] - P[6][16] * SK_MX[1] +
                                 P[6][17] * SK_MX[5] - P[6][18] * SK_MX[4]);
        Kfusion[7] = SK_MX[0] * (P[7][19] + P[7][1] * SH_MAG[0] +
                                 P[7][3] * SH_MAG[2] + P[7][0] * SK_MX[3] -
                                 P[7][2] * SK_MX[2] - P[7][16] * SK_MX[1] +
                                 P[7][17] * SK_MX[5] - P[7][18] * SK_MX[4]);
        Kfusion[8] = SK_MX[0] * (P[8][19] + P[8][1] * SH_MAG[0] +
                                 P[8][3] * SH_MAG[2] + P[8][0] * SK_MX[3] -
                                 P[8][2] * SK_MX[2] - P[8][16] * SK_MX[1] +
                                 P[8][17] * SK_MX[5] - P[8][18] * SK_MX[4]);
        Kfusion[9] = SK_MX[0] * (P[9][19] + P[9][1] * SH_MAG[0] +
                                 P[9][3] * SH_MAG[2] + P[9][0] * SK_MX[3] -
                                 P[9][2] * SK_MX[2] - P[9][16] * SK_MX[1] +
                                 P[9][17] * SK_MX[5] - P[9][18] * SK_MX[4]);
        Kfusion[10] =
            SK_MX[0] * (P[10][19] + P[10][1] * SH_MAG[0] +
                        P[10][3] * SH_MAG[2] + P[10][0] * SK_MX[3] -
                        P[10][2] * SK_MX[2] - P[10][16] * SK_MX[1] +
                        P[10][17] * SK_MX[5] - P[10][18] * SK_MX[4]);
        Kfusion[11] =
            SK_MX[0] * (P[11][19] + P[11][1] * SH_MAG[0] +
                        P[11][3] * SH_MAG[2] + P[11][0] * SK_MX[3] -
                        P[11][2] * SK_MX[2] - P[11][16] * SK_MX[1] +
                        P[11][17] * SK_MX[5] - P[11][18] * SK_MX[4]);
        Kfusion[12] =
            SK_MX[0] * (P[12][19] + P[12][1] * SH_MAG[0] +
                        P[12][3] * SH_MAG[2] + P[12][0] * SK_MX[3] -
                        P[12][2] * SK_MX[2] - P[12][16] * SK_MX[1] +
                        P[12][17] * SK_MX[5] - P[12][18] * SK_MX[4]);
        // this term has been zeroed to improve stability of the Z accel
        // bias
        Kfusion[13] = 0.0f; // SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] +
                            // P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[3] -
                            // P[13][2]*SK_MX[2] - P[13][16]*SK_MX[1] +
                            // P[13][17]*SK_MX[5] - P[13][18]*SK_MX[4]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates)
        {
            Kfusion[14] =
                SK_MX[0] * (P[14][19] + P[14][1] * SH_MAG[0] +
                            P[14][3] * SH_MAG[2] + P[14][0] * SK_MX[3] -
                            P[14][2] * SK_MX[2] - P[14][16] * SK_MX[1] +
                            P[14][17] * SK_MX[5] - P[14][18] * SK_MX[4]);
            Kfusion[15] =
                SK_MX[0] * (P[15][19] + P[15][1] * SH_MAG[0] +
                            P[15][3] * SH_MAG[2] + P[15][0] * SK_MX[3] -
                            P[15][2] * SK_MX[2] - P[15][16] * SK_MX[1] +
                            P[15][17] * SK_MX[5] - P[15][18] * SK_MX[4]);
        }
        else
        {
            Kfusion[14] = 0.0;
            Kfusion[15] = 0.0;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates)
        {
            Kfusion[16] =
                SK_MX[0] * (P[16][19] + P[16][1] * SH_MAG[0] +
                            P[16][3] * SH_MAG[2] + P[16][0] * SK_MX[3] -
                            P[16][2] * SK_MX[2] - P[16][16] * SK_MX[1] +
                            P[16][17] * SK_MX[5] - P[16][18] * SK_MX[4]);
            Kfusion[17] =
                SK_MX[0] * (P[17][19] + P[17][1] * SH_MAG[0] +
                            P[17][3] * SH_MAG[2] + P[17][0] * SK_MX[3] -
                            P[17][2] * SK_MX[2] - P[17][16] * SK_MX[1] +
                            P[17][17] * SK_MX[5] - P[17][18] * SK_MX[4]);
            Kfusion[18] =
                SK_MX[0] * (P[18][19] + P[18][1] * SH_MAG[0] +
                            P[18][3] * SH_MAG[2] + P[18][0] * SK_MX[3] -
                            P[18][2] * SK_MX[2] - P[18][16] * SK_MX[1] +
                            P[18][17] * SK_MX[5] - P[18][18] * SK_MX[4]);
            Kfusion[19] =
                SK_MX[0] * (P[19][19] + P[19][1] * SH_MAG[0] +
                            P[19][3] * SH_MAG[2] + P[19][0] * SK_MX[3] -
                            P[19][2] * SK_MX[2] - P[19][16] * SK_MX[1] +
                            P[19][17] * SK_MX[5] - P[19][18] * SK_MX[4]);
            Kfusion[20] =
                SK_MX[0] * (P[20][19] + P[20][1] * SH_MAG[0] +
                            P[20][3] * SH_MAG[2] + P[20][0] * SK_MX[3] -
                            P[20][2] * SK_MX[2] - P[20][16] * SK_MX[1] +
                            P[20][17] * SK_MX[5] - P[20][18] * SK_MX[4]);
            Kfusion[21] =
                SK_MX[0] * (P[21][19] + P[21][1] * SH_MAG[0] +
                            P[21][3] * SH_MAG[2] + P[21][0] * SK_MX[3] -
                            P[21][2] * SK_MX[2] - P[21][16] * SK_MX[1] +
                            P[21][17] * SK_MX[5] - P[21][18] * SK_MX[4]);
        }
        else
        {
            for (uint8_t i = 16; i <= 21; i++)
            {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate the observation innovation variance
        varInnovMag.v[X] = 1.0f / SK_MX[0];

        // reset the observation index to 0 (we start by fusing the X measurement)
        *obsIndex = 0;
    }
    else if (*obsIndex == 1) // we are now fusing the Y measurement
    {
        // calculate observation jacobians
        for (uint8_t i = 0; i <= 21; i++)
            H_MAG[i] = 0;
        H_MAG[0] = SH_MAG[2];
        H_MAG[1] = SH_MAG[1];
        H_MAG[2] = SH_MAG[0];
        H_MAG[3] = 2 * *magD * *q2 - SH_MAG[8] - SH_MAG[7];
        H_MAG[16] = 2 * *q1 * *q2 - 2 * *q0 * *q3;
        H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
        H_MAG[18] = 2 * *q0 * *q1 + 2 * *q2 * *q3;
        H_MAG[20] = 1;

        // calculate Kalman gain
        float temp =
            (P[20][20] + *R_MAG + P[0][20] * SH_MAG[2] +
             P[1][20] * SH_MAG[1] + P[2][20] * SH_MAG[0] -
             P[17][20] * (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
             (2 * *q0 * *q3 - 2 * *q1 * *q2) *
                 (P[20][16] + P[0][16] * SH_MAG[2] + P[1][16] * SH_MAG[1] +
                  P[2][16] * SH_MAG[0] -
                  P[17][16] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][16] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][16] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][16] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             (2 * *q0 * *q1 + 2 * *q2 * *q3) *
                 (P[20][18] + P[0][18] * SH_MAG[2] + P[1][18] * SH_MAG[1] +
                  P[2][18] * SH_MAG[0] -
                  P[17][18] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][18] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][18] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][18] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2) *
                 (P[20][3] + P[0][3] * SH_MAG[2] + P[1][3] * SH_MAG[1] +
                  P[2][3] * SH_MAG[0] -
                  P[17][3] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][3] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][3] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][3] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             P[16][20] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
             P[18][20] * (2 * *q0 * *q1 + 2 * *q2 * *q3) +
             SH_MAG[2] *
                 (P[20][0] + P[0][0] * SH_MAG[2] + P[1][0] * SH_MAG[1] +
                  P[2][0] * SH_MAG[0] -
                  P[17][0] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][0] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][0] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][0] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             SH_MAG[1] *
                 (P[20][1] + P[0][1] * SH_MAG[2] + P[1][1] * SH_MAG[1] +
                  P[2][1] * SH_MAG[0] -
                  P[17][1] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][1] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][1] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][1] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             SH_MAG[0] *
                 (P[20][2] + P[0][2] * SH_MAG[2] + P[1][2] * SH_MAG[1] +
                  P[2][2] * SH_MAG[0] -
                  P[17][2] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][2] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][2] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][2] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) *
                 (P[20][17] + P[0][17] * SH_MAG[2] + P[1][17] * SH_MAG[1] +
                  P[2][17] * SH_MAG[0] -
                  P[17][17] *
                      (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) -
                  P[16][17] * (2 * *q0 * *q3 - 2 * *q1 * *q2) +
                  P[18][17] * (2 * *q0 * *q1 + 2 * *q2 * *q3) -
                  P[3][17] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             P[3][20] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2));
        if (temp >= *R_MAG)
        {
            SK_MY[0] = 1.0f / temp;
            faultStatus.bad_ymag = false;
        }
        else
        {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            ekf_CovarianceInit();
            *obsIndex = 2;
            faultStatus.bad_ymag = true;
            return;
        }
        SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
        SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2;
        SK_MY[3] = 2 * *q0 * *q3 - 2 * *q1 * *q2;
        SK_MY[4] = 2 * *q0 * *q1 + 2 * *q2 * *q3;
        Kfusion[0] = SK_MY[0] * (P[0][20] + P[0][0] * SH_MAG[2] +
                                 P[0][1] * SH_MAG[1] + P[0][2] * SH_MAG[0] -
                                 P[0][3] * SK_MY[2] - P[0][17] * SK_MY[1] -
                                 P[0][16] * SK_MY[3] + P[0][18] * SK_MY[4]);
        Kfusion[1] = SK_MY[0] * (P[1][20] + P[1][0] * SH_MAG[2] +
                                 P[1][1] * SH_MAG[1] + P[1][2] * SH_MAG[0] -
                                 P[1][3] * SK_MY[2] - P[1][17] * SK_MY[1] -
                                 P[1][16] * SK_MY[3] + P[1][18] * SK_MY[4]);
        Kfusion[2] = SK_MY[0] * (P[2][20] + P[2][0] * SH_MAG[2] +
                                 P[2][1] * SH_MAG[1] + P[2][2] * SH_MAG[0] -
                                 P[2][3] * SK_MY[2] - P[2][17] * SK_MY[1] -
                                 P[2][16] * SK_MY[3] + P[2][18] * SK_MY[4]);
        Kfusion[3] = SK_MY[0] * (P[3][20] + P[3][0] * SH_MAG[2] +
                                 P[3][1] * SH_MAG[1] + P[3][2] * SH_MAG[0] -
                                 P[3][3] * SK_MY[2] - P[3][17] * SK_MY[1] -
                                 P[3][16] * SK_MY[3] + P[3][18] * SK_MY[4]);
        Kfusion[4] = SK_MY[0] * (P[4][20] + P[4][0] * SH_MAG[2] +
                                 P[4][1] * SH_MAG[1] + P[4][2] * SH_MAG[0] -
                                 P[4][3] * SK_MY[2] - P[4][17] * SK_MY[1] -
                                 P[4][16] * SK_MY[3] + P[4][18] * SK_MY[4]);
        Kfusion[5] = SK_MY[0] * (P[5][20] + P[5][0] * SH_MAG[2] +
                                 P[5][1] * SH_MAG[1] + P[5][2] * SH_MAG[0] -
                                 P[5][3] * SK_MY[2] - P[5][17] * SK_MY[1] -
                                 P[5][16] * SK_MY[3] + P[5][18] * SK_MY[4]);
        Kfusion[6] = SK_MY[0] * (P[6][20] + P[6][0] * SH_MAG[2] +
                                 P[6][1] * SH_MAG[1] + P[6][2] * SH_MAG[0] -
                                 P[6][3] * SK_MY[2] - P[6][17] * SK_MY[1] -
                                 P[6][16] * SK_MY[3] + P[6][18] * SK_MY[4]);
        Kfusion[7] = SK_MY[0] * (P[7][20] + P[7][0] * SH_MAG[2] +
                                 P[7][1] * SH_MAG[1] + P[7][2] * SH_MAG[0] -
                                 P[7][3] * SK_MY[2] - P[7][17] * SK_MY[1] -
                                 P[7][16] * SK_MY[3] + P[7][18] * SK_MY[4]);
        Kfusion[8] = SK_MY[0] * (P[8][20] + P[8][0] * SH_MAG[2] +
                                 P[8][1] * SH_MAG[1] + P[8][2] * SH_MAG[0] -
                                 P[8][3] * SK_MY[2] - P[8][17] * SK_MY[1] -
                                 P[8][16] * SK_MY[3] + P[8][18] * SK_MY[4]);
        Kfusion[9] = SK_MY[0] * (P[9][20] + P[9][0] * SH_MAG[2] +
                                 P[9][1] * SH_MAG[1] + P[9][2] * SH_MAG[0] -
                                 P[9][3] * SK_MY[2] - P[9][17] * SK_MY[1] -
                                 P[9][16] * SK_MY[3] + P[9][18] * SK_MY[4]);
        Kfusion[10] =
            SK_MY[0] * (P[10][20] + P[10][0] * SH_MAG[2] +
                        P[10][1] * SH_MAG[1] + P[10][2] * SH_MAG[0] -
                        P[10][3] * SK_MY[2] - P[10][17] * SK_MY[1] -
                        P[10][16] * SK_MY[3] + P[10][18] * SK_MY[4]);
        Kfusion[11] =
            SK_MY[0] * (P[11][20] + P[11][0] * SH_MAG[2] +
                        P[11][1] * SH_MAG[1] + P[11][2] * SH_MAG[0] -
                        P[11][3] * SK_MY[2] - P[11][17] * SK_MY[1] -
                        P[11][16] * SK_MY[3] + P[11][18] * SK_MY[4]);
        Kfusion[12] =
            SK_MY[0] * (P[12][20] + P[12][0] * SH_MAG[2] +
                        P[12][1] * SH_MAG[1] + P[12][2] * SH_MAG[0] -
                        P[12][3] * SK_MY[2] - P[12][17] * SK_MY[1] -
                        P[12][16] * SK_MY[3] + P[12][18] * SK_MY[4]);
        // this term has been zeroed to improve stability of the Z accel
        // bias
        Kfusion[13] = 0.0f; // SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] +
                            // P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] -
                            // P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] -
                            // P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates)
        {
            Kfusion[14] =
                SK_MY[0] * (P[14][20] + P[14][0] * SH_MAG[2] +
                            P[14][1] * SH_MAG[1] + P[14][2] * SH_MAG[0] -
                            P[14][3] * SK_MY[2] - P[14][17] * SK_MY[1] -
                            P[14][16] * SK_MY[3] + P[14][18] * SK_MY[4]);
            Kfusion[15] =
                SK_MY[0] * (P[15][20] + P[15][0] * SH_MAG[2] +
                            P[15][1] * SH_MAG[1] + P[15][2] * SH_MAG[0] -
                            P[15][3] * SK_MY[2] - P[15][17] * SK_MY[1] -
                            P[15][16] * SK_MY[3] + P[15][18] * SK_MY[4]);
        }
        else
        {
            Kfusion[14] = 0.0;
            Kfusion[15] = 0.0;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates)
        {
            Kfusion[16] =
                SK_MY[0] * (P[16][20] + P[16][0] * SH_MAG[2] +
                            P[16][1] * SH_MAG[1] + P[16][2] * SH_MAG[0] -
                            P[16][3] * SK_MY[2] - P[16][17] * SK_MY[1] -
                            P[16][16] * SK_MY[3] + P[16][18] * SK_MY[4]);
            Kfusion[17] =
                SK_MY[0] * (P[17][20] + P[17][0] * SH_MAG[2] +
                            P[17][1] * SH_MAG[1] + P[17][2] * SH_MAG[0] -
                            P[17][3] * SK_MY[2] - P[17][17] * SK_MY[1] -
                            P[17][16] * SK_MY[3] + P[17][18] * SK_MY[4]);
            Kfusion[18] =
                SK_MY[0] * (P[18][20] + P[18][0] * SH_MAG[2] +
                            P[18][1] * SH_MAG[1] + P[18][2] * SH_MAG[0] -
                            P[18][3] * SK_MY[2] - P[18][17] * SK_MY[1] -
                            P[18][16] * SK_MY[3] + P[18][18] * SK_MY[4]);
            Kfusion[19] =
                SK_MY[0] * (P[19][20] + P[19][0] * SH_MAG[2] +
                            P[19][1] * SH_MAG[1] + P[19][2] * SH_MAG[0] -
                            P[19][3] * SK_MY[2] - P[19][17] * SK_MY[1] -
                            P[19][16] * SK_MY[3] + P[19][18] * SK_MY[4]);
            Kfusion[20] =
                SK_MY[0] * (P[20][20] + P[20][0] * SH_MAG[2] +
                            P[20][1] * SH_MAG[1] + P[20][2] * SH_MAG[0] -
                            P[20][3] * SK_MY[2] - P[20][17] * SK_MY[1] -
                            P[20][16] * SK_MY[3] + P[20][18] * SK_MY[4]);
            Kfusion[21] =
                SK_MY[0] * (P[21][20] + P[21][0] * SH_MAG[2] +
                            P[21][1] * SH_MAG[1] + P[21][2] * SH_MAG[0] -
                            P[21][3] * SK_MY[2] - P[21][17] * SK_MY[1] -
                            P[21][16] * SK_MY[3] + P[21][18] * SK_MY[4]);
        }
        else
        {
            for (uint8_t i = 16; i <= 21; i++)
            {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate the observation innovation variance
        varInnovMag.v[Y] = 1.0f / SK_MY[0];
    }
    else if (*obsIndex == 2) // we are now fusing the Z measurement
    {
        // calculate observation jacobians
        for (uint8_t i = 0; i <= 21; i++)
            H_MAG[i] = 0;
        H_MAG[0] = SH_MAG[1];
        H_MAG[1] = 2 * *magN * *q3 - 2 * *magE * *q0 - 2 * *magD * *q1;
        H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2;
        H_MAG[3] = SH_MAG[0];
        H_MAG[16] = 2 * *q0 * *q2 + 2 * *q1 * *q3;
        H_MAG[17] = 2 * *q2 * *q3 - 2 * *q0 * *q1;
        H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
        H_MAG[21] = 1;

        // calculate Kalman gain
        float temp =
            (P[21][21] + *R_MAG + P[0][21] * SH_MAG[1] +
             P[3][21] * SH_MAG[0] +
             P[18][21] * (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) -
             (2 * *magD * *q1 + 2 * *magE * *q0 - 2 * *magN * *q3) *
                 (P[21][1] + P[0][1] * SH_MAG[1] + P[3][1] * SH_MAG[0] +
                  P[18][1] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][1] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][1] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][1] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                             2 * *magN * *q3) +
                  P[2][1] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2) *
                 (P[21][2] + P[0][2] * SH_MAG[1] + P[3][2] * SH_MAG[0] +
                  P[18][2] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][2] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][2] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][2] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                             2 * *magN * *q3) +
                  P[2][2] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             SH_MAG[1] *
                 (P[21][0] + P[0][0] * SH_MAG[1] + P[3][0] * SH_MAG[0] +
                  P[18][0] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][0] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][0] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][0] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                             2 * *magN * *q3) +
                  P[2][0] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             SH_MAG[0] *
                 (P[21][3] + P[0][3] * SH_MAG[1] + P[3][3] * SH_MAG[0] +
                  P[18][3] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][3] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][3] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][3] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                             2 * *magN * *q3) +
                  P[2][3] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) *
                 (P[21][18] + P[0][18] * SH_MAG[1] + P[3][18] * SH_MAG[0] +
                  P[18][18] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][18] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][18] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][18] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                              2 * *magN * *q3) +
                  P[2][18] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             P[16][21] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
             P[17][21] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
             P[1][21] *
                 (2 * *magD * *q1 + 2 * *magE * *q0 - 2 * *magN * *q3) +
             (2 * *q0 * *q2 + 2 * *q1 * *q3) *
                 (P[21][16] + P[0][16] * SH_MAG[1] + P[3][16] * SH_MAG[0] +
                  P[18][16] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][16] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][16] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][16] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                              2 * *magN * *q3) +
                  P[2][16] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) -
             (2 * *q0 * *q1 - 2 * *q2 * *q3) *
                 (P[21][17] + P[0][17] * SH_MAG[1] + P[3][17] * SH_MAG[0] +
                  P[18][17] *
                      (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) +
                  P[16][17] * (2 * *q0 * *q2 + 2 * *q1 * *q3) -
                  P[17][17] * (2 * *q0 * *q1 - 2 * *q2 * *q3) -
                  P[1][17] * (2 * *magD * *q1 + 2 * *magE * *q0 -
                              2 * *magN * *q3) +
                  P[2][17] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2)) +
             P[2][21] * (SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2));
        if (temp >= *R_MAG)
        {
            SK_MZ[0] = 1.0f / temp;
            faultStatus.bad_zmag = false;
        }
        else
        {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            ekf_CovarianceInit();
            *obsIndex = 3;
            faultStatus.bad_zmag = true;
            return;
        }
        SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
        SK_MZ[2] = 2 * *magD * *q1 + 2 * *magE * *q0 - 2 * *magN * *q3;
        SK_MZ[3] = SH_MAG[7] + SH_MAG[8] - 2 * *magD * *q2;
        SK_MZ[4] = 2 * *q0 * *q1 - 2 * *q2 * *q3;
        SK_MZ[5] = 2 * *q0 * *q2 + 2 * *q1 * *q3;
        Kfusion[0] = SK_MZ[0] * (P[0][21] + P[0][0] * SH_MAG[1] +
                                 P[0][3] * SH_MAG[0] - P[0][1] * SK_MZ[2] +
                                 P[0][2] * SK_MZ[3] + P[0][18] * SK_MZ[1] +
                                 P[0][16] * SK_MZ[5] - P[0][17] * SK_MZ[4]);
        Kfusion[1] = SK_MZ[0] * (P[1][21] + P[1][0] * SH_MAG[1] +
                                 P[1][3] * SH_MAG[0] - P[1][1] * SK_MZ[2] +
                                 P[1][2] * SK_MZ[3] + P[1][18] * SK_MZ[1] +
                                 P[1][16] * SK_MZ[5] - P[1][17] * SK_MZ[4]);
        Kfusion[2] = SK_MZ[0] * (P[2][21] + P[2][0] * SH_MAG[1] +
                                 P[2][3] * SH_MAG[0] - P[2][1] * SK_MZ[2] +
                                 P[2][2] * SK_MZ[3] + P[2][18] * SK_MZ[1] +
                                 P[2][16] * SK_MZ[5] - P[2][17] * SK_MZ[4]);
        Kfusion[3] = SK_MZ[0] * (P[3][21] + P[3][0] * SH_MAG[1] +
                                 P[3][3] * SH_MAG[0] - P[3][1] * SK_MZ[2] +
                                 P[3][2] * SK_MZ[3] + P[3][18] * SK_MZ[1] +
                                 P[3][16] * SK_MZ[5] - P[3][17] * SK_MZ[4]);
        Kfusion[4] = SK_MZ[0] * (P[4][21] + P[4][0] * SH_MAG[1] +
                                 P[4][3] * SH_MAG[0] - P[4][1] * SK_MZ[2] +
                                 P[4][2] * SK_MZ[3] + P[4][18] * SK_MZ[1] +
                                 P[4][16] * SK_MZ[5] - P[4][17] * SK_MZ[4]);
        Kfusion[5] = SK_MZ[0] * (P[5][21] + P[5][0] * SH_MAG[1] +
                                 P[5][3] * SH_MAG[0] - P[5][1] * SK_MZ[2] +
                                 P[5][2] * SK_MZ[3] + P[5][18] * SK_MZ[1] +
                                 P[5][16] * SK_MZ[5] - P[5][17] * SK_MZ[4]);
        Kfusion[6] = SK_MZ[0] * (P[6][21] + P[6][0] * SH_MAG[1] +
                                 P[6][3] * SH_MAG[0] - P[6][1] * SK_MZ[2] +
                                 P[6][2] * SK_MZ[3] + P[6][18] * SK_MZ[1] +
                                 P[6][16] * SK_MZ[5] - P[6][17] * SK_MZ[4]);
        Kfusion[7] = SK_MZ[0] * (P[7][21] + P[7][0] * SH_MAG[1] +
                                 P[7][3] * SH_MAG[0] - P[7][1] * SK_MZ[2] +
                                 P[7][2] * SK_MZ[3] + P[7][18] * SK_MZ[1] +
                                 P[7][16] * SK_MZ[5] - P[7][17] * SK_MZ[4]);
        Kfusion[8] = SK_MZ[0] * (P[8][21] + P[8][0] * SH_MAG[1] +
                                 P[8][3] * SH_MAG[0] - P[8][1] * SK_MZ[2] +
                                 P[8][2] * SK_MZ[3] + P[8][18] * SK_MZ[1] +
                                 P[8][16] * SK_MZ[5] - P[8][17] * SK_MZ[4]);
        Kfusion[9] = SK_MZ[0] * (P[9][21] + P[9][0] * SH_MAG[1] +
                                 P[9][3] * SH_MAG[0] - P[9][1] * SK_MZ[2] +
                                 P[9][2] * SK_MZ[3] + P[9][18] * SK_MZ[1] +
                                 P[9][16] * SK_MZ[5] - P[9][17] * SK_MZ[4]);
        Kfusion[10] =
            SK_MZ[0] * (P[10][21] + P[10][0] * SH_MAG[1] +
                        P[10][3] * SH_MAG[0] - P[10][1] * SK_MZ[2] +
                        P[10][2] * SK_MZ[3] + P[10][18] * SK_MZ[1] +
                        P[10][16] * SK_MZ[5] - P[10][17] * SK_MZ[4]);
        Kfusion[11] =
            SK_MZ[0] * (P[11][21] + P[11][0] * SH_MAG[1] +
                        P[11][3] * SH_MAG[0] - P[11][1] * SK_MZ[2] +
                        P[11][2] * SK_MZ[3] + P[11][18] * SK_MZ[1] +
                        P[11][16] * SK_MZ[5] - P[11][17] * SK_MZ[4]);
        Kfusion[12] =
            SK_MZ[0] * (P[12][21] + P[12][0] * SH_MAG[1] +
                        P[12][3] * SH_MAG[0] - P[12][1] * SK_MZ[2] +
                        P[12][2] * SK_MZ[3] + P[12][18] * SK_MZ[1] +
                        P[12][16] * SK_MZ[5] - P[12][17] * SK_MZ[4]);
        // this term has been zeroed to improve stability of the Z accel
        // bias
        Kfusion[13] = 0.0f; // SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] +
                            // P[13][3]*SH_MAG[0] - P[13][1]*SK_MZ[2] +
                            // P[13][2]*SK_MZ[3] + P[13][18]*SK_MZ[1] +
                            // P[13][16]*SK_MZ[5] - P[13][17]*SK_MZ[4]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates)
        {
            Kfusion[14] =
                SK_MZ[0] * (P[14][21] + P[14][0] * SH_MAG[1] +
                            P[14][3] * SH_MAG[0] - P[14][1] * SK_MZ[2] +
                            P[14][2] * SK_MZ[3] + P[14][18] * SK_MZ[1] +
                            P[14][16] * SK_MZ[5] - P[14][17] * SK_MZ[4]);
            Kfusion[15] =
                SK_MZ[0] * (P[15][21] + P[15][0] * SH_MAG[1] +
                            P[15][3] * SH_MAG[0] - P[15][1] * SK_MZ[2] +
                            P[15][2] * SK_MZ[3] + P[15][18] * SK_MZ[1] +
                            P[15][16] * SK_MZ[5] - P[15][17] * SK_MZ[4]);
        }
        else
        {
            Kfusion[14] = 0.0;
            Kfusion[15] = 0.0;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates)
        {
            Kfusion[16] =
                SK_MZ[0] * (P[16][21] + P[16][0] * SH_MAG[1] +
                            P[16][3] * SH_MAG[0] - P[16][1] * SK_MZ[2] +
                            P[16][2] * SK_MZ[3] + P[16][18] * SK_MZ[1] +
                            P[16][16] * SK_MZ[5] - P[16][17] * SK_MZ[4]);
            Kfusion[17] =
                SK_MZ[0] * (P[17][21] + P[17][0] * SH_MAG[1] +
                            P[17][3] * SH_MAG[0] - P[17][1] * SK_MZ[2] +
                            P[17][2] * SK_MZ[3] + P[17][18] * SK_MZ[1] +
                            P[17][16] * SK_MZ[5] - P[17][17] * SK_MZ[4]);
            Kfusion[18] =
                SK_MZ[0] * (P[18][21] + P[18][0] * SH_MAG[1] +
                            P[18][3] * SH_MAG[0] - P[18][1] * SK_MZ[2] +
                            P[18][2] * SK_MZ[3] + P[18][18] * SK_MZ[1] +
                            P[18][16] * SK_MZ[5] - P[18][17] * SK_MZ[4]);
            Kfusion[19] =
                SK_MZ[0] * (P[19][21] + P[19][0] * SH_MAG[1] +
                            P[19][3] * SH_MAG[0] - P[19][1] * SK_MZ[2] +
                            P[19][2] * SK_MZ[3] + P[19][18] * SK_MZ[1] +
                            P[19][16] * SK_MZ[5] - P[19][17] * SK_MZ[4]);
            Kfusion[20] =
                SK_MZ[0] * (P[20][21] + P[20][0] * SH_MAG[1] +
                            P[20][3] * SH_MAG[0] - P[20][1] * SK_MZ[2] +
                            P[20][2] * SK_MZ[3] + P[20][18] * SK_MZ[1] +
                            P[20][16] * SK_MZ[5] - P[20][17] * SK_MZ[4]);
            Kfusion[21] =
                SK_MZ[0] * (P[21][21] + P[21][0] * SH_MAG[1] +
                            P[21][3] * SH_MAG[0] - P[21][1] * SK_MZ[2] +
                            P[21][2] * SK_MZ[3] + P[21][18] * SK_MZ[1] +
                            P[21][16] * SK_MZ[5] - P[21][17] * SK_MZ[4]);
        }
        else
        {
            for (uint8_t i = 16; i <= 21; i++)
            {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate the observation innovation variance
        varInnovMag.v[Z] = 1.0f / SK_MZ[0];
    }
    // calculate the measurement innovation
    innovMag.v[*obsIndex] = MagPred->v[*obsIndex] - magData.v[*obsIndex];
    // calculate the innovation test ratio
    magTestRatio.v[*obsIndex] = sq(innovMag.v[*obsIndex]) / (sq(ekfParam.magInnovGate) * varInnovMag.v[*obsIndex]);
    // check the last values from all components and set magnetometer health accordingly
    magHealth = (magTestRatio.v[X] < 1.0f && magTestRatio.v[Y] < 1.0f && magTestRatio.v[Z] < 1.0f);
    // Don't fuse unless all componenets pass. The exception is if the bad
    // health has timed out and we are not a fly forward vehicle
    // In this case we might as well try using the magnetometer, but with a reduced weighting
    if (magHealth || ((magTestRatio.v[*obsIndex] < 1.0f) && !STATE(FIXED_WING_LEGACY) && magTimeout))
    {
        // Attitude, velocity and position corrections are averaged across
        // multiple prediction cycles between now and the anticipated time
        // for the next measurement. Don't do averaging of quaternion state
        // corrections if total angle change across predicted interval is
        // going to exceed 0.1 rad
        bool highRates = ((magUpdateCountMax * calc_length_pythagorean_3D(correctedDelAng.x, correctedDelAng.y, correctedDelAng.z)) > 0.1f);
        // Calculate the number of averaging frames left to go. This is
        // required becasue magnetometer fusion is applied across three
        // consecutive prediction cycles There is no point averaging if the
        // number of cycles left is less than 2
        float minorFramesToGo = (float)magUpdateCountMax - (float)magUpdateCount;
        // correct the state vector or store corrections to be applied
        // incrementally
        for (uint8_t j = 0; j <= 21; j++)
        {
            // If we are forced to use a bad compass, we reduce the weighting by a factor of 4
            if (!magHealth)
            {
                Kfusion[j] *= 0.25f;
            }

            // If in the air and there is no other form of heading reference or we are yawing rapidly which creates larger inertial yaw errors,
            // we strengthen the magnetometer attitude correction
            if (vehicleArmed && highYawRate && j <= 3)
            {
                Kfusion[j] *= 4.0f;
            }

            // We don't need to spread corrections for non-dynamic states
            // We can't spread corrections if there is not enough time remaining
            // We don't spread corrections to attitude states if we are rotating rapidly
            if ((j <= 3 && highRates) || j >= 10 || !vehicleArmed || minorFramesToGo < 1.5f)
            {
                ekfStates.statesArray[j] = ekfStates.statesArray[j] - Kfusion[j] * innovMag.v[*obsIndex];
            }
            else
            {
                // scale the correction based on the number of averaging frames left to go
                magIncrStateDelta[j] -= Kfusion[j] * innovMag.v[*obsIndex] * (magUpdateCountMaxInv * (float)magUpdateCountMax / minorFramesToGo);
            }
        }

        // normalise the quaternion states
        quaternionNormalize(&ekfStates.stateStruct.quat, &ekfStates.stateStruct.quat);

        // correct the covariance P = (I - K * H) * P take advantage of the empty columns in KH to reduce the number of operations
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 3; j++)
            {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (uint8_t j = 4; j <= 15; j++)
            {
                KH[i][j] = 0.0f;
            }
            if (!inhibitMagStates)
            {
                for (uint8_t j = 16; j <= 21; j++)
                {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                }
            }
            else
            {
                for (uint8_t j = 16; j <= 21; j++)
                {
                    KH[i][j] = 0.0f;
                }
            }
        }
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 21; j++)
            {
                KHP[i][j] = 0;
                for (uint8_t k = 0; k <= 3; k++)
                {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
                if (!inhibitMagStates)
                {
                    for (uint8_t k = 16; k <= 21; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
        }
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 21; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
    ekf_ForceSymmetry();
    ekf_ConstrainVariances();
}

// fuse true airspeed measurements
void ekf_FuseAirspeed(void)
{
    // declarations
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    const float R_IAS = sq(constrainf(ekfParam.iasNoise, 0.5f, 5.0f));
    fpVector3_t SH_IAS;
    float SK_IAS;
    Vector22 H_IAS;
    float VtasPred;

    // health is set bad until test passed
    iasHealth = false;

    // copy required states to local variable names
    vn = statesAtViasMeasTime.velocity.x;
    ve = statesAtViasMeasTime.velocity.y;
    vd = statesAtViasMeasTime.velocity.z;
    vwn = statesAtViasMeasTime.wind_vel.x;
    vwe = statesAtViasMeasTime.wind_vel.y;

    // calculate the predicted airspeed
    VtasPred = calc_length_pythagorean_3D((ve - vwe), (vn - vwn), vd);
    // perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // calculate observation jacobians
        SH_IAS.v[0] = 1.0f / VtasPred;
        SH_IAS.v[1] = (SH_IAS.v[0] * (2 * ve - 2 * vwe)) / 2;
        SH_IAS.v[2] = (SH_IAS.v[0] * (2 * vn - 2 * vwn)) / 2;
        for (uint8_t i = 0; i <= 21; i++)
            H_IAS[i] = 0.0f;
        H_IAS[4] = SH_IAS.v[2];
        H_IAS[5] = SH_IAS.v[1];
        H_IAS[6] = vd * SH_IAS.v[0];
        H_IAS[14] = -SH_IAS.v[2];
        H_IAS[15] = -SH_IAS.v[1];

        // calculate Kalman gains
        float temp =
            (R_IAS +
             SH_IAS.v[2] * (P[4][4] * SH_IAS.v[2] + P[5][4] * SH_IAS.v[1] -
                            P[14][4] * SH_IAS.v[2] - P[15][4] * SH_IAS.v[1] +
                            P[6][4] * vd * SH_IAS.v[0]) +
             SH_IAS.v[1] * (P[4][5] * SH_IAS.v[2] + P[5][5] * SH_IAS.v[1] -
                            P[14][5] * SH_IAS.v[2] - P[15][5] * SH_IAS.v[1] +
                            P[6][5] * vd * SH_IAS.v[0]) -
             SH_IAS.v[2] * (P[4][14] * SH_IAS.v[2] + P[5][14] * SH_IAS.v[1] -
                            P[14][14] * SH_IAS.v[2] - P[15][14] * SH_IAS.v[1] +
                            P[6][14] * vd * SH_IAS.v[0]) -
             SH_IAS.v[1] * (P[4][15] * SH_IAS.v[2] + P[5][15] * SH_IAS.v[1] -
                            P[14][15] * SH_IAS.v[2] - P[15][15] * SH_IAS.v[1] +
                            P[6][15] * vd * SH_IAS.v[0]) +
             vd * SH_IAS.v[0] *
                 (P[4][6] * SH_IAS.v[2] + P[5][6] * SH_IAS.v[1] -
                  P[14][6] * SH_IAS.v[2] - P[15][6] * SH_IAS.v[1] +
                  P[6][6] * vd * SH_IAS.v[0]));

        if (temp >= R_IAS)
        {
            SK_IAS = 1.0f / temp;
            faultStatus.bad_airspeed = false;
        }
        else
        {
            // the calculation is badly conditioned, so we cannot perform fusion
            // on this step we increase the wind state variances and try again next time
            P[14][14] += 0.05f * R_IAS;
            P[15][15] += 0.05f * R_IAS;
            faultStatus.bad_airspeed = true;
            return;
        }

        Kfusion[0] = SK_IAS * (P[0][4] * SH_IAS.v[2] - P[0][14] * SH_IAS.v[2] +
                               P[0][5] * SH_IAS.v[1] - P[0][15] * SH_IAS.v[1] +
                               P[0][6] * vd * SH_IAS.v[0]);
        Kfusion[1] = SK_IAS * (P[1][4] * SH_IAS.v[2] - P[1][14] * SH_IAS.v[2] +
                               P[1][5] * SH_IAS.v[1] - P[1][15] * SH_IAS.v[1] +
                               P[1][6] * vd * SH_IAS.v[0]);
        Kfusion[2] = SK_IAS * (P[2][4] * SH_IAS.v[2] - P[2][14] * SH_IAS.v[2] +
                               P[2][5] * SH_IAS.v[1] - P[2][15] * SH_IAS.v[1] +
                               P[2][6] * vd * SH_IAS.v[0]);
        Kfusion[3] = SK_IAS * (P[3][4] * SH_IAS.v[2] - P[3][14] * SH_IAS.v[2] +
                               P[3][5] * SH_IAS.v[1] - P[3][15] * SH_IAS.v[1] +
                               P[3][6] * vd * SH_IAS.v[0]);
        Kfusion[4] = SK_IAS * (P[4][4] * SH_IAS.v[2] - P[4][14] * SH_IAS.v[2] +
                               P[4][5] * SH_IAS.v[1] - P[4][15] * SH_IAS.v[1] +
                               P[4][6] * vd * SH_IAS.v[0]);
        Kfusion[5] = SK_IAS * (P[5][4] * SH_IAS.v[2] - P[5][14] * SH_IAS.v[2] +
                               P[5][5] * SH_IAS.v[1] - P[5][15] * SH_IAS.v[1] +
                               P[5][6] * vd * SH_IAS.v[0]);
        Kfusion[6] = SK_IAS * (P[6][4] * SH_IAS.v[2] - P[6][14] * SH_IAS.v[2] +
                               P[6][5] * SH_IAS.v[1] - P[6][15] * SH_IAS.v[1] +
                               P[6][6] * vd * SH_IAS.v[0]);
        Kfusion[7] = SK_IAS * (P[7][4] * SH_IAS.v[2] - P[7][14] * SH_IAS.v[2] +
                               P[7][5] * SH_IAS.v[1] - P[7][15] * SH_IAS.v[1] +
                               P[7][6] * vd * SH_IAS.v[0]);
        Kfusion[8] = SK_IAS * (P[8][4] * SH_IAS.v[2] - P[8][14] * SH_IAS.v[2] +
                               P[8][5] * SH_IAS.v[1] - P[8][15] * SH_IAS.v[1] +
                               P[8][6] * vd * SH_IAS.v[0]);
        Kfusion[9] = SK_IAS * (P[9][4] * SH_IAS.v[2] - P[9][14] * SH_IAS.v[2] +
                               P[9][5] * SH_IAS.v[1] - P[9][15] * SH_IAS.v[1] +
                               P[9][6] * vd * SH_IAS.v[0]);
        Kfusion[10] =
            SK_IAS * (P[10][4] * SH_IAS.v[2] - P[10][14] * SH_IAS.v[2] +
                      P[10][5] * SH_IAS.v[1] - P[10][15] * SH_IAS.v[1] +
                      P[10][6] * vd * SH_IAS.v[0]);
        Kfusion[11] =
            SK_IAS * (P[11][4] * SH_IAS.v[2] - P[11][14] * SH_IAS.v[2] +
                      P[11][5] * SH_IAS.v[1] - P[11][15] * SH_IAS.v[1] +
                      P[11][6] * vd * SH_IAS.v[0]);
        Kfusion[12] =
            SK_IAS * (P[12][4] * SH_IAS.v[2] - P[12][14] * SH_IAS.v[2] +
                      P[12][5] * SH_IAS.v[1] - P[12][15] * SH_IAS.v[1] +
                      P[12][6] * vd * SH_IAS.v[0]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[13] =
            0.0f; // SK_IAS*(P[13][4]*SH_IAS.v[2] - P[13][14]*SH_IAS.v[2] +
                  // P[13][5]*SH_IAS.v[1] - P[13][15]*SH_IAS.v[1] +
                  // P[13][6]*vd*SH_IAS.v[0]);
        Kfusion[14] =
            SK_IAS * (P[14][4] * SH_IAS.v[2] - P[14][14] * SH_IAS.v[2] +
                      P[14][5] * SH_IAS.v[1] - P[14][15] * SH_IAS.v[1] +
                      P[14][6] * vd * SH_IAS.v[0]);
        Kfusion[15] =
            SK_IAS * (P[15][4] * SH_IAS.v[2] - P[15][14] * SH_IAS.v[2] +
                      P[15][5] * SH_IAS.v[1] - P[15][15] * SH_IAS.v[1] +
                      P[15][6] * vd * SH_IAS.v[0]);

        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates)
        {
            Kfusion[16] =
                SK_IAS * (P[16][4] * SH_IAS.v[2] - P[16][14] * SH_IAS.v[2] +
                          P[16][5] * SH_IAS.v[1] - P[16][15] * SH_IAS.v[1] +
                          P[16][6] * vd * SH_IAS.v[0]);
            Kfusion[17] =
                SK_IAS * (P[17][4] * SH_IAS.v[2] - P[17][14] * SH_IAS.v[2] +
                          P[17][5] * SH_IAS.v[1] - P[17][15] * SH_IAS.v[1] +
                          P[17][6] * vd * SH_IAS.v[0]);
            Kfusion[18] =
                SK_IAS * (P[18][4] * SH_IAS.v[2] - P[18][14] * SH_IAS.v[2] +
                          P[18][5] * SH_IAS.v[1] - P[18][15] * SH_IAS.v[1] +
                          P[18][6] * vd * SH_IAS.v[0]);
            Kfusion[19] =
                SK_IAS * (P[19][4] * SH_IAS.v[2] - P[19][14] * SH_IAS.v[2] +
                          P[19][5] * SH_IAS.v[1] - P[19][15] * SH_IAS.v[1] +
                          P[19][6] * vd * SH_IAS.v[0]);
            Kfusion[20] =
                SK_IAS * (P[20][4] * SH_IAS.v[2] - P[20][14] * SH_IAS.v[2] +
                          P[20][5] * SH_IAS.v[1] - P[20][15] * SH_IAS.v[1] +
                          P[20][6] * vd * SH_IAS.v[0]);
            Kfusion[21] =
                SK_IAS * (P[21][4] * SH_IAS.v[2] - P[21][14] * SH_IAS.v[2] +
                          P[21][5] * SH_IAS.v[1] - P[21][15] * SH_IAS.v[1] +
                          P[21][6] * vd * SH_IAS.v[0]);
        }
        else
        {
            for (uint8_t i = 16; i <= 21; i++)
            {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate measurement innovation variance
        varInnovVias = 1.0f / SK_IAS;

        // calculate measurement innovation
        innovVias = VtasPred - ViasMeas;

        // calculate the innovation consistency test ratio
        iasTestRatio = sq(innovVias) / (sq(ekfParam.iasInnovGate) * varInnovVias);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        iasHealth = ((iasTestRatio < 1.0f) || badIMUdata);
        iasTimeout = (imuSampleTime_ms - lastIASPassTime) > iasRetryTime;

        // test the ratio before fusing data, forcing fusion if airspeed and
        // position are timed out as we have no choice but to try and use
        // airspeed to constrain error growth
        if (iasHealth || (iasTimeout && posTimeout))
        {
            // restart the counter
            lastIASPassTime = imuSampleTime_ms;

            // correct the state vector
            for (uint8_t j = 0; j <= 21; j++)
            {
                ekfStates.statesArray[j] =
                    ekfStates.statesArray[j] - Kfusion[j] * innovVias;
            }

            // normalise the quaternion states
            quaternionNormalize(&ekfStates.stateStruct.quat, &ekfStates.stateStruct.quat);

            // correct the covariance P = (I - K * H) * P take advantage of the empty columns in H to reduce the number of operations
            for (uint8_t i = 0; i <= 21; i++)
            {
                for (uint8_t j = 0; j <= 3; j++)
                    KH[i][j] = 0.0f;
                for (uint8_t j = 4; j <= 6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_IAS[j];
                }
                for (uint8_t j = 7; j <= 13; j++)
                    KH[i][j] = 0.0f;
                for (uint8_t j = 14; j <= 15; j++)
                {
                    KH[i][j] = Kfusion[i] * H_IAS[j];
                }
                for (uint8_t j = 16; j <= 21; j++)
                    KH[i][j] = 0.0f;
            }
            for (uint8_t i = 0; i <= 21; i++)
            {
                for (uint8_t j = 0; j <= 21; j++)
                {
                    KHP[i][j] = 0;
                    for (uint8_t k = 4; k <= 6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    for (uint8_t k = 14; k <= 15; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
            for (uint8_t i = 0; i <= 21; i++)
            {
                for (uint8_t j = 0; j <= 21; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ekf_ForceSymmetry();
    ekf_ConstrainVariances();
}

// fuse sythetic sideslip measurement of zero
void ekf_FuseSideslip(void)
{
    // declarations
    float q0;
    float q1;
    float q2;
    float q3;
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    const float R_BETA = 0.03f; // assume a sideslip angle RMS of ~10 deg
    Vector13 SH_BETA;
    Vector8 SK_BETA;
    fpVector3_t vel_rel_wind;
    Vector22 H_BETA;
    float innovBeta;

    // copy required states to local variable names
    q0 = ekfStates.stateStruct.quat.q0;
    q1 = ekfStates.stateStruct.quat.q1;
    q2 = ekfStates.stateStruct.quat.q2;
    q3 = ekfStates.stateStruct.quat.q3;
    vn = ekfStates.stateStruct.velocity.x;
    ve = ekfStates.stateStruct.velocity.y;
    vd = ekfStates.stateStruct.velocity.z;
    vwn = ekfStates.stateStruct.wind_vel.x;
    vwe = ekfStates.stateStruct.wind_vel.y;

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = vn - vwn;
    vel_rel_wind.y = ve - vwe;
    vel_rel_wind.z = vd;

    // rotate into body axes
    vel_rel_wind = multiplyMatrixByVector(prevTnb, vel_rel_wind);

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        SH_BETA[0] = (vn - vwn) * (sq(q0) + sq(q1) - sq(q2) - sq(q3)) -
                     vd * (2 * q0 * q2 - 2 * q1 * q3) +
                     (ve - vwe) * (2 * q0 * q3 + 2 * q1 * q2);

        if (fabsf(SH_BETA[0]) <= 1e-9f)
        {
            faultStatus.bad_sideslip = true;
            return;
        }
        else
        {
            faultStatus.bad_sideslip = false;
        }

        SH_BETA[1] = (ve - vwe) * (sq(q0) - sq(q1) + sq(q2) - sq(q3)) +
                     vd * (2 * q0 * q1 + 2 * q2 * q3) -
                     (vn - vwn) * (2 * q0 * q3 - 2 * q1 * q2);
        SH_BETA[2] = vn - vwn;
        SH_BETA[3] = ve - vwe;
        SH_BETA[4] = 1 / sq(SH_BETA[0]);
        SH_BETA[5] = 1 / SH_BETA[0];
        SH_BETA[6] = SH_BETA[5] * (sq(q0) - sq(q1) + sq(q2) - sq(q3));
        SH_BETA[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_BETA[8] = 2 * q0 * SH_BETA[3] - 2 * q3 * SH_BETA[2] + 2 * q1 * vd;
        SH_BETA[9] = 2 * q0 * SH_BETA[2] + 2 * q3 * SH_BETA[3] - 2 * q2 * vd;
        SH_BETA[10] = 2 * q2 * SH_BETA[2] - 2 * q1 * SH_BETA[3] + 2 * q0 * vd;
        SH_BETA[11] = 2 * q1 * SH_BETA[2] + 2 * q2 * SH_BETA[3] + 2 * q3 * vd;
        SH_BETA[12] = 2 * q0 * q3;
        for (uint8_t i = 0; i <= 21; i++)
        {
            H_BETA[i] = 0.0f;
        }
        H_BETA[0] =
            SH_BETA[5] * SH_BETA[8] - SH_BETA[1] * SH_BETA[4] * SH_BETA[9];
        H_BETA[1] =
            SH_BETA[5] * SH_BETA[10] - SH_BETA[1] * SH_BETA[4] * SH_BETA[11];
        H_BETA[2] =
            SH_BETA[5] * SH_BETA[11] + SH_BETA[1] * SH_BETA[4] * SH_BETA[10];
        H_BETA[3] =
            -SH_BETA[5] * SH_BETA[9] - SH_BETA[1] * SH_BETA[4] * SH_BETA[8];
        H_BETA[4] = -SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) -
                    SH_BETA[1] * SH_BETA[4] * SH_BETA[7];
        H_BETA[5] =
            SH_BETA[6] - SH_BETA[1] * SH_BETA[4] * (SH_BETA[12] + 2 * q1 * q2);
        H_BETA[6] = SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                    SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3);
        H_BETA[14] = SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                     SH_BETA[1] * SH_BETA[4] * SH_BETA[7];
        H_BETA[15] =
            SH_BETA[1] * SH_BETA[4] * (SH_BETA[12] + 2 * q1 * q2) - SH_BETA[6];

        // Calculate Kalman gains
        float temp =
            (R_BETA -
             (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) *
                 (P[14][4] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][4] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][4] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][4] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][4] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][4] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][4] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][4] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][4] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) +
             (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) *
                 (P[14][14] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                               SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][14] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][14] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][14] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                                (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][14] * (SH_BETA[5] * SH_BETA[8] -
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][14] * (SH_BETA[5] * SH_BETA[10] -
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][14] * (SH_BETA[5] * SH_BETA[11] +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][14] * (SH_BETA[5] * SH_BETA[9] +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][14] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) +
             (SH_BETA[6] -
              SH_BETA[1] * SH_BETA[4] * (SH_BETA[12] + 2 * q1 * q2)) *
                 (P[14][5] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][5] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][5] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][5] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][5] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][5] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][5] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][5] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][5] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) -
             (SH_BETA[6] -
              SH_BETA[1] * SH_BETA[4] * (SH_BETA[12] + 2 * q1 * q2)) *
                 (P[14][15] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                               SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][15] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][15] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][15] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                                (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][15] * (SH_BETA[5] * SH_BETA[8] -
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][15] * (SH_BETA[5] * SH_BETA[10] -
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][15] * (SH_BETA[5] * SH_BETA[11] +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][15] * (SH_BETA[5] * SH_BETA[9] +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][15] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) +
             (SH_BETA[5] * SH_BETA[8] - SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) *
                 (P[14][0] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][0] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][0] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][0] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][0] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][0] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][0] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][0] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][0] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) +
             (SH_BETA[5] * SH_BETA[10] -
              SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) *
                 (P[14][1] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][1] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][1] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][1] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][1] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][1] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][1] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][1] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][1] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) +
             (SH_BETA[5] * SH_BETA[11] +
              SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) *
                 (P[14][2] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][2] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][2] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][2] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][2] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][2] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][2] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][2] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][2] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) -
             (SH_BETA[5] * SH_BETA[9] + SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) *
                 (P[14][3] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][3] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][3] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][3] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][3] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][3] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][3] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][3] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][3] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))) +
             (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
              SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3)) *
                 (P[14][6] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                              SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) -
                  P[4][6] * (SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[7]) +
                  P[5][6] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                              (SH_BETA[12] + 2 * q1 * q2)) -
                  P[15][6] * (SH_BETA[6] - SH_BETA[1] * SH_BETA[4] *
                                               (SH_BETA[12] + 2 * q1 * q2)) +
                  P[0][6] * (SH_BETA[5] * SH_BETA[8] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[9]) +
                  P[1][6] * (SH_BETA[5] * SH_BETA[10] -
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[11]) +
                  P[2][6] * (SH_BETA[5] * SH_BETA[11] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[10]) -
                  P[3][6] * (SH_BETA[5] * SH_BETA[9] +
                             SH_BETA[1] * SH_BETA[4] * SH_BETA[8]) +
                  P[6][6] *
                      (SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                       SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3))));

        if (temp >= R_BETA)
        {
            SK_BETA[0] = 1.0f / temp;
            faultStatus.bad_sideslip = false;
        }
        else
        {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            faultStatus.bad_sideslip = true;
            return;
        }

        SK_BETA[1] = SH_BETA[5] * (SH_BETA[12] - 2 * q1 * q2) +
                     SH_BETA[1] * SH_BETA[4] * SH_BETA[7];
        SK_BETA[2] =
            SH_BETA[6] - SH_BETA[1] * SH_BETA[4] * (SH_BETA[12] + 2 * q1 * q2);
        SK_BETA[3] = SH_BETA[5] * (2 * q0 * q1 + 2 * q2 * q3) +
                     SH_BETA[1] * SH_BETA[4] * (2 * q0 * q2 - 2 * q1 * q3);
        SK_BETA[4] =
            SH_BETA[5] * SH_BETA[10] - SH_BETA[1] * SH_BETA[4] * SH_BETA[11];
        SK_BETA[5] =
            SH_BETA[5] * SH_BETA[8] - SH_BETA[1] * SH_BETA[4] * SH_BETA[9];
        SK_BETA[6] =
            SH_BETA[5] * SH_BETA[11] + SH_BETA[1] * SH_BETA[4] * SH_BETA[10];
        SK_BETA[7] =
            SH_BETA[5] * SH_BETA[9] + SH_BETA[1] * SH_BETA[4] * SH_BETA[8];
        Kfusion[0] =
            SK_BETA[0] * (P[0][0] * SK_BETA[5] + P[0][1] * SK_BETA[4] -
                          P[0][4] * SK_BETA[1] + P[0][5] * SK_BETA[2] +
                          P[0][2] * SK_BETA[6] + P[0][6] * SK_BETA[3] -
                          P[0][3] * SK_BETA[7] + P[0][14] * SK_BETA[1] -
                          P[0][15] * SK_BETA[2]);
        Kfusion[1] =
            SK_BETA[0] * (P[1][0] * SK_BETA[5] + P[1][1] * SK_BETA[4] -
                          P[1][4] * SK_BETA[1] + P[1][5] * SK_BETA[2] +
                          P[1][2] * SK_BETA[6] + P[1][6] * SK_BETA[3] -
                          P[1][3] * SK_BETA[7] + P[1][14] * SK_BETA[1] -
                          P[1][15] * SK_BETA[2]);
        Kfusion[2] =
            SK_BETA[0] * (P[2][0] * SK_BETA[5] + P[2][1] * SK_BETA[4] -
                          P[2][4] * SK_BETA[1] + P[2][5] * SK_BETA[2] +
                          P[2][2] * SK_BETA[6] + P[2][6] * SK_BETA[3] -
                          P[2][3] * SK_BETA[7] + P[2][14] * SK_BETA[1] -
                          P[2][15] * SK_BETA[2]);
        Kfusion[3] =
            SK_BETA[0] * (P[3][0] * SK_BETA[5] + P[3][1] * SK_BETA[4] -
                          P[3][4] * SK_BETA[1] + P[3][5] * SK_BETA[2] +
                          P[3][2] * SK_BETA[6] + P[3][6] * SK_BETA[3] -
                          P[3][3] * SK_BETA[7] + P[3][14] * SK_BETA[1] -
                          P[3][15] * SK_BETA[2]);
        Kfusion[4] =
            SK_BETA[0] * (P[4][0] * SK_BETA[5] + P[4][1] * SK_BETA[4] -
                          P[4][4] * SK_BETA[1] + P[4][5] * SK_BETA[2] +
                          P[4][2] * SK_BETA[6] + P[4][6] * SK_BETA[3] -
                          P[4][3] * SK_BETA[7] + P[4][14] * SK_BETA[1] -
                          P[4][15] * SK_BETA[2]);
        Kfusion[5] =
            SK_BETA[0] * (P[5][0] * SK_BETA[5] + P[5][1] * SK_BETA[4] -
                          P[5][4] * SK_BETA[1] + P[5][5] * SK_BETA[2] +
                          P[5][2] * SK_BETA[6] + P[5][6] * SK_BETA[3] -
                          P[5][3] * SK_BETA[7] + P[5][14] * SK_BETA[1] -
                          P[5][15] * SK_BETA[2]);
        Kfusion[6] =
            SK_BETA[0] * (P[6][0] * SK_BETA[5] + P[6][1] * SK_BETA[4] -
                          P[6][4] * SK_BETA[1] + P[6][5] * SK_BETA[2] +
                          P[6][2] * SK_BETA[6] + P[6][6] * SK_BETA[3] -
                          P[6][3] * SK_BETA[7] + P[6][14] * SK_BETA[1] -
                          P[6][15] * SK_BETA[2]);
        Kfusion[7] =
            SK_BETA[0] * (P[7][0] * SK_BETA[5] + P[7][1] * SK_BETA[4] -
                          P[7][4] * SK_BETA[1] + P[7][5] * SK_BETA[2] +
                          P[7][2] * SK_BETA[6] + P[7][6] * SK_BETA[3] -
                          P[7][3] * SK_BETA[7] + P[7][14] * SK_BETA[1] -
                          P[7][15] * SK_BETA[2]);
        Kfusion[8] =
            SK_BETA[0] * (P[8][0] * SK_BETA[5] + P[8][1] * SK_BETA[4] -
                          P[8][4] * SK_BETA[1] + P[8][5] * SK_BETA[2] +
                          P[8][2] * SK_BETA[6] + P[8][6] * SK_BETA[3] -
                          P[8][3] * SK_BETA[7] + P[8][14] * SK_BETA[1] -
                          P[8][15] * SK_BETA[2]);
        Kfusion[9] =
            SK_BETA[0] * (P[9][0] * SK_BETA[5] + P[9][1] * SK_BETA[4] -
                          P[9][4] * SK_BETA[1] + P[9][5] * SK_BETA[2] +
                          P[9][2] * SK_BETA[6] + P[9][6] * SK_BETA[3] -
                          P[9][3] * SK_BETA[7] + P[9][14] * SK_BETA[1] -
                          P[9][15] * SK_BETA[2]);
        Kfusion[10] =
            SK_BETA[0] * (P[10][0] * SK_BETA[5] + P[10][1] * SK_BETA[4] -
                          P[10][4] * SK_BETA[1] + P[10][5] * SK_BETA[2] +
                          P[10][2] * SK_BETA[6] + P[10][6] * SK_BETA[3] -
                          P[10][3] * SK_BETA[7] + P[10][14] * SK_BETA[1] -
                          P[10][15] * SK_BETA[2]);
        Kfusion[11] =
            SK_BETA[0] * (P[11][0] * SK_BETA[5] + P[11][1] * SK_BETA[4] -
                          P[11][4] * SK_BETA[1] + P[11][5] * SK_BETA[2] +
                          P[11][2] * SK_BETA[6] + P[11][6] * SK_BETA[3] -
                          P[11][3] * SK_BETA[7] + P[11][14] * SK_BETA[1] -
                          P[11][15] * SK_BETA[2]);
        Kfusion[12] =
            SK_BETA[0] * (P[12][0] * SK_BETA[5] + P[12][1] * SK_BETA[4] -
                          P[12][4] * SK_BETA[1] + P[12][5] * SK_BETA[2] +
                          P[12][2] * SK_BETA[6] + P[12][6] * SK_BETA[3] -
                          P[12][3] * SK_BETA[7] + P[12][14] * SK_BETA[1] -
                          P[12][15] * SK_BETA[2]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[13] = 0.0f; // SK_BETA[0]*(P[13][0]*SK_BETA[5] +
                            // P[13][1]*SK_BETA[4] - P[13][4]*SK_BETA[1] +
                            // P[13][5]*SK_BETA[2] + P[13][2]*SK_BETA[6] +
                            // P[13][6]*SK_BETA[3] - P[13][3]*SK_BETA[7] +
                            // P[13][14]*SK_BETA[1] - P[13][15]*SK_BETA[2]);
        Kfusion[14] =
            SK_BETA[0] * (P[14][0] * SK_BETA[5] + P[14][1] * SK_BETA[4] -
                          P[14][4] * SK_BETA[1] + P[14][5] * SK_BETA[2] +
                          P[14][2] * SK_BETA[6] + P[14][6] * SK_BETA[3] -
                          P[14][3] * SK_BETA[7] + P[14][14] * SK_BETA[1] -
                          P[14][15] * SK_BETA[2]);
        Kfusion[15] =
            SK_BETA[0] * (P[15][0] * SK_BETA[5] + P[15][1] * SK_BETA[4] -
                          P[15][4] * SK_BETA[1] + P[15][5] * SK_BETA[2] +
                          P[15][2] * SK_BETA[6] + P[15][6] * SK_BETA[3] -
                          P[15][3] * SK_BETA[7] + P[15][14] * SK_BETA[1] -
                          P[15][15] * SK_BETA[2]);

        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates)
        {
            Kfusion[16] =
                SK_BETA[0] * (P[16][0] * SK_BETA[5] + P[16][1] * SK_BETA[4] -
                              P[16][4] * SK_BETA[1] + P[16][5] * SK_BETA[2] +
                              P[16][2] * SK_BETA[6] + P[16][6] * SK_BETA[3] -
                              P[16][3] * SK_BETA[7] + P[16][14] * SK_BETA[1] -
                              P[16][15] * SK_BETA[2]);
            Kfusion[17] =
                SK_BETA[0] * (P[17][0] * SK_BETA[5] + P[17][1] * SK_BETA[4] -
                              P[17][4] * SK_BETA[1] + P[17][5] * SK_BETA[2] +
                              P[17][2] * SK_BETA[6] + P[17][6] * SK_BETA[3] -
                              P[17][3] * SK_BETA[7] + P[17][14] * SK_BETA[1] -
                              P[17][15] * SK_BETA[2]);
            Kfusion[18] =
                SK_BETA[0] * (P[18][0] * SK_BETA[5] + P[18][1] * SK_BETA[4] -
                              P[18][4] * SK_BETA[1] + P[18][5] * SK_BETA[2] +
                              P[18][2] * SK_BETA[6] + P[18][6] * SK_BETA[3] -
                              P[18][3] * SK_BETA[7] + P[18][14] * SK_BETA[1] -
                              P[18][15] * SK_BETA[2]);
            Kfusion[19] =
                SK_BETA[0] * (P[19][0] * SK_BETA[5] + P[19][1] * SK_BETA[4] -
                              P[19][4] * SK_BETA[1] + P[19][5] * SK_BETA[2] +
                              P[19][2] * SK_BETA[6] + P[19][6] * SK_BETA[3] -
                              P[19][3] * SK_BETA[7] + P[19][14] * SK_BETA[1] -
                              P[19][15] * SK_BETA[2]);
            Kfusion[20] =
                SK_BETA[0] * (P[20][0] * SK_BETA[5] + P[20][1] * SK_BETA[4] -
                              P[20][4] * SK_BETA[1] + P[20][5] * SK_BETA[2] +
                              P[20][2] * SK_BETA[6] + P[20][6] * SK_BETA[3] -
                              P[20][3] * SK_BETA[7] + P[20][14] * SK_BETA[1] -
                              P[20][15] * SK_BETA[2]);
            Kfusion[21] =
                SK_BETA[0] * (P[21][0] * SK_BETA[5] + P[21][1] * SK_BETA[4] -
                              P[21][4] * SK_BETA[1] + P[21][5] * SK_BETA[2] +
                              P[21][2] * SK_BETA[6] + P[21][6] * SK_BETA[3] -
                              P[21][3] * SK_BETA[7] + P[21][14] * SK_BETA[1] -
                              P[21][15] * SK_BETA[2]);
        }
        else
        {
            for (uint8_t i = 16; i <= 21; i++)
            {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate predicted sideslip angle and innovation using small angle approximation
        innovBeta = vel_rel_wind.y / vel_rel_wind.x;

        // reject measurement if greater than 3-sigma inconsistency
        if (innovBeta > 0.5f)
        {
            return;
        }

        // correct the state vector
        for (uint8_t j = 0; j <= 21; j++)
        {
            ekfStates.statesArray[j] = ekfStates.statesArray[j] - Kfusion[j] * innovBeta;
        }

        // normalise the quaternion states
        quaternionNormalize(&ekfStates.stateStruct.quat, &ekfStates.stateStruct.quat);

        // correct the covariance P = (I - K * H) * P take advantage of the empty columns in H to reduce the number of operations
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 6; j++)
            {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (uint8_t j = 7; j <= 13; j++)
                KH[i][j] = 0.0f;
            for (uint8_t j = 14; j <= 15; j++)
            {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (uint8_t j = 16; j <= 21; j++)
                KH[i][j] = 0.0f;
        }
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 21; j++)
            {
                KHP[i][j] = 0;
                for (uint8_t k = 0; k <= 6; k++)
                {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
                for (uint8_t k = 14; k <= 15; k++)
                {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
            }
        }
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 21; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ekf_ForceSymmetry();
    ekf_ConstrainVariances();
}

// calculate the predicted state covariance matrix
void ekf_CovariancePrediction(void)
{
    float windVelSigma;  // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma; // delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma; // delta velocity bias 1-sigma process noise - m/s
    float magEarthSigma; // earth magnetic field 1-sigma process noise
    float magBodySigma;  // body magnetic field 1-sigma process noise
    float daxCov;        // X axis delta angle variance rad^2
    float dayCov;        // Y axis delta angle variance rad^2
    float dazCov;        // Z axis delta angle variance rad^2
    float dvxCov;        // X axis delta velocity variance (m/s)^2
    float dvyCov;        // Y axis delta velocity variance (m/s)^2
    float dvzCov;        // Z axis delta velocity variance (m/s)^2
    float dvx;           // X axis delta velocity (m/s)
    float dvy;           // Y axis delta velocity (m/s)
    float dvz;           // Z axis delta velocity (m/s)
    float dax;           // X axis delta angle (rad)
    float day;           // Y axis delta angle (rad)
    float daz;           // Z axis delta angle (rad)
    float q0;            // attitude quaternion
    float q1;            // attitude quaternion
    float q2;            // attitude quaternion
    float q3;            // attitude quaternion
    float dax_b;         // X axis delta angle measurement bias (rad)
    float day_b;         // Y axis delta angle measurement bias (rad)
    float daz_b;         // Z axis delta angle measurement bias (rad)
    float dvz_b;         // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or
    // descending this allows for wind gradient effects. filter height rate
    // using a 10 second time constant filter
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - ekfStates.stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending this allows for wind gradient effects.
    if (!inhibitWindStates)
    {
        windVelSigma = dt * constrainf(ekfParam.windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrainf(ekfParam.wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    }
    else
    {
        windVelSigma = 0.0f;
    }

    dAngBiasSigma = dt * constrainf(ekfParam.gyroBiasProcessNoise, 1e-7f, 1e-5f);
    dVelBiasSigma = dt * constrainf(ekfParam.accelBiasProcessNoise, 1e-5f, 1e-3f);

    if (!inhibitMagStates)
    {
        magEarthSigma = dt * constrainf(ekfParam.magEarthProcessNoise, 1e-4f, 1e-2f);
        magBodySigma = dt * constrainf(ekfParam.magBodyProcessNoise, 1e-4f, 1e-2f);
    }
    else
    {
        magEarthSigma = 0.0f;
        magBodySigma = 0.0f;
    }

    for (uint8_t i = 0; i <= 9; i++)
        processNoise[i] = 1.0e-9f;

    // scale gyro bias noise when in vehicle disarmed to allow for faster bias estimation
    for (uint8_t i = 10; i <= 12; i++)
    {
        processNoise[i] = dAngBiasSigma;
        if (!vehicleArmed)
        {
            processNoise[i] *= gyroBiasNoiseScaler;
        }
    }

    // if we are yawing rapidly, inhibit yaw gyro bias learning to prevent gyro scale factor errors from corrupting the bias estimate
    if (highYawRate)
    {
        processNoise[12] = 0.0f;
        P[12][12] = 0.0f;
    }

    processNoise[13] = dVelBiasSigma;

    for (uint8_t i = 14; i <= 15; i++)
        processNoise[i] = windVelSigma;
    for (uint8_t i = 16; i <= 18; i++)
        processNoise[i] = magEarthSigma;
    for (uint8_t i = 19; i <= 21; i++)
        processNoise[i] = magBodySigma;
    for (uint8_t i = 0; i <= 21; i++)
        processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = ekfStates.stateStruct.quat.q0;
    q1 = ekfStates.stateStruct.quat.q1;
    q2 = ekfStates.stateStruct.quat.q2;
    q3 = ekfStates.stateStruct.quat.q3;
    dax_b = ekfStates.stateStruct.gyro_bias.x;
    day_b = ekfStates.stateStruct.gyro_bias.y;
    daz_b = ekfStates.stateStruct.gyro_bias.z;
    dvz_b = IMU1_weighting * ekfStates.stateStruct.accel_zbias1 + (1.0f - IMU1_weighting) * ekfStates.stateStruct.accel_zbias2;
    ekfParam.gyroNoise = constrainf(ekfParam.gyroNoise, 1e-3f, 5e-2f);
    daxCov = sq(dt * ekfParam.gyroNoise);
    dayCov = sq(dt * ekfParam.gyroNoise);
    // Account for 3% scale factor error on Z angular rate. This reduces chance of continuous fast rotations causing loss of yaw reference.
    dazCov = sq(dt * ekfParam.gyroNoise) + sq(dt * 0.03f * yawRateFilt);
    ekfParam.accNoise = constrainf(ekfParam.accNoise, 5e-2f, 1.0f);
    dvxCov = sq(dt * ekfParam.accNoise);
    dvyCov = sq(dt * ekfParam.accNoise);
    dvzCov = sq(dt * ekfParam.accNoise);

    // calculate the predicted covariance due to inertial sensor error propagation
    SF[0] = dvz - dvz_b;
    SF[1] = 2 * q3 * SF[0] + 2 * dvx * q1 + 2 * dvy * q2;
    SF[2] = 2 * dvx * q3 - 2 * q1 * SF[0] + 2 * dvy * q0;
    SF[3] = 2 * q2 * SF[0] + 2 * dvx * q0 - 2 * dvy * q3;
    SF[4] = day / 2 - day_b / 2;
    SF[5] = daz / 2 - daz_b / 2;
    SF[6] = dax / 2 - dax_b / 2;
    SF[7] = dax_b / 2 - dax / 2;
    SF[8] = daz_b / 2 - daz / 2;
    SF[9] = day_b / 2 - day / 2;
    SF[10] = 2 * q0 * SF[0];
    SF[11] = q1 / 2;
    SF[12] = q2 / 2;
    SF[13] = q3 / 2;
    SF[14] = 2 * dvy * q1;

    SG[0] = q0 / 2;
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);
    SG[5] = 2 * q2 * q3;
    SG[6] = 2 * q1 * q3;
    SG[7] = 2 * q1 * q2;

    SQ[0] = dvzCov * (SG[5] - 2 * q0 * q1) * (SG[1] - SG[2] - SG[3] + SG[4]) -
            dvyCov * (SG[5] + 2 * q0 * q1) * (SG[1] - SG[2] + SG[3] - SG[4]) +
            dvxCov * (SG[6] - 2 * q0 * q2) * (SG[7] + 2 * q0 * q3);
    SQ[1] = dvzCov * (SG[6] + 2 * q0 * q2) * (SG[1] - SG[2] - SG[3] + SG[4]) -
            dvxCov * (SG[6] - 2 * q0 * q2) * (SG[1] + SG[2] - SG[3] - SG[4]) +
            dvyCov * (SG[5] + 2 * q0 * q1) * (SG[7] - 2 * q0 * q3);
    SQ[2] = dvzCov * (SG[5] - 2 * q0 * q1) * (SG[6] + 2 * q0 * q2) -
            dvyCov * (SG[7] - 2 * q0 * q3) * (SG[1] - SG[2] + SG[3] - SG[4]) -
            dvxCov * (SG[7] + 2 * q0 * q3) * (SG[1] + SG[2] - SG[3] - SG[4]);
    SQ[3] = (dayCov * q1 * SG[0]) / 2 - (dazCov * q1 * SG[0]) / 2 -
            (daxCov * q2 * q3) / 4;
    SQ[4] = (dazCov * q2 * SG[0]) / 2 - (daxCov * q2 * SG[0]) / 2 -
            (dayCov * q1 * q3) / 4;
    SQ[5] = (daxCov * q3 * SG[0]) / 2 - (dayCov * q3 * SG[0]) / 2 -
            (dazCov * q1 * q2) / 4;
    SQ[6] = (daxCov * q1 * q2) / 4 - (dazCov * q3 * SG[0]) / 2 -
            (dayCov * q1 * q2) / 4;
    SQ[7] = (dazCov * q1 * q3) / 4 - (daxCov * q1 * q3) / 4 -
            (dayCov * q2 * SG[0]) / 2;
    SQ[8] = (dayCov * q2 * q3) / 4 - (daxCov * q1 * SG[0]) / 2 -
            (dazCov * q2 * q3) / 4;
    SQ[9] = sq(SG[0]);
    SQ[10] = sq(q1);

    SPP[0] = SF[10] + SF[14] - 2 * dvx * q2;
    SPP[1] = 2 * q2 * SF[0] + 2 * dvx * q0 - 2 * dvy * q3;
    SPP[2] = 2 * dvx * q3 - 2 * q1 * SF[0] + 2 * dvy * q0;
    SPP[3] = 2 * q0 * q1 - 2 * q2 * q3;
    SPP[4] = 2 * q0 * q2 + 2 * q1 * q3;
    SPP[5] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    SPP[6] = SF[13];
    SPP[7] = SF[12];

    nextP[0][0] =
        P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
        P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6] +
        (daxCov * SQ[10]) / 4 +
        SF[7] * (P[0][1] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
                 P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6]) +
        SF[9] * (P[0][2] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
                 P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6]) +
        SF[8] * (P[0][3] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
                 P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6]) +
        SF[11] *
            (P[0][10] + P[1][10] * SF[7] + P[2][10] * SF[9] + P[3][10] * SF[8] +
             P[10][10] * SF[11] + P[11][10] * SPP[7] + P[12][10] * SPP[6]) +
        SPP[7] *
            (P[0][11] + P[1][11] * SF[7] + P[2][11] * SF[9] + P[3][11] * SF[8] +
             P[10][11] * SF[11] + P[11][11] * SPP[7] + P[12][11] * SPP[6]) +
        SPP[6] *
            (P[0][12] + P[1][12] * SF[7] + P[2][12] * SF[9] + P[3][12] * SF[8] +
             P[10][12] * SF[11] + P[11][12] * SPP[7] + P[12][12] * SPP[6]) +
        (dayCov * sq(q2)) / 4 + (dazCov * sq(q3)) / 4;
    nextP[0][1] =
        P[0][1] + SQ[8] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
        P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6] +
        SF[6] * (P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
                 P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6]) +
        SF[5] * (P[0][2] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
                 P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6]) +
        SF[9] * (P[0][3] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
                 P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6]) +
        SPP[6] *
            (P[0][11] + P[1][11] * SF[7] + P[2][11] * SF[9] + P[3][11] * SF[8] +
             P[10][11] * SF[11] + P[11][11] * SPP[7] + P[12][11] * SPP[6]) -
        SPP[7] *
            (P[0][12] + P[1][12] * SF[7] + P[2][12] * SF[9] + P[3][12] * SF[8] +
             P[10][12] * SF[11] + P[11][12] * SPP[7] + P[12][12] * SPP[6]) -
        (q0 *
         (P[0][10] + P[1][10] * SF[7] + P[2][10] * SF[9] + P[3][10] * SF[8] +
          P[10][10] * SF[11] + P[11][10] * SPP[7] + P[12][10] * SPP[6])) /
            2;
    nextP[0][2] =
        P[0][2] + SQ[7] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
        P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6] +
        SF[4] * (P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
                 P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6]) +
        SF[8] * (P[0][1] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
                 P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6]) +
        SF[6] * (P[0][3] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
                 P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6]) +
        SF[11] *
            (P[0][12] + P[1][12] * SF[7] + P[2][12] * SF[9] + P[3][12] * SF[8] +
             P[10][12] * SF[11] + P[11][12] * SPP[7] + P[12][12] * SPP[6]) -
        SPP[6] *
            (P[0][10] + P[1][10] * SF[7] + P[2][10] * SF[9] + P[3][10] * SF[8] +
             P[10][10] * SF[11] + P[11][10] * SPP[7] + P[12][10] * SPP[6]) -
        (q0 *
         (P[0][11] + P[1][11] * SF[7] + P[2][11] * SF[9] + P[3][11] * SF[8] +
          P[10][11] * SF[11] + P[11][11] * SPP[7] + P[12][11] * SPP[6])) /
            2;
    nextP[0][3] =
        P[0][3] + SQ[6] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
        P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6] +
        SF[5] * (P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
                 P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6]) +
        SF[4] * (P[0][1] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
                 P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6]) +
        SF[7] * (P[0][2] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
                 P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6]) -
        SF[11] *
            (P[0][11] + P[1][11] * SF[7] + P[2][11] * SF[9] + P[3][11] * SF[8] +
             P[10][11] * SF[11] + P[11][11] * SPP[7] + P[12][11] * SPP[6]) +
        SPP[7] *
            (P[0][10] + P[1][10] * SF[7] + P[2][10] * SF[9] + P[3][10] * SF[8] +
             P[10][10] * SF[11] + P[11][10] * SPP[7] + P[12][10] * SPP[6]) -
        (q0 *
         (P[0][12] + P[1][12] * SF[7] + P[2][12] * SF[9] + P[3][12] * SF[8] +
          P[10][12] * SF[11] + P[11][12] * SPP[7] + P[12][12] * SPP[6])) /
            2;
    nextP[0][4] =
        P[0][4] + P[1][4] * SF[7] + P[2][4] * SF[9] + P[3][4] * SF[8] +
        P[10][4] * SF[11] + P[11][4] * SPP[7] + P[12][4] * SPP[6] +
        SF[3] * (P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
                 P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6]) +
        SF[1] * (P[0][1] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
                 P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6]) +
        SPP[0] *
            (P[0][2] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
             P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6]) -
        SPP[2] *
            (P[0][3] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
             P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6]) -
        SPP[4] *
            (P[0][13] + P[1][13] * SF[7] + P[2][13] * SF[9] + P[3][13] * SF[8] +
             P[10][13] * SF[11] + P[11][13] * SPP[7] + P[12][13] * SPP[6]);
    nextP[0][5] =
        P[0][5] + P[1][5] * SF[7] + P[2][5] * SF[9] + P[3][5] * SF[8] +
        P[10][5] * SF[11] + P[11][5] * SPP[7] + P[12][5] * SPP[6] +
        SF[2] * (P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
                 P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6]) +
        SF[1] * (P[0][2] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
                 P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6]) +
        SF[3] * (P[0][3] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
                 P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6]) -
        SPP[0] *
            (P[0][1] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
             P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6]) +
        SPP[3] *
            (P[0][13] + P[1][13] * SF[7] + P[2][13] * SF[9] + P[3][13] * SF[8] +
             P[10][13] * SF[11] + P[11][13] * SPP[7] + P[12][13] * SPP[6]);
    nextP[0][6] =
        P[0][6] + P[1][6] * SF[7] + P[2][6] * SF[9] + P[3][6] * SF[8] +
        P[10][6] * SF[11] + P[11][6] * SPP[7] + P[12][6] * SPP[6] +
        SF[2] * (P[0][1] + P[1][1] * SF[7] + P[2][1] * SF[9] + P[3][1] * SF[8] +
                 P[10][1] * SF[11] + P[11][1] * SPP[7] + P[12][1] * SPP[6]) +
        SF[1] * (P[0][3] + P[1][3] * SF[7] + P[2][3] * SF[9] + P[3][3] * SF[8] +
                 P[10][3] * SF[11] + P[11][3] * SPP[7] + P[12][3] * SPP[6]) +
        SPP[0] *
            (P[0][0] + P[1][0] * SF[7] + P[2][0] * SF[9] + P[3][0] * SF[8] +
             P[10][0] * SF[11] + P[11][0] * SPP[7] + P[12][0] * SPP[6]) -
        SPP[1] *
            (P[0][2] + P[1][2] * SF[7] + P[2][2] * SF[9] + P[3][2] * SF[8] +
             P[10][2] * SF[11] + P[11][2] * SPP[7] + P[12][2] * SPP[6]) -
        (sq(q0) - sq(q1) - sq(q2) + sq(q3)) *
            (P[0][13] + P[1][13] * SF[7] + P[2][13] * SF[9] + P[3][13] * SF[8] +
             P[10][13] * SF[11] + P[11][13] * SPP[7] + P[12][13] * SPP[6]);
    nextP[0][7] =
        P[0][7] + P[1][7] * SF[7] + P[2][7] * SF[9] + P[3][7] * SF[8] +
        P[10][7] * SF[11] + P[11][7] * SPP[7] + P[12][7] * SPP[6] +
        dt * (P[0][4] + P[1][4] * SF[7] + P[2][4] * SF[9] + P[3][4] * SF[8] +
              P[10][4] * SF[11] + P[11][4] * SPP[7] + P[12][4] * SPP[6]);
    nextP[0][8] =
        P[0][8] + P[1][8] * SF[7] + P[2][8] * SF[9] + P[3][8] * SF[8] +
        P[10][8] * SF[11] + P[11][8] * SPP[7] + P[12][8] * SPP[6] +
        dt * (P[0][5] + P[1][5] * SF[7] + P[2][5] * SF[9] + P[3][5] * SF[8] +
              P[10][5] * SF[11] + P[11][5] * SPP[7] + P[12][5] * SPP[6]);
    nextP[0][9] =
        P[0][9] + P[1][9] * SF[7] + P[2][9] * SF[9] + P[3][9] * SF[8] +
        P[10][9] * SF[11] + P[11][9] * SPP[7] + P[12][9] * SPP[6] +
        dt * (P[0][6] + P[1][6] * SF[7] + P[2][6] * SF[9] + P[3][6] * SF[8] +
              P[10][6] * SF[11] + P[11][6] * SPP[7] + P[12][6] * SPP[6]);
    nextP[0][10] = P[0][10] + P[1][10] * SF[7] + P[2][10] * SF[9] +
                   P[3][10] * SF[8] + P[10][10] * SF[11] + P[11][10] * SPP[7] +
                   P[12][10] * SPP[6];
    nextP[0][11] = P[0][11] + P[1][11] * SF[7] + P[2][11] * SF[9] +
                   P[3][11] * SF[8] + P[10][11] * SF[11] + P[11][11] * SPP[7] +
                   P[12][11] * SPP[6];
    nextP[0][12] = P[0][12] + P[1][12] * SF[7] + P[2][12] * SF[9] +
                   P[3][12] * SF[8] + P[10][12] * SF[11] + P[11][12] * SPP[7] +
                   P[12][12] * SPP[6];
    nextP[0][13] = P[0][13] + P[1][13] * SF[7] + P[2][13] * SF[9] +
                   P[3][13] * SF[8] + P[10][13] * SF[11] + P[11][13] * SPP[7] +
                   P[12][13] * SPP[6];
    nextP[0][14] = P[0][14] + P[1][14] * SF[7] + P[2][14] * SF[9] +
                   P[3][14] * SF[8] + P[10][14] * SF[11] + P[11][14] * SPP[7] +
                   P[12][14] * SPP[6];
    nextP[0][15] = P[0][15] + P[1][15] * SF[7] + P[2][15] * SF[9] +
                   P[3][15] * SF[8] + P[10][15] * SF[11] + P[11][15] * SPP[7] +
                   P[12][15] * SPP[6];
    nextP[0][16] = P[0][16] + P[1][16] * SF[7] + P[2][16] * SF[9] +
                   P[3][16] * SF[8] + P[10][16] * SF[11] + P[11][16] * SPP[7] +
                   P[12][16] * SPP[6];
    nextP[0][17] = P[0][17] + P[1][17] * SF[7] + P[2][17] * SF[9] +
                   P[3][17] * SF[8] + P[10][17] * SF[11] + P[11][17] * SPP[7] +
                   P[12][17] * SPP[6];
    nextP[0][18] = P[0][18] + P[1][18] * SF[7] + P[2][18] * SF[9] +
                   P[3][18] * SF[8] + P[10][18] * SF[11] + P[11][18] * SPP[7] +
                   P[12][18] * SPP[6];
    nextP[0][19] = P[0][19] + P[1][19] * SF[7] + P[2][19] * SF[9] +
                   P[3][19] * SF[8] + P[10][19] * SF[11] + P[11][19] * SPP[7] +
                   P[12][19] * SPP[6];
    nextP[0][20] = P[0][20] + P[1][20] * SF[7] + P[2][20] * SF[9] +
                   P[3][20] * SF[8] + P[10][20] * SF[11] + P[11][20] * SPP[7] +
                   P[12][20] * SPP[6];
    nextP[0][21] = P[0][21] + P[1][21] * SF[7] + P[2][21] * SF[9] +
                   P[3][21] * SF[8] + P[10][21] * SF[11] + P[11][21] * SPP[7] +
                   P[12][21] * SPP[6];
    nextP[1][0] =
        P[1][0] + SQ[8] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
        P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2 +
        SF[7] * (P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
                 P[11][1] * SPP[6] - P[12][1] * SPP[7] - (P[10][1] * q0) / 2) +
        SF[9] * (P[1][2] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
                 P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2) +
        SF[8] * (P[1][3] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
                 P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2) +
        SF[11] *
            (P[1][10] + P[0][10] * SF[6] + P[2][10] * SF[5] + P[3][10] * SF[9] +
             P[11][10] * SPP[6] - P[12][10] * SPP[7] - (P[10][10] * q0) / 2) +
        SPP[7] *
            (P[1][11] + P[0][11] * SF[6] + P[2][11] * SF[5] + P[3][11] * SF[9] +
             P[11][11] * SPP[6] - P[12][11] * SPP[7] - (P[10][11] * q0) / 2) +
        SPP[6] *
            (P[1][12] + P[0][12] * SF[6] + P[2][12] * SF[5] + P[3][12] * SF[9] +
             P[11][12] * SPP[6] - P[12][12] * SPP[7] - (P[10][12] * q0) / 2);
    nextP[1][1] =
        P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
        P[11][1] * SPP[6] - P[12][1] * SPP[7] + daxCov * SQ[9] -
        (P[10][1] * q0) / 2 +
        SF[6] * (P[1][0] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
                 P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2) +
        SF[5] * (P[1][2] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
                 P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2) +
        SF[9] * (P[1][3] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
                 P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2) +
        SPP[6] *
            (P[1][11] + P[0][11] * SF[6] + P[2][11] * SF[5] + P[3][11] * SF[9] +
             P[11][11] * SPP[6] - P[12][11] * SPP[7] - (P[10][11] * q0) / 2) -
        SPP[7] *
            (P[1][12] + P[0][12] * SF[6] + P[2][12] * SF[5] + P[3][12] * SF[9] +
             P[11][12] * SPP[6] - P[12][12] * SPP[7] - (P[10][12] * q0) / 2) +
        (dayCov * sq(q3)) / 4 + (dazCov * sq(q2)) / 4 -
        (q0 *
         (P[1][10] + P[0][10] * SF[6] + P[2][10] * SF[5] + P[3][10] * SF[9] +
          P[11][10] * SPP[6] - P[12][10] * SPP[7] - (P[10][10] * q0) / 2)) /
            2;
    nextP[1][2] =
        P[1][2] + SQ[5] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
        P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2 +
        SF[4] * (P[1][0] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
                 P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2) +
        SF[8] * (P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
                 P[11][1] * SPP[6] - P[12][1] * SPP[7] - (P[10][1] * q0) / 2) +
        SF[6] * (P[1][3] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
                 P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2) +
        SF[11] *
            (P[1][12] + P[0][12] * SF[6] + P[2][12] * SF[5] + P[3][12] * SF[9] +
             P[11][12] * SPP[6] - P[12][12] * SPP[7] - (P[10][12] * q0) / 2) -
        SPP[6] *
            (P[1][10] + P[0][10] * SF[6] + P[2][10] * SF[5] + P[3][10] * SF[9] +
             P[11][10] * SPP[6] - P[12][10] * SPP[7] - (P[10][10] * q0) / 2) -
        (q0 *
         (P[1][11] + P[0][11] * SF[6] + P[2][11] * SF[5] + P[3][11] * SF[9] +
          P[11][11] * SPP[6] - P[12][11] * SPP[7] - (P[10][11] * q0) / 2)) /
            2;
    nextP[1][3] =
        P[1][3] + SQ[4] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
        P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2 +
        SF[5] * (P[1][0] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
                 P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2) +
        SF[4] * (P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
                 P[11][1] * SPP[6] - P[12][1] * SPP[7] - (P[10][1] * q0) / 2) +
        SF[7] * (P[1][2] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
                 P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2) -
        SF[11] *
            (P[1][11] + P[0][11] * SF[6] + P[2][11] * SF[5] + P[3][11] * SF[9] +
             P[11][11] * SPP[6] - P[12][11] * SPP[7] - (P[10][11] * q0) / 2) +
        SPP[7] *
            (P[1][10] + P[0][10] * SF[6] + P[2][10] * SF[5] + P[3][10] * SF[9] +
             P[11][10] * SPP[6] - P[12][10] * SPP[7] - (P[10][10] * q0) / 2) -
        (q0 *
         (P[1][12] + P[0][12] * SF[6] + P[2][12] * SF[5] + P[3][12] * SF[9] +
          P[11][12] * SPP[6] - P[12][12] * SPP[7] - (P[10][12] * q0) / 2)) /
            2;
    nextP[1][4] =
        P[1][4] + P[0][4] * SF[6] + P[2][4] * SF[5] + P[3][4] * SF[9] +
        P[11][4] * SPP[6] - P[12][4] * SPP[7] - (P[10][4] * q0) / 2 +
        SF[3] * (P[1][0] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
                 P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2) +
        SF[1] * (P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
                 P[11][1] * SPP[6] - P[12][1] * SPP[7] - (P[10][1] * q0) / 2) +
        SPP[0] *
            (P[1][2] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
             P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2) -
        SPP[2] *
            (P[1][3] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
             P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2) -
        SPP[4] *
            (P[1][13] + P[0][13] * SF[6] + P[2][13] * SF[5] + P[3][13] * SF[9] +
             P[11][13] * SPP[6] - P[12][13] * SPP[7] - (P[10][13] * q0) / 2);
    nextP[1][5] =
        P[1][5] + P[0][5] * SF[6] + P[2][5] * SF[5] + P[3][5] * SF[9] +
        P[11][5] * SPP[6] - P[12][5] * SPP[7] - (P[10][5] * q0) / 2 +
        SF[2] * (P[1][0] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
                 P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2) +
        SF[1] * (P[1][2] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
                 P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2) +
        SF[3] * (P[1][3] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
                 P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2) -
        SPP[0] *
            (P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
             P[11][1] * SPP[6] - P[12][1] * SPP[7] - (P[10][1] * q0) / 2) +
        SPP[3] *
            (P[1][13] + P[0][13] * SF[6] + P[2][13] * SF[5] + P[3][13] * SF[9] +
             P[11][13] * SPP[6] - P[12][13] * SPP[7] - (P[10][13] * q0) / 2);
    nextP[1][6] =
        P[1][6] + P[0][6] * SF[6] + P[2][6] * SF[5] + P[3][6] * SF[9] +
        P[11][6] * SPP[6] - P[12][6] * SPP[7] - (P[10][6] * q0) / 2 +
        SF[2] * (P[1][1] + P[0][1] * SF[6] + P[2][1] * SF[5] + P[3][1] * SF[9] +
                 P[11][1] * SPP[6] - P[12][1] * SPP[7] - (P[10][1] * q0) / 2) +
        SF[1] * (P[1][3] + P[0][3] * SF[6] + P[2][3] * SF[5] + P[3][3] * SF[9] +
                 P[11][3] * SPP[6] - P[12][3] * SPP[7] - (P[10][3] * q0) / 2) +
        SPP[0] *
            (P[1][0] + P[0][0] * SF[6] + P[2][0] * SF[5] + P[3][0] * SF[9] +
             P[11][0] * SPP[6] - P[12][0] * SPP[7] - (P[10][0] * q0) / 2) -
        SPP[1] *
            (P[1][2] + P[0][2] * SF[6] + P[2][2] * SF[5] + P[3][2] * SF[9] +
             P[11][2] * SPP[6] - P[12][2] * SPP[7] - (P[10][2] * q0) / 2) -
        (sq(q0) - sq(q1) - sq(q2) + sq(q3)) *
            (P[1][13] + P[0][13] * SF[6] + P[2][13] * SF[5] + P[3][13] * SF[9] +
             P[11][13] * SPP[6] - P[12][13] * SPP[7] - (P[10][13] * q0) / 2);
    nextP[1][7] =
        P[1][7] + P[0][7] * SF[6] + P[2][7] * SF[5] + P[3][7] * SF[9] +
        P[11][7] * SPP[6] - P[12][7] * SPP[7] - (P[10][7] * q0) / 2 +
        dt * (P[1][4] + P[0][4] * SF[6] + P[2][4] * SF[5] + P[3][4] * SF[9] +
              P[11][4] * SPP[6] - P[12][4] * SPP[7] - (P[10][4] * q0) / 2);
    nextP[1][8] =
        P[1][8] + P[0][8] * SF[6] + P[2][8] * SF[5] + P[3][8] * SF[9] +
        P[11][8] * SPP[6] - P[12][8] * SPP[7] - (P[10][8] * q0) / 2 +
        dt * (P[1][5] + P[0][5] * SF[6] + P[2][5] * SF[5] + P[3][5] * SF[9] +
              P[11][5] * SPP[6] - P[12][5] * SPP[7] - (P[10][5] * q0) / 2);
    nextP[1][9] =
        P[1][9] + P[0][9] * SF[6] + P[2][9] * SF[5] + P[3][9] * SF[9] +
        P[11][9] * SPP[6] - P[12][9] * SPP[7] - (P[10][9] * q0) / 2 +
        dt * (P[1][6] + P[0][6] * SF[6] + P[2][6] * SF[5] + P[3][6] * SF[9] +
              P[11][6] * SPP[6] - P[12][6] * SPP[7] - (P[10][6] * q0) / 2);
    nextP[1][10] = P[1][10] + P[0][10] * SF[6] + P[2][10] * SF[5] +
                   P[3][10] * SF[9] + P[11][10] * SPP[6] - P[12][10] * SPP[7] -
                   (P[10][10] * q0) / 2;
    nextP[1][11] = P[1][11] + P[0][11] * SF[6] + P[2][11] * SF[5] +
                   P[3][11] * SF[9] + P[11][11] * SPP[6] - P[12][11] * SPP[7] -
                   (P[10][11] * q0) / 2;
    nextP[1][12] = P[1][12] + P[0][12] * SF[6] + P[2][12] * SF[5] +
                   P[3][12] * SF[9] + P[11][12] * SPP[6] - P[12][12] * SPP[7] -
                   (P[10][12] * q0) / 2;
    nextP[1][13] = P[1][13] + P[0][13] * SF[6] + P[2][13] * SF[5] +
                   P[3][13] * SF[9] + P[11][13] * SPP[6] - P[12][13] * SPP[7] -
                   (P[10][13] * q0) / 2;
    nextP[1][14] = P[1][14] + P[0][14] * SF[6] + P[2][14] * SF[5] +
                   P[3][14] * SF[9] + P[11][14] * SPP[6] - P[12][14] * SPP[7] -
                   (P[10][14] * q0) / 2;
    nextP[1][15] = P[1][15] + P[0][15] * SF[6] + P[2][15] * SF[5] +
                   P[3][15] * SF[9] + P[11][15] * SPP[6] - P[12][15] * SPP[7] -
                   (P[10][15] * q0) / 2;
    nextP[1][16] = P[1][16] + P[0][16] * SF[6] + P[2][16] * SF[5] +
                   P[3][16] * SF[9] + P[11][16] * SPP[6] - P[12][16] * SPP[7] -
                   (P[10][16] * q0) / 2;
    nextP[1][17] = P[1][17] + P[0][17] * SF[6] + P[2][17] * SF[5] +
                   P[3][17] * SF[9] + P[11][17] * SPP[6] - P[12][17] * SPP[7] -
                   (P[10][17] * q0) / 2;
    nextP[1][18] = P[1][18] + P[0][18] * SF[6] + P[2][18] * SF[5] +
                   P[3][18] * SF[9] + P[11][18] * SPP[6] - P[12][18] * SPP[7] -
                   (P[10][18] * q0) / 2;
    nextP[1][19] = P[1][19] + P[0][19] * SF[6] + P[2][19] * SF[5] +
                   P[3][19] * SF[9] + P[11][19] * SPP[6] - P[12][19] * SPP[7] -
                   (P[10][19] * q0) / 2;
    nextP[1][20] = P[1][20] + P[0][20] * SF[6] + P[2][20] * SF[5] +
                   P[3][20] * SF[9] + P[11][20] * SPP[6] - P[12][20] * SPP[7] -
                   (P[10][20] * q0) / 2;
    nextP[1][21] = P[1][21] + P[0][21] * SF[6] + P[2][21] * SF[5] +
                   P[3][21] * SF[9] + P[11][21] * SPP[6] - P[12][21] * SPP[7] -
                   (P[10][21] * q0) / 2;
    nextP[2][0] =
        P[2][0] + SQ[7] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
        P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2 +
        SF[7] * (P[2][1] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
                 P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2) +
        SF[9] * (P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
                 P[12][2] * SF[11] - P[10][2] * SPP[6] - (P[11][2] * q0) / 2) +
        SF[8] * (P[2][3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
                 P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2) +
        SF[11] *
            (P[2][10] + P[0][10] * SF[4] + P[1][10] * SF[8] + P[3][10] * SF[6] +
             P[12][10] * SF[11] - P[10][10] * SPP[6] - (P[11][10] * q0) / 2) +
        SPP[7] *
            (P[2][11] + P[0][11] * SF[4] + P[1][11] * SF[8] + P[3][11] * SF[6] +
             P[12][11] * SF[11] - P[10][11] * SPP[6] - (P[11][11] * q0) / 2) +
        SPP[6] *
            (P[2][12] + P[0][12] * SF[4] + P[1][12] * SF[8] + P[3][12] * SF[6] +
             P[12][12] * SF[11] - P[10][12] * SPP[6] - (P[11][12] * q0) / 2);
    nextP[2][1] =
        P[2][1] + SQ[5] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
        P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2 +
        SF[6] * (P[2][0] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
                 P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2) +
        SF[5] * (P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
                 P[12][2] * SF[11] - P[10][2] * SPP[6] - (P[11][2] * q0) / 2) +
        SF[9] * (P[2][3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
                 P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2) +
        SPP[6] *
            (P[2][11] + P[0][11] * SF[4] + P[1][11] * SF[8] + P[3][11] * SF[6] +
             P[12][11] * SF[11] - P[10][11] * SPP[6] - (P[11][11] * q0) / 2) -
        SPP[7] *
            (P[2][12] + P[0][12] * SF[4] + P[1][12] * SF[8] + P[3][12] * SF[6] +
             P[12][12] * SF[11] - P[10][12] * SPP[6] - (P[11][12] * q0) / 2) -
        (q0 *
         (P[2][10] + P[0][10] * SF[4] + P[1][10] * SF[8] + P[3][10] * SF[6] +
          P[12][10] * SF[11] - P[10][10] * SPP[6] - (P[11][10] * q0) / 2)) /
            2;
    nextP[2][2] =
        P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
        P[12][2] * SF[11] - P[10][2] * SPP[6] + dayCov * SQ[9] +
        (dazCov * SQ[10]) / 4 - (P[11][2] * q0) / 2 +
        SF[4] * (P[2][0] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
                 P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2) +
        SF[8] * (P[2][1] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
                 P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2) +
        SF[6] * (P[2][3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
                 P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2) +
        SF[11] *
            (P[2][12] + P[0][12] * SF[4] + P[1][12] * SF[8] + P[3][12] * SF[6] +
             P[12][12] * SF[11] - P[10][12] * SPP[6] - (P[11][12] * q0) / 2) -
        SPP[6] *
            (P[2][10] + P[0][10] * SF[4] + P[1][10] * SF[8] + P[3][10] * SF[6] +
             P[12][10] * SF[11] - P[10][10] * SPP[6] - (P[11][10] * q0) / 2) +
        (daxCov * sq(q3)) / 4 -
        (q0 *
         (P[2][11] + P[0][11] * SF[4] + P[1][11] * SF[8] + P[3][11] * SF[6] +
          P[12][11] * SF[11] - P[10][11] * SPP[6] - (P[11][11] * q0) / 2)) /
            2;
    nextP[2][3] =
        P[2][3] + SQ[3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
        P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2 +
        SF[5] * (P[2][0] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
                 P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2) +
        SF[4] * (P[2][1] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
                 P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2) +
        SF[7] * (P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
                 P[12][2] * SF[11] - P[10][2] * SPP[6] - (P[11][2] * q0) / 2) -
        SF[11] *
            (P[2][11] + P[0][11] * SF[4] + P[1][11] * SF[8] + P[3][11] * SF[6] +
             P[12][11] * SF[11] - P[10][11] * SPP[6] - (P[11][11] * q0) / 2) +
        SPP[7] *
            (P[2][10] + P[0][10] * SF[4] + P[1][10] * SF[8] + P[3][10] * SF[6] +
             P[12][10] * SF[11] - P[10][10] * SPP[6] - (P[11][10] * q0) / 2) -
        (q0 *
         (P[2][12] + P[0][12] * SF[4] + P[1][12] * SF[8] + P[3][12] * SF[6] +
          P[12][12] * SF[11] - P[10][12] * SPP[6] - (P[11][12] * q0) / 2)) /
            2;
    nextP[2][4] =
        P[2][4] + P[0][4] * SF[4] + P[1][4] * SF[8] + P[3][4] * SF[6] +
        P[12][4] * SF[11] - P[10][4] * SPP[6] - (P[11][4] * q0) / 2 +
        SF[3] * (P[2][0] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
                 P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2) +
        SF[1] * (P[2][1] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
                 P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2) +
        SPP[0] *
            (P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
             P[12][2] * SF[11] - P[10][2] * SPP[6] - (P[11][2] * q0) / 2) -
        SPP[2] *
            (P[2][3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
             P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2) -
        SPP[4] *
            (P[2][13] + P[0][13] * SF[4] + P[1][13] * SF[8] + P[3][13] * SF[6] +
             P[12][13] * SF[11] - P[10][13] * SPP[6] - (P[11][13] * q0) / 2);
    nextP[2][5] =
        P[2][5] + P[0][5] * SF[4] + P[1][5] * SF[8] + P[3][5] * SF[6] +
        P[12][5] * SF[11] - P[10][5] * SPP[6] - (P[11][5] * q0) / 2 +
        SF[2] * (P[2][0] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
                 P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2) +
        SF[1] * (P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
                 P[12][2] * SF[11] - P[10][2] * SPP[6] - (P[11][2] * q0) / 2) +
        SF[3] * (P[2][3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
                 P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2) -
        SPP[0] *
            (P[2][1] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
             P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2) +
        SPP[3] *
            (P[2][13] + P[0][13] * SF[4] + P[1][13] * SF[8] + P[3][13] * SF[6] +
             P[12][13] * SF[11] - P[10][13] * SPP[6] - (P[11][13] * q0) / 2);
    nextP[2][6] =
        P[2][6] + P[0][6] * SF[4] + P[1][6] * SF[8] + P[3][6] * SF[6] +
        P[12][6] * SF[11] - P[10][6] * SPP[6] - (P[11][6] * q0) / 2 +
        SF[2] * (P[2][1] + P[0][1] * SF[4] + P[1][1] * SF[8] + P[3][1] * SF[6] +
                 P[12][1] * SF[11] - P[10][1] * SPP[6] - (P[11][1] * q0) / 2) +
        SF[1] * (P[2][3] + P[0][3] * SF[4] + P[1][3] * SF[8] + P[3][3] * SF[6] +
                 P[12][3] * SF[11] - P[10][3] * SPP[6] - (P[11][3] * q0) / 2) +
        SPP[0] *
            (P[2][0] + P[0][0] * SF[4] + P[1][0] * SF[8] + P[3][0] * SF[6] +
             P[12][0] * SF[11] - P[10][0] * SPP[6] - (P[11][0] * q0) / 2) -
        SPP[1] *
            (P[2][2] + P[0][2] * SF[4] + P[1][2] * SF[8] + P[3][2] * SF[6] +
             P[12][2] * SF[11] - P[10][2] * SPP[6] - (P[11][2] * q0) / 2) -
        (sq(q0) - sq(q1) - sq(q2) + sq(q3)) *
            (P[2][13] + P[0][13] * SF[4] + P[1][13] * SF[8] + P[3][13] * SF[6] +
             P[12][13] * SF[11] - P[10][13] * SPP[6] - (P[11][13] * q0) / 2);
    nextP[2][7] =
        P[2][7] + P[0][7] * SF[4] + P[1][7] * SF[8] + P[3][7] * SF[6] +
        P[12][7] * SF[11] - P[10][7] * SPP[6] - (P[11][7] * q0) / 2 +
        dt * (P[2][4] + P[0][4] * SF[4] + P[1][4] * SF[8] + P[3][4] * SF[6] +
              P[12][4] * SF[11] - P[10][4] * SPP[6] - (P[11][4] * q0) / 2);
    nextP[2][8] =
        P[2][8] + P[0][8] * SF[4] + P[1][8] * SF[8] + P[3][8] * SF[6] +
        P[12][8] * SF[11] - P[10][8] * SPP[6] - (P[11][8] * q0) / 2 +
        dt * (P[2][5] + P[0][5] * SF[4] + P[1][5] * SF[8] + P[3][5] * SF[6] +
              P[12][5] * SF[11] - P[10][5] * SPP[6] - (P[11][5] * q0) / 2);
    nextP[2][9] =
        P[2][9] + P[0][9] * SF[4] + P[1][9] * SF[8] + P[3][9] * SF[6] +
        P[12][9] * SF[11] - P[10][9] * SPP[6] - (P[11][9] * q0) / 2 +
        dt * (P[2][6] + P[0][6] * SF[4] + P[1][6] * SF[8] + P[3][6] * SF[6] +
              P[12][6] * SF[11] - P[10][6] * SPP[6] - (P[11][6] * q0) / 2);
    nextP[2][10] = P[2][10] + P[0][10] * SF[4] + P[1][10] * SF[8] +
                   P[3][10] * SF[6] + P[12][10] * SF[11] - P[10][10] * SPP[6] -
                   (P[11][10] * q0) / 2;
    nextP[2][11] = P[2][11] + P[0][11] * SF[4] + P[1][11] * SF[8] +
                   P[3][11] * SF[6] + P[12][11] * SF[11] - P[10][11] * SPP[6] -
                   (P[11][11] * q0) / 2;
    nextP[2][12] = P[2][12] + P[0][12] * SF[4] + P[1][12] * SF[8] +
                   P[3][12] * SF[6] + P[12][12] * SF[11] - P[10][12] * SPP[6] -
                   (P[11][12] * q0) / 2;
    nextP[2][13] = P[2][13] + P[0][13] * SF[4] + P[1][13] * SF[8] +
                   P[3][13] * SF[6] + P[12][13] * SF[11] - P[10][13] * SPP[6] -
                   (P[11][13] * q0) / 2;
    nextP[2][14] = P[2][14] + P[0][14] * SF[4] + P[1][14] * SF[8] +
                   P[3][14] * SF[6] + P[12][14] * SF[11] - P[10][14] * SPP[6] -
                   (P[11][14] * q0) / 2;
    nextP[2][15] = P[2][15] + P[0][15] * SF[4] + P[1][15] * SF[8] +
                   P[3][15] * SF[6] + P[12][15] * SF[11] - P[10][15] * SPP[6] -
                   (P[11][15] * q0) / 2;
    nextP[2][16] = P[2][16] + P[0][16] * SF[4] + P[1][16] * SF[8] +
                   P[3][16] * SF[6] + P[12][16] * SF[11] - P[10][16] * SPP[6] -
                   (P[11][16] * q0) / 2;
    nextP[2][17] = P[2][17] + P[0][17] * SF[4] + P[1][17] * SF[8] +
                   P[3][17] * SF[6] + P[12][17] * SF[11] - P[10][17] * SPP[6] -
                   (P[11][17] * q0) / 2;
    nextP[2][18] = P[2][18] + P[0][18] * SF[4] + P[1][18] * SF[8] +
                   P[3][18] * SF[6] + P[12][18] * SF[11] - P[10][18] * SPP[6] -
                   (P[11][18] * q0) / 2;
    nextP[2][19] = P[2][19] + P[0][19] * SF[4] + P[1][19] * SF[8] +
                   P[3][19] * SF[6] + P[12][19] * SF[11] - P[10][19] * SPP[6] -
                   (P[11][19] * q0) / 2;
    nextP[2][20] = P[2][20] + P[0][20] * SF[4] + P[1][20] * SF[8] +
                   P[3][20] * SF[6] + P[12][20] * SF[11] - P[10][20] * SPP[6] -
                   (P[11][20] * q0) / 2;
    nextP[2][21] = P[2][21] + P[0][21] * SF[4] + P[1][21] * SF[8] +
                   P[3][21] * SF[6] + P[12][21] * SF[11] - P[10][21] * SPP[6] -
                   (P[11][21] * q0) / 2;
    nextP[3][0] =
        P[3][0] + SQ[6] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
        P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2 +
        SF[7] * (P[3][1] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
                 P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2) +
        SF[9] * (P[3][2] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
                 P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2) +
        SF[8] * (P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
                 P[11][3] * SF[11] + P[10][3] * SPP[7] - (P[12][3] * q0) / 2) +
        SF[11] *
            (P[3][10] + P[0][10] * SF[5] + P[1][10] * SF[4] + P[2][10] * SF[7] -
             P[11][10] * SF[11] + P[10][10] * SPP[7] - (P[12][10] * q0) / 2) +
        SPP[7] *
            (P[3][11] + P[0][11] * SF[5] + P[1][11] * SF[4] + P[2][11] * SF[7] -
             P[11][11] * SF[11] + P[10][11] * SPP[7] - (P[12][11] * q0) / 2) +
        SPP[6] *
            (P[3][12] + P[0][12] * SF[5] + P[1][12] * SF[4] + P[2][12] * SF[7] -
             P[11][12] * SF[11] + P[10][12] * SPP[7] - (P[12][12] * q0) / 2);
    nextP[3][1] =
        P[3][1] + SQ[4] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
        P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2 +
        SF[6] * (P[3][0] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
                 P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2) +
        SF[5] * (P[3][2] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
                 P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2) +
        SF[9] * (P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
                 P[11][3] * SF[11] + P[10][3] * SPP[7] - (P[12][3] * q0) / 2) +
        SPP[6] *
            (P[3][11] + P[0][11] * SF[5] + P[1][11] * SF[4] + P[2][11] * SF[7] -
             P[11][11] * SF[11] + P[10][11] * SPP[7] - (P[12][11] * q0) / 2) -
        SPP[7] *
            (P[3][12] + P[0][12] * SF[5] + P[1][12] * SF[4] + P[2][12] * SF[7] -
             P[11][12] * SF[11] + P[10][12] * SPP[7] - (P[12][12] * q0) / 2) -
        (q0 *
         (P[3][10] + P[0][10] * SF[5] + P[1][10] * SF[4] + P[2][10] * SF[7] -
          P[11][10] * SF[11] + P[10][10] * SPP[7] - (P[12][10] * q0) / 2)) /
            2;
    nextP[3][2] =
        P[3][2] + SQ[3] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
        P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2 +
        SF[4] * (P[3][0] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
                 P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2) +
        SF[8] * (P[3][1] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
                 P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2) +
        SF[6] * (P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
                 P[11][3] * SF[11] + P[10][3] * SPP[7] - (P[12][3] * q0) / 2) +
        SF[11] *
            (P[3][12] + P[0][12] * SF[5] + P[1][12] * SF[4] + P[2][12] * SF[7] -
             P[11][12] * SF[11] + P[10][12] * SPP[7] - (P[12][12] * q0) / 2) -
        SPP[6] *
            (P[3][10] + P[0][10] * SF[5] + P[1][10] * SF[4] + P[2][10] * SF[7] -
             P[11][10] * SF[11] + P[10][10] * SPP[7] - (P[12][10] * q0) / 2) -
        (q0 *
         (P[3][11] + P[0][11] * SF[5] + P[1][11] * SF[4] + P[2][11] * SF[7] -
          P[11][11] * SF[11] + P[10][11] * SPP[7] - (P[12][11] * q0) / 2)) /
            2;
    nextP[3][3] =
        P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
        P[11][3] * SF[11] + P[10][3] * SPP[7] + (dayCov * SQ[10]) / 4 +
        dazCov * SQ[9] - (P[12][3] * q0) / 2 +
        SF[5] * (P[3][0] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
                 P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2) +
        SF[4] * (P[3][1] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
                 P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2) +
        SF[7] * (P[3][2] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
                 P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2) -
        SF[11] *
            (P[3][11] + P[0][11] * SF[5] + P[1][11] * SF[4] + P[2][11] * SF[7] -
             P[11][11] * SF[11] + P[10][11] * SPP[7] - (P[12][11] * q0) / 2) +
        SPP[7] *
            (P[3][10] + P[0][10] * SF[5] + P[1][10] * SF[4] + P[2][10] * SF[7] -
             P[11][10] * SF[11] + P[10][10] * SPP[7] - (P[12][10] * q0) / 2) +
        (daxCov * sq(q2)) / 4 -
        (q0 *
         (P[3][12] + P[0][12] * SF[5] + P[1][12] * SF[4] + P[2][12] * SF[7] -
          P[11][12] * SF[11] + P[10][12] * SPP[7] - (P[12][12] * q0) / 2)) /
            2;
    nextP[3][4] =
        P[3][4] + P[0][4] * SF[5] + P[1][4] * SF[4] + P[2][4] * SF[7] -
        P[11][4] * SF[11] + P[10][4] * SPP[7] - (P[12][4] * q0) / 2 +
        SF[3] * (P[3][0] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
                 P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2) +
        SF[1] * (P[3][1] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
                 P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2) +
        SPP[0] *
            (P[3][2] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
             P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2) -
        SPP[2] *
            (P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
             P[11][3] * SF[11] + P[10][3] * SPP[7] - (P[12][3] * q0) / 2) -
        SPP[4] *
            (P[3][13] + P[0][13] * SF[5] + P[1][13] * SF[4] + P[2][13] * SF[7] -
             P[11][13] * SF[11] + P[10][13] * SPP[7] - (P[12][13] * q0) / 2);
    nextP[3][5] =
        P[3][5] + P[0][5] * SF[5] + P[1][5] * SF[4] + P[2][5] * SF[7] -
        P[11][5] * SF[11] + P[10][5] * SPP[7] - (P[12][5] * q0) / 2 +
        SF[2] * (P[3][0] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
                 P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2) +
        SF[1] * (P[3][2] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
                 P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2) +
        SF[3] * (P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
                 P[11][3] * SF[11] + P[10][3] * SPP[7] - (P[12][3] * q0) / 2) -
        SPP[0] *
            (P[3][1] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
             P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2) +
        SPP[3] *
            (P[3][13] + P[0][13] * SF[5] + P[1][13] * SF[4] + P[2][13] * SF[7] -
             P[11][13] * SF[11] + P[10][13] * SPP[7] - (P[12][13] * q0) / 2);
    nextP[3][6] =
        P[3][6] + P[0][6] * SF[5] + P[1][6] * SF[4] + P[2][6] * SF[7] -
        P[11][6] * SF[11] + P[10][6] * SPP[7] - (P[12][6] * q0) / 2 +
        SF[2] * (P[3][1] + P[0][1] * SF[5] + P[1][1] * SF[4] + P[2][1] * SF[7] -
                 P[11][1] * SF[11] + P[10][1] * SPP[7] - (P[12][1] * q0) / 2) +
        SF[1] * (P[3][3] + P[0][3] * SF[5] + P[1][3] * SF[4] + P[2][3] * SF[7] -
                 P[11][3] * SF[11] + P[10][3] * SPP[7] - (P[12][3] * q0) / 2) +
        SPP[0] *
            (P[3][0] + P[0][0] * SF[5] + P[1][0] * SF[4] + P[2][0] * SF[7] -
             P[11][0] * SF[11] + P[10][0] * SPP[7] - (P[12][0] * q0) / 2) -
        SPP[1] *
            (P[3][2] + P[0][2] * SF[5] + P[1][2] * SF[4] + P[2][2] * SF[7] -
             P[11][2] * SF[11] + P[10][2] * SPP[7] - (P[12][2] * q0) / 2) -
        (sq(q0) - sq(q1) - sq(q2) + sq(q3)) *
            (P[3][13] + P[0][13] * SF[5] + P[1][13] * SF[4] + P[2][13] * SF[7] -
             P[11][13] * SF[11] + P[10][13] * SPP[7] - (P[12][13] * q0) / 2);
    nextP[3][7] =
        P[3][7] + P[0][7] * SF[5] + P[1][7] * SF[4] + P[2][7] * SF[7] -
        P[11][7] * SF[11] + P[10][7] * SPP[7] - (P[12][7] * q0) / 2 +
        dt * (P[3][4] + P[0][4] * SF[5] + P[1][4] * SF[4] + P[2][4] * SF[7] -
              P[11][4] * SF[11] + P[10][4] * SPP[7] - (P[12][4] * q0) / 2);
    nextP[3][8] =
        P[3][8] + P[0][8] * SF[5] + P[1][8] * SF[4] + P[2][8] * SF[7] -
        P[11][8] * SF[11] + P[10][8] * SPP[7] - (P[12][8] * q0) / 2 +
        dt * (P[3][5] + P[0][5] * SF[5] + P[1][5] * SF[4] + P[2][5] * SF[7] -
              P[11][5] * SF[11] + P[10][5] * SPP[7] - (P[12][5] * q0) / 2);
    nextP[3][9] =
        P[3][9] + P[0][9] * SF[5] + P[1][9] * SF[4] + P[2][9] * SF[7] -
        P[11][9] * SF[11] + P[10][9] * SPP[7] - (P[12][9] * q0) / 2 +
        dt * (P[3][6] + P[0][6] * SF[5] + P[1][6] * SF[4] + P[2][6] * SF[7] -
              P[11][6] * SF[11] + P[10][6] * SPP[7] - (P[12][6] * q0) / 2);
    nextP[3][10] = P[3][10] + P[0][10] * SF[5] + P[1][10] * SF[4] +
                   P[2][10] * SF[7] - P[11][10] * SF[11] + P[10][10] * SPP[7] -
                   (P[12][10] * q0) / 2;
    nextP[3][11] = P[3][11] + P[0][11] * SF[5] + P[1][11] * SF[4] +
                   P[2][11] * SF[7] - P[11][11] * SF[11] + P[10][11] * SPP[7] -
                   (P[12][11] * q0) / 2;
    nextP[3][12] = P[3][12] + P[0][12] * SF[5] + P[1][12] * SF[4] +
                   P[2][12] * SF[7] - P[11][12] * SF[11] + P[10][12] * SPP[7] -
                   (P[12][12] * q0) / 2;
    nextP[3][13] = P[3][13] + P[0][13] * SF[5] + P[1][13] * SF[4] +
                   P[2][13] * SF[7] - P[11][13] * SF[11] + P[10][13] * SPP[7] -
                   (P[12][13] * q0) / 2;
    nextP[3][14] = P[3][14] + P[0][14] * SF[5] + P[1][14] * SF[4] +
                   P[2][14] * SF[7] - P[11][14] * SF[11] + P[10][14] * SPP[7] -
                   (P[12][14] * q0) / 2;
    nextP[3][15] = P[3][15] + P[0][15] * SF[5] + P[1][15] * SF[4] +
                   P[2][15] * SF[7] - P[11][15] * SF[11] + P[10][15] * SPP[7] -
                   (P[12][15] * q0) / 2;
    nextP[3][16] = P[3][16] + P[0][16] * SF[5] + P[1][16] * SF[4] +
                   P[2][16] * SF[7] - P[11][16] * SF[11] + P[10][16] * SPP[7] -
                   (P[12][16] * q0) / 2;
    nextP[3][17] = P[3][17] + P[0][17] * SF[5] + P[1][17] * SF[4] +
                   P[2][17] * SF[7] - P[11][17] * SF[11] + P[10][17] * SPP[7] -
                   (P[12][17] * q0) / 2;
    nextP[3][18] = P[3][18] + P[0][18] * SF[5] + P[1][18] * SF[4] +
                   P[2][18] * SF[7] - P[11][18] * SF[11] + P[10][18] * SPP[7] -
                   (P[12][18] * q0) / 2;
    nextP[3][19] = P[3][19] + P[0][19] * SF[5] + P[1][19] * SF[4] +
                   P[2][19] * SF[7] - P[11][19] * SF[11] + P[10][19] * SPP[7] -
                   (P[12][19] * q0) / 2;
    nextP[3][20] = P[3][20] + P[0][20] * SF[5] + P[1][20] * SF[4] +
                   P[2][20] * SF[7] - P[11][20] * SF[11] + P[10][20] * SPP[7] -
                   (P[12][20] * q0) / 2;
    nextP[3][21] = P[3][21] + P[0][21] * SF[5] + P[1][21] * SF[4] +
                   P[2][21] * SF[7] - P[11][21] * SF[11] + P[10][21] * SPP[7] -
                   (P[12][21] * q0) / 2;
    nextP[4][0] =
        P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] + P[2][0] * SPP[0] -
        P[3][0] * SPP[2] - P[13][0] * SPP[4] +
        SF[7] * (P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] +
                 P[2][1] * SPP[0] - P[3][1] * SPP[2] - P[13][1] * SPP[4]) +
        SF[9] * (P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] +
                 P[2][2] * SPP[0] - P[3][2] * SPP[2] - P[13][2] * SPP[4]) +
        SF[8] * (P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] +
                 P[2][3] * SPP[0] - P[3][3] * SPP[2] - P[13][3] * SPP[4]) +
        SF[11] * (P[4][10] + P[0][10] * SF[3] + P[1][10] * SF[1] +
                  P[2][10] * SPP[0] - P[3][10] * SPP[2] - P[13][10] * SPP[4]) +
        SPP[7] * (P[4][11] + P[0][11] * SF[3] + P[1][11] * SF[1] +
                  P[2][11] * SPP[0] - P[3][11] * SPP[2] - P[13][11] * SPP[4]) +
        SPP[6] * (P[4][12] + P[0][12] * SF[3] + P[1][12] * SF[1] +
                  P[2][12] * SPP[0] - P[3][12] * SPP[2] - P[13][12] * SPP[4]);
    nextP[4][1] =
        P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] + P[2][1] * SPP[0] -
        P[3][1] * SPP[2] - P[13][1] * SPP[4] +
        SF[6] * (P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] +
                 P[2][0] * SPP[0] - P[3][0] * SPP[2] - P[13][0] * SPP[4]) +
        SF[5] * (P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] +
                 P[2][2] * SPP[0] - P[3][2] * SPP[2] - P[13][2] * SPP[4]) +
        SF[9] * (P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] +
                 P[2][3] * SPP[0] - P[3][3] * SPP[2] - P[13][3] * SPP[4]) +
        SPP[6] * (P[4][11] + P[0][11] * SF[3] + P[1][11] * SF[1] +
                  P[2][11] * SPP[0] - P[3][11] * SPP[2] - P[13][11] * SPP[4]) -
        SPP[7] * (P[4][12] + P[0][12] * SF[3] + P[1][12] * SF[1] +
                  P[2][12] * SPP[0] - P[3][12] * SPP[2] - P[13][12] * SPP[4]) -
        (q0 * (P[4][10] + P[0][10] * SF[3] + P[1][10] * SF[1] +
               P[2][10] * SPP[0] - P[3][10] * SPP[2] - P[13][10] * SPP[4])) /
            2;
    nextP[4][2] =
        P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] + P[2][2] * SPP[0] -
        P[3][2] * SPP[2] - P[13][2] * SPP[4] +
        SF[4] * (P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] +
                 P[2][0] * SPP[0] - P[3][0] * SPP[2] - P[13][0] * SPP[4]) +
        SF[8] * (P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] +
                 P[2][1] * SPP[0] - P[3][1] * SPP[2] - P[13][1] * SPP[4]) +
        SF[6] * (P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] +
                 P[2][3] * SPP[0] - P[3][3] * SPP[2] - P[13][3] * SPP[4]) +
        SF[11] * (P[4][12] + P[0][12] * SF[3] + P[1][12] * SF[1] +
                  P[2][12] * SPP[0] - P[3][12] * SPP[2] - P[13][12] * SPP[4]) -
        SPP[6] * (P[4][10] + P[0][10] * SF[3] + P[1][10] * SF[1] +
                  P[2][10] * SPP[0] - P[3][10] * SPP[2] - P[13][10] * SPP[4]) -
        (q0 * (P[4][11] + P[0][11] * SF[3] + P[1][11] * SF[1] +
               P[2][11] * SPP[0] - P[3][11] * SPP[2] - P[13][11] * SPP[4])) /
            2;
    nextP[4][3] =
        P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] + P[2][3] * SPP[0] -
        P[3][3] * SPP[2] - P[13][3] * SPP[4] +
        SF[5] * (P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] +
                 P[2][0] * SPP[0] - P[3][0] * SPP[2] - P[13][0] * SPP[4]) +
        SF[4] * (P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] +
                 P[2][1] * SPP[0] - P[3][1] * SPP[2] - P[13][1] * SPP[4]) +
        SF[7] * (P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] +
                 P[2][2] * SPP[0] - P[3][2] * SPP[2] - P[13][2] * SPP[4]) -
        SF[11] * (P[4][11] + P[0][11] * SF[3] + P[1][11] * SF[1] +
                  P[2][11] * SPP[0] - P[3][11] * SPP[2] - P[13][11] * SPP[4]) +
        SPP[7] * (P[4][10] + P[0][10] * SF[3] + P[1][10] * SF[1] +
                  P[2][10] * SPP[0] - P[3][10] * SPP[2] - P[13][10] * SPP[4]) -
        (q0 * (P[4][12] + P[0][12] * SF[3] + P[1][12] * SF[1] +
               P[2][12] * SPP[0] - P[3][12] * SPP[2] - P[13][12] * SPP[4])) /
            2;
    nextP[4][4] =
        P[4][4] + P[0][4] * SF[3] + P[1][4] * SF[1] + P[2][4] * SPP[0] -
        P[3][4] * SPP[2] - P[13][4] * SPP[4] +
        dvyCov * sq(SG[7] - 2 * q0 * q3) + dvzCov * sq(SG[6] + 2 * q0 * q2) +
        SF[3] * (P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] +
                 P[2][0] * SPP[0] - P[3][0] * SPP[2] - P[13][0] * SPP[4]) +
        SF[1] * (P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] +
                 P[2][1] * SPP[0] - P[3][1] * SPP[2] - P[13][1] * SPP[4]) +
        SPP[0] * (P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] +
                  P[2][2] * SPP[0] - P[3][2] * SPP[2] - P[13][2] * SPP[4]) -
        SPP[2] * (P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] +
                  P[2][3] * SPP[0] - P[3][3] * SPP[2] - P[13][3] * SPP[4]) -
        SPP[4] * (P[4][13] + P[0][13] * SF[3] + P[1][13] * SF[1] +
                  P[2][13] * SPP[0] - P[3][13] * SPP[2] - P[13][13] * SPP[4]) +
        dvxCov * sq(SG[1] + SG[2] - SG[3] - SG[4]);
    nextP[4][5] =
        P[4][5] + SQ[2] + P[0][5] * SF[3] + P[1][5] * SF[1] + P[2][5] * SPP[0] -
        P[3][5] * SPP[2] - P[13][5] * SPP[4] +
        SF[2] * (P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] +
                 P[2][0] * SPP[0] - P[3][0] * SPP[2] - P[13][0] * SPP[4]) +
        SF[1] * (P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] +
                 P[2][2] * SPP[0] - P[3][2] * SPP[2] - P[13][2] * SPP[4]) +
        SF[3] * (P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] +
                 P[2][3] * SPP[0] - P[3][3] * SPP[2] - P[13][3] * SPP[4]) -
        SPP[0] * (P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] +
                  P[2][1] * SPP[0] - P[3][1] * SPP[2] - P[13][1] * SPP[4]) +
        SPP[3] * (P[4][13] + P[0][13] * SF[3] + P[1][13] * SF[1] +
                  P[2][13] * SPP[0] - P[3][13] * SPP[2] - P[13][13] * SPP[4]);
    nextP[4][6] =
        P[4][6] + SQ[1] + P[0][6] * SF[3] + P[1][6] * SF[1] + P[2][6] * SPP[0] -
        P[3][6] * SPP[2] - P[13][6] * SPP[4] +
        SF[2] * (P[4][1] + P[0][1] * SF[3] + P[1][1] * SF[1] +
                 P[2][1] * SPP[0] - P[3][1] * SPP[2] - P[13][1] * SPP[4]) +
        SF[1] * (P[4][3] + P[0][3] * SF[3] + P[1][3] * SF[1] +
                 P[2][3] * SPP[0] - P[3][3] * SPP[2] - P[13][3] * SPP[4]) +
        SPP[0] * (P[4][0] + P[0][0] * SF[3] + P[1][0] * SF[1] +
                  P[2][0] * SPP[0] - P[3][0] * SPP[2] - P[13][0] * SPP[4]) -
        SPP[1] * (P[4][2] + P[0][2] * SF[3] + P[1][2] * SF[1] +
                  P[2][2] * SPP[0] - P[3][2] * SPP[2] - P[13][2] * SPP[4]) -
        (sq(q0) - sq(q1) - sq(q2) + sq(q3)) *
            (P[4][13] + P[0][13] * SF[3] + P[1][13] * SF[1] +
             P[2][13] * SPP[0] - P[3][13] * SPP[2] - P[13][13] * SPP[4]);
    nextP[4][7] =
        P[4][7] + P[0][7] * SF[3] + P[1][7] * SF[1] + P[2][7] * SPP[0] -
        P[3][7] * SPP[2] - P[13][7] * SPP[4] +
        dt * (P[4][4] + P[0][4] * SF[3] + P[1][4] * SF[1] + P[2][4] * SPP[0] -
              P[3][4] * SPP[2] - P[13][4] * SPP[4]);
    nextP[4][8] =
        P[4][8] + P[0][8] * SF[3] + P[1][8] * SF[1] + P[2][8] * SPP[0] -
        P[3][8] * SPP[2] - P[13][8] * SPP[4] +
        dt * (P[4][5] + P[0][5] * SF[3] + P[1][5] * SF[1] + P[2][5] * SPP[0] -
              P[3][5] * SPP[2] - P[13][5] * SPP[4]);
    nextP[4][9] =
        P[4][9] + P[0][9] * SF[3] + P[1][9] * SF[1] + P[2][9] * SPP[0] -
        P[3][9] * SPP[2] - P[13][9] * SPP[4] +
        dt * (P[4][6] + P[0][6] * SF[3] + P[1][6] * SF[1] + P[2][6] * SPP[0] -
              P[3][6] * SPP[2] - P[13][6] * SPP[4]);
    nextP[4][10] = P[4][10] + P[0][10] * SF[3] + P[1][10] * SF[1] +
                   P[2][10] * SPP[0] - P[3][10] * SPP[2] - P[13][10] * SPP[4];
    nextP[4][11] = P[4][11] + P[0][11] * SF[3] + P[1][11] * SF[1] +
                   P[2][11] * SPP[0] - P[3][11] * SPP[2] - P[13][11] * SPP[4];
    nextP[4][12] = P[4][12] + P[0][12] * SF[3] + P[1][12] * SF[1] +
                   P[2][12] * SPP[0] - P[3][12] * SPP[2] - P[13][12] * SPP[4];
    nextP[4][13] = P[4][13] + P[0][13] * SF[3] + P[1][13] * SF[1] +
                   P[2][13] * SPP[0] - P[3][13] * SPP[2] - P[13][13] * SPP[4];
    nextP[4][14] = P[4][14] + P[0][14] * SF[3] + P[1][14] * SF[1] +
                   P[2][14] * SPP[0] - P[3][14] * SPP[2] - P[13][14] * SPP[4];
    nextP[4][15] = P[4][15] + P[0][15] * SF[3] + P[1][15] * SF[1] +
                   P[2][15] * SPP[0] - P[3][15] * SPP[2] - P[13][15] * SPP[4];
    nextP[4][16] = P[4][16] + P[0][16] * SF[3] + P[1][16] * SF[1] +
                   P[2][16] * SPP[0] - P[3][16] * SPP[2] - P[13][16] * SPP[4];
    nextP[4][17] = P[4][17] + P[0][17] * SF[3] + P[1][17] * SF[1] +
                   P[2][17] * SPP[0] - P[3][17] * SPP[2] - P[13][17] * SPP[4];
    nextP[4][18] = P[4][18] + P[0][18] * SF[3] + P[1][18] * SF[1] +
                   P[2][18] * SPP[0] - P[3][18] * SPP[2] - P[13][18] * SPP[4];
    nextP[4][19] = P[4][19] + P[0][19] * SF[3] + P[1][19] * SF[1] +
                   P[2][19] * SPP[0] - P[3][19] * SPP[2] - P[13][19] * SPP[4];
    nextP[4][20] = P[4][20] + P[0][20] * SF[3] + P[1][20] * SF[1] +
                   P[2][20] * SPP[0] - P[3][20] * SPP[2] - P[13][20] * SPP[4];
    nextP[4][21] = P[4][21] + P[0][21] * SF[3] + P[1][21] * SF[1] +
                   P[2][21] * SPP[0] - P[3][21] * SPP[2] - P[13][21] * SPP[4];
    nextP[5][0] =
        P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] + P[3][0] * SF[3] -
        P[1][0] * SPP[0] + P[13][0] * SPP[3] +
        SF[7] * (P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] + P[3][1] * SF[3] -
                 P[1][1] * SPP[0] + P[13][1] * SPP[3]) +
        SF[9] * (P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] + P[3][2] * SF[3] -
                 P[1][2] * SPP[0] + P[13][2] * SPP[3]) +
        SF[8] * (P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] + P[3][3] * SF[3] -
                 P[1][3] * SPP[0] + P[13][3] * SPP[3]) +
        SF[11] * (P[5][10] + P[0][10] * SF[2] + P[2][10] * SF[1] +
                  P[3][10] * SF[3] - P[1][10] * SPP[0] + P[13][10] * SPP[3]) +
        SPP[7] * (P[5][11] + P[0][11] * SF[2] + P[2][11] * SF[1] +
                  P[3][11] * SF[3] - P[1][11] * SPP[0] + P[13][11] * SPP[3]) +
        SPP[6] * (P[5][12] + P[0][12] * SF[2] + P[2][12] * SF[1] +
                  P[3][12] * SF[3] - P[1][12] * SPP[0] + P[13][12] * SPP[3]);
    nextP[5][1] =
        P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] + P[3][1] * SF[3] -
        P[1][1] * SPP[0] + P[13][1] * SPP[3] +
        SF[6] * (P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] + P[3][0] * SF[3] -
                 P[1][0] * SPP[0] + P[13][0] * SPP[3]) +
        SF[5] * (P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] + P[3][2] * SF[3] -
                 P[1][2] * SPP[0] + P[13][2] * SPP[3]) +
        SF[9] * (P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] + P[3][3] * SF[3] -
                 P[1][3] * SPP[0] + P[13][3] * SPP[3]) +
        SPP[6] * (P[5][11] + P[0][11] * SF[2] + P[2][11] * SF[1] +
                  P[3][11] * SF[3] - P[1][11] * SPP[0] + P[13][11] * SPP[3]) -
        SPP[7] * (P[5][12] + P[0][12] * SF[2] + P[2][12] * SF[1] +
                  P[3][12] * SF[3] - P[1][12] * SPP[0] + P[13][12] * SPP[3]) -
        (q0 * (P[5][10] + P[0][10] * SF[2] + P[2][10] * SF[1] +
               P[3][10] * SF[3] - P[1][10] * SPP[0] + P[13][10] * SPP[3])) /
            2;
    nextP[5][2] =
        P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] + P[3][2] * SF[3] -
        P[1][2] * SPP[0] + P[13][2] * SPP[3] +
        SF[4] * (P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] + P[3][0] * SF[3] -
                 P[1][0] * SPP[0] + P[13][0] * SPP[3]) +
        SF[8] * (P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] + P[3][1] * SF[3] -
                 P[1][1] * SPP[0] + P[13][1] * SPP[3]) +
        SF[6] * (P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] + P[3][3] * SF[3] -
                 P[1][3] * SPP[0] + P[13][3] * SPP[3]) +
        SF[11] * (P[5][12] + P[0][12] * SF[2] + P[2][12] * SF[1] +
                  P[3][12] * SF[3] - P[1][12] * SPP[0] + P[13][12] * SPP[3]) -
        SPP[6] * (P[5][10] + P[0][10] * SF[2] + P[2][10] * SF[1] +
                  P[3][10] * SF[3] - P[1][10] * SPP[0] + P[13][10] * SPP[3]) -
        (q0 * (P[5][11] + P[0][11] * SF[2] + P[2][11] * SF[1] +
               P[3][11] * SF[3] - P[1][11] * SPP[0] + P[13][11] * SPP[3])) /
            2;
    nextP[5][3] =
        P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] + P[3][3] * SF[3] -
        P[1][3] * SPP[0] + P[13][3] * SPP[3] +
        SF[5] * (P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] + P[3][0] * SF[3] -
                 P[1][0] * SPP[0] + P[13][0] * SPP[3]) +
        SF[4] * (P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] + P[3][1] * SF[3] -
                 P[1][1] * SPP[0] + P[13][1] * SPP[3]) +
        SF[7] * (P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] + P[3][2] * SF[3] -
                 P[1][2] * SPP[0] + P[13][2] * SPP[3]) -
        SF[11] * (P[5][11] + P[0][11] * SF[2] + P[2][11] * SF[1] +
                  P[3][11] * SF[3] - P[1][11] * SPP[0] + P[13][11] * SPP[3]) +
        SPP[7] * (P[5][10] + P[0][10] * SF[2] + P[2][10] * SF[1] +
                  P[3][10] * SF[3] - P[1][10] * SPP[0] + P[13][10] * SPP[3]) -
        (q0 * (P[5][12] + P[0][12] * SF[2] + P[2][12] * SF[1] +
               P[3][12] * SF[3] - P[1][12] * SPP[0] + P[13][12] * SPP[3])) /
            2;
    nextP[5][4] =
        P[5][4] + SQ[2] + P[0][4] * SF[2] + P[2][4] * SF[1] + P[3][4] * SF[3] -
        P[1][4] * SPP[0] + P[13][4] * SPP[3] +
        SF[3] * (P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] + P[3][0] * SF[3] -
                 P[1][0] * SPP[0] + P[13][0] * SPP[3]) +
        SF[1] * (P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] + P[3][1] * SF[3] -
                 P[1][1] * SPP[0] + P[13][1] * SPP[3]) +
        SPP[0] * (P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] +
                  P[3][2] * SF[3] - P[1][2] * SPP[0] + P[13][2] * SPP[3]) -
        SPP[2] * (P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] +
                  P[3][3] * SF[3] - P[1][3] * SPP[0] + P[13][3] * SPP[3]) -
        SPP[4] * (P[5][13] + P[0][13] * SF[2] + P[2][13] * SF[1] +
                  P[3][13] * SF[3] - P[1][13] * SPP[0] + P[13][13] * SPP[3]);
    nextP[5][5] =
        P[5][5] + P[0][5] * SF[2] + P[2][5] * SF[1] + P[3][5] * SF[3] -
        P[1][5] * SPP[0] + P[13][5] * SPP[3] +
        dvxCov * sq(SG[7] + 2 * q0 * q3) + dvzCov * sq(SG[5] - 2 * q0 * q1) +
        SF[2] * (P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] + P[3][0] * SF[3] -
                 P[1][0] * SPP[0] + P[13][0] * SPP[3]) +
        SF[1] * (P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] + P[3][2] * SF[3] -
                 P[1][2] * SPP[0] + P[13][2] * SPP[3]) +
        SF[3] * (P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] + P[3][3] * SF[3] -
                 P[1][3] * SPP[0] + P[13][3] * SPP[3]) -
        SPP[0] * (P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] +
                  P[3][1] * SF[3] - P[1][1] * SPP[0] + P[13][1] * SPP[3]) +
        SPP[3] * (P[5][13] + P[0][13] * SF[2] + P[2][13] * SF[1] +
                  P[3][13] * SF[3] - P[1][13] * SPP[0] + P[13][13] * SPP[3]) +
        dvyCov * sq(SG[1] - SG[2] + SG[3] - SG[4]);
    nextP[5][6] =
        P[5][6] + SQ[0] + P[0][6] * SF[2] + P[2][6] * SF[1] + P[3][6] * SF[3] -
        P[1][6] * SPP[0] + P[13][6] * SPP[3] +
        SF[2] * (P[5][1] + P[0][1] * SF[2] + P[2][1] * SF[1] + P[3][1] * SF[3] -
                 P[1][1] * SPP[0] + P[13][1] * SPP[3]) +
        SF[1] * (P[5][3] + P[0][3] * SF[2] + P[2][3] * SF[1] + P[3][3] * SF[3] -
                 P[1][3] * SPP[0] + P[13][3] * SPP[3]) +
        SPP[0] * (P[5][0] + P[0][0] * SF[2] + P[2][0] * SF[1] +
                  P[3][0] * SF[3] - P[1][0] * SPP[0] + P[13][0] * SPP[3]) -
        SPP[1] * (P[5][2] + P[0][2] * SF[2] + P[2][2] * SF[1] +
                  P[3][2] * SF[3] - P[1][2] * SPP[0] + P[13][2] * SPP[3]) -
        (sq(q0) - sq(q1) - sq(q2) + sq(q3)) *
            (P[5][13] + P[0][13] * SF[2] + P[2][13] * SF[1] + P[3][13] * SF[3] -
             P[1][13] * SPP[0] + P[13][13] * SPP[3]);
    nextP[5][7] = P[5][7] + P[0][7] * SF[2] + P[2][7] * SF[1] +
                  P[3][7] * SF[3] - P[1][7] * SPP[0] + P[13][7] * SPP[3] +
                  dt * (P[5][4] + P[0][4] * SF[2] + P[2][4] * SF[1] +
                        P[3][4] * SF[3] - P[1][4] * SPP[0] + P[13][4] * SPP[3]);
    nextP[5][8] = P[5][8] + P[0][8] * SF[2] + P[2][8] * SF[1] +
                  P[3][8] * SF[3] - P[1][8] * SPP[0] + P[13][8] * SPP[3] +
                  dt * (P[5][5] + P[0][5] * SF[2] + P[2][5] * SF[1] +
                        P[3][5] * SF[3] - P[1][5] * SPP[0] + P[13][5] * SPP[3]);
    nextP[5][9] = P[5][9] + P[0][9] * SF[2] + P[2][9] * SF[1] +
                  P[3][9] * SF[3] - P[1][9] * SPP[0] + P[13][9] * SPP[3] +
                  dt * (P[5][6] + P[0][6] * SF[2] + P[2][6] * SF[1] +
                        P[3][6] * SF[3] - P[1][6] * SPP[0] + P[13][6] * SPP[3]);
    nextP[5][10] = P[5][10] + P[0][10] * SF[2] + P[2][10] * SF[1] +
                   P[3][10] * SF[3] - P[1][10] * SPP[0] + P[13][10] * SPP[3];
    nextP[5][11] = P[5][11] + P[0][11] * SF[2] + P[2][11] * SF[1] +
                   P[3][11] * SF[3] - P[1][11] * SPP[0] + P[13][11] * SPP[3];
    nextP[5][12] = P[5][12] + P[0][12] * SF[2] + P[2][12] * SF[1] +
                   P[3][12] * SF[3] - P[1][12] * SPP[0] + P[13][12] * SPP[3];
    nextP[5][13] = P[5][13] + P[0][13] * SF[2] + P[2][13] * SF[1] +
                   P[3][13] * SF[3] - P[1][13] * SPP[0] + P[13][13] * SPP[3];
    nextP[5][14] = P[5][14] + P[0][14] * SF[2] + P[2][14] * SF[1] +
                   P[3][14] * SF[3] - P[1][14] * SPP[0] + P[13][14] * SPP[3];
    nextP[5][15] = P[5][15] + P[0][15] * SF[2] + P[2][15] * SF[1] +
                   P[3][15] * SF[3] - P[1][15] * SPP[0] + P[13][15] * SPP[3];
    nextP[5][16] = P[5][16] + P[0][16] * SF[2] + P[2][16] * SF[1] +
                   P[3][16] * SF[3] - P[1][16] * SPP[0] + P[13][16] * SPP[3];
    nextP[5][17] = P[5][17] + P[0][17] * SF[2] + P[2][17] * SF[1] +
                   P[3][17] * SF[3] - P[1][17] * SPP[0] + P[13][17] * SPP[3];
    nextP[5][18] = P[5][18] + P[0][18] * SF[2] + P[2][18] * SF[1] +
                   P[3][18] * SF[3] - P[1][18] * SPP[0] + P[13][18] * SPP[3];
    nextP[5][19] = P[5][19] + P[0][19] * SF[2] + P[2][19] * SF[1] +
                   P[3][19] * SF[3] - P[1][19] * SPP[0] + P[13][19] * SPP[3];
    nextP[5][20] = P[5][20] + P[0][20] * SF[2] + P[2][20] * SF[1] +
                   P[3][20] * SF[3] - P[1][20] * SPP[0] + P[13][20] * SPP[3];
    nextP[5][21] = P[5][21] + P[0][21] * SF[2] + P[2][21] * SF[1] +
                   P[3][21] * SF[3] - P[1][21] * SPP[0] + P[13][21] * SPP[3];
    nextP[6][0] = P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                  P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                  P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
                  SF[7] * (P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                           P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                           P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[9] * (P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                           P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                           P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[8] * (P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                           P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                           P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[11] * (P[6][10] + P[1][10] * SF[2] + P[3][10] * SF[1] +
                            P[0][10] * SPP[0] - P[2][10] * SPP[1] -
                            P[13][10] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SPP[7] * (P[6][11] + P[1][11] * SF[2] + P[3][11] * SF[1] +
                            P[0][11] * SPP[0] - P[2][11] * SPP[1] -
                            P[13][11] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SPP[6] * (P[6][12] + P[1][12] * SF[2] + P[3][12] * SF[1] +
                            P[0][12] * SPP[0] - P[2][12] * SPP[1] -
                            P[13][12] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    nextP[6][1] = P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                  P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                  P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
                  SF[6] * (P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                           P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                           P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[5] * (P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                           P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                           P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[9] * (P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                           P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                           P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SPP[6] * (P[6][11] + P[1][11] * SF[2] + P[3][11] * SF[1] +
                            P[0][11] * SPP[0] - P[2][11] * SPP[1] -
                            P[13][11] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  SPP[7] * (P[6][12] + P[1][12] * SF[2] + P[3][12] * SF[1] +
                            P[0][12] * SPP[0] - P[2][12] * SPP[1] -
                            P[13][12] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  (q0 * (P[6][10] + P[1][10] * SF[2] + P[3][10] * SF[1] +
                         P[0][10] * SPP[0] - P[2][10] * SPP[1] -
                         P[13][10] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)))) /
                      2;
    nextP[6][2] = P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                  P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                  P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
                  SF[4] * (P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                           P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                           P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[8] * (P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                           P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                           P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[6] * (P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                           P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                           P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[11] * (P[6][12] + P[1][12] * SF[2] + P[3][12] * SF[1] +
                            P[0][12] * SPP[0] - P[2][12] * SPP[1] -
                            P[13][12] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  SPP[6] * (P[6][10] + P[1][10] * SF[2] + P[3][10] * SF[1] +
                            P[0][10] * SPP[0] - P[2][10] * SPP[1] -
                            P[13][10] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  (q0 * (P[6][11] + P[1][11] * SF[2] + P[3][11] * SF[1] +
                         P[0][11] * SPP[0] - P[2][11] * SPP[1] -
                         P[13][11] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)))) /
                      2;
    nextP[6][3] = P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                  P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                  P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
                  SF[5] * (P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                           P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                           P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[4] * (P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                           P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                           P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[7] * (P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                           P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                           P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  SF[11] * (P[6][11] + P[1][11] * SF[2] + P[3][11] * SF[1] +
                            P[0][11] * SPP[0] - P[2][11] * SPP[1] -
                            P[13][11] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SPP[7] * (P[6][10] + P[1][10] * SF[2] + P[3][10] * SF[1] +
                            P[0][10] * SPP[0] - P[2][10] * SPP[1] -
                            P[13][10] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  (q0 * (P[6][12] + P[1][12] * SF[2] + P[3][12] * SF[1] +
                         P[0][12] * SPP[0] - P[2][12] * SPP[1] -
                         P[13][12] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)))) /
                      2;
    nextP[6][4] = P[6][4] + SQ[1] + P[1][4] * SF[2] + P[3][4] * SF[1] +
                  P[0][4] * SPP[0] - P[2][4] * SPP[1] -
                  P[13][4] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
                  SF[3] * (P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                           P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                           P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[1] * (P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                           P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                           P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SPP[0] * (P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                            P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                            P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  SPP[2] * (P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                            P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                            P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  SPP[4] * (P[6][13] + P[1][13] * SF[2] + P[3][13] * SF[1] +
                            P[0][13] * SPP[0] - P[2][13] * SPP[1] -
                            P[13][13] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    nextP[6][5] = P[6][5] + SQ[0] + P[1][5] * SF[2] + P[3][5] * SF[1] +
                  P[0][5] * SPP[0] - P[2][5] * SPP[1] -
                  P[13][5] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
                  SF[2] * (P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                           P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                           P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[1] * (P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                           P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                           P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SF[3] * (P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                           P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                           P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
                  SPP[0] * (P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                            P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                            P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
                  SPP[3] * (P[6][13] + P[1][13] * SF[2] + P[3][13] * SF[1] +
                            P[0][13] * SPP[0] - P[2][13] * SPP[1] -
                            P[13][13] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    nextP[6][6] =
        P[6][6] + P[1][6] * SF[2] + P[3][6] * SF[1] + P[0][6] * SPP[0] -
        P[2][6] * SPP[1] - P[13][6] * (sq(q0) - sq(q1) - sq(q2) + sq(q3)) +
        dvxCov * sq(SG[6] - 2 * q0 * q2) + dvyCov * sq(SG[5] + 2 * q0 * q1) -
        SPP[5] * (P[6][13] + P[1][13] * SF[2] + P[3][13] * SF[1] +
                  P[0][13] * SPP[0] - P[2][13] * SPP[1] - P[13][13] * SPP[5]) +
        SF[2] * (P[6][1] + P[1][1] * SF[2] + P[3][1] * SF[1] +
                 P[0][1] * SPP[0] - P[2][1] * SPP[1] -
                 P[13][1] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
        SF[1] * (P[6][3] + P[1][3] * SF[2] + P[3][3] * SF[1] +
                 P[0][3] * SPP[0] - P[2][3] * SPP[1] -
                 P[13][3] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
        SPP[0] * (P[6][0] + P[1][0] * SF[2] + P[3][0] * SF[1] +
                  P[0][0] * SPP[0] - P[2][0] * SPP[1] -
                  P[13][0] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) -
        SPP[1] * (P[6][2] + P[1][2] * SF[2] + P[3][2] * SF[1] +
                  P[0][2] * SPP[0] - P[2][2] * SPP[1] -
                  P[13][2] * (sq(q0) - sq(q1) - sq(q2) + sq(q3))) +
        dvzCov * sq(SG[1] - SG[2] - SG[3] + SG[4]);
    nextP[6][7] =
        P[6][7] + P[1][7] * SF[2] + P[3][7] * SF[1] + P[0][7] * SPP[0] -
        P[2][7] * SPP[1] - P[13][7] * SPP[5] +
        dt * (P[6][4] + P[1][4] * SF[2] + P[3][4] * SF[1] + P[0][4] * SPP[0] -
              P[2][4] * SPP[1] - P[13][4] * SPP[5]);
    nextP[6][8] =
        P[6][8] + P[1][8] * SF[2] + P[3][8] * SF[1] + P[0][8] * SPP[0] -
        P[2][8] * SPP[1] - P[13][8] * SPP[5] +
        dt * (P[6][5] + P[1][5] * SF[2] + P[3][5] * SF[1] + P[0][5] * SPP[0] -
              P[2][5] * SPP[1] - P[13][5] * SPP[5]);
    nextP[6][9] =
        P[6][9] + P[1][9] * SF[2] + P[3][9] * SF[1] + P[0][9] * SPP[0] -
        P[2][9] * SPP[1] - P[13][9] * SPP[5] +
        dt * (P[6][6] + P[1][6] * SF[2] + P[3][6] * SF[1] + P[0][6] * SPP[0] -
              P[2][6] * SPP[1] - P[13][6] * SPP[5]);
    nextP[6][10] = P[6][10] + P[1][10] * SF[2] + P[3][10] * SF[1] +
                   P[0][10] * SPP[0] - P[2][10] * SPP[1] - P[13][10] * SPP[5];
    nextP[6][11] = P[6][11] + P[1][11] * SF[2] + P[3][11] * SF[1] +
                   P[0][11] * SPP[0] - P[2][11] * SPP[1] - P[13][11] * SPP[5];
    nextP[6][12] = P[6][12] + P[1][12] * SF[2] + P[3][12] * SF[1] +
                   P[0][12] * SPP[0] - P[2][12] * SPP[1] - P[13][12] * SPP[5];
    nextP[6][13] = P[6][13] + P[1][13] * SF[2] + P[3][13] * SF[1] +
                   P[0][13] * SPP[0] - P[2][13] * SPP[1] - P[13][13] * SPP[5];
    nextP[6][14] = P[6][14] + P[1][14] * SF[2] + P[3][14] * SF[1] +
                   P[0][14] * SPP[0] - P[2][14] * SPP[1] - P[13][14] * SPP[5];
    nextP[6][15] = P[6][15] + P[1][15] * SF[2] + P[3][15] * SF[1] +
                   P[0][15] * SPP[0] - P[2][15] * SPP[1] - P[13][15] * SPP[5];
    nextP[6][16] = P[6][16] + P[1][16] * SF[2] + P[3][16] * SF[1] +
                   P[0][16] * SPP[0] - P[2][16] * SPP[1] - P[13][16] * SPP[5];
    nextP[6][17] = P[6][17] + P[1][17] * SF[2] + P[3][17] * SF[1] +
                   P[0][17] * SPP[0] - P[2][17] * SPP[1] - P[13][17] * SPP[5];
    nextP[6][18] = P[6][18] + P[1][18] * SF[2] + P[3][18] * SF[1] +
                   P[0][18] * SPP[0] - P[2][18] * SPP[1] - P[13][18] * SPP[5];
    nextP[6][19] = P[6][19] + P[1][19] * SF[2] + P[3][19] * SF[1] +
                   P[0][19] * SPP[0] - P[2][19] * SPP[1] - P[13][19] * SPP[5];
    nextP[6][20] = P[6][20] + P[1][20] * SF[2] + P[3][20] * SF[1] +
                   P[0][20] * SPP[0] - P[2][20] * SPP[1] - P[13][20] * SPP[5];
    nextP[6][21] = P[6][21] + P[1][21] * SF[2] + P[3][21] * SF[1] +
                   P[0][21] * SPP[0] - P[2][21] * SPP[1] - P[13][21] * SPP[5];
    nextP[7][0] = P[7][0] + P[4][0] * dt + SF[7] * (P[7][1] + P[4][1] * dt) +
                  SF[9] * (P[7][2] + P[4][2] * dt) +
                  SF[8] * (P[7][3] + P[4][3] * dt) +
                  SF[11] * (P[7][10] + P[4][10] * dt) +
                  SPP[7] * (P[7][11] + P[4][11] * dt) +
                  SPP[6] * (P[7][12] + P[4][12] * dt);
    nextP[7][1] = P[7][1] + P[4][1] * dt + SF[6] * (P[7][0] + P[4][0] * dt) +
                  SF[5] * (P[7][2] + P[4][2] * dt) +
                  SF[9] * (P[7][3] + P[4][3] * dt) +
                  SPP[6] * (P[7][11] + P[4][11] * dt) -
                  SPP[7] * (P[7][12] + P[4][12] * dt) -
                  (q0 * (P[7][10] + P[4][10] * dt)) / 2;
    nextP[7][2] = P[7][2] + P[4][2] * dt + SF[4] * (P[7][0] + P[4][0] * dt) +
                  SF[8] * (P[7][1] + P[4][1] * dt) +
                  SF[6] * (P[7][3] + P[4][3] * dt) +
                  SF[11] * (P[7][12] + P[4][12] * dt) -
                  SPP[6] * (P[7][10] + P[4][10] * dt) -
                  (q0 * (P[7][11] + P[4][11] * dt)) / 2;
    nextP[7][3] = P[7][3] + P[4][3] * dt + SF[5] * (P[7][0] + P[4][0] * dt) +
                  SF[4] * (P[7][1] + P[4][1] * dt) +
                  SF[7] * (P[7][2] + P[4][2] * dt) -
                  SF[11] * (P[7][11] + P[4][11] * dt) +
                  SPP[7] * (P[7][10] + P[4][10] * dt) -
                  (q0 * (P[7][12] + P[4][12] * dt)) / 2;
    nextP[7][4] =
        P[7][4] + P[4][4] * dt + SF[1] * (P[7][1] + P[4][1] * dt) +
        SF[3] * (P[7][0] + P[4][0] * dt) + SPP[0] * (P[7][2] + P[4][2] * dt) -
        SPP[2] * (P[7][3] + P[4][3] * dt) - SPP[4] * (P[7][13] + P[4][13] * dt);
    nextP[7][5] =
        P[7][5] + P[4][5] * dt + SF[2] * (P[7][0] + P[4][0] * dt) +
        SF[1] * (P[7][2] + P[4][2] * dt) + SF[3] * (P[7][3] + P[4][3] * dt) -
        SPP[0] * (P[7][1] + P[4][1] * dt) + SPP[3] * (P[7][13] + P[4][13] * dt);
    nextP[7][6] =
        P[7][6] + P[4][6] * dt + SF[2] * (P[7][1] + P[4][1] * dt) +
        SF[1] * (P[7][3] + P[4][3] * dt) + SPP[0] * (P[7][0] + P[4][0] * dt) -
        SPP[1] * (P[7][2] + P[4][2] * dt) - SPP[5] * (P[7][13] + P[4][13] * dt);
    nextP[7][7] = P[7][7] + P[4][7] * dt + dt * (P[7][4] + P[4][4] * dt);
    nextP[7][8] = P[7][8] + P[4][8] * dt + dt * (P[7][5] + P[4][5] * dt);
    nextP[7][9] = P[7][9] + P[4][9] * dt + dt * (P[7][6] + P[4][6] * dt);
    nextP[7][10] = P[7][10] + P[4][10] * dt;
    nextP[7][11] = P[7][11] + P[4][11] * dt;
    nextP[7][12] = P[7][12] + P[4][12] * dt;
    nextP[7][13] = P[7][13] + P[4][13] * dt;
    nextP[7][14] = P[7][14] + P[4][14] * dt;
    nextP[7][15] = P[7][15] + P[4][15] * dt;
    nextP[7][16] = P[7][16] + P[4][16] * dt;
    nextP[7][17] = P[7][17] + P[4][17] * dt;
    nextP[7][18] = P[7][18] + P[4][18] * dt;
    nextP[7][19] = P[7][19] + P[4][19] * dt;
    nextP[7][20] = P[7][20] + P[4][20] * dt;
    nextP[7][21] = P[7][21] + P[4][21] * dt;
    nextP[8][0] = P[8][0] + P[5][0] * dt + SF[7] * (P[8][1] + P[5][1] * dt) +
                  SF[9] * (P[8][2] + P[5][2] * dt) +
                  SF[8] * (P[8][3] + P[5][3] * dt) +
                  SF[11] * (P[8][10] + P[5][10] * dt) +
                  SPP[7] * (P[8][11] + P[5][11] * dt) +
                  SPP[6] * (P[8][12] + P[5][12] * dt);
    nextP[8][1] = P[8][1] + P[5][1] * dt + SF[6] * (P[8][0] + P[5][0] * dt) +
                  SF[5] * (P[8][2] + P[5][2] * dt) +
                  SF[9] * (P[8][3] + P[5][3] * dt) +
                  SPP[6] * (P[8][11] + P[5][11] * dt) -
                  SPP[7] * (P[8][12] + P[5][12] * dt) -
                  (q0 * (P[8][10] + P[5][10] * dt)) / 2;
    nextP[8][2] = P[8][2] + P[5][2] * dt + SF[4] * (P[8][0] + P[5][0] * dt) +
                  SF[8] * (P[8][1] + P[5][1] * dt) +
                  SF[6] * (P[8][3] + P[5][3] * dt) +
                  SF[11] * (P[8][12] + P[5][12] * dt) -
                  SPP[6] * (P[8][10] + P[5][10] * dt) -
                  (q0 * (P[8][11] + P[5][11] * dt)) / 2;
    nextP[8][3] = P[8][3] + P[5][3] * dt + SF[5] * (P[8][0] + P[5][0] * dt) +
                  SF[4] * (P[8][1] + P[5][1] * dt) +
                  SF[7] * (P[8][2] + P[5][2] * dt) -
                  SF[11] * (P[8][11] + P[5][11] * dt) +
                  SPP[7] * (P[8][10] + P[5][10] * dt) -
                  (q0 * (P[8][12] + P[5][12] * dt)) / 2;
    nextP[8][4] =
        P[8][4] + P[5][4] * dt + SF[1] * (P[8][1] + P[5][1] * dt) +
        SF[3] * (P[8][0] + P[5][0] * dt) + SPP[0] * (P[8][2] + P[5][2] * dt) -
        SPP[2] * (P[8][3] + P[5][3] * dt) - SPP[4] * (P[8][13] + P[5][13] * dt);
    nextP[8][5] =
        P[8][5] + P[5][5] * dt + SF[2] * (P[8][0] + P[5][0] * dt) +
        SF[1] * (P[8][2] + P[5][2] * dt) + SF[3] * (P[8][3] + P[5][3] * dt) -
        SPP[0] * (P[8][1] + P[5][1] * dt) + SPP[3] * (P[8][13] + P[5][13] * dt);
    nextP[8][6] =
        P[8][6] + P[5][6] * dt + SF[2] * (P[8][1] + P[5][1] * dt) +
        SF[1] * (P[8][3] + P[5][3] * dt) + SPP[0] * (P[8][0] + P[5][0] * dt) -
        SPP[1] * (P[8][2] + P[5][2] * dt) - SPP[5] * (P[8][13] + P[5][13] * dt);
    nextP[8][7] = P[8][7] + P[5][7] * dt + dt * (P[8][4] + P[5][4] * dt);
    nextP[8][8] = P[8][8] + P[5][8] * dt + dt * (P[8][5] + P[5][5] * dt);
    nextP[8][9] = P[8][9] + P[5][9] * dt + dt * (P[8][6] + P[5][6] * dt);
    nextP[8][10] = P[8][10] + P[5][10] * dt;
    nextP[8][11] = P[8][11] + P[5][11] * dt;
    nextP[8][12] = P[8][12] + P[5][12] * dt;
    nextP[8][13] = P[8][13] + P[5][13] * dt;
    nextP[8][14] = P[8][14] + P[5][14] * dt;
    nextP[8][15] = P[8][15] + P[5][15] * dt;
    nextP[8][16] = P[8][16] + P[5][16] * dt;
    nextP[8][17] = P[8][17] + P[5][17] * dt;
    nextP[8][18] = P[8][18] + P[5][18] * dt;
    nextP[8][19] = P[8][19] + P[5][19] * dt;
    nextP[8][20] = P[8][20] + P[5][20] * dt;
    nextP[8][21] = P[8][21] + P[5][21] * dt;
    nextP[9][0] = P[9][0] + P[6][0] * dt + SF[7] * (P[9][1] + P[6][1] * dt) +
                  SF[9] * (P[9][2] + P[6][2] * dt) +
                  SF[8] * (P[9][3] + P[6][3] * dt) +
                  SF[11] * (P[9][10] + P[6][10] * dt) +
                  SPP[7] * (P[9][11] + P[6][11] * dt) +
                  SPP[6] * (P[9][12] + P[6][12] * dt);
    nextP[9][1] = P[9][1] + P[6][1] * dt + SF[6] * (P[9][0] + P[6][0] * dt) +
                  SF[5] * (P[9][2] + P[6][2] * dt) +
                  SF[9] * (P[9][3] + P[6][3] * dt) +
                  SPP[6] * (P[9][11] + P[6][11] * dt) -
                  SPP[7] * (P[9][12] + P[6][12] * dt) -
                  (q0 * (P[9][10] + P[6][10] * dt)) / 2;
    nextP[9][2] = P[9][2] + P[6][2] * dt + SF[4] * (P[9][0] + P[6][0] * dt) +
                  SF[8] * (P[9][1] + P[6][1] * dt) +
                  SF[6] * (P[9][3] + P[6][3] * dt) +
                  SF[11] * (P[9][12] + P[6][12] * dt) -
                  SPP[6] * (P[9][10] + P[6][10] * dt) -
                  (q0 * (P[9][11] + P[6][11] * dt)) / 2;
    nextP[9][3] = P[9][3] + P[6][3] * dt + SF[5] * (P[9][0] + P[6][0] * dt) +
                  SF[4] * (P[9][1] + P[6][1] * dt) +
                  SF[7] * (P[9][2] + P[6][2] * dt) -
                  SF[11] * (P[9][11] + P[6][11] * dt) +
                  SPP[7] * (P[9][10] + P[6][10] * dt) -
                  (q0 * (P[9][12] + P[6][12] * dt)) / 2;
    nextP[9][4] =
        P[9][4] + P[6][4] * dt + SF[1] * (P[9][1] + P[6][1] * dt) +
        SF[3] * (P[9][0] + P[6][0] * dt) + SPP[0] * (P[9][2] + P[6][2] * dt) -
        SPP[2] * (P[9][3] + P[6][3] * dt) - SPP[4] * (P[9][13] + P[6][13] * dt);
    nextP[9][5] =
        P[9][5] + P[6][5] * dt + SF[2] * (P[9][0] + P[6][0] * dt) +
        SF[1] * (P[9][2] + P[6][2] * dt) + SF[3] * (P[9][3] + P[6][3] * dt) -
        SPP[0] * (P[9][1] + P[6][1] * dt) + SPP[3] * (P[9][13] + P[6][13] * dt);
    nextP[9][6] =
        P[9][6] + P[6][6] * dt + SF[2] * (P[9][1] + P[6][1] * dt) +
        SF[1] * (P[9][3] + P[6][3] * dt) + SPP[0] * (P[9][0] + P[6][0] * dt) -
        SPP[1] * (P[9][2] + P[6][2] * dt) - SPP[5] * (P[9][13] + P[6][13] * dt);
    nextP[9][7] = P[9][7] + P[6][7] * dt + dt * (P[9][4] + P[6][4] * dt);
    nextP[9][8] = P[9][8] + P[6][8] * dt + dt * (P[9][5] + P[6][5] * dt);
    nextP[9][9] = P[9][9] + P[6][9] * dt + dt * (P[9][6] + P[6][6] * dt);
    nextP[9][10] = P[9][10] + P[6][10] * dt;
    nextP[9][11] = P[9][11] + P[6][11] * dt;
    nextP[9][12] = P[9][12] + P[6][12] * dt;
    nextP[9][13] = P[9][13] + P[6][13] * dt;
    nextP[9][14] = P[9][14] + P[6][14] * dt;
    nextP[9][15] = P[9][15] + P[6][15] * dt;
    nextP[9][16] = P[9][16] + P[6][16] * dt;
    nextP[9][17] = P[9][17] + P[6][17] * dt;
    nextP[9][18] = P[9][18] + P[6][18] * dt;
    nextP[9][19] = P[9][19] + P[6][19] * dt;
    nextP[9][20] = P[9][20] + P[6][20] * dt;
    nextP[9][21] = P[9][21] + P[6][21] * dt;
    nextP[10][0] = P[10][0] + P[10][1] * SF[7] + P[10][2] * SF[9] +
                   P[10][3] * SF[8] + P[10][10] * SF[11] + P[10][11] * SPP[7] +
                   P[10][12] * SPP[6];
    nextP[10][1] = P[10][1] + P[10][0] * SF[6] + P[10][2] * SF[5] +
                   P[10][3] * SF[9] + P[10][11] * SPP[6] - P[10][12] * SPP[7] -
                   (P[10][10] * q0) / 2;
    nextP[10][2] = P[10][2] + P[10][0] * SF[4] + P[10][1] * SF[8] +
                   P[10][3] * SF[6] + P[10][12] * SF[11] - P[10][10] * SPP[6] -
                   (P[10][11] * q0) / 2;
    nextP[10][3] = P[10][3] + P[10][0] * SF[5] + P[10][1] * SF[4] +
                   P[10][2] * SF[7] - P[10][11] * SF[11] + P[10][10] * SPP[7] -
                   (P[10][12] * q0) / 2;
    nextP[10][4] = P[10][4] + P[10][1] * SF[1] + P[10][0] * SF[3] +
                   P[10][2] * SPP[0] - P[10][3] * SPP[2] - P[10][13] * SPP[4];
    nextP[10][5] = P[10][5] + P[10][0] * SF[2] + P[10][2] * SF[1] +
                   P[10][3] * SF[3] - P[10][1] * SPP[0] + P[10][13] * SPP[3];
    nextP[10][6] = P[10][6] + P[10][1] * SF[2] + P[10][3] * SF[1] +
                   P[10][0] * SPP[0] - P[10][2] * SPP[1] - P[10][13] * SPP[5];
    nextP[10][7] = P[10][7] + P[10][4] * dt;
    nextP[10][8] = P[10][8] + P[10][5] * dt;
    nextP[10][9] = P[10][9] + P[10][6] * dt;
    nextP[10][10] = P[10][10];
    nextP[10][11] = P[10][11];
    nextP[10][12] = P[10][12];
    nextP[10][13] = P[10][13];
    nextP[10][14] = P[10][14];
    nextP[10][15] = P[10][15];
    nextP[10][16] = P[10][16];
    nextP[10][17] = P[10][17];
    nextP[10][18] = P[10][18];
    nextP[10][19] = P[10][19];
    nextP[10][20] = P[10][20];
    nextP[10][21] = P[10][21];
    nextP[11][0] = P[11][0] + P[11][1] * SF[7] + P[11][2] * SF[9] +
                   P[11][3] * SF[8] + P[11][10] * SF[11] + P[11][11] * SPP[7] +
                   P[11][12] * SPP[6];
    nextP[11][1] = P[11][1] + P[11][0] * SF[6] + P[11][2] * SF[5] +
                   P[11][3] * SF[9] + P[11][11] * SPP[6] - P[11][12] * SPP[7] -
                   (P[11][10] * q0) / 2;
    nextP[11][2] = P[11][2] + P[11][0] * SF[4] + P[11][1] * SF[8] +
                   P[11][3] * SF[6] + P[11][12] * SF[11] - P[11][10] * SPP[6] -
                   (P[11][11] * q0) / 2;
    nextP[11][3] = P[11][3] + P[11][0] * SF[5] + P[11][1] * SF[4] +
                   P[11][2] * SF[7] - P[11][11] * SF[11] + P[11][10] * SPP[7] -
                   (P[11][12] * q0) / 2;
    nextP[11][4] = P[11][4] + P[11][1] * SF[1] + P[11][0] * SF[3] +
                   P[11][2] * SPP[0] - P[11][3] * SPP[2] - P[11][13] * SPP[4];
    nextP[11][5] = P[11][5] + P[11][0] * SF[2] + P[11][2] * SF[1] +
                   P[11][3] * SF[3] - P[11][1] * SPP[0] + P[11][13] * SPP[3];
    nextP[11][6] = P[11][6] + P[11][1] * SF[2] + P[11][3] * SF[1] +
                   P[11][0] * SPP[0] - P[11][2] * SPP[1] - P[11][13] * SPP[5];
    nextP[11][7] = P[11][7] + P[11][4] * dt;
    nextP[11][8] = P[11][8] + P[11][5] * dt;
    nextP[11][9] = P[11][9] + P[11][6] * dt;
    nextP[11][10] = P[11][10];
    nextP[11][11] = P[11][11];
    nextP[11][12] = P[11][12];
    nextP[11][13] = P[11][13];
    nextP[11][14] = P[11][14];
    nextP[11][15] = P[11][15];
    nextP[11][16] = P[11][16];
    nextP[11][17] = P[11][17];
    nextP[11][18] = P[11][18];
    nextP[11][19] = P[11][19];
    nextP[11][20] = P[11][20];
    nextP[11][21] = P[11][21];
    nextP[12][0] = P[12][0] + P[12][1] * SF[7] + P[12][2] * SF[9] +
                   P[12][3] * SF[8] + P[12][10] * SF[11] + P[12][11] * SPP[7] +
                   P[12][12] * SPP[6];
    nextP[12][1] = P[12][1] + P[12][0] * SF[6] + P[12][2] * SF[5] +
                   P[12][3] * SF[9] + P[12][11] * SPP[6] - P[12][12] * SPP[7] -
                   (P[12][10] * q0) / 2;
    nextP[12][2] = P[12][2] + P[12][0] * SF[4] + P[12][1] * SF[8] +
                   P[12][3] * SF[6] + P[12][12] * SF[11] - P[12][10] * SPP[6] -
                   (P[12][11] * q0) / 2;
    nextP[12][3] = P[12][3] + P[12][0] * SF[5] + P[12][1] * SF[4] +
                   P[12][2] * SF[7] - P[12][11] * SF[11] + P[12][10] * SPP[7] -
                   (P[12][12] * q0) / 2;
    nextP[12][4] = P[12][4] + P[12][1] * SF[1] + P[12][0] * SF[3] +
                   P[12][2] * SPP[0] - P[12][3] * SPP[2] - P[12][13] * SPP[4];
    nextP[12][5] = P[12][5] + P[12][0] * SF[2] + P[12][2] * SF[1] +
                   P[12][3] * SF[3] - P[12][1] * SPP[0] + P[12][13] * SPP[3];
    nextP[12][6] = P[12][6] + P[12][1] * SF[2] + P[12][3] * SF[1] +
                   P[12][0] * SPP[0] - P[12][2] * SPP[1] - P[12][13] * SPP[5];
    nextP[12][7] = P[12][7] + P[12][4] * dt;
    nextP[12][8] = P[12][8] + P[12][5] * dt;
    nextP[12][9] = P[12][9] + P[12][6] * dt;
    nextP[12][10] = P[12][10];
    nextP[12][11] = P[12][11];
    nextP[12][12] = P[12][12];
    nextP[12][13] = P[12][13];
    nextP[12][14] = P[12][14];
    nextP[12][15] = P[12][15];
    nextP[12][16] = P[12][16];
    nextP[12][17] = P[12][17];
    nextP[12][18] = P[12][18];
    nextP[12][19] = P[12][19];
    nextP[12][20] = P[12][20];
    nextP[12][21] = P[12][21];
    nextP[13][0] = P[13][0] + P[13][1] * SF[7] + P[13][2] * SF[9] +
                   P[13][3] * SF[8] + P[13][10] * SF[11] + P[13][11] * SPP[7] +
                   P[13][12] * SPP[6];
    nextP[13][1] = P[13][1] + P[13][0] * SF[6] + P[13][2] * SF[5] +
                   P[13][3] * SF[9] + P[13][11] * SPP[6] - P[13][12] * SPP[7] -
                   (P[13][10] * q0) / 2;
    nextP[13][2] = P[13][2] + P[13][0] * SF[4] + P[13][1] * SF[8] +
                   P[13][3] * SF[6] + P[13][12] * SF[11] - P[13][10] * SPP[6] -
                   (P[13][11] * q0) / 2;
    nextP[13][3] = P[13][3] + P[13][0] * SF[5] + P[13][1] * SF[4] +
                   P[13][2] * SF[7] - P[13][11] * SF[11] + P[13][10] * SPP[7] -
                   (P[13][12] * q0) / 2;
    nextP[13][4] = P[13][4] + P[13][1] * SF[1] + P[13][0] * SF[3] +
                   P[13][2] * SPP[0] - P[13][3] * SPP[2] - P[13][13] * SPP[4];
    nextP[13][5] = P[13][5] + P[13][0] * SF[2] + P[13][2] * SF[1] +
                   P[13][3] * SF[3] - P[13][1] * SPP[0] + P[13][13] * SPP[3];
    nextP[13][6] = P[13][6] + P[13][1] * SF[2] + P[13][3] * SF[1] +
                   P[13][0] * SPP[0] - P[13][2] * SPP[1] - P[13][13] * SPP[5];
    nextP[13][7] = P[13][7] + P[13][4] * dt;
    nextP[13][8] = P[13][8] + P[13][5] * dt;
    nextP[13][9] = P[13][9] + P[13][6] * dt;
    nextP[13][10] = P[13][10];
    nextP[13][11] = P[13][11];
    nextP[13][12] = P[13][12];
    nextP[13][13] = P[13][13];
    nextP[13][14] = P[13][14];
    nextP[13][15] = P[13][15];
    nextP[13][16] = P[13][16];
    nextP[13][17] = P[13][17];
    nextP[13][18] = P[13][18];
    nextP[13][19] = P[13][19];
    nextP[13][20] = P[13][20];
    nextP[13][21] = P[13][21];
    nextP[14][0] = P[14][0] + P[14][1] * SF[7] + P[14][2] * SF[9] +
                   P[14][3] * SF[8] + P[14][10] * SF[11] + P[14][11] * SPP[7] +
                   P[14][12] * SPP[6];
    nextP[14][1] = P[14][1] + P[14][0] * SF[6] + P[14][2] * SF[5] +
                   P[14][3] * SF[9] + P[14][11] * SPP[6] - P[14][12] * SPP[7] -
                   (P[14][10] * q0) / 2;
    nextP[14][2] = P[14][2] + P[14][0] * SF[4] + P[14][1] * SF[8] +
                   P[14][3] * SF[6] + P[14][12] * SF[11] - P[14][10] * SPP[6] -
                   (P[14][11] * q0) / 2;
    nextP[14][3] = P[14][3] + P[14][0] * SF[5] + P[14][1] * SF[4] +
                   P[14][2] * SF[7] - P[14][11] * SF[11] + P[14][10] * SPP[7] -
                   (P[14][12] * q0) / 2;
    nextP[14][4] = P[14][4] + P[14][1] * SF[1] + P[14][0] * SF[3] +
                   P[14][2] * SPP[0] - P[14][3] * SPP[2] - P[14][13] * SPP[4];
    nextP[14][5] = P[14][5] + P[14][0] * SF[2] + P[14][2] * SF[1] +
                   P[14][3] * SF[3] - P[14][1] * SPP[0] + P[14][13] * SPP[3];
    nextP[14][6] = P[14][6] + P[14][1] * SF[2] + P[14][3] * SF[1] +
                   P[14][0] * SPP[0] - P[14][2] * SPP[1] - P[14][13] * SPP[5];
    nextP[14][7] = P[14][7] + P[14][4] * dt;
    nextP[14][8] = P[14][8] + P[14][5] * dt;
    nextP[14][9] = P[14][9] + P[14][6] * dt;
    nextP[14][10] = P[14][10];
    nextP[14][11] = P[14][11];
    nextP[14][12] = P[14][12];
    nextP[14][13] = P[14][13];
    nextP[14][14] = P[14][14];
    nextP[14][15] = P[14][15];
    nextP[14][16] = P[14][16];
    nextP[14][17] = P[14][17];
    nextP[14][18] = P[14][18];
    nextP[14][19] = P[14][19];
    nextP[14][20] = P[14][20];
    nextP[14][21] = P[14][21];
    nextP[15][0] = P[15][0] + P[15][1] * SF[7] + P[15][2] * SF[9] +
                   P[15][3] * SF[8] + P[15][10] * SF[11] + P[15][11] * SPP[7] +
                   P[15][12] * SPP[6];
    nextP[15][1] = P[15][1] + P[15][0] * SF[6] + P[15][2] * SF[5] +
                   P[15][3] * SF[9] + P[15][11] * SPP[6] - P[15][12] * SPP[7] -
                   (P[15][10] * q0) / 2;
    nextP[15][2] = P[15][2] + P[15][0] * SF[4] + P[15][1] * SF[8] +
                   P[15][3] * SF[6] + P[15][12] * SF[11] - P[15][10] * SPP[6] -
                   (P[15][11] * q0) / 2;
    nextP[15][3] = P[15][3] + P[15][0] * SF[5] + P[15][1] * SF[4] +
                   P[15][2] * SF[7] - P[15][11] * SF[11] + P[15][10] * SPP[7] -
                   (P[15][12] * q0) / 2;
    nextP[15][4] = P[15][4] + P[15][1] * SF[1] + P[15][0] * SF[3] +
                   P[15][2] * SPP[0] - P[15][3] * SPP[2] - P[15][13] * SPP[4];
    nextP[15][5] = P[15][5] + P[15][0] * SF[2] + P[15][2] * SF[1] +
                   P[15][3] * SF[3] - P[15][1] * SPP[0] + P[15][13] * SPP[3];
    nextP[15][6] = P[15][6] + P[15][1] * SF[2] + P[15][3] * SF[1] +
                   P[15][0] * SPP[0] - P[15][2] * SPP[1] - P[15][13] * SPP[5];
    nextP[15][7] = P[15][7] + P[15][4] * dt;
    nextP[15][8] = P[15][8] + P[15][5] * dt;
    nextP[15][9] = P[15][9] + P[15][6] * dt;
    nextP[15][10] = P[15][10];
    nextP[15][11] = P[15][11];
    nextP[15][12] = P[15][12];
    nextP[15][13] = P[15][13];
    nextP[15][14] = P[15][14];
    nextP[15][15] = P[15][15];
    nextP[15][16] = P[15][16];
    nextP[15][17] = P[15][17];
    nextP[15][18] = P[15][18];
    nextP[15][19] = P[15][19];
    nextP[15][20] = P[15][20];
    nextP[15][21] = P[15][21];
    nextP[16][0] = P[16][0] + P[16][1] * SF[7] + P[16][2] * SF[9] +
                   P[16][3] * SF[8] + P[16][10] * SF[11] + P[16][11] * SPP[7] +
                   P[16][12] * SPP[6];
    nextP[16][1] = P[16][1] + P[16][0] * SF[6] + P[16][2] * SF[5] +
                   P[16][3] * SF[9] + P[16][11] * SPP[6] - P[16][12] * SPP[7] -
                   (P[16][10] * q0) / 2;
    nextP[16][2] = P[16][2] + P[16][0] * SF[4] + P[16][1] * SF[8] +
                   P[16][3] * SF[6] + P[16][12] * SF[11] - P[16][10] * SPP[6] -
                   (P[16][11] * q0) / 2;
    nextP[16][3] = P[16][3] + P[16][0] * SF[5] + P[16][1] * SF[4] +
                   P[16][2] * SF[7] - P[16][11] * SF[11] + P[16][10] * SPP[7] -
                   (P[16][12] * q0) / 2;
    nextP[16][4] = P[16][4] + P[16][1] * SF[1] + P[16][0] * SF[3] +
                   P[16][2] * SPP[0] - P[16][3] * SPP[2] - P[16][13] * SPP[4];
    nextP[16][5] = P[16][5] + P[16][0] * SF[2] + P[16][2] * SF[1] +
                   P[16][3] * SF[3] - P[16][1] * SPP[0] + P[16][13] * SPP[3];
    nextP[16][6] = P[16][6] + P[16][1] * SF[2] + P[16][3] * SF[1] +
                   P[16][0] * SPP[0] - P[16][2] * SPP[1] - P[16][13] * SPP[5];
    nextP[16][7] = P[16][7] + P[16][4] * dt;
    nextP[16][8] = P[16][8] + P[16][5] * dt;
    nextP[16][9] = P[16][9] + P[16][6] * dt;
    nextP[16][10] = P[16][10];
    nextP[16][11] = P[16][11];
    nextP[16][12] = P[16][12];
    nextP[16][13] = P[16][13];
    nextP[16][14] = P[16][14];
    nextP[16][15] = P[16][15];
    nextP[16][16] = P[16][16];
    nextP[16][17] = P[16][17];
    nextP[16][18] = P[16][18];
    nextP[16][19] = P[16][19];
    nextP[16][20] = P[16][20];
    nextP[16][21] = P[16][21];
    nextP[17][0] = P[17][0] + P[17][1] * SF[7] + P[17][2] * SF[9] +
                   P[17][3] * SF[8] + P[17][10] * SF[11] + P[17][11] * SPP[7] +
                   P[17][12] * SPP[6];
    nextP[17][1] = P[17][1] + P[17][0] * SF[6] + P[17][2] * SF[5] +
                   P[17][3] * SF[9] + P[17][11] * SPP[6] - P[17][12] * SPP[7] -
                   (P[17][10] * q0) / 2;
    nextP[17][2] = P[17][2] + P[17][0] * SF[4] + P[17][1] * SF[8] +
                   P[17][3] * SF[6] + P[17][12] * SF[11] - P[17][10] * SPP[6] -
                   (P[17][11] * q0) / 2;
    nextP[17][3] = P[17][3] + P[17][0] * SF[5] + P[17][1] * SF[4] +
                   P[17][2] * SF[7] - P[17][11] * SF[11] + P[17][10] * SPP[7] -
                   (P[17][12] * q0) / 2;
    nextP[17][4] = P[17][4] + P[17][1] * SF[1] + P[17][0] * SF[3] +
                   P[17][2] * SPP[0] - P[17][3] * SPP[2] - P[17][13] * SPP[4];
    nextP[17][5] = P[17][5] + P[17][0] * SF[2] + P[17][2] * SF[1] +
                   P[17][3] * SF[3] - P[17][1] * SPP[0] + P[17][13] * SPP[3];
    nextP[17][6] = P[17][6] + P[17][1] * SF[2] + P[17][3] * SF[1] +
                   P[17][0] * SPP[0] - P[17][2] * SPP[1] - P[17][13] * SPP[5];
    nextP[17][7] = P[17][7] + P[17][4] * dt;
    nextP[17][8] = P[17][8] + P[17][5] * dt;
    nextP[17][9] = P[17][9] + P[17][6] * dt;
    nextP[17][10] = P[17][10];
    nextP[17][11] = P[17][11];
    nextP[17][12] = P[17][12];
    nextP[17][13] = P[17][13];
    nextP[17][14] = P[17][14];
    nextP[17][15] = P[17][15];
    nextP[17][16] = P[17][16];
    nextP[17][17] = P[17][17];
    nextP[17][18] = P[17][18];
    nextP[17][19] = P[17][19];
    nextP[17][20] = P[17][20];
    nextP[17][21] = P[17][21];
    nextP[18][0] = P[18][0] + P[18][1] * SF[7] + P[18][2] * SF[9] +
                   P[18][3] * SF[8] + P[18][10] * SF[11] + P[18][11] * SPP[7] +
                   P[18][12] * SPP[6];
    nextP[18][1] = P[18][1] + P[18][0] * SF[6] + P[18][2] * SF[5] +
                   P[18][3] * SF[9] + P[18][11] * SPP[6] - P[18][12] * SPP[7] -
                   (P[18][10] * q0) / 2;
    nextP[18][2] = P[18][2] + P[18][0] * SF[4] + P[18][1] * SF[8] +
                   P[18][3] * SF[6] + P[18][12] * SF[11] - P[18][10] * SPP[6] -
                   (P[18][11] * q0) / 2;
    nextP[18][3] = P[18][3] + P[18][0] * SF[5] + P[18][1] * SF[4] +
                   P[18][2] * SF[7] - P[18][11] * SF[11] + P[18][10] * SPP[7] -
                   (P[18][12] * q0) / 2;
    nextP[18][4] = P[18][4] + P[18][1] * SF[1] + P[18][0] * SF[3] +
                   P[18][2] * SPP[0] - P[18][3] * SPP[2] - P[18][13] * SPP[4];
    nextP[18][5] = P[18][5] + P[18][0] * SF[2] + P[18][2] * SF[1] +
                   P[18][3] * SF[3] - P[18][1] * SPP[0] + P[18][13] * SPP[3];
    nextP[18][6] = P[18][6] + P[18][1] * SF[2] + P[18][3] * SF[1] +
                   P[18][0] * SPP[0] - P[18][2] * SPP[1] - P[18][13] * SPP[5];
    nextP[18][7] = P[18][7] + P[18][4] * dt;
    nextP[18][8] = P[18][8] + P[18][5] * dt;
    nextP[18][9] = P[18][9] + P[18][6] * dt;
    nextP[18][10] = P[18][10];
    nextP[18][11] = P[18][11];
    nextP[18][12] = P[18][12];
    nextP[18][13] = P[18][13];
    nextP[18][14] = P[18][14];
    nextP[18][15] = P[18][15];
    nextP[18][16] = P[18][16];
    nextP[18][17] = P[18][17];
    nextP[18][18] = P[18][18];
    nextP[18][19] = P[18][19];
    nextP[18][20] = P[18][20];
    nextP[18][21] = P[18][21];
    nextP[19][0] = P[19][0] + P[19][1] * SF[7] + P[19][2] * SF[9] +
                   P[19][3] * SF[8] + P[19][10] * SF[11] + P[19][11] * SPP[7] +
                   P[19][12] * SPP[6];
    nextP[19][1] = P[19][1] + P[19][0] * SF[6] + P[19][2] * SF[5] +
                   P[19][3] * SF[9] + P[19][11] * SPP[6] - P[19][12] * SPP[7] -
                   (P[19][10] * q0) / 2;
    nextP[19][2] = P[19][2] + P[19][0] * SF[4] + P[19][1] * SF[8] +
                   P[19][3] * SF[6] + P[19][12] * SF[11] - P[19][10] * SPP[6] -
                   (P[19][11] * q0) / 2;
    nextP[19][3] = P[19][3] + P[19][0] * SF[5] + P[19][1] * SF[4] +
                   P[19][2] * SF[7] - P[19][11] * SF[11] + P[19][10] * SPP[7] -
                   (P[19][12] * q0) / 2;
    nextP[19][4] = P[19][4] + P[19][1] * SF[1] + P[19][0] * SF[3] +
                   P[19][2] * SPP[0] - P[19][3] * SPP[2] - P[19][13] * SPP[4];
    nextP[19][5] = P[19][5] + P[19][0] * SF[2] + P[19][2] * SF[1] +
                   P[19][3] * SF[3] - P[19][1] * SPP[0] + P[19][13] * SPP[3];
    nextP[19][6] = P[19][6] + P[19][1] * SF[2] + P[19][3] * SF[1] +
                   P[19][0] * SPP[0] - P[19][2] * SPP[1] - P[19][13] * SPP[5];
    nextP[19][7] = P[19][7] + P[19][4] * dt;
    nextP[19][8] = P[19][8] + P[19][5] * dt;
    nextP[19][9] = P[19][9] + P[19][6] * dt;
    nextP[19][10] = P[19][10];
    nextP[19][11] = P[19][11];
    nextP[19][12] = P[19][12];
    nextP[19][13] = P[19][13];
    nextP[19][14] = P[19][14];
    nextP[19][15] = P[19][15];
    nextP[19][16] = P[19][16];
    nextP[19][17] = P[19][17];
    nextP[19][18] = P[19][18];
    nextP[19][19] = P[19][19];
    nextP[19][20] = P[19][20];
    nextP[19][21] = P[19][21];
    nextP[20][0] = P[20][0] + P[20][1] * SF[7] + P[20][2] * SF[9] +
                   P[20][3] * SF[8] + P[20][10] * SF[11] + P[20][11] * SPP[7] +
                   P[20][12] * SPP[6];
    nextP[20][1] = P[20][1] + P[20][0] * SF[6] + P[20][2] * SF[5] +
                   P[20][3] * SF[9] + P[20][11] * SPP[6] - P[20][12] * SPP[7] -
                   (P[20][10] * q0) / 2;
    nextP[20][2] = P[20][2] + P[20][0] * SF[4] + P[20][1] * SF[8] +
                   P[20][3] * SF[6] + P[20][12] * SF[11] - P[20][10] * SPP[6] -
                   (P[20][11] * q0) / 2;
    nextP[20][3] = P[20][3] + P[20][0] * SF[5] + P[20][1] * SF[4] +
                   P[20][2] * SF[7] - P[20][11] * SF[11] + P[20][10] * SPP[7] -
                   (P[20][12] * q0) / 2;
    nextP[20][4] = P[20][4] + P[20][1] * SF[1] + P[20][0] * SF[3] +
                   P[20][2] * SPP[0] - P[20][3] * SPP[2] - P[20][13] * SPP[4];
    nextP[20][5] = P[20][5] + P[20][0] * SF[2] + P[20][2] * SF[1] +
                   P[20][3] * SF[3] - P[20][1] * SPP[0] + P[20][13] * SPP[3];
    nextP[20][6] = P[20][6] + P[20][1] * SF[2] + P[20][3] * SF[1] +
                   P[20][0] * SPP[0] - P[20][2] * SPP[1] - P[20][13] * SPP[5];
    nextP[20][7] = P[20][7] + P[20][4] * dt;
    nextP[20][8] = P[20][8] + P[20][5] * dt;
    nextP[20][9] = P[20][9] + P[20][6] * dt;
    nextP[20][10] = P[20][10];
    nextP[20][11] = P[20][11];
    nextP[20][12] = P[20][12];
    nextP[20][13] = P[20][13];
    nextP[20][14] = P[20][14];
    nextP[20][15] = P[20][15];
    nextP[20][16] = P[20][16];
    nextP[20][17] = P[20][17];
    nextP[20][18] = P[20][18];
    nextP[20][19] = P[20][19];
    nextP[20][20] = P[20][20];
    nextP[20][21] = P[20][21];
    nextP[21][0] = P[21][0] + P[21][1] * SF[7] + P[21][2] * SF[9] +
                   P[21][3] * SF[8] + P[21][10] * SF[11] + P[21][11] * SPP[7] +
                   P[21][12] * SPP[6];
    nextP[21][1] = P[21][1] + P[21][0] * SF[6] + P[21][2] * SF[5] +
                   P[21][3] * SF[9] + P[21][11] * SPP[6] - P[21][12] * SPP[7] -
                   (P[21][10] * q0) / 2;
    nextP[21][2] = P[21][2] + P[21][0] * SF[4] + P[21][1] * SF[8] +
                   P[21][3] * SF[6] + P[21][12] * SF[11] - P[21][10] * SPP[6] -
                   (P[21][11] * q0) / 2;
    nextP[21][3] = P[21][3] + P[21][0] * SF[5] + P[21][1] * SF[4] +
                   P[21][2] * SF[7] - P[21][11] * SF[11] + P[21][10] * SPP[7] -
                   (P[21][12] * q0) / 2;
    nextP[21][4] = P[21][4] + P[21][1] * SF[1] + P[21][0] * SF[3] +
                   P[21][2] * SPP[0] - P[21][3] * SPP[2] - P[21][13] * SPP[4];
    nextP[21][5] = P[21][5] + P[21][0] * SF[2] + P[21][2] * SF[1] +
                   P[21][3] * SF[3] - P[21][1] * SPP[0] + P[21][13] * SPP[3];
    nextP[21][6] = P[21][6] + P[21][1] * SF[2] + P[21][3] * SF[1] +
                   P[21][0] * SPP[0] - P[21][2] * SPP[1] - P[21][13] * SPP[5];
    nextP[21][7] = P[21][7] + P[21][4] * dt;
    nextP[21][8] = P[21][8] + P[21][5] * dt;
    nextP[21][9] = P[21][9] + P[21][6] * dt;
    nextP[21][10] = P[21][10];
    nextP[21][11] = P[21][11];
    nextP[21][12] = P[21][12];
    nextP[21][13] = P[21][13];
    nextP[21][14] = P[21][14];
    nextP[21][15] = P[21][15];
    nextP[21][16] = P[21][16];
    nextP[21][17] = P[21][17];
    nextP[21][18] = P[21][18];
    nextP[21][19] = P[21][19];
    nextP[21][20] = P[21][20];
    nextP[21][21] = P[21][21];

    // add the general state process noise variances
    for (uint8_t i = 0; i <= 21; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods without GPS
    if ((P[7][7] + P[8][8]) > 1e4f)
    {
        for (uint8_t i = 7; i <= 8; i++)
        {
            for (uint8_t j = 0; j <= 21; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output and fix numerical errors
    ekf_CopyAndFixCovariances();

    // constrain diagonals to prevent ill-conditioning
    ekf_ConstrainVariances();

    // set the flag to indicate that covariance prediction has been performed and reset the increments used by the covariance prediction
    covPredStep = true;
    vectorZero(&summedDelAng);
    vectorZero(&summedDelVel);
    dt = 0.0f;
}

// select fusion of magnetometer data
void ekf_SelectMagFusion(void)
{
    // check for and read new magnetometer measurements
    ekf_readMagData();

    // If we are using the compass and the magnetometer has been unhealthy
    // for too long we declare a timeout If we have a vehicle that can fly
    // without a compass (a vehicle that doesn't have significant sideslip)
    // then the compass is permanently failed and will not be used until the
    // filter is reset
    if (magHealth)
    {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    }
    else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > magFailTimeLimit_ms && ekf_useCompass())
    {
        magTimeout = true;
    }

    // determine if conditions are right to start a new fusion cycle
    bool dataReady = statesInitialised && ekf_useCompass() && newDataMag;

    if (dataReady)
    {
        // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
        memset(&magIncrStateDelta[0], 0, sizeof(magIncrStateDelta));
        magUpdateCount = 0;

        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep)
        {
            ekf_CovariancePrediction();
        }

        // fuse the XYZ magnetometer componenents sequentially
        for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++)
        {
            ekf_FuseMagnetometer();
        }
    }

    // Fuse corrections to quaternion, position and velocity states across several time steps to reduce 10Hz pulsing in the output
    if (magUpdateCount < magUpdateCountMax)
    {
        magUpdateCount++;
        for (uint8_t i = 0; i <= 9; i++)
        {
            ekfStates.statesArray[i] += magIncrStateDelta[i];
        }
    }
}

// fuse selected position, velocity and height measurements, checking dat for
// consistency provide a vehicleArmed that allows maintenance of the attitude
// reference without GPS provided the vehicle is not accelerating check
// innovation consistency of velocity states calculated using IMU1 and IMU2 and
// calculate the optimal weighting of accel data
void ekf_FuseVelPosNED(void)
{
    // health is set bad until test passed
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    fpVector3_t velInnov;
    fpVector3_t velInnov1;
    fpVector3_t velInnov2;
    fpVector2_t posInnov;
    float hgtInnov = 0.0f;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false, false, false, false, false, false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    float posErr;
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 observation;
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        // if vehicleArmed is false use the current states to calculate the
        // predicted measurement rather than use states from a previous time. We
        // need to do this because there may be no stored states due to lack of
        // real measurements. in vehicle disarmed, only position and height
        // fusion is used
        if (!vehicleArmed)
        {
            statesAtPosTime = ekfStates.stateStruct;
            statesAtHgtTime = ekfStates.stateStruct;
        }

        // set the GPS data timeout depending on whether airspeed data is present
        timeMs_t gpsRetryTime = ekf_useAirspeed() ? 20000 : 10000;

        // form the observation vector and zero velocity and horizontal position observations if in vehicle disarmed
        if (vehicleArmed)
        {
            observation[0] = velNED.x;
            observation[1] = velNED.y;
            observation[2] = velNED.z;
            observation[3] = gpsPosNE.x;
            observation[4] = gpsPosNE.y;
        }
        else
        {
            for (uint8_t i = 0; i <= 4; i++)
            {
                observation[i] = 0.0f;
            }
        }

        observation[5] = hgtMeas;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        R_OBS[0] = sq(constrainf(ekfParam.gpsHorizVelNoise, 0.05f, 5.0f)) + sq(gpsNEVelVarAccScale * accNavMag);
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = sq(constrainf(ekfParam.gpsVertVelNoise, 0.05f, 5.0f)) + sq(gpsDVelVarAccScale * accNavMag);
        R_OBS[3] = sq(constrainf(ekfParam.gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(constrainf(ekfParam.baroAltNoise, 0.1f, 10.0f));

        // if vertical GPS velocity data is being used, check to see if the GPS
        // vertical velocity and barometer innovations have the same sign and
        // are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer
        // innovation consistency checks.
        if (fuseVelData && (imuSampleTime_ms - lastHgtTime_ms) < (2 * msecHgtAvg))
        {
            // calculate innovations for height and vertical GPS vel measurements
            float hgtErr = statesAtHgtTime.position.z - observation[5];
            float velDErr = statesAtVelTime.velocity.z - observation[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            if ((hgtErr * velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[9][9] + R_OBS[5])) && (sq(velDErr) > 9.0f * (P[6][6] + R_OBS[2])))
            {
                badIMUdata = true;
            }
            else
            {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation
        // consistency check test position measurements
        if (fusePosData)
        {
            // test horizontal position measurements
            posInnov.v[X] = statesAtPosTime.position.x - observation[3];
            posInnov.v[Y] = statesAtPosTime.position.y - observation[4];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply an innovation consistency threshold test, but don't fail if
            // bad IMU data calculate max valid position innovation squared
            // based on a maximum horizontal inertial nav accel error and GPS
            // noise parameter max inertial nav error is scaled with horizontal
            // g to allow for increased errors when manoeuvring
            float accelScale = (1.0f + 0.1f * accNavMag);
            float maxPosInnov2 = sq(ekfParam.gpsPosInnovGate * ekfParam.gpsHorizPosNoise + 0.005f * accelScale * ekfParam.gpsGlitchAccelMax * sq((float)MS2S(imuSampleTime_ms - lastPosPassTime_ms)));
            posTestRatio = (sq(posInnov.v[X]) + sq(posInnov.v[Y])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // declare a timeout condition if we have been too long without data
            posTimeout = ((imuSampleTime_ms - lastPosPassTime_ms) > gpsRetryTime);
            // use position data if healthy, timed out, or in vehicle disarmed
            if (posHealth || posTimeout || !vehicleArmed)
            {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
                // if timed out or outside the specified glitch radius, reset to the GPS position
                if (posTimeout || (maxPosInnov2 > sq(ekfParam.gpsGlitchRadiusMax)))
                {
                    // reset the position to the current GPS position
                    ekf_ResetPosition();
                    // reset the velocity to the GPS velocity
                    ekf_ResetVelocity();
                    // don't fuse data on this time step
                    fusePosData = false;
                }
            }
            else
            {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData)
        {
            // test velocity measurements
            uint8_t imax = 2;
            float K1 = 0;            // innovation to error ratio for IMU1
            float K2 = 0;            // innovation to error ratio for IMU2
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0;     // sum of velocity innovation variances
            for (uint8_t i = 0; i <= imax; i++)
            {
                // velocity states start at index 4
                stateIndex = i + 4;

                // calculate innovations using blended and single IMU predicted states
                velInnov.v[i] = statesAtVelTime.velocity.v[i] - observation[i]; // blended
                velInnov1.v[i] = statesAtVelTime.vel1.v[i] - observation[i];    // IMU1
                velInnov2.v[i] = statesAtVelTime.vel2.v[i] - observation[i];    // IMU2

                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];

                // calculate error weightings for single IMU velocity states using observation error to normalise
                float R_hgt;

                if (i == 2)
                {
                    R_hgt = sq(constrainf(ekfParam.gpsVertVelNoise, 0.05f, 5.0f));
                }
                else
                {
                    R_hgt = sq(constrainf(ekfParam.gpsHorizVelNoise, 0.05f, 5.0f));
                }

                K1 += R_hgt / (R_hgt + sq(velInnov1.v[i]));
                K2 += R_hgt / (R_hgt + sq(velInnov2.v[i]));

                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov.v[i]);
                varVelSum += varInnovVelPos[i];
            }

            // calculate weighting used by fuseVelPosNED to do IMU accel data blending
            // this is used to detect and compensate for aliasing errors with the accelerometers
            // provide for a first order lowpass filter to reduce noise on the weighting if required
            // set weighting to 0.5 when on ground to allow more rapid learning of bias errors without 'ringing' in bias estimates
            // NOTE: this weighting can be overwritten in UpdateStrapdownEquationsNED
            if (vehicleArmed)
            {
                IMU1_weighting = 1.0f * (K1 / (K1 + K2)) + 0.0f * IMU1_weighting; // filter currently inactive
            }
            else
            {
                IMU1_weighting = 0.5f;
            }

            // apply an innovation consistency threshold test, but don't fail if bad IMU data calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(ekfParam.gpsVelInnovGate));

            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f) || badIMUdata);

            // declare a timeout if we have not fused velocity data for too long
            velTimeout = (imuSampleTime_ms - lastVelPassTime_ms) > gpsRetryTime;

            // if data is healthy  or in vehicle disarmed we fuse it
            if (velHealth || !vehicleArmed)
            {
                velHealth = true;
                lastVelPassTime_ms = imuSampleTime_ms;
            }
            else if (velTimeout && !posHealth)
            {
                // if data is not healthy and timed out and position is
                // unhealthy we reset the velocity, but do not fuse data on this time step
                ekf_ResetVelocity();
                ekf_StoreStatesReset();
                fuseVelData = false;
            }
            else
            {
                // if data is unhealthy and position is healthy, we do not fuse it
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData)
        {
            // calculate height innovations
            hgtInnov = statesAtHgtTime.position.z - observation[5];
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(hgtInnov) / (sq(ekfParam.hgtInnovGate) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            hgtTimeout = (imuSampleTime_ms - lastHgtPassTime_ms) > hgtRetryTime_ms;
            // fuse height data if healthy
            // force a reset if timed out to prevent the possibility of inertial errors causing persistent loss of height reference
            // force fusion in constant position mode on the ground to allow large accelerometer biases to be learned without rejecting barometer
            if (hgtHealth || hgtTimeout || !vehicleArmed)
            {
                // calculate a filtered value to be used by pre-flight health checks
                // we need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (!vehicleArmed)
                {
                    float dtBaro = MS2S(imuSampleTime_ms - lastHgtPassTime_ms);
                    const float hgtInnovFiltTC = 2.0f;
                    float alpha = constrainf(dtBaro / (dtBaro + hgtInnovFiltTC), 0.0f, 1.0f);
                    hgtInnovFiltState += (innovVelPos[5] - hgtInnovFiltState) * alpha;
                }
                // declare height healthy and able to be fused
                lastHgtPassTime_ms = imuSampleTime_ms;
                hgtHealth = true;
                // if timed out, reset the height, but do not fuse data on this time step
                if (hgtTimeout)
                {
                    ekf_ResetHeight();
                    fuseHgtData = false;
                }
            }
            else
            {
                hgtHealth = false;
            }
        }

        // set range for sequential fusion of velocity and position measurements
        // depending on which data is available and its health
        if (fuseVelData && velHealth && vehicleArmed)
        {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }

        if ((fusePosData && posHealth) || !vehicleArmed)
        {
            fuseData[3] = true;
            fuseData[4] = true;
        }

        if ((fuseHgtData && hgtHealth) || !vehicleArmed)
        {
            fuseData[5] = true;
        }

        // fuse measurements sequentially
        for (obsIndex = 0; obsIndex <= 5; obsIndex++)
        {
            if (fuseData[obsIndex])
            {
                stateIndex = 4 + obsIndex;
                // calculate the measurement innovation, using states from a
                // different time coordinate if fusing height data adjust
                // scaling on GPS measurement noise variances if not enough satellites
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = statesAtVelTime.velocity.v[obsIndex] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4)
                {
                    innovVelPos[obsIndex] = statesAtPosTime.position.v[obsIndex - 3] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else
                {
                    innovVelPos[obsIndex] = statesAtHgtTime.position.v[obsIndex - 3] - observation[obsIndex];
                }

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f / varInnovVelPos[obsIndex];
                for (uint8_t i = 0; i <= 12; i++)
                {
                    Kfusion[i] = P[i][stateIndex] * SK;
                }

                // Only height observations are used to update z accel bias
                // estimate Protect Kalman gain from ill-conditioning Don't
                // update Z accel bias if off-level by greater than 60 degrees
                // to avoid scale factor error effects
                if (obsIndex == 5 && prevTnb.m[2][2] > 0.5f)
                {
                    Kfusion[13] = constrainf(P[13][stateIndex] * SK, -1.0f, 0.0f);
                }
                else
                {
                    Kfusion[13] = 0.0f;
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates)
                {
                    Kfusion[14] = P[14][stateIndex] * SK;
                    Kfusion[15] = P[15][stateIndex] * SK;
                }
                else
                {
                    Kfusion[14] = 0.0f;
                    Kfusion[15] = 0.0f;
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                if (!inhibitMagStates)
                {
                    for (uint8_t i = 16; i <= 21; i++)
                    {
                        Kfusion[i] = P[i][stateIndex] * SK;
                    }
                }
                else
                {
                    for (uint8_t i = 16; i <= 21; i++)
                    {
                        Kfusion[i] = 0.0f;
                    }
                }

                // Set the Kalman gain values for the single IMU states
                Kfusion[26] = Kfusion[9]; // IMU1 posD
                Kfusion[30] = Kfusion[9]; // IMU2 posD

                for (uint8_t i = 0; i <= 2; i++)
                {
                    Kfusion[i + 23] = Kfusion[i + 4]; // IMU1 velNED
                    Kfusion[i + 27] = Kfusion[i + 4]; // IMU2 velNED
                }

                // Don't update Z accel bias values for an acceleraometer we have hard switched away from
                if ((IMU1_weighting >= 0.1f) && (IMU1_weighting <= 0.9f))
                {
                    // both IMU's OK
                    Kfusion[22] = Kfusion[13];
                }
                else if (IMU1_weighting < 0.1f)
                {
                    // IMU1 bad
                    Kfusion[22] = Kfusion[13];
                    Kfusion[13] = 0.0f;
                }
                else
                {
                    // IMU2 bad
                    Kfusion[22] = 0.0f;
                }

                // Correct states that have been predicted using single (not blended) IMU data
                if (obsIndex == 5)
                {
                    // Calculate height measurement innovations using single IMU states
                    float hgtInnov1 = statesAtHgtTime.posD1 - observation[obsIndex];
                    float hgtInnov2 = statesAtHgtTime.posD2 - observation[obsIndex];

                    if (vehicleArmed)
                    {
                        // Correct single IMU prediction states using height measurement, limiting rate of change of bias to 0.005 m/s3
                        float correctionLimit = 0.005f * dtIMU * 0.2f;
                        ekfStates.stateStruct.accel_zbias1 -= constrainf(Kfusion[13] * hgtInnov1, -correctionLimit, correctionLimit); // IMU1 Z accel bias
                        ekfStates.stateStruct.accel_zbias2 -= constrainf(Kfusion[22] * hgtInnov2, -correctionLimit, correctionLimit); // IMU2 Z accel bias
                    }
                    else
                    {
                        // When disarmed, do not rate limit accel bias learning
                        ekfStates.stateStruct.accel_zbias1 -= Kfusion[13] * hgtInnov1; // IMU1 Z accel bias
                        ekfStates.stateStruct.accel_zbias2 -= Kfusion[22] * hgtInnov2; // IMU2 Z accel bias
                    }
                    for (uint8_t i = 23; i <= 26; i++)
                    {
                        ekfStates.statesArray[i] = ekfStates.statesArray[i] - Kfusion[i] * hgtInnov1; // IMU1 velNED, posD
                    }
                    for (uint8_t i = 27; i <= 30; i++)
                    {
                        ekfStates.statesArray[i] = ekfStates.statesArray[i] - Kfusion[i] * hgtInnov2; // IMU2 velNED, posD
                    }
                }
                else if (obsIndex == 0 || obsIndex == 1 || obsIndex == 2)
                {
                    // Correct single IMU prediction states using velocity measurements
                    for (uint8_t i = 23; i <= 26; i++)
                    {
                        ekfStates.statesArray[i] = ekfStates.statesArray[i] - Kfusion[i] * velInnov1.v[obsIndex]; // IMU1 velNED, posD
                    }
                    for (uint8_t i = 27; i <= 30; i++)
                    {
                        ekfStates.statesArray[i] = ekfStates.statesArray[i] - Kfusion[i] * velInnov2.v[obsIndex]; // IMU2 velNED, posD
                    }
                }

                // calculate state corrections and re-normalise the quaternions
                // for states predicted using the blended IMU data attitude,
                // velocity and position corrections are spread across multiple
                // prediction cycles between now and the anticipated time for
                // the next measurement. Don't spread quaternion corrections if
                // total angle change across predicted interval is going to exceed 0.1 rad
                bool highRates = ((gpsUpdateCountMax * calc_length_pythagorean_3D(correctedDelAng.x, correctedDelAng.y, correctedDelAng.z)) > 0.1f);
                for (uint8_t i = 0; i <= 21; i++)
                {
                    if ((i <= 3 && highRates) || i >= 10 || !vehicleArmed)
                    {
                        ekfStates.statesArray[i] = ekfStates.statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }
                    else
                    {
                        if (obsIndex == 5)
                        {
                            hgtIncrStateDelta[i] -= Kfusion[i] * innovVelPos[obsIndex] * hgtUpdateCountMaxInv;
                        }
                        else
                        {
                            gpsIncrStateDelta[i] -= Kfusion[i] * innovVelPos[obsIndex] * gpsUpdateCountMaxInv;
                        }
                    }
                }

                quaternionNormalize(&ekfStates.stateStruct.quat, &ekfStates.stateStruct.quat);

                // update the covariance - take advantage of direct observation
                // of a single state at index = stateIndex to reduce
                // computations this is a numerically optimised implementation
                // of standard equation P = (I - K * H) * P;
                for (uint8_t i = 0; i <= 21; i++)
                {
                    for (uint8_t j = 0; j <= 21; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i = 0; i <= 21; i++)
                {
                    for (uint8_t j = 0; j <= 21; j++)
                    {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
    ekf_ForceSymmetry();
    ekf_ConstrainVariances();
}

// select fusion of velocity, position and height measurements
void ekf_SelectVelPosFusion(void)
{
    // check for and read new height data
    ekf_readHgtData();

    // If we haven't received height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = !velTimeout ? 10000 : 5000;
    if (imuSampleTime_ms - (timeMs_t)US2MS(lastHgtMeasTime_us) > hgtRetryTime_ms)
    {
        hgtTimeout = true;
    }

    // command fusion of height data
    if (newDataHgt)
    {
        // reset data arrived flag
        newDataHgt = false;
        // reset state updates and counter used to spread fusion updates across
        // several frames to reduce 10Hz pulsing
        memset(&hgtIncrStateDelta[0], 0, sizeof(hgtIncrStateDelta));
        hgtUpdateCount = 0;
        // enable fusion
        fuseHgtData = true;
    }
    else
    {
        fuseHgtData = false;
    }

    // check for and read new GPS data
    ekf_readGpsData();

    // check for new data, specify which measurements should be used and check data for freshness
    if (vehicleArmed)
    {
        // command fusion of GPS data and reset states as required
        if (newDataGps)
        {
            // reset data arrived flag
            newDataGps = false;
            // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
            memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
            gpsUpdateCount = 0;
            // enable fusion
            fuseVelData = true;
            fusePosData = true;
            // If a long time since last GPS update, then reset position and velocity and reset stored state history
            timeMs_t gpsRetryTimeout = ekf_useAirspeed() ? 20000 : 10000;
            if (imuSampleTime_ms - secondLastFixTime_ms > gpsRetryTimeout)
            {
                ekf_ResetPosition();
                ekf_ResetVelocity();
                ekf_StoreStatesReset();
            }
        }
        else
        {
            fuseVelData = false;
            fusePosData = false;
        }
    }
    else
    {
        // in vehicle disarmed use synthetic position measurements set to zero
        // only fuse synthetic measurements when rate of change of velocity is
        // less than 0.5g to reduce attitude errors due to launch acceleration
        // do not use velocity fusion to reduce the effect of movement on attitude
        if (accNavMag < 4.9f)
        {
            fusePosData = true;
        }
        else
        {
            fusePosData = false;
        }

        fuseVelData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep)
        {
            ekf_CovariancePrediction();
        }
        ekf_FuseVelPosNED();
    }

    // Fuse corrections to quaternion, position and velocity states across several time steps to reduce 5 and 10Hz pulsing in the output
    if (gpsUpdateCount < gpsUpdateCountMax)
    {
        gpsUpdateCount++;
        for (uint8_t i = 0; i <= 9; i++)
        {
            ekfStates.statesArray[i] += gpsIncrStateDelta[i];
        }
    }

    if (hgtUpdateCount < hgtUpdateCountMax)
    {
        hgtUpdateCount++;
        for (uint8_t i = 0; i <= 9; i++)
        {
            ekfStates.statesArray[i] += hgtIncrStateDelta[i];
        }
    }
}

// select fusion of true airspeed measurements
void ekf_SelectIASFusion(void)
{
    // get true airspeed measurement
    ekf_readAirSpdData();

    // If we haven't received airspeed data for a while, then declare the airspeed data as being timed out
    if (imuSampleTime_ms - lastAirspeedUpdate_ms > iasRetryTime)
    {
        iasTimeout = true;
    }

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform IAS fusion
    iasDataWaiting = (statesInitialised && !inhibitWindStates && newDataIAS);
    if (iasDataWaiting)
    {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep)
        {
            ekf_CovariancePrediction();
        }
        ekf_FuseAirspeed();
        iasDataWaiting = false;
        newDataIAS = false;
    }
}

// select fusion of synthetic sideslip measurements
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// it requires a stable wind for best results and should not be used for aerobatic flight with manoeuvres that induce large sidslip angles (eg knife-edge, spins, etc)
void ekf_SelectBetaFusion(void)
{
    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - BETAmsecPrev) >= msecBetaAvg);

    // set true when use of synthetic sideslip fusion is necessary because we have limited sensor data or are dead reckoning position
    bool f_required = !(ekf_useCompass() && ekf_useAirspeed() && posHealth);

    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    bool f_feasible = (STATE(FIXED_WING_LEGACY) && !inhibitWindStates);

    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_feasible && f_required && f_timeTrigger)
    {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep)
        {
            ekf_CovariancePrediction();
        }
        ekf_FuseSideslip();
        BETAmsecPrev = imuSampleTime_ms;
    }
}

// This function is used to do a forced alignment of the wind velocity
// states so that they are set to the reciprocal of the ground speed
// and scaled to STARTUP_WIND_SPEED m/s. This is used when launching a
// fly-forward vehicle without an airspeed sensor on the assumption
// that launch will be into wind and STARTUP_WIND_SPEED is
// representative of typical launch wind
void ekf_setWindVelStates(void)
{
    float gndSpd = calc_length_pythagorean_2D(ekfStates.stateStruct.velocity.x, ekfStates.stateStruct.velocity.y);
    if (gndSpd > 4.0f)
    {
        // set the wind states to be the reciprocal of the velocity and scale
        float scaleFactor = STARTUP_WIND_SPEED / gndSpd;
        ekfStates.stateStruct.wind_vel.x = -ekfStates.stateStruct.velocity.x * scaleFactor;
        ekfStates.stateStruct.wind_vel.y = -ekfStates.stateStruct.velocity.y * scaleFactor;
        // reinitialise the wind state covariances
        ekf_zeroRows(P, 14, 15);
        ekf_zeroCols(P, 14, 15);
        P[14][14] = 64.0f;
        P[15][15] = P[14][14];
    }
}

// update the quaternion, velocity and position states using IMU measurements
void ekf_UpdateStrapdownEquationsNED(void)
{
    fpVector3_t delVelNav;  // delta velocity vector calculated using a blend of IMU1 and IMU2 data
    fpVector3_t delVelNav1; // delta velocity vector calculated using IMU1 data
    fpVector3_t delVelNav2; // delta velocity vector calculated using IMU2 data

    // remove sensor bias errors
    correctedDelAng.x = dAngIMU.x - ekfStates.stateStruct.gyro_bias.x;
    correctedDelAng.y = dAngIMU.y - ekfStates.stateStruct.gyro_bias.y;
    correctedDelAng.z = dAngIMU.z - ekfStates.stateStruct.gyro_bias.z;
    correctedDelVel1 = dVelIMU1;
    correctedDelVel2 = dVelIMU2;
    correctedDelVel1.z -= ekfStates.stateStruct.accel_zbias1;
    correctedDelVel2.z -= ekfStates.stateStruct.accel_zbias2;

    // use weighted average of both IMU units for delta velocities
    correctedDelVel12.x = correctedDelVel1.x * IMU1_weighting + correctedDelVel2.x * (1.0f - IMU1_weighting);
    correctedDelVel12.y = correctedDelVel1.y * IMU1_weighting + correctedDelVel2.y * (1.0f - IMU1_weighting);
    correctedDelVel12.z = correctedDelVel1.z * IMU1_weighting + correctedDelVel2.z * (1.0f - IMU1_weighting);

    // apply correction for earths rotation rate
    const fpVector3_t earthNED = multiplyMatrixByVector(prevTnb, earthRateNED);
    correctedDelAng.x = correctedDelAng.x - earthNED.x * dtIMU;
    correctedDelAng.y = correctedDelAng.y - earthNED.y * dtIMU;
    correctedDelAng.z = correctedDelAng.z - earthNED.z * dtIMU;

    // convert the rotation vector to its equivalent quaternion
    QuaternionFromAxisAngle(correctedDelAng, &correctedDelAngQuat);

    // update the quaternion states by rotating from the previous attitude through the delta angle rotation quaternion and normalise
    quaternionMultiply(&ekfStates.stateStruct.quat, &ekfStates.stateStruct.quat, &correctedDelAngQuat);
    quaternionNormalize(&ekfStates.stateStruct.quat, &ekfStates.stateStruct.quat);

    // calculate the body to nav cosine matrix
    fpMatrix3_t Tbn_temp;
    quaternionToRotationMatrix(ekfStates.stateStruct.quat, &Tbn_temp);
    prevTnb = matrixTransposed(Tbn_temp);

    // calculate earth frame delta velocity due to gravity
    float delVelGravity1_z = -GRAVITY_MSS * dtIMU;
    float delVelGravity2_z = -GRAVITY_MSS * dtIMU;
    float delVelGravity_z = delVelGravity1_z * IMU1_weighting + delVelGravity2_z * (1.0f - IMU1_weighting);

    // blended IMU calc
    delVelNav = multiplyMatrixByVector(Tbn_temp, correctedDelVel12);
    delVelNav.z += delVelGravity_z;

    // IMU1
    delVelNav1 = multiplyMatrixByVector(Tbn_temp, correctedDelVel1);
    delVelNav1.z += delVelGravity1_z;

    // IMU2
    delVelNav2 = multiplyMatrixByVector(Tbn_temp, correctedDelVel2);
    delVelNav2.z += delVelGravity2_z;

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED.x = delVelNav.x / dtIMU;
    velDotNED.y = delVelNav.y / dtIMU;
    velDotNED.z = delVelNav.z / dtIMU;

    // apply a first order lowpass filter
    velDotNEDfilt.x = velDotNED.x * 0.05f + velDotNEDfilt.x * 0.95f;
    velDotNEDfilt.y = velDotNED.y * 0.05f + velDotNEDfilt.y * 0.95f;
    velDotNEDfilt.z = velDotNED.z * 0.05f + velDotNEDfilt.z * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = calc_length_pythagorean_3D(velDotNEDfilt.x, velDotNEDfilt.y, velDotNEDfilt.z);
    accNavMagHoriz = calc_length_pythagorean_2D(velDotNEDfilt.x, velDotNEDfilt.y);

    // save velocity for use in trapezoidal intergration for position calcuation
    const fpVector3_t lastVelocity = {.v = {ekfStates.stateStruct.velocity.x, ekfStates.stateStruct.velocity.y, ekfStates.stateStruct.velocity.z}};
    const fpVector3_t lastVel1 = {.v = {ekfStates.stateStruct.vel1.x, ekfStates.stateStruct.vel1.y, ekfStates.stateStruct.vel1.z}};
    const fpVector3_t lastVel2 = {.v = {ekfStates.stateStruct.vel2.x, ekfStates.stateStruct.vel2.y, ekfStates.stateStruct.vel2.z}};

    // sum delta velocities to get velocity
    ekfStates.stateStruct.velocity.x += delVelNav.x;
    ekfStates.stateStruct.velocity.y += delVelNav.y;
    ekfStates.stateStruct.velocity.z += delVelNav.z;
    ekfStates.stateStruct.vel1.x += delVelNav1.x;
    ekfStates.stateStruct.vel1.y += delVelNav1.y;
    ekfStates.stateStruct.vel1.z += delVelNav1.z;
    ekfStates.stateStruct.vel2.x += delVelNav2.x;
    ekfStates.stateStruct.vel2.y += delVelNav2.y;
    ekfStates.stateStruct.vel2.z += delVelNav2.z;

    // apply a trapezoidal integration to velocities to calculate position
    ekfStates.stateStruct.position.x += (ekfStates.stateStruct.velocity.x + lastVelocity.x) * (dtIMU * 0.5f);
    ekfStates.stateStruct.position.y += (ekfStates.stateStruct.velocity.y + lastVelocity.y) * (dtIMU * 0.5f);
    ekfStates.stateStruct.position.z += (ekfStates.stateStruct.velocity.z + lastVelocity.z) * (dtIMU * 0.5f);
    ekfStates.stateStruct.posD1 += (ekfStates.stateStruct.vel1.z + lastVel1.z) * (dtIMU * 0.5f);
    ekfStates.stateStruct.posD2 += (ekfStates.stateStruct.vel2.z + lastVel2.z) * (dtIMU * 0.5f);

    // capture current angular rate to augmented state vector for use by optical flow fusion
    const float omega_z = correctedDelAng.z / dtIMU;

    // LPF the yaw rate using a 1 second time constant yaw rate and determine if we are doing continual
    // fast rotations that can cause problems due to gyro scale factor errors.
    float alphaLPF = constrainf(dtIMU, 0.0f, 1.0f);
    yawRateFilt += (omega_z - yawRateFilt) * alphaLPF;
    if (fabsf(yawRateFilt) > 1.0f)
    {
        highYawRate = true;
    }
    else
    {
        highYawRate = false;
    }

    // limit states to protect against divergence
    ekf_ConstrainStates();
}

// this function is used to do a forced alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void ekf_alignYawGPS(void)
{
    if ((sq(velNED.v[X]) + sq(velNED.v[Y])) > 16.0f)
    {
        float roll;
        float pitch;
        float oldYaw;
        float newYaw;
        float yawErr;

        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        quaternionToEuler(ekfStates.stateStruct.quat, &roll, &pitch, &oldYaw);

        // calculate course yaw angle
        oldYaw = atan2_approx(ekfStates.stateStruct.velocity.y, ekfStates.stateStruct.velocity.x);

        // calculate yaw angle from GPS velocity
        newYaw = atan2_approx(velNED.v[Y], velNED.v[X]);

        // estimate the yaw error
        yawErr = wrap_PI(newYaw - oldYaw);

        // If the inertial course angle disagrees with the GPS by more than 45 degrees, we declare the compass as bad
        badMag = (fabsf(yawErr) > 0.7854f);

        if (badMag || !yawAligned)
        {
            // correct the yaw angle
            newYaw = oldYaw + yawErr;

            // calculate new filter quaternion states from Euler angles
            quaternionFromEuler(&ekfStates.stateStruct.quat, roll, pitch, newYaw);

            // the yaw angle is now aligned so update its status
            yawAligned = true;

            // reset the position and velocity states
            ekf_ResetPosition();
            ekf_ResetVelocity();

            // reset the covariance for the quaternion, velocity and position states
            // zero the matrix entries
            ekf_zeroRows(P, 0, 9);
            ekf_zeroCols(P, 0, 9);

            // quaternions - TODO maths that sets them based on different roll, yaw and pitch uncertainties
            P[0][0] = 1.0e-9f;
            P[1][1] = 0.25f * sq(DEGREES_TO_RADIANS(1.0f));
            P[2][2] = 0.25f * sq(DEGREES_TO_RADIANS(1.0f));
            P[3][3] = 0.25f * sq(DEGREES_TO_RADIANS(1.0f));

            // velocities - we could have a big error coming out of constant position mode due to GPS lag
            P[4][4] = 400.0f;
            P[5][5] = P[4][4];
            P[6][6] = sq(0.7f);

            // positions - we could have a big error coming out of constant position mode due to GPS lag
            P[7][7] = 400.0f;
            P[8][8] = P[7][7];
            P[9][9] = sq(5.0f);
        }

        // Update magnetic field states if the magnetometer is bad
        if (badMag)
        {
            fpVector3_t euler;
            quaternionToEuler(ekfStates.stateStruct.quat, &euler.x, &euler.y, &euler.z);
            ekf_calcQuatAndFieldStates(euler.x, euler.y);
        }
    }
}

// calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
void ekf_OnGroundCheck(void)
{
    // determine if the vehicle is manoevring
    if (accNavMagHoriz > 0.5f)
    {
        manoeuvring = true;
    }
    else
    {
        manoeuvring = false;
    }

    // if we are a fly forward type vehicle, then in-air mode can be determined through a combination of speed and height criteria
    if (STATE(FIXED_WING_LEGACY))
    {
        // Evaluate a numerical score that defines the likelihood we are in the air
        float gndSpdSq = sq(velNED.v[X]) + sq(velNED.v[Y]);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 10 m/s airspeed
        if (ekf_useAirspeed())
        {
            if (CENTIMETERS_TO_METERS(getAirspeedEstimate()) > 10.0f)
            {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f)
        {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(hgtMeas) > 10.0f)
        {
            largeHgtChange = true;
        }

        // to go to in-air mode we also need enough GPS velocity to be able to calculate a reliable ground track heading and either a lerge height or airspeed change
        if (onGround && highGndSpd && (highAirSpd || largeHgtChange))
        {
            onGround = false;
        }

        // if is possible we are in flight, set the time this condition was last detected
        if (highGndSpd || highAirSpd || largeHgtChange)
        {
            airborneDetectTime_ms = imuSampleTime_ms;
        }

        // after 5 seconds of not detecting a possible flight condition, we transition to on-ground mode
        if (!onGround && ((imuSampleTime_ms - airborneDetectTime_ms) > 5000))
        {
            onGround = true;
        }

        // perform a yaw alignment check against GPS if exiting on-ground mode, bu tonly if we have enough ground speed
        // this is done to protect against unrecoverable heading alignment errors due to compass faults
        if (!onGround && prevOnGround)
        {
            ekf_alignYawGPS();
        }

        // If we aren't using an airspeed sensor we set the wind velocity to the reciprocal
        // of the velocity vector and scale states so that the wind speed is equal to 3m/s. This helps prevent gains
        // being too high at the start of flight if launching into a headwind until the first turn when the EKF can form
        // a wind speed estimate and also corrects bad initial wind estimates due to heading errors
        if (!onGround && prevOnGround && !ekf_useAirspeed())
        {
            ekf_setWindVelStates();
        }
    }

    // store current on-ground status for next time
    prevOnGround = onGround;

    // If we are on ground, or don't have the right vehicle and sensing to estimate wind, inhibit wind states
    inhibitWindStates = ((!ekf_useAirspeed() && !STATE(FIXED_WING_LEGACY)) || onGround);

    // request mag calibration for both in-air and manoeuvre threshold options
    bool magCalRequested = (STATE(FIXED_WING_LEGACY) && !onGround) || (!STATE(FIXED_WING_LEGACY) && manoeuvring);

    // inhibit the magnetic field calibration if not requested or denied
    inhibitMagStates = (!magCalRequested || !ekf_useCompass());
}

// Update Filter States - this should be called whenever new IMU data is available
void ekf_UpdateFilter(void)
{
    // zero the delta quaternion used by the strapdown navigation because it is published
    // and we need to return a zero rotation of the INS fails to update it
    quaternionInitUnit(&correctedDelAngQuat);

    // don't run filter updates if states have not been initialised
    if (!statesInitialised)
    {
        return;
    }

    // read IMU data and convert to delta angles and velocities
    ekf_readIMUData();

    // detect if the filter update has been delayed for too long
    if (dtIMU > 0.2f)
    {
        // we have stalled for too long - reset states
        ekf_ResetVelocity();
        ekf_ResetPosition();
        ekf_ResetHeight();
        ekf_StoreStatesReset();
        // Initialise IMU pre-processing states
        ekf_readIMUData();
        return;
    }

    // check if on ground
    ekf_OnGroundCheck();

    vehicleArmed = ARMING_FLAG(ARMED);

    // check to see if vehicleArmed has changed and reset states
    if (vehicleArmed != prevVehicleArmed)
    {
        if (vehicleArmed)
        {
            // only reset the magnetic field and heading on the first arm.
            if (!firstArmComplete)
            {
                ekfStates.stateStruct.quat = ekf_calcQuatAndFieldStates(DECIDEGREES_TO_RADIANS(attitude.values.roll), DECIDEGREES_TO_RADIANS(attitude.values.pitch));
                firstArmComplete = true;
            }

            // save the gyro bias so that the in-flight gyro bias state limits can be adjusted to provide the same amount of offset change in either direction
            delAngBiasAtArming = ekfStates.stateStruct.gyro_bias;

            // Reset filter position to GPS when transitioning into flight mode
            // We need to do this becasue the vehicle may have moved since the EKF origin was set
            ekf_ResetPosition();
            ekf_StoreStatesReset();
        }
        else
        {
            // Reset all position and velocity states when transitioning out of flight mode
            // We need to do this becasue we are going into a mode that assumes zero position and velocity
            ekf_ResetVelocity();
            ekf_ResetPosition();
            ekf_StoreStatesReset();
            earthRateNEDReset = true;
        }

        prevVehicleArmed = vehicleArmed;
    }

    // run the strapdown INS equations every IMU update
    ekf_UpdateStrapdownEquationsNED();

    // store the predicted states for subsequent use by measurement fusion
    ekf_StoreStates();

    // sum delta angles and time used by covariance prediction
    summedDelAng.x += correctedDelAng.x;
    summedDelAng.y += correctedDelAng.y;
    summedDelAng.z += correctedDelAng.z;
    summedDelVel.x += correctedDelVel12.x;
    summedDelVel.y += correctedDelVel12.y;
    summedDelVel.z += correctedDelVel12.z;
    dt += dtIMU;

    // perform a covariance prediction if the total delta angle has exceeded the limit or the time limit will be exceeded at the next IMU update
    if (((dt >= (covTimeStepMax - dtIMU)) || (calc_length_pythagorean_3D(summedDelAng.x, summedDelAng.y, summedDelAng.z) > covDelAngMax)))
    {
        ekf_CovariancePrediction();
    }
    else
    {
        covPredStep = false;
    }

    // Update states using GPS, barometer, compass, airspeed and sideslip observations
    ekf_SelectVelPosFusion();
    ekf_SelectMagFusion();
    ekf_SelectIASFusion();
    ekf_SelectBetaFusion();
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
fpQuaternion_t ekf_getQuaternion(void)
{
    return ekfStates.stateStruct.quat;
}

// return the transformation matrix from XYZ (body) to NED axes
void ekf_getRotationBodyToNED(fpMatrix3_t *mat)
{
    quaternionToRotationMatrix(ekfStates.stateStruct.quat, mat);
}

// return the Euler roll, pitch and yaw angle in radians
void ekf_getEulerAngles(fpVector3_t *euler)
{
    quaternionToEuler(ekfStates.stateStruct.quat, &euler->x, &euler->y, &euler->z);
    euler->z = -euler->z;
}

// return the individual Z-accel bias estimates in m/s^2
void ekf_getAccelZBias(float *zbias1, float *zbias2)
{
    if (dtIMU > 0)
    {
        *zbias1 = ekfStates.stateStruct.accel_zbias1 / dtIMU;
        *zbias2 = ekfStates.stateStruct.accel_zbias2 / dtIMU;
    }
    else
    {
        *zbias1 = 0.0f;
        *zbias2 = 0.0f;
    }
}

// blended accelerometer values in the earth frame in m/s/s
fpVector3_t ekf_getAccelEarthFrameBlended(void)
{
    fpVector3_t accel_ef_ekf;
    fpVector3_t accel = dVelIMU1;
    float abias1, abias2;

    ekf_getAccelZBias(&abias1, &abias2);

    accel.z -= abias1;
    accel_ef_ekf = multiplyMatrixByVector(ahrsMatrix, accel);

    return accel_ef_ekf;
}

// return body axis gyro bias estimates in rad/sec
void ekf_getGyroBias(fpVector3_t *gyroBias)
{
    if (dtIMU < 1e-6f)
    {
        vectorZero(gyroBias);
        return;
    }

    gyroBias->x = ekfStates.stateStruct.gyro_bias.x / dtIMU;
    gyroBias->y = ekfStates.stateStruct.gyro_bias.y / dtIMU;
    gyroBias->z = ekfStates.stateStruct.gyro_bias.z / dtIMU;
}

// return NEU position in cm/s
void ekf_getPosNEU(fpVector3_t *pos)
{
    pos->x = METERS_TO_CENTIMETERS(ekfStates.stateStruct.position.x);
    pos->y = METERS_TO_CENTIMETERS(ekfStates.stateStruct.position.y);
    pos->z = METERS_TO_CENTIMETERS(ekfStates.stateStruct.position.z);
}

// return NEU velocity in cm/s
void ekf_getVelNEU(fpVector3_t *vel)
{
    vel->x = METERS_TO_CENTIMETERS(ekfStates.stateStruct.velocity.x);
    vel->y = METERS_TO_CENTIMETERS(ekfStates.stateStruct.velocity.y);
    vel->z = METERS_TO_CENTIMETERS(ekfStates.stateStruct.velocity.z);
}

// return the NED wind speed estimates in cm/s (positive is air moving in the direction of the axis)
void ekf_getWind(fpVector3_t *wind)
{
    wind->x = METERS_TO_CENTIMETERS(ekfStates.stateStruct.wind_vel.x);
    wind->y = METERS_TO_CENTIMETERS(ekfStates.stateStruct.wind_vel.y);
    wind->z = 0.0f; // currently don't estimate this
}

void ekf_Update(float deltaTime)
{
    if (forceEKFDisable() || !ekfConfig()->ekfEnabled)
    {
        return;
    }

    // LPF to filter the spike
    dtIMU = 0.98f * dtIMU + 0.02f * deltaTime;

    if (!ekf_started)
    {
        if (ekfStartTime_ms == 0)
        {
            ekfStartTime_ms = millis();
        }
        ekf_started = ekf_InitialiseFilterDynamic();
    }

    if (ekf_started)
    {
        ekf_UpdateFilter();

        if (ekf_HealthyToUse())
        {
            fpVector3_t ekf_eulers;
            fpVector3_t ekf_gyro_bias;
            fpVector3_t ekf_wind_speed;

            // ekf_getRotationBodyToNED(&ahrsMatrix);
            ekf_getEulerAngles(&ekf_eulers);
            ekf_getGyroBias(&ekf_gyro_bias);
            ekf_getPosNEU(&ekfPosVel.pos);
            ekf_getVelNEU(&ekfPosVel.vel);
            ekf_getWind(&ekf_wind_speed);

            /*attitude.values.roll = RADIANS_TO_DECIDEGREES(ekf_eulers.x);
            attitude.values.pitch = RADIANS_TO_DECIDEGREES(ekf_eulers.y);
            attitude.values.yaw = RADIANS_TO_DECIDEGREES(ekf_eulers.z);

            if (attitude.values.yaw < 0)
            {
                attitude.values.yaw += 3600;
            }*/

            int16_t ekf_eulers_z = RADIANS_TO_DECIDEGREES(ekf_eulers.z);

            if (ekf_eulers_z < 0)
            {
                ekf_eulers_z += 3600;
            }

            float velVar;
            float posVar;
            float hgtVar;
            float magVar;
            float tasVar;

            ekf_getVariances(&velVar, &posVar, &hgtVar, &magVar, &tasVar);

            // float ekfGroundSpeed = calc_length_pythagorean_2D(ekf_wind_speed.x, ekf_wind_speed.y);

            /*DEBUG_SET(DEBUG_EKF, 0, velVar * 100.0f);
            DEBUG_SET(DEBUG_EKF, 1, posVar * 100.0f);
            DEBUG_SET(DEBUG_EKF, 2, hgtVar * 100.0f);
            DEBUG_SET(DEBUG_EKF, 3, magVar * 100.0f);
            DEBUG_SET(DEBUG_EKF, 4, tasVar * 100.0f);*/

            DEBUG_SET(DEBUG_EKF, 0, ekfPosVel.pos.x);
            DEBUG_SET(DEBUG_EKF, 1, ekfPosVel.pos.y);
            DEBUG_SET(DEBUG_EKF, 2, ekfPosVel.pos.z);
            DEBUG_SET(DEBUG_EKF, 3, ekfPosVel.vel.x);
            DEBUG_SET(DEBUG_EKF, 4, ekfPosVel.vel.y);
            DEBUG_SET(DEBUG_EKF, 5, ekfPosVel.vel.z);
            DEBUG_SET(DEBUG_EKF, 6, ekf_eulers_z / 10);
            DEBUG_SET(DEBUG_EKF, 7, magVar * 100.0f);

            /*DEBUG_SET(DEBUG_EKF, 0, ekf_gyro_bias.x);
            DEBUG_SET(DEBUG_EKF, 1, ekf_gyro_bias.y);
            DEBUG_SET(DEBUG_EKF, 2, ekf_gyro_bias.z);
            DEBUG_SET(DEBUG_EKF, 3, ekfPosVel.pos.z);*/

            // gyro.gyroADCf[X] -= RADIANS_TO_DEGREES(ekf_gyro_bias.x);
            // gyro.gyroADCf[Y] -= RADIANS_TO_DEGREES(ekf_gyro_bias.y);
            // gyro.gyroADCf[Z] -= RADIANS_TO_DEGREES(ekf_gyro_bias.z);
        }
    }
}