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

#pragma GCC optimize("O3")

#include "navigation/ekf.h"

#include "build/debug.h"
#include "common/quaternion.h"
#include "common/vector.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "io/gps.h"
#include "io/gps_private.h"
#include "navigation/navigation.h"
#include "navigation/navigation_pos_estimator_private.h"
#include "navigation/navigation_private.h"
#include "platform.h"
#include "scheduler/scheduler.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "fc/config.h"

typedef union
{
    float v[2];
    struct
    {
        float x, y;
    };
} fpVector2_t;

float states[31];         // EKF of 31 states
typedef struct
{
    fpQuaternion_t quat;         // 0..3
    fpVector3_t velocity;        // 4..6
    fpVector3_t position;        // 7..9
    fpVector3_t gyro_bias;       // 10..12
    float accel_zbias1;          // 13
    fpVector2_t wind_vel;        // 14..15
    fpVector3_t earth_magfield;  // 16..18
    fpVector3_t body_magfield;   // 19..21
    float accel_zbias2;          // 22
    fpVector3_t vel1;            // 23 .. 25
    float posD1;                 // 26
    fpVector3_t vel2;            // 27 .. 29
    float posD2;                 // 30
} state_elements_t;

typedef struct
{
    bool diverged : 1;
    bool large_covarience : 1;
    bool bad_xmag : 1;
    bool bad_ymag : 1;
    bool bad_zmag : 1;
    bool bad_airspeed : 1;
    bool bad_sideslip : 1;
} fault_status_t;

#define IGNORE_COMPASS_EKF

// earth rotation rate (rad/sec)
#define EARTHRATE 0.000072921f

// maximum value for any element in the covariance matrix
#define EKF_COVARIENCE_MAX 1.0e8f

// initial gyro bias uncertainty (deg/sec)
#define INIT_GYRO_BIAS_UNCERTAINTY 0.1f

fault_status_t faultStatus;

state_elements_t *state = (state_elements_t *)states; // Casting of the 31 states
state_elements_t
    storedStates[50];  // state vectors stored for the last 50 time steps
state_elements_t
    statesAtVelTime;  // States at the effective time of velNED measurements
state_elements_t
    statesAtPosTime;  // States at the effective time of posNE measurements
state_elements_t
    statesAtHgtTime;  // States at the effective time of hgtMea measurement

gpsOrigin_t ekfOrigin;

fpVector3_t summedDelAng;  // corrected & summed delta angles about the xyz body
                           // axes (rad)
fpVector3_t summedDelVel;  // corrected & summed delta velocities along the XYZ
                           // body axes (m/s)
fpVector3_t correctedDelAng;  // delta angles about the xyz body axes corrected
// for errors (rad)
fpVector3_t
    prevDelAng;  // previous delta angle use for INS coning error compensation
fpVector3_t
    correctedDelVel12;  // delta velocities along the XYZ body axes for weighted
                        // average of IMU1 and IMU2 corrected for errors (m/s)
fpVector3_t correctedDelVel1;  // delta velocities along the XYZ body axes for
// IMU1 corrected for errors (m/s)
fpVector3_t correctedDelVel2;  // delta velocities along the XYZ body axes for
                               // IMU2 corrected for errors (m/s)
fpVector3_t
    dAngIMU;  // delta angle vector in XYZ body axes measured by the IMU (rad)
fpVector3_t lastAngRate;  // angular rate from previous IMU sample used for
                          // trapezoidal integrator
fpVector3_t
    lastGyroBias;  // previous gyro bias vector used by filter divergence check
fpVector3_t
    dVelIMU1;  // delta velocity vector in XYZ body axes measured by IMU1 (m/s)
fpVector3_t
    dVelIMU2;  // delta velocity vector in XYZ body axes measured by IMU2 (m/s)
fpVector3_t lastAccel1;  // acceleration from previous IMU1 sample used for
                         // trapezoidal integrator
fpVector3_t lastAccel2;  // acceleration from previous IMU2 sample used for
                         // trapezoidal integrator
fpVector2_t
    gpsPosGlitchOffsetNE;   // offset applied to GPS data in the NE direction to
                            // compensate for rapid changes in GPS solution
fpVector3_t gpsPosNE;       // North, East position measurements (m)
fpVector3_t velNED;         // North, East, Down velocity measurements (m/s)
fpVector3_t velDotNED;      // rate of change of velocity in NED frame
fpVector3_t velDotNEDfilt;  // low pass filtered velDotNED
fpVector3_t earthRateNED;   // earths angular rate vector in NED (rad/s)

fpMat3_t prevTnb;  // previous nav to body transformation used for INS earth
                   // rotation compensation

float Kfusion[31];    // Kalman gain vector
float P[22][22];      // covariance matrix
float nextP[22][22];  // Predicted covariance matrix before addition of process
                      // noise to diagonals
float KHP[22][22];    // intermediate result used for covariance updates
float SF[15];  // intermediate variables used to calculate predicted covariance
               // matrix
float SG[8];   // intermediate variables used to calculate predicted covariance
               // matrix
float SQ[11];  // intermediate variables used to calculate predicted covariance
               // matrix
float SPP[8];  // intermediate variables used to calculate predicted covariance
               // matrix
float processNoise[22];   // process noise added to diagonals of predicted
                          // covariance matrix
float varInnovVelPos[6];  // innovation variance output for a group of
                          // measurements
float innovVelPos[6];     // innovation output for a group of measurements

bool ekf_started;
bool
    statesInitialised;  // boolean true when filter states have been initialised
bool filterDiverged;    // boolean true if the filter has diverged
bool staticMode = true;  // boolean to force position and velocity measurements
                         // to zero for pre-arm or bench testing
bool prevStaticMode = true;  // value of static mode from last update
bool yawAligned;             // true when the yaw angle has been aligned
bool newDataHgt;             // true when new height data has arrived
bool newDataGps;             // true when new GPS data has arrived
bool fuseVelData;  // this boolean causes the velNED measurements to be fused
bool fusePosData;  // this boolean causes the posNE measurements to be fused
bool fuseHgtData;  // this boolean causes the hgtMea measurements to be fused
bool velHealth;  // boolean true if velocity measurements have passed innovation
                 // consistency check
bool posHealth;  // boolean true if position measurements have passed innovation
                 // consistency check
bool hgtHealth;  // boolean true if height measurements have passed innovation
                 // consistency check
bool velTimeout;  // boolean true if velocity measurements have failed
                  // innovation consistency check and timed out
bool posTimeout;  // boolean true if position measurements have failed
                  // innovation consistency check and timed out
bool hgtTimeout;  // boolean true if height measurements have failed innovation
                  // consistency check and timed out
bool magFailed;   // boolean true if the magnetometer has failed
bool magTimeout;  // boolean true if magnetometer measurements have failed for
                  // too long and have timed out
bool onGround;    // boolean true when the flight vehicle is on the ground (not
                  // flying)
bool prevOnGround;  // value of onGround from previous update
bool inhibitWindStates =
    true;  // true when wind states and covariances are to remain constant
bool inhibitMagStates = true;  // true when magnetic field states and
                               // covariances are to remain constant

uint8_t magUpdateCount;  // count of the number of minor state corrections using
                         // Magnetometer data
uint8_t magUpdateCountMax;  // limit on the number of minor state corrections
                            // using Magnetometer data
uint8_t gpsUpdateCount;  // count of the number of minor state corrections using
                         // GPS data
uint8_t gpsUpdateCountMax;  // limit on the number of minor state corrections
                            // using GPS data
uint8_t hgtUpdateCount;  // count of the number of minor state corrections using
                         // Baro data
uint8_t hgtUpdateCountMax;  // limit on the number of minor state corrections
                            // using Baro data
uint8_t storeIndex;         // State vector storage index

float dt;              // time lapsed since the last covariance prediction (sec)
float dtIMU;           // time lapsed since the last IMU measurement (sec)
float accNavMag;       // magnitude of navigation accel - used to adjust GPS obs
                       // variance (m/s^2)
float accNavMagHoriz;  // magnitude of navigation accel in horizontal plane
                       // (m/s^2)
float hgtMea;          //  height measurement relative to reference point  (m)
float hgtRate;         // state for rate of change of height filter
float gyroBiasNoiseScaler = 2.0f;  // scale factor applied to gyro bias state
                                   // process noise when on ground
float dtVelPos =
    0.02f;  // average of msec between position and velocity corrections
float IMU1_weighting =
    0.5f;  // Weighting applied to use of IMU1. Varies between 0 and 1.
float _gpsNEVelVarAccScale =
    0.05f;  // Scale factor applied to NE velocity measurement variance due to
            // manoeuvre acceleration
float _gpsDVelVarAccScale =
    0.07f;  // Scale factor applied to vertical velocity measurement variance
            // due to manoeuvre acceleration
float _gpsPosVarAccScale =
    0.05f;  // Scale factor applied to horizontal position measurement variance
            // due to manoeuvre acceleration
const float covTimeStepMax =
    0.07f;  // maximum time allowed between covariance predictions
const float covDelAngMax =
    0.05f;           // maximum delta angle between covariance predictions
float velTestRatio;  // sum of squares of GPS velocity innovation divided by
                     // fail threshold
float posTestRatio;  // sum of squares of GPS position innovation divided by
                     // fail threshold
float hgtTestRatio;  // sum of squares of baro height innovation divided by fail
                     // threshold
// Used by smoothing of state corrections
float gpsIncrStateDelta[10];  // vector of corrections to attitude, velocity and
                              // position to be applied over the period between
                              // the current and next GPS measurement
float hgtIncrStateDelta[10];  // vector of corrections to attitude, velocity and
// position to be applied over the period between
// the current and next height measurement
float magIncrStateDelta[10];  // vector of corrections to attitude, velocity and
                              // position to be applied over the period between
                              // the current and next magnetometer measurement
float magUpdateCountMaxInv;   // floating point inverse of magFilterCountMax
float gpsUpdateCountMaxInv;   // floating point inverse of gpsFilterCountMax
float hgtUpdateCountMaxInv;   // floating point inverse of hgtFilterCountMax
float gpsNoiseScaler = 1.0f;         // Used to scale the  GPS measurement noise and
// consistency gates to compensate for operation with
// small satellite counts
float scaledDeltaGyrBiasLgth;  // scaled delta gyro bias vector length used to
                               // test for filter divergence

int16_t _msecHgtDelay =
    60;  // effective average delay of height measurements rel to (msec)
int16_t _hgtRetryTimeMode0 =
    10000;  // Height retry time with vertical velocity measurement (msec)

uint16_t _msecHgtAvg =
    100;  // average number of msec between height measurements
uint16_t _msecMagAvg =
    100;  // average number of msec between magnetometer measurements
uint16_t _msecGpsAvg = 200;  // average number of msec between GPS measurements

uint32_t statetimeStamp[50];     // time stamp for each state vector stored
uint32_t imuSampleTime_ms;       // time that the last IMU value was taken
uint32_t lastStateStoreTime_ms;  // time of last state vector storage
uint32_t lastHgtTime_ms;  // time of last height update (msec) used to calculate
                          // timeout
uint32_t lastHgtMeasTime;  // time of last height measurement used to determine
                           // if new data has arrived
uint32_t velFailTime;  // time stamp when GPS velocity measurement last failed
                       // covaraiance consistency check (msec)
uint32_t posFailTime;  // time stamp when GPS position measurement last failed
                       // covaraiance consistency check (msec)
uint32_t hgtFailTime;  // time stamp when height measurement last failed
// covaraiance consistency check (msec)
uint32_t lastMagUpdate;  // last time compass was updated
uint32_t
    lastHealthyMagTime_ms;  // time the magnetometer was last declared healthy
uint32_t lastFixTime_ms;  // time of last GPS fix used to determine if new data
                          // has arrived
uint32_t secondLastFixTime_ms;  // time of second last GPS fix used to determine
                                // how long since last update
uint32_t lastDecayTime_ms;      // time of last decay of GPS position offset
uint32_t lastDivergeTime_ms;  // time in msec divergence of filter last detected
uint32_t start_time_ms;

bool _ekf_use = true;  // convert to param

// @Description: This noise controls the growth of estimated error due to gyro
// measurement errors excluding bias. Increasing it makes the flter trust the
// gyro measurements less and other measurements more. (Param in rad/s)
// @Range: 0.001 0.05
// @Increment: 0.001
float _gyrNoise = GYRO_PNOISE_DEFAULT;

// @Description: This noise controls the growth of estimated error due to
// accelerometer measurement errors excluding bias. Increasing it makes the
// flter trust the accelerometer measurements less and other measurements more.
// (Param in m/s^2)
// @Range: 0.05 1.0
// @Increment: 0.01
float _accNoise = ACC_PNOISE_DEFAULT;

// @Description: This noise controls the growth of gyro bias state error
// estimates. Increasing it makes rate gyro bias estimation faster and noisier.
// (Param in rad/s)
// @Range: 0.0000001 0.00001
float _gyroBiasProcessNoise = GBIAS_PNOISE_DEFAULT;

// @Description: This noise controls the growth of the vertical acelerometer
// bias state error estimate. Increasing it makes accelerometer bias estimation
// faster and noisier. (Param in m/s^2)
// @Range: 0.00001 0.001
float _accelBiasProcessNoise = ABIAS_PNOISE_DEFAULT;

// @Description: This noise controls the growth of earth magnetic field state
// error estimates. Increasing it makes earth magnetic field bias estimation
// faster and noisier. (Param in gauss/s)
// @Range: 0.0001 0.01
float _magEarthProcessNoise = MAGE_PNOISE_DEFAULT;

// @Description: This noise controls the growth of body magnetic field state
// error estimates. Increasing it makes compass offset estimation faster and
// noisier. (Param in gauss/s)
// @Range: 0.0001 0.01
float _magBodyProcessNoise = MAGB_PNOISE_DEFAULT;

// @Description: This parameter sets the number of standard deviations applied
// to the height measurement innovation consistency check. Decreasing it makes
// it more likely that good measurements will be rejected. Increasing it makes
// it more likely that bad measurements will be accepted.
// @Range: 1 100
// @Increment: 1
float _hgtInnovGate = HGT_GATE_DEFAULT;

// @Description: This is the scaler that is applied to the speed accuracy
// reported by the receiver to estimate the horizontal velocity observation
// noise. If the model of receiver used does not provide a speed accurcy
// estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the
// weighting on these measurements.
// @Range: 0.05 5.0
// @Increment: 0.05
float _gpsHorizVelNoise = VELNE_NOISE_DEFAULT;

// @Description: This is the scaler that is applied to the speed accuracy
// reported by the receiver to estimate the vertical velocity observation noise.
// If the model of receiver used does not provide a speed accurcy estimate, then
// a speed acuracy of 1 is assumed. Increasing it reduces the weighting on this
// measurement.
// @Range: 0.05 5.0
// @Increment: 0.05
float _gpsVertVelNoise = VELD_NOISE_DEFAULT;

// @Description: This is the RMS value of noise in the GPS horizontal position
// measurements. Increasing it reduces the weighting on these measurements.
// (Param in meters)
// @Range: 0.1 10.0
// @Increment: 0.1
float _gpsHorizPosNoise = POSNE_NOISE_DEFAULT;

// @Description: This is the RMS value of noise in the altitude measurement.
// Increasing it reduces the weighting on this measurement. (Param in meters)
// @Range: 0.1 10.0
// @Increment: 0.1
float _baroAltNoise = ALT_NOISE_DEFAULT;

// @Description: This parameter sets the number of standard deviations applied
// to the GPS position measurement innovation consistency check. Decreasing it
// makes it more likely that good measurements will be rejected. Increasing it
// makes it more likely that bad measurements will be accepted.
// @Range: 1 100
// @Increment: 1
float _gpsPosInnovGate = POS_GATE_DEFAULT;

// @Description: This parameter controls the maximum amount of difference in
// horizontal acceleration between the value predicted by the filter and the
// value measured by the GPS before the GPS position data is rejected. If this
// value is set too low, then valid GPS data will be regularly discarded, and
// the position accuracy will degrade. If this parameter is set too high, then
// large GPS glitches will cause large rapid changes in position. (Param in
// cm/s^2)
// @Range: 100 500
// @Increment: 50
float _gpsGlitchAccelMax = GLITCH_ACCEL_DEFAULT;

// @Description: This parameter controls the maximum amount of difference in
// horizontal position (in m) between the value predicted by the filter and the
// value measured by the GPS before the long term glitch protection logic is
// activated and an offset is applied to the GPS measurement to compensate.
// Position steps smaller than this value will be temporarily ignored, but will
// then be accepted and the filter will move to the new position. Position steps
// larger than this value will be ignored initially, but the filter will then
// apply an offset to the GPS position measurement. (Param in meters)
// @Range: 10 50
// @Increment: 5
float _gpsGlitchRadiusMax = GLITCH_RADIUS_DEFAULT;

// @Description: This parameter sets the number of standard deviations applied
// to the GPS velocity measurement innovation consistency check. Decreasing it
// makes it more likely that good measurements willbe rejected. Increasing it
// makes it more likely that bad measurements will be accepted.
// @Range: 1 100
// @Increment: 1
float _gpsVelInnovGate = VEL_GATE_DEFAULT;

// @Description: This is the number of msec that the GPS velocity measurements
// lag behind the inertial measurements. (Param in msec)
// @Range: 0 500
// @Increment: 10
float _msecVelDelay = 220;

// @Description: This is the number of msec that the GPS position measurements
// lag behind the inertial measurements. (Param in msec)
// @Range: 0 500
// @Increment: 10
float _msecPosDelay = 220;

// @Description: This noise controls the growth of wind state error estimates.
// Increasing it makes wind estimation faster and noisier. (Param in m/s^2)
// @Range: 0.01 1.0
// @Increment: 0.1
float _windVelProcessNoise = 0.1f;

// @Description: Increasing this parameter increases how rapidly the wind states
// adapt when changing altitude, but does make wind speed estimation noiser.
// @Range: 0.0 1.0
// @Increment: 0.1
float _wndVarHgtRateScale = 0.5f;

// return true if we should use the airspeed sensor
bool ekf_useAirspeed(void)
{
    // pitot tube enabled?
    return false;
}

// return true if we should use the magnetometer sensor
bool ekf_use_compass(void)
{
    // magnetometer enabled?
    return false;
}

static bool is_zero(const float x)
{
    // zero check
    return fabsf(x) < 1.19209290e-7F;
}

void quaternion_normalize(fpQuaternion_t *q)
{
    const float quatMag = sqrtf(sq(q->q0) + sq(q->q1) + sq(q->q2) + sq(q->q3));

    if (!is_zero(quatMag)) {
        const float quatMagInv = 1.0f / quatMag;
        q->q0 *= quatMagInv;
        q->q1 *= quatMagInv;
        q->q2 *= quatMagInv;
        q->q3 *= quatMagInv;
    }
}

// populate the supplied rotation matrix equivalent from this quaternion
void quaternion_to_rotation_matrix(fpQuaternion_t q, fpMat3_t *m)
{
    const float q3q3 = q.q2 * q.q2;
    const float q3q4 = q.q2 * q.q3;
    const float q2q2 = q.q1 * q.q1;
    const float q2q3 = q.q1 * q.q2;
    const float q2q4 = q.q1 * q.q3;
    const float q1q2 = q.q0 * q.q1;
    const float q1q3 = q.q0 * q.q2;
    const float q1q4 = q.q0 * q.q3;
    const float q4q4 = q.q3 * q.q3;

    m->m[0][0] = 1.0f - 2.0f * (q3q3 + q4q4);
    m->m[0][1] = 2.0f * (q2q3 - q1q4);
    m->m[0][2] = 2.0f * (q2q4 + q1q3);
    m->m[1][0] = 2.0f * (q2q3 + q1q4);
    m->m[1][1] = 1.0f - 2.0f * (q2q2 + q4q4);
    m->m[1][2] = 2.0f * (q3q4 - q1q2);
    m->m[2][0] = 2.0f * (q2q4 - q1q3);
    m->m[2][1] = 2.0f * (q3q4 + q1q2);
    m->m[2][2] = 1.0f - 2.0f * (q2q2 + q3q3);
}

// create a quaternion from Euler angles
void quaternion_from_euler(fpQuaternion_t *q, float roll, float pitch,
                           float yaw)
{
    const float cr2 = cos(roll * 0.5f);
    const float cp2 = cos(pitch * 0.5f);
    const float cy2 = cos(yaw * 0.5f);
    const float sr2 = sin(roll * 0.5f);
    const float sp2 = sin(pitch * 0.5f);
    const float sy2 = sin(yaw * 0.5f);

    q->q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q->q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q->q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q->q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
}

void quaternion_to_euler(fpQuaternion_t q, float *roll, float *pitch,
                         float *yaw)
{
    *roll = atan2f(2.0f * (q.q0 * q.q1 + q.q2 * q.q3),
                   1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2));
    *pitch = asin(2.0f * (q.q0 * q.q2 - q.q3 * q.q1));
    *yaw = -atan2f(2.0f * (q.q0 * q.q3 + q.q1 * q.q2),
                  1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3));
}

// vector cross product
fpVector3_t get_vector_cross_product(const fpVector3_t v1, const fpVector3_t v2)
{
    fpVector3_t temp;

    temp.x = v1.y * v2.z - v1.z * v2.y;
    temp.y = v1.z * v2.x - v1.x * v2.z;
    temp.z = v1.x * v2.y - v1.y * v2.x;

    return temp;
}

// matrix multiplication by a vector
fpVector3_t multiply_matrix_by_vector(fpMat3_t m, fpVector3_t v)
{
    fpVector3_t vRet;

    vRet.x = m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z;
    vRet.y = m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z;
    vRet.z = m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z;

    return vRet;
}

void matrix_from_euler(fpMat3_t *m, float roll, float pitch, float yaw)
{
    const float cp = cos(pitch);
    const float sp = sin(pitch);
    const float sr = sin(roll);
    const float cr = cos(roll);
    const float sy = sin(yaw);
    const float cy = cos(yaw);

    m->m[0][0] = cp * cy;
    m->m[0][1] = (sr * sp * cy) - (cr * sy);
    m->m[0][2] = (cr * sp * cy) + (sr * sy);
    m->m[1][0] = cp * sy;
    m->m[1][1] = (sr * sp * sy) + (cr * cy);
    m->m[1][2] = (cr * sp * sy) - (sr * cy);
    m->m[2][0] = -sp;
    m->m[2][1] = sr * cp;
    m->m[2][2] = cr * cp;
}

fpMat3_t matrix_transposed(const fpMat3_t matrix) {
    fpMat3_t result = {
        {
            {matrix.m[0][0], matrix.m[1][0], matrix.m[2][0]},
            {matrix.m[0][1], matrix.m[1][1], matrix.m[2][1]},
            {matrix.m[0][2], matrix.m[1][2], matrix.m[2][2]}
        }
    };

    return result;
}


// recall state vector stored at closest time to the one specified by msec
void ekf_RecallStates(state_elements_t *statesForFusion, uint32_t msec)
{
    uint32_t timeDelta;
    uint32_t bestTimeDelta = 200;
    uint8_t bestStoreIndex = 0;

    for (uint8_t i = 0; i <= 49; i++) {
        timeDelta = msec - statetimeStamp[i];
        if (timeDelta < bestTimeDelta) {
            bestStoreIndex = i;
            bestTimeDelta = timeDelta;
        }
    }

    if (bestTimeDelta <
        200)  // only output stored state if < 200 msec retrieval error
    {
        *statesForFusion = storedStates[bestStoreIndex];
    } else  // otherwise output current state
    {
        statesForFusion = state;
    }
}

// zero specified range of rows in the state covariance matrix
void ekf_zeroRows(float covMat[22][22], uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = first; row <= last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0]) * 22);
    }
}

// zero specified range of columns in the state covariance matrix
void ekf_zeroCols(float covMat[22][22], uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = 0; row <= 21; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0]) * (1 + last - first));
    }
}

// update IMU delta angle and delta velocity measurements
void ekf_readIMUData(void)
{
    fpVector3_t angRate;  // angular rate vector in XYZ body axes measured by
                          // the IMU (rad/s)
    fpVector3_t accel1;   // acceleration vector in XYZ body axes measured by
                          // IMU1 (m/s^2)
    fpVector3_t accel2;   // acceleration vector in XYZ body axes measured by
                          // IMU2 (m/s^2)

    // the imu sample time is sued as a common time reference throughout the
    // filter
    imuSampleTime_ms = millis();

    accGetMeasuredAcceleration(
        &accel1);  // Calculate accel in body frame in cm/s
    gyroGetMeasuredRotationRate(
        &angRate);  // Calculate gyro rate in body frame in rad/s

    accel1.x = CENTIMETERS_TO_METERS(accel1.x);
    accel1.y = CENTIMETERS_TO_METERS(accel1.y);
    accel1.z = CENTIMETERS_TO_METERS(accel1.z);
    accel2 = accel1;

    // trapezoidal integration
    dAngIMU.x = (angRate.x + lastAngRate.x) * dtIMU * 0.5f;
    dAngIMU.y = (angRate.y + lastAngRate.y) * dtIMU * 0.5f;
    dAngIMU.z = (angRate.z + lastAngRate.z) * dtIMU * 0.5f;
    lastAngRate.x = angRate.x;
    lastAngRate.y = angRate.y;
    lastAngRate.z = angRate.z;
    dVelIMU1.x = (accel1.x + lastAccel1.x) * dtIMU * 0.5f;
    dVelIMU1.y = (accel1.y + lastAccel1.y) * dtIMU * 0.5f;
    dVelIMU1.z = (accel1.z + lastAccel1.z) * dtIMU * 0.5f;
    lastAccel1.x = accel1.x;
    lastAccel1.y = accel1.y;
    lastAccel1.z = accel1.z;
    dVelIMU2.x = (accel2.x + lastAccel2.x) * dtIMU * 0.5f;
    dVelIMU2.y = (accel2.y + lastAccel2.y) * dtIMU * 0.5f;
    dVelIMU2.z = (accel2.z + lastAccel2.x) * dtIMU * 0.5f;
    lastAccel2.x = accel2.x;
    lastAccel2.y = accel2.y;
    lastAccel2.z = accel2.z;
}

// check for new altitude measurement data and update stored measurement if
// available
void ekf_readHgtData(void)
{
    // check to see if baro measurement has changed so we know if a new
    // measurement has arrived
    if (baroIsHealthy() &&
        posEstimator.baro.lastUpdateTime != lastHgtMeasTime) {
        // time stamp used to check for new measurement
        lastHgtMeasTime = posEstimator.baro.lastUpdateTime;

        // time stamp used to check for timeout
        lastHgtTime_ms = imuSampleTime_ms;

        // get measurement and set flag to let other functions know new data has
        // arrived
        hgtMea =
            baroCalculateAltitude() /
            100.0f;  // posEstimator.baro.alt / 100.0f;  // convert to meters

        newDataHgt = true;

        // get states that wer stored at the time closest to the measurement
        // time, taking measurement delay into account
        ekf_RecallStates(&statesAtHgtTime, (imuSampleTime_ms - _msecHgtDelay));
    } else {
        newDataHgt = false;
    }
}

// decay GPS horizontal position offset to close to zero at a rate of 1 m/s
// limit radius to a maximum of 100m
// apply glitch offset to GPS measurements
void ekf_decayGpsOffset(void)
{
    float lapsedTime = 0.001f * (float)(imuSampleTime_ms - lastDecayTime_ms);
    lastDecayTime_ms = imuSampleTime_ms;
    float offsetRadius = calc_length_pythagorean_2D(gpsPosGlitchOffsetNE.x,
                                                    gpsPosGlitchOffsetNE.y);
    // decay radius if larger than velocity of 1.0 multiplied by lapsed time
    // (plus a margin to prevent divide by zero)
    if (offsetRadius > (lapsedTime + 0.1f)) {
        // calculate scale factor to be applied to both offset components
        float scaleFactor =
            constrainf((offsetRadius - lapsedTime), 0.0f, 100.0f) /
            offsetRadius;
        gpsPosGlitchOffsetNE.x *= scaleFactor;
        gpsPosGlitchOffsetNE.y *= scaleFactor;
    }
}

// check for new valid GPS data and update stored measurement if available
void ekf_readGpsData(void)
{
    // check for new GPS data
    if ((gpsState.lastMessageMs != lastFixTime_ms) && isGPSTrustworthy()) {
        // store fix time from previous read
        secondLastFixTime_ms = lastFixTime_ms;

        // get current fix time
        lastFixTime_ms = gpsState.lastMessageMs;

        // set flag that lets other functions know that new GPS data has arrived
        newDataGps = true;

        // get state vectors that were stored at the time that is closest to
        // when the the GPS measurement time after accounting for measurement
        // delays
        ekf_RecallStates(&statesAtVelTime,
                         (imuSampleTime_ms -
                          (int16_t)constrainf(_msecVelDelay, 0.0f, 500.0f)));
        ekf_RecallStates(&statesAtPosTime,
                         (imuSampleTime_ms -
                          (int16_t)constrainf(_msecPosDelay, 0.0f, 500.0f)));

        // read the NED velocity from the GPS
        velNED.x = gpsSol.velNED[X] * 0.01f;
        velNED.y = gpsSol.velNED[Y] * 0.01f;
        velNED.z = gpsSol.velNED[Z] * 0.01f;

        // read latitutde and longitude from GPS and convert to NE position
        if (!ekfOrigin.valid) {
            geoSetOrigin(&ekfOrigin, &gpsSol.llh, GEO_ORIGIN_SET);
        } else {
            geoConvertGeodeticToLocal(&gpsPosNE, &ekfOrigin, &gpsSol.llh,
                                      GEO_ALT_ABSOLUTE);
        }

        // decay and limit the position offset which is applied to NE position
        // wherever it is used throughout code to allow GPS position jumps to be
        // accommodated gradually
        ekf_decayGpsOffset();
    }
}

// resets position states to last GPS measurement or to zero if in static mode
void ekf_ResetPosition(void)
{
    if (staticMode) {
        state->position.x = 0.0f;
        state->position.y = 0.0f;
    } else if (isGPSTrustworthy()) {
        // read the GPS
        ekf_readGpsData();
        // write to state vector and compensate for GPS latency
        state->position.x = gpsPosNE.x + gpsPosGlitchOffsetNE.x +
                           0.001f * velNED.x * _msecPosDelay;
        state->position.y = gpsPosNE.y + gpsPosGlitchOffsetNE.y +
                           0.001f * velNED.y * _msecPosDelay;
    }
    // stored horizontal position states to prevent subsequent GPS measurements
    // from being rejected
    for (uint8_t i = 0; i <= 49; i++) {
        storedStates[i].position.v[0] = state->position.v[0];
        storedStates[i].position.v[1] = state->position.v[1];
    }
}

// resets velocity states to last GPS measurement or to zero if in static mode
void ekf_ResetVelocity(void)
{
    if (staticMode) {
        vectorZero(&state->velocity);
        vectorZero(&state->vel1);
        vectorZero(&state->vel2);
    } else if (isGPSTrustworthy()) {
        // read the GPS
        ekf_readGpsData();
        // reset filter velocity states
        state->velocity = velNED;
        state->vel1 = velNED;
        state->vel2 = velNED;
        // reset stored velocity states to prevent subsequent GPS measurements
        // from being rejected
        for (uint8_t i = 0; i <= 49; i++) {
            storedStates[i].velocity = velNED;
        }
    }
}

// reset the vertical position state using the last height measurement
void ekf_ResetHeight(void)
{
    // read the altimeter
    ekf_readHgtData();
    // write to the state vector
    state->position.z = -hgtMea;  // down position from blended accel data
    state->posD1 = -hgtMea;       // down position from IMU1 accel data
    state->posD2 = -hgtMea;       // down position from IMU2 accel data
    // reset stored vertical position states to prevent subsequent GPS
    // measurements from being rejected
    for (uint8_t i = 0; i <= 49; i++) {
        storedStates[i].position.z = -hgtMea;
    }
}

// store states in a history array along with time stamp
void ekf_StoreStates(void)
{
    // Don't need to store states more often than every 10 msec
    if (imuSampleTime_ms - lastStateStoreTime_ms >= 10) {
        lastStateStoreTime_ms = imuSampleTime_ms;
        if (storeIndex > 49) {
            storeIndex = 0;
        }
        storedStates[storeIndex] = *state;
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
    storedStates[storeIndex] = *state;
    statetimeStamp[storeIndex] = imuSampleTime_ms;
    storeIndex = storeIndex + 1;
}

// Check basic filter health metrics and return a consolidated health status
bool ekf_healthy(void)
{
    if (!statesInitialised) {
        return false;
    }

    if (isnan(state->quat.q0) || isnan(state->quat.q1) || isnan(state->quat.q2) ||
        isnan(state->quat.q3)) {
        return false;
    }

    if (isnan(state->velocity.x) || isnan(state->velocity.y) ||
        isnan(state->velocity.z)) {
        return false;
    }

    if (filterDiverged || (imuSampleTime_ms - lastDivergeTime_ms < 10000)) {
        return false;
    }

    // If measurements have failed innovation consistency checks for long enough
    // to time-out and force fusion then the nav solution can be conidered to be
    // unhealthy This will only be set as a transient
    if (posTimeout || velTimeout || hgtTimeout) {
        return false;
    }

    // all OK
    return true;
}

bool system_using_EKF(void)
{
    // check if the EKF is configured to use and if the healthy is ok
    return ekf_started && _ekf_use && ekf_healthy();
}

// zero stored variables - this needs to be called before a full filter
// initialisation
void ekf_ZeroVariables(void)
{
    // initialise time stamps
    imuSampleTime_ms = millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    lastDivergeTime_ms = imuSampleTime_ms;
    // TASmsecPrev = imuSampleTime_ms;
    // BETAmsecPrev = imuSampleTime_ms;
    lastMagUpdate = imuSampleTime_ms;
    lastHgtMeasTime = imuSampleTime_ms;
    lastHgtTime_ms = imuSampleTime_ms;
    velFailTime = imuSampleTime_ms;
    posFailTime = imuSampleTime_ms;
    hgtFailTime = imuSampleTime_ms;
    lastStateStoreTime_ms = imuSampleTime_ms;
    lastFixTime_ms = imuSampleTime_ms;
    secondLastFixTime_ms = imuSampleTime_ms;
    lastDecayTime_ms = imuSampleTime_ms;

    velTimeout = false;
    posTimeout = false;
    hgtTimeout = false;
    filterDiverged = false;
    magTimeout = false;
    magFailed = false;
    storeIndex = 0;
    dtIMU = 0.0f;
    dt = 0.0f;
    hgtMea = 0.0f;
    vectorZero(&lastGyroBias);
    vectorZero(&prevDelAng);
    vectorZero(&lastAngRate);
    vectorZero(&lastAccel1);
    vectorZero(&lastAccel2);
    vectorZero(&velDotNEDfilt);
    vectorZero(&summedDelAng);
    vectorZero(&summedDelVel);
    vectorZero(&velNED);
    gpsPosGlitchOffsetNE.x = 0.0f;
    gpsPosGlitchOffsetNE.y = 0.0f;
    vectorZero(&gpsPosNE);
    prevTnb.m[0][0] = prevTnb.m[1][1] = prevTnb.m[2][2] = 0.0f;
    prevTnb.m[0][1] = prevTnb.m[0][2] = 0.0f;
    prevTnb.m[1][0] = prevTnb.m[1][2] = 0.0f;
    prevTnb.m[2][0] = prevTnb.m[2][1] = 0.0f;
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
    for (uint8_t i = 1; i <= 21; i++) {
        for (uint8_t j = 0; j <= 21; j++) {
            P[i][j] = 0.0f;
        }
    }
    // quaternions - TODO better maths for initial quaternion covariances that
    // uses roll, pitch and yaw
    P[0][0] = 1.0e-9f;
    P[1][1] = 0.25f * sq(DEGREES_TO_RADIANS(1.0f));
    P[2][2] = 0.25f * sq(DEGREES_TO_RADIANS(1.0f));
    P[3][3] = 0.25f * sq(DEGREES_TO_RADIANS(1.0f));
    // velocities
    P[4][4] = sq(0.7f);
    P[5][5] = P[4][4];
    P[6][6] = sq(0.7f);
    // positions
    P[7][7] = sq(15.0f);
    P[8][8] = P[7][7];
    P[9][9] = sq(5.0f);
    // delta angle biases
    P[10][10] = sq(DEGREES_TO_RADIANS(INIT_GYRO_BIAS_UNCERTAINTY * dtIMU));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // Z delta velocity bias
    P[13][13] = 0.0f;
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
#ifndef IGNORE_COMPASS_EKF
    // declare local variables required to calculate initial orientation and
    // magnetic field
    float yaw;
    fpMat3_t Tbn;
    fpVector3_t initMagNED;
    fpQuaternion_t initQuat;

    if (ekf_use_compass()) {
        // calculate rotation matrix from body to NED frame
        matrix_from_euler(&Tbn, roll, pitch, 0.0f);

        // read the magnetometer data
        ekf_readMagData();

        // rotate the magnetic field into NED axes
        const fpVector3_t vecMagData = {.v = {magData.x - magBias.x,
                                              magData.y - magBias.y,
                                              magData.z - magBias.z}};
        initMagNED = multiply_matrix_by_vector(Tbn, vecMagData);

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = 0.0f;  //_ahrs->get_compass()->get_declination();

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAligned = true;

        // calculate initial filter quaternion states
        quaternion_from_euler(&initQuat, roll, pitch, yaw);

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        quaternion_to_rotation_matrix(initQuat, &Tbn);

        // write to earth magnetic field state vector
        state->earth_magfield = multiply_matrix_by_vector(Tbn, vecMagData);
    } else {
        quaternion_from_euler(&initQuat, roll, pitch, 0.0f);
        yawAligned = false;
    }

#else

    fpQuaternion_t initQuat;
    quaternion_from_euler(&initQuat, roll, pitch, 0.0f);
    yawAligned = false;

#endif

    // return attitude quaternion
    return initQuat;
}

// calculate the NED earth spin vector in rad/sec
void ekf_calcEarthRateNED(fpVector3_t *omega, float latitude)
{
    float lat_rad = DEGREES_TO_RADIANS(latitude);
    omega->x = EARTHRATE * cosf(lat_rad);
    omega->y = 0.0f;
    omega->z = -EARTHRATE * sinf(lat_rad);
}

// this function is used to initialise the filter whilst moving, using the AHRS
// DCM solution it should NOT be used to re-initialise after a timeout as DCM
// will also be corrupted
void ekf_InitialiseFilterDynamic(void)
{
    // this forces healthy() to be false so that when we ask for ahrs
    // attitude we get the DCM attitude regardless of the state of AHRS_EKF_USE
    statesInitialised = false;

    // Set re-used variables to zero
    ekf_ZeroVariables();
    
    // This should be a multiple of the imu update interval.
    dtVelPos = US2S(getLooptime()) * 2.0f;

    // set number of updates over which gps and baro measurements are applied to
    // the velocity and position states
    gpsUpdateCountMaxInv = (dtIMU * 1000.0f) / _msecGpsAvg;
    gpsUpdateCountMax = (uint8_t)(1.0f / gpsUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMU * 1000.0f) / _msecHgtAvg;
    hgtUpdateCountMax = (uint8_t)(1.0f / hgtUpdateCountMaxInv);
    magUpdateCountMaxInv = (dtIMU * 1000.0f) / _msecMagAvg;
    magUpdateCountMax = (uint8_t)(1.0f / magUpdateCountMaxInv);

    // calculate initial orientation and earth magnetic field states
    state->quat =
        ekf_calcQuatAndFieldStates(attitude.values.roll, attitude.values.pitch);

    // write to state vector
    vectorZero(&state->gyro_bias);
    state->accel_zbias1 = 0.0f;
    state->accel_zbias2 = 0.0f;
    state->wind_vel.x = 0.0f;
    state->wind_vel.y = 0.0f;
    ekf_ResetVelocity();
    ekf_ResetPosition();
    ekf_ResetHeight();
#ifndef IGNORE_COMPASS_EKF
    state->body_magfield = magBias;
#endif

    // set to true now that states have be initialised
    statesInitialised = true;

    // initialise the covariance matrix
    ekf_CovarianceInit();

    // define Earth rotation vector in the NED navigation frame
    ekf_calcEarthRateNED(&earthRateNED, posControl.rthState.homePosition.pos.x);

    // initialise IMU pre-processing states
    ekf_readIMUData();
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void ekf_ForceSymmetry(void)
{
    for (uint8_t i = 1; i <= 21; i++) {
        for (uint8_t j = 0; j <= i - 1; j++) {
            if (fabsf(P[i][j]) > EKF_COVARIENCE_MAX ||
                fabsf(P[j][i]) > EKF_COVARIENCE_MAX) {
                // set the filter status as diverged and re-initialise the
                // filter
                filterDiverged = true;
                faultStatus.diverged = true;
                lastDivergeTime_ms = imuSampleTime_ms;
                ekf_InitialiseFilterDynamic();
                return;
            }
            float temp = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to
// prevent ill-conditioning
void ekf_ConstrainVariances(void)
{
    for (uint8_t i = 0; i <= 3; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0f);  // quaternions
    for (uint8_t i = 4; i <= 6; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e3f);  // velocities
    for (uint8_t i = 7; i <= 9; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e6f);  // positions
    for (uint8_t i = 10; i <= 12; i++)
        P[i][i] = constrainf(P[i][i], 0.0f,
                             sq(0.175f * dtIMU));  // delta angle biases
    P[13][13] =
        constrainf(P[13][13], 0.0f, sq(10.0f * dtIMU));  // delta velocity bias
    for (uint8_t i = 14; i <= 15; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e3f);  // earth magnetic field
    for (uint8_t i = 16; i <= 21; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0f);  // body magnetic field
}

// constrain states to prevent ill-conditioning
void ekf_ConstrainStates(void)
{
    // quaternions are limited between +-1
    for (uint8_t i = 0; i <= 3; i++)
        states[i] = constrainf(states[i], -1.0f, 1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max
    // airspeed * EAS2TAS)
    for (uint8_t i = 4; i <= 6; i++)
        states[i] = constrainf(states[i], -5.0e2f, 5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i = 7; i <= 8; i++)
        states[i] = constrainf(states[i], -1.0e6f, 1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and
    // ballon drop
    states[9] = constrainf(states[9], -4.0e4f, 1.0e4f);
    // gyro bias limit ~6 deg/sec (this needs to be set based on manufacturers
    // specs)
    for (uint8_t i = 10; i <= 12; i++)
        states[i] = constrainf(states[i], -0.1f * dtIMU, 0.1f * dtIMU);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test
    // data)
    states[13] = constrainf(states[13], -1.0f * dtIMU, 1.0f * dtIMU);
    states[22] = constrainf(states[22], -1.0f * dtIMU, 1.0f * dtIMU);
    // wind velocity limit 100 m/s (could be based on some multiple of max
    // airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i = 14; i <= 15; i++)
        states[i] = constrainf(states[i], -100.0f, 100.0f);
    // earth magnetic field limit
    for (uint8_t i = 16; i <= 18; i++)
        states[i] = constrainf(states[i], -1.0f, 1.0f);
    // body magnetic field limit
    for (uint8_t i = 19; i <= 21; i++)
        states[i] = constrainf(states[i], -0.5f, 0.5f);
}

// copy covariances across from covariance prediction calculation and fix
// numerical errors
void ekf_CopyAndFixCovariances(void)
{
    // copy predicted variances
    for (uint8_t i = 0; i <= 21; i++) {
        P[i][i] = nextP[i][i];
    }
    // copy predicted covariances and force symmetry
    for (uint8_t i = 1; i <= 21; i++) {
        for (uint8_t j = 0; j <= i - 1; j++) {
            P[i][j] = 0.5f * (nextP[i][j] + nextP[j][i]);
            P[j][i] = P[i][j];
        }
    }
}

// fuse selected position, velocity and height measurements, checking dat for
// consistency provide a static mode that allows maintenance of the attitude
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
    float NEvelErr;
    float DvelErr;
    float posErr;
    float R_OBS[6];  // Measurement variances used for fusion
    float observation[6];
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // if static mode is active use the current states to calculate the
        // predicted measurement rather than use states from a previous time. We
        // need to do this because there may be no stored states due to lack of
        // real measurements. in static mode, only position and height fusion is
        // used
        if (staticMode) {
            statesAtPosTime = *state;
            statesAtHgtTime = *state;
        }

        // set the GPS data timeout depending on whether airspeed data is
        // present
        uint32_t gpsRetryTime;
        if (ekf_useAirspeed()) {
            gpsRetryTime =
                20000;  // GPS retry time with airspeed measurements (msec)
        } else {
            gpsRetryTime =
                10000;  // GPS retry time without airspeed measurements (msec)
        }

        // form the observation vector and zero velocity and horizontal position
        // observations if in static mode
        if (!staticMode) {
            for (uint8_t i = 0; i <= 2; i++) {
                observation[i] = velNED.v[i];
            }
            observation[3] = gpsPosNE.x + gpsPosGlitchOffsetNE.x;
            observation[4] = gpsPosNE.y + gpsPosGlitchOffsetNE.y;
        } else {
            for (uint8_t i = 0; i <= 4; i++) {
                observation[i] = 0.0f;
            }
        }

        observation[5] = -hgtMea;

        // calculate additional error in GPS velocity caused by manoeuvring
        NEvelErr = _gpsNEVelVarAccScale * accNavMag;
        DvelErr = _gpsDVelVarAccScale * accNavMag;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = _gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement
        // variances.
        R_OBS[0] =
            sq(constrainf(_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(NEvelErr);
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = sq(constrainf(_gpsVertVelNoise, 0.05f, 5.0f)) + sq(DvelErr);
        R_OBS[3] = sq(constrainf(_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(constrainf(_baroAltNoise, 0.1f, 10.0f));

        // if vertical GPS velocity data is being used, check to see if the GPS
        // vertical velocity and barometer innovations have the same sign and
        // are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer
        // innovation consistency checks.
        bool badIMUdata = false;
        if (fuseVelData &&
            (imuSampleTime_ms - lastHgtTime_ms) < (2 * _msecHgtAvg)) {
            // calculate innovations for height and vertical GPS vel
            // measurements
            float hgtErr = statesAtHgtTime.position.z - observation[5];
            float velDErr = statesAtVelTime.velocity.z - observation[2];
            // check if they are the same sign and both more than 3-sigma out of
            // bounds
            if ((hgtErr * velDErr > 0.0f) &&
                (sq(hgtErr) > 9.0f * (P[9][9] + R_OBS[5])) &&
                (sq(velDErr) > 9.0f * (P[6][6] + R_OBS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation
        // consistency check test position measurements
        if (fusePosData) {
            // test horizontal position measurements
            posInnov.v[0] = statesAtPosTime.position.x - observation[3];
            posInnov.v[1] = statesAtPosTime.position.y - observation[4];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply an innovation consistency threshold test, but don't fail if
            // bad IMU data calculate max valid position innovation squared
            // based on a maximum horizontal inertial nav accel error and GPS
            // noise parameter max inertial nav error is scaled with horizontal
            // g to allow for increased errors when manoeuvring
            float accelScale = (1.0f + 0.1f * accNavMag);
            float maxPosInnov2 =
                sq(_gpsPosInnovGate * _gpsHorizPosNoise +
                   0.005f * accelScale * _gpsGlitchAccelMax *
                       sq(0.001f * (float)(imuSampleTime_ms - posFailTime)));
            posTestRatio =
                (sq(posInnov.v[0]) + sq(posInnov.v[1])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // declare a timeout condition if we have been too long without data
            posTimeout = ((imuSampleTime_ms - posFailTime) > gpsRetryTime);
            // use position data if healthy, timed out, or in static mode
            if (posHealth || posTimeout || staticMode) {
                posHealth = true;
                posFailTime = imuSampleTime_ms;
                // if timed out or outside the specified glitch radius,
                // increment the offset applied to GPS data to compensate for
                // large GPS position jumps
                if (posTimeout || (maxPosInnov2 > sq(_gpsGlitchRadiusMax))) {
                    gpsPosGlitchOffsetNE.x += posInnov.v[0];
                    gpsPosGlitchOffsetNE.y += posInnov.v[1];
                    // limit the radius of the offset to 100m and decay the
                    // offset to zero radially
                    ekf_decayGpsOffset();
                    // reset the position to the current GPS position which will
                    // include the glitch correction offset
                    ekf_ResetPosition();
                    // don't fuse data on this time step
                    fusePosData = false;
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            float K1 = 0;             // innovation to error ratio for IMU1
            float K2 = 0;             // innovation to error ratio for IMU2
            float innovVelSumSq = 0;  // sum of squares of velocity innovations
            float varVelSum = 0;      // sum of velocity innovation variances
            for (uint8_t i = 0; i <= imax; i++) {
                // velocity states start at index 4
                stateIndex = i + 4;
                // calculate innovations using blended and single IMU predicted
                // states
                velInnov.v[i] =
                    statesAtVelTime.velocity.v[i] - observation[i];  // blended
                velInnov1.v[i] =
                    statesAtVelTime.vel1.v[i] - observation[i];  // IMU1
                velInnov2.v[i] =
                    statesAtVelTime.vel2.v[i] - observation[i];  // IMU2
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];
                // calculate error weightings for single IMU velocity states
                // using observation error to normalise
                float R_hgt;
                if (i == 2) {
                    R_hgt = sq(constrainf(_gpsVertVelNoise, 0.05f, 5.0f));
                } else {
                    R_hgt = sq(constrainf(_gpsHorizVelNoise, 0.05f, 5.0f));
                }
                K1 += R_hgt / (R_hgt + sq(velInnov1.v[i]));
                K2 += R_hgt / (R_hgt + sq(velInnov2.v[i]));
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov.v[i]);
                varVelSum += varInnovVelPos[i];
            }
            // calculate weighting used by fuseVelPosNED to do IMU accel data
            // blending this is used to detect and compensate for aliasing
            // errors with the accelerometers provide for a first order lowpass
            // filter to reduce noise on the weighting if required
            IMU1_weighting =
                1.0f * (K1 / (K1 + K2)) +
                0.0f * IMU1_weighting;  // filter currently inactive
            // apply an innovation consistency threshold test, but don't fail if
            // bad IMU data calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(_gpsVelInnovGate));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f) || badIMUdata);
            // declare a timeout if we have not fused velocity data for too long
            velTimeout = (imuSampleTime_ms - velFailTime) > gpsRetryTime;
            // if data is healthy  or in static mode we fuse it
            if (velHealth || staticMode) {
                velHealth = true;
                velFailTime = imuSampleTime_ms;
            } else if (velTimeout && !posHealth) {
                // if data is not healthy and timed out and position is
                // unhealthy we reset the velocity, but do not fuse data on this
                // time step
                ekf_ResetVelocity();
                ekf_StoreStatesReset();
                fuseVelData = false;
            } else {
                // if data is unhealthy and position is healthy, we do not fuse
                // it
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData) {
            // set the height data timeout depending on whether vertical
            // velocity data is being used
            uint32_t hgtRetryTime;
            hgtRetryTime = _hgtRetryTimeMode0;
            // calculate height innovations
            hgtInnov = statesAtHgtTime.position.z - observation[5];
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio =
                sq(hgtInnov) / (sq(_hgtInnovGate) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            hgtTimeout = (imuSampleTime_ms - hgtFailTime) > hgtRetryTime;
            // Fuse height data if healthy or timed out or in static mode
            if (hgtHealth || hgtTimeout || staticMode) {
                hgtHealth = true;
                hgtFailTime = imuSampleTime_ms;
                // if timed out, reset the height, but do not fuse data on this
                // time step
                if (hgtTimeout) {
                    ekf_ResetHeight();
                    ekf_StoreStatesReset();
                    fuseHgtData = false;
                }
            } else {
                hgtHealth = false;
            }
        }

        // set range for sequential fusion of velocity and position measurements
        // depending on which data is available and its health
        if (fuseVelData && velHealth && !staticMode) {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }

        if ((fusePosData && posHealth) || staticMode) {
            fuseData[3] = true;
            fuseData[4] = true;
        }

        if ((fuseHgtData && hgtHealth) || staticMode) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially
        for (obsIndex = 0; obsIndex <= 5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 4 + obsIndex;
                // calculate the measurement innovation, using states from a
                // different time coordinate if fusing height data adjust
                // scaling on GPS measurement noise variances if not enough
                // satellites
                if (obsIndex <= 2) {
                    innovVelPos[obsIndex] =
                        statesAtVelTime.velocity.v[obsIndex] -
                        observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] =
                        statesAtPosTime.position.v[obsIndex - 3] -
                        observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else {
                    innovVelPos[obsIndex] =
                        statesAtHgtTime.position.v[obsIndex - 3] -
                        observation[obsIndex];
                }

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] =
                    P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f / varInnovVelPos[obsIndex];
                for (uint8_t i = 0; i <= 12; i++) {
                    Kfusion[i] = P[i][stateIndex] * SK;
                }
                // Only height observations are used to update z accel bias
                // estimate Protect Kalman gain from ill-conditioning Don't
                // update Z accel bias if off-level by greater than 60 degrees
                // to avoid scale factor error effects
                if (obsIndex == 5 && prevTnb.m[2][2] > 0.5f) {
                    Kfusion[13] =
                        constrainf(P[13][stateIndex] * SK, -1.0f, 0.0f);
                } else {
                    Kfusion[13] = 0.0f;
                }
                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates) {
                    Kfusion[14] = P[14][stateIndex] * SK;
                    Kfusion[15] = P[15][stateIndex] * SK;
                } else {
                    Kfusion[14] = 0.0f;
                    Kfusion[15] = 0.0f;
                }
                // inhibit magnetic field state estimation by setting Kalman
                // gains to zero
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i <= 21; i++) {
                        Kfusion[i] = P[i][stateIndex] * SK;
                    }
                } else {
                    for (uint8_t i = 16; i <= 21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }
                // Set the Kalman gain values for the single IMU states
                Kfusion[22] = Kfusion[13];  // IMU2 Z accel bias
                Kfusion[26] = Kfusion[9];   // IMU1 posD
                Kfusion[30] = Kfusion[9];   // IMU2 posD
                for (uint8_t i = 0; i <= 2; i++) {
                    Kfusion[i + 23] = Kfusion[i + 4];  // IMU1 velNED
                    Kfusion[i + 27] = Kfusion[i + 4];  // IMU2 velNED
                }

                // Correct states that have been predicted using single (not
                // blended) IMU data
                if (obsIndex == 5) {
                    // Calculate height measurement innovations using single IMU
                    // states
                    float hgtInnov1 =
                        statesAtHgtTime.posD1 - observation[obsIndex];
                    float hgtInnov2 =
                        statesAtHgtTime.posD2 - observation[obsIndex];
                    // Correct single IMU prediction states using height
                    // measurement, limiting rate of change of bias to 0.02 m/s3
                    float correctionLimit = 0.02f * dtIMU * dtVelPos;
                    state->accel_zbias1 -=
                        constrainf(Kfusion[13] * hgtInnov1, -correctionLimit,
                                   correctionLimit);  // IMU1 Z accel bias
                    state->accel_zbias2 -=
                        constrainf(Kfusion[22] * hgtInnov2, -correctionLimit,
                                   correctionLimit);  // IMU2 Z accel bias
                    for (uint8_t i = 23; i <= 26; i++) {
                        states[i] = states[i] -
                                    Kfusion[i] * hgtInnov1;  // IMU1 velNED,posD
                    }
                    for (uint8_t i = 27; i <= 30; i++) {
                        states[i] = states[i] -
                                    Kfusion[i] * hgtInnov2;  // IMU2 velNED,posD
                    }
                } else if (obsIndex == 0 || obsIndex == 1 || obsIndex == 2) {
                    // Correct single IMU prediction states using velocity
                    // measurements
                    for (uint8_t i = 23; i <= 26; i++) {
                        states[i] =
                            states[i] -
                            Kfusion[i] *
                                velInnov1.v[obsIndex];  // IMU1 velNED,posD
                    }
                    for (uint8_t i = 27; i <= 30; i++) {
                        states[i] =
                            states[i] -
                            Kfusion[i] *
                                velInnov2.v[obsIndex];  // IMU2 velNED,posD
                    }
                }

                // calculate state corrections and re-normalise the quaternions
                // for states predicted using the blended IMU data attitude,
                // velocity and position corrections are spread across multiple
                // prediction cycles between now and the anticipated time for
                // the next measurement. Don't spread quaternion corrections if
                // total angle change across predicted interval is going to
                // exceed 0.1 rad
                bool highRates = ((gpsUpdateCountMax *
                                   calc_length_pythagorean_3D(
                                       correctedDelAng.x, correctedDelAng.y,
                                       correctedDelAng.z)) > 0.1f);
                for (uint8_t i = 0; i <= 21; i++) {
                    if ((i <= 3 && highRates) || i >= 10 || staticMode) {
                        states[i] =
                            states[i] - Kfusion[i] * innovVelPos[obsIndex];
                    } else {
                        if (obsIndex == 5) {
                            hgtIncrStateDelta[i] -= Kfusion[i] *
                                                    innovVelPos[obsIndex] *
                                                    hgtUpdateCountMaxInv;
                        } else {
                            gpsIncrStateDelta[i] -= Kfusion[i] *
                                                    innovVelPos[obsIndex] *
                                                    gpsUpdateCountMaxInv;
                        }
                    }
                }

                quaternion_normalize(&state->quat);

                // update the covariance - take advantage of direct observation
                // of a single state at index = stateIndex to reduce
                // computations this is a numerically optimised implementation
                // of standard equation P = (I - K*H)*P;
                for (uint8_t i = 0; i <= 21; i++) {
                    for (uint8_t j = 0; j <= 21; j++) {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i = 0; i <= 21; i++) {
                    for (uint8_t j = 0; j <= 21; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to
    // prevent ill-condiioning.
    ekf_ForceSymmetry();
    ekf_ConstrainVariances();
}

// select fusion of velocity, position and height measurements
void ekf_SelectVelPosFusion(void)
{
    // check for new data, specify which measurements should be used and check
    // data for freshness
    if (!staticMode) {
        // check for and read new GPS data
        ekf_readGpsData();

        // command fusion of GPS data and reset states as required
        if (newDataGps) {
            // reset data arrived flag
            newDataGps = false;
            // reset state updates and counter used to spread fusion updates
            // across several frames to reduce 10Hz pulsing
            memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
            gpsUpdateCount = 0;
            // enable fusion
            fuseVelData = true;
            fusePosData = true;
            // If a long time since last GPS update, then reset position and
            // velocity and reset stored state history
            uint32_t gpsRetryTimeout = ekf_useAirspeed() ? 20000 : 10000;
            if (imuSampleTime_ms - secondLastFixTime_ms > gpsRetryTimeout) {
                ekf_ResetPosition();
                ekf_ResetVelocity();
                ekf_StoreStatesReset();
            }
        } else {
            fuseVelData = false;
            fusePosData = false;
        }

    } else {
        // in static mode use synthetic position measurements set to zero
        // only fuse synthetic measurements when rate of change of velocity is
        // less than 0.5g to reduce attitude errors due to launch acceleration
        // do not use velocity fusion to reduce the effect of movement on
        // attitude
        if (accNavMag < 4.9f) {
            fusePosData = true;
        } else {
            fusePosData = false;
        }
        fuseVelData = false;
    }

    // check for and read new height data
    ekf_readHgtData();

    // command fusion of height data
    if (newDataHgt) {
        // reset data arrived flag
        newDataHgt = false;
        // reset state updates and counter used to spread fusion updates across
        // several frames to reduce 10Hz pulsing
        memset(&hgtIncrStateDelta[0], 0, sizeof(hgtIncrStateDelta));
        hgtUpdateCount = 0;
        // enable fusion
        fuseHgtData = true;
    } else {
        fuseHgtData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        ekf_FuseVelPosNED();
    }

    // Fuse corrections to quaternion, position and velocity states across
    // several time steps to reduce 5 and 10Hz pulsing in the output
    if (gpsUpdateCount < gpsUpdateCountMax) {
        gpsUpdateCount++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += gpsIncrStateDelta[i];
        }
    }
    if (hgtUpdateCount < hgtUpdateCountMax) {
        hgtUpdateCount++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += hgtIncrStateDelta[i];
        }
    }
}

// update the quaternion, velocity and position states using IMU measurements
void ekf_UpdateStrapdownEquationsNED(void)
{
    fpVector3_t delVelNav;  // delta velocity vector calculated using a blend of
                            // IMU1 and IMU2 data
    fpVector3_t delVelNav1;  // delta velocity vector calculated using IMU1 data
    fpVector3_t delVelNav2;  // delta velocity vector calculated using IMU2 data
    float rotationMag;  // magnitude of rotation vector from previous to current
                        // time step
    float rotScaler;    // scaling variable used to calculate delta quaternion
                        // from last to current time step
    fpQuaternion_t qUpdated;   // quaternion at current time step after
                               // application of delta quaternion
    fpQuaternion_t deltaQuat;  // quaternion from last to current time step
    fpVector3_t gravityNED;    // NED gravity vector m/s^2
    gravityNED.x = 0;
    gravityNED.y = 0;
    gravityNED.z = GRAVITY_MSS;

    // remove sensor bias errors
    correctedDelAng.x = dAngIMU.x - state->gyro_bias.x;
    correctedDelAng.y = dAngIMU.y - state->gyro_bias.y;
    correctedDelAng.z = dAngIMU.z - state->gyro_bias.z;
    correctedDelVel1 = dVelIMU1;
    correctedDelVel2 = dVelIMU2;
    correctedDelVel1.z -= state->accel_zbias1;
    correctedDelVel2.z -= state->accel_zbias2;

    // use weighted average of both IMU units for delta velocities
    correctedDelVel12.x = correctedDelVel1.x * IMU1_weighting +
                          correctedDelVel2.x * (1.0f - IMU1_weighting);
    correctedDelVel12.y = correctedDelVel1.y * IMU1_weighting +
                          correctedDelVel2.y * (1.0f - IMU1_weighting);
    correctedDelVel12.z = correctedDelVel1.z * IMU1_weighting +
                          correctedDelVel2.z * (1.0f - IMU1_weighting);

    // save current measurements
    prevDelAng = correctedDelAng;

    // apply corrections for earths rotation rate and coning errors
    const fpVector3_t delAngCrossProduct =
        get_vector_cross_product(prevDelAng, correctedDelAng);
    const fpVector3_t earthNED =
        multiply_matrix_by_vector(prevTnb, earthRateNED);
    const fpVector3_t nedRate = {
        .v = {earthNED.x * dtIMU, earthNED.y * dtIMU, earthNED.z * dtIMU}};
    correctedDelAng.x = correctedDelAng.x - nedRate.x + delAngCrossProduct.x * 8.333333e-2f;
    correctedDelAng.y = correctedDelAng.y - nedRate.y + delAngCrossProduct.y * 8.333333e-2f;
    correctedDelAng.z = correctedDelAng.z - nedRate.z + delAngCrossProduct.z * 8.333333e-2f;

    // convert the rotation vector to its equivalent quaternion
    rotationMag = calc_length_pythagorean_3D(
        correctedDelAng.x, correctedDelAng.y, correctedDelAng.z);

    if (rotationMag < 1e-12f) {
        deltaQuat.q0 = 1.0f;
        deltaQuat.q1 = 0.0f;
        deltaQuat.q2 = 0.0f;
        deltaQuat.q3 = 0.0f;
    } else {
        deltaQuat.q0 = cosf(0.5f * rotationMag);
        rotScaler = (sinf(0.5f * rotationMag)) / rotationMag;
        deltaQuat.q1 = correctedDelAng.x * rotScaler;
        deltaQuat.q2 = correctedDelAng.y * rotScaler;
        deltaQuat.q3 = correctedDelAng.z * rotScaler;
    }

    // update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    qUpdated.q0 = states[0] * deltaQuat.q0 - states[1] * deltaQuat.q1 -
                  states[2] * deltaQuat.q2 - states[3] * deltaQuat.q3;
    qUpdated.q1 = states[0] * deltaQuat.q1 + states[1] * deltaQuat.q0 +
                  states[2] * deltaQuat.q3 - states[3] * deltaQuat.q2;
    qUpdated.q2 = states[0] * deltaQuat.q2 + states[2] * deltaQuat.q0 +
                  states[3] * deltaQuat.q1 - states[1] * deltaQuat.q3;
    qUpdated.q3 = states[0] * deltaQuat.q3 + states[3] * deltaQuat.q0 +
                  states[1] * deltaQuat.q2 - states[2] * deltaQuat.q1;

    // normalise the quaternions and update the quaternion states
    quaternion_normalize(&qUpdated);
    state->quat = qUpdated;

    // calculate the body to nav cosine matrix
    fpMat3_t Tbn_temp;
    quaternion_to_rotation_matrix(state->quat, &Tbn_temp);
    prevTnb = matrix_transposed(Tbn_temp);
    
    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    // blended IMU calc
    const fpVector3_t dtVel12 = {
        .v = {correctedDelVel12.x + gravityNED.x * dtIMU,
              correctedDelVel12.y + gravityNED.y * dtIMU,
              correctedDelVel12.z + gravityNED.z * dtIMU}};
    delVelNav = multiply_matrix_by_vector(Tbn_temp, dtVel12);

    // single IMU calcs
    const fpVector3_t dtVel1 = {
        .v = {correctedDelVel1.x + gravityNED.x * dtIMU,
              correctedDelVel1.y + gravityNED.y * dtIMU,
              correctedDelVel1.z + gravityNED.z * dtIMU}};
    delVelNav1 = multiply_matrix_by_vector(Tbn_temp, dtVel1);

    const fpVector3_t dtVel2 = {
        .v = {correctedDelVel2.x + gravityNED.x * dtIMU,
              correctedDelVel2.y + gravityNED.y * dtIMU,
              correctedDelVel2.z + gravityNED.z * dtIMU}};
    delVelNav2 = multiply_matrix_by_vector(Tbn_temp, dtVel2);

    // calculate the rate of change of velocity (used for launch detect and
    // other functions)
    velDotNED.x = delVelNav.x / dtIMU;
    velDotNED.y = delVelNav.y / dtIMU;
    velDotNED.z = delVelNav.z / dtIMU;

    // apply a first order lowpass filter
    velDotNEDfilt.x = velDotNED.x * 0.05f + velDotNEDfilt.x * 0.95f;
    velDotNEDfilt.y = velDotNED.y * 0.05f + velDotNEDfilt.y * 0.95f;
    velDotNEDfilt.z = velDotNED.z * 0.05f + velDotNEDfilt.z * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = calc_length_pythagorean_3D(velDotNEDfilt.x, velDotNEDfilt.y,
                                           velDotNEDfilt.z);
    accNavMagHoriz =
        calc_length_pythagorean_2D(velDotNEDfilt.x, velDotNEDfilt.y);

    // save velocity for use in trapezoidal intergration for position calcuation
    const fpVector3_t lastVelocity = {.v = {state->velocity.x, state->velocity.y, state->velocity.z}};
    const fpVector3_t lastVel1 = {.v = {state->vel1.x, state->vel1.y, state->vel1.z}};
    const fpVector3_t lastVel2 = {.v = {state->vel2.x, state->vel2.y, state->vel2.z}};

    // sum delta velocities to get velocity
    state->velocity.x += delVelNav.x;
    state->velocity.y += delVelNav.y;
    state->velocity.z += delVelNav.z;
    state->vel1.x += delVelNav1.x;
    state->vel1.y += delVelNav1.y;
    state->vel1.z += delVelNav1.z;
    state->vel2.x += delVelNav2.x;
    state->vel2.y += delVelNav2.y;
    state->vel2.z += delVelNav2.z;

    // apply a trapezoidal integration to velocities to calculate position
    state->position.x += (state->velocity.x + lastVelocity.x) * (dtIMU * 0.5f);
    state->position.y += (state->velocity.y + lastVelocity.y) * (dtIMU * 0.5f);
    state->position.z += (state->velocity.z + lastVelocity.z) * (dtIMU * 0.5f);
    state->posD1 += (state->vel1.z + lastVel1.z) * (dtIMU * 0.5f);
    state->posD2 += (state->vel2.z + lastVel2.z) * (dtIMU * 0.5f);

    // limit states to protect against divergence
    ekf_ConstrainStates();
}

// calculate the predicted state covariance matrix
void ekf_CovariancePrediction(void)
{
    float windVelSigma;   // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;  // delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;  // delta velocity bias 1-sigma process noise - m/s
    float magEarthSigma;  // earth magnetic field 1-sigma process noise
    float magBodySigma;   // body magnetic field 1-sigma process noise
    float daxCov;         // X axis delta angle variance rad^2
    float dayCov;         // Y axis delta angle variance rad^2
    float dazCov;         // Z axis delta angle variance rad^2
    float dvxCov;         // X axis delta velocity variance (m/s)^2
    float dvyCov;         // Y axis delta velocity variance (m/s)^2
    float dvzCov;         // Z axis delta velocity variance (m/s)^2
    float dvx;            // X axis delta velocity (m/s)
    float dvy;            // Y axis delta velocity (m/s)
    float dvz;            // Z axis delta velocity (m/s)
    float dax;            // X axis delta angle (rad)
    float day;            // Y axis delta angle (rad)
    float daz;            // Z axis delta angle (rad)
    float q0;             // attitude quaternion
    float q1;             // attitude quaternion
    float q2;             // attitude quaternion
    float q3;             // attitude quaternion
    float dax_b;          // X axis delta angle measurement bias (rad)
    float day_b;          // Y axis delta angle measurement bias (rad)
    float daz_b;          // Z axis delta angle measurement bias (rad)
    float dvz_b;          // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or
    // descending this allows for wind gradient effects. filter height rate
    // using a 10 second time constant filter
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - state->velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or
    // descending this allows for wind gradient effects.
    if (!inhibitWindStates) {
        windVelSigma = dt * constrainf(_windVelProcessNoise, 0.01f, 1.0f) *
                       (1.0f + constrainf(_wndVarHgtRateScale, 0.0f, 1.0f) *
                                   fabsf(hgtRate));
    } else {
        windVelSigma = 0.0f;
    }

    dAngBiasSigma = dt * constrainf(_gyroBiasProcessNoise, 1e-7f, 1e-5f);
    dVelBiasSigma = dt * constrainf(_accelBiasProcessNoise, 1e-5f, 1e-3f);

    if (!inhibitMagStates) {
        magEarthSigma = dt * constrainf(_magEarthProcessNoise, 1e-4f, 1e-2f);
        magBodySigma = dt * constrainf(_magBodyProcessNoise, 1e-4f, 1e-2f);
    } else {
        magEarthSigma = 0.0f;
        magBodySigma = 0.0f;
    }

    for (uint8_t i = 0; i <= 9; i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i = 10; i <= 12; i++) processNoise[i] = dAngBiasSigma;

    // scale gyro bias noise when in static mode to allow for faster bias
    // estimation
    for (uint8_t i = 10; i <= 12; i++) {
        processNoise[i] = dAngBiasSigma;
        if (staticMode) {
            processNoise[i] *= gyroBiasNoiseScaler;
        }
    }

    processNoise[13] = dVelBiasSigma;

    for (uint8_t i = 14; i <= 15; i++) processNoise[i] = windVelSigma;
    for (uint8_t i = 16; i <= 18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i = 19; i <= 21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i = 0; i <= 21; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = state->quat.q0;
    q1 = state->quat.q1;
    q2 = state->quat.q2;
    q3 = state->quat.q3;
    dax_b = state->gyro_bias.x;
    day_b = state->gyro_bias.y;
    daz_b = state->gyro_bias.z;
    dvz_b = IMU1_weighting * state->accel_zbias1 +
            (1.0f - IMU1_weighting) * state->accel_zbias2;
    _gyrNoise = constrainf(_gyrNoise, 1e-3f, 5e-2f);
    daxCov = sq(dt * _gyrNoise);
    dayCov = sq(dt * _gyrNoise);
    dazCov = sq(dt * _gyrNoise);
    _accNoise = constrainf(_accNoise, 5e-2f, 1.0f);
    dvxCov = sq(dt * _accNoise);
    dvyCov = sq(dt * _accNoise);
    dvzCov = sq(dt * _accNoise);

    // calculate the predicted covariance due to inertial sensor error
    // propagation
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
    for (uint8_t i = 0; i <= 21; i++) {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1E6 (1000m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1e6f) {
        for (uint8_t i = 7; i <= 8; i++) {
            for (uint8_t j = 0; j <= 21; j++) {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output and fix numerical errors
    ekf_CopyAndFixCovariances();

    // constrain diagonals to prevent ill-conditioning
    ekf_ConstrainVariances();
}

// this function is used to do a forced alignment of the yaw angle to aligwith the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff without a magnetometer.
void ekf_alignYawGPS(void)
{
    if ((sq(velNED.v[0]) + sq(velNED.v[1])) > 16.0f) {
        float roll;
        float pitch;
        float oldYaw;
        float newYaw;
        float yawErr;

        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        quaternion_to_euler(state->quat, &roll, &pitch, &oldYaw);

        // calculate yaw angle from GPS velocity
        newYaw = atan2f(velNED.v[1], velNED.v[0]);

        // modify yaw angle using GPS ground course if more than 45 degrees away or if not previously aligned
        yawErr = fabsf(newYaw - oldYaw);

        if (((yawErr > 0.7854f) && (yawErr < 5.4978f)) || !yawAligned) {
            // calculate new filter quaternion states from Euler angles
            quaternion_from_euler(&state->quat, roll, pitch, newYaw);

            // the yaw angle is now aligned so update its status
            yawAligned =  true;

            // set the velocity states
            state->velocity.x = velNED.x;
            state->velocity.y = velNED.y;

            // reinitialise the quaternion, velocity and position covariances
            // zero the matrix entries
            ekf_zeroRows(P, 0, 9);
            ekf_zeroCols(P, 0, 9);

            // quaternions - TODO maths that sets them based on different roll, yaw and pitch uncertainties
            P[0][0]   = 1.0e-9f;
            P[1][1]   = 0.25f*sq(DEGREES_TO_RADIANS(1.0f));
            P[2][2]   = 0.25f*sq(DEGREES_TO_RADIANS(1.0f));
            P[3][3]   = 0.25f*sq(DEGREES_TO_RADIANS(1.0f));
            // velocities - we could have a big error coming out of static mode due to GPS lag
            P[4][4]   = 400.0f;
            P[5][5]   = P[4][4];
            P[6][6]   = sq(0.7f);
            // positions - we could have a big error coming out of static mode due to GPS lag
            P[7][7]   = 400.0f;
            P[8][8]   = P[7][7];
            P[9][9]   = sq(5.0f);
        }
    }
}

// Check for filter divergence
void ekf_checkDivergence(void)
{
    // If filter is diverging, then fail for 10 seconds
    // delay checking to allow bias estimate to settle after reset
    // filter divergence is detected by looking for rapid changes in gyro bias
    fpVector3_t tempVec = {.v = {state->gyro_bias.x - lastGyroBias.x,
                                 state->gyro_bias.y - lastGyroBias.y,
                                 state->gyro_bias.z - lastGyroBias.z}};
    float tempLength =
        calc_length_pythagorean_3D(tempVec.x, tempVec.y, tempVec.z);

    if (tempLength != 0.0f) {
        float temp =
            constrainf((P[10][10] + P[11][11] + P[12][12]), 1e-12f, 1e-8f);
        scaledDeltaGyrBiasLgth = (5e-8f / temp) * tempLength / dtIMU;
    }

    bool divergenceDetected = (scaledDeltaGyrBiasLgth > 1.0f);

    lastGyroBias = state->gyro_bias;

    if (imuSampleTime_ms - lastDivergeTime_ms > 10000) {
        if (divergenceDetected) {
            filterDiverged = true;
            faultStatus.diverged = true;
            lastDivergeTime_ms = imuSampleTime_ms;
        } else {
            filterDiverged = false;
        }
    }
}

// calculate whether the flight vehicle is on the ground or flying from height,
// airspeed and GPS speed
void ekf_OnGroundCheck(void)
{
    uint8_t highAirSpd = CENTIMETERS_TO_METERS(getAirspeedEstimate()) > 8.0f;
    float gndSpdSq = sq(velNED.v[0]) + sq(velNED.v[1]);
    uint8_t highGndSpdStage1 = (uint8_t)(gndSpdSq > 9.0f);
    uint8_t highGndSpdStage2 = (uint8_t)(gndSpdSq > 36.0f);
    uint8_t highGndSpdStage3 = (uint8_t)(gndSpdSq > 81.0f);
    uint8_t largeHgt = (uint8_t)(fabsf(hgtMea) > 15.0f);
    uint8_t inAirSum = highAirSpd + highGndSpdStage1 + highGndSpdStage2 +
                       highGndSpdStage3 + largeHgt;

    // detect on-ground to in-air transition
    // if we are already on the ground then 3 or more out of 5 criteria are
    // required if we are in the air then only 2 or more are required this
    // prevents rapid tansitions
    if ((onGround && (inAirSum >= 3)) || (!onGround && (inAirSum >= 2))) {
        onGround = false;
    } else {
        onGround = true;
    }

    // force a yaw alignment if exiting onGround without a compass or if compass
    // is timed out and we are a fly forward vehicle
    if (!onGround && prevOnGround &&
        (!ekf_use_compass() || (magTimeout && STATE(FIXED_WING_LEGACY)))) {
        ekf_alignYawGPS();
    }

    // If we are flying a fly-forward type vehicle without an airspeed sensor
    // and exiting onGround we set the wind velocity to the reciprocal of the
    // velocity vector and scale states so that the wind speed is equal to the
    // 6m/s. This prevents gains being too high at the start of flight if
    // launching into a headwind until the first turn when the EKF can form a
    // wind speed estimate
    if (!onGround && prevOnGround && !ekf_useAirspeed() &&
        STATE(FIXED_WING_LEGACY)) {
        // setWindVelStates();
    }

    // store current on-ground status for next time
    prevOnGround = onGround;

    // If we are on ground, or in static mode, or don't have the right vehicle
    // and sensing to estimate wind, inhibit wind states
    inhibitWindStates = ((!ekf_useAirspeed() && !STATE(FIXED_WING_LEGACY)) ||
                         onGround || staticMode);

    // If magnetometer calibration mode is turned off by the user or we are on
    // ground or in static mode, then inhibit magnetometer states
    inhibitMagStates = (!ekf_use_compass() || onGround || staticMode);
}

bool FLAG_ARMED = false;

// return true if the vehicle code has requested use of static mode
// in static mode, position and height are constrained to zero, allowing an attitude
// reference to be initialised and maintained when on the ground and without GPS lock
bool ekf_static_mode_demanded(void)
{
    return !FLAG_ARMED;
}

// Update Filter States - this should be called whenever new IMU data is
// available
void ekf_UpdateFilter(void)
{
    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // read IMU data and convert to delta angles and velocities
    ekf_readIMUData();

    // detect if filter has diverged and do a dynamic reset using the DCM
    // solution
    ekf_checkDivergence();

    if (filterDiverged) {
        ekf_InitialiseFilterDynamic();
        return;
    }

    // detect if the filter update has been delayed for too long
    if (dtIMU > 0.2f) {
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

    // define rules used to set staticMode
    // staticMode enables ground operation without GPS by fusing zeros for position and height measurements
    if (ekf_static_mode_demanded()) {
        staticMode = true;
    } else {
        staticMode = false;
    }

    // check to see if static mode has changed and reset states if it has
    if (prevStaticMode != staticMode) {
        ekf_ResetVelocity();
        ekf_ResetPosition();
        ekf_ResetHeight();
        ekf_StoreStatesReset();
        ekf_calcQuatAndFieldStates(attitude.values.roll, attitude.values.pitch);
        prevStaticMode = staticMode;
    }

    // run the strapdown INS equations every IMU update
    ekf_UpdateStrapdownEquationsNED();

    // store the predicted states for subsequent use by measurement fusion
    ekf_StoreStates();

    // sum delta angles and time used by covariance prediction
    summedDelAng.x = summedDelAng.x + correctedDelAng.x;
    summedDelAng.y = summedDelAng.y + correctedDelAng.y;
    summedDelAng.z = summedDelAng.z + correctedDelAng.z;
    summedDelVel.x = summedDelVel.x + correctedDelVel1.x;
    summedDelVel.y = summedDelVel.y + correctedDelVel1.y;
    summedDelVel.z = summedDelVel.z + correctedDelVel1.z;
    dt += dtIMU;

    // perform a covariance prediction if the total delta angle has exceeded the
    // limit or the time limit will be exceeded at the next IMU update
    if (((dt >= (covTimeStepMax - dtIMU)) ||
         (calc_length_pythagorean_3D(summedDelAng.x, summedDelAng.y,
                                     summedDelAng.z) > covDelAngMax))) {
        ekf_CovariancePrediction();
        vectorZero(&summedDelAng);
        vectorZero(&summedDelVel);
        dt = 0.0f;
    }

    // Update states using GPS, altimeter, compass, airspeed and synthetic
    // sideslip observations
    ekf_SelectVelPosFusion();
    // SelectMagFusion();
    // SelectTasFusion();
    // SelectBetaFusion();
}

// return the transformation matrix from XYZ (body) to NED axes
void ekf_getRotationBodyToNED(fpMat3_t *mat)
{
    quaternion_to_rotation_matrix(state->quat, mat);
}

// return the Euler roll, pitch and yaw angle in radians
void ekf_getEulerAngles(fpVector3_t *euler)
{
    quaternion_to_euler(state->quat, &euler->x, &euler->y, &euler->z);
}

// return NED velocity in cm/s
void ekf_getVelNED(fpVector3_t *vel)
{
    vel->x = METERS_TO_CENTIMETERS(state->velocity.x);
    vel->y = METERS_TO_CENTIMETERS(state->velocity.y);
    vel->z = METERS_TO_CENTIMETERS(state->velocity.z);
}

// return NED position in cm/s
void ekf_getPosNED(fpVector3_t *pos)
{
    pos->x = METERS_TO_CENTIMETERS(state->position.x);
    pos->y = METERS_TO_CENTIMETERS(state->position.y);
    pos->z = METERS_TO_CENTIMETERS(state->position.z);
}

void ekf_update(float deltaTime)
{
    // limit IMU delta time to prevent numerical problems elsewhere
    dtIMU = constrainf(deltaTime, 0.001f, 1.0f);

    if (!ekf_started) {
        // if we have a GPS lock and more than 6 satellites, we can start the
        // EKF
        // if (isGPSTrustworthy())
        {
            if (start_time_ms == 0) {
                start_time_ms = millis();
            }
            if (millis() - start_time_ms > 10000) {
                ekf_started = true;
                ekf_InitialiseFilterDynamic();
            }
        }
    }

    DEBUG_SET(DEBUG_CRUISE, 4, posTimeout);
    DEBUG_SET(DEBUG_CRUISE, 5, velTimeout);
    DEBUG_SET(DEBUG_CRUISE, 6, hgtTimeout);
    DEBUG_SET(DEBUG_CRUISE, 7, filterDiverged);

    if (ekf_started) {
        ekf_UpdateFilter();
        // ekf_getRotationBodyToNED(&_dcm_matrix);

        if (system_using_EKF()) {
            fpVector3_t eulers;

            ekf_getEulerAngles(&eulers);

            int32_t roll_sensor = RADIANS_TO_DECIDEGREES(eulers.x);
            int32_t pitch_sensor = RADIANS_TO_DECIDEGREES(eulers.y);
            int32_t yaw_sensor = RADIANS_TO_DECIDEGREES(eulers.z);

            if (yaw_sensor < 0) {
                yaw_sensor += 3600;
            }

            if (millis() - start_time_ms > 30000) {
                FLAG_ARMED = true;
            }

            attitude.values.roll = roll_sensor;
            attitude.values.pitch = pitch_sensor;
            attitude.values.yaw = yaw_sensor;

            fpVector3_t _relpos_cm;    // NEU
            fpVector3_t _velocity_cm;  // NEU

            ekf_getPosNED(&_relpos_cm);

            ekf_getVelNED(&_velocity_cm);

            // Inertial Navigation is NEU
            _relpos_cm.z = -_relpos_cm.z;
            _velocity_cm.z = -_velocity_cm.z;

            DEBUG_SET(DEBUG_CRUISE, 0, _relpos_cm.z);
            DEBUG_SET(DEBUG_CRUISE, 1,
                      navGetCurrentActualPositionAndVelocity()->pos.z);
            DEBUG_SET(DEBUG_CRUISE, 2, _velocity_cm.z);
            DEBUG_SET(DEBUG_CRUISE, 3,
                      navGetCurrentActualPositionAndVelocity()->vel.z);

            // keep _gyro_bias for get_gyro_drift()
            /*EKF.getGyroBias(_gyro_bias);
            _gyro_bias = -_gyro_bias;
            _gyro_estimate += _gyro_bias;*/
        }
    }
}