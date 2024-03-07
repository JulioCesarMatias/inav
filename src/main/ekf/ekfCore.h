/*
  24 state EKF based on the derivation in https://github.com/priseborough/
  InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/
  GenerateNavFilterEquations.m

  Converted from Matlab to C++ by Paul Riseborough

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#pragma GCC optimize("O2")

#include <inttypes.h>
#include <stdbool.h>
#include "ekf/ekf.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "common/maths.h"
#include "fc/runtime_config.h"
#include "io/gps_private.h"
#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/navigation_pos_estimator_private.h"
#include "scheduler/scheduler.h"

#define SOLVE_AFTER

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS (1 << 0)
#define MASK_GPS_HDOP (1 << 1)
#define MASK_GPS_SPD_ERR (1 << 2)
#define MASK_GPS_POS_ERR (1 << 3)
#define MASK_GPS_YAW_ERR (1 << 4)
#define MASK_GPS_POS_DRIFT (1 << 5)
#define MASK_GPS_VERT_SPD (1 << 6)
#define MASK_GPS_HORIZ_SPD (1 << 7)

// active height source
#define HGT_SOURCE_BARO 0
#define HGT_SOURCE_RNG 1
#define HGT_SOURCE_GPS 2

// target EKF update time step
#define EKF_TARGET_DT 0.01f

// mag fusion final reset altitude
#define EKF_MAG_FINAL_RESET_ALT 2.5f

// maximum number of yaw resets due to detected magnetic anomaly allowed per flight
#define MAG_ANOMALY_RESET_MAX 2

// number of seconds a request to reset the yaw to the GSF estimate is active before it times out
#define YAW_RESET_TO_GSF_TIMEOUT_MS 5000

// limit on horizontal position states
#define EKF_POSXY_STATE_LIMIT 1.0e6

// Possible values for _flowUse
#define FLOW_USE_NONE 0
#define FLOW_USE_NAV 1
#define FLOW_USE_TERRAIN 2

#define COMPASS_MAX_XYZ_ANG_DIFF DEGREES_TO_RADIANS(90.0f)
#define COMPASS_MAX_XY_ANG_DIFF DEGREES_TO_RADIANS(60.0f)
#define COMPASS_MAX_XY_LENGTH_DIFF 200.0f

typedef float Vector2[2];
typedef float Vector3[3];
typedef float Vector5[5];
typedef float Vector6[6];
typedef float Vector8[8];
typedef float Vector9[9];
typedef float Vector10[10];
typedef float Vector14[14];
typedef float Vector23[23];
typedef float Vector24[24];
typedef float Vector25[25];
typedef float Vector28[28];
typedef float Matrix24[24][24];

typedef union
{
    struct
    {
        bool attitude : 1;           // 0 - true if attitude estimate is valid
        bool horiz_vel : 1;          // 1 - true if horizontal velocity estimate is valid
        bool vert_vel : 1;           // 2 - true if the vertical velocity estimate is valid
        bool horiz_pos_rel : 1;      // 3 - true if the relative horizontal position estimate is valid
        bool horiz_pos_abs : 1;      // 4 - true if the absolute horizontal position estimate is valid
        bool vert_pos : 1;           // 5 - true if the vertical position estimate is valid
        bool terrain_alt : 1;        // 6 - true if the terrain height estimate is valid
        bool const_pos_mode : 1;     // 7 - true if we are in const position mode
        bool pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
        bool pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
        bool takeoff_detected : 1;   // 10 - true if optical flow takeoff has been detected
        bool takeoff : 1;            // 11 - true if filter is compensating for baro errors during takeoff
        bool touchdown : 1;          // 12 - true if filter is compensating for baro errors during touchdown
        bool using_gps : 1;          // 13 - true if we are using GPS position
        bool gps_glitching : 1;      // 14 - true if GPS glitching is affecting navigation accuracy
        bool gps_quality_good : 1;   // 15 - true if we can use GPS for navigation
        bool initalized : 1;         // 16 - true if the EKF has ever been healthy
        bool rejecting_airspeed : 1; // 17 - true if we are rejecting airspeed data
        bool dead_reckoning : 1;     // 18 - true if we are dead reckoning (e.g. no position or velocity source)
    } flags;
    uint32_t value;
} nav_filter_status_t;

// variables used to calculate a vertical velocity that is kinematically consistent with the verical position
typedef struct
{
    float pos;
    float vel;
    float acc;
} vertCompFiltState_t;

// flags indicating severe numerical errors in innovation variance calculation for different fusion operations
typedef struct
{
    bool bad_xmag : 1;
    bool bad_ymag : 1;
    bool bad_zmag : 1;
    bool bad_airspeed : 1;
    bool bad_sideslip : 1;
    bool bad_nvel : 1;
    bool bad_evel : 1;
    bool bad_dvel : 1;
    bool bad_npos : 1;
    bool bad_epos : 1;
    bool bad_dpos : 1;
    bool bad_yaw : 1;
    bool bad_decl : 1;
    bool bad_xflow : 1;
    bool bad_yflow : 1;
    bool bad_rngbcn : 1;
} faultStatus_t;

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
    fpMat3_t DCM;
    fpVector3_t MagPred;
    float R_MAG;
    Vector9 SH_MAG;
} mag_state_t;

typedef struct
{
    fpVector3_t angErr;         // 0..2
    fpVector3_t velocity;       // 3..5
    fpVector3_t position;       // 6..8
    fpVector3_t gyro_bias;      // 9..11
    fpVector3_t gyro_scale;     // 12..14
    float accel_zbias;          // 15
    fpVector3_t earth_magfield; // 16..18
    fpVector3_t body_magfield;  // 19..21
    fpVector2_t wind_vel;       // 22..23
    fpQuaternion_t quat;        // 24..27
} state_elements_t;

typedef union
{
    Vector28 statesArray;
    state_elements_t stateStruct;
} ekfStates_U;

typedef struct
{
    fpQuaternion_t quat;  // 0..3
    fpVector3_t velocity; // 4..6
    fpVector3_t position; // 7..9
} output_elements_t;

typedef struct
{
    fpVector3_t delAng; // 0..2
    fpVector3_t delVel; // 3..5
    float delAngDT;     // 6
    float delVelDT;     // 7
    uint32_t time_ms;   // 8
} imu_elements_t;

typedef struct
{
    uint8_t elsize;
    union
    {
        imu_elements_t *imu_buffer;
        output_elements_t *output_buffer;
    } buffer;
    uint8_t size;
    uint8_t oldest;
    uint8_t youngest;
    bool filled;
} ekf_imu_buffer;

typedef struct
{
    // measurement timestamp (msec)
    uint32_t time_ms;
} EKF_obs_element_t;

typedef struct
{
    fpVector2_t pos;
    float hgt;
    fpVector3_t vel;
    EKF_obs_element_t obs;
} gps_elements_t;

typedef struct
{
    fpVector3_t mag;
    EKF_obs_element_t obs;
} mag_elements_t;

typedef struct
{
    float hgt;
    EKF_obs_element_t obs;
} baro_elements_t;

typedef struct
{
    float rng;
    EKF_obs_element_t obs;
} range_elements_t;

typedef struct
{
    float tas;
    EKF_obs_element_t obs;
} tas_elements_t;

typedef struct
{
    fpVector2_t flowRadXY;
    fpVector2_t flowRadXYcomp;
    fpVector3_t bodyRadXYZ;
    fpVector3_t body_offset;
    float heightOverride;
    EKF_obs_element_t obs;
} of_elements_t;

typedef struct
{
    uint8_t elsize;
    union
    {
        gps_elements_t *gps_buffer;
        mag_elements_t *mag_buffer;
        baro_elements_t *baro_buffer;
        range_elements_t *range_buffer;
        tas_elements_t *tas_buffer;
        of_elements_t *of_buffer;
    } buffer;
    uint8_t size;
    uint8_t oldest;
    uint8_t count;
} ekf_ring_buffer;

typedef enum
{
    IMU_RING_BUFFER = 0,
    OUTPUT_RING_BUFFER,
    GPS_RING_BUFFER,
    MAG_RING_BUFFER,
    BARO_RING_BUFFER,
    RANGE_RING_BUFFER,
    TAS_RING_BUFFER,
    OPTFLOW_RING_BUFFER
} EKF_Buffer_e;

typedef enum
{
    AID_ABSOLUTE = 0, // GPS or some other form of absolute position reference aiding is being used (optical flow may also be used in parallel) so position estimates are absolute.
    AID_NONE = 1,     // no aiding is being used so only attitude and height estimates are available. Either constVelMode or constPosMode must be used to constrain tilt drift.
    AID_RELATIVE = 2, // only optical flow aiding is being used so position estimates will be relative
} AidingMode;

// structure to hold EKF timing statistics
typedef struct
{
    uint32_t count;
    float dtIMUavg_min;
    float dtIMUavg_max;
    float dtEKFavg_min;
    float dtEKFavg_max;
    float delAngDT_max;
    float delAngDT_min;
    float delVelDT_max;
    float delVelDT_min;
} ekf_timing_t;

extern Matrix24 KH;      // intermediate result used for covariance updates
extern Matrix24 KHP;     // intermediate result used for covariance updates
extern Matrix24 nextP;   // Predicted covariance matrix before addition of process noise to diagonals
extern Vector28 Kfusion; // intermediate fusion vector
extern vertCompFiltState_t vertCompFiltState;
extern faultStatus_t faultStatus;
extern mag_state_t mag_state;
extern ekf_timing_t timing; // timing statistics
extern ekfStates_U ekfStates;

// Variables
extern bool statesInitialised; // boolean true when filter states have been initialised
extern bool magHealth;         // boolean true if magnetometer has passed innovation consistency check
extern bool velTimeout;        // boolean true if velocity measurements have failed innovation consistency check and timed out
extern bool posTimeout;        // boolean true if position measurements have failed innovation consistency check and timed out
extern bool hgtTimeout;        // boolean true if height measurements have failed innovation consistency check and timed out
extern bool magTimeout;        // boolean true if magnetometer measurements have failed for too long and have timed out
extern bool tasTimeout;        // boolean true if true airspeed measurements have failed for too long and have timed out
extern bool badIMUdata;        // boolean true if the bad IMU data is detected

extern float gpsNoiseScaler;                 // Used to scale the  GPS measurement noise and consistency gates to compensate for operation with small satellite counts
extern Matrix24 P;                           // covariance matrix
extern ekf_imu_buffer storedIMU;             // IMU data buffer
extern ekf_ring_buffer storedGPS;            // GPS data buffer
extern ekf_ring_buffer storedMag;            // Magnetometer data buffer
extern ekf_ring_buffer storedBaro;           // Baro data buffer
extern ekf_ring_buffer storedTAS;            // TAS data buffer
extern ekf_ring_buffer storedRange;          // Range finder data buffer
extern ekf_imu_buffer storedOutput;          // output state buffer
extern fpMat3_t prevTnb;                     // previous nav to body transformation used for INS earth rotation compensation
extern float accNavMag;                      // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
extern float accNavMagHoriz;                 // magnitude of navigation accel in horizontal plane (m/s^2)
extern fpVector3_t earthRateNED;             // earths angular rate vector in NED (rad/s)
extern float dtIMUavg;                       // expected time between IMU measurements (sec)
extern float dtEkfAvg;                       // expected time between EKF updates (sec)
extern float dt;                             // time lapsed since the last covariance prediction (sec)
extern float hgtRate;                        // state for rate of change of height filter
extern bool onGround;                        // true when the flight vehicle is definitely on the ground
extern bool prevOnGround;                    // value of onGround from previous frame - used to detect transition
extern bool inFlight;                        // true when the vehicle is definitely flying
extern bool prevInFlight;                    // value inFlight from previous frame - used to detect transition
extern bool manoeuvring;                     // boolean true when the flight vehicle is performing horizontal changes in velocity
extern uint32_t airborneDetectTime_ms;       // last time flight movement was detected
extern Vector6 innovVelPos;                  // innovation output for a group of measurements
extern Vector6 varInnovVelPos;               // innovation variance output for a group of measurements
extern Vector6 velPosObs;                    // observations for combined velocity and positon group of measurements (3x1 m , 3x1 m/s)
extern bool fuseVelData;                     // this boolean causes the velNED measurements to be fused
extern bool fusePosData;                     // this boolean causes the posNE measurements to be fused
extern bool fuseHgtData;                     // this boolean causes the hgtMea measurements to be fused
extern fpVector3_t innovMag;                 // innovation output from fusion of X,Y,Z compass measurements
extern fpVector3_t varInnovMag;              // innovation variance output from fusion of X,Y,Z compass measurements
extern float innovVtas;                      // innovation output from fusion of airspeed measurements
extern float varInnovVtas;                   // innovation variance output from fusion of airspeed measurements
extern bool magFusePerformed;                // boolean set to true when magnetometer fusion has been performed in that time step
extern uint32_t prevTasStep_ms;              // time stamp of last TAS fusion step
extern uint32_t prevBetaStep_ms;             // time stamp of last synthetic sideslip fusion step
extern uint32_t lastMagUpdate_us;            // last time compass was updated in usec
extern uint32_t lastMagRead_ms;              // last time compass data was successfully read
extern fpVector3_t velDotNED;                // rate of change of velocity in NED frame
extern fpVector3_t velDotNEDfilt;            // low pass filtered velDotNED
extern uint32_t imuSampleTime_ms;            // time that the last IMU value was taken
extern bool tasDataToFuse;                   // true when new airspeed data is waiting to be fused
extern uint32_t lastBaroReceived_ms;         // time last time we received baro height data
extern uint16_t hgtRetryTime_ms;             // time allowed without use of height measurements before a height timeout is declared
extern uint32_t lastVelPassTime_ms;          // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
extern uint32_t lastPosPassTime_ms;          // time stamp when GPS position measurement last passed innovation consistency check (msec)
extern uint32_t lastHgtPassTime_ms;          // time stamp when height measurement last passed innovation consistency check (msec)
extern uint32_t lastTasPassTime_ms;          // time stamp when airspeed measurement last passed innovation consistency check (msec)
extern uint32_t lastTasFailTime_ms;          // time stamp when airspeed measurement last failed innovation consistency check (msec)
extern uint32_t lastTimeGpsReceived_ms;      // last time we received GPS data
extern uint32_t timeAtLastAuxEKF_ms;         // last time the auxiliary filter was run to fuse range or optical flow measurements
extern uint32_t lastHealthyMagTime_ms;       // time the magnetometer was last declared healthy
extern bool magSensorFailed;                 // true if all magnetometer sensors have timed out on this flight and we are no longer using magnetometer data
extern uint32_t lastYawTime_ms;              // time stamp when yaw observation was last fused (msec)
extern uint32_t ekfStartTime_ms;             // time the EKF was started (msec)
extern fpVector2_t lastKnownPositionNE;      // last known position
extern float velTestRatio;                   // sum of squares of GPS velocity innovation divided by fail threshold
extern float posTestRatio;                   // sum of squares of GPS position innovation divided by fail threshold
extern float hgtTestRatio;                   // sum of squares of baro height innovation divided by fail threshold
extern fpVector3_t magTestRatio;             // sum of squares of magnetometer innovations divided by fail threshold
extern float tasTestRatio;                   // sum of squares of true airspeed innovation divided by fail threshold
extern float defaultAirSpeed;                // default equivalent airspeed in m/s to be used if the measurement is unavailable. Do not use if not positive.
extern bool inhibitWindStates;               // true when wind states and covariances are to remain constant
extern bool inhibitMagStates;                // true when magnetic field states and covariances are to remain constant
extern bool lastInhibitMagStates;            // previous inhibitMagStates
extern bool needMagBodyVarReset;             // we need to reset mag body variances at next CovariancePrediction
extern bool gpsNotAvailable;                 // bool true when valid GPS data is not available
extern gpsLocation_t EKF_origin;             // LLH origin of the NED axis system
extern bool validOrigin;                     // true when the EKF origin is valid
extern float gpsSpdAccuracy;                 // estimated speed accuracy in m/s returned by the GPS receiver
extern float gpsPosAccuracy;                 // estimated position accuracy in m returned by the GPS receiver
extern float gpsHgtAccuracy;                 // estimated height accuracy in m returned by the GPS receiver
extern uint32_t lastGpsVelFail_ms;           // time of last GPS vertical velocity consistency check fail
extern uint32_t lastGpsVelPass_ms;           // time of last GPS vertical velocity consistency check pass
extern uint32_t lastGpsAidBadTime_ms;        // time in msec gps aiding was last detected to be bad
extern float posDownAtTakeoff;               // flight vehicle vertical position sampled at transition from on-ground to in-air and used as a reference (m)
extern bool useGpsVertVel;                   // true if GPS vertical velocity should be used
extern float yawResetAngle;                  // Change in yaw angle due to last in-flight yaw reset in radians. A positive value means the yaw angle has increased.
extern uint32_t lastYawReset_ms;             // System time at which the last yaw reset occurred. Returned by getLastYawResetAngle
extern fpVector3_t tiltErrVec;               // Vector of most recent attitude error correction from Vel,Pos fusion
extern float tiltErrFilt;                    // Filtered tilt error metric
extern bool tiltAlignComplete;               // true when tilt alignment is complete
extern bool yawAlignComplete;                // true when yaw alignment is complete
extern bool magStateInitComplete;            // true when the magnetic field sttes have been initialised
extern uint8_t stateIndexLim;                // Max state index used during matrix and array operations
extern imu_elements_t imuDataDelayed;        // IMU data at the fusion time horizon
extern imu_elements_t imuDataNew;            // IMU data at the current time horizon
extern imu_elements_t imuDataDownSampledNew; // IMU data at the current time horizon that has been downsampled to a 100Hz rate
extern fpQuaternion_t imuQuatDownSampleNew;  // Quaternion obtained by rotating through the IMU delta angles since the start of the current down sampled frame
extern baro_elements_t baroDataNew;          // Baro data at the current time horizon
extern baro_elements_t baroDataDelayed;      // Baro data at the fusion time horizon
extern range_elements_t rangeDataNew;        // Range finder data at the current time horizon
extern range_elements_t rangeDataDelayed;    // Range finder data at the fusion time horizon
extern tas_elements_t tasDataNew;            // TAS data at the current time horizon
extern tas_elements_t tasDataDelayed;        // TAS data at the fusion time horizon
extern mag_elements_t magDataDelayed;        // Magnetometer data at the fusion time horizon
extern gps_elements_t gpsDataNew;            // GPS data at the current time horizon
extern gps_elements_t gpsDataDelayed;        // GPS data at the fusion time horizon
extern output_elements_t outputDataNew;      // output state data at the current time step
extern output_elements_t outputDataDelayed;  // output state data at the current time step
extern fpVector3_t delAngCorrection;         // correction applied to delta angles used by output observer to track the EKF
extern fpVector3_t velErrintegral;           // integral of output predictor NED velocity tracking error (m)
extern fpVector3_t posErrintegral;           // integral of output predictor NED position tracking error (m.sec)
extern float innovYaw;                       // compass yaw angle innovation (rad)
extern uint32_t timeTasReceived_ms;          // time last TAS data was received (msec)
extern bool gpsGoodToAlign;                  // true when the GPS quality can be used to initialise the navigation system
extern uint32_t magYawResetTimer_ms;         // timer in msec used to track how long good magnetometer data is failing innovation consistency checks
extern bool consistentMagData;               // true when the magnetometers are passing consistency checks
extern bool motorsArmed;                     // true when the motors have been armed
extern bool prevMotorsArmed;                 // value of motorsArmed from previous frame
extern bool posVelFusionDelayed;             // true when the position and velocity fusion has been delayed
extern bool optFlowFusionDelayed;            // true when the optical flow fusion has been delayed
extern bool airSpdFusionDelayed;             // true when the air speed fusion has been delayed
extern bool sideSlipFusionDelayed;           // true when the sideslip fusion has been delayed
extern fpVector3_t lastMagOffsets;           // Last magnetometer offsets from COMPASS_ parameters. Used to detect parameter changes.
extern bool lastMagOffsetsValid;             // True when lastMagOffsets has been initialized
extern fpVector2_t posResetNE;               // Change in North/East position due to last in-flight reset in metres. Returned by getLastPosNorthEastReset
extern uint32_t lastPosReset_ms;             // System time at which the last position reset occurred. Returned by getLastPosNorthEastReset
extern fpVector2_t velResetNE;               // Change in North/East velocity due to last in-flight reset in metres/sec. Returned by getLastVelNorthEastReset
extern uint32_t lastVelReset_ms;             // System time at which the last velocity reset occurred. Returned by getLastVelNorthEastReset
extern float posResetD;                      // Change in Down position due to last in-flight reset in metres. Returned by getLastPosDowntReset
extern uint32_t lastPosResetD_ms;            // System time at which the last position reset occurred. Returned by getLastPosDownReset
extern float yawTestRatio;                   // square of magnetometer yaw angle innovation divided by fail threshold
extern fpQuaternion_t prevQuatMagReset;      // Quaternion from the last time the magnetic field state reset condition test was performed
extern float hgtInnovFiltState;              // state used for fitering of the height innovations used for pre-flight checks
extern bool runUpdates;                      // boolean true when the EKF updates can be run
extern uint32_t framesSincePredict;          // number of frames lapsed since EKF instance did a state prediction
extern bool startPredictEnabled;             // boolean true when the frontend has given permission to start a new state prediciton cycele
extern uint8_t localFilterTimeStep_ms;       // average number of msec between filter updates
extern float posDownObsNoise;                // observation noise variance on the vertical position used by the state and covariance update step (m^2)
extern fpVector3_t delAngCorrected;          // corrected IMU delta angle vector at the EKF time horizon (rad)
extern fpVector3_t delVelCorrected;          // corrected IMU delta velocity vector at the EKF time horizon (m/s)
extern bool magFieldLearned;                 // true when the magnetic field has been learned
extern uint32_t wasLearningCompass_ms;       // time when we were last waiting for compass learn to complete
extern fpVector3_t earthMagFieldVar;         // NED earth mag field variances for last learned field (mGauss^2)
extern fpVector3_t bodyMagFieldVar;          // XYZ body mag field variances for last learned field (mGauss^2)
extern bool delAngBiasLearned;               // true when the gyro bias has been learned
extern nav_filter_status_t filterStatus;     // contains the status of various filter outputs
extern float ekfOriginHgtVar;                // Variance of the EKF WGS-84 origin height estimate (m^2)
extern float ekfGpsRefHgt;                   // floating point representation of the WGS-84 reference height used to convert GPS height to local height (m)
extern uint32_t lastOriginHgtTime_ms;        // last time the ekf's WGS-84 origin height was corrected
extern uint8_t imu_buffer_length;            // Reference to the global EKF frontend for parameters

// variables used by the pre-initialisation GPS checks
extern gpsLocation_t gpsloc_prev;            // LLH location of previous GPS measurement
extern uint32_t lastPreAlignGpsCheckTime_ms; // last time in msec the GPS quality was checked during pre alignment checks
extern float gpsDriftNE;                     // amount of drift detected in the GPS position during pre-flight GPs checks
extern float gpsVertVelFilt;                 // amount of filterred vertical GPS velocity detected durng pre-flight GPS checks
extern float gpsHorizVelFilt;                // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

// variable used by the in-flight GPS quality check
extern bool gpsSpdAccPass;            // true when reported GPS speed accuracy passes in-flight checks
extern bool ekfInnovationsPass;       // true when GPS innovations pass in-flight checks
extern float sAccFilterState1;        // state variable for LPF applid to reported GPS speed accuracy
extern float sAccFilterState2;        // state variable for peak hold filter applied to reported GPS speed
extern uint32_t lastGpsCheckTime_ms;  // last time in msec the GPS quality was checked
extern uint32_t lastInnovPassTime_ms; // last time in msec the GPS innovations passed
extern uint32_t lastInnovFailTime_ms; // last time in msec the GPS innovations failed
extern bool gpsAccuracyGood;          // true when the GPS accuracy is considered to be good enough for safe flight.

// variables added for optical flow fusion
extern ekf_ring_buffer storedOF;     // OF data buffer
extern of_elements_t ofDataNew;      // OF data at the current time horizon
extern of_elements_t ofDataDelayed;  // OF data at the fusion time horizon
extern bool flowDataToFuse;          // true when optical flow data is ready for fusion
extern bool flowDataValid;           // true while optical flow data is still fresh
extern fpVector2_t auxFlowObsInnov;  // optical flow rate innovation from 1-state terrain offset estimator
extern uint32_t flowValidMeaTime_ms; // time stamp from latest valid flow measurement (msec)
extern uint32_t rngValidMeaTime_ms;  // time stamp from latest valid range measurement (msec)
extern uint32_t flowMeaTime_ms;      // time stamp from latest flow measurement (msec)
extern uint32_t gndHgtValidTime_ms;  // time stamp from last terrain offset state update (msec)
extern fpMat3_t Tbn_flow;            // transformation matrix from body to nav axes at the middle of the optical flow sample period
extern Vector2 varInnovOptFlow;      // optical flow innovations variances (rad/sec)^2
extern Vector2 innovOptFlow;         // optical flow LOS innovations (rad/sec)
extern float Popt;                   // Optical flow terrain height state covariance (m^2)
extern float terrainState;           // terrain position state (m)
extern float prevPosN;               // north position at last measurement
extern float prevPosE;               // east position at last measurement
extern float varInnovRng;            // range finder observation innovation variance (m^2)
extern float innovRng;               // range finder observation innovation (m)
extern float hgtMea;                 // height measurement derived from either baro, gps or range finder data (m)
extern bool inhibitGndState;         // true when the terrain position state is to remain constant
extern uint32_t prevFlowFuseTime_ms; // time both flow measurement components passed their innovation consistency checks
extern Vector2 flowTestRatio;        // square of optical flow innovations divided by fail threshold used by main filter where >1.0 is a fail
extern fpVector2_t auxFlowTestRatio; // sum of squares of optical flow innovation divided by fail threshold used by 1-state terrain offset estimator
extern float R_LOS;                  // variance of optical flow rate measurements (rad/sec)^2
extern float auxRngTestRatio;        // square of range finder innovations divided by fail threshold used by main filter where >1.0 is a fail
extern fpVector2_t flowGyroBias;     // bias error of optical flow sensor gyro output
extern bool rangeDataToFuse;         // true when valid range finder height data has arrived at the fusion time horizon.
extern bool baroDataToFuse;          // true when valid baro height finder data has arrived at the fusion time horizon.
extern bool gpsDataToFuse;           // true when valid GPS data has arrived at the fusion time horizon.
extern bool magDataToFuse;           // true when valid magnetometer data has arrived at the fusion time horizon

extern AidingMode PV_AidingMode;     // Defines the preferred mode for aiding of velocity and position estimates from the INS
extern AidingMode PV_AidingModePrev; // Value of PV_AidingMode from the previous frame - used to detect transitions
extern bool gndOffsetValid;          // true when the ground offset state can still be considered valid
extern fpVector3_t delAngBodyOF;     // bias corrected delta angle of the vehicle IMU measured summed across the time since the last OF measurement
extern float delTimeOF;              // time that delAngBodyOF is summed across

// Range finder
extern float baroHgtOffset;              // offset applied when when switching to use of Baro height
extern float rngOnGnd;                   // Expected range finder reading in metres when vehicle is on ground
extern uint32_t lastRngMeasTime_ms;      // Timestamp of last range measurement
extern bool terrainHgtStable;            // true when the terrain height is stable enough to be used as a height reference
extern float storedRngMeas[3];           // Ringbuffer of stored range measurements for range sensor
extern uint32_t storedRngMeasTime_ms[3]; // Ringbuffers of stored range measurement times for range sensor
extern uint8_t rngMeasIndex;             // Current range measurement ringbuffer index for range sensor

// height source selection logic
extern uint8_t activeHgtSource; // integer defining active height source

// Movement detector
extern bool takeOffDetected;     // true when takeoff for optical flow navigation has been detected
extern float rngAtStartOfFlight; // range finder measurement at start of flight
extern uint32_t timeAtArming_ms; // time in msec that the vehicle armed

// baro ground effect
extern float meaHgtAtTakeOff; // height measured at commencement of takeoff

// control of post takeoff magnetic field and heading resets
extern bool finalInflightYawInit;         // true when the final post takeoff initialisation of yaw angle has been performed
extern bool finalInflightMagInit;         // true when the final post takeoff initialisation of magnetic field states been performed
extern bool magStateResetRequest;         // true if magnetic field states need to be reset using the magneteomter measurements
extern bool magYawResetRequest;           // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
extern bool gpsYawResetRequest;           // true if the vehicle yaw needs to be reset to the GPS course
extern float posDownAtLastMagReset;       // vertical position last time the mag states were reset (m)
extern float yawInnovAtLastMagReset;      // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
extern fpQuaternion_t quatAtLastMagReset; // quaternion states last time the mag states were reset
extern uint8_t magYawAnomallyCount;       // Number of times the yaw has been reset due to a magnetic anomaly during initial ascent

// string for using EKF messages in the OSD
extern char ekf_status_string[50];

// earth field from WMM tables
extern bool have_table_earth_field;      // true when we have initialised table_earth_field_ga
extern fpVector3_t table_earth_field_ga; // earth field from WMM tables
extern float table_declination;          // declination in radians from the tables

// when was attitude filter status last non-zero?
extern uint32_t last_filter_ok_ms;

// The following declarations are used to control when the main navigation filter resets it's yaw to the estimate provided by the GSF
extern uint32_t EKFGSF_yaw_reset_ms;         // timestamp of last emergency yaw reset (uSec)
extern uint32_t EKFGSF_yaw_reset_request_ms; // timestamp of last emergency yaw reset request (uSec)
extern uint8_t EKFGSF_yaw_reset_count;       // number of emergency yaw resets performed
extern bool EKFGSF_run_filterbank;           // true when the filter bank is active

bool setup_backend(void);

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool coreInitialiseFilterBootstrap(void);

// Update Filter States - this should be called whenever new IMU data is available
// The predict flag is set true when a new prediction cycle can be started
void coreUpdateFilter(bool predict);

// Check basic filter health metrics and return a consolidated health status
bool coreHealthy(void);

// Write the last calculated NE position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool coreGetPosNE(fpVector2_t *posNE);

// Write the last calculated D position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool coreGetPosD(float *posD);

// return NED velocity in m/s
void coreGetVelNED(fpVector3_t *vel);

// return estimate of true airspeed vector in body frame in m/s
// returns false if estimate is unavailable
bool coreGetAirSpdVec(fpVector3_t *vel);

// Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
// This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
// but will always be kinematically consistent with the z component of the EKF position state
float coreGetPosDownDerivative(void);

// return body axis gyro bias estimates in rad/sec
void coreGetGyroBias(fpVector3_t *gyroBias);

// return body axis gyro scale factor error as a percentage
void getGyroScaleErrorPercentage(fpVector3_t *gyroScale);

// reset body axis gyro bias estimates
void coreResetGyroBias(void);

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void getEkfControlLimits(float *ekfGndSpdLimit, float *ekfNavVelGainScaler);

// return the Z-accel bias estimate in m/s^2
void getAccelZBias(float *zbias);

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void getWind(fpVector3_t *wind);

// return earth magnetic field estimates in measurement units / 1000
void getMagNED(fpVector3_t *magNED);

// return body magnetic field estimates in measurement units / 1000
void getMagXYZ(fpVector3_t *magXYZ);

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool getMagOffsets(fpVector3_t *magOffsets);

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool getLLH(gpsLocation_t *loc);

// return the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool getOriginLLH(gpsLocation_t *loc);

// set the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter will be relative to this location
// returns false if absolute aiding and GPS is being used or if the origin is already set
bool controlSetOriginLLH(gpsLocation_t *loc);

// return estimated height above ground level
// return false if ground height is not being estimated.
bool getHAGL(float *HAGL);

// return the Euler roll, pitch and yaw angle in radians
void getEulerAngles(fpVector3_t *eulers);

// return the transformation matrix from XYZ (body) to NED axes
void getRotationBodyToNED(fpMat3_t *mat);

// return the quaternions defining the rotation from NED to XYZ (body) axes
void getQuaternion(fpQuaternion_t *quat);

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
bool getInnovations(fpVector3_t *velInnov, fpVector3_t *posInnov, fpVector3_t *magInnov, float *tasInnov, float *yawInnov);

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
bool getVariances(float *velVar, float *posVar, float *hgtVar, fpVector3_t *magVar, float *tasVar, fpVector2_t *offset);

// should we use the compass? This is public so it can be used for
// reporting via ahrs.ekf_useCompass()
bool ekf_useCompass(void);

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference. Use to prevent range finder operation otherwise
// enabled by the combination of EKF_RNG_AID_HGT and EKF_RNG_USE_SPD parameters.
void setTerrainHgtStable(bool val);

/*
return the filter fault status as a bitmasked integer
 0 = quaternions are NaN
 1 = velocities are NaN
 2 = badly conditioned X magnetometer fusion
 3 = badly conditioned Y magnetometer fusion
 4 = badly conditioned Z magnetometer fusion
 5 = badly conditioned airspeed fusion
 6 = badly conditioned synthetic sideslip fusion
 7 = filter is not initialised
*/
void getFilterFaults(uint16_t *faults);

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool getHeightControlLimit(float *height);

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t getLastYawResetAngle(float *yawAng);

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t getLastPosNorthEastReset(fpVector2_t *pos);

// return the amount of D position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t getLastPosDownReset(float *posD);

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t getLastVelNorthEastReset(fpVector2_t *vel);

// report the number of frames lapsed since the last state prediction
// this is used by other instances to level load
uint8_t getFramesSincePredict(void);

// Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
void writeDefaultAirSpeed(float airspeed);

// request a reset the yaw to the EKF-GSF value
void EKFGSF_requestYawReset(void);

// update the navigation filter status
void updateFilterStatus(void);

// update the quaternion, velocity and position states using IMU measurements
void UpdateStrapdownEquationsNED(void);

// calculate the predicted state covariance matrix
void CovariancePrediction(void);

// force symmetry on the state covariance matrix
void ForceSymmetry(void);

// copy covariances across from covariance prediction calculation and fix numerical errors
void CopyCovariances(void);

// constrain variances (diagonal terms) in the state covariance matrix
void ConstrainVariances(void);

// constrain states
void ConstrainStates(void);

// constrain earth field using WMM tables
void MagTableConstrain(void);

// fuse selected position, velocity and height measurements
void FuseVelPosNED(void);

// fuse magnetometer measurements
void FuseMagnetometer(void);

// fuse true airspeed measurements
void FuseAirspeed(void);

// fuse synthetic sideslip measurement of zero
void FuseSideslip(void);

// zero specified range of rows in the state covariance matrix
void zeroRows(Matrix24 covMat, uint8_t first, uint8_t last);

// zero specified range of columns in the state covariance matrix
void zeroCols(Matrix24 covMat, uint8_t first, uint8_t last);

// Reset the stored output history to current data
void StoreOutputReset(void);

// Rotate the stored output quaternion history through a quaternion rotation
void StoreQuatRotate(fpQuaternion_t *deltaQuat);

// calculate the NED earth spin vector in rad/sec
void calcEarthRateNED(fpVector3_t *omega, int32_t latitude);

// initialise the covariance matrix
void CovarianceInit(void);

// helper functions for readIMUData
void readDeltaVelocity(fpVector3_t *dVel, float *dVel_dt);
void readDeltaAngle(fpVector3_t *dAng, float *dAng_dt);

// helper functions for correcting IMU data
void correctDeltaAngle(fpVector3_t *delAng, float delAngDT);
void correctDeltaVelocity(fpVector3_t *delVel, float delVelDT);

// update IMU delta angle and delta velocity measurements
void readIMUData(void);

// check for new valid GPS data and update stored measurement if available
void readGpsData(void);

// check for new altitude measurement data and update stored measurement if available
void readBaroData(void);

// check for new magnetometer data and update store measurements if available
void readMagData(void);

// check for new airspeed data and update stored measurements if available
void readAirSpdData(void);

// determine when to perform fusion of GPS position and  velocity measurements
void SelectVelPosFusion(void);

// determine when to perform fusion of magnetometer measurements
void SelectMagFusion(void);

// determine when to perform fusion of true airspeed measurements
void SelectTasFusion(void);

// determine when to perform fusion of synthetic sideslp measurements
void SelectBetaFusion(void);

// force alignment of the yaw angle using GPS velocity data
void realignYawGPS(void);

// initialise the earth magnetic field states using declination and current attitude and magnetometer measurements
// and return attitude quaternion
fpQuaternion_t calcQuatAndFieldStates(float roll, float pitch);

// zero stored variables
void InitialiseVariables(void);

void InitialiseVariablesMag(void);

// reset the horizontal position states uing the last GPS measurement
void ResetPosition(void);

// reset velocity states using the last GPS measurement
void ResetVelocity(void);

// reset the vertical position state using the last height measurement
void ResetHeight(void);

// return true if we should use the airspeed sensor
bool useAirspeed(void);

// return true if the vehicle code has requested the filter to be ready for flight
bool readyToUseGPS(void);

// return true if optical flow data is available
bool optFlowDataPresent(void);

// determine when to perform fusion of optical flow measurements
void SelectFlowFusion(void);

// Estimate terrain offset using a single state EKF
void EstimateTerrainOffset(void);

// fuse optical flow measurements into the main filter
void FuseOptFlow(void);

// Control filter mode changes
void controlFilterModes(void);

// Determine if we are flying or on the ground
void detectFlight(void);

// Set inertial navigaton aiding mode
void setAidingMode(void);

// Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
// avoid unnecessary operations
void setWindMagStateLearningMode(void);

// Check the alignmnent status of the tilt attitude
// Used during initial bootstrap alignment of the filter
void checkAttitudeAlignmentStatus(void);

// Control reset of yaw and magnetic field states
void controlMagYawReset(void);

// set the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter will be relative to this location
// returns false if the origin has already been set
bool setOrigin(gpsLocation_t *loc);

// Assess GPS data quality and set gpsGoodToAlign if good enough to align the EKF
void calcGpsGoodToAlign(void);

// return true and set the class variable true if the delta angle bias has been learned
bool checkGyroCalStatus(void);

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
void calcGpsGoodForFlight(void);

// Read the range finder and take new measurements if available
// Apply a median filter to range finder data
void readRangeFinder(void);

// check if the vehicle has taken off during optical flow navigation by looking at inertial and range finder data
void detectOptFlowTakeoff(void);

// align the NE earth magnetic field states with the published declination
void alignMagStateDeclination(void);

// Fuse compass measurements using a simple declination observation (doesn't require magnetic field states)
void fuseEulerYaw(void);

// Fuse declination angle to keep earth field declination from changing when we don't have earth relative observations.
// Input is 1-sigma uncertainty in published declination
void FuseDeclination(float declErr);

// return magnetic declination in radians
float MagDeclination(void);

// Propagate PVA solution forward from the fusion time horizon to the current time horizon
// using a simple observer
void calcOutputStates(void);

// calculate a filtered offset between baro height measurement and EKF height estimate
void calcFiltBaroOffset(void);

// correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
void correctEkfOriginHeight(void);

// Select height data to be fused from the available baro, range finder and GPS sources
void selectHeightForFusion(void);

// zero attitude state covariances, but preserve variances
void zeroAttCovOnly(void);

// record a yaw reset event
void recordYawReset(void);

// record a magnetic field state reset event
void recordMagReset(void);

// update timing statistics structure
void updateTimingStatistics(void);

// Runs the IMU prediction step for an independent GSF yaw estimator algorithm
// that uses IMU, GPS horizontal velocity and optionally true airspeed data.
void runYawEstimatorPrediction(void);

// Run the GPS velocity correction step for the GSF yaw estimator and use the
// yaw estimate to reset the main EKF yaw if requested
void runYawEstimatorCorrection(void);

// reset the quaternion states using the supplied yaw angle, maintaining the previous roll and pitch
// also reset the body to nav frame rotation matrix
// reset the quaternion state covariances using the supplied yaw variance
// yaw          : new yaw angle (rad)
// yaw_variance : variance of new yaw angle (rad^2)
// isDeltaYaw   : true when the yaw should be added to the existing yaw angle
void resetQuatStateYawOnly(float yaw, float yawVariance, bool isDeltaYaw);

// attempt to reset the yaw to the EKF-GSF value
// returns false if unsuccessful
bool EKFGSF_resetMainFilterYaw(void);

/*
Return a filter function status that indicates:
    Which outputs are valid
    If the filter has detected takeoff
    If the filter has activated the mode that mitigates against ground effect static pressure errors
    If GPS data is being used
*/
void getFilterStatus(nav_filter_status_t *status);

// return true if we are tilt aligned
static inline bool have_aligned_tilt(void)
{
    return tiltAlignComplete;
}

// return true if we are yaw aligned
static inline bool have_aligned_yaw(void)
{
    return yawAlignComplete;
}

// should we assume zero sideslip?
bool assume_zero_sideslip(void);

// vehicle specific initial gyro bias uncertainty
float InitialGyroBiasUncertainty(void);

fpVector2_t get_distance_NE(gpsLocation_t EKF_origin, gpsLocation_t loc);
float get_horizontal_distance(gpsLocation_t actualLoc, gpsLocation_t prevLoc);
void offset_latlng(int32_t *lat, int32_t *lng, float ofs_north, float ofs_east);

static inline float rangeFinderMaxAltitude(void)
{
    return 700.0f; // in cm
}