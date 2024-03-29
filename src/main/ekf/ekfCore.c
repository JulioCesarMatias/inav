#include "ekf/ekf.h"
#include "ekf/ekfCore.h"
#include "ekf/ekfBuffer.h"
#include "common/log.h"
#include "common/utils.h"
#include "drivers/time.h"

// Variables
bool statesInitialised; // boolean true when filter states have been initialised
bool magHealth;         // boolean true if magnetometer has passed innovation consistency check
bool velTimeout;        // boolean true if velocity measurements have failed innovation consistency check and timed out
bool posTimeout;        // boolean true if position measurements have failed innovation consistency check and timed out
bool hgtTimeout;        // boolean true if height measurements have failed innovation consistency check and timed out
bool magTimeout;        // boolean true if magnetometer measurements have failed for too long and have timed out
bool tasTimeout;        // boolean true if true airspeed measurements have failed for too long and have timed out
bool badIMUdata;        // boolean true if the bad IMU data is detected

Matrix24 KH;      // intermediate result used for covariance updates
Matrix24 KHP;     // intermediate result used for covariance updates
Matrix24 nextP;   // Predicted covariance matrix before addition of process noise to diagonals
Vector28 Kfusion; // intermediate fusion vector
Matrix24 P;       // covariance matrix

uint8_t imu_buffer_length;
ekf_imu_buffer storedIMU;    // IMU data buffer
ekf_ring_buffer storedGPS;   // GPS data buffer
ekf_ring_buffer storedMag;   // Magnetometer data buffer
ekf_ring_buffer storedBaro;  // Baro data buffer
ekf_ring_buffer storedTAS;   // TAS data buffer
ekf_ring_buffer storedRange; // Range finder data buffer
ekf_imu_buffer storedOutput; // output state buffer
fpMat3_t prevTnb;                     // previous nav to body transformation used for INS earth rotation compensation
float accNavMag;                      // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
float accNavMagHoriz;                 // magnitude of navigation accel in horizontal plane (m/s^2)
fpVector3_t earthRateNED;             // earths angular rate vector in NED (rad/s)
float dtIMUavg;                       // expected time between IMU measurements (sec)
float dtEkfAvg;                       // expected time between EKF updates (sec)
float dt;                             // time lapsed since the last covariance prediction (sec)
float hgtRate;                        // state for rate of change of height filter
float gpsNoiseScaler;                 // Used to scale the GPS measurement noise and consistency gates to compensate for operation with small satellite counts
bool onGround;                        // true when the flight vehicle is definitely on the ground
bool prevOnGround;                    // value of onGround from previous frame - used to detect transition
bool inFlight;                        // true when the vehicle is definitely flying
bool prevInFlight;                    // value inFlight from previous frame - used to detect transition
bool manoeuvring;                     // boolean true when the flight vehicle is performing horizontal changes in velocity
uint32_t airborneDetectTime_ms;       // last time flight movement was detected
Vector6 innovVelPos;                  // innovation output for a group of measurements
Vector6 varInnovVelPos;               // innovation variance output for a group of measurements
Vector6 velPosObs;                    // observations for combined velocity and positon group of measurements (3x1 m , 3x1 m/s)
bool fuseVelData;                     // this boolean causes the velNED measurements to be fused
bool fusePosData;                     // this boolean causes the posNE measurements to be fused
bool fuseHgtData;                     // this boolean causes the hgtMea measurements to be fused
fpVector3_t innovMag;                 // innovation output from fusion of X,Y,Z compass measurements
fpVector3_t varInnovMag;              // innovation variance output from fusion of X,Y,Z compass measurements
float innovVtas;                      // innovation output from fusion of airspeed measurements
float varInnovVtas;                   // innovation variance output from fusion of airspeed measurements
bool magFusePerformed;                // boolean set to true when magnetometer fusion has been performed in that time step
uint32_t prevTasStep_ms;              // time stamp of last TAS fusion step
uint32_t prevBetaStep_ms;             // time stamp of last synthetic sideslip fusion step
uint32_t lastMagUpdate_us;            // last time compass was updated in usec
uint32_t lastMagRead_ms;              // last time compass data was successfully read
fpVector3_t velDotNED;                // rate of change of velocity in NED frame
fpVector3_t velDotNEDfilt;            // low pass filtered velDotNED
uint32_t imuSampleTime_ms;            // time that the last IMU value was taken
bool tasDataToFuse;                   // true when new airspeed data is waiting to be fused
uint32_t lastBaroReceived_ms;         // time last time we received baro height data
uint16_t hgtRetryTime_ms;             // time allowed without use of height measurements before a height timeout is declared
uint32_t lastVelPassTime_ms;          // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
uint32_t lastPosPassTime_ms;          // time stamp when GPS position measurement last passed innovation consistency check (msec)
uint32_t lastHgtPassTime_ms;          // time stamp when height measurement last passed innovation consistency check (msec)
uint32_t lastTasPassTime_ms;          // time stamp when airspeed measurement last passed innovation consistency check (msec)
uint32_t lastTasFailTime_ms;          // time stamp when airspeed measurement last failed innovation consistency check (msec)
uint32_t lastTimeGpsReceived_ms;      // last time we received GPS data
uint32_t timeAtLastAuxEKF_ms;         // last time the auxiliary filter was run to fuse range or optical flow measurements
uint32_t lastHealthyMagTime_ms;       // time the magnetometer was last declared healthy
bool magSensorFailed;                 // true if all magnetometer sensors have timed out on this flight and we are no longer using magnetometer data
uint32_t lastYawTime_ms;              // time stamp when yaw observation was last fused (msec)
uint64_t ekfStartTime_us;             // time the EKF was started (usec)
fpVector2_t lastKnownPositionNE;      // last known position
float velTestRatio;                   // sum of squares of GPS velocity innovation divided by fail threshold
float posTestRatio;                   // sum of squares of GPS position innovation divided by fail threshold
float hgtTestRatio;                   // sum of squares of baro height innovation divided by fail threshold
fpVector3_t magTestRatio;             // sum of squares of magnetometer innovations divided by fail threshold
float tasTestRatio;                   // sum of squares of true airspeed innovation divided by fail threshold
float defaultAirSpeed;                // default equivalent airspeed in m/s to be used if the measurement is unavailable. Do not use if not positive.
bool inhibitWindStates;               // true when wind states and covariances are to remain constant
bool inhibitMagStates;                // true when magnetic field states and covariances are to remain constant
bool lastInhibitMagStates;            // previous inhibitMagStates
bool needMagBodyVarReset;             // we need to reset mag body variances at next CovariancePrediction
bool gpsNotAvailable;                 // bool true when valid GPS data is not available
gpsLocation_t EKF_origin;             // LLH origin of the NED axis system
bool validOrigin;                     // true when the EKF origin is valid
float gpsSpdAccuracy;                 // estimated speed accuracy in m/s returned by the GPS receiver
float gpsPosAccuracy;                 // estimated position accuracy in m returned by the GPS receiver
float gpsHgtAccuracy;                 // estimated height accuracy in m returned by the GPS receiver
uint32_t lastGpsVelFail_ms;           // time of last GPS vertical velocity consistency check fail
uint32_t lastGpsVelPass_ms;           // time of last GPS vertical velocity consistency check pass
uint32_t lastGpsAidBadTime_ms;        // time in msec gps aiding was last detected to be bad
float posDownAtTakeoff;               // flight vehicle vertical position sampled at transition from on-ground to in-air and used as a reference (m)
bool useGpsVertVel;                   // true if GPS vertical velocity should be used
float yawResetAngle;                  // Change in yaw angle due to last in-flight yaw reset in radians. A positive value means the yaw angle has increased.
uint32_t lastYawReset_ms;             // System time at which the last yaw reset occurred. Returned by getLastYawResetAngle
fpVector3_t tiltErrVec;               // Vector of most recent attitude error correction from Vel,Pos fusion
float tiltErrFilt;                    // Filtered tilt error metric
bool tiltAlignComplete;               // true when tilt alignment is complete
bool yawAlignComplete;                // true when yaw alignment is complete
bool magStateInitComplete;            // true when the magnetic field sttes have been initialised
uint8_t stateIndexLim;                // Max state index used during matrix and array operations
imu_elements_t imuDataDelayed;        // IMU data at the fusion time horizon
imu_elements_t imuDataNew;            // IMU data at the current time horizon
imu_elements_t imuDataDownSampledNew; // IMU data at the current time horizon that has been downsampled to a 100Hz rate
fpQuaternion_t imuQuatDownSampleNew;  // Quaternion obtained by rotating through the IMU delta angles since the start of the current down sampled frame
baro_elements_t baroDataNew;          // Baro data at the current time horizon
baro_elements_t baroDataDelayed;      // Baro data at the fusion time horizon
range_elements_t rangeDataNew;        // Range finder data at the current time horizon
range_elements_t rangeDataDelayed;    // Range finder data at the fusion time horizon
tas_elements_t tasDataNew;            // TAS data at the current time horizon
tas_elements_t tasDataDelayed;        // TAS data at the fusion time horizon
mag_elements_t magDataDelayed;        // Magnetometer data at the fusion time horizon
gps_elements_t gpsDataNew;            // GPS data at the current time horizon
gps_elements_t gpsDataDelayed;        // GPS data at the fusion time horizon
output_elements_t outputDataNew;      // output state data at the current time step
output_elements_t outputDataDelayed;  // output state data at the current time step
fpVector3_t delAngCorrection;         // correction applied to delta angles used by output observer to track the EKF
fpVector3_t velErrintegral;           // integral of output predictor NED velocity tracking error (m)
fpVector3_t posErrintegral;           // integral of output predictor NED position tracking error (m.sec)
float innovYaw;                       // compass yaw angle innovation (rad)
uint32_t timeTasReceived_ms;          // time last TAS data was received (msec)
bool gpsGoodToAlign;                  // true when the GPS quality can be used to initialise the navigation system
uint32_t magYawResetTimer_ms;         // timer in msec used to track how long good magnetometer data is failing innovation consistency checks
bool consistentMagData;               // true when the magnetometers are passing consistency checks
bool motorsArmed;                     // true when the motors have been armed
bool prevMotorsArmed;                 // value of motorsArmed from previous frame
bool posVelFusionDelayed;             // true when the position and velocity fusion has been delayed
bool optFlowFusionDelayed;            // true when the optical flow fusion has been delayed
bool airSpdFusionDelayed;             // true when the air speed fusion has been delayed
bool sideSlipFusionDelayed;           // true when the sideslip fusion has been delayed
fpVector3_t lastMagOffsets;           // Last magnetometer offsets from COMPASS_ parameters. Used to detect parameter changes.
bool lastMagOffsetsValid;             // True when lastMagOffsets has been initialized
fpVector2_t posResetNE;               // Change in North/East position due to last in-flight reset in metres. Returned by getLastPosNorthEastReset
uint32_t lastPosReset_ms;             // System time at which the last position reset occurred. Returned by getLastPosNorthEastReset
fpVector2_t velResetNE;               // Change in North/East velocity due to last in-flight reset in metres/sec. Returned by getLastVelNorthEastReset
uint32_t lastVelReset_ms;             // System time at which the last velocity reset occurred. Returned by getLastVelNorthEastReset
float posResetD;                      // Change in Down position due to last in-flight reset in metres. Returned by getLastPosDowntReset
uint32_t lastPosResetD_ms;            // System time at which the last position reset occurred. Returned by getLastPosDownReset
float yawTestRatio;                   // square of magnetometer yaw angle innovation divided by fail threshold
fpQuaternion_t prevQuatMagReset;      // Quaternion from the last time the magnetic field state reset condition test was performed
float hgtInnovFiltState;              // state used for fitering of the height innovations used for pre-flight checks
bool runUpdates;                      // boolean true when the EKF updates can be run
uint32_t framesSincePredict;          // number of frames lapsed since EKF instance did a state prediction
bool startPredictEnabled;             // boolean true when the frontend has given permission to start a new state prediciton cycele
uint8_t localFilterTimeStep_ms;       // average number of msec between filter updates
float posDownObsNoise;                // observation noise variance on the vertical position used by the state and covariance update step (m^2)
fpVector3_t delAngCorrected;          // corrected IMU delta angle vector at the EKF time horizon (rad)
fpVector3_t delVelCorrected;          // corrected IMU delta velocity vector at the EKF time horizon (m/s)
bool magFieldLearned;                 // true when the magnetic field has been learned
uint32_t wasLearningCompass_ms;       // time when we were last waiting for compass learn to complete
fpVector3_t earthMagFieldVar;         // NED earth mag field variances for last learned field (mGauss^2)
fpVector3_t bodyMagFieldVar;          // XYZ body mag field variances for last learned field (mGauss^2)
bool delAngBiasLearned;               // true when the gyro bias has been learned
nav_filter_status_t filterStatus;     // contains the status of various filter outputs
float ekfOriginHgtVar;                // Variance of the EKF WGS-84 origin height estimate (m^2)
float ekfGpsRefHgt;                   // floating point representation of the WGS-84 reference height used to convert GPS height to local height (m)
uint32_t lastOriginHgtTime_ms;        // last time the ekf's WGS-84 origin height was corrected

// variables used by the pre-initialisation GPS checks
gpsLocation_t gpsloc_prev;            // LLH location of previous GPS measurement
uint32_t lastPreAlignGpsCheckTime_ms; // last time in msec the GPS quality was checked during pre alignment checks
float gpsDriftNE;                     // amount of drift detected in the GPS position during pre-flight GPs checks
float gpsVertVelFilt;                 // amount of filterred vertical GPS velocity detected durng pre-flight GPS checks
float gpsHorizVelFilt;                // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

// variable used by the in-flight GPS quality check
bool gpsSpdAccPass;            // true when reported GPS speed accuracy passes in-flight checks
bool ekfInnovationsPass;       // true when GPS innovations pass in-flight checks
float sAccFilterState1;        // state variable for LPF applid to reported GPS speed accuracy
float sAccFilterState2;        // state variable for peak hold filter applied to reported GPS speed
uint32_t lastGpsCheckTime_ms;  // last time in msec the GPS quality was checked
uint32_t lastInnovPassTime_ms; // last time in msec the GPS innovations passed
uint32_t lastInnovFailTime_ms; // last time in msec the GPS innovations failed
bool gpsAccuracyGood;          // true when the GPS accuracy is considered to be good enough for safe flight.

// variables added for optical flow fusion
ekf_ring_buffer storedOF;     // OF data buffer
of_elements_t ofDataNew;      // OF data at the current time horizon
of_elements_t ofDataDelayed;  // OF data at the fusion time horizon
bool flowDataToFuse;          // true when optical flow data is ready for fusion
bool flowDataValid;           // true while optical flow data is still fresh
fpVector2_t auxFlowObsInnov;  // optical flow rate innovation from 1-state terrain offset estimator
uint32_t flowValidMeaTime_ms; // time stamp from latest valid flow measurement (msec)
uint32_t rngValidMeaTime_ms;  // time stamp from latest valid range measurement (msec)
uint32_t flowMeaTime_ms;      // time stamp from latest flow measurement (msec)
uint32_t gndHgtValidTime_ms;  // time stamp from last terrain offset state update (msec)
fpMat3_t Tbn_flow;            // transformation matrix from body to nav axes at the middle of the optical flow sample period
Vector2 varInnovOptFlow;      // optical flow innovations variances (rad/sec)^2
Vector2 innovOptFlow;         // optical flow LOS innovations (rad/sec)
float Popt;                   // Optical flow terrain height state covariance (m^2)
float terrainState;           // terrain position state (m)
float prevPosN;               // north position at last measurement
float prevPosE;               // east position at last measurement
float varInnovRng;            // range finder observation innovation variance (m^2)
float innovRng;               // range finder observation innovation (m)
float hgtMea;                 // height measurement derived from either baro, gps or range finder data (m)
bool inhibitGndState;         // true when the terrain position state is to remain constant
uint32_t prevFlowFuseTime_ms; // time both flow measurement components passed their innovation consistency checks
Vector2 flowTestRatio;        // square of optical flow innovations divided by fail threshold used by main filter where >1.0 is a fail
fpVector2_t auxFlowTestRatio; // sum of squares of optical flow innovation divided by fail threshold used by 1-state terrain offset estimator
float R_LOS;                  // variance of optical flow rate measurements (rad/sec)^2
float auxRngTestRatio;        // square of range finder innovations divided by fail threshold used by main filter where >1.0 is a fail
fpVector2_t flowGyroBias;     // bias error of optical flow sensor gyro output
bool rangeDataToFuse;         // true when valid range finder height data has arrived at the fusion time horizon.
bool baroDataToFuse;          // true when valid baro height finder data has arrived at the fusion time horizon.
bool gpsDataToFuse;           // true when valid GPS data has arrived at the fusion time horizon.
bool magDataToFuse;           // true when valid magnetometer data has arrived at the fusion time horizon

AidingMode PV_AidingMode;     // Defines the preferred mode for aiding of velocity and position estimates from the INS
AidingMode PV_AidingModePrev; // Value of PV_AidingMode from the previous frame - used to detect transitions
bool gndOffsetValid;          // true when the ground offset state can still be considered valid
fpVector3_t delAngBodyOF;     // bias corrected delta angle of the vehicle IMU measured summed across the time since the last OF measurement
float delTimeOF;              // time that delAngBodyOF is summed across

// Range finder
float baroHgtOffset;              // offset applied when when switching to use of Baro height
float rngOnGnd;                   // Expected range finder reading in metres when vehicle is on ground
uint32_t lastRngMeasTime_ms;      // Timestamp of last range measurement
bool terrainHgtStable;            // true when the terrain height is stable enough to be used as a height reference
float storedRngMeas[3];           // Ringbuffer of stored range measurements for range sensor
uint32_t storedRngMeasTime_ms[3]; // Ringbuffers of stored range measurement times for range sensor
uint8_t rngMeasIndex;             // Current range measurement ringbuffer index for range sensor

// height source selection logic
uint8_t activeHgtSource; // integer defining active height source

// Movement detector
bool takeOffDetected;     // true when takeoff for optical flow navigation has been detected
float rngAtStartOfFlight; // range finder measurement at start of flight
uint32_t timeAtArming_ms; // time in msec that the vehicle armed

// baro ground effect
float meaHgtAtTakeOff; // height measured at commencement of takeoff

// control of post takeoff magnetic field and heading resets
bool finalInflightYawInit;         // true when the final post takeoff initialisation of yaw angle has been performed
bool finalInflightMagInit;         // true when the final post takeoff initialisation of magnetic field states been performed
bool magStateResetRequest;         // true if magnetic field states need to be reset using the magneteomter measurements
bool magYawResetRequest;           // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
bool gpsYawResetRequest;           // true if the vehicle yaw needs to be reset to the GPS course
float posDownAtLastMagReset;       // vertical position last time the mag states were reset (m)
float yawInnovAtLastMagReset;      // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
fpQuaternion_t quatAtLastMagReset; // quaternion states last time the mag states were reset
uint8_t magYawAnomallyCount;       // Number of times the yaw has been reset due to a magnetic anomaly during initial ascent

vertCompFiltState_t vertCompFiltState;
faultStatus_t faultStatus;
mag_state_t mag_state;

// earth field from WMM tables
bool have_table_earth_field;      // true when we have initialised table_earth_field_ga
fpVector3_t table_earth_field_ga; // earth field from WMM tables
float table_declination;          // declination in radians from the tables

// timing statistics
ekf_timing_t timing;

char last_ekf_msg[50];
bool ekf_msg_sended[2];
uint32_t last_filter_ok_ms; // when was attitude filter status last non-zero?

// The following declarations are used to control when the main navigation filter resets it's yaw to the estimate provided by the GSF
uint32_t EKFGSF_yaw_reset_ms;         // timestamp of last emergency yaw reset (uSec)
uint32_t EKFGSF_yaw_reset_request_ms; // timestamp of last emergency yaw reset request (uSec)
uint8_t EKFGSF_yaw_reset_count;       // number of emergency yaw resets performed
bool EKFGSF_run_filterbank;           // true when the filter bank is active

ekfStates_U ekfStates;

bool setupEKFRingBuffer(void)
{
    // if the last buffer is successfully allocated, avoid running this void more than once
    if (storedOutput.buffer.output_buffer != NULL)
    {
        return true;
    }

    /*
      The imu_buffer_length needs to cope with a 260ms delay at a maximum fusion rate of 100Hz.
      Non-imu data coming in at faster than 100Hz is downsampled.
      For 50Hz main loop rate we need a shorter buffer.
     */
    if (ekfParam._imuTimeHz < 100)
    {
        imu_buffer_length = 13;
    }
    else
    {
        // maximum 260 msec delay at 100 Hz fusion rate
        imu_buffer_length = 26;
    }

    if (!ekf_ring_buffer_init_size(&storedGPS, GPS_RING_BUFFER, OBS_BUFFER_LENGTH, sizeof(gps_elements_t)))
    {
        return false;
    }

    if (!ekf_ring_buffer_init_size(&storedMag, MAG_RING_BUFFER, OBS_BUFFER_LENGTH, sizeof(mag_elements_t)))
    {
        return false;
    }

    if (!ekf_ring_buffer_init_size(&storedBaro, BARO_RING_BUFFER, OBS_BUFFER_LENGTH, sizeof(baro_elements_t)))
    {
        return false;
    }

    if (!ekf_ring_buffer_init_size(&storedTAS, TAS_RING_BUFFER, OBS_BUFFER_LENGTH, sizeof(tas_elements_t)))
    {
        return false;
    }

    if (!ekf_ring_buffer_init_size(&storedRange, RANGE_RING_BUFFER, OBS_BUFFER_LENGTH, sizeof(range_elements_t)))
    {
        return false;
    }

    if (!ekf_ring_buffer_init_size(&storedOF, OPTFLOW_RING_BUFFER, FLOW_BUFFER_LENGTH, sizeof(of_elements_t)))
    {
        return false;
    }

    if (!ekf_imu_buffer_init_size(&storedIMU, IMU_RING_BUFFER, imu_buffer_length, sizeof(imu_elements_t)))
    {
        return false;
    }

    if (!ekf_imu_buffer_init_size(&storedOutput, OUTPUT_RING_BUFFER, imu_buffer_length, sizeof(output_elements_t)))
    {
        return false;
    }
    
    sendEKFLogMessage("");
    sendEKFLogMessage("EKF enabled");

    return true;
}

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void InitialiseVariables(void)
{
    // calculate the nominal filter update rate
    localFilterTimeStep_ms = (uint8_t)(1000.0f * (1.0f / (float)ekfParam._imuTimeHz));
    localFilterTimeStep_ms = MAX(localFilterTimeStep_ms, 10);

    // initialise time stamps
    imuSampleTime_ms = millis();
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaStep_ms = imuSampleTime_ms;
    lastBaroReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = 0;
    lastPosPassTime_ms = 0;
    lastHgtPassTime_ms = 0;
    lastTasPassTime_ms = 0;
    lastYawTime_ms = imuSampleTime_ms;
    lastTimeGpsReceived_ms = 0;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = 0;
    gndHgtValidTime_ms = 0;
    ekfStartTime_us = MS2US(imuSampleTime_ms);
    lastGpsVelFail_ms = 0;
    lastGpsVelPass_ms = 0;
    lastGpsAidBadTime_ms = 0;
    timeTasReceived_ms = 0;
    lastPreAlignGpsCheckTime_ms = imuSampleTime_ms;
    lastPosReset_ms = 0;
    lastVelReset_ms = 0;
    lastPosResetD_ms = 0;
    lastRngMeasTime_ms = 0;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    tasTimeout = true;
    badIMUdata = false;
    dtIMUavg = 0.0025f;
    dtEkfAvg = EKF_TARGET_DT;
    dt = 0.0f;
    vectorZero(&velDotNEDfilt);
    lastKnownPositionNE.x = 0.0f;
    lastKnownPositionNE.y = 0.0f;
    zeroMatrix(&prevTnb);
    memset(&P[0][0], 0, sizeof(P));
    memset(&KH[0][0], 0, sizeof(KH));
    memset(&KHP[0][0], 0, sizeof(KHP));
    memset(&nextP[0][0], 0, sizeof(nextP));
    flowDataValid = false;
    rangeDataToFuse = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = ekfStates.stateStruct.position.x;
    prevPosE = ekfStates.stateStruct.position.y;
    inhibitGndState = false;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    PV_AidingMode = AID_NONE;
    PV_AidingModePrev = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1.0f;
    identityMatrix(&mag_state.DCM);
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    inhibitWindStates = true;
    gndOffsetValid = false;
    validOrigin = false;
    gpsSpdAccuracy = 0.0f;
    gpsPosAccuracy = 0.0f;
    gpsHgtAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    rngOnGnd = 0.05f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    tiltErrFilt = 1.0f;
    tiltAlignComplete = false;
    stateIndexLim = 23;
    vectorZero(&delAngCorrection);
    vectorZero(&velErrintegral);
    vectorZero(&posErrintegral);
    gpsGoodToAlign = false;
    gpsNotAvailable = true;
    motorsArmed = false;
    prevMotorsArmed = false;
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastInnovPassTime_ms = 0;
    lastInnovFailTime_ms = 0;
    gpsAccuracyGood = false;
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    memset(&ekfStates.statesArray, 0, sizeof(ekfStates.statesArray));
    memset(&vertCompFiltState, 0, sizeof(vertCompFiltState));
    posVelFusionDelayed = false;
    optFlowFusionDelayed = false;
    airSpdFusionDelayed = false;
    sideSlipFusionDelayed = false;
    posResetNE.x = 0.0f;
    posResetNE.y = 0.0f;
    velResetNE.x = 0.0f;
    velResetNE.y = 0.0f;
    posResetD = 0.0f;
    hgtInnovFiltState = 0.0f;

    vectorZero(&imuDataDownSampledNew.delAng);
    vectorZero(&imuDataDownSampledNew.delVel);
    imuDataDownSampledNew.delAngDT = 0.0f;
    imuDataDownSampledNew.delVelDT = 0.0f;
    runUpdates = false;
    framesSincePredict = 0;
    gpsYawResetRequest = false;
    quaternionInitialise(&ekfStates.stateStruct.quat);
    quatAtLastMagReset = ekfStates.stateStruct.quat;
    delAngBiasLearned = false;
    activeHgtSource = 0;
    memset(&filterStatus, 0, sizeof(filterStatus));
    memset(&rngMeasIndex, 0, sizeof(rngMeasIndex));
    memset(&storedRngMeasTime_ms, 0, sizeof(storedRngMeasTime_ms));
    memset(&storedRngMeas, 0, sizeof(storedRngMeas));
    memset(&velPosObs, 0, sizeof(velPosObs));
    terrainHgtStable = true;
    ekfOriginHgtVar = 0.0f;
    ekfGpsRefHgt = 0.0;

    // zero data buffers
    ekf_imu_buffer_reset(&storedIMU, IMU_RING_BUFFER);
    ekf_ring_buffer_reset(&storedGPS);
    ekf_ring_buffer_reset(&storedBaro);
    ekf_ring_buffer_reset(&storedTAS);
    ekf_ring_buffer_reset(&storedRange);
    ekf_imu_buffer_reset(&storedOutput, OUTPUT_RING_BUFFER);

    // now init mag variables
    yawAlignComplete = false;
    have_table_earth_field = false;

    // initialise pre-arm message
    sendEKFLogMessage("EKF still initialising");

    InitialiseVariablesMag();

    // emergency reset of yaw to EKFGSF estimate
    EKFGSF_yaw_reset_ms = 0;
    EKFGSF_yaw_reset_request_ms = 0;
    EKFGSF_yaw_reset_count = 0;
    EKFGSF_run_filterbank = false;
}

// separate out the mag reset so it can be used when compass learning completes
void InitialiseVariablesMag(void)
{
    lastHealthyMagTime_ms = imuSampleTime_ms;
    lastMagUpdate_us = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    magTimeout = false;
    magSensorFailed = false;
    finalInflightYawInit = false;
    finalInflightMagInit = false;

    inhibitMagStates = true;

    lastMagOffsetsValid = false;
    magStateResetRequest = false;
    magStateInitComplete = false;
    magYawResetRequest = false;

    posDownAtLastMagReset = ekfStates.stateStruct.position.z;
    yawInnovAtLastMagReset = 0.0f;
    magFieldLearned = false;

    ekf_ring_buffer_reset(&storedMag);
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool coreInitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && gpsSol.fixType < GPS_FIX_3D)
    {
        sendEKFLogMessage("EKF init failure: No GPS lock");
        statesInitialised = false;
        return false;
    }

    if (statesInitialised)
    {
        // we are initialised, but we don't return true until the IMU
        // buffer has been filled. This prevents a timing
        // vulnerability with a pause in IMU data during filter startup
        readIMUData();
        readMagData();
        readGpsData();
        readBaroData();

        bool imu_buffer_is_filled = ekf_imu_buffer_is_filled(&storedIMU);

        if (imu_buffer_is_filled) 
        {
            sendEKFLogMessage("EKF IMU Buffer has been filled");
        } 
        else 
        {
            sendEKFLogMessage("EKF IMU Buffer not filled");
        }

        return imu_buffer_is_filled;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // Initialise IMU data
    dtIMUavg = 1.0f / (float)ekfParam._imuTimeHz;
    readIMUData();
    ekf_imu_buffer_reset_history(&storedIMU, &imuDataNew);
    imuDataDelayed = imuDataNew;

    // read the magnetometer data
    readMagData();

    // acceleration vector in XYZ body axes measured by the IMU
    fpVector3_t initAccVec;
    accGetMeasuredAcceleration(&initAccVec);

    // convert the accel in body frame in cm/s to m/s
    initAccVec.x *= 0.01f;
    initAccVec.y *= 0.01f;
    initAccVec.z *= 0.01f;

    // normalise the acceleration vector
    float pitch = 0.0f;
    float roll = 0.0f;
    if (calc_length_pythagorean_3D(initAccVec.x, initAccVec.y, initAccVec.z) > 0.001f)
    {
        vectorNormalize(&initAccVec, &initAccVec);

        // calculate initial roll angle
        roll = atan2f(initAccVec.x, initAccVec.z);

        // calculate initial pitch angle
        pitch = asinf(initAccVec.y);
    }

    // calculate initial roll and pitch orientation
    quaternionFromEuler(&ekfStates.stateStruct.quat, roll, pitch, 0.0f);

    // initialise dynamic states
    vectorZero(&ekfStates.stateStruct.velocity);
    vectorZero(&ekfStates.stateStruct.position);
    vectorZero(&ekfStates.stateStruct.angErr);

    // initialise static process model states
    vectorZero(&ekfStates.stateStruct.gyro_bias);
    ekfStates.stateStruct.gyro_scale.x = 1.0f;
    ekfStates.stateStruct.gyro_scale.y = 1.0f;
    ekfStates.stateStruct.gyro_scale.z = 1.0f;
    ekfStates.stateStruct.accel_zbias = 0.0f;
    ekfStates.stateStruct.wind_vel.x = 0.0f;
    ekfStates.stateStruct.wind_vel.y = 0.0f;
    vectorZero(&ekfStates.stateStruct.earth_magfield);
    vectorZero(&ekfStates.stateStruct.body_magfield);

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readBaroData();
    ResetHeight();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(&earthRateNED, posControl.gpsOrigin.lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset output states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // we initially return false to wait for the IMU buffer to fill
    return false;
}

// initialise the covariance matrix
void CovarianceInit(void)
{
    // zero the matrix
    memset(&P[0][0], 0, sizeof(P));

    // attitude error
    P[0][0] = 0.1f;
    P[1][1] = 0.1f;
    P[2][2] = 0.1f;

    // velocities
    P[3][3] = sq(ekfParam._gpsHorizVelNoise);
    P[4][4] = P[3][3];
    P[5][5] = sq(ekfParam._gpsVertVelNoise);

    // positions
    P[6][6] = sq(ekfParam._gpsHorizPosNoise);
    P[7][7] = P[6][6];
    P[8][8] = sq(ekfParam._baroAltNoise);

    // gyro delta angle biases
    P[9][9] = sq(DEGREES_TO_RADIANS(InitialGyroBiasUncertainty() * dtEkfAvg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];

    // gyro scale factor biases
    P[12][12] = sq(1e-3);
    P[13][13] = P[12][12];
    P[14][14] = P[12][12];

    // Z delta velocity bias
    P[15][15] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtEkfAvg);

    // earth magnetic field
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];

    // body magnetic field
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];

    // wind velocities
    P[22][22] = 0.0f;
    P[23][23] = P[22][22];

    // optical flow ground height covariance
    Popt = 0.25f;
}

// Update Filter States - this should be called whenever new IMU data is available
void FAST_CODE coreUpdateFilter(bool predict, timeUs_t time_us)
{
    // Set the flag to indicate to the filter that the front-end has given permission for a new state prediction cycle to be started
    startPredictEnabled = predict;

    // don't run filter updates if states have not been initialised
    if (!statesInitialised)
    {
        return;
    }

    // get starting time for update step
    imuSampleTime_ms = US2MS(time_us);

    // Check arm status and perform required checks and mode changes
    controlFilterModes();

    // read IMU data as delta angles and velocities
    readIMUData();

    // Run the EKF equations to estimate at the fusion time horizon if new IMU data is available in the buffer
    if (runUpdates)
    {
        // Predict states using IMU data from the delayed time horizon
        UpdateStrapdownEquationsNED();

        // Predict the covariance growth
        CovariancePrediction();

        // Run the IMU prediction step for the GSF yaw estimator algorithm
        // using IMU and optionally true airspeed data.
        // Must be run before SelectMagFusion() to provide an up to date yaw estimate
        runYawEstimatorPrediction();

        // Update states using  magnetometer data
        SelectMagFusion();

        // Update states using GPS and altimeter data
        SelectVelPosFusion();

        // Run the GPS velocity correction step for the GSF yaw estimator algorithm
        // and use the yaw estimate to reset the main EKF yaw if requested
        // Muat be run after SelectVelPosFusion() so that fresh GPS data is available
        runYawEstimatorCorrection();

        // Update states using optical flow data
        SelectFlowFusion();

        // Update states using airspeed data
        SelectTasFusion();

        // Update states using sideslip constraint assumption for fly-forward vehicles
        SelectBetaFusion();

        // Update the filter status
        updateFilterStatus();
    }

    // Wind output forward from the fusion to output time horizon
    calcOutputStates();

    /*
      this is a check to cope with a vehicle sitting idle on the
      ground and getting over-confident of the state. The symptoms
      would be "gyros still settling" when the user tries to arm. In
      that state the EKF can't recover, so we do a hard reset and let
      it try again.
     */
    if (filterStatus.value != 0)
    {
        last_filter_ok_ms = US2MS(time_us);
    }

    if (filterStatus.value == 0 && last_filter_ok_ms != 0 && US2MS(time_us) - last_filter_ok_ms > 5000 && !get_armed())
    {
        // we've been unhealthy for 5 seconds after being healthy, reset the filter
        sendEKFLogMessage("EKF IMU forced reset");
        last_filter_ok_ms = 0;
        statesInitialised = false;
        coreInitialiseFilterBootstrap();
    }
}

void correctDeltaAngle(fpVector3_t *delAng, float delAngDT)
{
    delAng->x *= ekfStates.stateStruct.gyro_scale.x;
    delAng->y *= ekfStates.stateStruct.gyro_scale.y;
    delAng->z *= ekfStates.stateStruct.gyro_scale.z;
    delAng->x -= ekfStates.stateStruct.gyro_bias.x * (delAngDT / dtEkfAvg);
    delAng->y -= ekfStates.stateStruct.gyro_bias.y * (delAngDT / dtEkfAvg);
    delAng->z -= ekfStates.stateStruct.gyro_bias.z * (delAngDT / dtEkfAvg);
}

void correctDeltaVelocity(fpVector3_t *delVel, float delVelDT)
{
    delVel->z -= ekfStates.stateStruct.accel_zbias * (delVelDT / dtEkfAvg);
}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observation is fused. This attitude error is then used to correct
 * the quaternion.
 */
void UpdateStrapdownEquationsNED(void)
{
    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    // apply correction for earth's rotation rate
    // % * - and + operators have been overloaded
    fpVector3_t mulResult = multiplyMatrixByVector(prevTnb, earthRateNED);
    mulResult.x *= imuDataDelayed.delAngDT;
    mulResult.y *= imuDataDelayed.delAngDT;
    mulResult.z *= imuDataDelayed.delAngDT;
    fpVector3_t vectorDiff = {.v = {delAngCorrected.x - mulResult.x, delAngCorrected.y - mulResult.y, delAngCorrected.z - mulResult.z}};
    quaternion_rotate(&ekfStates.stateStruct.quat, vectorDiff);
    quaternion_normalize(&ekfStates.stateStruct.quat);

    // transform body delta velocities to delta velocities in the nav frame
    // use the nav frame from previous time step as the delta velocities
    // have been rotated into that frame
    // * and + operators have been overloaded
    fpVector3_t delVelNav; // delta velocity vector in earth axes
    delVelNav = multiplyMatrixTransposeByVector(prevTnb, delVelCorrected);
    delVelNav.z += GRAVITY_MSS * imuDataDelayed.delVelDT;

    // calculate the nav to body cosine matrix
    quaternionToRotationMatrix(quaternion_inverse(ekfStates.stateStruct.quat), &prevTnb);

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED.x = delVelNav.x / imuDataDelayed.delVelDT;
    velDotNED.y = delVelNav.y / imuDataDelayed.delVelDT;
    velDotNED.z = delVelNav.z / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter
    velDotNEDfilt.x = velDotNED.x * 0.05f + velDotNEDfilt.x * 0.95f;
    velDotNEDfilt.y = velDotNED.y * 0.05f + velDotNEDfilt.y * 0.95f;
    velDotNEDfilt.z = velDotNED.z * 0.05f + velDotNEDfilt.z * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS variance estimation)
    accNavMag = calc_length_pythagorean_3D(velDotNEDfilt.x, velDotNEDfilt.y, velDotNEDfilt.z);
    accNavMagHoriz = calc_length_pythagorean_2D(velDotNEDfilt.x, velDotNEDfilt.y);

    // if we are not aiding, then limit the horizontal magnitude of acceleration
    // to prevent large manoeuvre transients disturbing the attitude
    if ((PV_AidingMode == AID_NONE) && (accNavMagHoriz > 5.0f))
    {
        float gain = 5.0f / accNavMagHoriz;
        delVelNav.x *= gain;
        delVelNav.y *= gain;
    }

    // save velocity for use in trapezoidal integration for position calcuation
    fpVector3_t lastVelocity = ekfStates.stateStruct.velocity;

    // sum delta velocities to get velocity
    ekfStates.stateStruct.velocity.x += delVelNav.x;
    ekfStates.stateStruct.velocity.y += delVelNav.y;
    ekfStates.stateStruct.velocity.z += delVelNav.z;

    // apply a trapezoidal integration to velocities to calculate position
    ekfStates.stateStruct.position.x += (ekfStates.stateStruct.velocity.x + lastVelocity.x) * (imuDataDelayed.delVelDT * 0.5f);
    ekfStates.stateStruct.position.y += (ekfStates.stateStruct.velocity.y + lastVelocity.y) * (imuDataDelayed.delVelDT * 0.5f);
    ekfStates.stateStruct.position.z += (ekfStates.stateStruct.velocity.z + lastVelocity.z) * (imuDataDelayed.delVelDT * 0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    delAngBodyOF.x += delAngCorrected.x;
    delAngBodyOF.y += delAngCorrected.y;
    delAngBodyOF.z += delAngCorrected.z;
    delTimeOF += imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();
}

/*
 * Propagate PVA solution forward from the fusion time horizon to the current time horizon
 * using simple observer which performs two functions:
 * 1) Corrects for the delayed time horizon used by the EKF.
 * 2) Applies a LPF to state corrections to prevent 'stepping' in states due to measurement
 * fusion introducing unwanted noise into the control loops.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian.
 *
 * "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements"
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
 */
void calcOutputStates(void)
{
    // apply corrections to the IMU data
    fpVector3_t delAngNewCorrected = imuDataNew.delAng;
    fpVector3_t delVelNewCorrected = imuDataNew.delVel;
    correctDeltaAngle(&delAngNewCorrected, imuDataNew.delAngDT);
    correctDeltaVelocity(&delVelNewCorrected, imuDataNew.delVelDT);

    // apply corections to track EKF solution
    fpVector3_t delAng = {.v = {delAngNewCorrected.x + delAngCorrection.x,
                                delAngNewCorrected.y + delAngCorrection.y,
                                delAngNewCorrected.z + delAngCorrection.z}};

    // convert the rotation vector to its equivalent quaternion
    fpQuaternion_t deltaQuat;
    quaternion_from_axis_angle(&deltaQuat, delAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    quaternion_multiply_assign(&outputDataNew.quat, deltaQuat);
    quaternion_normalize(&outputDataNew.quat);

    // calculate the body to nav cosine matrix
    fpMat3_t Tbn_temp;
    quaternionToRotationMatrix(outputDataNew.quat, &Tbn_temp);

    // transform body delta velocities to delta velocities in the nav frame
    fpVector3_t delVelNav = multiplyMatrixByVector(Tbn_temp, delVelNewCorrected);
    delVelNav.z += GRAVITY_MSS * imuDataNew.delVelDT;

    // save velocity for use in trapezoidal integration for position calcuation
    fpVector3_t lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity.x += delVelNav.x;
    outputDataNew.velocity.y += delVelNav.y;
    outputDataNew.velocity.z += delVelNav.z;

    // Implement third order complementary filter for height and height rate
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    // Perform filter calculation using backwards Euler integration
    // Coefficients selected to place all three filter poles at omega
    const float CompFiltOmega = (M_PIf * 2.0f) * constrainf(ekfParam._hrt_filt_freq, 0.1f, 30.0f);
    float omega2 = CompFiltOmega * CompFiltOmega;
    float pos_err = constrainf(outputDataNew.position.z - vertCompFiltState.pos, -1e5f, 1e5f);
    float integ1_input = pos_err * omega2 * CompFiltOmega * imuDataNew.delVelDT;
    vertCompFiltState.acc += integ1_input;
    float integ2_input = delVelNav.z + (vertCompFiltState.acc + pos_err * omega2 * 3.0f) * imuDataNew.delVelDT;
    vertCompFiltState.vel += integ2_input;
    float integ3_input = (vertCompFiltState.vel + pos_err * CompFiltOmega * 3.0f) * imuDataNew.delVelDT;
    vertCompFiltState.pos += integ3_input;

    // apply a trapezoidal integration to velocities to calculate position
    outputDataNew.position.x += (outputDataNew.velocity.x + lastVelocity.x) * (imuDataNew.delVelDT * 0.5f);
    outputDataNew.position.y += (outputDataNew.velocity.y + lastVelocity.y) * (imuDataNew.delVelDT * 0.5f);
    outputDataNew.position.z += (outputDataNew.velocity.z + lastVelocity.z) * (imuDataNew.delVelDT * 0.5f);
    
    // store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
    if (runUpdates)
    {
        // store the states at the output time horizon
        memcpy(ekf_output_buffer_get(&storedOutput, storedIMU.youngest), &outputDataNew, sizeof(output_elements_t));

        // recall the states from the fusion time horizon
        memcpy(&outputDataDelayed, ekf_output_buffer_get(&storedOutput, storedIMU.oldest), sizeof(output_elements_t));

        // divide the demanded quaternion by the estimated to get the error
        fpQuaternion_t quatErr = quaternionDivision(ekfStates.stateStruct.quat, outputDataDelayed.quat);

        // Convert to a delta rotation using a small angle approximation
        quaternion_normalize(&quatErr);

        float scaler;

        if (quatErr.q0 >= 0.0f)
        {
            scaler = 2.0f;
        }
        else
        {
            scaler = -2.0f;
        }

        fpVector3_t deltaAngErr = {.v = {scaler * quatErr.q1, scaler * quatErr.q2, scaler * quatErr.q3}};

        // calculate a gain that provides tight tracking of the estimator states and
        // adjust for changes in time delay to maintain consistent damping ratio of ~0.7
        float timeDelay = 1e-3f * (float)(imuDataNew.time_ms - imuDataDelayed.time_ms);
        timeDelay = fmaxf(timeDelay, dtIMUavg);
        float errorGain = 0.5f / timeDelay;

        // calculate a correction to the delta angle
        // that will cause the INS to track the EKF quaternions
        delAngCorrection.x = deltaAngErr.x * errorGain * dtIMUavg;
        delAngCorrection.y = deltaAngErr.y * errorGain * dtIMUavg;
        delAngCorrection.z = deltaAngErr.z * errorGain * dtIMUavg;

        // calculate velocity and position tracking errors
        fpVector3_t velErr = {.v = {(ekfStates.stateStruct.velocity.x - outputDataDelayed.velocity.x),
                                    (ekfStates.stateStruct.velocity.y - outputDataDelayed.velocity.y),
                                    (ekfStates.stateStruct.velocity.z - outputDataDelayed.velocity.z)}};

        fpVector3_t posErr = {.v = {(ekfStates.stateStruct.position.x - outputDataDelayed.position.x),
                                    (ekfStates.stateStruct.position.y - outputDataDelayed.position.y),
                                    (ekfStates.stateStruct.position.z - outputDataDelayed.position.z)}};

        // convert user specified time constant from centi-seconds to seconds
        float tauPosVel = constrainf(0.01f * (float)ekfParam._tauVelPosOutput, 0.1f, 0.5f);

        // calculate a gain to track the EKF position states with the specified time constant
        float velPosGain = dtEkfAvg / constrainf(tauPosVel, dtEkfAvg, 10.0f);

        // use a PI feedback to calculate a correction that will be applied to the output state history
        posErrintegral.x += posErr.x;
        posErrintegral.y += posErr.y;
        posErrintegral.z += posErr.z;

        velErrintegral.x += velErr.x;
        velErrintegral.y += velErr.y;
        velErrintegral.z += velErr.z;

        fpVector3_t velCorrection = {.v = {velErr.x * velPosGain + velErrintegral.x * sq(velPosGain) * 0.1f,
                                           velErr.y * velPosGain + velErrintegral.y * sq(velPosGain) * 0.1f,
                                           velErr.z * velPosGain + velErrintegral.z * sq(velPosGain) * 0.1f}};

        fpVector3_t posCorrection = {.v = {posErr.x * velPosGain + posErrintegral.x * sq(velPosGain) * 0.1f,
                                           posErr.y * velPosGain + posErrintegral.y * sq(velPosGain) * 0.1f,
                                           posErr.z * velPosGain + posErrintegral.z * sq(velPosGain) * 0.1f}};

        // loop through the output filter state history and apply the corrections to the velocity and position states
        // this method is too expensive to use for the attitude states due to the quaternion operations required
        // but does not introduce a time delay in the 'correction loop' and allows smaller tracking time constants
        // to be used
        output_elements_t outputStates;
        for (uint8_t index = 0; index < imu_buffer_length; index++)
        {
            memcpy(&outputStates, ekf_output_buffer_get(&storedOutput, index), sizeof(output_elements_t));

            // a constant  velocity correction is applied
            outputStates.velocity.x += velCorrection.x;
            outputStates.velocity.y += velCorrection.y;
            outputStates.velocity.z += velCorrection.z;

            // a constant position correction is applied
            outputStates.position.x += posCorrection.x;
            outputStates.position.y += posCorrection.y;
            outputStates.position.z += posCorrection.z;

            // push the updated data to the buffer
            memcpy(ekf_output_buffer_get(&storedOutput, index), &outputStates, sizeof(output_elements_t));
        }

        // update output state to corrected values
        memcpy(&outputDataNew, ekf_output_buffer_get(&storedOutput, storedIMU.youngest), sizeof(output_elements_t));
    }
}

/*
 * Calculate the predicted state covariance matrix using algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 */
void CovariancePrediction(void)
{
    float windVelSigma;   // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;  // delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;  // delta velocity bias 1-sigma process noise - m/s
    float dAngScaleSigma; // delta angle scale factor 1-Sigma process noise
    float magEarthSigma;  // earth magnetic field 1-sigma process noise
    float magBodySigma;   // body magnetic field 1-sigma process noise
    float daxNoise;       // X axis delta angle noise variance rad^2
    float dayNoise;       // Y axis delta angle noise variance rad^2
    float dazNoise;       // Z axis delta angle noise variance rad^2
    float dvxNoise;       // X axis delta velocity variance noise (m/s)^2
    float dvyNoise;       // Y axis delta velocity variance noise (m/s)^2
    float dvzNoise;       // Z axis delta velocity variance noise (m/s)^2
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
    float dax_s;          // X axis delta angle measurement scale factor
    float day_s;          // Y axis delta angle measurement scale factor
    float daz_s;          // Z axis delta angle measurement scale factor
    float dvz_b;          // Z axis delta velocity measurement bias (rad)
    Vector25 SF;
    Vector5 SG;
    Vector8 SQ;
    Vector24 processNoise;

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter
    dt = imuDataDelayed.delAngDT;
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - ekfStates.stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma = dt * constrainf(ekfParam._windVelProcessNoise, 0.0f, 1.0f) * (1.0f + constrainf(ekfParam._wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = sq(dt) * constrainf(ekfParam._gyroBiasProcessNoise, 0.0f, 1.0f);
    dVelBiasSigma = sq(dt) * constrainf(ekfParam._accelBiasProcessNoise, 0.0f, 1.0f);
    dAngScaleSigma = dt * constrainf(ekfParam._gyroScaleProcessNoise, 0.0f, 1.0f);
    magEarthSigma = dt * constrainf(ekfParam._magEarthProcessNoise, 0.0f, 1.0f);
    magBodySigma = dt * constrainf(ekfParam._magBodyProcessNoise, 0.0f, 1.0f);

    for (uint8_t i = 0; i <= 8; i++)
        processNoise[i] = 0.0f;

    for (uint8_t i = 9; i <= 11; i++)
        processNoise[i] = dAngBiasSigma;

    for (uint8_t i = 12; i <= 14; i++)
        processNoise[i] = dAngScaleSigma;

    if (get_takeoff_expected())
    {
        processNoise[15] = 0.0f;
    }
    else
    {
        processNoise[15] = dVelBiasSigma;
    }

    for (uint8_t i = 16; i <= 18; i++)
        processNoise[i] = magEarthSigma;

    for (uint8_t i = 19; i <= 21; i++)
        processNoise[i] = magBodySigma;

    for (uint8_t i = 22; i <= 23; i++)
        processNoise[i] = windVelSigma;

    for (uint8_t i = 0; i <= stateIndexLim; i++)
        processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = imuDataDelayed.delVel.x;
    dvy = imuDataDelayed.delVel.y;
    dvz = imuDataDelayed.delVel.z;
    dax = imuDataDelayed.delAng.x;
    day = imuDataDelayed.delAng.y;
    daz = imuDataDelayed.delAng.z;
    q0 = ekfStates.stateStruct.quat.q0;
    q1 = ekfStates.stateStruct.quat.q1;
    q2 = ekfStates.stateStruct.quat.q2;
    q3 = ekfStates.stateStruct.quat.q3;
    dax_b = ekfStates.stateStruct.gyro_bias.x;
    day_b = ekfStates.stateStruct.gyro_bias.y;
    daz_b = ekfStates.stateStruct.gyro_bias.z;
    dax_s = ekfStates.stateStruct.gyro_scale.x;
    day_s = ekfStates.stateStruct.gyro_scale.y;
    daz_s = ekfStates.stateStruct.gyro_scale.z;
    dvz_b = ekfStates.stateStruct.accel_zbias;
    float _gyrNoise = constrainf(ekfParam._gyrNoise, 0.0f, 1.0f);
    daxNoise = dayNoise = dazNoise = sq(dt * _gyrNoise);
    float _accNoise = constrainf(ekfParam._accNoise, 0.0f, 10.0f);
    dvxNoise = dvyNoise = dvzNoise = sq(dt * _accNoise);

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the upper diagonal and copy to take advantage of symmetry
    SF[0] = daz_b / 2 - (daz * daz_s) / 2;
    SF[1] = day_b / 2 - (day * day_s) / 2;
    SF[2] = dax_b / 2 - (dax * dax_s) / 2;
    SF[3] = q3 / 2 - (q0 * SF[0]) / 2 + (q1 * SF[1]) / 2 - (q2 * SF[2]) / 2;
    SF[4] = q0 / 2 - (q1 * SF[2]) / 2 - (q2 * SF[1]) / 2 + (q3 * SF[0]) / 2;
    SF[5] = q1 / 2 + (q0 * SF[2]) / 2 - (q2 * SF[0]) / 2 - (q3 * SF[1]) / 2;
    SF[6] = q3 / 2 + (q0 * SF[0]) / 2 - (q1 * SF[1]) / 2 - (q2 * SF[2]) / 2;
    SF[7] = q0 / 2 - (q1 * SF[2]) / 2 + (q2 * SF[1]) / 2 - (q3 * SF[0]) / 2;
    SF[8] = q0 / 2 + (q1 * SF[2]) / 2 - (q2 * SF[1]) / 2 - (q3 * SF[0]) / 2;
    SF[9] = q2 / 2 + (q0 * SF[1]) / 2 + (q1 * SF[0]) / 2 + (q3 * SF[2]) / 2;
    SF[10] = q2 / 2 - (q0 * SF[1]) / 2 - (q1 * SF[0]) / 2 + (q3 * SF[2]) / 2;
    SF[11] = q2 / 2 + (q0 * SF[1]) / 2 - (q1 * SF[0]) / 2 - (q3 * SF[2]) / 2;
    SF[12] = q1 / 2 + (q0 * SF[2]) / 2 + (q2 * SF[0]) / 2 + (q3 * SF[1]) / 2;
    SF[13] = q1 / 2 - (q0 * SF[2]) / 2 + (q2 * SF[0]) / 2 - (q3 * SF[1]) / 2;
    SF[14] = q3 / 2 + (q0 * SF[0]) / 2 + (q1 * SF[1]) / 2 + (q2 * SF[2]) / 2;
    SF[15] = -sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SF[16] = dvz_b - dvz;
    SF[17] = dvx;
    SF[18] = dvy;
    SF[19] = sq(q2);
    SF[20] = SF[19] - sq(q0) + sq(q1) - sq(q3);
    SF[21] = SF[19] + sq(q0) - sq(q1) - sq(q3);
    SF[22] = 2 * q0 * q1 - 2 * q2 * q3;
    SF[23] = SF[19] - sq(q0) - sq(q1) + sq(q3);
    SF[24] = 2 * q1 * q2;

    SG[0] = -sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);

    SQ[0] = -dvyNoise * (2 * q0 * q1 + 2 * q2 * q3) * (SG[1] - SG[2] + SG[3] - SG[4]) - dvzNoise * (2 * q0 * q1 - 2 * q2 * q3) * (SG[1] - SG[2] - SG[3] + SG[4]) - dvxNoise * (2 * q0 * q2 - 2 * q1 * q3) * (2 * q0 * q3 + 2 * q1 * q2);
    SQ[1] = dvxNoise * (2 * q0 * q2 - 2 * q1 * q3) * (SG[1] + SG[2] - SG[3] - SG[4]) + dvzNoise * (2 * q0 * q2 + 2 * q1 * q3) * (SG[1] - SG[2] - SG[3] + SG[4]) - dvyNoise * (2 * q0 * q1 + 2 * q2 * q3) * (2 * q0 * q3 - 2 * q1 * q2);
    SQ[2] = dvyNoise * (2 * q0 * q3 - 2 * q1 * q2) * (SG[1] - SG[2] + SG[3] - SG[4]) - dvxNoise * (2 * q0 * q3 + 2 * q1 * q2) * (SG[1] + SG[2] - SG[3] - SG[4]) - dvzNoise * (2 * q0 * q1 - 2 * q2 * q3) * (2 * q0 * q2 + 2 * q1 * q3);
    SQ[3] = sq(SG[0]);
    SQ[4] = 2 * q2 * q3;
    SQ[5] = 2 * q1 * q3;
    SQ[6] = 2 * q1 * q2;
    SQ[7] = SG[4];

    Vector23 SPP;
    SPP[0] = SF[17] * (2 * q0 * q1 + 2 * q2 * q3) + SF[18] * (2 * q0 * q2 - 2 * q1 * q3);
    SPP[1] = SF[18] * (2 * q0 * q2 + 2 * q1 * q3) + SF[16] * (SF[24] - 2 * q0 * q3);
    SPP[2] = 2 * q3 * SF[8] + 2 * q1 * SF[11] - 2 * q0 * SF[14] - 2 * q2 * SF[13];
    SPP[3] = 2 * q1 * SF[7] + 2 * q2 * SF[6] - 2 * q0 * SF[12] - 2 * q3 * SF[10];
    SPP[4] = 2 * q0 * SF[6] - 2 * q3 * SF[7] - 2 * q1 * SF[10] + 2 * q2 * SF[12];
    SPP[5] = 2 * q0 * SF[8] + 2 * q2 * SF[11] + 2 * q1 * SF[13] + 2 * q3 * SF[14];
    SPP[6] = 2 * q0 * SF[7] + 2 * q3 * SF[6] + 2 * q2 * SF[10] + 2 * q1 * SF[12];
    SPP[7] = SF[18] * SF[20] - SF[16] * (2 * q0 * q1 + 2 * q2 * q3);
    SPP[8] = 2 * q1 * SF[3] - 2 * q2 * SF[4] - 2 * q3 * SF[5] + 2 * q0 * SF[9];
    SPP[9] = 2 * q0 * SF[5] - 2 * q1 * SF[4] - 2 * q2 * SF[3] + 2 * q3 * SF[9];
    SPP[10] = SF[17] * SF[20] + SF[16] * (2 * q0 * q2 - 2 * q1 * q3);
    SPP[11] = SF[17] * SF[21] - SF[18] * (SF[24] + 2 * q0 * q3);
    SPP[12] = SF[17] * SF[22] - SF[16] * (SF[24] + 2 * q0 * q3);
    SPP[13] = 2 * q0 * SF[4] + 2 * q1 * SF[5] + 2 * q3 * SF[3] + 2 * q2 * SF[9];
    SPP[14] = 2 * q2 * SF[8] - 2 * q0 * SF[11] - 2 * q1 * SF[14] + 2 * q3 * SF[13];
    SPP[15] = SF[18] * SF[23] + SF[17] * (SF[24] - 2 * q0 * q3);
    SPP[16] = daz * SF[19] + daz * sq(q0) + daz * sq(q1) + daz * sq(q3);
    SPP[17] = day * SF[19] + day * sq(q0) + day * sq(q1) + day * sq(q3);
    SPP[18] = dax * SF[19] + dax * sq(q0) + dax * sq(q1) + dax * sq(q3);
    SPP[19] = SF[16] * SF[23] - SF[17] * (2 * q0 * q2 + 2 * q1 * q3);
    SPP[20] = SF[16] * SF[21] - SF[18] * SF[22];
    SPP[21] = 2 * q0 * q2 + 2 * q1 * q3;
    SPP[22] = SF[15];

    if (inhibitMagStates)
    {
        zeroRows(P, 16, 21);
        zeroCols(P, 16, 21);
    }
    else if (inhibitWindStates)
    {
        zeroRows(P, 22, 23);
        zeroCols(P, 22, 23);
    }

    if (!inhibitMagStates && lastInhibitMagStates)
    {
        // when starting 3D fusion we want to reset body mag variances
        needMagBodyVarReset = true;
    }

    if (needMagBodyVarReset)
    {
        // reset body mag variances
        needMagBodyVarReset = false;
        zeroCols(P, 19, 21);
        zeroRows(P, 19, 21);
        P[19][19] = sq(ekfParam._magNoise);
        P[20][20] = P[19][19];
        P[21][21] = P[19][19];
    }

    lastInhibitMagStates = inhibitMagStates;

    nextP[0][0] = daxNoise * SQ[3] + SPP[5] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[9][0] * SPP[22] + P[12][0] * SPP[18] + P[2][0] * (2 * q1 * SF[3] - 2 * q2 * SF[4] - 2 * q3 * SF[5] + 2 * q0 * SF[9])) - SPP[4] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[9][1] * SPP[22] + P[12][1] * SPP[18] + P[2][1] * (2 * q1 * SF[3] - 2 * q2 * SF[4] - 2 * q3 * SF[5] + 2 * q0 * SF[9])) + SPP[8] * (P[0][2] * SPP[5] + P[2][2] * SPP[8] + P[9][2] * SPP[22] + P[12][2] * SPP[18] - P[1][2] * (2 * q0 * SF[6] - 2 * q3 * SF[7] - 2 * q1 * SF[10] + 2 * q2 * SF[12])) + SPP[22] * (P[0][9] * SPP[5] - P[1][9] * SPP[4] + P[9][9] * SPP[22] + P[12][9] * SPP[18] + P[2][9] * (2 * q1 * SF[3] - 2 * q2 * SF[4] - 2 * q3 * SF[5] + 2 * q0 * SF[9])) + SPP[18] * (P[0][12] * SPP[5] - P[1][12] * SPP[4] + P[9][12] * SPP[22] + P[12][12] * SPP[18] + P[2][12] * (2 * q1 * SF[3] - 2 * q2 * SF[4] - 2 * q3 * SF[5] + 2 * q0 * SF[9]));
    nextP[0][1] = SPP[6] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[8] + P[9][1] * SPP[22] + P[12][1] * SPP[18]) - SPP[2] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[8] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) + SPP[22] * (P[0][10] * SPP[5] - P[1][10] * SPP[4] + P[2][10] * SPP[8] + P[9][10] * SPP[22] + P[12][10] * SPP[18]) + SPP[17] * (P[0][13] * SPP[5] - P[1][13] * SPP[4] + P[2][13] * SPP[8] + P[9][13] * SPP[22] + P[12][13] * SPP[18]) - (2 * q0 * SF[5] - 2 * q1 * SF[4] - 2 * q2 * SF[3] + 2 * q3 * SF[9]) * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[8] + P[9][2] * SPP[22] + P[12][2] * SPP[18]);
    nextP[1][1] = dayNoise * SQ[3] - SPP[2] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[9] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) + SPP[6] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[9] + P[10][1] * SPP[22] + P[13][1] * SPP[17]) - SPP[9] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[9] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) + SPP[22] * (P[1][10] * SPP[6] - P[0][10] * SPP[2] - P[2][10] * SPP[9] + P[10][10] * SPP[22] + P[13][10] * SPP[17]) + SPP[17] * (P[1][13] * SPP[6] - P[0][13] * SPP[2] - P[2][13] * SPP[9] + P[10][13] * SPP[22] + P[13][13] * SPP[17]);
    nextP[0][2] = SPP[13] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[8] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) - SPP[3] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[8] + P[9][1] * SPP[22] + P[12][1] * SPP[18]) + SPP[22] * (P[0][11] * SPP[5] - P[1][11] * SPP[4] + P[2][11] * SPP[8] + P[9][11] * SPP[22] + P[12][11] * SPP[18]) + SPP[16] * (P[0][14] * SPP[5] - P[1][14] * SPP[4] + P[2][14] * SPP[8] + P[9][14] * SPP[22] + P[12][14] * SPP[18]) + (2 * q2 * SF[8] - 2 * q0 * SF[11] - 2 * q1 * SF[14] + 2 * q3 * SF[13]) * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[8] + P[9][0] * SPP[22] + P[12][0] * SPP[18]);
    nextP[1][2] = SPP[13] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[9] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) - SPP[3] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[9] + P[10][1] * SPP[22] + P[13][1] * SPP[17]) + SPP[22] * (P[1][11] * SPP[6] - P[0][11] * SPP[2] - P[2][11] * SPP[9] + P[10][11] * SPP[22] + P[13][11] * SPP[17]) + SPP[16] * (P[1][14] * SPP[6] - P[0][14] * SPP[2] - P[2][14] * SPP[9] + P[10][14] * SPP[22] + P[13][14] * SPP[17]) + (2 * q2 * SF[8] - 2 * q0 * SF[11] - 2 * q1 * SF[14] + 2 * q3 * SF[13]) * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[9] + P[10][0] * SPP[22] + P[13][0] * SPP[17]);
    nextP[2][2] = dazNoise * SQ[3] - SPP[3] * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]) + SPP[14] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) + SPP[13] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]) + SPP[22] * (P[0][11] * SPP[14] - P[1][11] * SPP[3] + P[2][11] * SPP[13] + P[11][11] * SPP[22] + P[14][11] * SPP[16]) + SPP[16] * (P[0][14] * SPP[14] - P[1][14] * SPP[3] + P[2][14] * SPP[13] + P[11][14] * SPP[22] + P[14][14] * SPP[16]);
    nextP[0][3] = P[0][3] * SPP[5] - P[1][3] * SPP[4] + P[2][3] * SPP[8] + P[9][3] * SPP[22] + P[12][3] * SPP[18] + SPP[1] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[8] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) + SPP[15] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[8] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) - SPP[21] * (P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[8] + P[9][15] * SPP[22] + P[12][15] * SPP[18]) + (SF[16] * SF[23] - SF[17] * SPP[21]) * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[8] + P[9][1] * SPP[22] + P[12][1] * SPP[18]);
    nextP[1][3] = P[1][3] * SPP[6] - P[0][3] * SPP[2] - P[2][3] * SPP[9] + P[10][3] * SPP[22] + P[13][3] * SPP[17] + SPP[1] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[9] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) + SPP[15] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[9] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) - SPP[21] * (P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[9] + P[10][15] * SPP[22] + P[13][15] * SPP[17]) + (SF[16] * SF[23] - SF[17] * SPP[21]) * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[9] + P[10][1] * SPP[22] + P[13][1] * SPP[17]);
    nextP[2][3] = P[0][3] * SPP[14] - P[1][3] * SPP[3] + P[2][3] * SPP[13] + P[11][3] * SPP[22] + P[14][3] * SPP[16] + SPP[1] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) + SPP[15] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]) - SPP[21] * (P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16]) + (SF[16] * SF[23] - SF[17] * SPP[21]) * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]);
    nextP[3][3] = P[3][3] + P[0][3] * SPP[1] + P[1][3] * SPP[19] + P[2][3] * SPP[15] - P[15][3] * SPP[21] + dvyNoise * sq(SQ[6] - 2 * q0 * q3) + dvzNoise * sq(SQ[5] + 2 * q0 * q2) + SPP[1] * (P[3][0] + P[0][0] * SPP[1] + P[1][0] * SPP[19] + P[2][0] * SPP[15] - P[15][0] * SPP[21]) + SPP[19] * (P[3][1] + P[0][1] * SPP[1] + P[1][1] * SPP[19] + P[2][1] * SPP[15] - P[15][1] * SPP[21]) + SPP[15] * (P[3][2] + P[0][2] * SPP[1] + P[1][2] * SPP[19] + P[2][2] * SPP[15] - P[15][2] * SPP[21]) - SPP[21] * (P[3][15] + P[0][15] * SPP[1] + P[2][15] * SPP[15] - P[15][15] * SPP[21] + P[1][15] * (SF[16] * SF[23] - SF[17] * SPP[21])) + dvxNoise * sq(SG[1] + SG[2] - SG[3] - SQ[7]);
    nextP[0][4] = P[0][4] * SPP[5] - P[1][4] * SPP[4] + P[2][4] * SPP[8] + P[9][4] * SPP[22] + P[12][4] * SPP[18] + SF[22] * (P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[8] + P[9][15] * SPP[22] + P[12][15] * SPP[18]) + SPP[12] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[8] + P[9][1] * SPP[22] + P[12][1] * SPP[18]) + SPP[20] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[8] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) + SPP[11] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[8] + P[9][2] * SPP[22] + P[12][2] * SPP[18]);
    nextP[1][4] = P[1][4] * SPP[6] - P[0][4] * SPP[2] - P[2][4] * SPP[9] + P[10][4] * SPP[22] + P[13][4] * SPP[17] + SF[22] * (P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[9] + P[10][15] * SPP[22] + P[13][15] * SPP[17]) + SPP[12] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[9] + P[10][1] * SPP[22] + P[13][1] * SPP[17]) + SPP[20] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[9] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) + SPP[11] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[9] + P[10][2] * SPP[22] + P[13][2] * SPP[17]);
    nextP[2][4] = P[0][4] * SPP[14] - P[1][4] * SPP[3] + P[2][4] * SPP[13] + P[11][4] * SPP[22] + P[14][4] * SPP[16] + SF[22] * (P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16]) + SPP[12] * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]) + SPP[20] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) + SPP[11] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]);
    nextP[3][4] = P[3][4] + SQ[2] + P[0][4] * SPP[1] + P[1][4] * SPP[19] + P[2][4] * SPP[15] - P[15][4] * SPP[21] + SF[22] * (P[3][15] + P[0][15] * SPP[1] + P[1][15] * SPP[19] + P[2][15] * SPP[15] - P[15][15] * SPP[21]) + SPP[12] * (P[3][1] + P[0][1] * SPP[1] + P[1][1] * SPP[19] + P[2][1] * SPP[15] - P[15][1] * SPP[21]) + SPP[20] * (P[3][0] + P[0][0] * SPP[1] + P[1][0] * SPP[19] + P[2][0] * SPP[15] - P[15][0] * SPP[21]) + SPP[11] * (P[3][2] + P[0][2] * SPP[1] + P[1][2] * SPP[19] + P[2][2] * SPP[15] - P[15][2] * SPP[21]);
    nextP[4][4] = P[4][4] + P[15][4] * SF[22] + P[0][4] * SPP[20] + P[1][4] * SPP[12] + P[2][4] * SPP[11] + dvxNoise * sq(SQ[6] + 2 * q0 * q3) + dvzNoise * sq(SQ[4] - 2 * q0 * q1) + SF[22] * (P[4][15] + P[15][15] * SF[22] + P[0][15] * SPP[20] + P[1][15] * SPP[12] + P[2][15] * SPP[11]) + SPP[12] * (P[4][1] + P[15][1] * SF[22] + P[0][1] * SPP[20] + P[1][1] * SPP[12] + P[2][1] * SPP[11]) + SPP[20] * (P[4][0] + P[15][0] * SF[22] + P[0][0] * SPP[20] + P[1][0] * SPP[12] + P[2][0] * SPP[11]) + SPP[11] * (P[4][2] + P[15][2] * SF[22] + P[0][2] * SPP[20] + P[1][2] * SPP[12] + P[2][2] * SPP[11]) + dvyNoise * sq(SG[1] - SG[2] + SG[3] - SQ[7]);
    nextP[0][5] = P[0][5] * SPP[5] - P[1][5] * SPP[4] + P[2][5] * SPP[8] + P[9][5] * SPP[22] + P[12][5] * SPP[18] + SF[20] * (P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[8] + P[9][15] * SPP[22] + P[12][15] * SPP[18]) - SPP[7] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[8] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) + SPP[0] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[8] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) + SPP[10] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[8] + P[9][1] * SPP[22] + P[12][1] * SPP[18]);
    nextP[1][5] = P[1][5] * SPP[6] - P[0][5] * SPP[2] - P[2][5] * SPP[9] + P[10][5] * SPP[22] + P[13][5] * SPP[17] + SF[20] * (P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[9] + P[10][15] * SPP[22] + P[13][15] * SPP[17]) - SPP[7] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[9] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) + SPP[0] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[9] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) + SPP[10] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[9] + P[10][1] * SPP[22] + P[13][1] * SPP[17]);
    nextP[2][5] = P[0][5] * SPP[14] - P[1][5] * SPP[3] + P[2][5] * SPP[13] + P[11][5] * SPP[22] + P[14][5] * SPP[16] + SF[20] * (P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16]) - SPP[7] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) + SPP[0] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]) + SPP[10] * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]);
    nextP[3][5] = P[3][5] + SQ[1] + P[0][5] * SPP[1] + P[1][5] * SPP[19] + P[2][5] * SPP[15] - P[15][5] * SPP[21] + SF[20] * (P[3][15] + P[0][15] * SPP[1] + P[1][15] * SPP[19] + P[2][15] * SPP[15] - P[15][15] * SPP[21]) - SPP[7] * (P[3][0] + P[0][0] * SPP[1] + P[1][0] * SPP[19] + P[2][0] * SPP[15] - P[15][0] * SPP[21]) + SPP[0] * (P[3][2] + P[0][2] * SPP[1] + P[1][2] * SPP[19] + P[2][2] * SPP[15] - P[15][2] * SPP[21]) + SPP[10] * (P[3][1] + P[0][1] * SPP[1] + P[1][1] * SPP[19] + P[2][1] * SPP[15] - P[15][1] * SPP[21]);
    nextP[4][5] = P[4][5] + SQ[0] + P[15][5] * SF[22] + P[0][5] * SPP[20] + P[1][5] * SPP[12] + P[2][5] * SPP[11] + SF[20] * (P[4][15] + P[15][15] * SF[22] + P[0][15] * SPP[20] + P[1][15] * SPP[12] + P[2][15] * SPP[11]) - SPP[7] * (P[4][0] + P[15][0] * SF[22] + P[0][0] * SPP[20] + P[1][0] * SPP[12] + P[2][0] * SPP[11]) + SPP[0] * (P[4][2] + P[15][2] * SF[22] + P[0][2] * SPP[20] + P[1][2] * SPP[12] + P[2][2] * SPP[11]) + SPP[10] * (P[4][1] + P[15][1] * SF[22] + P[0][1] * SPP[20] + P[1][1] * SPP[12] + P[2][1] * SPP[11]);
    nextP[5][5] = P[5][5] + P[15][5] * SF[20] - P[0][5] * SPP[7] + P[1][5] * SPP[10] + P[2][5] * SPP[0] + dvxNoise * sq(SQ[5] - 2 * q0 * q2) + dvyNoise * sq(SQ[4] + 2 * q0 * q1) + SF[20] * (P[5][15] + P[15][15] * SF[20] - P[0][15] * SPP[7] + P[1][15] * SPP[10] + P[2][15] * SPP[0]) - SPP[7] * (P[5][0] + P[15][0] * SF[20] - P[0][0] * SPP[7] + P[1][0] * SPP[10] + P[2][0] * SPP[0]) + SPP[0] * (P[5][2] + P[15][2] * SF[20] - P[0][2] * SPP[7] + P[1][2] * SPP[10] + P[2][2] * SPP[0]) + SPP[10] * (P[5][1] + P[15][1] * SF[20] - P[0][1] * SPP[7] + P[1][1] * SPP[10] + P[2][1] * SPP[0]) + dvzNoise * sq(SG[1] - SG[2] - SG[3] + SQ[7]);
    nextP[0][6] = P[0][6] * SPP[5] - P[1][6] * SPP[4] + P[2][6] * SPP[8] + P[9][6] * SPP[22] + P[12][6] * SPP[18] + dt * (P[0][3] * SPP[5] - P[1][3] * SPP[4] + P[2][3] * SPP[8] + P[9][3] * SPP[22] + P[12][3] * SPP[18]);
    nextP[1][6] = P[1][6] * SPP[6] - P[0][6] * SPP[2] - P[2][6] * SPP[9] + P[10][6] * SPP[22] + P[13][6] * SPP[17] + dt * (P[1][3] * SPP[6] - P[0][3] * SPP[2] - P[2][3] * SPP[9] + P[10][3] * SPP[22] + P[13][3] * SPP[17]);
    nextP[2][6] = P[0][6] * SPP[14] - P[1][6] * SPP[3] + P[2][6] * SPP[13] + P[11][6] * SPP[22] + P[14][6] * SPP[16] + dt * (P[0][3] * SPP[14] - P[1][3] * SPP[3] + P[2][3] * SPP[13] + P[11][3] * SPP[22] + P[14][3] * SPP[16]);
    nextP[3][6] = P[3][6] + P[0][6] * SPP[1] + P[1][6] * SPP[19] + P[2][6] * SPP[15] - P[15][6] * SPP[21] + dt * (P[3][3] + P[0][3] * SPP[1] + P[1][3] * SPP[19] + P[2][3] * SPP[15] - P[15][3] * SPP[21]);
    nextP[4][6] = P[4][6] + P[15][6] * SF[22] + P[0][6] * SPP[20] + P[1][6] * SPP[12] + P[2][6] * SPP[11] + dt * (P[4][3] + P[15][3] * SF[22] + P[0][3] * SPP[20] + P[1][3] * SPP[12] + P[2][3] * SPP[11]);
    nextP[5][6] = P[5][6] + P[15][6] * SF[20] - P[0][6] * SPP[7] + P[1][6] * SPP[10] + P[2][6] * SPP[0] + dt * (P[5][3] + P[15][3] * SF[20] - P[0][3] * SPP[7] + P[1][3] * SPP[10] + P[2][3] * SPP[0]);
    nextP[6][6] = P[6][6] + P[3][6] * dt + dt * (P[6][3] + P[3][3] * dt);
    nextP[0][7] = P[0][7] * SPP[5] - P[1][7] * SPP[4] + P[2][7] * SPP[8] + P[9][7] * SPP[22] + P[12][7] * SPP[18] + dt * (P[0][4] * SPP[5] - P[1][4] * SPP[4] + P[2][4] * SPP[8] + P[9][4] * SPP[22] + P[12][4] * SPP[18]);
    nextP[1][7] = P[1][7] * SPP[6] - P[0][7] * SPP[2] - P[2][7] * SPP[9] + P[10][7] * SPP[22] + P[13][7] * SPP[17] + dt * (P[1][4] * SPP[6] - P[0][4] * SPP[2] - P[2][4] * SPP[9] + P[10][4] * SPP[22] + P[13][4] * SPP[17]);
    nextP[2][7] = P[0][7] * SPP[14] - P[1][7] * SPP[3] + P[2][7] * SPP[13] + P[11][7] * SPP[22] + P[14][7] * SPP[16] + dt * (P[0][4] * SPP[14] - P[1][4] * SPP[3] + P[2][4] * SPP[13] + P[11][4] * SPP[22] + P[14][4] * SPP[16]);
    nextP[3][7] = P[3][7] + P[0][7] * SPP[1] + P[1][7] * SPP[19] + P[2][7] * SPP[15] - P[15][7] * SPP[21] + dt * (P[3][4] + P[0][4] * SPP[1] + P[1][4] * SPP[19] + P[2][4] * SPP[15] - P[15][4] * SPP[21]);
    nextP[4][7] = P[4][7] + P[15][7] * SF[22] + P[0][7] * SPP[20] + P[1][7] * SPP[12] + P[2][7] * SPP[11] + dt * (P[4][4] + P[15][4] * SF[22] + P[0][4] * SPP[20] + P[1][4] * SPP[12] + P[2][4] * SPP[11]);
    nextP[5][7] = P[5][7] + P[15][7] * SF[20] - P[0][7] * SPP[7] + P[1][7] * SPP[10] + P[2][7] * SPP[0] + dt * (P[5][4] + P[15][4] * SF[20] - P[0][4] * SPP[7] + P[1][4] * SPP[10] + P[2][4] * SPP[0]);
    nextP[6][7] = P[6][7] + P[3][7] * dt + dt * (P[6][4] + P[3][4] * dt);
    nextP[7][7] = P[7][7] + P[4][7] * dt + dt * (P[7][4] + P[4][4] * dt);
    nextP[0][8] = P[0][8] * SPP[5] - P[1][8] * SPP[4] + P[2][8] * SPP[8] + P[9][8] * SPP[22] + P[12][8] * SPP[18] + dt * (P[0][5] * SPP[5] - P[1][5] * SPP[4] + P[2][5] * SPP[8] + P[9][5] * SPP[22] + P[12][5] * SPP[18]);
    nextP[1][8] = P[1][8] * SPP[6] - P[0][8] * SPP[2] - P[2][8] * SPP[9] + P[10][8] * SPP[22] + P[13][8] * SPP[17] + dt * (P[1][5] * SPP[6] - P[0][5] * SPP[2] - P[2][5] * SPP[9] + P[10][5] * SPP[22] + P[13][5] * SPP[17]);
    nextP[2][8] = P[0][8] * SPP[14] - P[1][8] * SPP[3] + P[2][8] * SPP[13] + P[11][8] * SPP[22] + P[14][8] * SPP[16] + dt * (P[0][5] * SPP[14] - P[1][5] * SPP[3] + P[2][5] * SPP[13] + P[11][5] * SPP[22] + P[14][5] * SPP[16]);
    nextP[3][8] = P[3][8] + P[0][8] * SPP[1] + P[1][8] * SPP[19] + P[2][8] * SPP[15] - P[15][8] * SPP[21] + dt * (P[3][5] + P[0][5] * SPP[1] + P[1][5] * SPP[19] + P[2][5] * SPP[15] - P[15][5] * SPP[21]);
    nextP[4][8] = P[4][8] + P[15][8] * SF[22] + P[0][8] * SPP[20] + P[1][8] * SPP[12] + P[2][8] * SPP[11] + dt * (P[4][5] + P[15][5] * SF[22] + P[0][5] * SPP[20] + P[1][5] * SPP[12] + P[2][5] * SPP[11]);
    nextP[5][8] = P[5][8] + P[15][8] * SF[20] - P[0][8] * SPP[7] + P[1][8] * SPP[10] + P[2][8] * SPP[0] + dt * (P[5][5] + P[15][5] * SF[20] - P[0][5] * SPP[7] + P[1][5] * SPP[10] + P[2][5] * SPP[0]);
    nextP[6][8] = P[6][8] + P[3][8] * dt + dt * (P[6][5] + P[3][5] * dt);
    nextP[7][8] = P[7][8] + P[4][8] * dt + dt * (P[7][5] + P[4][5] * dt);
    nextP[8][8] = P[8][8] + P[5][8] * dt + dt * (P[8][5] + P[5][5] * dt);
    nextP[0][9] = P[0][9] * SPP[5] - P[1][9] * SPP[4] + P[2][9] * SPP[8] + P[9][9] * SPP[22] + P[12][9] * SPP[18];
    nextP[1][9] = P[1][9] * SPP[6] - P[0][9] * SPP[2] - P[2][9] * SPP[9] + P[10][9] * SPP[22] + P[13][9] * SPP[17];
    nextP[2][9] = P[0][9] * SPP[14] - P[1][9] * SPP[3] + P[2][9] * SPP[13] + P[11][9] * SPP[22] + P[14][9] * SPP[16];
    nextP[3][9] = P[3][9] + P[0][9] * SPP[1] + P[1][9] * SPP[19] + P[2][9] * SPP[15] - P[15][9] * SPP[21];
    nextP[4][9] = P[4][9] + P[15][9] * SF[22] + P[0][9] * SPP[20] + P[1][9] * SPP[12] + P[2][9] * SPP[11];
    nextP[5][9] = P[5][9] + P[15][9] * SF[20] - P[0][9] * SPP[7] + P[1][9] * SPP[10] + P[2][9] * SPP[0];
    nextP[6][9] = P[6][9] + P[3][9] * dt;
    nextP[7][9] = P[7][9] + P[4][9] * dt;
    nextP[8][9] = P[8][9] + P[5][9] * dt;
    nextP[9][9] = P[9][9];
    nextP[0][10] = P[0][10] * SPP[5] - P[1][10] * SPP[4] + P[2][10] * SPP[8] + P[9][10] * SPP[22] + P[12][10] * SPP[18];
    nextP[1][10] = P[1][10] * SPP[6] - P[0][10] * SPP[2] - P[2][10] * SPP[9] + P[10][10] * SPP[22] + P[13][10] * SPP[17];
    nextP[2][10] = P[0][10] * SPP[14] - P[1][10] * SPP[3] + P[2][10] * SPP[13] + P[11][10] * SPP[22] + P[14][10] * SPP[16];
    nextP[3][10] = P[3][10] + P[0][10] * SPP[1] + P[1][10] * SPP[19] + P[2][10] * SPP[15] - P[15][10] * SPP[21];
    nextP[4][10] = P[4][10] + P[15][10] * SF[22] + P[0][10] * SPP[20] + P[1][10] * SPP[12] + P[2][10] * SPP[11];
    nextP[5][10] = P[5][10] + P[15][10] * SF[20] - P[0][10] * SPP[7] + P[1][10] * SPP[10] + P[2][10] * SPP[0];
    nextP[6][10] = P[6][10] + P[3][10] * dt;
    nextP[7][10] = P[7][10] + P[4][10] * dt;
    nextP[8][10] = P[8][10] + P[5][10] * dt;
    nextP[9][10] = P[9][10];
    nextP[10][10] = P[10][10];
    nextP[0][11] = P[0][11] * SPP[5] - P[1][11] * SPP[4] + P[2][11] * SPP[8] + P[9][11] * SPP[22] + P[12][11] * SPP[18];
    nextP[1][11] = P[1][11] * SPP[6] - P[0][11] * SPP[2] - P[2][11] * SPP[9] + P[10][11] * SPP[22] + P[13][11] * SPP[17];
    nextP[2][11] = P[0][11] * SPP[14] - P[1][11] * SPP[3] + P[2][11] * SPP[13] + P[11][11] * SPP[22] + P[14][11] * SPP[16];
    nextP[3][11] = P[3][11] + P[0][11] * SPP[1] + P[1][11] * SPP[19] + P[2][11] * SPP[15] - P[15][11] * SPP[21];
    nextP[4][11] = P[4][11] + P[15][11] * SF[22] + P[0][11] * SPP[20] + P[1][11] * SPP[12] + P[2][11] * SPP[11];
    nextP[5][11] = P[5][11] + P[15][11] * SF[20] - P[0][11] * SPP[7] + P[1][11] * SPP[10] + P[2][11] * SPP[0];
    nextP[6][11] = P[6][11] + P[3][11] * dt;
    nextP[7][11] = P[7][11] + P[4][11] * dt;
    nextP[8][11] = P[8][11] + P[5][11] * dt;
    nextP[9][11] = P[9][11];
    nextP[10][11] = P[10][11];
    nextP[11][11] = P[11][11];
    nextP[0][12] = P[0][12] * SPP[5] - P[1][12] * SPP[4] + P[2][12] * SPP[8] + P[9][12] * SPP[22] + P[12][12] * SPP[18];
    nextP[1][12] = P[1][12] * SPP[6] - P[0][12] * SPP[2] - P[2][12] * SPP[9] + P[10][12] * SPP[22] + P[13][12] * SPP[17];
    nextP[2][12] = P[0][12] * SPP[14] - P[1][12] * SPP[3] + P[2][12] * SPP[13] + P[11][12] * SPP[22] + P[14][12] * SPP[16];
    nextP[3][12] = P[3][12] + P[0][12] * SPP[1] + P[1][12] * SPP[19] + P[2][12] * SPP[15] - P[15][12] * SPP[21];
    nextP[4][12] = P[4][12] + P[15][12] * SF[22] + P[0][12] * SPP[20] + P[1][12] * SPP[12] + P[2][12] * SPP[11];
    nextP[5][12] = P[5][12] + P[15][12] * SF[20] - P[0][12] * SPP[7] + P[1][12] * SPP[10] + P[2][12] * SPP[0];
    nextP[6][12] = P[6][12] + P[3][12] * dt;
    nextP[7][12] = P[7][12] + P[4][12] * dt;
    nextP[8][12] = P[8][12] + P[5][12] * dt;
    nextP[9][12] = P[9][12];
    nextP[10][12] = P[10][12];
    nextP[11][12] = P[11][12];
    nextP[12][12] = P[12][12];
    nextP[0][13] = P[0][13] * SPP[5] - P[1][13] * SPP[4] + P[2][13] * SPP[8] + P[9][13] * SPP[22] + P[12][13] * SPP[18];
    nextP[1][13] = P[1][13] * SPP[6] - P[0][13] * SPP[2] - P[2][13] * SPP[9] + P[10][13] * SPP[22] + P[13][13] * SPP[17];
    nextP[2][13] = P[0][13] * SPP[14] - P[1][13] * SPP[3] + P[2][13] * SPP[13] + P[11][13] * SPP[22] + P[14][13] * SPP[16];
    nextP[3][13] = P[3][13] + P[0][13] * SPP[1] + P[1][13] * SPP[19] + P[2][13] * SPP[15] - P[15][13] * SPP[21];
    nextP[4][13] = P[4][13] + P[15][13] * SF[22] + P[0][13] * SPP[20] + P[1][13] * SPP[12] + P[2][13] * SPP[11];
    nextP[5][13] = P[5][13] + P[15][13] * SF[20] - P[0][13] * SPP[7] + P[1][13] * SPP[10] + P[2][13] * SPP[0];
    nextP[6][13] = P[6][13] + P[3][13] * dt;
    nextP[7][13] = P[7][13] + P[4][13] * dt;
    nextP[8][13] = P[8][13] + P[5][13] * dt;
    nextP[9][13] = P[9][13];
    nextP[10][13] = P[10][13];
    nextP[11][13] = P[11][13];
    nextP[12][13] = P[12][13];
    nextP[13][13] = P[13][13];
    nextP[0][14] = P[0][14] * SPP[5] - P[1][14] * SPP[4] + P[2][14] * SPP[8] + P[9][14] * SPP[22] + P[12][14] * SPP[18];
    nextP[1][14] = P[1][14] * SPP[6] - P[0][14] * SPP[2] - P[2][14] * SPP[9] + P[10][14] * SPP[22] + P[13][14] * SPP[17];
    nextP[2][14] = P[0][14] * SPP[14] - P[1][14] * SPP[3] + P[2][14] * SPP[13] + P[11][14] * SPP[22] + P[14][14] * SPP[16];
    nextP[3][14] = P[3][14] + P[0][14] * SPP[1] + P[1][14] * SPP[19] + P[2][14] * SPP[15] - P[15][14] * SPP[21];
    nextP[4][14] = P[4][14] + P[15][14] * SF[22] + P[0][14] * SPP[20] + P[1][14] * SPP[12] + P[2][14] * SPP[11];
    nextP[5][14] = P[5][14] + P[15][14] * SF[20] - P[0][14] * SPP[7] + P[1][14] * SPP[10] + P[2][14] * SPP[0];
    nextP[6][14] = P[6][14] + P[3][14] * dt;
    nextP[7][14] = P[7][14] + P[4][14] * dt;
    nextP[8][14] = P[8][14] + P[5][14] * dt;
    nextP[9][14] = P[9][14];
    nextP[10][14] = P[10][14];
    nextP[11][14] = P[11][14];
    nextP[12][14] = P[12][14];
    nextP[13][14] = P[13][14];
    nextP[14][14] = P[14][14];
    nextP[0][15] = P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[8] + P[9][15] * SPP[22] + P[12][15] * SPP[18];
    nextP[1][15] = P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[9] + P[10][15] * SPP[22] + P[13][15] * SPP[17];
    nextP[2][15] = P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16];
    nextP[3][15] = P[3][15] + P[0][15] * SPP[1] + P[1][15] * SPP[19] + P[2][15] * SPP[15] - P[15][15] * SPP[21];
    nextP[4][15] = P[4][15] + P[15][15] * SF[22] + P[0][15] * SPP[20] + P[1][15] * SPP[12] + P[2][15] * SPP[11];
    nextP[5][15] = P[5][15] + P[15][15] * SF[20] - P[0][15] * SPP[7] + P[1][15] * SPP[10] + P[2][15] * SPP[0];
    nextP[6][15] = P[6][15] + P[3][15] * dt;
    nextP[7][15] = P[7][15] + P[4][15] * dt;
    nextP[8][15] = P[8][15] + P[5][15] * dt;
    nextP[9][15] = P[9][15];
    nextP[10][15] = P[10][15];
    nextP[11][15] = P[11][15];
    nextP[12][15] = P[12][15];
    nextP[13][15] = P[13][15];
    nextP[14][15] = P[14][15];
    nextP[15][15] = P[15][15];

    if (stateIndexLim > 15)
    {
        nextP[0][16] = P[0][16] * SPP[5] - P[1][16] * SPP[4] + P[2][16] * SPP[8] + P[9][16] * SPP[22] + P[12][16] * SPP[18];
        nextP[1][16] = P[1][16] * SPP[6] - P[0][16] * SPP[2] - P[2][16] * SPP[9] + P[10][16] * SPP[22] + P[13][16] * SPP[17];
        nextP[2][16] = P[0][16] * SPP[14] - P[1][16] * SPP[3] + P[2][16] * SPP[13] + P[11][16] * SPP[22] + P[14][16] * SPP[16];
        nextP[3][16] = P[3][16] + P[0][16] * SPP[1] + P[1][16] * SPP[19] + P[2][16] * SPP[15] - P[15][16] * SPP[21];
        nextP[4][16] = P[4][16] + P[15][16] * SF[22] + P[0][16] * SPP[20] + P[1][16] * SPP[12] + P[2][16] * SPP[11];
        nextP[5][16] = P[5][16] + P[15][16] * SF[20] - P[0][16] * SPP[7] + P[1][16] * SPP[10] + P[2][16] * SPP[0];
        nextP[6][16] = P[6][16] + P[3][16] * dt;
        nextP[7][16] = P[7][16] + P[4][16] * dt;
        nextP[8][16] = P[8][16] + P[5][16] * dt;
        nextP[9][16] = P[9][16];
        nextP[10][16] = P[10][16];
        nextP[11][16] = P[11][16];
        nextP[12][16] = P[12][16];
        nextP[13][16] = P[13][16];
        nextP[14][16] = P[14][16];
        nextP[15][16] = P[15][16];
        nextP[16][16] = P[16][16];
        nextP[0][17] = P[0][17] * SPP[5] - P[1][17] * SPP[4] + P[2][17] * SPP[8] + P[9][17] * SPP[22] + P[12][17] * SPP[18];
        nextP[1][17] = P[1][17] * SPP[6] - P[0][17] * SPP[2] - P[2][17] * SPP[9] + P[10][17] * SPP[22] + P[13][17] * SPP[17];
        nextP[2][17] = P[0][17] * SPP[14] - P[1][17] * SPP[3] + P[2][17] * SPP[13] + P[11][17] * SPP[22] + P[14][17] * SPP[16];
        nextP[3][17] = P[3][17] + P[0][17] * SPP[1] + P[1][17] * SPP[19] + P[2][17] * SPP[15] - P[15][17] * SPP[21];
        nextP[4][17] = P[4][17] + P[15][17] * SF[22] + P[0][17] * SPP[20] + P[1][17] * SPP[12] + P[2][17] * SPP[11];
        nextP[5][17] = P[5][17] + P[15][17] * SF[20] - P[0][17] * SPP[7] + P[1][17] * SPP[10] + P[2][17] * SPP[0];
        nextP[6][17] = P[6][17] + P[3][17] * dt;
        nextP[7][17] = P[7][17] + P[4][17] * dt;
        nextP[8][17] = P[8][17] + P[5][17] * dt;
        nextP[9][17] = P[9][17];
        nextP[10][17] = P[10][17];
        nextP[11][17] = P[11][17];
        nextP[12][17] = P[12][17];
        nextP[13][17] = P[13][17];
        nextP[14][17] = P[14][17];
        nextP[15][17] = P[15][17];
        nextP[16][17] = P[16][17];
        nextP[17][17] = P[17][17];
        nextP[0][18] = P[0][18] * SPP[5] - P[1][18] * SPP[4] + P[2][18] * SPP[8] + P[9][18] * SPP[22] + P[12][18] * SPP[18];
        nextP[1][18] = P[1][18] * SPP[6] - P[0][18] * SPP[2] - P[2][18] * SPP[9] + P[10][18] * SPP[22] + P[13][18] * SPP[17];
        nextP[2][18] = P[0][18] * SPP[14] - P[1][18] * SPP[3] + P[2][18] * SPP[13] + P[11][18] * SPP[22] + P[14][18] * SPP[16];
        nextP[3][18] = P[3][18] + P[0][18] * SPP[1] + P[1][18] * SPP[19] + P[2][18] * SPP[15] - P[15][18] * SPP[21];
        nextP[4][18] = P[4][18] + P[15][18] * SF[22] + P[0][18] * SPP[20] + P[1][18] * SPP[12] + P[2][18] * SPP[11];
        nextP[5][18] = P[5][18] + P[15][18] * SF[20] - P[0][18] * SPP[7] + P[1][18] * SPP[10] + P[2][18] * SPP[0];
        nextP[6][18] = P[6][18] + P[3][18] * dt;
        nextP[7][18] = P[7][18] + P[4][18] * dt;
        nextP[8][18] = P[8][18] + P[5][18] * dt;
        nextP[9][18] = P[9][18];
        nextP[10][18] = P[10][18];
        nextP[11][18] = P[11][18];
        nextP[12][18] = P[12][18];
        nextP[13][18] = P[13][18];
        nextP[14][18] = P[14][18];
        nextP[15][18] = P[15][18];
        nextP[16][18] = P[16][18];
        nextP[17][18] = P[17][18];
        nextP[18][18] = P[18][18];
        nextP[0][19] = P[0][19] * SPP[5] - P[1][19] * SPP[4] + P[2][19] * SPP[8] + P[9][19] * SPP[22] + P[12][19] * SPP[18];
        nextP[1][19] = P[1][19] * SPP[6] - P[0][19] * SPP[2] - P[2][19] * SPP[9] + P[10][19] * SPP[22] + P[13][19] * SPP[17];
        nextP[2][19] = P[0][19] * SPP[14] - P[1][19] * SPP[3] + P[2][19] * SPP[13] + P[11][19] * SPP[22] + P[14][19] * SPP[16];
        nextP[3][19] = P[3][19] + P[0][19] * SPP[1] + P[1][19] * SPP[19] + P[2][19] * SPP[15] - P[15][19] * SPP[21];
        nextP[4][19] = P[4][19] + P[15][19] * SF[22] + P[0][19] * SPP[20] + P[1][19] * SPP[12] + P[2][19] * SPP[11];
        nextP[5][19] = P[5][19] + P[15][19] * SF[20] - P[0][19] * SPP[7] + P[1][19] * SPP[10] + P[2][19] * SPP[0];
        nextP[6][19] = P[6][19] + P[3][19] * dt;
        nextP[7][19] = P[7][19] + P[4][19] * dt;
        nextP[8][19] = P[8][19] + P[5][19] * dt;
        nextP[9][19] = P[9][19];
        nextP[10][19] = P[10][19];
        nextP[11][19] = P[11][19];
        nextP[12][19] = P[12][19];
        nextP[13][19] = P[13][19];
        nextP[14][19] = P[14][19];
        nextP[15][19] = P[15][19];
        nextP[16][19] = P[16][19];
        nextP[17][19] = P[17][19];
        nextP[18][19] = P[18][19];
        nextP[19][19] = P[19][19];
        nextP[0][20] = P[0][20] * SPP[5] - P[1][20] * SPP[4] + P[2][20] * SPP[8] + P[9][20] * SPP[22] + P[12][20] * SPP[18];
        nextP[1][20] = P[1][20] * SPP[6] - P[0][20] * SPP[2] - P[2][20] * SPP[9] + P[10][20] * SPP[22] + P[13][20] * SPP[17];
        nextP[2][20] = P[0][20] * SPP[14] - P[1][20] * SPP[3] + P[2][20] * SPP[13] + P[11][20] * SPP[22] + P[14][20] * SPP[16];
        nextP[3][20] = P[3][20] + P[0][20] * SPP[1] + P[1][20] * SPP[19] + P[2][20] * SPP[15] - P[15][20] * SPP[21];
        nextP[4][20] = P[4][20] + P[15][20] * SF[22] + P[0][20] * SPP[20] + P[1][20] * SPP[12] + P[2][20] * SPP[11];
        nextP[5][20] = P[5][20] + P[15][20] * SF[20] - P[0][20] * SPP[7] + P[1][20] * SPP[10] + P[2][20] * SPP[0];
        nextP[6][20] = P[6][20] + P[3][20] * dt;
        nextP[7][20] = P[7][20] + P[4][20] * dt;
        nextP[8][20] = P[8][20] + P[5][20] * dt;
        nextP[9][20] = P[9][20];
        nextP[10][20] = P[10][20];
        nextP[11][20] = P[11][20];
        nextP[12][20] = P[12][20];
        nextP[13][20] = P[13][20];
        nextP[14][20] = P[14][20];
        nextP[15][20] = P[15][20];
        nextP[16][20] = P[16][20];
        nextP[17][20] = P[17][20];
        nextP[18][20] = P[18][20];
        nextP[19][20] = P[19][20];
        nextP[20][20] = P[20][20];
        nextP[0][21] = P[0][21] * SPP[5] - P[1][21] * SPP[4] + P[2][21] * SPP[8] + P[9][21] * SPP[22] + P[12][21] * SPP[18];
        nextP[1][21] = P[1][21] * SPP[6] - P[0][21] * SPP[2] - P[2][21] * SPP[9] + P[10][21] * SPP[22] + P[13][21] * SPP[17];
        nextP[2][21] = P[0][21] * SPP[14] - P[1][21] * SPP[3] + P[2][21] * SPP[13] + P[11][21] * SPP[22] + P[14][21] * SPP[16];
        nextP[3][21] = P[3][21] + P[0][21] * SPP[1] + P[1][21] * SPP[19] + P[2][21] * SPP[15] - P[15][21] * SPP[21];
        nextP[4][21] = P[4][21] + P[15][21] * SF[22] + P[0][21] * SPP[20] + P[1][21] * SPP[12] + P[2][21] * SPP[11];
        nextP[5][21] = P[5][21] + P[15][21] * SF[20] - P[0][21] * SPP[7] + P[1][21] * SPP[10] + P[2][21] * SPP[0];
        nextP[6][21] = P[6][21] + P[3][21] * dt;
        nextP[7][21] = P[7][21] + P[4][21] * dt;
        nextP[8][21] = P[8][21] + P[5][21] * dt;
        nextP[9][21] = P[9][21];
        nextP[10][21] = P[10][21];
        nextP[11][21] = P[11][21];
        nextP[12][21] = P[12][21];
        nextP[13][21] = P[13][21];
        nextP[14][21] = P[14][21];
        nextP[15][21] = P[15][21];
        nextP[16][21] = P[16][21];
        nextP[17][21] = P[17][21];
        nextP[18][21] = P[18][21];
        nextP[19][21] = P[19][21];
        nextP[20][21] = P[20][21];
        nextP[21][21] = P[21][21];

        if (stateIndexLim > 21)
        {
            nextP[0][22] = P[0][22] * SPP[5] - P[1][22] * SPP[4] + P[2][22] * SPP[8] + P[9][22] * SPP[22] + P[12][22] * SPP[18];
            nextP[1][22] = P[1][22] * SPP[6] - P[0][22] * SPP[2] - P[2][22] * SPP[9] + P[10][22] * SPP[22] + P[13][22] * SPP[17];
            nextP[2][22] = P[0][22] * SPP[14] - P[1][22] * SPP[3] + P[2][22] * SPP[13] + P[11][22] * SPP[22] + P[14][22] * SPP[16];
            nextP[3][22] = P[3][22] + P[0][22] * SPP[1] + P[1][22] * SPP[19] + P[2][22] * SPP[15] - P[15][22] * SPP[21];
            nextP[4][22] = P[4][22] + P[15][22] * SF[22] + P[0][22] * SPP[20] + P[1][22] * SPP[12] + P[2][22] * SPP[11];
            nextP[5][22] = P[5][22] + P[15][22] * SF[20] - P[0][22] * SPP[7] + P[1][22] * SPP[10] + P[2][22] * SPP[0];
            nextP[6][22] = P[6][22] + P[3][22] * dt;
            nextP[7][22] = P[7][22] + P[4][22] * dt;
            nextP[8][22] = P[8][22] + P[5][22] * dt;
            nextP[9][22] = P[9][22];
            nextP[10][22] = P[10][22];
            nextP[11][22] = P[11][22];
            nextP[12][22] = P[12][22];
            nextP[13][22] = P[13][22];
            nextP[14][22] = P[14][22];
            nextP[15][22] = P[15][22];
            nextP[16][22] = P[16][22];
            nextP[17][22] = P[17][22];
            nextP[18][22] = P[18][22];
            nextP[19][22] = P[19][22];
            nextP[20][22] = P[20][22];
            nextP[21][22] = P[21][22];
            nextP[22][22] = P[22][22];
            nextP[0][23] = P[0][23] * SPP[5] - P[1][23] * SPP[4] + P[2][23] * SPP[8] + P[9][23] * SPP[22] + P[12][23] * SPP[18];
            nextP[1][23] = P[1][23] * SPP[6] - P[0][23] * SPP[2] - P[2][23] * SPP[9] + P[10][23] * SPP[22] + P[13][23] * SPP[17];
            nextP[2][23] = P[0][23] * SPP[14] - P[1][23] * SPP[3] + P[2][23] * SPP[13] + P[11][23] * SPP[22] + P[14][23] * SPP[16];
            nextP[3][23] = P[3][23] + P[0][23] * SPP[1] + P[1][23] * SPP[19] + P[2][23] * SPP[15] - P[15][23] * SPP[21];
            nextP[4][23] = P[4][23] + P[15][23] * SF[22] + P[0][23] * SPP[20] + P[1][23] * SPP[12] + P[2][23] * SPP[11];
            nextP[5][23] = P[5][23] + P[15][23] * SF[20] - P[0][23] * SPP[7] + P[1][23] * SPP[10] + P[2][23] * SPP[0];
            nextP[6][23] = P[6][23] + P[3][23] * dt;
            nextP[7][23] = P[7][23] + P[4][23] * dt;
            nextP[8][23] = P[8][23] + P[5][23] * dt;
            nextP[9][23] = P[9][23];
            nextP[10][23] = P[10][23];
            nextP[11][23] = P[11][23];
            nextP[12][23] = P[12][23];
            nextP[13][23] = P[13][23];
            nextP[14][23] = P[14][23];
            nextP[15][23] = P[15][23];
            nextP[16][23] = P[16][23];
            nextP[17][23] = P[17][23];
            nextP[18][23] = P[18][23];
            nextP[19][23] = P[19][23];
            nextP[20][23] = P[20][23];
            nextP[21][23] = P[21][23];
            nextP[22][23] = P[22][23];
            nextP[23][23] = P[23][23];
        }
    }

    // Copy upper diagonal to lower diagonal taking advantage of symmetry
    for (uint8_t colIndex = 0; colIndex <= stateIndexLim; colIndex++)
    {
        for (uint8_t rowIndex = 0; rowIndex < colIndex; rowIndex++)
        {
            nextP[colIndex][rowIndex] = nextP[rowIndex][colIndex];
        }
    }

    // add the general state process noise variances
    for (uint8_t i = 0; i <= stateIndexLim; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[6][6] + P[7][7]) > 1e4f)
    {
        for (uint8_t i = 6; i <= 7; i++)
        {
            for (uint8_t j = 0; j <= stateIndexLim; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output
    CopyCovariances();

    // constrain diagonals to prevent ill-conditioning
    ConstrainVariances();
}

// zero part of an array for index range [n1,n2]
static void zero_range(float *v, uint8_t n1, uint8_t n2)
{
    memset(&v[n1], 0, sizeof(float) * (1 + (n2 - n1)));
}

// zero specified range of rows in the state covariance matrix
void zeroRows(Matrix24 covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = first; row <= last; row++)
    {
        zero_range(&covMat[row][0], 0, 23);
    }
}

// zero specified range of columns in the state covariance matrix
void zeroCols(Matrix24 covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = 0; row <= 23; row++)
    {
        zero_range(&covMat[row][0], first, last);
    }
}

// reset the output data to the current EKF state
void StoreOutputReset(void)
{
    outputDataNew.quat = ekfStates.stateStruct.quat;
    outputDataNew.velocity = ekfStates.stateStruct.velocity;
    outputDataNew.position = ekfStates.stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i = 0; i < imu_buffer_length; i++)
    {
        memcpy(ekf_output_buffer_get(&storedOutput, i), &outputDataNew, sizeof(output_elements_t));
    }
    outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position dervative output
    vertCompFiltState.pos = ekfStates.stateStruct.position.z;
    vertCompFiltState.vel = ekfStates.stateStruct.velocity.z;
}

// Rotate the stored output quaternion history through a quaternion rotation
void StoreQuatRotate(fpQuaternion_t *deltaQuat)
{
    outputDataNew.quat = quaternion_multiply(outputDataNew.quat, *deltaQuat);

    // write current measurement to entire table
    for (uint8_t i = 0; i < imu_buffer_length; i++)
    {
        storedOutput.buffer.output_buffer[i].quat = quaternion_multiply(storedOutput.buffer.output_buffer[i].quat, *deltaQuat);
    }

    outputDataDelayed.quat = quaternion_multiply(outputDataDelayed.quat, *deltaQuat);
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void ForceSymmetry(void)
{
    for (uint8_t i = 1; i <= stateIndexLim; i++)
    {
        for (uint8_t j = 0; j <= i - 1; j++)
        {
            float temp = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// copy covariances across from covariance prediction calculation
void CopyCovariances(void)
{
    // copy predicted covariances
    for (uint8_t i = 0; i <= stateIndexLim; i++)
    {
        for (uint8_t j = 0; j <= stateIndexLim; j++)
        {
            P[i][j] = nextP[i][j];
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
void ConstrainVariances(void)
{
    for (uint8_t i = 0; i <= 2; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0f); // attitude error

    for (uint8_t i = 3; i <= 5; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e3f); // velocities

    for (uint8_t i = 6; i <= 7; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, EKF_POSXY_STATE_LIMIT);

    P[8][8] = constrainf(P[8][8], 0.0f, 1.0e6f); // vertical position

    for (uint8_t i = 9; i <= 11; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, sq(0.175f * dtEkfAvg)); // delta angle biases

    if (PV_AidingMode != AID_NONE)
    {
        for (uint8_t i = 12; i <= 14; i++)
            P[i][i] = constrainf(P[i][i], 0.0f, 0.01f); // delta angle scale factors
    }
    else
    {
        // we can't reliably estimate scale factors when there is no aiding data due to transient manoeuvre induced innovations
        // so inhibit estimation by keeping covariance elements at zero
        zeroRows(P, 12, 14);
        zeroCols(P, 12, 14);
    }

    P[15][15] = constrainf(P[15][15], 0.0f, sq(10.0f * dtEkfAvg)); // delta velocity bias

    for (uint8_t i = 16; i <= 18; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 0.01f); // earth magnetic field

    for (uint8_t i = 19; i <= 21; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 0.01f); // body magnetic field

    for (uint8_t i = 22; i <= 23; i++)
        P[i][i] = constrainf(P[i][i], 0.0f, 1.0e3f); // wind velocity
}

// constrain states using WMM tables and specified limit
void MagTableConstrain(void)
{
    // constrain to error from table earth field
    float limit_ga = ekfParam._mag_ef_limit * 0.001f;

    ekfStates.stateStruct.earth_magfield.x = constrainf(ekfStates.stateStruct.earth_magfield.x,
                                                        table_earth_field_ga.x - limit_ga,
                                                        table_earth_field_ga.x + limit_ga);

    ekfStates.stateStruct.earth_magfield.y = constrainf(ekfStates.stateStruct.earth_magfield.y,
                                                        table_earth_field_ga.y - limit_ga,
                                                        table_earth_field_ga.y + limit_ga);

    ekfStates.stateStruct.earth_magfield.z = constrainf(ekfStates.stateStruct.earth_magfield.z,
                                                        table_earth_field_ga.z - limit_ga,
                                                        table_earth_field_ga.z + limit_ga);
}

// constrain states to prevent ill-conditioning
void ConstrainStates(void)
{
    // attitude errors are limited between +-1
    for (uint8_t i = 0; i <= 2; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -1.0f, 1.0f);

    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i = 3; i <= 5; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -5.0e2f, 5.0e2f);

    // position limit - TODO apply circular limit
    for (uint8_t i = 6; i <= 7; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -EKF_POSXY_STATE_LIMIT, EKF_POSXY_STATE_LIMIT);

    // height limit covers home alt on everest through to home alt at SL and ballon drop
    ekfStates.stateStruct.position.z = constrainf(ekfStates.stateStruct.position.z, -4.0e4f, 1.0e4f);

    // gyro bias limit (this needs to be set based on manufacturers specs)
    for (uint8_t i = 9; i <= 11; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -GYRO_BIAS_LIMIT * dtEkfAvg, GYRO_BIAS_LIMIT * dtEkfAvg);

    // gyro scale factor limit of +-5% (this needs to be set based on manufacturers specs)
    for (uint8_t i = 12; i <= 14; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], 0.95f, 1.05f);

    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    ekfStates.stateStruct.accel_zbias = constrainf(ekfStates.stateStruct.accel_zbias, -1.0f * dtEkfAvg, 1.0f * dtEkfAvg);

    // earth magnetic field limit
    if (ekfParam._mag_ef_limit <= 0 || !have_table_earth_field)
    {
        // constrain to +/-1Ga
        for (uint8_t i = 16; i <= 18; i++)
            ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -1.0f, 1.0f);
    }
    else
    {
        // use table constrain
        MagTableConstrain();
    }

    // body magnetic field limit
    for (uint8_t i = 19; i <= 21; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -0.5f, 0.5f);

    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i = 22; i <= 23; i++)
        ekfStates.statesArray[i] = constrainf(ekfStates.statesArray[i], -100.0f, 100.0f);

    // constrain the terrain state to be below the vehicle height unless we are using terrain as the height datum
    if (!inhibitGndState)
    {
        terrainState = MAX(terrainState, ekfStates.stateStruct.position.z + rngOnGnd);
    }
}

// calculate the NED earth spin vector in rad/sec
void calcEarthRateNED(fpVector3_t *omega, int32_t latitude)
{
    float lat_rad = DEGREES_TO_RADIANS(latitude * 1.0e-7f);
    omega->x = EARTH_RATE * cosf(lat_rad);
    omega->y = 0.0f;
    omega->z = -EARTH_RATE * sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
fpQuaternion_t calcQuatAndFieldStates(float roll, float pitch)
{
    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    fpMat3_t Tbn;
    fpVector3_t initMagNED;
    fpQuaternion_t initQuat;

    if (ekf_useCompass())
    {
        // calculate rotation matrix from body to NED frame
        matrixFromEuler(&Tbn, roll, pitch, 0.0f);

        // read the magnetometer data
        readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = multiplyMatrixByVector(Tbn, magDataDelayed.mag);

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = MagDeclination();

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;

        // calculate initial filter quaternion states using yaw from magnetometer
        // store the yaw change so that it can be retrieved externally for use by the control loops to prevent yaw disturbances following a reset
        fpVector3_t tempEuler;

        quaternionToEuler(ekfStates.stateStruct.quat, &tempEuler.x, &tempEuler.y, &tempEuler.z);

        // this check ensures we accumulate the resets that occur within a single iteration of the EKF
        if (imuSampleTime_ms != lastYawReset_ms)
        {
            yawResetAngle = 0.0f;
        }

        yawResetAngle += wrap_PI(yaw - tempEuler.z);
        lastYawReset_ms = imuSampleTime_ms;

        // calculate an initial quaternion using the new yaw value
        quaternionFromEuler(&initQuat, roll, pitch, yaw);

        // zero the attitude covariances because the corelations will now be invalid
        zeroAttCovOnly();

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        // don't do this if the earth field has already been learned
        if (!magFieldLearned)
        {
            quaternionToRotationMatrix(initQuat, &Tbn);
            if (have_table_earth_field && ekfParam._mag_ef_limit > 0)
            {
                ekfStates.stateStruct.earth_magfield = table_earth_field_ga;
            }
            else
            {
                ekfStates.stateStruct.earth_magfield = multiplyMatrixByVector(Tbn, magDataDelayed.mag);
            }

            // set the NE earth magnetic field states using the published declination
            // and set the corresponding variances and covariances
            alignMagStateDeclination();

            // set the remaining variances and covariances
            zeroRows(P, 18, 21);
            zeroCols(P, 18, 21);
            P[18][18] = sq(ekfParam._magNoise);
            P[19][19] = P[18][18];
            P[20][20] = P[18][18];
            P[21][21] = P[18][18];
        }

        // record the fact we have initialised the magnetic field states
        recordMagReset();

        // clear mag state reset request
        magStateResetRequest = false;
    }
    else
    {
        // this function should not be called if there is no compass data but if is is, return the
        // current attitude
        initQuat = ekfStates.stateStruct.quat;
    }

    // return attitude quaternion
    return initQuat;
}

// zero the attitude covariances, but preserve the variances
void zeroAttCovOnly(void)
{
    float varTemp[3];

    for (uint8_t index = 0; index <= 2; index++)
    {
        varTemp[index] = P[index][index];
    }

    zeroCols(P, 0, 2);
    zeroRows(P, 0, 2);

    for (uint8_t index = 0; index <= 2; index++)
    {
        P[index][index] = varTemp[index];
    }
}

void sendEKFLogMessage(char msg[])
{
    // check if the previous message is different from the current message
    if (!(strcmp(last_ekf_msg, msg)) == 0)
    {
        LOG_INFO(EKF, "%s", msg);
        strcpy(last_ekf_msg, msg);
    }
}

// get lon1-lon2, wrapping at -180e7 to 180e7
static int32_t diff_longitude(int32_t lon1, int32_t lon2)
{
    if ((lon1 & 0x80000000) == (lon2 & 0x80000000))
    {
        // common case of same sign
        return lon1 - lon2;
    }

    int64_t dlon = (int64_t)(lon1) - (int64_t)(lon2);

    if (dlon > 1800000000LL)
    {
        dlon -= 3600000000LL;
    }
    else if (dlon < -1800000000LL)
    {
        dlon += 3600000000LL;
    }

    return (int32_t)(dlon);
}

static float longitude_scale(int32_t lat)
{
    float scale = cosf(lat * (1.0e-7 * (M_PIf / 180.0f)));

    return MAX(scale, 0.01f);
}

// limit lattitude to -90e7 to 90e7
static int32_t limit_lattitude(int32_t lat)
{
    if (lat > 900000000L)
    {
        lat = 1800000000LL - lat;
    }
    else if (lat < -900000000L)
    {
        lat = -(1800000000LL + lat);
    }

    return lat;
}

// wrap longitude for -180e7 to 180e7
static int32_t wrap_longitude(int64_t lon)
{
    if (lon > 1800000000L)
    {
        lon = (int32_t)(lon - 3600000000LL);
    }
    else if (lon < -1800000000L)
    {
        lon = (int32_t)(lon + 3600000000LL);
    }

    return (int32_t)(lon);
}

// return the distance in meters in North/East plane as a N/E vector to loc
fpVector2_t get_distance_NE(gpsLocation_t EKF_origin, gpsLocation_t loc)
{
    fpVector2_t vRet = {.v = {(loc.lat - EKF_origin.lat) * 0.011131884502145034f,
                              diff_longitude(loc.lon, EKF_origin.lon) * 0.011131884502145034f * longitude_scale((EKF_origin.lat + loc.lat) / 2)}};

    return vRet;
}

// return horizontal distance in meters between two locations
float get_horizontal_distance(gpsLocation_t actualLoc, gpsLocation_t prevLoc)
{
    float dlat = (float)(actualLoc.lat - prevLoc.lat);
    float dlng = ((float)diff_longitude(actualLoc.lon, prevLoc.lon)) * longitude_scale((prevLoc.lat + actualLoc.lat) / 2);

    return calc_length_pythagorean_2D(dlat, dlng) * 0.011131884502145034f;
}

// extrapolate latitude/longitude given distances (in meters) north and east
void offset_latlng(int32_t *lat, int32_t *lng, float ofs_north, float ofs_east)
{
    const int32_t dlat = ofs_north * 89.83204953368922f;
    const int64_t dlng = (ofs_east * 89.83204953368922f) / longitude_scale(*lat + dlat / 2);
    *lat += dlat;
    *lat = limit_lattitude(*lat);
    *lng = wrap_longitude(dlng + *lng);
}