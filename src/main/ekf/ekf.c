#include "ekf/ekf.h"
#include "ekf/ekfCore.h"
#include "ekf/ekfBuffer.h"
#include "drivers/time.h"
#include "fc/config.h"
#include "build/debug.h"

// copter defaults
#define VELNE_M_NSE_DEFAULT 0.3f
#define VELD_M_NSE_DEFAULT 0.5f
#define POSNE_M_NSE_DEFAULT 1.0f
#define ALT_M_NSE_DEFAULT 3.0f
#define MAG_M_NSE_DEFAULT 0.05f
#define GYRO_P_NSE_DEFAULT 3.0E-02f
#define ACC_P_NSE_DEFAULT 6.0E-01f
#define GBIAS_P_NSE_DEFAULT 1.0E-04f
#define GSCALE_P_NSE_DEFAULT 5.0E-04f
#define ABIAS_P_NSE_DEFAULT 5.0E-03f
#define MAGB_P_NSE_DEFAULT 1.0E-04f
#define MAGE_P_NSE_DEFAULT 1.0E-03f
#define VEL_I_GATE_DEFAULT 500
#define POS_I_GATE_DEFAULT 500
#define HGT_I_GATE_DEFAULT 500
#define MAG_I_GATE_DEFAULT 300
#define GLITCH_RADIUS_DEFAULT 25
#define FLOW_MEAS_DELAY 10
#define FLOW_M_NSE_DEFAULT 0.25f
#define FLOW_I_GATE_DEFAULT 300
#define CHECK_SCALER_DEFAULT 100
#define FLOW_USE_DEFAULT 1
/*
// rover defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.25f
#define FLOW_I_GATE_DEFAULT     300
#define CHECK_SCALER_DEFAULT    100
#define FLOW_USE_DEFAULT        1

// plane defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.15f
#define FLOW_I_GATE_DEFAULT     500
#define CHECK_SCALER_DEFAULT    150
#define FLOW_USE_DEFAULT        2
*/

ekfParam_t ekfParam;

bool ekf_started; // boolean true when EKF have been initialised

// Initialise the filter
bool ekfInitialiseFilter(void)
{
    if (ekfParam._enable == 0)
    {
        // return false;
    }

    // User confgurable params
    ekfParam._gpsHorizVelNoise = VELNE_M_NSE_DEFAULT;       // GPS horizontal velocity measurement noise : m/s
    ekfParam._gpsVertVelNoise = VELD_M_NSE_DEFAULT;         // GPS vertical velocity measurement noise : m/s
    ekfParam._gpsHorizPosNoise = POSNE_M_NSE_DEFAULT;       // GPS horizontal position measurement noise m
    ekfParam._baroAltNoise = ALT_M_NSE_DEFAULT;             // Baro height measurement noise : m
    ekfParam._magNoise = MAG_M_NSE_DEFAULT;                 // magnetometer measurement noise : gauss
    ekfParam._easNoise = 1.4f;                              // equivalent airspeed measurement noise : m/s
    ekfParam._windVelProcessNoise = 0.1f;                   // wind velocity state process noise : m/s^2
    ekfParam._wndVarHgtRateScale = 0.5f;                    // scale factor applied to wind process noise due to height rate
    ekfParam._magEarthProcessNoise = MAGE_P_NSE_DEFAULT;    // Earth magnetic field process noise : gauss/sec
    ekfParam._magBodyProcessNoise = MAGB_P_NSE_DEFAULT;     // Body magnetic field process noise : gauss/sec
    ekfParam._gyrNoise = GYRO_P_NSE_DEFAULT;                // gyro process noise : rad/s
    ekfParam._accNoise = ACC_P_NSE_DEFAULT;                 // accelerometer process noise : m/s^2
    ekfParam._gyroBiasProcessNoise = GBIAS_P_NSE_DEFAULT;   // gyro bias state process noise : rad/s
    ekfParam._accelBiasProcessNoise = ABIAS_P_NSE_DEFAULT;  // accel bias state process noise : m/s^2
    ekfParam._hgtDelay_ms = 60;                             // effective average delay of Height measurements relative to inertial measurements (msec)
    ekfParam._fusionModeGPS = 0;                            // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
    ekfParam._gpsVelInnovGate = VEL_I_GATE_DEFAULT;         // Percentage number of standard deviations applied to GPS velocity innovation consistency check
    ekfParam._gpsPosInnovGate = POS_I_GATE_DEFAULT;         // Percentage number of standard deviations applied to GPS position innovation consistency check
    ekfParam._hgtInnovGate = HGT_I_GATE_DEFAULT;            // Percentage number of standard deviations applied to height innovation consistency check
    ekfParam._magInnovGate = MAG_I_GATE_DEFAULT;            // Percentage number of standard deviations applied to magnetometer innovation consistency check
    ekfParam._tasInnovGate = 400;                           // Percentage number of standard deviations applied to true airspeed innovation consistency check
    ekfParam._magCal = 2;                                   // Sets activation condition for in-flight magnetometer calibration 0:When flying,1:When manoeuvring,2:Never,3:After first climb yaw reset,4:Always
    ekfParam._gpsGlitchRadiusMax = GLITCH_RADIUS_DEFAULT;   // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    ekfParam._flowNoise = FLOW_M_NSE_DEFAULT;               // optical flow rate measurement noise
    ekfParam._flowInnovGate = FLOW_I_GATE_DEFAULT;          // Percentage number of standard deviations applied to optical flow innovation consistency check
    ekfParam._flowDelay_ms = FLOW_MEAS_DELAY;               // effective average delay of optical flow measurements rel to IMU (msec)
    ekfParam._rngInnovGate = 500;                           // Percentage number of standard deviations applied to range finder innovation consistency check
    ekfParam._maxFlowRate = 2.5f;                           // Maximum flow rate magnitude that will be accepted by the filter
    ekfParam._altSource = HGT_SOURCE_BARO;                  // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder, 2 = use GPS
    ekfParam._gyroScaleProcessNoise = GSCALE_P_NSE_DEFAULT; // gyro scale factor state process noise : 1/s
    ekfParam._rngNoise = 0.5f;                              // Range finder noise : m
    ekfParam._gpsCheck = 31;                                // Bitmask controlling which preflight GPS checks are bypasse. Set to 0 to bypass all checks. Set to 255 perform all checks.
    ekfParam._gpsCheckScaler = CHECK_SCALER_DEFAULT;        // Percentage increase to be applied to GPS pre-flight accuracy and drift thresholds
    ekfParam._noaidHorizNoise = 10.0f;                      // horizontal position measurement noise assumed when synthesised zero position measurements are used to constrain attitude drift : m
    ekfParam._yawNoise = 0.5f;                              // magnetic yaw measurement noise : rad
    ekfParam._yawInnovGate = 300;                           // Percentage number of standard deviations applied to magnetic yaw innovation consistency check
    ekfParam._tauVelPosOutput = 25;                         // Time constant of output complementary filter : csec (centi-seconds)
    ekfParam._useRngSwHgt = -1;                             // Maximum valid range of the range finder as a percentage of the maximum range specified by the sensor driver
    ekfParam._terrGradMax = 0.1f;                           // Maximum terrain gradient below the vehicle
    ekfParam._useRngSwSpd = 2.0f;                           // Maximum horizontal ground speed to use range finder as the primary height source (m/s)
    ekfParam._originHgtMode = 0;                            // Bitmask control of EKF reference height correction. 0:Correct when using Baro height, 1:Correct when using range finder height, 2:Apply corrections to local position
    ekfParam._flowUse = FLOW_USE_DEFAULT;                   // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.
    ekfParam._mag_ef_limit = 50;                            // limit on difference between WMM tables and learned earth field.
    ekfParam._hrt_filt_freq = 2.0f;                         // frequency of output observer height rate complementary filter in Hz
    ekfParam._gsfResetMaxCount = 2;                         // maximum number of times the EKF is allowed to reset it's yaw to the EKF-GSF estimate

    // Developer configurable params
    ekfParam.gpsNEVelVarAccScale = 0.05f;                                                                         // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    ekfParam.gpsDVelVarAccScale = 0.07f;                                                                          // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    ekfParam.gpsPosVarAccScale = 0.05f;                                                                           // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    ekfParam.magDelay_ms = 60;                                                                                    // Magnetometer measurement delay (msec)
    ekfParam.tasDelay_ms = 240;                                                                                   // Airspeed measurement delay (msec)
    ekfParam.tiltDriftTimeMax_ms = 15000;                                                                         // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    ekfParam.posRetryTimeUseVel_ms = 10000;                                                                       // Position aiding retry time with velocity measurements (msec)
    ekfParam.posRetryTimeNoVel_ms = 7000;                                                                         // Position aiding retry time without velocity measurements (msec)
    ekfParam.hgtRetryTimeMode0_ms = 10000;                                                                        // Height retry time with vertical velocity measurement (msec)
    ekfParam.hgtRetryTimeMode12_ms = 5000;                                                                        // Height retry time without vertical velocity measurement (msec)
    ekfParam.tasRetryTime_ms = 5000;                                                                              // True airspeed timeout and retry interval (msec)
    ekfParam.magFailTimeLimit_ms = 10000;                                                                         // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    ekfParam.magVarRateScale = 0.005f;                                                                            // scale factor applied to magnetometer variance due to angular rate
    ekfParam.gyroBiasNoiseScaler = 2.0f;                                                                          // scale factor applied to gyro bias state process noise when on ground
    ekfParam.hgtAvg_ms = 100;                                                                                     // average number of msec between height measurements
    ekfParam.betaAvg_ms = 100;                                                                                    // average number of msec between synthetic sideslip measurements
    ekfParam.covTimeStepMax = 0.1f;                                                                               // maximum time (sec) between covariance prediction updates
    ekfParam.covDelAngMax = 0.05f;                                                                                // maximum delta angle between covariance prediction updates
    ekfParam.DCM33FlowMin = 0.71f;                                                                                // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    ekfParam.fScaleFactorPnoise = 1e-10f;                                                                         // Process noise added to focal length scale factor state variance at each time step
    ekfParam.flowTimeDeltaAvg_ms = 100;                                                                           // average interval between optical flow measurements (msec)
    ekfParam.flowIntervalMax_ms = 100;                                                                            // maximum allowable time between flow fusion events
    ekfParam.gndEffectBaroScaler = 4.0f;                                                                          // scaler applied to the barometer observation variance when ground effect mode is active
    ekfParam.maxYawEstVelInnov = 2.0f;                                                                            // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)
    ekfParam._frameTimeUsec = getLooptime();                                                                      // get the value of loop-time
    ekfParam._imuTimeHz = 1000000 / ekfParam._frameTimeUsec;                                                      // convert loop-time in us to Hz
    ekfParam._framesPerPrediction = (uint8_t)(EKF_TARGET_DT / ((float)ekfParam._frameTimeUsec * 1.0e-6f) + 0.5f); // expected number of IMU frames per prediction
    ekfParam.common_origin_valid = false;                                                                         // invalidate shared origin

    if (!setupEKFRingBuffer())
    {
        sendEKFLogMessage("EKF setup failed (buffer allocation)");
        return false;
    }

    // initialise the EKF. We return success only if EKF initialise successfully
    if (!coreInitialiseFilterBootstrap())
    {
        return false;
    }

    sendEKFLogMessage("EKF started");

    return true;
}

// Update Filter States - this should be called whenever new IMU data is available
void ekfUpdateFilter(void)
{
    timeUs_t ekf_time_us = micros();

    if (!ekf_started)
    {
        if (ekfStartTime_us == 0)
        {
            ekfStartTime_us = ekf_time_us;
        }
        if (ekf_time_us - ekfStartTime_us > MS2US(2000))
        {
            ekf_started = ekfInitialiseFilter();
        }
    }

    if (ekf_started)
    {
        TIME_SECTION_BEGIN(0);

        bool statePredictEnabled = true;

        if (getFramesSincePredict() < (ekfParam._framesPerPrediction + 3) &&
            US2S(ekf_time_us - cfTasks[TASK_PID].lastExecutedAt) > (1.0f / (float)ekfParam._imuTimeHz) * 0.33f)
        {
            statePredictEnabled = false;
        }

        coreUpdateFilter(statePredictEnabled, ekf_time_us);

        fpVector3_t eulers;

        getEulerAngles(&eulers);

        int16_t ekf_eulers_z = RADIANS_TO_CENTIDEGREES(eulers.z);

        if (ekf_eulers_z < 0)
        {
            ekf_eulers_z += 36000;
        }

        debug[1] = RADIANS_TO_CENTIDEGREES(eulers.x);
        debug[2] = RADIANS_TO_CENTIDEGREES(eulers.y);
        debug[3] = ekf_eulers_z;

        float ekf_alt;
        coreGetPosD(&ekf_alt);
        debug[4] = RADIANS_TO_DECIDEGREES(outputDataNew.quat.q0); //-ekf_alt * 100; // convert from m in NED to cm in NEU
        debug[5] = RADIANS_TO_DECIDEGREES(outputDataNew.quat.q1);
        debug[6] = RADIANS_TO_DECIDEGREES(outputDataNew.quat.q2);
        debug[7] = RADIANS_TO_DECIDEGREES(outputDataNew.quat.q3);

        TIME_SECTION_END(0);
    }
}

bool get_armed(void)
{
    return false;
}

bool get_takeoff_expected(void)
{
    return false;
}

bool get_touchdown_expected(void)
{
    return false;
}

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool resetHeightDatum(void)
{
    if (activeHgtSource == HGT_SOURCE_RNG || !onGround)
    {
        // only allow resets when on the ground.
        // If using using rangefinder for height then never perform a
        // reset of the height datum
        return false;
    }

    // record the old height estimate
    float oldHgt = -ekfStates.stateStruct.position.z;

    // reset the height state
    ekfStates.stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and after the reset is the same

    if (validOrigin)
    {
        if (!gpsGoodToAlign)
        {
            // if we don't have GPS lock then we shouldn't be doing a
            // resetHeightDatum, but if we do then the best option is
            // to maintain the old error
            EKF_origin.alt += (int32_t)(100.0f * oldHgt);
        }
        else
        {
            // if we have a good GPS lock then reset to the GPS
            // altitude. This ensures the reported AMSL alt from
            // getLLH() is equal to GPS altitude, while also ensuring
            // that the relative alt is zero
            EKF_origin.alt = gpsSol.llh.alt;
        }
        ekfGpsRefHgt = 0.01f * EKF_origin.alt;
    }

    // set the terrain state to zero (on ground). The adjustment for
    // frame height will get added in the later constraints
    terrainState = 0;
    return true;
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference. Use to prevent range finder operation otherwise
// enabled by the combination of EKF_RNG_AID_HGT and EKF_RNG_USE_SPD parameters.
void setTerrainHgtStable(bool val)
{
    terrainHgtStable = val;
}

// set the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool ekfSetOriginLLH(gpsLocation_t *loc)
{
    if (ekfParam._fusionModeGPS != 3 || ekfParam.common_origin_valid)
    {
        return false;
    }

    bool ret = controlSetOriginLLH(loc);

    // return true if accepts the new origin
    return ret;
}

// Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
void writeDefaultAirSpeed(float airspeed)
{
    defaultAirSpeed = airspeed;
}

// check if configured to use GPS for horizontal position estimation
bool configuredToUseGPSForPosXY(void)
{
    // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
    return (ekfParam._fusionModeGPS != 3);
}