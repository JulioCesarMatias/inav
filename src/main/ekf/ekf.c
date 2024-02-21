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
#define MAG_CAL_DEFAULT 3
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
#define MAG_CAL_DEFAULT         2
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
#define MAG_CAL_DEFAULT         0
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
    ekfParam._magCal = MAG_CAL_DEFAULT;                     // Sets activation condition for in-flight magnetometer calibration
    ekfParam._gpsGlitchRadiusMax = GLITCH_RADIUS_DEFAULT;   // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    ekfParam._flowNoise = FLOW_M_NSE_DEFAULT;               // optical flow rate measurement noise
    ekfParam._flowInnovGate = FLOW_I_GATE_DEFAULT;          // Percentage number of standard deviations applied to optical flow innovation consistency check
    ekfParam._flowDelay_ms = FLOW_MEAS_DELAY;               // effective average delay of optical flow measurements rel to IMU (msec)
    ekfParam._rngInnovGate = 500;                           // Percentage number of standard deviations applied to range finder innovation consistency check
    ekfParam._maxFlowRate = 2.5f;                           // Maximum flow rate magnitude that will be accepted by the filter
    ekfParam._altSource = 0;                                // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder, 2 = use GPS
    ekfParam._gyroScaleProcessNoise = GSCALE_P_NSE_DEFAULT; // gyro scale factor state process noise : 1/s
    ekfParam._rngNoise = 0.5f;                              // Range finder noise : m
    ekfParam._gpsCheck = 31;                                // Bitmask controlling which preflight GPS checks are bypassed
    ekfParam._gpsCheckScaler = CHECK_SCALER_DEFAULT;        // Percentage increase to be applied to GPS pre-flight accuracy and drift thresholds
    ekfParam._noaidHorizNoise = 10.0f;                      // horizontal position measurement noise assumed when synthesised zero position measurements are used to constrain attitude drift : m
    ekfParam._yawNoise = 0.5f;                              // magnetic yaw measurement noise : rad
    ekfParam._yawInnovGate = 300;                           // Percentage number of standard deviations applied to magnetic yaw innovation consistency check
    ekfParam._tauVelPosOutput = 25;                         // Time constant of output complementary filter : csec (centi-seconds)
    ekfParam._useRngSwHgt = -1;                             // Maximum valid range of the range finder as a percentage of the maximum range specified by the sensor driver
    ekfParam._terrGradMax = 0.1f;                           // Maximum terrain gradient below the vehicle
    ekfParam._useRngSwSpd = 2.0f;                           // Maximum horizontal ground speed to use range finder as the primary height source (m/s)
    ekfParam._magMask = 0;                                  // Bitmask forcng specific EKF core instances to use simple heading magnetometer fusion.
    ekfParam._originHgtMode = 0;                            // Bitmask controlling post alignment correction and reporting of the EKF origin height.
    ekfParam._flowUse = FLOW_USE_DEFAULT;                   // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.
    ekfParam._mag_ef_limit = 50;                            // limit on difference between WMM tables and learned earth field.
    ekfParam._hrt_filt_freq = 2.0f;                         // frequency of output observer height rate complementary filter in Hz
    ekfParam._gsfRunMask = 3;                               // mask controlling which EKF instances run a separate EKF-GSF yaw estimator
    ekfParam._gsfUseMask = 3;                               // mask controlling which EKF instances will use EKF-GSF yaw estimator data to assit with yaw resets
    ekfParam._gsfResetMaxCount = 2;                         // maximum number of times the EKF is allowed to reset it's yaw to the EKF-GSF estimate

    // Developer configurable params
    ekfParam.gpsNEVelVarAccScale = 0.05f;   // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    ekfParam.gpsDVelVarAccScale = 0.07f;    // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    ekfParam.gpsPosVarAccScale = 0.05f;     // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    ekfParam.magDelay_ms = 60;              // Magnetometer measurement delay (msec)
    ekfParam.tasDelay_ms = 240;             // Airspeed measurement delay (msec)
    ekfParam.tiltDriftTimeMax_ms = 15000;   // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    ekfParam.posRetryTimeUseVel_ms = 10000; // Position aiding retry time with velocity measurements (msec)
    ekfParam.posRetryTimeNoVel_ms = 7000;   // Position aiding retry time without velocity measurements (msec)
    ekfParam.hgtRetryTimeMode0_ms = 10000;  // Height retry time with vertical velocity measurement (msec)
    ekfParam.hgtRetryTimeMode12_ms = 5000;  // Height retry time without vertical velocity measurement (msec)
    ekfParam.tasRetryTime_ms = 5000;        // True airspeed timeout and retry interval (msec)
    ekfParam.magFailTimeLimit_ms = 10000;   // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    ekfParam.magVarRateScale = 0.005f;      // scale factor applied to magnetometer variance due to angular rate
    ekfParam.gyroBiasNoiseScaler = 2.0f;    // scale factor applied to gyro bias state process noise when on ground
    ekfParam.hgtAvg_ms = 100;               // average number of msec between height measurements
    ekfParam.betaAvg_ms = 100;              // average number of msec between synthetic sideslip measurements
    ekfParam.covTimeStepMax = 0.1f;         // maximum time (sec) between covariance prediction updates
    ekfParam.covDelAngMax = 0.05f;          // maximum delta angle between covariance prediction updates
    ekfParam.DCM33FlowMin = 0.71f;          // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    ekfParam.fScaleFactorPnoise = 1e-10f;   // Process noise added to focal length scale factor state variance at each time step
    ekfParam.flowTimeDeltaAvg_ms = 100;     // average interval between optical flow measurements (msec)
    ekfParam.flowIntervalMax_ms = 100;      // maximum allowable time between flow fusion events
    ekfParam.gndEffectBaroScaler = 4.0f;    // scaler applied to the barometer observation variance when ground effect mode is active
    ekfParam.fusionTimeStep_ms = 10;        // The minimum time interval between covariance predictions and measurement fusions in msec
    ekfParam.maxYawEstVelInnov = 2.0f;      // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)
    ekfParam.imuSampleTime_us = micros();
    ekfParam._frameTimeUsec = 1e6 / getLooptime();
    ekfParam._framesPerPrediction = (uint8_t)(EKF_TARGET_DT / (ekfParam._frameTimeUsec * 1.0e-6) + 0.5f); // expected number of IMU frames per prediction

    if (!statesInitialised)
    {
        setup_core();
    }

    // invalidate shared origin
    ekfParam.common_origin_valid = false;

    // initialise the EKF. We return success only if EKF initialise successfully
    bool ret = coreInitialiseFilterBootstrap();

    return ret;
}

// Update Filter States - this should be called whenever new IMU data is available
void ekfUpdateFilter(void)
{
    if (!ekf_started)
    {
        if (ekfStartTime_ms == 0)
        {
            ekfStartTime_ms = millis();
        }
        if (millis() - ekfStartTime_ms > 1000)
        {
            ekf_started = ekfInitialiseFilter();
        }
    }

    if (ekf_started)
    {
        TIME_SECTION_BEGIN(0);

        ekfParam.imuSampleTime_us = micros();

        bool statePredictEnabled;

        if (getFramesSincePredict() < (ekfParam._framesPerPrediction + 3) &&
            (ekfParam.imuSampleTime_us - cfTasks[TASK_PID].lastExecutedAt) > getLooptime() * 0.33f)
        {
            statePredictEnabled = false;
        }
        else
        {
            statePredictEnabled = true;
        }

        coreUpdateFilter(statePredictEnabled);

        fpVector3_t eulers;

        getEulerAngles(&eulers);

        int16_t ekf_eulers_z = RADIANS_TO_DECIDEGREES(eulers.z);

        if (ekf_eulers_z < 0)
        {
            ekf_eulers_z += 3600;
        }

        debug[1] = RADIANS_TO_DECIDEGREES(eulers.x);
        debug[2] = RADIANS_TO_DECIDEGREES(eulers.y);
        debug[3] = ekf_eulers_z;

        float ekf_alt;
        coreGetPosD(&ekf_alt);
        debug[4] = -ekf_alt * 100; // convert from m in NED to cm in NEU

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

    // return true if any core accepts the new origin
    return ret;
}

// write the raw optical flow measurements
// this needs to be called externally.
void writeOptFlowMeas(const uint8_t rawFlowQuality, const fpVector2_t rawFlowRates, const fpVector2_t rawGyroRates, const fpVector3_t posOffset, float heightOverride)
{
    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    // reset the accumulated body delta angle and time
    // don't do the calculation if not enough time lapsed for a reliable body rate measurement
    if (delTimeOF > 0.01f)
    {
        flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrainf((rawGyroRates.x - delAngBodyOF.x / delTimeOF), -0.1f, 0.1f);
        flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrainf((rawGyroRates.y - delAngBodyOF.y / delTimeOF), -0.1f, 0.1f);
        vectorZero(&delAngBodyOF);
        delTimeOF = 0.0f;
    }
    // by definition if this function is called, then flow measurements have been provided so we
    // need to run the optical flow takeoff detection
    detectOptFlowTakeoff();

    // calculate rotation matrices at mid sample time for flow observations
    quaternionToRotationMatrix(ekfStates.stateStruct.quat, &Tbn_flow);

    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && calc_length_pythagorean_2D(rawFlowRates.x, rawFlowRates.y) < 4.2f && calc_length_pythagorean_2D(rawGyroRates.x, rawGyroRates.y) < 4.2f)
    {
        // correct flow sensor body rates for bias and write
        ofDataNew.bodyRadXYZ.x = rawGyroRates.x - flowGyroBias.x;
        ofDataNew.bodyRadXYZ.y = rawGyroRates.y - flowGyroBias.y;
        // the sensor interface doesn't provide a z axis rate so use the rate from the nav sensor instead
        if (delTimeOF > 0.001f)
        {
            // first preference is to use the rate averaged over the same sampling period as the flow sensor
            ofDataNew.bodyRadXYZ.z = delAngBodyOF.z / delTimeOF;
        }
        else if (imuDataNew.delAngDT > 0.001f)
        {
            // second preference is to use most recent IMU data
            ofDataNew.bodyRadXYZ.z = imuDataNew.delAng.z / imuDataNew.delAngDT;
        }
        else
        {
            // third preference is use zero
            ofDataNew.bodyRadXYZ.z = 0.0f;
        }
        // write uncorrected flow rate measurements
        // note correction for different axis and sign conventions used by the px4flow sensor
        ofDataNew.flowRadXY.x = -rawFlowRates.x; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        ofDataNew.flowRadXY.y = -rawFlowRates.y; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        // write the flow sensor position in body frame
        ofDataNew.body_offset = posOffset;
        // write the flow sensor height override
        ofDataNew.heightOverride = heightOverride;
        // write flow rate measurements corrected for body rates
        ofDataNew.flowRadXYcomp.x = ofDataNew.flowRadXY.x + ofDataNew.bodyRadXYZ.x;
        ofDataNew.flowRadXYcomp.y = ofDataNew.flowRadXY.y + ofDataNew.bodyRadXYZ.y;
        // record time last observation was received so we can detect loss of data elsewhere
        flowValidMeaTime_ms = imuSampleTime_ms;
        // estimate sample time of the measurement
        ofDataNew.time_ms = imuSampleTime_ms - ekfParam._flowDelay_ms - ekfParam.flowTimeDeltaAvg_ms / 2;
        // Correct for the average intersampling delay due to the filter updaterate
        ofDataNew.time_ms -= localFilterTimeStep_ms / 2;
        // Prevent time delay exceeding age of oldest IMU data in the buffer
        ofDataNew.time_ms = MAX(ofDataNew.time_ms, imuDataDelayed.time_ms);
        // Save data to buffer
        ekf_ring_buffer_push(&storedOF, &ofDataNew);
        // Check for data at the fusion time horizon
        flowDataToFuse = ekf_ring_buffer_recall(&storedOF, &ofDataDelayed, imuDataDelayed.time_ms);
    }
}

// check if configured to use GPS for horizontal position estimation
bool configuredToUseGPSForPosXY(void)
{
    // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
    return (ekfParam._fusionModeGPS != 3);
}