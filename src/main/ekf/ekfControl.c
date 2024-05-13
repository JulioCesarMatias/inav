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
#include "ekf/ekf.h"
#include "ekf/ekfCore.h"
#include "ekf/EKFGSF_yaw.h"

// Control filter mode transitions
void controlFilterModes(void)
{
    // Determine motor arm status
    prevMotorsArmed = motorsArmed;
    motorsArmed = get_armed();

    if (motorsArmed && !prevMotorsArmed)
    {
        // set the time at which we arm to assist with checks
        timeAtArming_ms = imuSampleTime_ms;
    }

    // Detect if we are in flight on or ground
    detectFlight();

    // Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
    // avoid unnecessary operations
    setWindMagStateLearningMode();

    // Check the alignmnent status of the tilt and yaw attitude
    // Used during initial bootstrap alignment of the filter
    checkAttitudeAlignmentStatus();

    // Set the type of inertial navigation aiding used
    setAidingMode();
}

// Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to avoid unnecessary operations
void setWindMagStateLearningMode(void)
{
    // If we are on ground, or in constant position mode, or don't have the right vehicle and sensing to estimate wind, inhibit wind states
    bool setWindInhibit = (!useAirspeed() && !assume_zero_sideslip()) || onGround || (PV_AidingMode == AID_NONE);
    if (!inhibitWindStates && setWindInhibit)
    {
        inhibitWindStates = true;
    }
    else if (inhibitWindStates && !setWindInhibit)
    {
        inhibitWindStates = false;
        // set states and variances
        if (yawAlignComplete && useAirspeed())
        {
            // if we have airspeed and a valid heading, set the wind states to the reciprocal of the vehicle heading
            // which assumes the vehicle has launched into the wind
            fpVector3_t tempEuler;
            quaternionToEuler(ekfStates.stateStruct.quat, &tempEuler.x, &tempEuler.y, &tempEuler.z);
            float windSpeed = sqrtf(sq(ekfStates.stateStruct.velocity.x) + sq(ekfStates.stateStruct.velocity.y)) - tasDataDelayed.tas;
            ekfStates.stateStruct.wind_vel.x = windSpeed * cosf(tempEuler.z);
            ekfStates.stateStruct.wind_vel.y = windSpeed * sinf(tempEuler.z);

            // set the wind sate variances to the measurement uncertainty
            for (uint8_t index = 22; index <= 23; index++)
            {
                P[index][index] = sq(constrainf(ekfParam._easNoise, 0.5f, 5.0f));
            }
        }
        else
        {
            // set the variances using a typical wind speed
            for (uint8_t index = 22; index <= 23; index++)
            {
                P[index][index] = sq(5.0f);
            }
        }
    }

    // determine if the vehicle is manoeuvring
    if (accNavMagHoriz > 0.5f)
    {
        manoeuvring = true;
    }
    else
    {
        manoeuvring = false;
    }

    // Determine if learning of magnetic field states has been requested by the user
    bool magCalRequested = ((ekfParam._magCal == 0) && inFlight) ||                                     // when flying
                           ((ekfParam._magCal == 1) && manoeuvring) ||                                  // when manoeuvring
                           ((ekfParam._magCal == 3) && finalInflightYawInit && finalInflightMagInit) || // when initial in-air yaw and mag field reset is complete
                           (ekfParam._magCal == 4);                                                     // all the time

    // Deny mag calibration request if we aren't using the compass, it has been inhibited by the user,
    // we do not have an absolute position reference or are on the ground (unless explicitly requested by the user)
    bool magCalDenied = !ekf_useCompass() || (ekfParam._magCal == 2) || (onGround && ekfParam._magCal != 4);

    // Inhibit the magnetic field calibration if not requested or denied
    bool setMagInhibit = !magCalRequested || magCalDenied;
    if (!inhibitMagStates && setMagInhibit)
    {
        inhibitMagStates = true;
        // variances will be reset in CovariancePrediction
    }
    else if (inhibitMagStates && !setMagInhibit)
    {
        inhibitMagStates = false;
        if (magFieldLearned)
        {
            // if we have already learned the field states, then retain the learned variances
            P[16][16] = earthMagFieldVar.x;
            P[17][17] = earthMagFieldVar.y;
            P[18][18] = earthMagFieldVar.z;
            P[19][19] = bodyMagFieldVar.x;
            P[20][20] = bodyMagFieldVar.y;
            P[21][21] = bodyMagFieldVar.z;
        }
        else
        {
            // set the variances equal to the observation variances
            for (uint8_t index = 16; index <= 21; index++)
            {
                P[index][index] = sq(ekfParam._magNoise);
            }

            // set the NE earth magnetic field states using the published declination
            // and set the corresponding variances and covariances
            alignMagStateDeclination();
        }
        // request a reset of the yaw and magnetic field states if not done before
        if (!magStateInitComplete || (!finalInflightMagInit && inFlight))
        {
            magYawResetRequest = true;
        }
    }

    // If on ground we clear the flag indicating that the magnetic field in-flight initialisation has been completed
    // because we want it re-done for each takeoff
    if (onGround)
    {
        finalInflightYawInit = false;
        finalInflightMagInit = false;
    }

    // Adjust the indexing limits used to address the covariance, states and other EKF arrays to avoid unnecessary operations
    // if we are not using those states
    if (inhibitMagStates && inhibitWindStates)
    {
        stateIndexLim = 15;
    }
    else if (inhibitWindStates)
    {
        stateIndexLim = 21;
    }
    else
    {
        stateIndexLim = 23;
    }
}

// Set inertial navigation aiding mode
void setAidingMode(void)
{
    // Save the previous status so we can detect when it has changed
    PV_AidingModePrev = PV_AidingMode;

    // Determine if we should change aiding mode
    switch (PV_AidingMode)
    {
    case AID_NONE:
        // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
        // and IMU gyro bias estimates have stabilised
        const bool ekf_gyro_cal_ok = checkGyroCalStatus();
        const bool filterIsStable = tiltAlignComplete && yawAlignComplete && ekf_gyro_cal_ok;
        if (!ekf_gyro_cal_ok)
        {
            if (!ekf_msg_sended[0])
            {
                sendEKFLogMessage("EKF Gyro Calibration error!");
                ekf_msg_sended[0] = true;
                ekf_msg_sended[1] = false;
            }
        }
        else
        {
            if (!ekf_msg_sended[1])
            {
                sendEKFLogMessage("EKF Gyro Calibration successful!");
                ekf_msg_sended[0] = false;
                ekf_msg_sended[1] = true;
            }
        }
        // If GPS usage has been prohiited then we use flow aiding provided optical flow data is present
        // GPS aiding is the preferred option unless excluded by the user
        bool canUseGPS = ((ekfParam._fusionModeGPS) != 3 && readyToUseGPS() && filterIsStable);
        if (canUseGPS)
        {
            PV_AidingMode = AID_ABSOLUTE;
        }
        else if (optFlowDataPresent() && (ekfParam._flowUse == FLOW_USE_NAV) && filterIsStable)
        {
            PV_AidingMode = AID_RELATIVE;
        }
        break;

    case AID_RELATIVE:
        // Check if the optical flow sensor has timed out
        bool flowSensorTimeout = ((imuSampleTime_ms - flowValidMeaTime_ms) > 5000);
        // Check if the fusion has timed out (flow measurements have been rejected for too long)
        bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowFuseTime_ms) > 5000);
        // Enable switch to absolute position mode if GPS is available
        // If GPS is not available and flow fusion has timed out, then fall-back to no-aiding
        if ((ekfParam._fusionModeGPS) != 3 && readyToUseGPS())
        {
            PV_AidingMode = AID_ABSOLUTE;
        }
        else if (flowSensorTimeout || flowFusionTimeout)
        {
            PV_AidingMode = AID_NONE;
        }
        break;

    case AID_ABSOLUTE:
        // Find the minimum time without data required to trigger any check
        uint16_t minTestTime_ms = MIN(ekfInternalParam.tiltDriftTimeMax_ms, MIN(ekfInternalParam.posRetryTimeNoVel_ms, ekfInternalParam.posRetryTimeUseVel_ms));

        // Check if optical flow data is being used
        bool optFlowUsed = (imuSampleTime_ms - prevFlowFuseTime_ms <= minTestTime_ms);

        // Check if airspeed data is being used
        bool airSpdUsed = (imuSampleTime_ms - lastTasPassTime_ms <= minTestTime_ms);

        // Check if GPS is being used
        bool posUsed = (imuSampleTime_ms - lastPosPassTime_ms <= minTestTime_ms);
        bool gpsVelUsed = (imuSampleTime_ms - lastVelPassTime_ms <= minTestTime_ms);

        // Check if attitude drift has been constrained by a measurement source
        bool attAiding = posUsed || gpsVelUsed || optFlowUsed || airSpdUsed;

        // check if velocity drift has been constrained by a measurement source
        bool velAiding = gpsVelUsed || airSpdUsed || optFlowUsed;

        // check if position drift has been constrained by a measurement source
        bool posAiding = posUsed;

        // Check if the loss of attitude aiding has become critical
        bool attAidLossCritical = false;
        if (!attAiding)
        {
            attAidLossCritical = (imuSampleTime_ms - prevFlowFuseTime_ms > ekfInternalParam.tiltDriftTimeMax_ms) &&
                                 (imuSampleTime_ms - lastTasPassTime_ms > ekfInternalParam.tiltDriftTimeMax_ms) &&
                                 (imuSampleTime_ms - lastPosPassTime_ms > ekfInternalParam.tiltDriftTimeMax_ms) &&
                                 (imuSampleTime_ms - lastVelPassTime_ms > ekfInternalParam.tiltDriftTimeMax_ms);
        }

        // Check if the loss of position accuracy has become critical
        bool posAidLossCritical = false;
        if (!posAiding)
        {
            uint16_t maxLossTime_ms;
            if (!velAiding)
            {
                maxLossTime_ms = ekfInternalParam.posRetryTimeNoVel_ms;
            }
            else
            {
                maxLossTime_ms = ekfInternalParam.posRetryTimeUseVel_ms;
            }
            posAidLossCritical = (imuSampleTime_ms - lastPosPassTime_ms > maxLossTime_ms);
        }

        if (attAidLossCritical)
        {
            // if the loss of attitude data is critical, then put the filter into a constant position mode
            PV_AidingMode = AID_NONE;
            posTimeout = true;
            velTimeout = true;
            tasTimeout = true;
            gpsNotAvailable = true;
        }
        else if (posAidLossCritical)
        {
            if ((ekfParam._flowUse == FLOW_USE_NAV) && optFlowDataPresent() && (imuSampleTime_ms - rngValidMeaTime_ms < 500))
            {
                PV_AidingMode = AID_NONE;
            }
            // if the loss of position is critical, declare all sources of position aiding as being timed out
            posTimeout = true;
            velTimeout = true;
            gpsNotAvailable = true;
        }
        break;
    }

    // check to see if we are starting or stopping aiding and set states and modes as required
    if (PV_AidingMode != PV_AidingModePrev)
    {
        // set various usage modes based on the condition when we start aiding. These are then held until aiding is stopped.
        switch (PV_AidingMode)
        {
        case AID_NONE:
            // We have ceased aiding
            sendEKFLogMessage("EKF IMU has stopped aiding");
            // When not aiding, estimate orientation & height fusing synthetic constant position and zero velocity measurement to constrain tilt errors
            posTimeout = true;
            velTimeout = true;
            // Reset the normalised innovation to avoid false failing bad fusion tests
            velTestRatio = 0.0f;
            posTestRatio = 0.0f;
            // store the current position to be used to keep reporting the last known position
            lastKnownPositionNE.x = ekfStates.stateStruct.position.x;
            lastKnownPositionNE.y = ekfStates.stateStruct.position.y;
            // initialise filtered altitude used to provide a takeoff reference to current baro on disarm
            // this reduces the time required for the baro noise filter to settle before the filtered baro data can be used
            meaHgtAtTakeOff = baroDataDelayed.hgt;
            // reset the vertical position state to faster recover from baro errors experienced during touchdown
            ekfStates.stateStruct.position.z = -meaHgtAtTakeOff;
            break;

        case AID_RELATIVE:
            // We have commenced aiding, but GPS usage has been prohibited so use optical flow only
            sendEKFLogMessage("EKF IMU is using optical flow");
            posTimeout = true;
            velTimeout = true;
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
            break;

        case AID_ABSOLUTE:
            bool canUseGPS = ((ekfParam._fusionModeGPS) != 3 && readyToUseGPS());
            // We have commenced aiding and GPS usage is allowed
            if (canUseGPS)
            {
                sendEKFLogMessage("EKF IMU is using GPS");
            }
            posTimeout = false;
            velTimeout = false;
            // reset the last fusion accepted times to prevent unwanted activation of timeout logic
            lastPosPassTime_ms = imuSampleTime_ms;
            lastVelPassTime_ms = imuSampleTime_ms;
            break;
        }

        // Always reset the position and velocity when changing mode
        ResetVelocity();
        ResetPosition();
    }
}

// Check the tilt and yaw alignmnent status
// Used during initial bootstrap alignment of the filter
void checkAttitudeAlignmentStatus(void)
{
    // Check for tilt convergence - used during initial alignment
    float alpha = 1.0f * imuDataDelayed.delAngDT;
    float temp = calc_length_pythagorean_3D(tiltErrVec.x, tiltErrVec.y, tiltErrVec.z);
    tiltErrFilt = alpha * temp + (1.0f - alpha) * tiltErrFilt;

    if (!tiltAlignComplete)
    {
        if (tiltErrFilt < TILT_ERROR_LIMIT)
        {
            tiltAlignComplete = true;
            sendEKFLogMessage("EKF IMU tilt alignment complete!");
        }
        else
        {
            sendEKFLogMessage("EKF IMU tilt alignment fail!");
        }
    }

    // submit yaw and magnetic field reset requests depending on whether we have compass data
    if (tiltAlignComplete && !yawAlignComplete)
    {
        if (ekf_useCompass())
        {
            magYawResetRequest = true;
            gpsYawResetRequest = false;
        }
        else
        {
            magYawResetRequest = false;
            gpsYawResetRequest = true;
        }
    }
}

// return true if we should use the airspeed sensor
bool useAirspeed(void)
{
    return sensors(SENSOR_PITOT) && pitot.calibrationFinished;
}

// return true if optical flow data is available
bool optFlowDataPresent(void)
{
    return (imuSampleTime_ms - flowMeaTime_ms < 200);
}

// return true if the filter to be ready to use gps
bool readyToUseGPS(void)
{
    return validOrigin && tiltAlignComplete && yawAlignComplete && gpsGoodToAlign && (ekfParam._fusionModeGPS != 3) && gpsDataToFuse;
}

// return true if we should use the compass
bool ekf_useCompass(void)
{
    return sensors(SENSOR_MAG) && compassIsCalibrationComplete() && !magSensorFailed;
}

// should we assume zero sideslip?
bool assume_zero_sideslip(void)
{
    return STATE(FIXED_WING_LEGACY);
}

// sets the local NED origin using a LLH location (latitude, longitude, height)
// returns false if absolute aiding and GPS is being used or if the origin is already set
bool controlSetOriginLLH(gpsLocation_t *loc)
{
    if (PV_AidingMode == AID_ABSOLUTE)
    {
        // reject attempts to set the origin if GPS is being used
        return false;
    }

    return setOrigin(loc);
}

// sets the local NED origin using a LLH location (latitude, longitude, height)
// returns false if the origin has already been set
bool setOrigin(gpsLocation_t *loc)
{
    // if the origin is valid reject setting a new origin
    if (validOrigin)
    {
        return false;
    }

    EKF_origin = *loc;
    ekfGpsRefHgt = 0.01f * EKF_origin.alt;
    // define Earth rotation vector in the NED navigation frame at the origin
    calcEarthRateNED(&earthRateNED, EKF_origin.lat);
    validOrigin = true;
    sendEKFLogMessage("EKF IMU origin set");

    // put origin in frontend as well to ensure it stays in sync between lanes
    common_EKF_origin = EKF_origin;
    ekf_common_origin_valid = true;

    return true;
}

// record a yaw reset event
void recordYawReset(void)
{
    yawAlignComplete = true;
    
    if (inFlight)
    {
        finalInflightYawInit = true;
    }
}
#include "build/debug.h"
// return true and set the class variable true if the delta angle bias has been learned
bool checkGyroCalStatus(void)
{
    // check delta angle bias variances
    const float delAngBiasVarMax = sq(DEGREES_TO_RADIANS(0.15f * dtEkfAvg));

    if (!ekf_useCompass())
    {
        // rotate the variances into earth frame and evaluate horizontal terms only as yaw component is poorly observable without a compass
        // which can make this check fail
        fpVector3_t delAngBiasVarVec = {.v = {P[9][9], P[10][10], P[11][11]}};
        fpVector3_t temp = multiplyMatrixByVector(prevTnb, delAngBiasVarVec);
        delAngBiasLearned = (fabsf(temp.x) < delAngBiasVarMax) &&
                            (fabsf(temp.y) < delAngBiasVarMax);

        debug[4] = delAngBiasVarMax * 1e10;
        debug[5] = temp.x * 1e10;
        debug[6] = temp.y * 1e10;
    }
    else
    {
        delAngBiasLearned = (P[9][9] <= delAngBiasVarMax) &&
                            (P[10][10] <= delAngBiasVarMax) &&
                            (P[11][11] <= delAngBiasVarMax);
    }

    return delAngBiasLearned;
}

// Update the filter status
void updateFilterStatus(void)
{
    // init return value
    filterStatus.value = 0;
    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool someVertRefData = (!velTimeout && useGpsVertVel) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && delAngBiasLearned;
    bool gpsNavPossible = !gpsNotAvailable && gpsGoodToAlign && delAngBiasLearned;
    bool filterHealthy = coreHealthy() && tiltAlignComplete && (yawAlignComplete || (!ekf_useCompass() && (PV_AidingMode == AID_NONE)));
    // If GPS height usage is specified, height is considered to be inaccurate until the GPS passes all checks
    bool hgtNotAccurate = (ekfParam._altSource == HGT_SOURCE_GPS) && !validOrigin;

    // set individual flags
    filterStatus.flags.attitude = !(isnan(ekfStates.stateStruct.quat.q0) ||
                                    isnan(ekfStates.stateStruct.quat.q1) ||
                                    isnan(ekfStates.stateStruct.quat.q2) ||
                                    isnan(ekfStates.stateStruct.quat.q3)) &&
                                  filterHealthy;                                                                                           // attitude valid (we need a better check)
    filterStatus.flags.horiz_vel = someHorizRefData && filterHealthy;                                                                      // horizontal velocity estimate valid
    filterStatus.flags.vert_vel = someVertRefData && filterHealthy;                                                                        // vertical velocity estimate valid
    filterStatus.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav) && filterHealthy;        // relative horizontal position estimate valid
    filterStatus.flags.horiz_pos_abs = doingNormalGpsNav && filterHealthy;                                                                 // absolute horizontal position estimate valid
    filterStatus.flags.vert_pos = !hgtTimeout && filterHealthy && !hgtNotAccurate;                                                         // vertical position estimate valid
    filterStatus.flags.terrain_alt = gndOffsetValid && filterHealthy;                                                                      // terrain height estimate valid
    filterStatus.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;                                                      // constant position mode
    filterStatus.flags.pred_horiz_pos_rel = ((optFlowNavPossible || gpsNavPossible) && filterHealthy) || filterStatus.flags.horiz_pos_rel; // we should be able to estimate a relative position when we enter flight mode
    filterStatus.flags.pred_horiz_pos_abs = (gpsNavPossible && filterHealthy) || filterStatus.flags.horiz_pos_abs;                         // we should be able to estimate an absolute position when we enter flight mode
    filterStatus.flags.takeoff_detected = takeOffDetected;                                                                                 // takeoff for optical flow navigation has been detected
    filterStatus.flags.takeoff = get_takeoff_expected();                                                                                   // The EKF has been told to expect takeoff and is in a ground effect mitigation mode
    filterStatus.flags.touchdown = get_touchdown_expected();                                                                               // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    filterStatus.flags.using_gps = ((imuSampleTime_ms - lastPosPassTime_ms) < 4000) && (PV_AidingMode == AID_ABSOLUTE);
    filterStatus.flags.gps_glitching = !gpsAccuracyGood && (PV_AidingMode == AID_ABSOLUTE); // GPS glitching is affecting navigation accuracy
    filterStatus.flags.gps_quality_good = gpsGoodToAlign;
    // for reporting purposes we report rejecting airspeed after 3s of not fusing when we want to fuse the data
    filterStatus.flags.rejecting_airspeed = lastTasFailTime_ms != 0 &&
                                            (imuSampleTime_ms - lastTasFailTime_ms) < 1000 &&
                                            (imuSampleTime_ms - lastTasPassTime_ms) > 3000;
    filterStatus.flags.initalized = filterStatus.flags.initalized || coreHealthy();
}

void runYawEstimatorPrediction(void)
{
    if (ekfParam._fusionModeGPS <= 1)
    {
        float trueAirspeed;
        if (defaultAirSpeed >= 0 && assume_zero_sideslip())
        {
            if (imuDataDelayed.time_ms - tasDataDelayed.obs.time_ms < 5000)
            {
                trueAirspeed = tasDataDelayed.tas;
            }
            else
            {
                trueAirspeed = defaultAirSpeed;
            }
        }
        else
        {
            trueAirspeed = 0.0f;
        }

        EKFGSF_yaw_update(imuDataDelayed.delAng, imuDataDelayed.delVel, imuDataDelayed.delAngDT, imuDataDelayed.delVelDT, EKFGSF_run_filterbank, trueAirspeed);
    }
}

void runYawEstimatorCorrection(void)
{
    if (ekfParam._fusionModeGPS <= 1 && EKFGSF_run_filterbank)
    {
        if (gpsDataToFuse)
        {
            fpVector2_t gpsVelNE = {.v = {gpsDataDelayed.vel.x, gpsDataDelayed.vel.y}};
            float gpsVelAcc = fmaxf(gpsSpdAccuracy, ekfParam._gpsHorizVelNoise);
            EKFGSF_yaw_fuseVelData(gpsVelNE, gpsVelAcc);
        }

        // action an external reset request
        if (EKFGSF_yaw_reset_request_ms > 0 && imuSampleTime_ms - EKFGSF_yaw_reset_request_ms < YAW_RESET_TO_GSF_TIMEOUT_MS)
        {
            EKFGSF_resetMainFilterYaw();
        }
    }
}

// request a reset the yaw to the GSF estimate
// request times out after YAW_RESET_TO_GSF_TIMEOUT_MS if it cannot be actioned
void EKFGSF_requestYawReset(void)
{
    EKFGSF_yaw_reset_request_ms = imuSampleTime_ms;
}
