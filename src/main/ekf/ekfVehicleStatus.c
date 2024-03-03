#include "ekf/ekfCore.h"
#include "ekf/EKFGSF_yaw.h"
#include "fc/fc_core.h"

/* Monitor GPS data to see if quality is good enough to initialise the EKF
   Monitor magnetometer innovations to see if the heading is good enough to use GPS
   Return true if all criteria pass for 10 seconds

   This sets gpsGoodToAlign class variable
*/
void calcGpsGoodToAlign(void)
{
    if (inFlight && assume_zero_sideslip() && !ekf_useCompass())
    {
        // this is a special case where a plane has launched without magnetometer
        // is now in the air and needs to align yaw to the GPS and start navigating as soon as possible
        gpsGoodToAlign = true;
        return;
    }

    // User defined multiplier to be applied to check thresholds
    float checkScaler = 0.01f * (float)ekfParam._gpsCheckScaler;

    if (gpsGoodToAlign)
    {
        /*
          if we have already passed GPS alignment checks then raise
          the check threshold so that we have some hysterisis and
          don't continuously change from able to arm to not able to
          arm
         */
        checkScaler *= 1.3f;
    }

    // If we have good magnetometer consistency and bad innovations for longer than 5 seconds then we reset heading and field states
    // This enables us to handle large changes to the external magnetic field environment that occur before arming
    if ((magTestRatio.x <= 1.0f && magTestRatio.y <= 1.0f && yawTestRatio <= 1.0f) || !consistentMagData)
    {
        magYawResetTimer_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - magYawResetTimer_ms > 5000) && !motorsArmed)
    {
        // request reset of heading and magnetic field states
        magYawResetRequest = true;
        // reset timer to ensure that bad magnetometer data cannot cause a heading reset more often than every 5 seconds
        magYawResetTimer_ms = imuSampleTime_ms;
    }

    // Check for significant change in GPS position if disarmed which indicates bad GPS
    // This check can only be used when the vehicle is stationary
    gpsLocation_t gpsloc = {.lat = gpsSol.llh.lat, .lon = gpsSol.llh.lon, .alt = gpsSol.llh.alt}; // Current location
    const float posFiltTimeConst = 10.0f;                                                         // time constant used to decay position drift
    // calculate time lapsed since last update and limit to prevent numerical errors
    float deltaTime = constrainf((float)(imuDataDelayed.time_ms - lastPreAlignGpsCheckTime_ms) * 0.001f, 0.01f, posFiltTimeConst);
    lastPreAlignGpsCheckTime_ms = imuDataDelayed.time_ms;
    // Sum distance moved
    gpsDriftNE += get_horizontal_distance(gpsloc, gpsloc_prev);
    gpsloc_prev = gpsloc;
    // Decay distance moved exponentially to zero
    gpsDriftNE *= (1.0f - deltaTime / posFiltTimeConst);
    // Clamp the filter state to prevent excessive persistence of large transients
    gpsDriftNE = MIN(gpsDriftNE, 10.0f);
    // Fail if more than 3 metres drift after filtering whilst on-ground
    // This corresponds to a maximum acceptable average drift rate of 0.3 m/s or single glitch event of 3m
    bool gpsDriftFail = (gpsDriftNE > 3.0f * checkScaler) && onGround && (ekfParam._gpsCheck & MASK_GPS_POS_DRIFT);

    // Report check result as a text string and bitmask
    if (gpsDriftFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS drift");
    }

    // Check that the vertical GPS vertical velocity is reasonable after noise filtering
    bool gpsVertVelFail;
    if (gpsSol.flags.validVelD && onGround)
    {
        // check that the average vertical GPS velocity is close to zero
        gpsVertVelFilt = 0.1f * gpsDataNew.vel.z + 0.9f * gpsVertVelFilt;
        gpsVertVelFilt = constrainf(gpsVertVelFilt, -10.0f, 10.0f);
        gpsVertVelFail = (fabsf(gpsVertVelFilt) > 0.3f * checkScaler) && (ekfParam._gpsCheck & MASK_GPS_VERT_SPD);
    }
    else
    {
        gpsVertVelFail = false;
    }

    // Report check result as a text string and bitmask
    if (gpsVertVelFail)
    {

        strcpy(osd_ekf_status_string, "EKF GPS vertical speed fail");
    }

    // Check that the horizontal GPS vertical velocity is reasonable after noise filtering
    // This check can only be used if the vehicle is stationary
    bool gpsHorizVelFail;
    if (onGround)
    {
        gpsHorizVelFilt = 0.1f * calc_length_pythagorean_2D(gpsDataDelayed.vel.x, gpsDataDelayed.vel.y) + 0.9f * gpsHorizVelFilt;
        gpsHorizVelFilt = constrainf(gpsHorizVelFilt, -10.0f, 10.0f);
        gpsHorizVelFail = (fabsf(gpsHorizVelFilt) > 0.3f * checkScaler) && (ekfParam._gpsCheck & MASK_GPS_HORIZ_SPD);
    }
    else
    {
        gpsHorizVelFail = false;
    }

    // Report check result as a text string and bitmask
    if (gpsHorizVelFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS horizontal speed fail");
    }

    // fail if horiziontal position accuracy not sufficient
    float hAcc = gpsSolDRV.eph / 10;
    bool hAccFail;
    if (gpsSol.flags.validVelNE)
    {
        hAccFail = (hAcc > 5.0f * checkScaler) && (ekfParam._gpsCheck & MASK_GPS_POS_ERR);
    }
    else
    {
        hAccFail = false;
    }

    // Report check result as a text string and bitmask
    if (hAccFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS horizontal acc error");
    }

    // Check for vertical GPS accuracy
    float vAcc = gpsSolDRV.epv / 10;
    bool vAccFail = false;
    if (gpsSol.flags.validVelD)
    {
        vAccFail = (vAcc > 7.5f * checkScaler) && (ekfParam._gpsCheck & MASK_GPS_POS_ERR);
    }
    // Report check result as a text string and bitmask
    if (vAccFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS vertical acc error");
    }

    // fail if reported speed accuracy greater than threshold
    bool gpsSpdAccFail = (gpsSpdAccuracy > 1.0f * checkScaler) && (ekfParam._gpsCheck & MASK_GPS_SPD_ERR);

    // Report check result as a text string and bitmask
    if (gpsSpdAccFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS speed error");
    }

    // fail if satellite geometry is poor
    bool hdopFail = (gpsSol.hdop > 250) && (ekfParam._gpsCheck & MASK_GPS_HDOP);

    // Report check result as a text string and bitmask
    if (hdopFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS HDOP error");
    }

    // fail if not enough sats
    bool numSatsFail = (gpsSol.numSat < 6) && (ekfParam._gpsCheck & MASK_GPS_NSATS);

    // Report check result as a text string and bitmask
    if (numSatsFail)
    {
        strcpy(osd_ekf_status_string, "EKF GPS num sats error");
    }

    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    bool yawFail;
    if ((magTestRatio.x > 1.0f || magTestRatio.y > 1.0f || yawTestRatio > 1.0f) && (ekfParam._gpsCheck & MASK_GPS_YAW_ERR))
    {
        yawFail = true;
    }
    else
    {
        yawFail = false;
    }

    // Report check result as a text string and bitmask
    if (yawFail)
    {
        strcpy(osd_ekf_status_string, "EKF mag yaw error");
    }

    // assume failed first time through and notify user checks have started
    if (lastGpsVelFail_ms == 0)
    {
        strcpy(osd_ekf_status_string, "EKF starting GPS checks");
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // record time of fail or pass
    if (gpsSpdAccFail || numSatsFail || hdopFail || hAccFail || vAccFail || yawFail || gpsDriftFail || gpsVertVelFail || gpsHorizVelFail)
    {
        lastGpsVelFail_ms = imuSampleTime_ms;
    }
    else
    {
        lastGpsVelPass_ms = imuSampleTime_ms;
    }

    // continuous period of 10s without fail required to set healthy
    // continuous period of 5s without pass required to set unhealthy
    if (!gpsGoodToAlign && imuSampleTime_ms - lastGpsVelFail_ms > 10000)
    {
        gpsGoodToAlign = true;
    }
    else if (gpsGoodToAlign && imuSampleTime_ms - lastGpsVelPass_ms > 5000)
    {
        gpsGoodToAlign = false;
    }
}

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
void calcGpsGoodForFlight(void)
{
    // use a simple criteria based on the GPS receivers claimed speed accuracy and the EKF innovation consistency checks

    // set up varaibles and constants used by filter that is applied to GPS speed accuracy
    const float alpha1 = 0.2f; // coefficient for first stage LPF applied to raw speed accuracy data
    const float tau = 10.0f;   // time constant (sec) of peak hold decay
    if (lastGpsCheckTime_ms == 0)
    {
        lastGpsCheckTime_ms = imuSampleTime_ms;
    }
    float dtLPF = (imuSampleTime_ms - lastGpsCheckTime_ms) * 1e-3f;
    lastGpsCheckTime_ms = imuSampleTime_ms;
    float alpha2 = constrainf(dtLPF / tau, 0.0f, 1.0f);

    // get the receivers reported speed accuracy
    float gpsSpdAccRaw = gpsSol.speed_accuracy * 0.01f;
    if (!gpsSol.speed_accuracy)
    {
        gpsSpdAccRaw = 0.0f;
    }

    // filter the raw speed accuracy using a LPF
    sAccFilterState1 = constrainf((alpha1 * gpsSpdAccRaw + (1.0f - alpha1) * sAccFilterState1), 0.0f, 10.0f);

    // apply a peak hold filter to the LPF output
    sAccFilterState2 = MAX(sAccFilterState1, ((1.0f - alpha2) * sAccFilterState2));

    // Apply a threshold test with hysteresis to the filtered GPS speed accuracy data
    if (sAccFilterState2 > 1.5f)
    {
        gpsSpdAccPass = false;
    }
    else if (sAccFilterState2 < 1.0f)
    {
        gpsSpdAccPass = true;
    }

    // Apply a threshold test with hysteresis to the normalised position and velocity innovations
    // Require a fail for one second and a pass for 10 seconds to transition
    if (lastInnovFailTime_ms == 0)
    {
        lastInnovFailTime_ms = imuSampleTime_ms;
        lastInnovPassTime_ms = imuSampleTime_ms;
    }
    if (velTestRatio < 1.0f && posTestRatio < 1.0f)
    {
        lastInnovPassTime_ms = imuSampleTime_ms;
    }
    else if (velTestRatio > 0.7f || posTestRatio > 0.7f)
    {
        lastInnovFailTime_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - lastInnovPassTime_ms) > 1000)
    {
        ekfInnovationsPass = false;
    }
    else if ((imuSampleTime_ms - lastInnovFailTime_ms) > 10000)
    {
        ekfInnovationsPass = true;
    }

    // both GPS speed accuracy and EKF innovations must pass
    gpsAccuracyGood = gpsSpdAccPass && ekfInnovationsPass;
}

// Detect if we are in flight or on ground
void detectFlight(void)
{
    /*
        If we are a fly forward type vehicle (eg plane), then in-air status can be determined through a combination of speed and height criteria.
        Because of the differing certainty requirements of algorithms that need the in-flight / on-ground status we use two booleans where
        onGround indicates a high certainty we are not flying and inFlight indicates a high certainty we are flying. It is possible for
        both onGround and inFlight to be false if the status is uncertain, but they cannot both be true.

        If we are a plane as indicated by the assume_zero_sideslip() status, then different logic is used

        TODO - this logic should be moved out of the EKF and into the flight vehicle code.
    */

    if (assume_zero_sideslip())
    {
        // To be confident we are in the air we use a criteria which combines arm status, ground speed, airspeed and height change
        float gndSpdSq = sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 10 m/s airspeed
        if (useAirspeed())
        {
            if (CENTIMETERS_TO_METERS(getAirspeedEstimate()) > 10.0f)
            {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f && gpsSpdAccuracy < 1.0f)
        {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(hgtMea) > 10.0f)
        {
            largeHgtChange = true;
        }

        // Determine to a high certainty we are flying
        if (motorsArmed && highGndSpd && (highAirSpd || largeHgtChange))
        {
            onGround = false;
            inFlight = true;
        }

        // if is possible we are in flight, set the time this condition was last detected
        if (motorsArmed && (highGndSpd || highAirSpd || largeHgtChange))
        {
            airborneDetectTime_ms = imuSampleTime_ms;
            onGround = false;
        }

        // Determine to a high certainty we are not flying
        // after 5 seconds of not detecting a possible flight condition or we are disarmed, we transition to on-ground mode
        if (!motorsArmed || ((imuSampleTime_ms - airborneDetectTime_ms) > 5000))
        {
            onGround = true;
            inFlight = false;
        }
    }
    else
    {
        // Non fly forward vehicle, so can only use height and motor arm status

        // If the motors are armed then we could be flying and if they are not armed then we are definitely not flying
        if (motorsArmed)
        {
            onGround = false;
        }
        else
        {
            inFlight = false;
            onGround = true;
        }

        if (!onGround)
        {
            // If height has increased since exiting on-ground, then we definitely are flying
            if ((ekfStates.stateStruct.position.z - posDownAtTakeoff) < -1.5f)
            {
                inFlight = true;
            }

            // If rangefinder has increased since exiting on-ground, then we definitely are flying
            if ((rangeDataNew.rng - rngAtStartOfFlight) > 0.5f)
            {
                inFlight = true;
            }

            // If more than 5 seconds since likely_flying was set true, then set inFlight true
            if (getFlightTime() > 5)
            {
                inFlight = true;
            }
        }
    }

    // handle reset of counters used to control how many times we will try to reset the yaw to the EKF-GSF value per flight
    if ((!prevOnGround && onGround) || !gpsSpdAccPass)
    {
        // disable filter bank
        EKFGSF_run_filterbank = false;
    }
    else if (!EKFGSF_run_filterbank && inFlight && gpsSpdAccPass)
    {
        // flying so reset counters and enable filter bank when GPS is good
        EKFGSF_yaw_reset_ms = 0;
        EKFGSF_yaw_reset_request_ms = 0;
        EKFGSF_yaw_reset_count = 0;
        EKFGSF_run_filterbank = true;
        fpVector3_t gyroBias;
        coreGetGyroBias(&gyroBias);
        EKFGSF_yaw_setGyroBias(gyroBias);
    }

    // store current on-ground  and in-air status for next time
    prevOnGround = onGround;
    prevInFlight = inFlight;

    // Store vehicle height and range prior to takeoff for use in post takeoff checks
    if (onGround)
    {
        // store vertical position at start of flight to use as a reference for ground relative checks
        posDownAtTakeoff = ekfStates.stateStruct.position.z;
        // store the range finder measurement which will be used as a reference to detect when we have taken off
        rngAtStartOfFlight = rangeDataNew.rng;
        // if the magnetic field states have been set, then continue to update the vertical position
        // quaternion and yaw innovation snapshots to use as a reference when we start to fly.
        if (magStateInitComplete)
        {
            posDownAtLastMagReset = ekfStates.stateStruct.position.z;
            quatAtLastMagReset = ekfStates.stateStruct.quat;
            yawInnovAtLastMagReset = innovYaw;
        }
    }
}

// Detect takeoff for optical flow navigation
void detectOptFlowTakeoff(void)
{
    if (!onGround && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000)
    {
        // we are no longer confidently on the ground so check the range finder and gyro for signs of takeoff
        fpVector3_t angRateVec;
        fpVector3_t gyroBias;
        coreGetGyroBias(&gyroBias);
        gyroGetMeasuredRotationRate(&angRateVec); // Calculate gyro rate in body frame in rad/s
        angRateVec.x -= gyroBias.x;
        angRateVec.y -= gyroBias.y;
        angRateVec.z -= gyroBias.z;

        takeOffDetected = (takeOffDetected || (calc_length_pythagorean_3D(angRateVec.x, angRateVec.y, angRateVec.z) > 0.1f) || (rangeDataNew.rng > (rngAtStartOfFlight + 0.1f)));
    }
    else if (onGround)
    {
        // we are confidently on the ground so set the takeoff detected status to false
        takeOffDetected = false;
    }
}
