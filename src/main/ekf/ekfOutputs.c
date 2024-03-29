#include "ekf/ekfCore.h"

// Check basic filter health metrics and return a consolidated health status
bool coreHealthy(void)
{
    uint16_t faultInt;

    getFilterFaults(&faultInt);

    if (faultInt > 0)
    {
        return false;
    }

    if (velTestRatio > 1.0f && posTestRatio > 1.0f && hgtTestRatio > 1.0f)
    {
        // all three metrics being above 1 means the filter is
        // extremely unhealthy.
        return false;
    }

    // position and height innovations must be within limits when on-ground and in a static mode of operation
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (onGround && (PV_AidingMode == AID_NONE) && ((horizErrSq > 1.0f) || (fabsf(hgtInnovFiltState) > 1.0f)))
    {
        return false;
    }

    // all OK
    return true;
}

// Return a consolidated error score where higher numbers represent larger errors
// Intended to be used by the front-end to determine which is the primary EKF
float errorScore(void)
{
    float score = 0.0f;
    
    if (tiltAlignComplete && yawAlignComplete) {
        // Check GPS fusion performance
        score = MAX(score, 0.5f * (velTestRatio + posTestRatio));
        // Check altimeter fusion performance
        score = MAX(score, hgtTestRatio);
        // Check attitude corrections
        const float tiltErrThreshold = 0.05f;
        score = MAX(score, tiltErrFilt / tiltErrThreshold);
    }

    return score;
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool getHeightControlLimit(float *height)
{
    // only ask for limiting if we are doing optical flow only navigation
    if (ekfParam._fusionModeGPS == 3 && (PV_AidingMode == AID_RELATIVE) && flowDataValid)
    {
        // If are doing optical flow nav, ensure the height above ground is within range finder limits after accounting for vehicle tilt and control errors
        *height = MAX(rangeFinderMaxAltitude() * 0.007f - 1.0f, 1.0f);
        // If we are are not using the range finder as the height reference, then compensate for the difference between terrain and EKF origin
        if (ekfParam._altSource != HGT_SOURCE_RNG)
        {
            *height -= terrainState;
        }
        return true;
    }
    else
    {
        return false;
    }
}

// return the Euler roll, pitch and yaw angle in radians
void getEulerAngles(fpVector3_t *euler)
{
    quaternionToEuler(outputDataNew.quat, &euler->x, &euler->y, &euler->z);
}

// return body axis gyro bias estimates in rad/sec
void coreGetGyroBias(fpVector3_t *gyroBias)
{
    if (dtEkfAvg < 1e-6f)
    {
        vectorZero(gyroBias);
        return;
    }
    
    gyroBias->x = ekfStates.stateStruct.gyro_bias.x / dtEkfAvg;
    gyroBias->y = ekfStates.stateStruct.gyro_bias.y / dtEkfAvg;
    gyroBias->z = ekfStates.stateStruct.gyro_bias.z / dtEkfAvg;
}

// return body axis gyro scale factor error as a percentage
void getGyroScaleErrorPercentage(fpVector3_t *gyroScale)
{
    if (!statesInitialised)
    {
        gyroScale->x = gyroScale->y = gyroScale->z = 0.0f;
        return;
    }

    gyroScale->x = 100.0f / ekfStates.stateStruct.gyro_scale.x - 100.0f;
    gyroScale->y = 100.0f / ekfStates.stateStruct.gyro_scale.y - 100.0f;
    gyroScale->z = 100.0f / ekfStates.stateStruct.gyro_scale.z - 100.0f;
}

// return the transformation matrix from XYZ (body) to NED axes
void getRotationBodyToNED(fpMat3_t *mat)
{
    quaternionToRotationMatrix(outputDataNew.quat, mat);
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void getQuaternion(fpQuaternion_t *ret)
{
    *ret = outputDataNew.quat;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t getLastYawResetAngle(float *yawAng)
{
    *yawAng = yawResetAngle;
    return lastYawReset_ms;
}

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t getLastPosNorthEastReset(fpVector2_t *pos)
{
    *pos = posResetNE;
    return lastPosReset_ms;
}

// return the amount of vertical position change due to the last vertical position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t getLastPosDownReset(float *posD)
{
    *posD = posResetD;
    return lastPosResetD_ms;
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t getLastVelNorthEastReset(fpVector2_t *vel)
{
    *vel = velResetNE;
    return lastVelReset_ms;
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void getWind(fpVector3_t *wind)
{
    wind->x = ekfStates.stateStruct.wind_vel.x;
    wind->y = ekfStates.stateStruct.wind_vel.y;
    wind->z = 0.0f; // currently don't estimate this
}

// return the NED velocity of the body frame origin in m/s
void coreGetVelNED(fpVector3_t *vel)
{
    // correct for the IMU position offset (EKF calculations are at the IMU)
    *vel = outputDataNew.velocity;
}

// return estimate of true airspeed vector in body frame in m/s
// returns false if estimate is unavailable
bool coreGetAirSpdVec(fpVector3_t *vel)
{
    if (PV_AidingMode == AID_NONE)
    {
        return false;
    }

    *vel = outputDataNew.velocity;

    if (!inhibitWindStates)
    {
        vel->x -= ekfStates.stateStruct.wind_vel.x;
        vel->y -= ekfStates.stateStruct.wind_vel.y;
    }

    fpMat3_t Tnb; // rotation from nav to body frame
    quaternionToRotationMatrix(quaternion_inverse(outputDataNew.quat), &Tnb);
    *vel = multiplyMatrixByVector(Tnb, *vel);

    return true;
}

// Return the rate of change of vertical position in the down direction (dPosD/dt) of the body frame origin in m/s
float coreGetPosDownDerivative(void)
{
    // return the value calculated from a complementary filter applied to the EKF height and vertical acceleration
    // correct for the IMU offset (EKF calculations are at the IMU)
    return vertCompFiltState.vel;
}

// return the Z-accel bias estimate in m/s^2
void getAccelZBias(float *zbias)
{
    if (dtEkfAvg > 0)
    {
        *zbias = ekfStates.stateStruct.accel_zbias / dtEkfAvg;
    }
    else
    {
        *zbias = 0.0f;
    }
}

// Write the last estimated NE position of the body frame origin relative to the reference point (m).
// Return true if the estimate is valid
bool coreGetPosNE(fpVector2_t *posNE)
{
    // There are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no position estimate available)
    if (PV_AidingMode != AID_NONE)
    {
        // This is the normal mode of operation where we can use the EKF position states
        // correct for the IMU offset (EKF calculations are at the IMU)
        posNE->x = outputDataNew.position.x;
        posNE->y = outputDataNew.position.y;
        return true;
    }
    else
    {
        // In constant position mode the EKF position states are at the origin, so we cannot use them as a position estimate
        if (validOrigin)
        {
            if (gpsSol.fixType >= GPS_FIX_2D)
            {
                // If the origin has been set and we have GPS, then return the GPS position relative to the origin
                gpsLocation_t gpsloc = {.lat = gpsSol.llh.lat, .lon = gpsSol.llh.lon, .alt = gpsSol.llh.alt};
                const fpVector2_t tempPosNE = get_distance_NE(EKF_origin, gpsloc);
                posNE->x = tempPosNE.x;
                posNE->y = tempPosNE.y;
                return false;
            }
            else
            {
                // If no GPS fix is available, all we can do is provide the last known position
                posNE->x = outputDataNew.position.x;
                posNE->y = outputDataNew.position.y;
                return false;
            }
        }
        else
        {
            // If the origin has not been set, then we have no means of providing a relative position
            posNE->x = 0.0f;
            posNE->y = 0.0f;
            return false;
        }
    }
    return false;
}

// Write the last calculated D position of the body frame origin relative to the EKF origin (m).
// Return true if the estimate is valid
bool coreGetPosD(float *posD)
{
    // The EKF always has a height estimate regardless of mode of operation
    // Correct for the IMU offset in body frame (EKF calculations are at the IMU)
    // Also correct for changes to the origin height
    if ((ekfParam._originHgtMode & (1 << HGT_SOURCE_GPS)) == 0)
    {
        // Any sensor height drift corrections relative to the WGS-84 reference are applied to the origin.
        *posD = outputDataNew.position.z;
    }
    else
    {
        // The origin height is static and corrections are applied to the local vertical position
        // so that height returned by getLLH() = height returned by getOriginLLH - posD
        *posD = outputDataNew.position.z + 0.01f * (float)EKF_origin.alt - ekfGpsRefHgt;
    }

    // Return the current height solution status
    return filterStatus.flags.vert_pos;
}

// return the estimated height of body frame origin above ground level
bool getHAGL(float *HAGL)
{
    *HAGL = terrainState - outputDataNew.position.z;
    // If we know the terrain offset and altitude, then we have a valid height above ground estimate
    return !hgtTimeout && gndOffsetValid && coreHealthy();
}

// Return the last calculated latitude, longitude and height of the body frame origin in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool getLLH(gpsLocation_t *loc)
{
    gpsLocation_t origin;
    float posD;

    if (coreGetPosD(&posD) && getOriginLLH(&origin))
    {
        // Altitude returned is an absolute altitude relative to the WGS-84 spherioid
        loc->alt = origin.alt - posD * 100;

        // there are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no aiding)
        if (filterStatus.flags.horiz_pos_abs || filterStatus.flags.horiz_pos_rel)
        {
            loc->lat = EKF_origin.lat;
            loc->lon = EKF_origin.lon;
            // correct for IMU offset (EKF calculations are at the IMU position)
            offset_latlng(&loc->lat, &loc->lon, outputDataNew.position.x, outputDataNew.position.y);
            return true;
        }
        else
        {
            // we could be in constant position mode  because the vehicle has taken off without GPS, or has lost GPS
            // in this mode we cannot use the EKF states to estimate position so will return the best available data
            if (gpsSol.fixType >= GPS_FIX_2D)
            {
                // we have a GPS position fix to return
                gpsLocation_t gpsloc = {.lat = gpsSol.llh.lat, .lon = gpsSol.llh.lon, .alt = gpsSol.llh.alt};
                loc->lat = gpsloc.lat;
                loc->lon = gpsloc.lon;
                return true;
            }
            else
            {
                loc->lat = EKF_origin.lat;
                loc->lon = EKF_origin.lon;
                if (PV_AidingMode == AID_NONE)
                {
                    offset_latlng(&loc->lat, &loc->lon, lastKnownPositionNE.x, lastKnownPositionNE.y);
                }
                else
                {
                    offset_latlng(&loc->lat, &loc->lon, outputDataNew.position.x, outputDataNew.position.y);
                }
                return false;
            }
        }
    }
    else
    {
        // If no origin has been defined for the EKF, then we cannot use its position states so return a raw
        // GPS reading if available and return false
        if (gpsSol.fixType >= GPS_FIX_3D)
        {
            gpsLocation_t gpsloc = {.lat = gpsSol.llh.lat, .lon = gpsSol.llh.lon, .alt = gpsSol.llh.alt};
            *loc = gpsloc;
        }
        return false;
    }
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void getEkfControlLimits(float *ekfGndSpdLimit, float *ekfNavVelGainScaler)
{
    if (PV_AidingMode == AID_RELATIVE)
    {
        // allow 1.0 rad/sec margin for angular motion
        *ekfGndSpdLimit = MAX((ekfParam._maxFlowRate - 1.0f), 0.0f) * MAX((terrainState - ekfStates.stateStruct.position.v[2]), rngOnGnd);
        // use standard gains up to 5.0 metres height and reduce above that
        *ekfNavVelGainScaler = 4.0f / MAX((terrainState - ekfStates.stateStruct.position.v[2]), 4.0f);
    }
    else
    {
        *ekfGndSpdLimit = 400.0f; // return 80% of max filter speed
        *ekfNavVelGainScaler = 1.0f;
    }
}

// return the LLH location of the filters NED origin
bool getOriginLLH(gpsLocation_t *loc)
{
    if (validOrigin)
    {
        *loc = EKF_origin;
        // report internally corrected reference height if enabled
        if ((ekfParam._originHgtMode & (1 << HGT_SOURCE_GPS)) == 0)
        {
            loc->alt = (int32_t)(100.0f * ekfGpsRefHgt);
        }
    }
    return validOrigin;
}

// return earth magnetic field estimates in measurement units / 1000
void getMagNED(fpVector3_t *magNED)
{
    magNED->x = (ekfStates.stateStruct.earth_magfield.x * 1000.0f);
    magNED->y = (ekfStates.stateStruct.earth_magfield.y * 1000.0f);
    magNED->z = (ekfStates.stateStruct.earth_magfield.z * 1000.0f);
}

// return body magnetic field estimates in measurement units / 1000
void getMagXYZ(fpVector3_t *magXYZ)
{
    magXYZ->x = (ekfStates.stateStruct.body_magfield.x * 1000.0f);
    magXYZ->y = (ekfStates.stateStruct.body_magfield.y * 1000.0f);
    magXYZ->z = (ekfStates.stateStruct.body_magfield.z * 1000.0f);
}

// return magnetometer offsets
// return true if offsets are valid
bool getMagOffsets(fpVector3_t *magOffsets)
{
    if (!ekf_useCompass())
    {
        return false;
    }

    // compass offsets are valid if we have finalised magnetic field initialisation, magnetic field learning is not prohibited,
    // primary compass is valid and state variances have converged
    const float maxMagVar = 5E-6f;
    bool variancesConverged = (P[19][19] < maxMagVar) && (P[20][20] < maxMagVar) && (P[21][21] < maxMagVar);
    if (finalInflightMagInit && !inhibitMagStates && variancesConverged)
    {
        magOffsets->x = (compassConfig()->magZero.raw[X] / 1024 * compassConfig()->magGain[X]) - (ekfStates.stateStruct.body_magfield.x * 1000.0f);
        magOffsets->y = (compassConfig()->magZero.raw[Y] / 1024 * compassConfig()->magGain[Y]) - (ekfStates.stateStruct.body_magfield.y * 1000.0f);
        magOffsets->z = (compassConfig()->magZero.raw[Z] / 1024 * compassConfig()->magGain[Z]) - (ekfStates.stateStruct.body_magfield.z * 1000.0f);
        return true;
    }
    else
    {
        magOffsets->x = (compassConfig()->magZero.raw[X] / 1024 * compassConfig()->magGain[X]);
        magOffsets->y = (compassConfig()->magZero.raw[Y] / 1024 * compassConfig()->magGain[Y]);
        magOffsets->z = (compassConfig()->magZero.raw[Z] / 1024 * compassConfig()->magGain[Z]);
        return false;
    }
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
bool getInnovations(fpVector3_t *velInnov, fpVector3_t *posInnov, fpVector3_t *magInnov, float *tasInnov, float *yawInnov)
{
    velInnov->x = innovVelPos[0];
    velInnov->y = innovVelPos[1];
    velInnov->z = innovVelPos[2];
    posInnov->x = innovVelPos[3];
    posInnov->y = innovVelPos[4];
    posInnov->z = innovVelPos[5];
    magInnov->x = 1e3f * innovMag.v[0]; // Convert back to sensor units
    magInnov->y = 1e3f * innovMag.v[1]; // Convert back to sensor units
    magInnov->z = 1e3f * innovMag.v[2]; // Convert back to sensor units
    *tasInnov = innovVtas;
    *yawInnov = innovYaw;

    return true;
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
// this indicates the amount of margin available when tuning the various error traps
// also return the delta in position due to the last position reset
bool getVariances(float *velVar, float *posVar, float *hgtVar, fpVector3_t *magVar, float *tasVar, fpVector2_t *offset)
{
    *velVar = sqrtf(velTestRatio);
    *posVar = sqrtf(posTestRatio);
    *hgtVar = sqrtf(hgtTestRatio);
    // If we are using simple compass yaw fusion, populate all three components with the yaw test ratio to provide an equivalent output
    magVar->x = sqrtf(MAX(magTestRatio.x, yawTestRatio));
    magVar->y = sqrtf(MAX(magTestRatio.y, yawTestRatio));
    magVar->z = sqrtf(MAX(magTestRatio.z, yawTestRatio));
    *tasVar = sqrtf(tasTestRatio);
    *offset = posResetNE;

    return true;
}

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
void getFilterFaults(uint16_t *faults)
{
    *faults = ((isnan(ekfStates.stateStruct.quat.q0) || isnan(ekfStates.stateStruct.quat.q1) || isnan(ekfStates.stateStruct.quat.q2) || isnan(ekfStates.stateStruct.quat.q3)) << 0 |
               (isnan(ekfStates.stateStruct.velocity.x) || isnan(ekfStates.stateStruct.velocity.y) || isnan(ekfStates.stateStruct.velocity.z)) << 1 |
               faultStatus.bad_xmag << 2 |
               faultStatus.bad_ymag << 3 |
               faultStatus.bad_zmag << 4 |
               faultStatus.bad_airspeed << 5 |
               faultStatus.bad_sideslip << 6 |
               !statesInitialised << 7);
}

/*
return filter timeout status as a bitmasked integer
 0 = position measurement timeout
 1 = velocity measurement timeout
 2 = height measurement timeout
 3 = magnetometer measurement timeout
 4 = true airspeed measurement timeout
 5 = unassigned
 6 = unassigned
 7 = unassigned
*/
// Return the navigation filter status message
void getFilterStatus(nav_filter_status_t *status)
{
    *status = filterStatus;
}

#if HAL_GCS_ENABLED
// send an EKF_STATUS message to GCS
void send_status_report(GCS_MAVLINK &link)
{
    // prepare flags
    uint16_t flags = 0;
    if (filterStatus.flags.attitude)
    {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel)
    {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel)
    {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel)
    {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs)
    {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos)
    {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt)
    {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode)
    {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel)
    {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs)
    {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized)
    {
        flags |= EKF_UNINITIALIZED;
    }

    // get variances
    float velVar = 0, posVar = 0, hgtVar = 0, tasVar = 0;
    fpVector3_t magVar;
    fpVector2_t offset;
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);

    const float mag_max = fmaxf(fmaxf(magVar.x, magVar.y), magVar.z);

    // Only report range finder normalised innovation levels if the EKF needs the data for primary
    // height estimation or optical flow operation. This prevents false alarms at the GCS if a
    // range finder is fitted for other applications
    float temp;
    if (((ekfParam._useRngSwHgt > 0) && activeHgtSource == HGT_SOURCE_RNG) || (PV_AidingMode == AID_RELATIVE && flowDataValid))
    {
        temp = sqrtf(auxRngTestRatio);
    }
    else
    {
        temp = 0.0f;
    }

    // send message
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags, velVar, posVar, hgtVar, mag_max, temp, tasVar);
}
#endif // HAL_GCS_ENABLED

// report the number of frames lapsed since the last state prediction
// this is used by other instances to level load
uint8_t getFramesSincePredict(void)
{
    return framesSincePredict;
}