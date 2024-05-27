/*
  22 state EKF based on https://github.com/priseborough/InertialNav

  Converted from Matlab to C++ by Paul Riseborough
  Converted from C++ to C by julio Cesar Matias

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

/*

// This returns the specific forces in the NED frame
void NavEKF::getAccelNED(fpVector3_t &accelNED) const
{
    if (core) {
        core->getAccelNED(accelNED);
    }
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool NavEKF::resetHeightDatum(void)
{
    if (!core) {
        return false;
    }
    return core->resetHeightDatum();
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (core) {
        core->getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    } else {
        ekfGndSpdLimit = 0;
        ekfNavVelGainScaler = 0;
    }
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF::getAccelZBias(float &zbias1, float &zbias2) const
{
    if (core) {
        core->getAccelZBias(zbias1, zbias2);
    } else {
        zbias1 = zbias2 = 0;
    }
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF::getMagNED(fpVector3_t &magNED) const
{
    if (core) {
        core->getMagNED(magNED);
    }
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF::getMagXYZ(fpVector3_t &magXYZ) const
{
    if (core) {
        core->getMagXYZ(magXYZ);
    }
}

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool NavEKF::getMagOffsets(uint8_t mag_idx, fpVector3_t &magOffsets) const
{
    if (!core) {
        return false;
    }
    return core->getMagOffsets(mag_idx, magOffsets);
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF::getLLH(struct Location &loc) const
{
    if (!core) {
        return false;
    }
    return core->getLLH(loc);
}

// return the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool NavEKF::getOriginLLH(struct Location &loc) const
{
    if (!core) {
        return false;
    }
    return core->getOriginLLH(loc);
}

// set the latitude and longitude and height used to set the NED origin
// All NED positions calcualted by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool NavEKF::setOriginLLH(struct Location &loc)
{
    if (!core) {
        return false;
    }
    return core->setOriginLLH(loc);
}

// return estimated height above ground level
// return false if ground height is not being estimated.
bool NavEKF::getHAGL(float &HAGL) const
{
    if (!core) {
        return false;
    }
    return core->getHAGL(HAGL);
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void NavEKF::getInnovations(fpVector3_t &velInnov, fpVector3_t &posInnov, fpVector3_t &magInnov, float &tasInnov) const
{
    if (core) {
        core->getInnovations(velInnov, posInnov, magInnov, tasInnov);
    } else {
        tasInnov = 0;
    }
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void NavEKF::getVariances(float &velVar, float &posVar, float &hgtVar, fpVector3_t &magVar, float &tasVar, fpVector2_t &offset) const
{
    if (core) {
        core->getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    } else {
        velVar = 0;
        posVar = 0;
        hgtVar = 0;
        tasVar = 0;
    }
}

// write the raw optical flow measurements
// rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
// rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
// rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
// The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
// msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
void NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, fpVector2_t &rawFlowRates, fpVector2_t &rawGyroRates, uint32_t &msecFlowMeas)
{
    if (core) {
        core->writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
    }
}

// return data for debugging optical flow fusion
void NavEKF::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov,
                           float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    if (core) {
        core->getFlowDebug(varFlow, gndOffset, flowInnovX, flowInnovY, auxInnov, HAGL, rngInnov, range, gndOffsetErr);
    } else {
        varFlow = 0;
        gndOffset = 0;
        flowInnovX = 0;
        flowInnovY = 0;
        auxInnov = 0;
        HAGL = 0;
        rngInnov = 0;
        range = 0;
        gndOffsetErr = 0;
    }
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF::setTakeoffExpected(bool val)
{
    if (core) {
        core->setTakeoffExpected(val);
    }
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF::setTouchdownExpected(bool val)
{
    if (core) {
        core->setTouchdownExpected(val);
    }
}


  return the filter fault status as a bitmasked integer
  0 = quaternions are NaN
  1 = velocities are NaN
  2 = badly conditioned X magnetometer fusion
  3 = badly conditioned Y magnetometer fusion
  5 = badly conditioned Z magnetometer fusion
  6 = badly conditioned airspeed fusion
  7 = badly conditioned synthetic sideslip fusion
  7 = filter is not initialised

void NavEKF::getFilterFaults(uint16_t &faults) const
{
    if (core) {
        core->getFilterFaults(faults);
    } else {
        faults = 0;
    }
}


  return filter timeout status as a bitmasked integer
  0 = position measurement timeout
  1 = velocity measurement timeout
  2 = height measurement timeout
  3 = magnetometer measurement timeout
  5 = unassigned
  6 = unassigned
  7 = unassigned
  7 = unassigned

void NavEKF::getFilterTimeouts(uint8_t &timeouts) const
{
    if (core) {
        core->getFilterTimeouts(timeouts);
    } else {
        timeouts = 0;
    }
}


  return filter status flags

void NavEKF::getFilterStatus(nav_filter_status &status) const
{
    if (core) {
        core->getFilterStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}


return filter gps quality check status
void  NavEKF::getFilterGpsStatus(nav_gps_status &status) const
{
    if (core) {
        core->getFilterGpsStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

// send an EKF_STATUS_REPORT message to GCS
void NavEKF::send_status_report(void)
{
    if (core) {
        core->send_status_report();
    }
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF::getHeightControlLimit(float &height) const
{
    if (!core) {
        return false;
    }
    return core->getHeightControlLimit(height);
}

// returns true of the EKF thinks the GPS is glitching
bool NavEKF::getGpsGlitchStatus(void) const
{
    if(!core) {
        return false;
    }
    return core->getGpsGlitchStatus();
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t NavEKF::getLastYawResetAngle(float &yawAng) const
{
    if (!core) {
        return 0;
    }
    return core->getLastYawResetAngle(yawAng);
}

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF::getLastPosNorthEastReset(fpVector2_t &pos) const
{
    if (!core) {
        return 0;
    }
    return core->getLastPosNorthEastReset(pos);
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF::getLastVelNorthEastReset(fpVector2_t &vel) const
{
    if (!core) {
        return 0;
    }
    return core->getLastVelNorthEastReset(vel);
}

// report the reason for why the backend is refusing to initialise
const char *NavEKF::prearm_failure_reason(void) const
{
    if (!core) {
        return nullptr;
    }
    return core->prearm_failure_reason();
}

// return weighting of first IMU in blending function
void NavEKF::getIMU1Weighting(float &ret) const
{
    if (core) {
        core->getIMU1Weighting(ret);
    } else {
        ret = 0;
    }
}
*/