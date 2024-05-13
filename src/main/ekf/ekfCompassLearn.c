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

#include "ekf/ekfCompassLearn.h"
#include "ekf/ekfCore.h"
#include "ekf/EKFGSF_yaw.h"
#include "fc/fc_core.h"
#include "flight/imu.h"

#ifndef SOLVE_AFTER

#define GSF_LEARN_NONE false
#define GSF_LEARN_RUNNING true

// accuracy threshold applied for GSF yaw estimate
#define YAW_ACCURACY_THRESHOLD_DEG 5.0f

bool gsfInitialised;
bool gsfLearnType = GSF_LEARN_NONE;
int16_t magAxisDeviation[XYZ_AXIS_COUNT];

/*
  Fast compass calibration given vehicle position and yaw.
  This is only suitable for vehicles where the field is close to spherical.
  It is useful for large vehicles where moving the vehicle to calibrate it
  is difficult.

  The offsets of the compass are set to values to bring
  them into consistency with the WMM tables at the given latitude and
  longitude. If compass_mask is zero then all enabled compass are
  calibrated.

  This assumes that the compass is correctly scaled in milliGauss
*/
static bool mag_cal_fixed_yaw(float yaw_deg, float lat_deg, float lon_deg)
{
    if (is_zero(lat_deg) && is_zero(lon_deg))
    {
        gpsLocation_t loc;
        // Get EKF position. If unavailable then try GPS location
        if (!getLLH(&loc))
        {
            if (gpsSol.fixType < GPS_FIX_3D)
            {
                sendEKFLogMessage("EKF Compass Learn GPS no position available");
                return false;
            }
            loc.lat = gpsSol.llh.lat;
            loc.lon = gpsSol.llh.lon;
        }
        lat_deg = loc.lat * 1.0e-7f;
        lon_deg = loc.lon * 1.0e-7f;
    }

    // get the magnetic field intensity and orientation
    float intensity;
    float declination;
    float inclination;

    if (!get_mag_field_ef(lat_deg, lon_deg, intensity, declination, inclination))
    {
        sendEKFLogMessage("EKF Compass Learn mag WMM table error");
        return false;
    }

    // create a field vector and rotate to the required orientation
    fpVector3_t field = {.v = {1e3f * intensity, 0.0f, 0.0f}};

    fpMat3_t R;
    matrixFromEuler(&R, 0.0f, -DEGREES_TO_RADIANS(inclination), DEGREES_TO_RADIANS(declination));

    field = multiplyMatrixByVector(R, field);

    fpMat3_t dcm;
    matrixFromEuler(&dcm, DECIDEGREES_TO_RADIANS(attitude.values.roll), DECIDEGREES_TO_RADIANS(attitude.values.pitch), DEGREES_TO_RADIANS(yaw_deg));

    // Rotate into body frame using provided yaw
    field = multiplyMatrixByVector(matrixTransposed(dcm), field);

    fpVector3_t measurement = getUncorrectedMagField();

    if (ABS(measurement.v[X]) > ABS(magAxisDeviation[X]))
    {
        magAxisDeviation[X] = measurement.v[X];
    }

    if (ABS(measurement.v[Y]) > ABS(magAxisDeviation[Y]))
    {
        magAxisDeviation[Y] = measurement.v[Y];
    }

    if (ABS(measurement.v[Z]) > ABS(magAxisDeviation[Z]))
    {
        magAxisDeviation[Z] = measurement.v[Z];
    }

    fpVector3_t offsets = {.v = {field.x - measurement.x, field.y - measurement.y, field.z - measurement.z}};

    compassConfigMutable()->magZero.raw[X] = lrintf(offsets.v[X]);
    compassConfigMutable()->magZero.raw[Y] = lrintf(offsets.v[Y]);
    compassConfigMutable()->magZero.raw[Z] = lrintf(offsets.v[Z]);

    compassConfigMutable()->magGain[X] = ABS(magAxisDeviation[X] - compassConfig()->magZero.raw[X]);
    compassConfigMutable()->magGain[Y] = ABS(magAxisDeviation[Y] - compassConfig()->magZero.raw[Y]);
    compassConfigMutable()->magGain[Z] = ABS(magAxisDeviation[Z] - compassConfig()->magZero.raw[Z]);

    // saveConfigAndNotify();

    return true;
}

void ekfCompassLearnUpdate(void)
{
    if (!ekfParam._ekfCompassLearn)
    {
        return;
    }

    if (!gsfInitialised)
    {
        sendEKFLogMessage("EKF Compass Learn initialised");
        gsfLearnType = GSF_LEARN_RUNNING;
        magAxisDeviation[X] = 0;
        magAxisDeviation[Y] = 0;
        magAxisDeviation[Z] = 0;
        gsfInitialised = true;
    }

    if (gsfLearnType != GSF_LEARN_RUNNING || !STATE(ARMED) || getFlightTime() < 3)
    {
        // only learn when flying and with enough time to be clear of the ground
        return;
    }

    if (fabsf(DECIDEGREES_TO_DEGREES(attitude.values.pitch)) > 50)
    {
        // we don't want to be too close to nose up, or yaw gets problematic.
        // Tailsitters need to wait till they are in forward flight
        return;
    }

    float yaw_rad;
    float yaw_variance;

    if (!EKFGSF_yaw_getYawData(&yaw_rad, &yaw_variance) ||
        yaw_variance <= 0.0f ||
        EKFGSF_yaw_getNClips() > 1 ||
        yaw_variance >= sq(DEGREES_TO_RADIANS(YAW_ACCURACY_THRESHOLD_DEG)))
    {
        // not converged
        return;
    }

    if (mag_cal_fixed_yaw(RADIANS_TO_DEGREES(yaw_rad), 0.0f, 0.0f))
    {
        gsfLearnType = GSF_LEARN_NONE;
        sendEKFLogMessage("EKF Compass Learn finished");
    }
}

#endif