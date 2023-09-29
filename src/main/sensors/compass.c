/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_ak8975.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/compass/compass_hmc5883l.h"
#include "drivers/compass/compass_mag3110.h"
#include "drivers/compass/compass_ist8310.h"
#include "drivers/compass/compass_ist8308.h"
#include "drivers/compass/compass_qmc5883l.h"
#include "drivers/compass/compass_mpu9250.h"
#include "drivers/compass/compass_lis3mdl.h"
#include "drivers/compass/compass_rm3100.h"
#include "drivers/compass/compass_vcm5883.h"
#include "drivers/compass/compass_mlx90393.h"
#include "drivers/compass/compass_msp.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/imu.h"

#include "io/gps.h"
#include "io/beeper.h"

#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/compass_calibration.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

mag_t mag; // mag access functions

#ifdef USE_MAG

PG_REGISTER_WITH_RESET_TEMPLATE(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 7);

PG_RESET_TEMPLATE(compassConfig_t, compassConfig,
                  .mag_align = SETTING_ALIGN_MAG_DEFAULT,
                  .mag_hardware = SETTING_MAG_HARDWARE_DEFAULT,
                  .mag_external = SETTING_MAG_EXTERNAL_DEFAULT,
                  .mag_auto_rotate = SETTING_MAG_ROTATE_AUTO_DEFAULT,
                  .mag_cal_type = SETTING_MAG_CAL_TYPE_DEFAULT,
                  .mag_declination = SETTING_MAG_DECLINATION_DEFAULT,
                  .offSet.x = SETTING_MAGOFFSET_X_DEFAULT,
                  .offSet.y = SETTING_MAGOFFSET_Y_DEFAULT,
                  .offSet.z = SETTING_MAGOFFSET_Z_DEFAULT,
                  .scaleFactor = SETTING_MAGSCALEFACTOR_DEFAULT,
                  .diagonals.x = SETTING_MAGDIAGONALS_X_DEFAULT,
                  .diagonals.y = SETTING_MAGDIAGONALS_Y_DEFAULT,
                  .diagonals.z = SETTING_MAGDIAGONALS_Z_DEFAULT,
                  .offDiagonals.x = SETTING_MAGOFFDIAGONALS_X_DEFAULT,
                  .offDiagonals.y = SETTING_MAGOFFDIAGONALS_Y_DEFAULT,
                  .offDiagonals.z = SETTING_MAGOFFDIAGONALS_Z_DEFAULT,
#ifdef USE_DUAL_MAG
                  .mag_to_use = SETTING_MAG_TO_USE_DEFAULT,
#endif
);

static float yawShoted;

bool compassDetect(magDev_t *dev, magSensor_e magHardwareToUse)
{
    magSensor_e magHardware = MAG_NONE;
    requestedSensors[SENSOR_INDEX_MAG] = magHardwareToUse;

    switch (magHardwareToUse)
    {
    case MAG_AUTODETECT:
        FALLTHROUGH;

    case MAG_QMC5883:
#ifdef USE_MAG_QMC5883
        if (qmc5883Detect(dev))
        {
            magHardware = MAG_QMC5883;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_HMC5883:
#ifdef USE_MAG_HMC5883
        if (hmc5883lDetect(dev))
        {
            magHardware = MAG_HMC5883;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_AK8975:
#ifdef USE_MAG_AK8975
        if (ak8975Detect(dev))
        {
            magHardware = MAG_AK8975;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_AK8963:
#ifdef USE_MAG_AK8963
        if (ak8963Detect(dev))
        {
            magHardware = MAG_AK8963;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_MAG3110:
#ifdef USE_MAG_MAG3110
        if (mag3110detect(dev))
        {
            magHardware = MAG_MAG3110;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_IST8310:
#ifdef USE_MAG_IST8310
        if (ist8310Detect(dev))
        {
            magHardware = MAG_IST8310;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_IST8308:
#ifdef USE_MAG_IST8308
        if (ist8308Detect(dev))
        {
            magHardware = MAG_IST8308;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_MPU9250:
#ifdef USE_MAG_MPU9250
        if (mpu9250CompassDetect(dev))
        {
            magHardware = MAG_MPU9250;
            break;
        }
#endif
        FALLTHROUGH;

    case MAG_LIS3MDL:
#ifdef USE_MAG_LIS3MDL
        if (lis3mdlDetect(dev))
        {
            magHardware = MAG_LIS3MDL;
            break;
        }
#endif

        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_MSP:
#ifdef USE_MAG_MSP
        // Skip autodetection for MSP mag
        if (magHardwareToUse != MAG_AUTODETECT && mspMagDetect(dev))
        {
            magHardware = MAG_MSP;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_RM3100:
#ifdef USE_MAG_RM3100
        if (rm3100MagDetect(dev))
        {
            magHardware = MAG_RM3100;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_VCM5883:
#ifdef USE_MAG_VCM5883
        if (vcm5883Detect(dev))
        {
            magHardware = MAG_VCM5883;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_MLX90393:
#ifdef USE_MAG_MLX90393
        if (mlx90393Detect(dev))
        {
            magHardware = MAG_MLX90393;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_FAKE:
#ifdef USE_FAKE_MAG
        if (fakeMagDetect(dev))
        {
            magHardware = MAG_FAKE;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT)
        {
            break;
        }
        FALLTHROUGH;

    case MAG_NONE:
        magHardware = MAG_NONE;
        break;
    }

    if (magHardware == MAG_NONE)
    {
        sensorsClear(SENSOR_MAG);
        return false;
    }

    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);

    return true;
}

bool compassInit(void)
{
#ifdef USE_DUAL_MAG
    mag.dev.magSensorToUse = compassConfig()->mag_to_use;
#else
    mag.dev.magSensorToUse = 0;
#endif

    if (!compassDetect(&mag.dev, compassConfig()->mag_hardware))
    {
        return false;
    }

    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    const bool ret = mag.dev.init(&mag.dev);
    LED1_OFF;

    if (!ret)
    {
        sensorsClear(SENSOR_MAG);
    }

    setCompassCalibrationFinished(true);

    return ret;
}

bool compassIsHealthy(void)
{
    return (mag.time - mag.lastUpdateUs < MS2US(500));
}

bool compassIsCalibrationComplete(void)
{
    if (STATE(COMPASS_CALIBRATED))
    {
        return true;
    }

    return false;
}

void compassUpdate(timeUs_t currentTimeUs)
{
#ifdef USE_SIMULATOR
    if (ARMING_FLAG(SIMULATOR_MODE_HITL))
    {
        return;
    }
#endif

#if defined(SITL_BUILD)
    ENABLE_STATE(COMPASS_CALIBRATED);
#else
    // Check offSet
    if (calc_length_pythagorean_3D(compassConfig()->offSet.x, compassConfig()->offSet.y, compassConfig()->offSet.z) == 0.0f)
    {
        DISABLE_STATE(COMPASS_CALIBRATED);
    }
    else
    {
        ENABLE_STATE(COMPASS_CALIBRATED);
    }
#endif

    mag.time = currentTimeUs;

    if (!mag.dev.read(&mag.dev))
    {
        mag.magADC[X] = 0.0f;
        mag.magADC[Y] = 0.0f;
        mag.magADC[Z] = 0.0f;
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        mag.magADC[axis] = mag.dev.magADCRaw[axis]; // int32_t copy to work with
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        mag.magADC[axis] *= 1000.0f / 3000.0f; // Apply the Magnetometer scale
    }

    applySensorAlignment(mag.magADC, mag.magADC, compassConfig()->mag_align);
    applyBoardAlignment(mag.magADC);

    if (STATE(CALIBRATE_MAG))
    {
        beeper(BEEPER_ACTION_SUCCESS);
        DISABLE_STATE(CALIBRATE_MAG);

        if (compassConfig()->mag_cal_type == MAG_CALIBRATION_USING_SAMPLES)
        {
            if (!getCompassCalibrationFinished()) // Did the user click the cancel calibration button? Yes...
            {
                // compassCalibrationStop();
                // setCompassCalibrationFinished(true);
                return;
            }

            // Internal compasses get twice the threshold. This is because internal compass tend to be a lot noisier
            if (compassConfig()->mag_external == MAG_INTERNAL)
            {
                compassCalibrationStart(COMPASS_CALIBRATION_INIT_DELAY, COMPASS_OFFSETS_MAX_DEFAULT, COMPASS_CALIBRATION_FITNESS_DEFAULT * 2);
            }
            else
            {
                compassCalibrationStart(COMPASS_CALIBRATION_INIT_DELAY, COMPASS_OFFSETS_MAX_DEFAULT, COMPASS_CALIBRATION_FITNESS_DEFAULT);
            }

            if (compassConfig()->mag_auto_rotate)
            {
                const bool isExternalCompass = compassConfig()->mag_external == MAG_EXTERNAL;
                sensor_align_e rotation = isExternalCompass ? compassConfig()->mag_align : ALIGN_DEFAULT;
                compassCalibrationSetOrientation(rotation, isExternalCompass, compassConfig()->mag_auto_rotate == 2);
            }
        }
        else
        {
            yawShoted = atan2_approx(rotationMatrix.m[1][0], rotationMatrix.m[0][0]);
        }

        setCompassCalibrationFinished(false); // Set false to init the calibration
    }

    if (!getCompassCalibrationFinished() && compassConfig()->mag_cal_type == MAG_CALIBRATION_USING_SAMPLES)
    {
        LED0_TOGGLE;

        fpVector3_t magSamples = {.v = {mag.magADC[X], mag.magADC[Y], mag.magADC[Z]}};

        compassCalibrationSetNewSample(magSamples);

        compassCalibrationUpdate();

        const Report cal_report = getCompassCalibrationReport();

        if (cal_report.status == COMPASS_CAL_SUCCESS)
        {
            compassConfigMutable()->offSet.x = cal_report.ofs.x;
            compassConfigMutable()->offSet.y = cal_report.ofs.y;
            compassConfigMutable()->offSet.z = cal_report.ofs.z;

            compassConfigMutable()->diagonals.x = cal_report.diag.x;
            compassConfigMutable()->diagonals.y = cal_report.diag.y;
            compassConfigMutable()->diagonals.z = cal_report.diag.z;

            compassConfigMutable()->offDiagonals.x = cal_report.offdiag.x;
            compassConfigMutable()->offDiagonals.y = cal_report.offdiag.y;
            compassConfigMutable()->offDiagonals.z = cal_report.offdiag.z;

            compassConfigMutable()->scaleFactor = cal_report.scale_factor;

            if (cal_report.check_orientation && compassConfig()->mag_auto_rotate == 2)
            {
                compassConfigMutable()->mag_align = cal_report.orientation;
            }

            if (!compassIsCalibrating())
            {
                saveConfigAndNotify();
                setCompassCalibrationFinished(true);
            }
        }
    }

    const fpVector3_t offsets = {.v = {compassConfig()->offSet.x, compassConfig()->offSet.y, compassConfig()->offSet.z}};
    const fpVector3_t diagonals = {.v = {compassConfig()->diagonals.x, compassConfig()->diagonals.y, compassConfig()->diagonals.z}};
    const fpVector3_t offdiagonals = {.v = {compassConfig()->offDiagonals.x, compassConfig()->offDiagonals.y, compassConfig()->offDiagonals.z}};
    const float magScaleFactor = compassConfig()->scaleFactor;

    fpVector3_t magCorrectField = {.v = {mag.magADC[X], mag.magADC[Y], mag.magADC[Z]}};

    // Add in the basic offsets
    magCorrectField.x -= offsets.x;
    magCorrectField.y -= offsets.y;
    magCorrectField.z -= offsets.z;

    // Add in scale factor, use a wide sanity check. The calibrator uses a narrower check.
    if (magScaleFactor > COMPASS_MIN_SCALE_FACTOR && magScaleFactor < COMPASS_MAX_SCALE_FACTOR)
    {
        magCorrectField.x *= magScaleFactor;
        magCorrectField.y *= magScaleFactor;
        magCorrectField.z *= magScaleFactor;
    }

    // Apply eliptical correction
    if (diagonals.x != 0.0f && diagonals.y != 0.0f && diagonals.z != 0.0f)
    {
        fpMatrix3_t mat = initMatrixUsingVector(diagonals.x, offdiagonals.x, offdiagonals.y,
                                                offdiagonals.x, diagonals.y, offdiagonals.z,
                                                offdiagonals.y, offdiagonals.z, diagonals.z);

        magCorrectField = multiplyMatrixByVector(mat, magCorrectField);
    }

    if (!getCompassCalibrationFinished() && compassConfig()->mag_cal_type == MAG_CALIBRATION_USING_GPS)
    {
        LED0_TOGGLE;

        if (CompassCalibrationFixedYaw(yawShoted, magCorrectField, &compassConfigMutable()->offSet, &compassConfigMutable()->diagonals, &compassConfigMutable()->offDiagonals))
        {
            saveConfigAndNotify();
            setCompassCalibrationFinished(true);
        }
    }

    mag.magADC[X] = magCorrectField.x;
    mag.magADC[Y] = magCorrectField.y;
    mag.magADC[Z] = magCorrectField.z;

    mag.lastUpdateUs = currentTimeUs;

    return;
}

#endif
