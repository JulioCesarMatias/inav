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
                  .mag_auto_rotate = SETTING_MAG_ROTATE_AUTO_DEFAULT,
                  .mag_declination = SETTING_MAG_DECLINATION_DEFAULT,
#ifdef USE_DUAL_MAG
                  .mag_to_use = SETTING_MAG_TO_USE_DEFAULT,
#endif
                  .rollDeciDegrees = SETTING_ALIGN_MAG_ROLL_DEFAULT,
                  .pitchDeciDegrees = SETTING_ALIGN_MAG_PITCH_DEFAULT,
                  .yawDeciDegrees = SETTING_ALIGN_MAG_YAW_DEFAULT, );

static bool magUpdatedAtLeastOnce = false;

bool compassDetect(magDev_t *dev, magSensor_e magHardwareToUse)
{
    magSensor_e magHardware = MAG_NONE;
    requestedSensors[SENSOR_INDEX_MAG] = magHardwareToUse;

    dev->magAlign.useExternal = false;
    dev->magAlign.onBoard = ALIGN_DEFAULT;

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

    if (compassConfig()->rollDeciDegrees != 0 || compassConfig()->pitchDeciDegrees != 0 || compassConfig()->yawDeciDegrees != 0)
    {
        // Externally aligned compass
        mag.dev.magAlign.useExternal = true;

        fp_angles_t compassAngles = {
            .angles.roll = DECIDEGREES_TO_RADIANS(compassConfig()->rollDeciDegrees),
            .angles.pitch = DECIDEGREES_TO_RADIANS(compassConfig()->pitchDeciDegrees),
            .angles.yaw = DECIDEGREES_TO_RADIANS(compassConfig()->yawDeciDegrees),
        };

        rotationMatrixFromEulerAngles(&mag.dev.magAlign.externalRotation, &compassAngles);
    }
    else
    {
        mag.dev.magAlign.useExternal = false;

        if (compassConfig()->mag_align != ALIGN_DEFAULT)
        {
            mag.dev.magAlign.onBoard = compassConfig()->mag_align;
        }
        else
        {
            // mag.dev.magAlign.onBoard = CW270_DEG_FLIP; // The most popular default is 270FLIP for external mags
        }
    }

    return ret;
}

bool compassIsHealthy(void)
{
    return (mag.magADC[X] != 0) || (mag.magADC[Y] != 0) || (mag.magADC[Z] != 0);
}

bool compassIsReady(void)
{
    return magUpdatedAtLeastOnce;
}

bool compassIsCalibrationComplete(void)
{
    if (STATE(COMPASS_CALIBRATED))
    {
        return true;
    }

    return false;
}

timeDelta_t compassUpdate(timeUs_t currentTimeUs)
{
    timeDelta_t compassDelay = HZ2US(10); // 10Hz in normal operation (calibration is not running)
    static bool compassCalibrationStarted = false;

#ifdef USE_SIMULATOR
    if (ARMING_FLAG(SIMULATOR_MODE_HITL))
    {
        magUpdatedAtLeastOnce = true;
        return compassDelay;
    }
#endif

#if defined(SITL_BUILD)
    ENABLE_STATE(COMPASS_CALIBRATED);
#else
    // Check offSet
    if (calc_length_pythagorean_3D(compassConfig()->offSet.raw[X], compassConfig()->offSet.raw[Y], compassConfig()->offSet.raw[Z]) == 0.0f)
    {
        DISABLE_STATE(COMPASS_CALIBRATED);
    }
    else
    {
        ENABLE_STATE(COMPASS_CALIBRATED);
    }
#endif

    if (!mag.dev.read(&mag.dev))
    {
        mag.magADC[X] = 0;
        mag.magADC[Y] = 0;
        mag.magADC[Z] = 0;
        return compassDelay;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        mag.magADC[axis] = mag.dev.magADCRaw[axis]; // int32_t copy to work with
    }

    if (STATE(CALIBRATE_MAG))
    {
        compassCalibrationStarted = true;

        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
        {
            compassConfigMutable()->offSet[axis] = 0.0f;
            compassConfigMutable()->scaleFactor = 0.0f;
            compassConfigMutable()->diagonals[axis] = 1.0f;
            compassConfigMutable()->offDiagonals[axis] = 0.0f;
        }

        beeper(BEEPER_ACTION_SUCCESS);
        DISABLE_STATE(CALIBRATE_MAG);
    }

    if (mag.dev.magAlign.useExternal)
    {
        fpVector3_t rotated = {
            .x = mag.magADC[X],
            .y = mag.magADC[Y],
            .z = mag.magADC[Z],
        };

        multiplicationTransposeMatrixByVector(mag.dev.magAlign.externalRotation, &rotated);

        mag.magADC[X] = rotated.x;
        mag.magADC[Y] = rotated.y;
        mag.magADC[Z] = rotated.z;
    }
    else
    {
        // On-board compass
        applySensorAlignment(mag.magADC, mag.magADC, mag.dev.magAlign.onBoard);
        applyBoardAlignment(mag.magADC);
    }

    if (compassCalibrationStarted)
    {
        LED0_TOGGLE;

        /*
            Set 40Hz in calibration mode (COMPASS_CAL_NUM_SAMPLES bursts in 7.5 sec)
            We cannot use 10Hz here, because COMPASS_CAL_NUM_SAMPLES would take a longer burst time
            Also, leaving it at 40Hz during calibration respects the reschedule time when the AK6983 compass is used
        */
        compassDelay = HZ2US(40);

        fpVector3_t magSamples = {.v = {mag.magADC[X], mag.magADC[Y], mag.magADC[Z]}};

        compassCalibrationSetNewSample(magSamples);

        if (compassConfig()->mag_auto_rotate)
        {
            sensor_align_e rotation = mag.dev.magAlign.useExternal ? compassConfig()->mag_align : ALIGN_DEFAULT;
            compassCalibrationSetOrientation(rotation, mag.dev.magAlign.useExternal, compassConfig()->mag_auto_rotate == 2);
        }

        compassCalibrationStart(COMPASS_CALIBRATION_INIT_DELAY, COMPASS_OFFSETS_MAX_DEFAULT, COMPASS_CALIBRATION_FITNESS_DEFAULT);
        compassCalibrationUpdate();

        const Report cal_report = getCompassCalibrationReport();

        if (cal_report.status == COMPASS_CAL_SUCCESS)
        {
            compassConfigMutable()->offSet[X] = cal_report.ofs.x;
            compassConfigMutable()->offSet[Y] = cal_report.ofs.y;
            compassConfigMutable()->offSet[Z] = cal_report.ofs.z;

            compassConfigMutable()->diagonals[X] = cal_report.diag.x;
            compassConfigMutable()->diagonals[Y] = cal_report.diag.y;
            compassConfigMutable()->diagonals[Z] = cal_report.diag.z;

            compassConfigMutable()->offDiagonals[X] = cal_report.offdiag.x;
            compassConfigMutable()->offDiagonals[Y] = cal_report.offdiag.y;
            compassConfigMutable()->offDiagonals[Z] = cal_report.offdiag.z;

            compassConfigMutable()->scaleFactor = cal_report.scale_factor;

            if (cal_report.check_orientation && mag.dev.magAlign.useExternal && compassConfig()->mag_auto_rotate == 2)
            {
                compassConfigMutable()->mag_align = cal_report.orientation;
            }

            if (!is_calibrating())
            {
                saveConfigAndNotify();
                compassCalibrationStarted = false;
            }
        }
    }

    fpVector3_t magCorrectField = {.v = {mag.magADC[X], mag.magADC[Y], mag.magADC[Z]}};

    const fpVector3_t offsets = {.v = {compassConfig()->offSet[X], compassConfig()->offSet[Y], compassConfig()->offSet[Z]}};
    const fpVector3_t diagonals = {.v = {compassConfig()->diagonals[X], compassConfig()->diagonals[Y], compassConfig()->diagonals[Z]}};
    const fpVector3_t offdiagonals = {.v = {compassConfig()->offDiagonals[X], compassConfig()->offDiagonals[Y], compassConfig()->offDiagonals[Z]}};

    // add in the basic offsets
    magCorrectField.x += offsets.x;
    magCorrectField.y += offsets.y;
    magCorrectField.z += offsets.z;

    // add in scale factor, use a wide sanity check. The calibrator uses a narrower check.
    if (compassConfig()->scaleFactor > COMPASS_MIN_SCALE_FACTOR || compassConfig()->scaleFactor < COMPASS_MAX_SCALE_FACTOR)
    {
        magCorrectField.x *= compassConfig()->scaleFactor;
        magCorrectField.y *= compassConfig()->scaleFactor;
        magCorrectField.z *= compassConfig()->scaleFactor;
    }

    // apply eliptical correction
    if (diagonals.x != 0.0f && diagonals.y != 0.0f && diagonals.z != 0.0f)
    {
        fpMatrix3_t mat;

        mat.m[0][0] = diagonals.x;
        mat.m[0][1] = offdiagonals.x;
        mat.m[0][2] = offdiagonals.y;
        mat.m[1][0] = offdiagonals.x;
        mat.m[1][1] = diagonals.y;
        mat.m[1][2] = offdiagonals.z;
        mat.m[2][0] = offdiagonals.y;
        mat.m[2][1] = offdiagonals.z;
        mat.m[2][2] = diagonals.z;

        magCorrectField = multiplyMatrixByVector(mat, magCorrectField);
    }

    mag.magADC[X] = magCorrectField.x;
    mag.magADC[Y] = magCorrectField.y;
    mag.magADC[Z] = magCorrectField.z;

    magUpdatedAtLeastOnce = true;

    return compassDelay;
}

#endif
