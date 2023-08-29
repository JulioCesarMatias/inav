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
#include "sensors/gyro.h"
#include "sensors/sensors.h"

mag_t mag; // mag access functions

#ifdef USE_MAG

PG_REGISTER_WITH_RESET_TEMPLATE(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 6);

PG_RESET_TEMPLATE(compassConfig_t, compassConfig,
                  .mag_align = SETTING_ALIGN_MAG_DEFAULT,
                  .mag_hardware = SETTING_MAG_HARDWARE_DEFAULT,
                  .mag_declination = SETTING_MAG_DECLINATION_DEFAULT,
#ifdef USE_DUAL_MAG
                  .mag_to_use = SETTING_MAG_TO_USE_DEFAULT,
#endif
                  .magCalibrationTimeLimit = SETTING_MAG_CALIBRATION_TIME_DEFAULT,
                  .rollDeciDegrees = SETTING_ALIGN_MAG_ROLL_DEFAULT,
                  .pitchDeciDegrees = SETTING_ALIGN_MAG_PITCH_DEFAULT,
                  .yawDeciDegrees = SETTING_ALIGN_MAG_YAW_DEFAULT,
                  .magGain = {SETTING_MAGGAIN_X_DEFAULT, SETTING_MAGGAIN_Y_DEFAULT, SETTING_MAGGAIN_Z_DEFAULT}, );

#define COMPASS_CAL_NUM_SAMPLES 300 // Number of samples required for calculate the orientation (Vertices / Compass task in Hz)

fpVector3_t compass_sample_buffer[COMPASS_CAL_NUM_SAMPLES];
fpVector3_t compass_offset;

sensor_align_e _orientation;          // Latest detected orientation
sensor_align_e _orientation_solution; // Latest solution

static bool magUpdatedAtLeastOnce = false;
bool autoOrientationPassed = false;
bool allSamplesCollected = false;

float _orientation_confidence; // Measure of confidence in automatic orientation detection

int8_t roll_from_att;
int8_t pitch_from_att;
int8_t yaw_from_att;

uint16_t _samples_collected;

// @Param: AUTO_ROT
// @Description: When enabled this will automatically check the orientation of compasses on successful completion of compass calibration. If set to 2 then external compasses will have their orientation automatically corrected.
// @Values: 0:Disabled,1:CheckOnly,2:CheckAndFix,3:use same tolerance to auto rotate 45 deg rotations
uint8_t _rotate_auto = 2;

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

    if (compassConfig()->rollDeciDegrees != 0 ||
        compassConfig()->pitchDeciDegrees != 0 ||
        compassConfig()->yawDeciDegrees != 0)
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

#ifdef USE_SIMULATOR
    if (ARMING_FLAG(SIMULATOR_MODE_HITL))
    {
        magUpdatedAtLeastOnce = true;
        return compassDelay;
    }
#endif

    static sensorCalibrationState_t calState;
    static timeUs_t calStartedAt = 0;
    static timeUs_t autoOrientationtimestamp = 0;
    static int16_t magPrev[XYZ_AXIS_COUNT];
    static int magAxisDeviation[XYZ_AXIS_COUNT];

#if defined(SITL_BUILD)
    ENABLE_STATE(COMPASS_CALIBRATED);
#else
    // Check magZero
    if (compassConfig()->magZero.raw[X] == 0 && compassConfig()->magZero.raw[Y] == 0 && compassConfig()->magZero.raw[Z] == 0 &&
        compassConfig()->magGain[X] == 1024 && compassConfig()->magGain[Y] == 1024 && compassConfig()->magGain[Z] == 1024)
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
        calStartedAt = currentTimeUs;

        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
        {
            compassConfigMutable()->magZero.raw[axis] = 0;
            compassConfigMutable()->magGain[axis] = 1024;
            magPrev[axis] = 0;
            magAxisDeviation[axis] = 0; // Gain is based on the biggest absolute deviation from the mag zero point. Gain computation starts at 0
        }

        beeper(BEEPER_ACTION_SUCCESS);

        sensorCalibrationResetState(&calState);
        DISABLE_STATE(CALIBRATE_MAG);
    }

    if (calStartedAt != 0)
    {
        /*
            40Hz in calibration mode (COMPASS_CAL_NUM_SAMPLES bursts in 7.5 sec)
            We cannot use 10Hz here, because COMPASS_CAL_NUM_SAMPLES would take a longer burst time
            Also, leaving it at 40Hz during calibration respects the reschedule time when the AK6983 compass is used
        */
        if (_rotate_auto == 0)
        {
            compassDelay = HZ2US(40);
        }

        if ((currentTimeUs - calStartedAt) < S2US(compassConfig()->magCalibrationTimeLimit))
        {
            LED0_TOGGLE;

            float diffMag = 0;
            float avgMag = 0;

            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
            {
                diffMag += (mag.magADC[axis] - magPrev[axis]) * (mag.magADC[axis] - magPrev[axis]);
                avgMag += (mag.magADC[axis] + magPrev[axis]) * (mag.magADC[axis] + magPrev[axis]) / 4.0f;

                // Find the biggest sample deviation together with sample' sign
                if (ABS(mag.magADC[axis]) > ABS(magAxisDeviation[axis]))
                {
                    magAxisDeviation[axis] = mag.magADC[axis];
                }
            }

            // sqrtf(diffMag / avgMag) is a rough approximation of tangent of angle between magADC and magPrev. tan(8 deg) = 0.14
            if ((avgMag > 0.01f) && ((diffMag / avgMag) > (0.14f * 0.14f)))
            {
                sensorCalibrationPushSampleForOffsetCalculation(&calState, mag.magADC);

                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
                {
                    magPrev[axis] = mag.magADC[axis];
                }
            }

            if (_rotate_auto == 0)
            {
                update_ahrs_new_samples();
                update_compass_new_samples(mag.magADC);

                if (!allSamplesCollected)
                {
                    // Expect to get the minimum number of samples
                    if (_samples_collected == COMPASS_CAL_NUM_SAMPLES)
                    {
                        // Set initial offset to the average value of the samples
                        compass_offset.x = 0.0f;
                        compass_offset.y = 0.0f;
                        compass_offset.z = 0.0f;

                        for (uint16_t k = 0; k < _samples_collected; k++)
                        {
                            compass_offset.x -= compass_sample_buffer[k].x;
                            compass_offset.y -= compass_sample_buffer[k].y;
                            compass_offset.z -= compass_sample_buffer[k].z;
                        }

                        compass_offset.x /= _samples_collected;
                        compass_offset.y /= _samples_collected;
                        compass_offset.z /= _samples_collected;

                        allSamplesCollected = true;
                    }
                }
                else
                {
                    // This function is very slow
                    if (cmpTimeUs(currentTimeUs, autoOrientationtimestamp) >= MS2US(1000) && !autoOrientationPassed)
                    {
                        autoOrientationPassed = calculate_compass_auto_orientation();
                        autoOrientationtimestamp = currentTimeUs;
                    }
                }
            }
        }
        else
        {
            float magZerof[3];

            sensorCalibrationSolveForOffset(&calState, magZerof);

            if (autoOrientationPassed)
            {
                calculate_new_offset_compass(magZerof);
                compassConfigMutable()->mag_align = _orientation_solution;
            }

            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
            {
                compassConfigMutable()->magZero.raw[axis] = lrintf(magZerof[axis]);
            }

            /*
             * Scale calibration
             * We use max absolute value of each axis as scale calibration with constant 1024 as base
             * It is dirty, but worth checking if this will solve the problem of changing mag vector when UAV is tilted
             */
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
            {
                compassConfigMutable()->magGain[axis] = ABS(magAxisDeviation[axis] - compassConfig()->magZero.raw[axis]);
            }

            calStartedAt = 0;
            reset_compass_auto_orientation();
            saveConfigAndNotify();
        }
    }
    else
    {
        _orientation = mag.dev.magAlign.useExternal ? compassConfig()->mag_align : ALIGN_DEFAULT;

        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
        {
            mag.magADC[axis] = (mag.magADC[axis] - compassConfig()->magZero.raw[axis]) * 1024 / compassConfig()->magGain[axis];
        }
    }

    if (mag.dev.magAlign.useExternal)
    {
        const fpVector3_t v = {
            .x = mag.magADC[X],
            .y = mag.magADC[Y],
            .z = mag.magADC[Z],
        };

        fpVector3_t rotated;

        rotationMatrixRotateVector(&rotated, &v, &mag.dev.magAlign.externalRotation);

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

    magUpdatedAtLeastOnce = true;

    return compassDelay;
}

static void vector_rotate(fpVector3_t *vec, const sensor_align_e rotation)
{
    float vecCopy[3] = {vec->x, vec->y, vec->z};
    applySensorAlignment(vecCopy, vecCopy, rotation);
    vec->x = vecCopy[X];
    vec->y = vecCopy[Y];
    vec->z = vecCopy[Z];
}

static void multiplication_transpose_by_vector(const fpMatrix3_t mat, fpVector3_t *vec)
{
    const fpVector3_t v = {.x = vec->x, .y = vec->y, .z = vec->z};

    vec->x = mat.m[0][0] * v.x + mat.m[1][0] * v.y + mat.m[2][0] * v.z;
    vec->y = mat.m[0][1] * v.x + mat.m[1][1] * v.y + mat.m[2][1] * v.z;
    vec->z = mat.m[0][2] * v.x + mat.m[1][2] * v.y + mat.m[2][2] * v.z;
}

static void vector_rotate_inverse(fpVector3_t *vec, const sensor_align_e rotation)
{
    fpVector3_t x_vec = {.x = 1.0f, .y = 0.0f, .z = 0.0f};
    fpVector3_t y_vec = {.x = 0.0f, .y = 1.0f, .z = 0.0f};
    fpVector3_t z_vec = {.x = 0.0f, .y = 0.0f, .z = 1.0f};

    vector_rotate(&x_vec, rotation);
    vector_rotate(&y_vec, rotation);
    vector_rotate(&z_vec, rotation);

    fpMatrix3_t vecToMatrix;

    vecToMatrix.m[0][0] = x_vec.x;
    vecToMatrix.m[0][1] = x_vec.y;
    vecToMatrix.m[0][2] = x_vec.z;

    vecToMatrix.m[1][0] = y_vec.x;
    vecToMatrix.m[1][1] = y_vec.y;
    vecToMatrix.m[1][2] = y_vec.z;

    vecToMatrix.m[2][0] = z_vec.x;
    vecToMatrix.m[2][1] = z_vec.y;
    vecToMatrix.m[2][2] = z_vec.z;

    multiplication_transpose_by_vector(vecToMatrix, vec);
}

void update_ahrs_new_samples(void)
{
    roll_from_att = constrain_int16(127 * (atan2_approx(rotationMatrix.m[2][1], rotationMatrix.m[2][2]) / M_PI), -127, 127);
    pitch_from_att = constrain_int16(127 * (((0.5f * M_PIf) - acos_approx(-rotationMatrix.m[2][0])) / M_PI_2), -127, 127);
    yaw_from_att = constrain_int16(127 * (-atan2_approx(rotationMatrix.m[1][0], rotationMatrix.m[0][0]) / M_PI), -127, 127);
}

static fpMatrix3_t get_rotationMatrix_from_ahrs(void)
{
    fpMatrix3_t ahrsMatrix;

    fp_angles_t euler_rad = {
        .angles.roll = roll_from_att * (M_PI / 127),
        .angles.pitch = pitch_from_att * (M_PI_2 / 127),
        .angles.yaw = yaw_from_att * (M_PI / 127),
    };

    rotationMatrixFromEulerAngles(&ahrsMatrix, &euler_rad);

    return ahrsMatrix;
}

/*
  Calculate the implied earth field for a compass sample and compass rotation. This is used to check for consistency between samples.
  If the orientation is correct then when rotated the same (or similar) earth field should be given for all samples.
  Note that this earth field uses an arbitrary north reference, so it may not match the true earth field.
 */
static fpVector3_t calculate_compass_earth_field(const fpVector3_t sample, const sensor_align_e r)
{
    fpVector3_t vec_sample = {.x = sample.x, .y = sample.y, .z = sample.z};

    // Convert the sample back to sensor frame
    vector_rotate_inverse(&vec_sample, _orientation);

    // Rotate to body frame for this rotation
    vector_rotate(&vec_sample, r);

    // Apply offsets, rotating them for the orientation we are testing
    fpVector3_t rot_offsets = {.x = compass_offset.x, .y = compass_offset.y, .z = compass_offset.z};

    vector_rotate_inverse(&rot_offsets, _orientation);
    vector_rotate(&rot_offsets, r);

    vec_sample.x += rot_offsets.x;
    vec_sample.y += rot_offsets.y;
    vec_sample.z += rot_offsets.z;

    // Rotate the sample from body frame back to earth frame
    const fpMatrix3_t rot = get_rotationMatrix_from_ahrs();

    const fpVector3_t efield = multiply_matrix_by_vector(rot, vec_sample);

    // Earth field is the mag sample in earth frame
    return efield;
}

static bool verify_for_right_angle_rotation(const sensor_align_e r)
{
    switch (r)
    {
    case ALIGN_DEFAULT:
    case CW0_DEG:
    case CW90_DEG:
    case CW180_DEG:
    case CW270_DEG:
    case CW0_DEG_FLIP:
    case CW90_DEG_FLIP:
    case CW180_DEG_FLIP:
    case CW270_DEG_FLIP:
        return true;

    default:
        return false;
    }
}

void update_compass_new_samples(const float sample[3])
{
    if (_samples_collected < COMPASS_CAL_NUM_SAMPLES)
    {
        compass_sample_buffer[_samples_collected].x = sample[X];
        compass_sample_buffer[_samples_collected].y = sample[Y];
        compass_sample_buffer[_samples_collected].z = sample[Z];
        _samples_collected++;
    }
}

void reset_compass_auto_orientation(void)
{
    _samples_collected = 0;
    allSamplesCollected = false;
    compass_offset.x = 0;
    compass_offset.y = 0;
    compass_offset.z = 0;
    memset(&compass_sample_buffer, 0, sizeof(compass_sample_buffer));
}

// Calculate compass orientation using the attitude estimate associated with each sample, and fix orientation on external compasses if the feature is enabled
bool calculate_compass_auto_orientation(void)
{
    const bool fix_orientation = _rotate_auto >= 2; // True if orientation should be fixed if necessary
    const bool always_45_deg = _rotate_auto >= 3;   // True if orentation should considder 45deg with equal tolerance

    float variance[ALIGN_ROTATION_MAX];

    _orientation_solution = _orientation;

    for (uint8_t n = 0; n < ARRAYLEN(variance); n++)
    {
        sensor_align_e r = (sensor_align_e)n;

        // Calculate the average implied earth field across all samples
        fpVector3_t total_ef = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

        for (uint16_t i = 0; i < _samples_collected; i++)
        {
            const fpVector3_t efield = calculate_compass_earth_field(compass_sample_buffer[i], r);
            total_ef.x += efield.x;
            total_ef.y += efield.y;
            total_ef.z += efield.z;
        }

        fpVector3_t avg_efield = {.x = 0.0f, .y = 0.0f, .z = 0.0f};
        avg_efield.x = total_ef.x / _samples_collected;
        avg_efield.y = total_ef.y / _samples_collected;
        avg_efield.z = total_ef.z / _samples_collected;

        // Now calculate the square error for this rotation against the average earth field
        for (uint16_t i = 0; i < _samples_collected; i++)
        {
            const fpVector3_t efield = calculate_compass_earth_field(compass_sample_buffer[i], r);
            float err = sq(efield.x - avg_efield.x) + sq(efield.y - avg_efield.y) + sq(efield.z - avg_efield.z);
            // Divide by number of samples collected to get the variance
            variance[n] += err / _samples_collected;
        }
    }

    // Find the rotation with the lowest variance
    sensor_align_e besti = ALIGN_DEFAULT;
    float bestv = variance[0];
    sensor_align_e besti_90 = ALIGN_DEFAULT;
    float bestv_90 = variance[0];

    for (uint8_t n = 0; n < ARRAYLEN(variance); n++)
    {
        sensor_align_e r = (sensor_align_e)n;
        if (variance[n] < bestv)
        {
            bestv = variance[n];
            besti = r;
        }

        if (verify_for_right_angle_rotation(r) && variance[n] < bestv_90)
        {
            bestv_90 = variance[n];
            besti_90 = r;
        }
    }

    float second_best = besti == ALIGN_DEFAULT ? variance[1] : variance[0];
    float second_best_90 = besti_90 == ALIGN_DEFAULT ? variance[2] : variance[0];

    for (uint8_t n = 0; n < ARRAYLEN(variance); n++)
    {
        sensor_align_e r = (sensor_align_e)n;
        if (besti != r)
        {
            if (variance[n] < second_best)
            {
                second_best = variance[n];
            }
        }

        if (verify_for_right_angle_rotation(r) && (besti_90 != r) && (variance[n] < second_best_90))
        {
            second_best_90 = variance[n];
        }
    }

    _orientation_confidence = second_best / bestv;

    bool pass;

    if (besti == _orientation)
    {
        // If the orientation matched then allow for a low threshold
        pass = true;
    }
    else
    {
        if (_orientation_confidence > 4.0f)
        {
            // Very confident, always pass
            pass = true;
        }
        else if (always_45_deg && (_orientation_confidence > 2.0f))
        {
            // Use same tolerance for all rotations
            pass = true;
        }
        else
        {
            // Just consider 90's
            _orientation_confidence = second_best_90 / bestv_90;
            pass = _orientation_confidence > 2.0f;
            besti = besti_90;
        }
    }

    if (!pass)
    {
        // Serial.print("bad orientation: ");
        // Serial.println(besti);
    }
    else if (besti == _orientation)
    {
        // no orientation change
        // Serial.print("good orientation: ");
        // Serial.print(besti);
        // Serial.println(" not necessary change");
    }
    else if (!mag.dev.magAlign.useExternal || !fix_orientation)
    {
        // Serial.print("internal bad orientation:");
        // Serial.println(besti);
    }
    else
    {
        // Serial.print("prev orientation: ");
        // Serial.print(_orientation);
        // Serial.print(" new orientation:");
        // Serial.println(besti);
    }

    if (!pass)
    {
        // Serial.print("BAD ORIENTATION Line:");
        // Serial.println(__LINE__);
        return false;
    }

    if (_orientation == besti)
    {
        // No orientation change
        return true;
    }

    if (!mag.dev.magAlign.useExternal || !fix_orientation)
    {
        // We won't change the orientation, but we set _orientation for reporting purposes
        _orientation = besti;
        _orientation_solution = besti;
        // Serial.print("BAD ORIENTATION Line:");
        // Serial.println(__LINE__);
        return false;
    }

    // Rotate the samples for the new orientation
    for (uint16_t i = 0; i < _samples_collected; i++)
    {
        fpVector3_t rotSamples = {.x = compass_sample_buffer[i].x, .y = compass_sample_buffer[i].y, .z = compass_sample_buffer[i].z};
        vector_rotate_inverse(&rotSamples, _orientation);
        vector_rotate(&rotSamples, besti);
        compass_sample_buffer[i].x = rotSamples.x;
        compass_sample_buffer[i].y = rotSamples.y;
        compass_sample_buffer[i].z = rotSamples.z;
    }

    _orientation = besti;
    _orientation_solution = besti;

    return true;
}

void calculate_new_offset_compass(float compassOffSet[3])
{
    // Correct the offsets for the new orientation
    fpVector3_t rot_offsets = {.x = compassOffSet[X], .y = compassOffSet[Y], .z = compassOffSet[Z]};
    vector_rotate_inverse(&rot_offsets, mag.dev.magAlign.onBoard);
    vector_rotate(&rot_offsets, _orientation_solution);
    compassOffSet[X] = rot_offsets.x;
    compassOffSet[Y] = rot_offsets.y;
    compassOffSet[Z] = rot_offsets.z;
}

#endif
