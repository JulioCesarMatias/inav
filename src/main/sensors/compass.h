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

#pragma once

#include "common/axis.h"
#include "common/time.h"

#include "config/parameter_group.h"

#include "drivers/compass/compass.h"

#include "sensors/sensors.h"

// Type of magnetometer used/detected
typedef enum
{
    MAG_NONE = 0,
    MAG_AUTODETECT,
    MAG_HMC5883,
    MAG_AK8975,
    MAG_MAG3110,
    MAG_AK8963,
    MAG_IST8310,
    MAG_QMC5883,
    MAG_MPU9250,
    MAG_IST8308,
    MAG_LIS3MDL,
    MAG_MSP,
    MAG_RM3100,
    MAG_VCM5883,
    MAG_MLX90393,
    MAG_FAKE,
    MAG_MAX = MAG_FAKE
} magSensor_e;

typedef enum
{
    MAG_INTERNAL = 0,
    MAG_EXTERNAL
} mag_external_e;

typedef enum
{
    MAG_CALIBRATION_USING_SAMPLES = 0,
    MAG_CALIBRATION_USING_GPS
} magCalibrationType_e;

typedef struct mag_s
{
    magDev_t dev;
    float magADC[XYZ_AXIS_COUNT];
    timeUs_t time;
    timeUs_t lastUpdateUs;
} mag_t;

extern mag_t mag;

#ifdef USE_MAG

typedef struct compassConfig_s
{
    int16_t mag_declination;  // Get your magnetic declination from here : http://magnetic-declination.com/  For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    sensor_align_e mag_align; // on-board mag alignment. Ignored if externally aligned via *DeciDegrees.
    uint8_t mag_hardware;     // Which mag hardware to use on boards with more than one device
    uint8_t mag_external;
    uint8_t mag_auto_rotate;
    uint8_t mag_cal_type;
    fpVector3_t offSet;
    float scaleFactor;
    fpVector3_t diagonals;
    fpVector3_t offDiagonals;
#ifdef USE_DUAL_MAG
    uint8_t mag_to_use;
#endif
} compassConfig_t;

PG_DECLARE(compassConfig_t, compassConfig);

bool compassDetect(magDev_t *dev, magSensor_e magHardwareToUse);
bool compassInit(void);
void compassUpdate(timeUs_t currentTimeUs);
bool compassIsHealthy(void);
bool compassIsCalibrationComplete(void);

#endif
