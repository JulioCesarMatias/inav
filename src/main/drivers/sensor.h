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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_types.h"
#include "drivers/bus.h"

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG,
    CW90_DEG,
    CW180_DEG,
    CW270_DEG,
    CW0_DEG_FLIP,
    CW90_DEG_FLIP,
    CW180_DEG_FLIP,
    CW270_DEG_FLIP,
    ALIGN_ROTATION_MAX
} sensor_align_e;

typedef bool (*sensorInitFuncPtr)(void);                    // sensor init prototype
typedef bool (*sensorReadFuncPtr)(int16_t *data);           // sensor read and align prototype
typedef bool (*sensorInterruptFuncPtr)(void);
struct accDev_s;
typedef void (*sensorAccInitFuncPtr)(struct accDev_s *acc);
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);
struct gyroDev_s;
typedef void (*sensorGyroInitFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroUpdateFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadDataFuncPtr)(struct gyroDev_s *gyro, int16_t *data);
typedef bool (*sensorGyroInterruptStatusFuncPtr)(struct gyroDev_s *gyro);
struct magDev_s;
typedef bool (*sensorMagInitFuncPtr)(struct magDev_s *mag);
typedef bool (*sensorMagReadFuncPtr)(struct magDev_s *mag);
struct temperatureDev_s;
typedef bool (*sensorTempReadFuncPtr)(struct temperatureDev_s *tempDev, int16_t *temperature);
struct opflowDev_s;
typedef bool (*sensorOpflowInitFuncPtr)(struct opflowDev_s *mag);
typedef bool (*sensorOpflowUpdateFuncPtr)(struct opflowDev_s *mag);
