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

#pragma once

#include <stdbool.h>
#include <stdint.h>

// copter defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       1.0f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-06f
#define ABIAS_PNOISE_DEFAULT    0.0001f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         1
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15

/*
// rover defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       1.0f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-06f
#define ABIAS_PNOISE_DEFAULT    0.0002f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         1
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15

// generic defaults (and for plane)
#define VELNE_NOISE_DEFAULT     0.3f
#define VELD_NOISE_DEFAULT      0.5f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       0.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-06f
#define ABIAS_PNOISE_DEFAULT    0.0002f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        20
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         0
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15
*/

void ekf_update(float deltaTime);