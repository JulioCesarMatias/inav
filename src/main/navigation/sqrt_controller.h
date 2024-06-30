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

typedef enum {
    SQRT_CONTROLLER_POS_XY = 1 << 0,
    SQRT_CONTROLLER_POS_Z  = 1 << 1,
} sqrtControllerFlags_e;

typedef struct { 
    float kp;             // proportional gain
    float error;          // error calced
    float error_min;      // error limit in negative direction
    float error_max;      // error limit in positive direction
    float derivative_max; // maximum derivative of output
} sqrt_controller_t;

void sqrtControllerInit(sqrt_controller_t *sqrt_controller_pointer, const float kp, const float output_min, const float output_max, const float derivative_out_max, sqrtControllerFlags_e sqrtControllerFlags);
float sqrtControllerApply(sqrt_controller_t *sqrt_controller_pointer, float target, float measurement, float deltaTime, sqrtControllerFlags_e sqrtControllerFlags);
float sqrtControllerInverse(float kp, float derivative_max, float output);