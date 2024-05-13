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
#include "common/vector.h"

typedef struct
{
    fpVector3_t alpha;        // integrated value before coning corrections are applied
    fpVector3_t last_val;     // previous input
    float reset_interval_min; // the interval after which the content will be published and the integrator reset
    float integral_dt;
    uint8_t reset_samples_min;
    uint8_t integrated_samples;
} imuIntegrator;

typedef struct
{
    imuIntegrator base;
    fpVector3_t beta;             // accumulated coning corrections
    fpVector3_t last_delta_alpha; // integral from previous previous sampling interval
    fpVector3_t last_alpha;       // previous value of alpha
} imuIntegratorConing;

void integrator_put(imuIntegrator *integrator, fpVector3_t val, float dt);
void integrator_reset(imuIntegrator *integrator);
bool integrator_ready(imuIntegrator *integrator);
bool integrator_reset_and_get_integral(imuIntegrator *integrator, fpVector3_t *integral, float *integral_dt);

void integrator_coning_put(imuIntegratorConing *integrator, fpVector3_t val, float dt);
void integrator_coning_reset(imuIntegratorConing *integrator);
bool integrator_coning_reset_and_get_integral(imuIntegratorConing *integrator, fpVector3_t *integral, float *integral_dt);