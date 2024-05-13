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
#include "ekf/ekfIntegrator.h"

#define DT_MIN 1e-6f
#define DT_MAX (UINT16_MAX * 1e-6f)

void integrator_put(imuIntegrator *integrator, fpVector3_t val, float dt)
{
    if ((dt > DT_MIN) && (integrator->integral_dt + dt < DT_MAX))
    {
        // Use trapezoidal integration to calculate the delta integral
        fpVector3_t delta_alpha;
        delta_alpha.x = (val.x + integrator->last_val.x) * dt * 0.5f;
        delta_alpha.y = (val.y + integrator->last_val.y) * dt * 0.5f;
        delta_alpha.z = (val.z + integrator->last_val.z) * dt * 0.5f;

        integrator->last_val = val;

        integrator->alpha.x += delta_alpha.x;
        integrator->alpha.y += delta_alpha.y;
        integrator->alpha.z += delta_alpha.z;

        integrator->integrated_samples++;
        integrator->integral_dt += dt;
    }
    else
    {
        integrator_reset(integrator);
        integrator->last_val = val;
    }
}

void integrator_reset(imuIntegrator *integrator)
{
    integrator->alpha.x = 0.0f;
    integrator->alpha.y = 0.0f;
    integrator->alpha.z = 0.0f;
    integrator->last_val.x = 0.0f;
    integrator->last_val.y = 0.0f;
    integrator->last_val.z = 0.0f;
    integrator->integral_dt = 0.0f;
    integrator->integrated_samples = 0;
}

bool integrator_ready(imuIntegrator *integrator)
{
    return (integrator->integrated_samples >= integrator->reset_samples_min) || (integrator->integral_dt >= integrator->reset_interval_min);
}

bool integrator_reset_and_get_integral(imuIntegrator *integrator, fpVector3_t *integral, float *integral_dt)
{
    if (integrator_ready(integrator))
    {
        *integral = integrator->alpha;
        *integral_dt = integrator->integral_dt;
        integrator_reset(integrator);
        return true;
    }

    return false;
}

void integrator_coning_put(imuIntegratorConing *integrator, fpVector3_t val, float dt)
{
    if ((dt > DT_MIN) && (integrator->base.integral_dt + dt < DT_MAX))
    {
        // Use trapezoidal integration to calculate the delta integral
        fpVector3_t delta_alpha;
        delta_alpha.x = (val.x + integrator->base.last_val.x) * dt * 0.5f;
        delta_alpha.y = (val.y + integrator->base.last_val.y) * dt * 0.5f;
        delta_alpha.z = (val.z + integrator->base.last_val.z) * dt * 0.5f;

        integrator->base.last_val = val;

        // Calculate coning corrections
        integrator->beta.x = (integrator->last_alpha.y + integrator->last_delta_alpha.y * (1.0f / 6.0f)) * delta_alpha.z - (integrator->last_alpha.z + integrator->last_delta_alpha.z * (1.0f / 6.0f)) * delta_alpha.y;
        integrator->beta.y = -(integrator->last_alpha.x + integrator->last_delta_alpha.x * (1.0f / 6.0f)) * delta_alpha.z + (integrator->last_alpha.z + integrator->last_delta_alpha.z * (1.0f / 6.0f)) * delta_alpha.x;
        integrator->beta.z = (integrator->last_alpha.x + integrator->last_delta_alpha.x * (1.0f / 6.0f)) * delta_alpha.y - (integrator->last_alpha.y + integrator->last_delta_alpha.y * (1.0f / 6.0f)) * delta_alpha.x;
        integrator->beta.x *= 0.5f;
        integrator->beta.y *= 0.5f;
        integrator->beta.z *= 0.5f;

        integrator->last_delta_alpha = delta_alpha;
        integrator->last_alpha = integrator->base.alpha;

        // Accumulate delta integrals
        integrator->base.alpha.x += delta_alpha.x;
        integrator->base.alpha.y += delta_alpha.y;
        integrator->base.alpha.z += delta_alpha.z;

        integrator->base.integrated_samples++;
        integrator->base.integral_dt += dt;
    }
    else
    {
        integrator_coning_reset(integrator);
        integrator->base.last_val = val;
    }
}

void integrator_coning_reset(imuIntegratorConing *integrator)
{
    integrator_reset(&integrator->base);
    integrator->beta.x = 0.0f;
    integrator->beta.y = 0.0f;
    integrator->beta.z = 0.0f;
    integrator->last_alpha.x = 0.0f;
    integrator->last_alpha.y = 0.0f;
    integrator->last_alpha.z = 0.0f;
}

bool integrator_coning_reset_and_get_integral(imuIntegratorConing *integrator, fpVector3_t *integral, float *integral_dt)
{
    if (integrator_ready(&integrator->base))
    {
        // Apply coning corrections
        integral->x = integrator->base.alpha.x + integrator->beta.x;
        integral->y = integrator->base.alpha.y + integrator->beta.y;
        integral->z = integrator->base.alpha.z + integrator->beta.z;
        *integral_dt = integrator->base.integral_dt;
        integrator_coning_reset(integrator);
        return true;
    }

    return false;
}