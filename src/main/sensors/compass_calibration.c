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

/*
 * The intention of a magnetometer in a compass application is to measure
 * Earth's magnetic field. Measurements other than those of Earth's magnetic
 * field are considered errors. This algorithm computes a set of correction
 * parameters that null out errors from various sources:
 *
 * - Sensor bias error
 * - "Hard iron" error caused by materials fixed to the vehicle body that
 *     produce static magnetic fields.
 * - Sensor scale-factor error
 * - Sensor cross-axis sensitivity
 * - "Soft iron" error caused by materials fixed to the vehicle body that
 *     distort magnetic fields.
 *
 * This is done by taking a set of samples that are assumed to be the product
 * of rotation in earth's magnetic field and fitting an offset ellipsoid to
 * them, determining the correction to be applied to adjust the samples into an
 * origin-centered sphere.
 *
 * The state machine of this library is described entirely by the
 * CompassCalibrator::Status enum, and all state transitions are managed by the
 * set_status function. Normally, the library is in the NOT_STARTED state. When
 * the start function is called, the state transitions to WAITING_TO_START,
 * until two conditions are met: the delay as elapsed, and the memory for the
 * sample buffer has been successfully allocated.
 * Once these conditions are met, the state transitions to RUNNING_STEP_ONE, and
 * samples are collected via calls to the new_sample function. These samples are
 * accepted or rejected based on distance to the nearest sample. The samples are
 * assumed to cover the surface of a sphere, and the radius of that sphere is
 * initialized to a conservative value. Based on a circle-packing pattern, the
 * minimum distance is set such that some percentage of the surface of that
 * sphere must be covered by samples.
 *
 * Once the sample buffer is full, a sphere fitting algorithm is run, which
 * computes a new sphere radius. The sample buffer is thinned of samples which
 * no longer meet the acceptance criteria, and the state transitions to
 * RUNNING_STEP_TWO. Samples continue to be collected until the buffer is full
 * again, the full ellipsoid fit is run, and the state transitions to either
 * SUCCESS or FAILED.
 *
 * The fitting algorithm used is Levenberg-Marquardt. See also:
 * http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
 */

// Ported from ArduPilot to INAV by Julio Cesar Matias

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/imu.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "sensors/boardalignment.h"
#include "sensors/compass_calibration.h"

State cal_state;
Report cal_report;
Settings cal_settings;

status_e _status; // current state of calibrator

// values provided by caller
float _delay_start_sec;  // seconds to delay start of calibration (provided by caller)
float _tolerance = 5.0f; // worst acceptable RMS tolerance (aka fitness).
uint16_t _offset_max;    // maximum acceptable offsets (provided by caller)

// behavioral state
uint32_t _start_time_us;       // system time start() function was last called
uint8_t _attempt;              // number of attempts have been made to calibrate
CompassSample *_sample_buffer; // buffer of sensor values
uint16_t _samples_collected;   // number of samples in buffer
uint16_t _samples_thinned;     // number of samples removed by the thin_samples() call (called before step 2 begins)

// fit state
param_t _params;         // latest calibration outputs
float _fitness;          // fitness (mean squared residuals) of current parameters
float _initial_fitness;  // fitness before latest "fit" was attempted (used to determine if fit was an improvement)
float _sphere_lambda;    // sphere fit's lambda
float _ellipsoid_lambda; // ellipsoid fit's lambda

// variables for orientation checking
sensor_align_e _orientation;          // latest detected orientation
sensor_align_e _orig_orientation;     // original orientation provided by caller
sensor_align_e _orientation_solution; // latest solution
bool _is_external;                    // true if compass is external (provided by caller)
bool _check_orientation;              // true if orientation should be automatically checked
bool _fix_orientation;                // true if orientation should be fixed if necessary
float _orientation_confidence;        // measure of confidence in automatic orientation detection
CompassSample _last_sample;

status_e _requested_status;
bool _status_set_requested;
bool _new_sample;

static float calc_mean_squared_residuals(const param_t params);
static bool set_status(status_e status);

static bool _running(void)
{
    return _status == RUNNING_STEP_ONE || _status == RUNNING_STEP_TWO;
}

static bool _fitting(void)
{
    return _running() && (_samples_collected == COMPASS_CAL_NUM_SAMPLES);
}

void compassCalibrationStart(float delay, uint16_t offset_max, float tolerance)
{
    // Don't do this while we are already started
    if (_running())
    {
        return;
    }

    set_status(NOT_STARTED);

    cal_settings.offset_max = offset_max;
    cal_settings.attempt = 1;
    cal_settings.delay_start_sec = delay;
    cal_settings.start_time_ms = micros();
    cal_settings.tolerance = tolerance;

    // Request status change to Waiting to start
    _requested_status = WAITING_TO_START;
    _status_set_requested = true;
}

// Request to cancel calibration
void compassCalibrationStop(void)
{
    _requested_status = NOT_STARTED;
    _status_set_requested = true;
}

// Record point mag sample and associated attitude sample to intermediate struct
void compassCalibrationSetNewSample(const fpVector3_t sample)
{
    _last_sample.x = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(sample.x);
    _last_sample.y = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(sample.y);
    _last_sample.z = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(sample.z);
    _last_sample.att.roll = constrain_int16(127 * (atan2_approx(rotationMatrix.m[2][1], rotationMatrix.m[2][2]) / M_PI), -127, 127);
    _last_sample.att.pitch = constrain_int16(127 * (((0.5f * M_PIf) - acos_approx(-rotationMatrix.m[2][0])) / M_PI_2), -127, 127);
    _last_sample.att.yaw = constrain_int16(127 * (-atan2_approx(rotationMatrix.m[1][0], rotationMatrix.m[0][0]) / M_PI), -127, 127);
    _new_sample = true;
}

void compassCalibrationSetOrientation(sensor_align_e orientation, bool is_external, bool fix_orientation)
{
    cal_settings.check_orientation = true;
    cal_settings.orientation = orientation;
    cal_settings.orig_orientation = orientation;
    cal_settings.is_external = is_external;
    cal_settings.fix_orientation = fix_orientation;
}

bool compassIsCalibrating(void)
{
    switch (cal_state.status)
    {
    case NOT_STARTED:
    case COMPASS_CAL_SUCCESS:
    case COMPASS_CAL_FAILED:
    case BAD_ORIENTATION:
    case BAD_RADIUS:
        return false;

    case WAITING_TO_START:
    case RUNNING_STEP_ONE:
    case RUNNING_STEP_TWO:
        return true;
    }

    return false;
}

/*
 * The sample acceptance distance is determined as follows:
 * For any regular polyhedron with triangular faces, the angle theta subtended
 * by two closest points is defined as
 *
 *      theta = arccos(cos(A)/(1-cos(A)))
 *
 * Where:
 *      A = (4pi/F + pi)/3
 * and
 *      F = 2V - 4 is the number of faces for the polyhedron in consideration,
 *      which depends on the number of vertices V
 *
 * The above equation was proved after solving for spherical triangular excess
 * and related equations.
 */
static bool accept_sample(const CompassSample sample, uint16_t skip_index)
{
    const uint16_t faces = (2 * COMPASS_CAL_NUM_SAMPLES - 4);
    const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    const float theta = 0.5f * acos_approx(cos_approx(a) / (1.0f - cos_approx(a)));

    if (_sample_buffer == NULL)
    {
        return false;
    }

    const float min_distance = _params.radius * 2.0f * sin_approx(theta / 2.0f);

    for (uint16_t i = 0; i < _samples_collected; i++)
    {
        if (i != skip_index)
        {
            const float distance = calc_length_pythagorean_3D(COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample.x) - COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].x),
                                                              COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample.y) - COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].y),
                                                              COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample.z) - COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].z));
            if (distance < min_distance)
            {
                return false;
            }
        }
    }

    return true;
}

static void thin_samples(void)
{
    if (_sample_buffer == NULL)
    {
        return;
    }

    _samples_thinned = 0;

    // Shuffle the samples http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
    // This is so that adjacent samples don't get sequentially eliminated
    for (uint16_t i = _samples_collected - 1; i >= 1; i--)
    {
        uint16_t j = get_random16() % (i + 1);
        CompassSample temp = _sample_buffer[i];
        _sample_buffer[i] = _sample_buffer[j];
        _sample_buffer[j] = temp;
    }

    // Remove any samples that are close together
    for (uint16_t i = 0; i < _samples_collected; i++)
    {
        if (!accept_sample(_sample_buffer[i], i))
        {
            _sample_buffer[i] = _sample_buffer[_samples_collected - 1];
            _samples_collected--;
            _samples_thinned++;
        }
    }
}

static void pull_sample(void)
{
    CompassSample mag_sample;

    if (!_new_sample)
    {
        return;
    }

    if (_status == WAITING_TO_START)
    {
        set_status(RUNNING_STEP_ONE);
    }

    _new_sample = false;
    mag_sample = _last_sample;

    if (_running() && _samples_collected < COMPASS_CAL_NUM_SAMPLES && accept_sample(mag_sample, 65535U))
    {
        _sample_buffer[_samples_collected] = mag_sample;
        _samples_collected++;
    }
}

static void update_cal_settings(void)
{
    _tolerance = cal_settings.tolerance;
    _check_orientation = cal_settings.check_orientation;
    _orientation = cal_settings.orientation;
    _orig_orientation = cal_settings.orig_orientation;
    _is_external = cal_settings.is_external;
    _fix_orientation = cal_settings.fix_orientation;
    _offset_max = cal_settings.offset_max;
    _attempt = cal_settings.attempt;
    _delay_start_sec = cal_settings.delay_start_sec;
    _start_time_us = cal_settings.start_time_ms;
}

static void update_cal_status(void)
{
    cal_state.status = _status;
    cal_state.attempt = _attempt;
    cal_state.completion_pct = 0.0f;

    // First sampling step is 1/3rd of the progress bar
    // Never return more than 99% unless _status is COMPASS_CAL_SUCCESS
    switch (_status)
    {
    case NOT_STARTED:
    case WAITING_TO_START:
        cal_state.completion_pct = 0.0f;
        break;

    case RUNNING_STEP_ONE:
        cal_state.completion_pct = 33.3f * _samples_collected / COMPASS_CAL_NUM_SAMPLES;
        break;

    case RUNNING_STEP_TWO:
        cal_state.completion_pct = 33.3f + 65.7f * ((float)(_samples_collected - _samples_thinned) / (COMPASS_CAL_NUM_SAMPLES - _samples_thinned));
        break;

    case COMPASS_CAL_SUCCESS:
        cal_state.completion_pct = 100.0f;
        break;

    case COMPASS_CAL_FAILED:
    case BAD_ORIENTATION:
    case BAD_RADIUS:
        cal_state.completion_pct = 0.0f;
        break;
    };
}

static void update_cal_report(void)
{
    cal_report.status = _status;
    cal_report.fitness = sqrtf(_fitness);
    cal_report.ofs = _params.offset;
    cal_report.diag = _params.diag;
    cal_report.offdiag = _params.offdiag;
    cal_report.scale_factor = _params.scale_factor;
    cal_report.orientation_confidence = _orientation_confidence;
    cal_report.original_orientation = _orig_orientation;
    cal_report.orientation = _orientation_solution;
    cal_report.check_orientation = _check_orientation;
}

Report getCompassCalibrationReport(void)
{
    return cal_report;
}

State getCompassCalibrationState(void)
{
    return cal_state;
}

void setCompassCalibrationFinished(bool state)
{
    cal_state.calibration_finished = state;
}

bool getCompassCalibrationFinished(void)
{
    return cal_state.calibration_finished;
}

// Initialize fitness before starting a fit
static void initialize_fit(void)
{
    if (_samples_collected != 0)
    {
        _fitness = calc_mean_squared_residuals(_params);
    }
    else
    {
        _fitness = 1.0e30f;
    }

    _initial_fitness = _fitness;
    _sphere_lambda = 1.0f;
    _ellipsoid_lambda = 1.0f;
    cal_state.fit_step = 0;
}

static void reset_state(void)
{
    _samples_collected = 0;
    _samples_thinned = 0;
    _params.radius = 200.0f;
    vectorZero(&_params.offset);
    _params.diag.x = 1.0f;
    _params.diag.y = 1.0f;
    _params.diag.z = 1.0f;
    vectorZero(&_params.offdiag);
    _params.scale_factor = 0.0f;
    initialize_fit();
}

static bool set_status(status_e status)
{
    if (status != NOT_STARTED && _status == status)
    {
        return true;
    }

    switch (status)
    {
    case NOT_STARTED:
        reset_state();
        _status = NOT_STARTED;
        if (_sample_buffer != NULL)
        {
            free(_sample_buffer);
            _sample_buffer = NULL;
        }
        return true;

    case WAITING_TO_START:
        reset_state();
        _status = WAITING_TO_START;
        set_status(RUNNING_STEP_ONE);
        return true;

    case RUNNING_STEP_ONE:
        if (_status != WAITING_TO_START)
        {
            return false;
        }

        // On first attempt delay start if requested by caller
        if (_attempt == 1 && (micros() - _start_time_us) < MS2US(_delay_start_sec))
        {
            return false;
        }

        if (_sample_buffer == NULL)
        {
            _sample_buffer = (CompassSample *)calloc(COMPASS_CAL_NUM_SAMPLES, sizeof(CompassSample));
        }

        if (_sample_buffer != NULL)
        {
            initialize_fit();
            _status = RUNNING_STEP_ONE;
            return true;
        }
        return false;

    case RUNNING_STEP_TWO:
        if (_status != RUNNING_STEP_ONE)
        {
            return false;
        }
        thin_samples();
        initialize_fit();
        _status = RUNNING_STEP_TWO;
        return true;

    case COMPASS_CAL_SUCCESS:
        if (_status != RUNNING_STEP_TWO)
        {
            return false;
        }

        if (_sample_buffer != NULL)
        {
            free(_sample_buffer);
            _sample_buffer = NULL;
        }

        _status = COMPASS_CAL_SUCCESS;
        return true;

    case COMPASS_CAL_FAILED:
        if (_status == BAD_ORIENTATION || _status == BAD_RADIUS)
        {
            // Don't overwrite bad orientation status
            return false;
        }
        FALLTHROUGH;

    case BAD_ORIENTATION:
    case BAD_RADIUS:
        if (_status == NOT_STARTED)
        {
            return false;
        }

        if (set_status(WAITING_TO_START))
        {
            _attempt++;
            return true;
        }

        if (_sample_buffer != NULL)
        {
            free(_sample_buffer);
            _sample_buffer = NULL;
        }

        _status = status;
        return true;

    default:
        return false;
    };
}

static bool fit_acceptable(void)
{
    if (!isnan(_fitness) &&
        _params.radius > FIELD_RADIUS_MIN && _params.radius < FIELD_RADIUS_MAX &&
        fabsf(_params.offset.x) < _offset_max &&
        fabsf(_params.offset.y) < _offset_max &&
        fabsf(_params.offset.z) < _offset_max &&
        _params.diag.x > 0.2f && _params.diag.x < 5.0f &&
        _params.diag.y > 0.2f && _params.diag.y < 5.0f &&
        _params.diag.z > 0.2f && _params.diag.z < 5.0f &&
        fabsf(_params.offdiag.x) < 1.0f && // Absolute of sine/cosine output cannot be greater than 1
        fabsf(_params.offdiag.y) < 1.0f &&
        fabsf(_params.offdiag.z) < 1.0f)
    {
        return _fitness <= sq(_tolerance);
    }

    return false;
}

static float calc_residual(const fpVector3_t sample, const param_t params)
{
    fpMatrix3_t softiron = initMatrixUsingVector(params.diag.x, params.offdiag.x, params.offdiag.y,
                                                 params.offdiag.x, params.diag.y, params.offdiag.z,
                                                 params.offdiag.y, params.offdiag.z, params.diag.z);

    fpVector3_t sampleSumByOffSet = {.v = {sample.x + params.offset.x, sample.y + params.offset.y, sample.z + params.offset.z}};
    fpVector3_t softIronBySample = multiplyMatrixByVector(softiron, sampleSumByOffSet);

    return params.radius - calc_length_pythagorean_3D(softIronBySample.x, softIronBySample.y, softIronBySample.z);
}

// Calc the fitness given a set of parameters (offsets, diagonals, off diagonals)
static float calc_mean_squared_residuals(const param_t params)
{
    if (_sample_buffer == NULL || _samples_collected == 0)
    {
        return 1.0e30f;
    }

    float sum = 0.0f;

    for (uint16_t i = 0; i < _samples_collected; i++)
    {
        fpVector3_t sample = {.v = {COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].x), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].y), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].z)}};
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }

    sum /= _samples_collected;

    return sum;
}

// Calculate initial offsets by simply taking the average values of the samples
static void calc_initial_offset(void)
{
    // Set initial offset to the average value of the samples
    vectorZero(&_params.offset);

    for (uint16_t k = 0; k < _samples_collected; k++)
    {
        _params.offset.x -= COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].x);
        _params.offset.y -= COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].y);
        _params.offset.z -= COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].z);
    }

    _params.offset.x /= _samples_collected;
    _params.offset.y /= _samples_collected;
    _params.offset.z /= _samples_collected;
}

static void calc_sphere_jacob(const fpVector3_t sample, const param_t params, float *ret)
{
    const fpVector3_t offset = params.offset;
    const fpVector3_t diag = params.diag;
    const fpVector3_t offdiag = params.offdiag;

    fpMatrix3_t softiron = initMatrixUsingVector(diag.x, offdiag.x, offdiag.y,
                                                 offdiag.x, diag.y, offdiag.z,
                                                 offdiag.y, offdiag.z, diag.z);

    float A = (diag.x * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B = (offdiag.x * (sample.x + offset.x)) + (diag.y * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C = (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z * (sample.z + offset.z));

    fpVector3_t sampleSumByOffSet = {.v = {sample.x + offset.x, sample.y + offset.y, sample.z + offset.z}};
    fpVector3_t softIronBySample = multiplyMatrixByVector(softiron, sampleSumByOffSet);

    float length = calc_length_pythagorean_3D(softIronBySample.x, softIronBySample.y, softIronBySample.z);

    // 0: partial derivative (radius wrt fitness fn) fn operated on sample
    ret[0] = 1.0f;
    // 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
    ret[1] = -1.0f * (((diag.x * A) + (offdiag.x * B) + (offdiag.y * C)) / length);
    ret[2] = -1.0f * (((offdiag.x * A) + (diag.y * B) + (offdiag.z * C)) / length);
    ret[3] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z * C)) / length);
}

float *get_sphere_params(param_t *param)
{
    return &(param->radius);
}

float *get_ellipsoid_params(param_t *param)
{
    return &(param->offset.x);
}

// run sphere fit to calculate diagonals and offdiagonals
static void run_sphere_fit(void)
{
    if (_sample_buffer == NULL)
    {
        return;
    }

    const float lma_damping = 10.0f;

    // take backup of fitness and parameters so we can determine later if this fit has improved the calibration
    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_SPHERE_PARAMS * COMPASS_CAL_NUM_SPHERE_PARAMS];
    float JTJ2[COMPASS_CAL_NUM_SPHERE_PARAMS * COMPASS_CAL_NUM_SPHERE_PARAMS];
    float JTFI[COMPASS_CAL_NUM_SPHERE_PARAMS];

    // Gauss Newton Part common for all kind of extensions including LM
    for (uint16_t k = 0; k < _samples_collected; k++)
    {
        fpVector3_t sample = {.v = {COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].x), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].y), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].z)}};

        float sphere_jacob[COMPASS_CAL_NUM_SPHERE_PARAMS];

        calc_sphere_jacob(sample, fit1_params, sphere_jacob);

        for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++)
        {
            // Compute JTJ
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_SPHERE_PARAMS; j++)
            {
                JTJ[i * COMPASS_CAL_NUM_SPHERE_PARAMS + j] += sphere_jacob[i] * sphere_jacob[j];
                JTJ2[i * COMPASS_CAL_NUM_SPHERE_PARAMS + j] += sphere_jacob[i] * sphere_jacob[j]; // A backup JTJ for LM
            }
            // Compute JTFI
            JTFI[i] += sphere_jacob[i] * calc_residual(sample, fit1_params);
        }
    }

    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    // refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter

    for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++)
    {
        JTJ[i * COMPASS_CAL_NUM_SPHERE_PARAMS + i] += _sphere_lambda;
        JTJ2[i * COMPASS_CAL_NUM_SPHERE_PARAMS + i] += _sphere_lambda / lma_damping;
    }

    if (!matrix_inverse4x4(JTJ, JTJ))
    {
        return;
    }

    if (!matrix_inverse4x4(JTJ2, JTJ2))
    {
        return;
    }

    // Extract radius, offset, diagonals and offdiagonal parameters
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_SPHERE_PARAMS; row++)
    {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_SPHERE_PARAMS; col++)
        {
            get_sphere_params(&fit1_params)[row] -= JTFI[col] * JTJ[row * COMPASS_CAL_NUM_SPHERE_PARAMS + col];
            get_sphere_params(&fit2_params)[row] -= JTFI[col] * JTJ2[row * COMPASS_CAL_NUM_SPHERE_PARAMS + col];
        }
    }

    // Calculate fitness of two possible sets of parameters
    fit1 = calc_mean_squared_residuals(fit1_params);
    fit2 = calc_mean_squared_residuals(fit2_params);

    // Decide which of the two sets of parameters is best and store in fit1_params
    if (fit1 > _fitness && fit2 > _fitness)
    {
        // If neither set of parameters provided better results, increase lambda
        _sphere_lambda *= lma_damping;
    }
    else if (fit2 < _fitness && fit2 < fit1)
    {
        // If fit2 was better we will use it. decrease lambda
        _sphere_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    }
    else if (fit1 < _fitness)
    {
        fitness = fit1;
    }

    //--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

    // Store new parameters and update fitness
    if (!isnan(fitness) && fitness < _fitness)
    {
        _fitness = fitness;
        _params = fit1_params;
    }
}

static void calc_ellipsoid_jacob(const fpVector3_t sample, const param_t params, float *ret)
{
    const fpVector3_t offset = params.offset;
    const fpVector3_t diag = params.diag;
    const fpVector3_t offdiag = params.offdiag;

    fpMatrix3_t softiron = initMatrixUsingVector(diag.x, offdiag.x, offdiag.y,
                                                 offdiag.x, diag.y, offdiag.z,
                                                 offdiag.y, offdiag.z, diag.z);

    float A = (diag.x * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B = (offdiag.x * (sample.x + offset.x)) + (diag.y * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C = (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z * (sample.z + offset.z));

    fpVector3_t sampleSumByOffSet = {.v = {sample.x + offset.x, sample.y + offset.y, sample.z + offset.z}};
    fpVector3_t softIronBySample = multiplyMatrixByVector(softiron, sampleSumByOffSet);

    float length = calc_length_pythagorean_3D(softIronBySample.x, softIronBySample.y, softIronBySample.z);

    // 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
    ret[0] = -1.0f * (((diag.x * A) + (offdiag.x * B) + (offdiag.y * C)) / length);
    ret[1] = -1.0f * (((offdiag.x * A) + (diag.y * B) + (offdiag.z * C)) / length);
    ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z * C)) / length);

    // 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
    ret[3] = -1.0f * ((sample.x + offset.x) * A) / length;
    ret[4] = -1.0f * ((sample.y + offset.y) * B) / length;
    ret[5] = -1.0f * ((sample.z + offset.z) * C) / length;

    // 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
    ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B)) / length;
    ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C)) / length;
    ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C)) / length;
}

static void run_ellipsoid_fit(void)
{
    if (_sample_buffer == NULL)
    {
        return;
    }

    const float lma_damping = 10.0f;

    // Take backup of fitness and parameters so we can determine later if this fit has improved the calibration
    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_ELLIPSOID_PARAMS * COMPASS_CAL_NUM_ELLIPSOID_PARAMS];
    float JTJ2[COMPASS_CAL_NUM_ELLIPSOID_PARAMS * COMPASS_CAL_NUM_ELLIPSOID_PARAMS];
    float JTFI[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];

    // Gauss Newton Part common for all kind of extensions including LM
    for (uint16_t k = 0; k < _samples_collected; k++)
    {
        fpVector3_t sample = {.v = {COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].x), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].y), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[k].z)}};

        float ellipsoid_jacob[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];

        calc_ellipsoid_jacob(sample, fit1_params, ellipsoid_jacob);

        for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++)
        {
            // compute JTJ
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; j++)
            {
                JTJ[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
                JTJ2[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
            }
            // compute JTFI
            JTFI[i] += ellipsoid_jacob[i] * calc_residual(sample, fit1_params);
        }
    }

    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    // refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter

    for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++)
    {
        JTJ[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + i] += _ellipsoid_lambda;
        JTJ2[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + i] += _ellipsoid_lambda / lma_damping;
    }

    if (!matrix_inverseN(JTJ, JTJ, 9))
    {
        return;
    }

    if (!matrix_inverseN(JTJ2, JTJ2, 9))
    {
        return;
    }

    // Extract radius, offset, diagonals and offdiagonal parameters
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; row++)
    {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; col++)
        {
            get_ellipsoid_params(&fit1_params)[row] -= JTFI[col] * JTJ[row * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + col];
            get_ellipsoid_params(&fit2_params)[row] -= JTFI[col] * JTJ2[row * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + col];
        }
    }

    // Calculate fitness of two possible sets of parameters
    fit1 = calc_mean_squared_residuals(fit1_params);
    fit2 = calc_mean_squared_residuals(fit2_params);

    // Decide which of the two sets of parameters is best and store in fit1_params
    if (fit1 > _fitness && fit2 > _fitness)
    {
        // If neither set of parameters provided better results, increase lambda
        _ellipsoid_lambda *= lma_damping;
    }
    else if (fit2 < _fitness && fit2 < fit1)
    {
        // If fit2 was better we will use it. decrease lambda
        _ellipsoid_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    }
    else if (fit1 < _fitness)
    {
        fitness = fit1;
    }

    //--------------------Levenberg-part-ends-here--------------------------------//

    // Store new parameters and update fitness
    if (fitness < _fitness)
    {
        _fitness = fitness;
        _params = fit1_params;
    }
}

static fpMatrix3_t getRotationMatrixFromAHRS(const CompassSample sample)
{
    fpMatrix3_t ahrsMatrix;
    fp_angles_t euler_rad;

    euler_rad.angles.roll = sample.att.roll * (M_PI / 127);
    euler_rad.angles.pitch = sample.att.pitch * (M_PI_2 / 127);
    euler_rad.angles.yaw = sample.att.yaw * (M_PI / 127);

    rotationMatrixFromEulerAngles(&ahrsMatrix, &euler_rad);

    return ahrsMatrix;
}

static bool verifyForRightAngleRotation(const sensor_align_e r)
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

/*
  Calculate the implied earth field for a compass sample and compass
  rotation. This is used to check for consistency between
  samples.

  If the orientation is correct then when rotated the same (or
  similar) earth field should be given for all samples.

  Note that this earth field uses an arbitrary north reference, so it
  may not match the true earth field.
 */
static fpVector3_t calculate_earth_field(const CompassSample sample, sensor_align_e r)
{
    fpVector3_t v = {.v = {COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample.x), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample.y), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample.z)}};

    // Convert the sample back to sensor frame
    vectorRotateInverse(&v, _orientation);

    // Rotate to body frame for this rotation
    vectorRotate(&v, r);

    // Apply offsets, rotating them for the orientation we are testing
    fpVector3_t rot_offsets = _params.offset;
    vectorRotateInverse(&rot_offsets, _orientation);

    vectorRotate(&rot_offsets, r);

    v.x += rot_offsets.x;
    v.y += rot_offsets.y;
    v.z += rot_offsets.z;

    // Rotate the sample from body frame back to earth frame
    fpMatrix3_t rot = getRotationMatrixFromAHRS(sample);

    fpVector3_t efield = multiplyMatrixByVector(rot, v);

    // Earth field is the mag sample in earth frame
    return efield;
}

static bool rotation_equal(sensor_align_e r1, sensor_align_e r2)
{
    if (r1 == r2)
    {
        return true;
    }

    fpVector3_t v = {.v = {1, 2, 3}};
    fpVector3_t v1 = v;
    fpVector3_t v2 = v;

    vectorRotate(&v1, r1);
    vectorRotate(&v2, r2);

    return calc_length_pythagorean_3D(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z) < 0.001f;
}

// Calculate compass orientation using the attitude estimate associated with each sample, and fix orientation on external compasses if the feature is enabled
static bool calculate_orientation(void)
{
    if (!_check_orientation)
    {
        // We are not checking orientation
        return true;
    }

    float variance[ALIGN_ROTATION_MAX];

    _orientation_solution = _orientation;

    for (sensor_align_e r = ALIGN_DEFAULT; r < ALIGN_ROTATION_MAX; r++)
    {
        // Calculate the average implied earth field across all samples
        fpVector3_t total_ef;

        for (uint16_t i = 0; i < _samples_collected; i++)
        {
            fpVector3_t efield = calculate_earth_field(_sample_buffer[i], r);
            total_ef.x += efield.x;
            total_ef.y += efield.y;
            total_ef.z += efield.z;
        }

        fpVector3_t avg_efield = {.v = {total_ef.x / _samples_collected, total_ef.y / _samples_collected, total_ef.z / _samples_collected}};

        // Now calculate the square error for this rotation against the average earth field
        for (uint16_t i = 0; i < _samples_collected; i++)
        {
            fpVector3_t efield = calculate_earth_field(_sample_buffer[i], r);
            fpVector3_t vecSubtraction = {.v = {efield.x - avg_efield.x, efield.y - avg_efield.y, efield.z - avg_efield.z}};
            float err = vectorLengthSquared(&vecSubtraction);
            // Divide by number of samples collected to get the variance
            variance[r] += err / _samples_collected;
        }
    }

    // Find the rotation with the lowest variance
    sensor_align_e besti = ALIGN_DEFAULT;
    float bestv = variance[0];

    for (sensor_align_e r = ALIGN_DEFAULT; r < ALIGN_ROTATION_MAX; r++)
    {
        if (variance[r] < bestv)
        {
            bestv = variance[r];
            besti = r;
        }
    }

    float second_best = besti == ALIGN_DEFAULT ? variance[1] : variance[0];

    for (sensor_align_e r = ALIGN_DEFAULT; r < ALIGN_ROTATION_MAX; r++)
    {
        if (!rotation_equal(besti, r))
        {
            if (variance[r] < second_best)
            {
                second_best = variance[r];
            }
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
        pass = _orientation_confidence > 2.0f; // Consider this a pass if the best orientation is 2x better variance than 2nd best
    }

    if (!pass)
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Mag(%u) bad orientation: %u/%u %.1f", _compass_idx, besti, besti2, (double)_orientation_confidence); (void)besti2;
    }
    else if (besti == _orientation)
    {
        // no orientation change
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mag(%u) good orientation: %u %.1f", _compass_idx, besti, (double)_orientation_confidence);
    }
    else if (!_is_external || !_fix_orientation)
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Mag(%u) internal bad orientation: %u %.1f", _compass_idx, besti, (double)_orientation_confidence);
    }
    else
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mag(%u) new orientation: %u was %u %.1f", _compass_idx, besti, _orientation, (double)_orientation_confidence);
    }

    if (!pass)
    {
        set_status(BAD_ORIENTATION);
        return false;
    }

    if (_orientation == besti)
    {
        // No orientation change
        return true;
    }

    if (!_is_external || !_fix_orientation)
    {
        // We won't change the orientation, but we set _orientation for reporting purposes
        _orientation = besti;
        _orientation_solution = besti;
        set_status(BAD_ORIENTATION);
        return false;
    }

    // Correct the offsets for the new orientation
    fpVector3_t rot_offsets = _params.offset;
    vectorRotateInverse(&rot_offsets, _orientation);
    vectorRotate(&rot_offsets, besti);
    _params.offset = rot_offsets;

    // Rotate the samples for the new orientation
    for (uint16_t i = 0; i < _samples_collected; i++)
    {
        fpVector3_t s = {.v = {COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].x), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].y), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].z)}};
        vectorRotateInverse(&s, _orientation);
        vectorRotate(&s, besti);
        _sample_buffer[i].x = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(s.x);
        _sample_buffer[i].y = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(s.y);
        _sample_buffer[i].z = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(s.z);
    }

    _orientation = besti;
    _orientation_solution = besti;

    // Re-run the fit to get the diagonals and off-diagonals for the new orientation
    initialize_fit();
    run_sphere_fit();
    run_ellipsoid_fit();

    return fit_acceptable();
}

#include "build/debug.h"
/*
// Calculate compass orientation using the attitude estimate associated with each sample, and fix orientation on external compasses if the feature is enabled
static bool calculate_orientation(void)
{
    if (!_check_orientation)
    {
        // We are not checking orientation
        return true;
    }

    delay(1000);

    DEBUG_SET(DEBUG_CRUISE, 4, 1000);

    float variance[ALIGN_ROTATION_MAX];

    _orientation_solution = _orientation;

    for (uint8_t n = 0; n < ARRAYLEN(variance); n++)
    {
        sensor_align_e r = (sensor_align_e)n;

        // Calculate the average implied earth field across all samples
        fpVector3_t total_ef;

        for (uint16_t i = 0; i < _samples_collected; i++)
        {
            DEBUG_SET(DEBUG_CRUISE, 0, i);
            fpVector3_t efield = calculate_earth_field(_sample_buffer[i], r);
            total_ef.x += efield.x;
            total_ef.y += efield.y;
            total_ef.z += efield.z;
        }

        fpVector3_t avg_efield = {.v = {total_ef.x / _samples_collected, total_ef.y / _samples_collected, total_ef.z / _samples_collected}};

        // Now calculate the square error for this rotation against the average earth field
        for (uint16_t i = 0; i < _samples_collected; i++)
        {
            DEBUG_SET(DEBUG_CRUISE, 1, i);
            fpVector3_t efield = calculate_earth_field(_sample_buffer[i], r);
            float err = sq(efield.x - avg_efield.x) + sq(efield.y - avg_efield.y) + sq(efield.z - avg_efield.z);
            // Divide by number of samples collected to get the variance
            variance[n] += err / _samples_collected;
        }
    }

    // Find the rotation with the lowest variance
    sensor_align_e besti = ALIGN_DEFAULT;
    sensor_align_e besti_90 = ALIGN_DEFAULT;
    float bestv = variance[0];
    float bestv_90 = variance[0];

    for (uint8_t n = 0; n < ARRAYLEN(variance); n++)
    {
        sensor_align_e r = (sensor_align_e)n;
        if (variance[n] < bestv)
        {
            bestv = variance[n];
            besti = r;
        }

        if (verifyForRightAngleRotation(r) && variance[n] < bestv_90)
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

        if (verifyForRightAngleRotation(r) && (besti_90 != r) && (variance[n] < second_best_90))
        {
            second_best_90 = variance[n];
        }
    }

    _orientation_confidence = second_best / bestv;

    bool pass;

    if (besti == _orientation) // If the orientation matched then allow for a low threshold
    {
        pass = true;
    }
    else
    {
        if (_orientation_confidence > 4.0f)
        {
            // Very confident, always pass
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
        // GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Mag(%u) bad orientation: %u/%u %.1f", _compass_idx,besti, besti2, (double)_orientation_confidence);
        DEBUG_SET(DEBUG_CRUISE, 2, 1);
    }
    else if (besti == _orientation)
    {
        // no orientation change
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mag(%u) good orientation: %u %.1f", _compass_idx, besti, (double)_orientation_confidence);
        DEBUG_SET(DEBUG_CRUISE, 2, 2);
    }
    else if (!_is_external || !_fix_orientation)
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Mag(%u) internal bad orientation: %u %.1f", _compass_idx, besti, (double)_orientation_confidence);
        DEBUG_SET(DEBUG_CRUISE, 2, 3);
    }
    else
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mag(%u) new orientation: %u was %u %.1f", _compass_idx, besti, _orientation, (double)_orientation_confidence);
        DEBUG_SET(DEBUG_CRUISE, 2, 4);
    }

    if (!pass)
    {
        set_status(BAD_ORIENTATION);
        return false;
    }

    if (_orientation == besti)
    {
        // No orientation change
        return true;
    }

    if (!_is_external || !_fix_orientation)
    {
        // We won't change the orientation, but we set _orientation for reporting purposes
        _orientation = besti;
        _orientation_solution = besti;
        set_status(BAD_ORIENTATION);
        return false;
    }

    // Correct the offsets for the new orientation
    fpVector3_t rot_offsets = _params.offset;
    vectorRotateInverse(&rot_offsets, _orientation);
    vectorRotate(&rot_offsets, besti);
    _params.offset = rot_offsets;

    // Rotate the samples for the new orientation
    for (uint16_t i = 0; i < _samples_collected; i++)
    {
        fpVector3_t s = {.v = {COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].x), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].y), COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(_sample_buffer[i].z)}};
        vectorRotateInverse(&s, _orientation);
        vectorRotate(&s, besti);
        _sample_buffer[i].x = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(s.x);
        _sample_buffer[i].y = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(s.y);
        _sample_buffer[i].z = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(s.z);
    }

    _orientation = besti;
    _orientation_solution = besti;

    // Re-run the fit to get the diagonals and off-diagonals for the new orientation
    initialize_fit();
    run_sphere_fit();
    run_ellipsoid_fit();

    return fit_acceptable();
}
*/
// Fix radius of the fit to compensate for sensor scale factor errors return false if radius is outside acceptable range
static bool fix_radius(void)
{
    if (!isGPSTrustworthy())
    {
        _params.scale_factor = 0.0f;
        return true;
    }

    float intensity;
    float declination;
    float inclination;

    gpsLocation_t newLLH = {gpsSol.llh.lat, gpsSol.llh.lon, gpsSol.llh.alt};

    getMagFieldEF(newLLH, &intensity, &declination, &inclination);

    float expected_radius = intensity * 1000.0f; // milliGauss
    float correction = expected_radius / _params.radius;

    if (correction > COMPASS_MAX_SCALE_FACTOR && correction < COMPASS_MIN_SCALE_FACTOR)
    {
        set_status(BAD_RADIUS);
        return false;
    }

    _params.scale_factor = correction;

    return true;
}

void compassCalibrationUpdate(void)
{
    // Pickup samples from intermediate struct
    pull_sample();

    const bool running = (cal_state.status == RUNNING_STEP_ONE || cal_state.status == RUNNING_STEP_TWO);

    // Update_settings
    if (!running)
    {
        update_cal_settings();
    }

    // Update requested state
    if (_status_set_requested)
    {
        _status_set_requested = false;
        set_status(_requested_status);
    }

    // Update report and status
    update_cal_status();
    update_cal_report();

    // Collect the minimum number of samples
    if (!_fitting())
    {
        return;
    }

    if (_status == RUNNING_STEP_ONE)
    {
        if (cal_state.fit_step >= 10)
        {
            if (_fitness == _initial_fitness || isnan(_fitness)) // If true, means that fitness is diverging instead of converging
            {
                set_status(COMPASS_CAL_FAILED);
            }
            else
            {
                set_status(RUNNING_STEP_TWO);
            }
        }
        else
        {
            if (cal_state.fit_step == 0)
            {
                calc_initial_offset();
            }
            run_sphere_fit();
            cal_state.fit_step++;
        }
    }
    else if (_status == RUNNING_STEP_TWO)
    {
        if (cal_state.fit_step >= 35)
        {
            if (fit_acceptable() && fix_radius())
            {
                bool oriok = calculate_orientation();
                DEBUG_SET(DEBUG_CRUISE, 0, oriok);
                set_status(COMPASS_CAL_SUCCESS);
            }
            else
            {
                set_status(COMPASS_CAL_FAILED);
            }
        }
        else if (cal_state.fit_step < 15)
        {
            run_sphere_fit();
            cal_state.fit_step++;
        }
        else
        {
            run_ellipsoid_fit();
            cal_state.fit_step++;
        }
    }
}

// Get mag field with the effects of offsets, diagonals and off-diagonals removed
static bool getUncorrectedField(fpVector3_t *field, fpVector3_t offsets, fpVector3_t diagonals, fpVector3_t offdiagonals)
{
    // Get corrected field
    fpVector3_t correctedField = *field;

    // Form eliptical correction matrix and invert it. This is needed to remove the effects of the eliptical correction when calculating new offsets
    if (diagonals.x != 0.0f && diagonals.y != 0.0f && diagonals.z != 0.0f)
    {
        fpMatrix3_t mat = initMatrixUsingVector(diagonals.x, offdiagonals.x, offdiagonals.y,
                                                offdiagonals.x, diagonals.y, offdiagonals.z,
                                                offdiagonals.y, offdiagonals.z, diagonals.z);

        if (!matrixInvert(&mat))
        {
            return false;
        }

        // Remove impact of diagonals and off-diagonals
        *field = multiplyMatrixByVector(mat, correctedField);
    }

    // Remove impact of offsets
    field->x -= offsets.x;
    field->y -= offsets.y;
    field->z -= offsets.z;

    return true;
}

/*
  Fast compass calibration given vehicle position and yaw.
  This results in zero diagonal and off-diagonal elements, so is only suitable for vehicles where the field is close to spherical. It is useful for large vehicles where moving the vehicle to calibrate it is difficult.
  The offsets of the selected compasses are set to values to bring them into consistency with the intensity, declination and inclination tables at the given latitude and longitude.
  This assumes that the compass is correctly scaled in milliGauss
*/
bool CompassCalibrationFixedYaw(float yaw_value, fpVector3_t magfield, fpVector3_t *offsets, fpVector3_t *diagonals, fpVector3_t *offdiagonals)
{
    if (!isGPSTrustworthy())
    {
        return false;
    }

    gpsLocation_t newLLH = {gpsSol.llh.lat, gpsSol.llh.lon, gpsSol.llh.alt};

    // Get the magnetic field intensity and orientation
    float intensity;
    float declination;
    float inclination;

    getMagFieldEF(newLLH, &intensity, &declination, &inclination);

    // Create a field vector and rotate to the required orientation
    fpVector3_t field = {.v = {1e3f * intensity, 0.0f, 0.0}};
    fpMatrix3_t R;
    fp_angles_t magFieldAngles = {.angles.roll = 0.0f,
                                  .angles.pitch = -DEGREES_TO_RADIANS(inclination),
                                  .angles.yaw = DEGREES_TO_RADIANS(declination)};

    rotationMatrixFromEulerAngles(&R, &magFieldAngles);

    field = multiplyMatrixByVector(R, field);

    fpMatrix3_t ahrs;
    fp_angles_t ahrsAngles = {.angles.roll = atan2_approx(rotationMatrix.m[2][1], rotationMatrix.m[2][2]),
                              .angles.pitch = ((0.5f * M_PIf) - acos_approx(-rotationMatrix.m[2][0])),
                              .angles.yaw = -yaw_value};

    rotationMatrixFromEulerAngles(&ahrs, &ahrsAngles);

    // Rotate into body frame using provided yaw
    ahrs = matrixTransposed(rotationMatrix);
    field = multiplyMatrixByVector(ahrs, field);

    fpVector3_t measurement = magfield;

    if (!getUncorrectedField(&measurement, *offsets, *diagonals, *offdiagonals))
    {
        return false;
    }

    offsets->x = field.x - measurement.x;
    offsets->y = field.y - measurement.y;
    offsets->z = field.z - measurement.z;

    diagonals->x = 1.0f;
    diagonals->y = 1.0f;
    diagonals->z = 1.0f;

    offdiagonals->x = 0.0f;
    offdiagonals->y = 0.0f;
    offdiagonals->z = 0.0f;

    return true;
}