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

#include "common/axis.h"
#include "common/time.h"

#include "drivers/compass/compass.h"

#include "sensors/sensors.h"

#define COMPASS_CAL_NUM_SPHERE_PARAMS 4
#define COMPASS_CAL_NUM_ELLIPSOID_PARAMS 9
#define COMPASS_CAL_NUM_SAMPLES 300 // number of samples required before fitting begins

#define COMPASS_MAX_SCALE_FACTOR 1.5
#define COMPASS_MIN_SCALE_FACTOR (1.0 / COMPASS_MAX_SCALE_FACTOR)
#define COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(__X) ((int16_t)constrainf(roundf(__X * 8.0f), INT16_MIN, INT16_MAX))
#define COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(__X) (__X / 8.0f)
#define FIELD_RADIUS_MIN 150
#define FIELD_RADIUS_MAX 950

#define COMPASS_CALIBRATION_INIT_DELAY 1000 // ms
#define COMPASS_OFFSETS_MAX_DEFAULT 1800    // This sets the maximum allowed compass offset in calibration
// Description: This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass).
// Values: 4:Very Strict, 8:Strict, 16:Default, 32:Relaxed
#define COMPASS_CALIBRATION_FITNESS_DEFAULT 16.0f

// compass calibration states
typedef enum
{
    NOT_STARTED = 0,
    WAITING_TO_START = 1,
    RUNNING_STEP_ONE = 2,
    RUNNING_STEP_TWO = 3,
    COMPASS_CAL_SUCCESS = 4,
    COMPASS_CAL_FAILED = 5,
    BAD_ORIENTATION = 6,
    BAD_RADIUS = 7,
} status_e;

// results
typedef struct
{
    float *radius_p;
    float *offsetX_p;
    float radius;        // magnetic field strength calculated from samples
    fpVector3_t offset;  // offsets
    fpVector3_t diag;    // diagonal scaling
    fpVector3_t offdiag; // off diagonal scaling
    float scale_factor;  // scaling factor to compensate for radius error
} param_t;

typedef union
{
    struct
    {
        int8_t roll;
        int8_t pitch;
        int8_t yaw;
    } att;
    int16_t x;
    int16_t y;
    int16_t z;
} CompassSample;

typedef struct
{
    status_e status;
    bool calibration_finished;
    uint8_t attempt;
    float completion_pct;
    uint16_t fit_step; // step during RUNNING_STEP_ONE/TWO which performs sphere fit and ellipsoid fit
} State;

// Structure accessed after calibration is finished/failed
typedef struct
{
    status_e status;
    float fitness;
    fpVector3_t ofs;
    fpVector3_t diag;
    fpVector3_t offdiag;
    float orientation_confidence;
    sensor_align_e original_orientation;
    sensor_align_e orientation;
    float scale_factor;
    bool check_orientation;
} Report;

// Structure setup to set calibration run settings
typedef struct
{
    float tolerance;
    bool check_orientation;
    sensor_align_e orientation;
    sensor_align_e orig_orientation;
    bool is_external;
    bool fix_orientation;
    uint16_t offset_max;
    uint8_t attempt;
    float delay_start_sec;
    uint32_t start_time_ms;
} Settings;

void compassCalibrationStart(float delay, uint16_t offset_max, float tolerance);
void compassCalibrationStop(void);
void compassCalibrationSetNewSample(const fpVector3_t sample);
void compassCalibrationSetOrientation(sensor_align_e orientation, bool is_external, bool fix_orientation);
bool compassIsCalibrating(void);
void compassCalibrationUpdate(timeUs_t currentTimeUs);
Report getCompassCalibrationReport(void);
State getCompassCalibrationState(void);
void setCompassCalibrationFinished(bool state);
bool getCompassCalibrationFinished(void);
bool CompassCalibrationFixedYaw(float yaw_deg, fpVector3_t magfield, fpVector3_t *offsets, fpVector3_t *diagonals, fpVector3_t *offdiagonals);