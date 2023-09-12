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

#include <stdint.h>
#include <math.h>

#include "common/maths.h"
#include "drivers/sensor.h"
#include "sensors/boardalignment.h"

typedef union {
    float v[3];
    struct {
       float x,y,z;
    };
} fpVector3_t;

typedef struct {
    float m[3][3];
} fpMatrix3_t;

typedef struct {
    fpVector3_t axis;
    float angle;
} fpAxisAngle_t;

void rotationMatrixFromEulerAngles(fpMatrix3_t * rmat, const fp_angles_t * angles);
void rotationMatrixFromAxisAngle(fpMatrix3_t * rmat, const fpAxisAngle_t * a);
fpVector3_t multiplyMatrixByVector(fpMatrix3_t m, fpVector3_t v);

static inline void vectorZero(fpVector3_t * v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

static inline void multiplicationTransposeMatrixByVector(const fpMatrix3_t mat, fpVector3_t *vec)
{
    const fpVector3_t v = {.v = {vec->x, vec->y, vec->z}};

    vec->x = mat.m[0][0] * v.x + mat.m[1][0] * v.y + mat.m[2][0] * v.z;
    vec->y = mat.m[0][1] * v.x + mat.m[1][1] * v.y + mat.m[2][1] * v.z;
    vec->z = mat.m[0][2] * v.x + mat.m[1][2] * v.y + mat.m[2][2] * v.z;
}

static inline float vectorNormSquared(const fpVector3_t * v)
{
    return sq(v->x) + sq(v->y) + sq(v->z);
}

static inline fpVector3_t * vectorNormalize(fpVector3_t * result, const fpVector3_t * v)
{
    float length = fast_fsqrtf(vectorNormSquared(v));
    if (length != 0) {
        result->x = v->x / length;
        result->y = v->y / length;
        result->z = v->z / length;
    }
    else {
        result->x = 0;
        result->y = 0;
        result->z = 0;
    }
    return result;
}

static inline fpVector3_t * vectorCrossProduct(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->y * b->z - a->z * b->y;
    ab.y = a->z * b->x - a->x * b->z;
    ab.z = a->x * b->y - a->y * b->x;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorAdd(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->x + b->x;
    ab.y = a->y + b->y;
    ab.z = a->z + b->z;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorScale(fpVector3_t * result, const fpVector3_t * a, const float b)
{
    fpVector3_t ab;

    ab.x = a->x * b;
    ab.y = a->y * b;
    ab.z = a->z * b;

    *result = ab;
    return result;
}

static inline void vectorRotate(fpVector3_t *vec, const sensor_align_e rotation)
{
    float vecCopy[3] = {vec->x, vec->y, vec->z};
    applySensorAlignment(vecCopy, vecCopy, rotation);
    vec->x = vecCopy[0];
    vec->y = vecCopy[1];
    vec->z = vecCopy[2];
}

static inline void vectorRotateInverse(fpVector3_t *vec, const sensor_align_e rotation)
{
    fpVector3_t x_vec = {.v = {1.0f, 0.0f, 0.0f}};
    fpVector3_t y_vec = {.v = {0.0f, 1.0f, 0.0f}};
    fpVector3_t z_vec = {.v = {0.0f, 0.0f, 1.0f}};

    vectorRotate(&x_vec, rotation);
    vectorRotate(&y_vec, rotation);
    vectorRotate(&z_vec, rotation);

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

    multiplicationTransposeMatrixByVector(vecToMatrix, vec);
}