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

#include "common/vector.h"

typedef float Matrix22[22][22];

static inline void zeroMatrix(fpMatrix3_t *matrix) 
{
    // A
    matrix->m[0][0] = 0.0f;
    matrix->m[0][1] = 0.0f;
    matrix->m[0][2] = 0.0f;

    // B
    matrix->m[1][0] = 0.0f;
    matrix->m[1][1] = 0.0f;
    matrix->m[1][2] = 0.0f;
    
    // C
    matrix->m[2][0] = 0.0f;
    matrix->m[2][1] = 0.0f;
    matrix->m[2][2] = 0.0f;
}

static inline void identityMatrix(fpMatrix3_t *matrix) 
{
    // A
    matrix->m[0][0] = 1.0f;
    matrix->m[0][1] = 0.0f;
    matrix->m[0][2] = 0.0f;

    // B
    matrix->m[1][0] = 0.0f;
    matrix->m[1][1] = 1.0f;
    matrix->m[1][2] = 0.0f;

    // C
    matrix->m[2][0] = 0.0f;
    matrix->m[2][1] = 0.0f;
    matrix->m[2][2] = 1.0f;
}

// matrix multiplication by a vector
static inline fpVector3_t multiplyMatrixByVector(fpMatrix3_t m, fpVector3_t v)
{
    fpVector3_t vRet;

    vRet.x = m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z;
    vRet.y = m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z;
    vRet.z = m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z;

    return vRet;
}

static inline void matrixFromEuler(fpMatrix3_t *m, float roll, float pitch, float yaw)
{
    const float cp = cos(pitch);
    const float sp = sin(pitch);
    const float sr = sin(roll);
    const float cr = cos(roll);
    const float sy = sin(yaw);
    const float cy = cos(yaw);

    m->m[0][0] = cp * cy;
    m->m[0][1] = (sr * sp * cy) - (cr * sy);
    m->m[0][2] = (cr * sp * cy) + (sr * sy);
    m->m[1][0] = cp * sy;
    m->m[1][1] = (sr * sp * sy) + (cr * cy);
    m->m[1][2] = (cr * sp * sy) - (sr * cy);
    m->m[2][0] = -sp;
    m->m[2][1] = sr * cp;
    m->m[2][2] = cr * cp;
}

static inline fpMatrix3_t matrixTransposed(const fpMatrix3_t m)
{
    fpMatrix3_t result = {{{m.m[0][0], m.m[1][0], m.m[2][0]},
                           {m.m[0][1], m.m[1][1], m.m[2][1]},
                           {m.m[0][2], m.m[1][2], m.m[2][2]}}};

    return result;
}