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
  24 state EKF based on the derivation in https://github.com/priseborough/
  InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/
  GenerateNavFilterEquations.m

  Converted from Matlab to C++ by Paul Riseborough
  Converted from C++ to C by Julio Cesar Matias

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <stdbool.h>
#include <inttypes.h>
#include "common/vector.h"

#define IMU_DT_MIN_SEC 0.001f // Minimum delta time between IMU samples (sec)
#define N_MODELS_EKFGSF 5U

// Update Filter States - this should be called whenever new IMU data is available
void EKFGSF_yaw_update(const fpVector3_t delAng, // IMU delta angle rotation vector measured in body frame (rad)
                       const fpVector3_t delVel, // IMU delta velocity vector measured in body frame (m/s)
                       const float delAngDT,     // time interval that delAng was integrated over (sec) - must be no less than IMU_DT_MIN_SEC
                       const float delVelDT,     // time interval that delVel was integrated over (sec) - must be no less than IMU_DT_MIN_SEC
                       bool runEKF,              // set to true when flying or movement suitable for yaw estimation
                       float TAS);               // true airspeed used for centripetal accel compensation - set to 0 when not required.

// Fuse NE velocty mesurements and update the EKF's and GSF state and covariance estimates
// Should be called after update(...) whenever new velocity data is available
void EKFGSF_yaw_fuseVelData(const fpVector2_t vel, // NE velocity measurement (m/s)
                            const float velAcc);   // 1-sigma accuracy of velocity measurement (m/s)

// set the gyro bias in rad/sec
void EKFGSF_yaw_setGyroBias(fpVector3_t gyroBias);

// get yaw estimated and corresponding variance return false if yaw estimation is inactive.
bool EKFGSF_yaw_getYawData(float *yaw, float *yawVariance);

// n_clips will contain the number of models which were *not* used to create the yaw and yawVariance return values.
uint8_t EKFGSF_yaw_getNClips(void);

// get the length of the weighted average velocity innovation vector
// return false if not available
bool EKFGSF_yaw_getVelInnovLength(float *velInnovLength);