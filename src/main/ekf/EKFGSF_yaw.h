#pragma once

#pragma GCC optimize("O2")

#include <stdbool.h>
#include <inttypes.h>
#include "common/vector.h"

#define IMU_DT_MIN_SEC 0.001f // Minimum delta time between IMU samples (sec)
#define N_MODELS_EKFGSF 5U

// Update Filter States - this should be called whenever new IMU data is available
void EKFGSF_yaw_update(const fpVector3_t delAng, // IMU delta angle rotation vector measured in body frame (rad)
                       const fpVector3_t delVel, // IMU delta velocity vector measured in body frame (m/s)
                       const float delAngDT,      // time interval that delAng was integrated over (sec) - must be no less than IMU_DT_MIN_SEC
                       const float delVelDT,      // time interval that delVel was integrated over (sec) - must be no less than IMU_DT_MIN_SEC
                       bool runEKF,               // set to true when flying or movement suitable for yaw estimation
                       float TAS);                // true airspeed used for centripetal accel compensation - set to 0 when not required.

// Fuse NE velocty mesurements and update the EKF's and GSF state and covariance estimates
// Should be called after update(...) whenever new velocity data is available
void EKFGSF_yaw_fuseVelData(const fpVector2_t vel, // NE velocity measurement (m/s)
                            const float velAcc);   // 1-sigma accuracy of velocity measurement (m/s)

// set the gyro bias in rad/sec
void EKFGSF_yaw_setGyroBias(fpVector3_t gyroBias);

// get yaw estimated and corresponding variance return false if
// yaw estimation is inactive.  n_clips will contain the number of
// models which were *not* used to create the yaw and yawVariance
// return values.
bool EKFGSF_yaw_getYawData(float *yaw, float *yawVariance);

// get the length of the weighted average velocity innovation vector
// return false if not available
bool EKFGSF_yaw_getVelInnovLength(float *velInnovLength);