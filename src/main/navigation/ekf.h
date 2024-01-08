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

#include "common/quaternion.h"
#include "navigation/navigation_private.h"

// earth rotation rate (rad/sec)
#define EARTH_RATE 0.000072921f

// initial gyro bias uncertainty (deg/sec)
#define INIT_GYRO_BIAS_UNCERTAINTY 0.035f

// maximum gyro bias in rad/sec that can be compensated for
#define EKF_MAX_GYRO_BIAS 0.1745f

// when the wind estimation first starts with no airspeed sensor, assume 3m/s to start
#define STARTUP_WIND_SPEED 3.0f

// copter defaults
#define MC_INIT_ACCEL_BIAS_UNCERTAINTY 0.1f
#define MC_VELNE_NOISE_DEFAULT 0.5f
#define MC_VELD_NOISE_DEFAULT 0.7f
#define MC_POSNE_NOISE_DEFAULT 0.5f
#define MC_ALT_NOISE_DEFAULT 2.0f
#define MC_MAG_NOISE_DEFAULT 0.05f
#define MC_GYRO_PNOISE_DEFAULT 0.015f
#define MC_ACC_PNOISE_DEFAULT 0.25f
#define MC_GBIAS_PNOISE_DEFAULT 1E-06f
#define MC_ABIAS_PNOISE_DEFAULT 0.00005f
#define MC_MAGE_PNOISE_DEFAULT 0.0006f
#define MC_MAGB_PNOISE_DEFAULT 0.0006f
#define MC_VEL_GATE_DEFAULT 5
#define MC_POS_GATE_DEFAULT 10
#define MC_HGT_GATE_DEFAULT 10
#define MC_MAG_GATE_DEFAULT 3
#define MC_GLITCH_ACCEL_DEFAULT 100
#define MC_GLITCH_RADIUS_DEFAULT 25

// fixed-wing defaults
#define FW_INIT_ACCEL_BIAS_UNCERTAINTY 0.3f
#define FW_VELNE_NOISE_DEFAULT 0.5f
#define FW_VELD_NOISE_DEFAULT 0.7f
#define FW_POSNE_NOISE_DEFAULT 0.5f
#define FW_ALT_NOISE_DEFAULT 0.5f
#define FW_MAG_NOISE_DEFAULT 0.05f
#define FW_GYRO_PNOISE_DEFAULT 0.015f
#define FW_ACC_PNOISE_DEFAULT 0.5f
#define FW_GBIAS_PNOISE_DEFAULT 8E-06f
#define FW_ABIAS_PNOISE_DEFAULT 0.00005f
#define FW_MAGE_PNOISE_DEFAULT 0.0003f
#define FW_MAGB_PNOISE_DEFAULT 0.0003f
#define FW_VEL_GATE_DEFAULT 6
#define FW_POS_GATE_DEFAULT 30
#define FW_HGT_GATE_DEFAULT 20
#define FW_MAG_GATE_DEFAULT 3
#define FW_GLITCH_ACCEL_DEFAULT 150
#define FW_GLITCH_RADIUS_DEFAULT 20

typedef struct
{
    float accBiasUncertainty;    // This is the assumed uncertainty in gyro bias in rad/sec used to initialise the covariance matrix.
    float accNoise;              // This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more. (Param in m/s^2)
    float gyroNoise;             // This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more. (Param in m/s^2)
    float accelBiasProcessNoise; // This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier. (Param in m/s^2)
    float gyroBiasProcessNoise;  // This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier. (Param in rad/s)
    float magNoise;              // This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements. (Param in gauss)
    float magEarthProcessNoise;  // This noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field bias estimation faster and noisier. (Param in gauss/s)
    float magBodyProcessNoise;   // This noise controls the growth of body magnetic field state error estimates. Increasing it makes compass offset estimation faster and noisier. (Param in gauss/s)
    float magInnovGate;          // This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    float hgtInnovGate;          // This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    float baroAltNoise;          // This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement. (Param in meters)
    float gpsHorizVelNoise;      // This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the  weighting on these measurements.
    float gpsVertVelNoise;       // This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
    float gpsHorizPosNoise;      // This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements. (Param in meters)
    float gpsPosInnovGate;       // This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    float gpsGlitchAccelMax;     // This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected. If this value is set too low, then valid GPS data will be regularly discarded, and the position accuracy will degrade. If this parameter is set too high, then large GPS glitches will cause large rapid changes in position. (Param in cm/s^2)
    float gpsGlitchRadiusMax;    // This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and the filter states are reset to the new GPS position. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
    float gpsVelInnovGate;       // This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    float iasNoise;              // This is the RMS value of noise in indicated airspeed measurements. Increasing it reduces the weighting on these measurements.
    float iasInnovGate;          // This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    float windVelProcessNoise;   // This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier. (Param in m/s^2)
    float wndVarHgtRateScale;    // Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
} ekfParam_t;

typedef struct ekfConfig_s
{
    bool ekfEnabled;
} ekfConfig_t;

PG_DECLARE(ekfConfig_t, ekfConfig);

extern navEstimatedPosVel_t ekfPosVel;
bool ekf_getLoopTime(uint16_t *newLoopTime);
void ekf_setMagDeclination(float declination);
bool ekf_HealthyToUse(void);
void ekf_getVariances(float *velVar, float *posVar, float *hgtVar, float *magVar, float *tasVar);
fpQuaternion_t ekf_getQuaternion(void);
void ekf_Update(float deltaTime);