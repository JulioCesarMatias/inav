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

  EKF Tuning parameters refactored by Tom Cauchois

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
#include <string.h>
#include "ekf/ekfMath.h"
#include "io/gps.h"

// User EKF Tuneable Parameters
typedef struct
{
  // User confgurable params
  bool _enable;                 // false to disable the EKF
  bool _ekfCompassLearn;        // true to active the auto EKF inflight Compass calibration
  float _gpsHorizVelNoise;      // GPS horizontal velocity measurement noise : m/s
  float _gpsVertVelNoise;       // GPS vertical velocity measurement noise : m/s
  float _gpsHorizPosNoise;      // GPS horizontal position measurement noise m
  float _baroAltNoise;          // Baro height measurement noise : m
  float _magNoise;              // magnetometer measurement noise : gauss
  float _easNoise;              // equivalent airspeed measurement noise : m/s
  float _windVelProcessNoise;   // wind velocity state process noise : m/s^2
  float _wndVarHgtRateScale;    // scale factor applied to wind process noise due to height rate
  float _magEarthProcessNoise;  // Earth magnetic field process noise : gauss/sec
  float _magBodyProcessNoise;   // Body magnetic field process noise : gauss/sec
  float _gyrNoise;              // gyro process noise : rad/s
  float _accNoise;              // accelerometer process noise : m/s^2
  float _gyroBiasProcessNoise;  // gyro bias state process noise : rad/s
  float _accelBiasProcessNoise; // accel bias state process noise : m/s^2
  int16_t _hgtDelay_ms;         // effective average delay of Height measurements relative to inertial measurements (msec)
  int8_t _fusionModeGPS;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
  int16_t _gpsVelInnovGate;     // Percentage number of standard deviations applied to GPS velocity innovation consistency check
  int16_t _gpsPosInnovGate;     // Percentage number of standard deviations applied to GPS position innovation consistency check
  int16_t _hgtInnovGate;        // Percentage number of standard deviations applied to height innovation consistency check
  int16_t _magInnovGate;        // Percentage number of standard deviations applied to magnetometer innovation consistency check
  int16_t _tasInnovGate;        // Percentage number of standard deviations applied to true airspeed innovation consistency check
  int8_t _magCal;               // Sets activation condition for in-flight magnetometer calibration
  int8_t _gpsGlitchRadiusMax;   // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
  float _flowNoise;             // optical flow rate measurement noise
  int16_t _flowInnovGate;       // Percentage number of standard deviations applied to optical flow innovation consistency check
  int8_t _flowDelay_ms;         // effective average delay of optical flow measurements rel to IMU (msec)
  int16_t _rngInnovGate;        // Percentage number of standard deviations applied to range finder innovation consistency check
  float _maxFlowRate;           // Maximum flow rate magnitude that will be accepted by the filter
  int8_t _altSource;            // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder, 2 = use GPS
  float _gyroScaleProcessNoise; // gyro scale factor state process noise : 1/s
  float _rngNoise;              // Range finder noise : m
  int8_t _gpsCheck;             // Bitmask controlling which preflight GPS checks are bypassed
  int16_t _gpsCheckScaler;      // Percentage increase to be applied to GPS pre-flight accuracy and drift thresholds
  float _noaidHorizNoise;       // horizontal position measurement noise assumed when synthesised zero position measurements are used to constrain attitude drift : m
  float _yawNoise;              // magnetic yaw measurement noise : rad
  int16_t _yawInnovGate;        // Percentage number of standard deviations applied to magnetic yaw innovation consistency check
  int8_t _tauVelPosOutput;      // Time constant of output complementary filter : csec (centi-seconds)
  int8_t _useRngSwHgt;          // Maximum valid range of the range finder as a percentage of the maximum range specified by the sensor driver
  float _terrGradMax;           // Maximum terrain gradient below the vehicle
  float _useRngSwSpd;           // Maximum horizontal ground speed to use range finder as the primary height source (m/s)
  int8_t _originHgtMode;        // Bitmask controlling post alignment correction and reporting of the EKF origin height.
  int8_t _flowUse;              // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.
  int16_t _mag_ef_limit;        // limit on difference between WMM tables and learned earth field.
  float _hrt_filt_freq;         // frequency of output observer height rate complementary filter in Hz
  int8_t _gsfResetMaxCount;     // maximum number of times the EKF is allowed to reset it's yaw to the EKF-GSF estimate
} ekfParam_t;

// Developer configurable params
typedef struct
{
  float gpsNEVelVarAccScale;      // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
  float gpsDVelVarAccScale;       // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
  float gpsPosVarAccScale;        // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
  uint8_t magDelay_ms;            // Magnetometer measurement delay (msec)
  uint8_t tasDelay_ms;            // Airspeed measurement delay (msec)
  uint16_t tiltDriftTimeMax_ms;   // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
  uint16_t posRetryTimeUseVel_ms; // Position aiding retry time with velocity measurements (msec)
  uint16_t posRetryTimeNoVel_ms;  // Position aiding retry time without velocity measurements (msec)
  uint16_t hgtRetryTimeMode0_ms;  // Height retry time with vertical velocity measurement (msec)
  uint16_t hgtRetryTimeMode12_ms; // Height retry time without vertical velocity measurement (msec)
  uint16_t tasRetryTime_ms;       // True airspeed timeout and retry interval (msec)
  uint16_t magFailTimeLimit_ms;   // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
  float magVarRateScale;          // scale factor applied to magnetometer variance due to angular rate
  uint8_t hgtAvg_ms;              // average number of msec between height measurements
  uint8_t betaAvg_ms;             // average number of msec between synthetic sideslip measurements
  float DCM33FlowMin;             // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
  uint8_t flowTimeDeltaAvg_ms;    // average interval between optical flow measurements (msec)
  float gndEffectBaroScaler;      // scaler applied to the barometer observation variance when ground effect mode is active
  float maxYawEstVelInnov;        // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)
  uint8_t framesPerPrediction;    // expected number of IMU frames per prediction
  uint16_t imuTimeHz;             // IMU time in Hz
} ekfInternalParam_t;

extern ekfParam_t ekfParam;
extern ekfInternalParam_t ekfInternalParam;

bool ekfInitialiseFilter(void);
void ekfUpdateFilter(void);
bool get_armed(void);
bool get_takeoff_expected(void);
bool get_touchdown_expected(void);
bool resetHeightDatum(void);
void setTerrainHgtStable(bool val);
bool ekfSetOriginLLH(gpsLocation_t *loc);
void writeOptFlowMeas(const uint8_t rawFlowQuality, const fpVector2_t rawFlowRates, const fpVector2_t rawGyroRates, const fpVector3_t posOffset, float heightOverride);
bool configuredToUseGPSForPosXY(void);