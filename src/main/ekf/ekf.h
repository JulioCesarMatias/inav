/*
  22 state EKF based on https://github.com/priseborough/InertialNav

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

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "common/vector.h"
#include "common/quaternion.h"

/* External functions:
 * ekf_getEkfControlLimits (max speed for auto-pilot modes)
 * ekf_getMagOffsets (EKF inflight mag calibration)
 * ekf_writeOptFlowMeas (used to set the optical-flow in the EKF)
 * vgetFlowDebug (add in the black-box)
 * ekf_getLastYawResetAngle (for yaw reset on INS)
 * ekf_getLastPosNorthEastReset (to use with pos-hold brake)
*/

// EKF Developer Tunable Parameters
typedef struct
{
    bool ekf_enable;           // Set false to disable the EKF.
    uint8_t ekf_altSource;     // This parameter controls which height sensor is used by the EKF during optical flow navigation (when fusionModeGPS = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will cause it to use range finder if available.
    uint8_t ekf_gpsCheck;      // Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
    uint8_t ekf_fusionModeGPS; // This parameter controls use of GPS measurements in the EKF: 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available).
    uint8_t ekf_magCal;        // EKF Mag Calibration = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF Mag Calibration = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF Mag Calibration = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.

    float ekf_gpsHorizVelNoise;      // GPS horizontal velocity measurement noise : m/s
    float ekf_gpsVertVelNoise;       // GPS vertical velocity measurement noise : m/s
    float ekf_gpsHorizPosNoise;      // GPS horizontal position measurement noise m
    float ekf_baroAltNoise;          // Baro height measurement noise : m^2
    float ekf_magNoise;              // magnetometer measurement noise : gauss
    float ekf_easNoise;              // equivalent airspeed measurement noise : m/s
    float ekf_windVelProcessNoise;   // wind velocity state process noise : m/s^2
    float ekf_wndVarHgtRateScale;    // scale factor applied to wind process noise due to height rate
    float ekf_magEarthProcessNoise;  // earth magnetic field process noise : gauss/sec
    float ekf_magBodyProcessNoise;   // earth magnetic field process noise : gauss/sec
    float ekf_gyrNoise;              // gyro process noise : rad/s
    float ekf_accNoise;              // accelerometer process noise : m/s^2
    float ekf_gyroBiasProcessNoise;  // gyro bias state process noise : rad/s
    float ekf_accelBiasProcessNoise; // accel bias state process noise : m/s^2
    uint16_t ekf_msecVelDelay;       // effective average delay of GPS velocity measurements rel to IMU (msec)
    uint16_t ekf_msecPosDelay;       // effective average delay of GPS position measurements rel to (msec)
    uint8_t ekf_gpsVelInnovGate;     // Number of standard deviations applied to GPS velocity innovation consistency check
    uint8_t ekf_gpsPosInnovGate;     // Number of standard deviations applied to GPS position innovation consistency check
    uint8_t ekf_hgtInnovGate;        // Number of standard deviations applied to height innovation consistency check
    uint8_t ekf_magInnovGate;        // Number of standard deviations applied to magnetometer innovation consistency check
    uint8_t ekf_iasInnovGate;        // Number of standard deviations applied to indicated airspeed innovation consistency check
    uint16_t ekf_gpsGlitchAccelMax;  // Maximum allowed discrepancy between inertial and GPS Horizontal acceleration before GPS data is ignored : cm/s^2
    uint8_t ekf_gpsGlitchRadiusMax;  // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    uint8_t ekf_gndGradientSigma;    // RMS terrain gradient percentage assumed by the terrain height estimation.
    float ekf_flowNoise;             // optical flow rate measurement noise
    uint8_t ekf_flowInnovGate;       // Number of standard deviations applied to optical flow innovation consistency check
    uint8_t ekf_msecFLowDelay;       // effective average delay of optical flow measurements rel to IMU (msec)
    uint8_t ekf_rngInnovGate;        // Number of standard deviations applied to range finder innovation consistency check
    float ekf_maxFlowRate;           // Maximum flow rate magnitude that will be accepted by the filter
} ekf_dev_param_t;

typedef float Vector2[2];
typedef float Vector4[4];
typedef float Vector6[6];
typedef float Vector8[8];
typedef float Vector9[9];
typedef float Vector10[10];
typedef float Vector11[11];
typedef float Vector13[13];
typedef float Vector15[15];
typedef float Vector22[22];
typedef float Vector31[31];
typedef float Vector34[34];
typedef float Matrix22[22][22];
typedef uint32_t Vector_u32_50[50];

typedef union
{
    float v[2];
    struct
    {
        float x, y;
    };
} fpVector2_t;

bool ekf_isEnabled(void);
bool ekf_InitialiseFilterDynamic(void);

// Update Filter States - this should be called whenever new IMU data is available
void ekf_UpdateFilter(void);

// Check basic filter health metrics and return a consolidated health status
bool ekf_healthy(void);

// return NEU position in cm/s
bool ekf_getPosNEU(fpVector3_t *pos);

// return NEU velocity in cm/s
void ekf_getVelNEU(fpVector3_t *vel);

// This returns the specific forces in the NED frame
void ekf_getAccelNED(fpVector3_t *accelNED);

// return body axis gyro bias estimates in rad/sec
void ekf_getGyroBias(fpVector3_t *gyroBias);

// reset body axis gyro bias estimates
void ekf_resetGyroBias(void);

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void ekf_getEkfControlLimits(float *ekfGndSpdLimit, float *ekfNavVelGainScaler);

// return the individual Z-accel bias estimates in m/s^2
void ekf_getAccelZBias(float *zbias1, float *zbias2);

// return the NE wind speed estimates in cm/s
void ekf_getWind(fpVector2_t *wind);

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool ekf_getMagOffsets(fpVector3_t *magOffsets);

// return the Euler roll, pitch and yaw angle in radians
void ekf_getEulerAngles(fpVector3_t *eulers);

// return the transformation matrix from XYZ (body) to NED axes
void ekf_getRotationBodyToNED(fpMat3_t *mat);

// return the quaternions defining the rotation from NED to XYZ (body) axes
void ekf_getQuaternion(fpQuaternion_t *quat);

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vias measurements
void ekf_getInnovations(fpVector3_t *velInnov, fpVector3_t *posInnov, fpVector3_t *magInnov, float *iasInnov);

// return the innovation consistency test ratios for the velocity, position, magnetometer and indicated airspeed measurements
void ekf_getVariances(float *velVar, float *posVar, float *hgtVar, float *magVar, float *iasVar, float *rngVar);

// write the raw optical flow measurements
// rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
// rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
// rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
// The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
// The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
void ekf_writeOptFlowMeas(uint8_t rawFlowQuality, fpVector2_t rawFlowRates, fpVector2_t rawGyroRates);

// return data for debugging optical flow fusion
void ekf_getFlowDebug(float *varFlow, float *gndOffset, float *flowInnovX, float *flowInnovY, float *auxInnov, float *HAGL, float *rngInnov, float *range, float *gndOffsetErr);

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void ekf_setTakeoffExpected(bool val);

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void ekf_setTouchdownExpected(bool val);

// Return the navigation filter status (use with MSP)
uint16_t ekf_send_status_report(void);

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t ekf_getLastYawResetAngle(float *yawAng);

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t ekf_getLastPosNorthEastReset(fpVector2_t *pos);

// update the navigation filter status
void ekf_updateFilterStatus(void);

// update the quaternion, velocity and position states using IMU measurements
void ekf_UpdateStrapdownEquationsNED(void);

// calculate the predicted state covariance matrix
void ekf_CovariancePrediction(void);

// force symmetry on the state covariance matrix
void ekf_ForceSymmetry(void);

// copy covariances across from covariance prediction calculation and fix numerical errors
void ekf_CopyAndFixCovariances(void);

// constrain variances (diagonal terms) in the state covariance matrix
void ekf_ConstrainVariances(void);

// constrain states
void ekf_ConstrainStates(void);

// fuse selected position, velocity and height measurements
void ekf_FuseVelPosNED(void);

// fuse magnetometer measurements
void ekf_FuseMagnetometer(void);

// fuse indicated airspeed measurements
void ekf_FuseAirspeed(void);

// fuse sythetic sideslip measurement of zero
void ekf_FuseSideslip(void);

// store states along with system time stamp in msces
void ekf_StoreStates(void);

// Reset the stored state history and store the current state
void ekf_StoreStatesReset(void);

// calculate the NED earth spin vector in rad/sec
void ekf_calcEarthRateNED(fpVector3_t *omega, int32_t latitude);

// calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
void ekf_SetFlightAndFusionModes(void);

// initialise the covariance matrix
void ekf_CovarianceInit(void);

// helper functions for readIMUData
bool ekf_readDeltaVelocity(fpVector3_t *dVel, float *dVel_dt);
bool ekf_readDeltaAngle(fpVector3_t *dAng);
bool ekf_readDeltaVelocity2(fpVector3_t *dVel, float *dVel_dt);
bool ekf_readDeltaAngle2(fpVector3_t *dAng);

// update IMU delta angle and delta velocity measurements
void ekf_readIMUData(void);

// check for new valid GPS data and update stored measurement if available
void ekf_readGpsData(void);

// check for new altitude measurement data and update stored measurement if available
void ekf_readHgtData(void);

// check for new magnetometer data and update store measurements if available
void ekf_readMagData(void);

// check for new airspeed data and update stored measurements if available
void ekf_readAirSpdData(void);

// determine when to perform fusion of GPS position and  velocity measurements
void ekf_SelectVelPosFusion(void);

// determine when to perform fusion of indicated airspeed measurements
void ekf_SelectIasFusion(void);

// determine when to perform fusion of synthetic sideslp measurements
void ekf_SelectBetaFusion(void);

// determine when to perform fusion of magnetometer measurements
void ekf_SelectMagFusion(void);

// force alignment of the yaw angle using GPS velocity data
void ekf_alignYawGPS(void);

// This function is used to do a forced alignment of the wind velocity
// states so that they are set to the reciprocal of the ground speed
// and scaled to STARTUP_WIND_SPEED m/s. This is used when launching a
// fly-forward vehicle without an airspeed sensor on the assumption
// that launch will be into wind and STARTUP_WIND_SPEED is
// representative of typical launch wind
void ekf_setWindVelStates(void);

// initialise the earth magnetic field states using declination and current attitude and magnetometer meaasurements and return attitude quaternion
fpQuaternion_t ekf_calcQuatAndFieldStates(float roll, float pitch);

// zero stored variables
void ekf_InitialiseVariables(void);

// reset the horizontal position states uing the last GPS measurement
void ekf_ResetPosition(void);

// reset velocity states using the last GPS measurement
void ekf_ResetVelocity(void);

// reset the vertical position state using the last height measurement
void ekf_ResetHeight(void);

// return true if we should use the airspeed sensor
bool ekf_useAirspeed(void);

// return true if optical flow data is available
bool ekf_optFlowDataPresent(void);

// determine when to perform fusion of optical flow measurements
void ekf_SelectFlowFusion(void);

// recall omega (angular rate vector) average from time specified by msec to current time
// this is useful for motion compensation of optical flow measurements
void ekf_RecallOmega(fpVector3_t *omegaAvg, uint32_t msecStart, uint32_t msecEnd);

// Estimate terrain offset using a single state EKF
void ekf_EstimateTerrainOffset(void);

// fuse optical flow measurements into the main filter
void ekf_FuseOptFlow(void);

// Check arm status and perform required checks and mode changes
void ekf_performArmingChecks(void);

// determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
bool ekf_getTakeoffExpected(void);

// determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
bool ekf_getTouchdownExpected(void);

// Assess GPS data quality and return true if good enough to align the EKF
bool ekf_calcGpsGoodToAlign(void);

// Read the range finder and take new measurements if available
// Apply a median filter to range finder data
void ekf_readRangeFinder(void);

// check if the vehicle has taken off during optical flow navigation by looking at inertial and range finder data
void ekf_detectOptFlowTakeoff(void);

// align the NE earth magnetic field states with the published declination
void ekf_alignMagStateDeclination(void);

// Check for signs of bad gyro health before flight
bool ekf_checkGyroHealthPreFlight(void);

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
void ekf_calcGpsGoodForFlight(void);

void ekf_updateGroundEffectDetector(void);

// vehicle specific initial gyro bias uncertainty
float ekf_InitialGyroBiasUncertainty(void);