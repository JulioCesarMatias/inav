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

#include "io/gps.h"

#define RANGE_FINDER_MAX_CM 700             // Maximum distance in centimeters that rangefinder can reliably read
#define RANGE_FINDER_GROUND_CLEARANCE_CM 10 // Distance (in cm) from the range finder to the ground

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS (1 << 0)
#define MASK_GPS_HDOP (1 << 1)
#define MASK_GPS_SPD_ERR (1 << 2)
#define MASK_GPS_POS_ERR (1 << 3)
#define MASK_GPS_YAW_ERR (1 << 4)
#define MASK_GPS_POS_DRIFT (1 << 5)
#define MASK_GPS_VERT_SPD (1 << 6)
#define MASK_GPS_HORIZ_SPD (1 << 7)

// copter defaults
#define VELNE_NOISE_DEFAULT 0.5f
#define VELD_NOISE_DEFAULT 0.7f
#define POSNE_NOISE_DEFAULT 0.5f
#define ALT_NOISE_DEFAULT 2.0f
#define MAG_NOISE_DEFAULT 0.05f
#define GYRO_PNOISE_DEFAULT 0.015f
#define ACC_PNOISE_DEFAULT 0.25f
#define GBIAS_PNOISE_DEFAULT 1E-06f
#define ABIAS_PNOISE_DEFAULT 0.00005f
#define MAGE_PNOISE_DEFAULT 0.0006f
#define MAGB_PNOISE_DEFAULT 0.0006f
#define VEL_GATE_DEFAULT 5
#define POS_GATE_DEFAULT 10
#define HGT_GATE_DEFAULT 10
#define MAG_GATE_DEFAULT 3
#define MAG_CAL_DEFAULT 3
#define GLITCH_ACCEL_DEFAULT 100
#define GLITCH_RADIUS_DEFAULT 25
#define FLOW_MEAS_DELAY 10
#define FLOW_NOISE_DEFAULT 0.25f
#define FLOW_GATE_DEFAULT 3

/*
// generic defaults (and for plane)
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       0.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.5f
#define GBIAS_PNOISE_DEFAULT    8E-06f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        30
#define HGT_GATE_DEFAULT        20
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         0
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   20
#define FLOW_MEAS_DELAY         25
#define FLOW_NOISE_DEFAULT      0.3f
#define FLOW_GATE_DEFAULT       3
*/

// EKF User Tunable Parameters
typedef struct
{
    uint8_t _enable;              // zero to disable EKF
    float _gpsHorizVelNoise;      // GPS horizontal velocity measurement noise : m/s
    float _gpsVertVelNoise;       // GPS vertical velocity measurement noise : m/s
    float _gpsHorizPosNoise;      // GPS horizontal position measurement noise m
    float _baroAltNoise;          // Baro height measurement noise : m^2
    float _magNoise;              // magnetometer measurement noise : gauss
    float _easNoise;              // equivalent airspeed measurement noise : m/s
    float _windVelProcessNoise;   // wind velocity state process noise : m/s^2
    float _wndVarHgtRateScale;    // scale factor applied to wind process noise due to height rate
    float _magEarthProcessNoise;  // earth magnetic field process noise : gauss/sec
    float _magBodyProcessNoise;   // earth magnetic field process noise : gauss/sec
    float _gyrNoise;              // gyro process noise : rad/s
    float _accNoise;              // accelerometer process noise : m/s^2
    float _gyroBiasProcessNoise;  // gyro bias state process noise : rad/s
    float _accelBiasProcessNoise; // accel bias state process noise : m/s^2
    uint16_t _msecVelDelay;       // effective average delay of GPS velocity measurements rel to IMU (msec)
    uint16_t _msecPosDelay;       // effective average delay of GPS position measurements rel to (msec)
    uint8_t _fusionModeGPS;       // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity
    uint8_t _gpsVelInnovGate;     // Number of standard deviations applied to GPS velocity innovation consistency check
    uint8_t _gpsPosInnovGate;     // Number of standard deviations applied to GPS position innovation consistency check
    uint8_t _hgtInnovGate;        // Number of standard deviations applied to height innovation consistency check
    uint8_t _magInnovGate;        // Number of standard deviations applied to magnetometer innovation consistency check
    uint8_t _tasInnovGate;        // Number of standard deviations applied to true airspeed innovation consistency check
    uint8_t _magCal;              // Sets activation condition for in-flight magnetometer calibration
    uint16_t _gpsGlitchAccelMax;  // Maximum allowed discrepancy between inertial and GPS Horizontal acceleration before GPS data is ignored : cm/s^2
    uint8_t _gpsGlitchRadiusMax;  // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    uint8_t _gndGradientSigma;    // RMS terrain gradient percentage assumed by the terrain height estimation.
    float _flowNoise;             // optical flow rate measurement noise
    uint8_t _flowInnovGate;       // Number of standard deviations applied to optical flow innovation consistency check
    uint8_t _msecFLowDelay;       // effective average delay of optical flow measurements rel to IMU (msec)
    uint8_t _rngInnovGate;        // Number of standard deviations applied to range finder innovation consistency check
    float _maxFlowRate;           // Maximum flow rate magnitude that will be accepted by the filter
    uint8_t _altSource;           // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder.
    uint8_t _gpsCheck;            // Bitmask controlling which preflight GPS checks are bypassed
} ekf_param_t;

typedef union
{
    struct
    {
        uint16_t attitude : 1;           // 0 - true if attitude estimate is valid
        uint16_t horiz_vel : 1;          // 1 - true if horizontal velocity estimate is valid
        uint16_t vert_vel : 1;           // 2 - true if the vertical velocity estimate is valid
        uint16_t horiz_pos_rel : 1;      // 3 - true if the relative horizontal position estimate is valid
        uint16_t horiz_pos_abs : 1;      // 4 - true if the absolute horizontal position estimate is valid
        uint16_t vert_pos : 1;           // 5 - true if the vertical position estimate is valid
        uint16_t terrain_alt : 1;        // 6 - true if the terrain height estimate is valid
        uint16_t const_pos_mode : 1;     // 7 - true if we are in const position mode
        uint16_t pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
        uint16_t pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
        uint16_t takeoff_detected : 1;   // 10 - true if optical flow takeoff has been detected
        uint16_t takeoff : 1;            // 11 - true if filter is compensating for baro errors during takeoff
        uint16_t touchdown : 1;          // 12 - true if filter is compensating for baro errors during touchdown
        uint16_t using_gps : 1;          // 13 - true if we are using GPS position
        uint16_t gps_glitching : 1;      // 14 - true if the the GPS is glitching
    } flags;
    uint16_t value;
} nav_filter_status;

typedef union
{
    struct
    {
        uint16_t bad_sAcc : 1;        // 0 - true if reported gps speed accuracy is insufficient to start using GPS
        uint16_t bad_hAcc : 1;        // 1 - true if reported gps horizontal position accuracy is insufficient to start using GPS
        uint16_t bad_yaw : 1;         // 2 - true if EKF yaw errors are too large to start using GPS
        uint16_t bad_sats : 1;        // 3 - true if the number of satellites is insufficient to start using GPS
        uint16_t bad_VZ : 1;          // 4 - true if the vertical velocity is inconsistent with the inertial/baro
        uint16_t bad_horiz_drift : 1; // 5 - true if the GPS horizontal position is drifting (this check assumes vehicle is static)
        uint16_t bad_hdop : 1;        // 6 - true if the reported HDoP is insufficient to start using GPS
        uint16_t bad_vert_vel : 1;    // 7 - true if the GPS vertical speed is too large to start using GPS (this check assumes vehicle is static)
        uint16_t bad_fix : 1;         // 8 - true if the GPS is not providing a 3D fix
        uint16_t bad_horiz_vel : 1;   // 9 - true if the GPS horizontal speed is excessive (this check assumes the vehicle is static)
    } flags;
    uint16_t value;
} nav_gps_status;

typedef float ftype;
typedef ftype Vector2[2];
typedef ftype Vector3[3];
typedef ftype Vector4[4];
typedef ftype Vector5[5];
typedef ftype Vector6[6];
typedef ftype Vector8[8];
typedef ftype Vector9[9];
typedef ftype Vector10[10];
typedef ftype Vector11[11];
typedef ftype Vector13[13];
typedef ftype Vector14[14];
typedef ftype Vector15[15];
typedef ftype Vector22[22];
typedef ftype Vector31[31];
typedef ftype Vector34[34];
typedef ftype Matrix3[3][3];
typedef ftype Matrix22[22][22];
typedef ftype Matrix34_50[34][50];
typedef uint32_t Vector_u32_50[50];

typedef union
{
    float v[2];
    struct
    {
        float x, y;
    };
} fpVector2_t;

bool ekf_InitialiseFilterDynamic(void);

// Update Filter States - this should be called whenever new IMU data is available
void ekf_UpdateFilter(void);

// Check basic filter health metrics and return a consolidated health status
bool ekf_healthy(void);

// Write the last calculated NE position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool ekf_getPosNED(fpVector3_t *pos);

// return NED velocity in m/s
void ekf_getVelNED(fpVector3_t *vel);

// Return the rate of change of vertical position in the down diection (dPosD/dt) in m/s
// This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
// but will always be kinematically consistent with the z component of the EKF position state
float ekf_getPosDownDerivative(void);

// This returns the specific forces in the NED frame
void ekf_getAccelNED(fpVector3_t *accelNED);

// return body axis gyro bias estimates in rad/sec
void ekf_getGyroBias(fpVector3_t *gyroBias);

// reset body axis gyro bias estimates
void ekf_resetGyroBias(void);

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool ekf_resetHeightDatum(void);

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void ekf_getEkfControlLimits(float *ekfGndSpdLimit, float *ekfNavVelGainScaler);

// return weighting of first IMU in blending function
void ekf_getIMU1Weighting(float *ret);

// return the individual Z-accel bias estimates in m/s^2
void ekf_getAccelZBias(float *zbias1, float *zbias2);

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void ekf_getWind(fpVector2_t *wind);

// return earth magnetic field estimates in measurement units / 1000
void ekf_getMagNED(fpVector3_t *magNED);

// return body magnetic field estimates in measurement units / 1000
void ekf_getMagXYZ(fpVector3_t *magXYZ);

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool ekf_getMagOffsets(fpVector3_t *magOffsets);

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool ekf_getLLH(gpsLocation_t *loc);

// return the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool ekf_getOriginLLH(gpsLocation_t *loc);

// set the latitude and longitude and height used to set the NED origin
// All NED positions calcualted by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool ekf_setOriginLLH(gpsLocation_t loc);

// return estimated height above ground level
// return false if ground height is not being estimated.
bool ekf_getHAGL(float *HAGL);

// return the Euler roll, pitch and yaw angle in radians
void ekf_getEulerAngles(fpVector3_t *eulers);

// return the transformation matrix from XYZ (body) to NED axes
void ekf_getRotationBodyToNED(fpMat3_t *mat);

// return the quaternions defining the rotation from NED to XYZ (body) axes
void ekf_getQuaternion(fpQuaternion_t *quat);

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void ekf_getInnovations(fpVector3_t *velInnov, fpVector3_t *posInnov, fpVector3_t *magInnov, float *tasInnov);

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void ekf_getVariances(float *velVar, float *posVar, float *hgtVar, fpVector3_t *magVar, float *tasVar, fpVector2_t *offset);

// should we use the compass? This is public so it can be used for
// reporting via ahrs.use_compass()
bool ekf_useCompass(void);

// write the raw optical flow measurements
// rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
// rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
// rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
// The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
void ekf_writeOptFlowMeas(uint8_t rawFlowQuality, fpVector2_t rawFlowRates, fpVector2_t rawGyroRates);

// return data for debugging optical flow fusion
void ekf_getFlowDebug(float *varFlow, float *gndOffset, float *flowInnovX, float *flowInnovY, float *auxInnov, float *HAGL, float *rngInnov, float *range, float *gndOffsetErr);

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void ekf_setTakeoffExpected(bool val);

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void ekf_setTouchdownExpected(bool val);

/*
return the filter fault status as a bitmasked integer
 0 = quaternions are NaN
 1 = velocities are NaN
 2 = badly conditioned X magnetometer fusion
 3 = badly conditioned Y magnetometer fusion
 4 = badly conditioned Z magnetometer fusion
 5 = badly conditioned airspeed fusion
 6 = badly conditioned synthetic sideslip fusion
 7 = filter is not initialised
*/
void ekf_getFilterFaults(uint16_t *faults);

/*
return filter timeout status as a bitmasked integer
 0 = position measurement timeout
 1 = velocity measurement timeout
 2 = height measurement timeout
 3 = magnetometer measurement timeout
 4 = true airspeed measurement timeout
 5 = unassigned
 6 = unassigned
 7 = unassigned
*/
void ekf_getFilterTimeouts(uint8_t *timeouts);

// return filter status flags
void ekf_getFilterStatus(nav_filter_status *status);

// return filter gps quality check status
void ekf_getFilterGpsStatus(nav_gps_status *status);

void ekf_send_status_report(void);

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool ekf_getHeightControlLimit(float *height);

// returns true of the EKF thinks the GPS is glitching
bool ekf_getGpsGlitchStatus(void);

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t ekf_getLastYawResetAngle(float *yawAng);

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t ekf_getLastPosNorthEastReset(fpVector2_t *pos);

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t ekf_getLastVelNorthEastReset(fpVector2_t *vel);

// report any reason for why the backend is refusing to initialise
const char *ekf_prearm_failure_reason(void);

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

// fuse true airspeed measurements
void ekf_FuseAirspeed(void);

// fuse sythetic sideslip measurement of zero
void ekf_FuseSideslip(void);

// zero specified range of rows in the state covariance matrix
void ekf_zeroRows(Matrix22 covMat, uint8_t first, uint8_t last);

// zero specified range of columns in the state covariance matrix
void ekf_zeroCols(Matrix22 covMat, uint8_t first, uint8_t last);

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

// determine when to perform fusion of true airspeed measurements
void ekf_SelectTasFusion(void);

// determine when to perform fusion of synthetic sideslp measurements
void ekf_SelectBetaFusion(void);

// determine when to perform fusion of magnetometer measurements
void ekf_SelectMagFusion(void);

// force alignment of the yaw angle using GPS velocity data
void ekf_alignYawGPS(void);

// Forced alignment of the wind velocity states so that they are set to the reciprocal of
// the ground speed and scaled to 6 m/s. This is used when launching a fly-forward vehicle without an airspeed sensor
// on the assumption that launch will be into wind and 6 is representative global average at height
// http://maps.google.com/gallery/details?id=zJuaSgXp_WLc.kTBytKPmNODY&hl=en
void ekf_setWindVelStates(void);

// initialise the earth magnetic field states using declination and current attitude and magnetometer meaasurements
// and return attitude quaternion
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

// Set the NED origin to be used until the next filter reset
void ekf_setOrigin(void);

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

// vehicle specific initial gyro bias uncertainty
float ekf_InitialGyroBiasUncertainty(void);