/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/utils.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/scurve.h"
#include "navigation/sqrt_controller.h"

#include "scheduler/scheduler.h"

#include "sensors/battery.h"

/*-----------------------------------------------------------
 * Altitude controller for multicopter aircraft
 *-----------------------------------------------------------*/

static int16_t rcCommandAdjustedThrottle;
static int16_t altHoldThrottleRCZero = 1500;
static pt1Filter_t altholdThrottleFilterState;
static bool prepareForTakeoffOnReset = false;
static sqrt_controller_t alt_hold_sqrt_controller;

float getSqrtControllerVelocity(float targetAltitude, timeDelta_t deltaMicros)
{
    return sqrtControllerApply(
            &alt_hold_sqrt_controller,
            targetAltitude,
            navGetCurrentActualPositionAndVelocity()->pos.z,
            US2S(deltaMicros),
            SQRT_CONTROLLER_POS_Z
    );
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityController_MC(timeDelta_t deltaMicros)
{
    float targetVel = getDesiredClimbRate(posControl.desiredState.pos.z, deltaMicros);

    posControl.pids.pos[Z].output_constrained = targetVel;      // only used for Blackbox and OSD info

    // Limit max up/down acceleration target
    const float smallVelChange = US2S(deltaMicros) * (GRAVITY_CMSS * 0.1f);
    const float velTargetChange = targetVel - posControl.desiredState.vel.z;

    if (velTargetChange <= -smallVelChange) {
        // Large & Negative - acceleration is _down_. We can't reach more than -1G in any possible condition. Hard limit to 0.8G to stay safe
        // This should be safe enough for stability since we only reduce throttle
        const float maxVelDifference = US2S(deltaMicros) * (GRAVITY_CMSS * 0.8f);
        posControl.desiredState.vel.z = constrainf(targetVel, posControl.desiredState.vel.z - maxVelDifference, posControl.desiredState.vel.z + maxVelDifference);
    }
    else if (velTargetChange >= smallVelChange) {
        // Large and positive - acceleration is _up_. We are limited by thrust/weight ratio which is usually about 2:1 (hover around 50% throttle).
        // T/W ratio = 2 means we are able to reach 1G acceleration in "UP" direction. Hard limit to 0.5G to be on a safe side and avoid abrupt throttle changes
        const float maxVelDifference = US2S(deltaMicros) * (GRAVITY_CMSS * 0.5f);
        posControl.desiredState.vel.z = constrainf(targetVel, posControl.desiredState.vel.z - maxVelDifference, posControl.desiredState.vel.z + maxVelDifference);
    }
    else {
        // Small - desired acceleration is less than 0.1G. We should be safe setting velocity target directly - any platform should be able to satisfy this
        posControl.desiredState.vel.z = targetVel;
    }

    navDesiredVelocity[Z] = constrain(lrintf(posControl.desiredState.vel.z), -32678, 32767);
}

static void updateAltitudeThrottleController_MC(timeDelta_t deltaMicros)
{
    // Calculate min and max throttle boundaries (to compensate for integral windup)
    const int16_t thrCorrectionMin = getThrottleIdleValue() - currentBatteryProfile->nav.mc.hover_throttle;
    const int16_t thrCorrectionMax = getMaxThrottle() - currentBatteryProfile->nav.mc.hover_throttle;

    float velocity_controller = navPidApply2(&posControl.pids.vel[Z], posControl.desiredState.vel.z, navGetCurrentActualPositionAndVelocity()->vel.z, US2S(deltaMicros), thrCorrectionMin, thrCorrectionMax, 0);

    int16_t rcThrottleCorrection = pt1FilterApply4(&altholdThrottleFilterState, velocity_controller, NAV_THROTTLE_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));
    rcThrottleCorrection = constrain(rcThrottleCorrection, thrCorrectionMin, thrCorrectionMax);

    posControl.rcAdjustment[THROTTLE] = setDesiredThrottle(currentBatteryProfile->nav.mc.hover_throttle + rcThrottleCorrection, false);
}

bool adjustMulticopterAltitudeFromRCInput(void)
{
    if (posControl.flags.isTerrainFollowEnabled) {
        const float altTarget = scaleRangef(rcCommand[THROTTLE], getThrottleIdleValue(), getMaxThrottle(), 0, navConfig()->general.max_terrain_follow_altitude);

        // In terrain follow mode we apply different logic for terrain control
        if (posControl.flags.estAglStatus == EST_TRUSTED && altTarget > 10.0f) {
            // We have solid terrain sensor signal - directly map throttle to altitude
            updateClimbRateToAltitudeController(0, altTarget, ROC_TO_ALT_TARGET);
        }
        else {
            updateClimbRateToAltitudeController(-50.0f, 0, ROC_TO_ALT_CONSTANT);
        }

        // In surface tracking we always indicate that we're adjusting altitude
        return true;
    }
    else {
        const int16_t rcThrottleAdjustment = applyDeadbandRescaled(rcCommand[THROTTLE] - altHoldThrottleRCZero, rcControlsConfig()->alt_hold_deadband, -500, 500);

        if (rcThrottleAdjustment) {
            // set velocity proportional to stick movement
            float rcClimbRate;

            // Make sure we can satisfy max_manual_climb_rate in both up and down directions
            if (rcThrottleAdjustment > 0) {
                // Scaling from altHoldThrottleRCZero to maxthrottle
                rcClimbRate = rcThrottleAdjustment * navConfig()->mc.max_manual_climb_rate / (float)(getMaxThrottle() - altHoldThrottleRCZero - rcControlsConfig()->alt_hold_deadband);
            }
            else {
                // Scaling from minthrottle to altHoldThrottleRCZero
                rcClimbRate = rcThrottleAdjustment * navConfig()->mc.max_manual_climb_rate / (float)(altHoldThrottleRCZero - getThrottleIdleValue() - rcControlsConfig()->alt_hold_deadband);
            }

            updateClimbRateToAltitudeController(rcClimbRate, 0, ROC_TO_ALT_CONSTANT);

            return true;
        }
        else {
            // Adjusting finished - reset desired position to stay exactly where pilot released the stick
            if (posControl.flags.isAdjustingAltitude) {
                updateClimbRateToAltitudeController(0, 0, ROC_TO_ALT_CURRENT);
            }

            return false;
        }
    }
}

void setupMulticopterAltitudeController(void)
{
    const bool throttleIsLow = throttleStickIsLow();
    const uint8_t throttleType = navConfig()->mc.althold_throttle_type;

    if (throttleType == MC_ALT_HOLD_STICK && !throttleIsLow) {
        // Only use current throttle if not LOW - use Thr Mid otherwise
        altHoldThrottleRCZero = rcCommand[THROTTLE];
    } else if (throttleType == MC_ALT_HOLD_HOVER) {
        altHoldThrottleRCZero = currentBatteryProfile->nav.mc.hover_throttle;
    } else {
        altHoldThrottleRCZero = rcLookupThrottleMid();
    }

    // Make sure we are able to satisfy the deadband
    altHoldThrottleRCZero = constrain(altHoldThrottleRCZero,
                                      getThrottleIdleValue() + rcControlsConfig()->alt_hold_deadband + 10,
                                      getMaxThrottle() - rcControlsConfig()->alt_hold_deadband - 10);

    // Force AH controller to initialize althold integral for pending takeoff on reset
    // Signal for that is low throttle _and_ low actual altitude
    if (throttleIsLow && fabsf(navGetCurrentActualPositionAndVelocity()->pos.z) <= 50.0f) {
        prepareForTakeoffOnReset = true;
    }
}

void resetMulticopterAltitudeController(void)
{
    const navEstimatedPosVel_t *posToUse = navGetCurrentActualPositionAndVelocity();
    float nav_speed_up = 0.0f;
    float nav_speed_down = 0.0f;
    float nav_accel_z = 0.0f;

    navPidReset(&posControl.pids.vel[Z]);
    navPidReset(&posControl.pids.surface);

    posControl.rcAdjustment[THROTTLE] = currentBatteryProfile->nav.mc.hover_throttle;

    posControl.desiredState.vel.z = posToUse->vel.z;   // Gradually transition from current climb

    pt1FilterReset(&altholdThrottleFilterState, 0.0f);
    pt1FilterReset(&posControl.pids.vel[Z].error_filter_state, 0.0f);
    pt1FilterReset(&posControl.pids.vel[Z].dterm_filter_state, 0.0f);

    if (FLIGHT_MODE(FAILSAFE_MODE) || FLIGHT_MODE(NAV_RTH_MODE) || FLIGHT_MODE(NAV_WP_MODE) || navigationIsExecutingAnEmergencyLanding()) {
        nav_speed_up = navConfig()->mc.max_auto_climb_rate;
        nav_accel_z = navConfig()->mc.max_auto_climb_rate;
        nav_speed_down = navConfig()->mc.max_auto_climb_rate;
    } else {
        nav_speed_up = navConfig()->mc.max_manual_climb_rate;
        nav_accel_z = navConfig()->mc.max_manual_climb_rate;
        nav_speed_down = navConfig()->mc.max_manual_climb_rate;
    }

    sqrtControllerInit(
        &alt_hold_sqrt_controller,
        posControl.pids.pos[Z].param.kP,
        -fabsf(nav_speed_down),
        nav_speed_up,
        nav_accel_z,
        SQRT_CONTROLLER_POS_Z
    );
}

static void applyMulticopterAltitudeController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate = 0;     // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)

    // If we have an update on vertical position data - update velocity and accel targets
    if (posControl.flags.verticalPositionDataNew) {
        const timeDeltaLarge_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTimeUs;

        // Check if last correction was not too long ago
        if (deltaMicrosPositionUpdate < MAX_POSITION_UPDATE_INTERVAL_US) {
            // If we are preparing for takeoff - start with lowset possible climb rate, adjust alt target and make sure throttle doesn't jump
            if (prepareForTakeoffOnReset) {
                const navEstimatedPosVel_t *posToUse = navGetCurrentActualPositionAndVelocity();

                posControl.desiredState.vel.z = -navConfig()->mc.max_manual_climb_rate;
                posControl.desiredState.pos.z = posToUse->pos.z - (navConfig()->mc.max_manual_climb_rate / posControl.pids.pos[Z].param.kP);
                posControl.pids.vel[Z].integrator = -500.0f;
                pt1FilterReset(&altholdThrottleFilterState, -500.0f);
                prepareForTakeoffOnReset = false;
            }

            // Execute actual altitude controllers
            updateAltitudeVelocityController_MC(deltaMicrosPositionUpdate);
            updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
        }
        else {
            // Position update has not occurred in time (first start or glitch), reset altitude controller
            resetMulticopterAltitudeController();
        }

        // Indicate that information is no longer usable
        posControl.flags.verticalPositionDataConsumed = true;
    }

    // Update throttle controller
    rcCommand[THROTTLE] = posControl.rcAdjustment[THROTTLE];

    // Save processed throttle for future use
    rcCommandAdjustedThrottle = rcCommand[THROTTLE];
}

/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
bool adjustMulticopterHeadingFromRCInput(void)
{
    if (ABS(rcCommand[YAW]) > rcControlsConfig()->pos_hold_deadband) {
        // Heading during Cruise Hold mode set by Nav function so no adjustment required here
        if (!FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {
            posControl.desiredState.yaw = posControl.actualState.yaw;
        }

        return true;
    }

    return false;
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static float lastAccelTargetX = 0.0f, lastAccelTargetY = 0.0f;

void resetMulticopterBrakingMode(void)
{
    DISABLE_STATE(NAV_CRUISE_BRAKING);
    DISABLE_STATE(NAV_CRUISE_BRAKING_BOOST);
    DISABLE_STATE(NAV_CRUISE_BRAKING_LOCKED);
}

static void processMulticopterBrakingMode(const bool isAdjusting)
{
#ifdef USE_MR_BRAKING_MODE
    static uint32_t brakingModeDisengageAt = 0;
    static uint32_t brakingBoostModeDisengageAt = 0;

    if (!(NAV_Status.state == MW_NAV_STATE_NONE || NAV_Status.state == MW_NAV_STATE_HOLD_INFINIT)) {
        resetMulticopterBrakingMode();
        return;
    }

    const bool brakingEntryAllowed =
        IS_RC_MODE_ACTIVE(BOXBRAKING) &&
        !STATE(NAV_CRUISE_BRAKING_LOCKED) &&
        posControl.actualState.velXY > navConfig()->mc.braking_speed_threshold &&
        !isAdjusting &&
        navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE &&
        navConfig()->mc.braking_speed_threshold > 0;


    /*
     * Case one, when we order to brake (sticks to the center) and we are moving above threshold
     * Speed is above 1m/s and sticks are centered
     * Extra condition: BRAKING flight mode has to be enabled
     */
    if (brakingEntryAllowed) {
        /*
         * Set currnt position and target position
         * Enabling NAV_CRUISE_BRAKING locks other routines from setting position!
         */
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, 0, NAV_POS_UPDATE_XY);

        ENABLE_STATE(NAV_CRUISE_BRAKING_LOCKED);
        ENABLE_STATE(NAV_CRUISE_BRAKING);

        //Set forced BRAKING disengage moment
        brakingModeDisengageAt = millis() + navConfig()->mc.braking_timeout;

        //If speed above threshold, start boost mode as well
        if (posControl.actualState.velXY > navConfig()->mc.braking_boost_speed_threshold) {
            ENABLE_STATE(NAV_CRUISE_BRAKING_BOOST);

            brakingBoostModeDisengageAt = millis() + navConfig()->mc.braking_boost_timeout;
        }

    }

    // We can enter braking only after user started to move the sticks
    if (STATE(NAV_CRUISE_BRAKING_LOCKED) && isAdjusting) {
        DISABLE_STATE(NAV_CRUISE_BRAKING_LOCKED);
    }

    /*
     * Case when speed dropped, disengage BREAKING_BOOST
     */
    if (
        STATE(NAV_CRUISE_BRAKING_BOOST) && (
            posControl.actualState.velXY <= navConfig()->mc.braking_boost_disengage_speed ||
            brakingBoostModeDisengageAt < millis()
    )) {
        DISABLE_STATE(NAV_CRUISE_BRAKING_BOOST);
    }

    /*
     * Case when we were braking but copter finally stopped or we started to move the sticks
     */
    if (STATE(NAV_CRUISE_BRAKING) && (
        posControl.actualState.velXY <= navConfig()->mc.braking_disengage_speed ||  //We stopped
        isAdjusting ||                                                              //Moved the sticks
        brakingModeDisengageAt < millis()                                           //Braking is done to timed disengage
    )) {
        DISABLE_STATE(NAV_CRUISE_BRAKING);
        DISABLE_STATE(NAV_CRUISE_BRAKING_BOOST);

        /*
         * When braking is done, store current position as desired one
         * We do not want to go back to the place where braking has started
         */
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, 0, NAV_POS_UPDATE_XY);
    }
#else
    UNUSED(isAdjusting);
#endif
}

static bool adjustMulticopterCruiseSpeed(int16_t rcPitchAdjustment)
{
    static timeMs_t lastUpdateTimeMs;
    const timeMs_t currentTimeMs = millis();
    const timeMs_t updateDeltaTimeMs = currentTimeMs - lastUpdateTimeMs;
    lastUpdateTimeMs = currentTimeMs;

    const float rcVelX = rcPitchAdjustment * navConfig()->general.max_manual_speed / (float)(500 - rcControlsConfig()->pos_hold_deadband);

    if (rcVelX > posControl.cruise.multicopterSpeed) {
        posControl.cruise.multicopterSpeed = rcVelX;
    } else if (rcVelX < 0 && updateDeltaTimeMs < 100) {
        posControl.cruise.multicopterSpeed += MS2S(updateDeltaTimeMs) * rcVelX / 2.0f;
    } else {
        return false;
    }
    posControl.cruise.multicopterSpeed = constrainf(posControl.cruise.multicopterSpeed, 10.0f, navConfig()->general.max_manual_speed);

    return true;
}

static void setMulticopterStopPosition(void)
{
    fpVector3_t stopPosition;
    calculateMulticopterInitialHoldPosition(&stopPosition);
    setDesiredPosition(&stopPosition, 0, NAV_POS_UPDATE_XY);
}

bool adjustMulticopterPositionFromRCInput(int16_t rcPitchAdjustment, int16_t rcRollAdjustment)
{
    if (navGetMappedFlightModes(posControl.navState) & NAV_COURSE_HOLD_MODE) {
        if (rcPitchAdjustment) {
            return adjustMulticopterCruiseSpeed(rcPitchAdjustment);
        }

        return false;
    }

    // Process braking mode
    processMulticopterBrakingMode(rcPitchAdjustment || rcRollAdjustment);

    // Actually change position
    if (rcPitchAdjustment || rcRollAdjustment) {
        // If mode is GPS_CRUISE, move target position, otherwise POS controller will passthru the RC input to ANGLE PID
        if (navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE) {
            const float rcVelX = rcPitchAdjustment * navConfig()->general.max_manual_speed / (float)(500 - rcControlsConfig()->pos_hold_deadband);
            const float rcVelY = rcRollAdjustment * navConfig()->general.max_manual_speed / (float)(500 - rcControlsConfig()->pos_hold_deadband);

            // Rotate these velocities from body frame to to earth frame
            const float neuVelX = rcVelX * posControl.actualState.cosYaw - rcVelY * posControl.actualState.sinYaw;
            const float neuVelY = rcVelX * posControl.actualState.sinYaw + rcVelY * posControl.actualState.cosYaw;

            // Calculate new position target, so Pos-to-Vel P-controller would yield desired velocity
            posControl.desiredState.pos.x = navGetCurrentActualPositionAndVelocity()->pos.x + (neuVelX / posControl.pids.pos[X].param.kP);
            posControl.desiredState.pos.y = navGetCurrentActualPositionAndVelocity()->pos.y + (neuVelY / posControl.pids.pos[Y].param.kP);
        }

        return true;
    }
    else if (posControl.flags.isAdjustingPosition) {
        // Adjusting finished - reset desired position to stay exactly where pilot released the stick
        setMulticopterStopPosition();
    }

    return false;
}

static float getVelocityHeadingAttenuationFactor(void)
{
    // In WP mode scale velocity if heading is different from bearing
    if (navConfig()->mc.slowDownForTurning && (navGetCurrentStateFlags() & NAV_AUTO_WP)) {
        const int32_t headingError = constrain(wrap_18000(posControl.desiredState.yaw - posControl.actualState.yaw), -9000, 9000);
        const float velScaling = cos_approx(CENTIDEGREES_TO_RADIANS(headingError));

        return constrainf(velScaling * velScaling, 0.05f, 1.0f);
    } else {
        return 1.0f;
    }
}

static float getVelocityExpoAttenuationFactor(float velTotal, float velMax)
{
    // Calculate factor of how velocity with applied expo is different from unchanged velocity
    const float velScale = constrainf(velTotal / velMax, 0.01f, 1.0f);

    // navConfig()->max_speed * ((velScale * velScale * velScale) * posControl.posResponseExpo + velScale * (1 - posControl.posResponseExpo)) / velTotal;
    // ((velScale * velScale * velScale) * posControl.posResponseExpo + velScale * (1 - posControl.posResponseExpo)) / velScale
    // ((velScale * velScale) * posControl.posResponseExpo + (1 - posControl.posResponseExpo));
    return 1.0f - posControl.posResponseExpo * (1.0f - (velScale * velScale));  // x^3 expo factor
}

static void updatePositionVelocityController_MC(const float maxSpeed)
{
    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {
        // Position held at cruise speeds below 0.5 m/s, otherwise desired neu velocities set directly from cruise speed
        if (posControl.cruise.multicopterSpeed >= 50) {
            // Rotate multicopter x velocity from body frame to earth frame
            posControl.desiredState.vel.x = posControl.cruise.multicopterSpeed * cos_approx(CENTIDEGREES_TO_RADIANS(posControl.cruise.course));
            posControl.desiredState.vel.y = posControl.cruise.multicopterSpeed * sin_approx(CENTIDEGREES_TO_RADIANS(posControl.cruise.course));

            return;
        } else if (posControl.flags.isAdjustingPosition) {
            setMulticopterStopPosition();
        }
    }

    const float posErrorX = posControl.desiredState.pos.x - navGetCurrentActualPositionAndVelocity()->pos.x;
    const float posErrorY = posControl.desiredState.pos.y - navGetCurrentActualPositionAndVelocity()->pos.y;

    // Calculate target velocity
    float neuVelX = posErrorX * posControl.pids.pos[X].param.kP;
    float neuVelY = posErrorY * posControl.pids.pos[Y].param.kP;

    // Scale velocity to respect max_speed
    float neuVelTotal = calc_length_pythagorean_2D(neuVelX, neuVelY);

    /*
     * We override computed speed with max speed in following cases:
     * 1 - computed velocity is > maxSpeed
     * 2 - in WP mission or RTH Trackback when: slowDownForTurning is OFF, not a hold waypoint and computed speed is < maxSpeed
     */
    if (
        ((navGetCurrentStateFlags() & NAV_AUTO_WP || posControl.flags.rthTrackbackActive) &&
        !isNavHoldPositionActive() &&
        neuVelTotal < maxSpeed &&
        !navConfig()->mc.slowDownForTurning
        ) || neuVelTotal > maxSpeed
    ) {
        neuVelX = maxSpeed * (neuVelX / neuVelTotal);
        neuVelY = maxSpeed * (neuVelY / neuVelTotal);
        neuVelTotal = maxSpeed;
    }

    posControl.pids.pos[X].output_constrained = neuVelX;
    posControl.pids.pos[Y].output_constrained = neuVelY;

    // Apply expo & attenuation if heading in wrong direction - turn first, accelerate later (effective only in WP mode)
    const float velHeadFactor = getVelocityHeadingAttenuationFactor();
    const float velExpoFactor = getVelocityExpoAttenuationFactor(neuVelTotal, maxSpeed);
    posControl.desiredState.vel.x = neuVelX * velHeadFactor * velExpoFactor;
    posControl.desiredState.vel.y = neuVelY * velHeadFactor * velExpoFactor;
}

static float computeNormalizedVelocity(const float value, const float maxValue)
{
    return constrainf(scaleRangef(fabsf(value), 0, maxValue, 0.0f, 1.0f), 0.0f, 1.0f);
}

static float computeVelocityScale(
    const float value,
    const float maxValue,
    const float attenuationFactor,
    const float attenuationStart,
    const float attenuationEnd
)
{
    const float normalized = computeNormalizedVelocity(value, maxValue);

    float scale = scaleRangef(normalized, attenuationStart, attenuationEnd, 0, attenuationFactor);
    return constrainf(scale, 0, attenuationFactor);
}

typedef enum {
    POSHOLD_PILOT_OVERRIDE = 0,          // pilot is controlling this axis (i.e. roll or pitch)
    POSHOLD_BRAKE,                       // this axis is braking towards zero
    POSHOLD_BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
    POSHOLD_BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
    POSHOLD_LOITER,                      // both vehicle axis are holding position
    POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
} poshold_rp_mode;

typedef struct {
    poshold_rp_mode roll_mode           : 3;    // roll mode: pilot override, brake or loiter
    poshold_rp_mode pitch_mode          : 3;    // pitch mode: pilot override, brake or loiter
    uint8_t braking_time_updated_roll   : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
    uint8_t braking_time_updated_pitch  : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking

    // braking related variables
    float brake_gain;                         // gain used during conversion of vehicle's velocity to lean angle during braking (calculated from brake_rate)
    float brake_roll;                         // target roll angle during braking periods
    float brake_pitch;                        // target pitch angle during braking periods
    int16_t brake_timeout_roll;               // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    int16_t brake_timeout_pitch;              // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    float brake_angle_max_roll;               // maximum lean angle achieved during braking.  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    float brake_angle_max_pitch;              // maximum lean angle achieved during braking  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    int16_t brake_to_loiter_timer;            // cycles to mix brake and loiter controls in POSHOLD_BRAKE_TO_LOITER

    // loiter related variables
    int16_t controller_to_pilot_timer_roll;   // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    int16_t controller_to_pilot_timer_pitch;  // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    float controller_final_roll;              // final roll angle from controller as we exit brake or loiter mode (used for mixing with pilot input)
    float controller_final_pitch;             // final pitch angle from controller as we exit brake or loiter mode (used for mixing with pilot input)

    // wind compensation related variables
    fpVector3_t wind_comp_ef;                 // wind compensation in earth frame, filtered lean angles from position controller
    float wind_comp_roll;                     // roll angle to compensate for wind
    float wind_comp_pitch;                    // pitch angle to compensate for wind
    uint16_t wind_comp_start_timer;           // counter to delay start of wind compensation for a short time after loiter is engaged
    int8_t wind_comp_timer;                   // counter to reduce wind comp roll/pitch lean angle calcs to 10hz

    uint8_t loop_rate_factor; // used to adapt Pos-Hold params to loop_rate

    // pilot input related variables
    float pilot_roll;  // pilot requested roll angle (filtered to slow returns to zero)
    float pilot_pitch; // pilot requested roll angle (filtered to slow returns to zero)

    // final output
    float roll;   // final roll angle sent to attitude controller
    float pitch;  // final pitch angle sent to attitude controller
} poshold_t;

poshold_t poshold;

#define LOOP_RATE_FACTOR                        (poshold.loop_rate_factor)               // used to adapt Pos-Hold params to loop_rate
#define POSHOLD_BRAKE_TIME_ESTIMATE_MAX         (600 * LOOP_RATE_FACTOR)                 // max number of cycles the brake will be applied before we switch to loiter
#define POSHOLD_BRAKE_TO_LOITER_TIMER           (150 * LOOP_RATE_FACTOR)                 // number of cycles to transition from brake mode to loiter mode.
#define POSHOLD_WIND_COMP_START_TIMER           (150 * LOOP_RATE_FACTOR)                 // number of cycles to start wind compensation update after loiter is engaged
#define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER   (50 * LOOP_RATE_FACTOR)                  // set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
#define POSHOLD_WIND_COMP_TIMER_10HZ            (10 * LOOP_RATE_FACTOR)                  // counter value used to reduce wind compensation to 10hz
#define TC_WIND_COMP                            (1.0f / (float)(100 * LOOP_RATE_FACTOR)) // time constant for posHoldUpdateWindCompEstimate()
#define POSHOLD_SMOOTH_RATE_FACTOR              (TC_WIND_COMP * 5.0f)                    // filter applied to pilot's roll/pitch input as it returns to center. A lower number will cause the roll/pitch to return to zero more slowly if the brake_rate is also low.
#define POSHOLD_SPEED_0                         10                                       // speed below which it is always safe to switch to loiter
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10                                       // wind compensation estimates will only run when velocity is at or below this speed in cm/s
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE      180                                      // max angle required after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f                                  // wind compensation no more than 2/3rds of angle max to ensure pilot can always override

sqrt_controller_t sqrt_controller_pos_xy;
sqrt_controller_t sqrt_controller_shapping_angle;
sqrt_controller_t sqrt_controller_loiter_brake;
sqrt_controller_t sqrt_controller_waypoint;

pt1Filter_t pid_vel_xy_error_filter[2];
pt1Filter_t pid_vel_xy_derivative_filter[2];

scurve_t scurve_prev_leg;          // previous scurve trajectory used to blend with current scurve trajectory
scurve_t scurve_this_leg;          // current scurve trajectory
scurve_t scurve_next_leg;          // next scurve trajectory used to blend with current scurve trajectory

splineCurve_t spline_this_leg;      // spline curve for current segment
splineCurve_t spline_next_leg;      // spline curve for next segment

fpVector3_t wp_origin;              // starting point of trip to next waypoint in cm
fpVector3_t wp_destination;         // target destination in cm
fpVector3_t _predicted_accel;
fpVector3_t _desired_accel;
fpVector3_t _predicted_euler_angle;
fpVector3_t _predicted_euler_rate;
fpVector3_t _pos_target;  // TODO: replace with posControl.desiredState.pos.x and posControl.desiredState.pos.y in a final version
fpVector3_t _vel_desired; // TODO: replace with posControl.desiredState.vel.x and posControl.desiredState.vel.y in a final version
fpVector3_t _accel_desired;
fpVector3_t _vel_target;
fpVector3_t _accel_target;
fpVector3_t vel_xy_error;
fpVector3_t pid_vel_xy_integrator;
fpVector3_t vel_xy_derivative;

bool reset_pid_vel_xy_filter = true;
bool wp_this_leg_is_spline;  // true if this leg is a spline
bool wp_next_leg_is_spline;  // true if the next leg is a spline
bool wp_reached_destination; // true if we have reached the destination
bool fast_waypoint;          // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint

float _brake_accel;
float _roll_target;     // desired roll angle in degrees calculated by position controller
float _pitch_target;    // desired pitch angle in degrees calculated by position controller
float pid_vel_xy_kimax = 1000.0f; // transform in a #define macro
float wp_offset_vel;
float wp_offset_accel;
float wp_desired_speed_xy_cms;
float wp_track_scalar_dt;
float wp_scurve_snap; // scurve snap in m/s/s/s/s

float wp_speed_cms = 1000.0f; // TODO: replace with navConfig()->general.auto_speed
float wp_radius_cm = 200.0f;  // TODO: replace with navConfig()->general.waypoint_radius

timeMs_t wp_last_update;
timeMs_t brake_timer;          // system time that brake was initiated
uint32_t last_update_xy_ticks; // ticks of last last updateXYController call

// Param: ANG_LIM_TC
// Description: Angle Limit (to maintain altitude) Time Constant
// Range: 1 10
uint8_t angle_limit_tc = 1;

// Param: PHLD_BRAKE_RATE
// Description: Pos-Hold braking rate. Pos-Hold flight mode's rotation rate during braking in deg/sec
// Range: 4 12
uint8_t poshold_brake_rate = 8;

// Param: PHLD_BRAKE_ANGLE
// Description: Pos-Hold braking angle max. Pos-Hold flight mode's max lean angle during braking in degrees
// Range: 200 450
uint16_t poshold_brake_angle_max = 300;

// Param: attitude_INPUT_TC
// Description: Attitude control input time constant for Pos-Hold, Loiter and Way-Point. Low numbers lead to sharper response, higher numbers to softer response. [Multirotor only]
// Range: 0 1
// Values: 50:Very Soft, 20:Soft, 15:Medium, 10:Crisp, 5:Very Crisp
uint8_t input_tc = 15;

// Param: LOIT_MAX_S
// Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
// Units: cm/s
// Range: 20 3500
uint16_t loiter_speed_cms = 1250; // TODO: replace with navConfig()->general.max_manual_speed

// Param: LOIT_MAX_A
// Description: Maximum horizontal acceleration in cm/s/s that Pos-Hold and Loiter navigation will request.
// Range: 100 981
uint16_t loiter_max_accel_xy = 500;

// Param: attitude_accel_r_m
// Description: Pos-Hold, Loiter and Way-Point maximum acceleration in roll axis (degrees/second). [Multirotor only]
// Range: 0 1800
// Values: 0:Disabled, 300:VerySlow, 720:Slow, 1080:Medium, 1620:Fast
uint16_t accel_roll_max = 1080;

// Param: attitude_accel_p_m
// Description: Pos-Hold, Loiter and Way-Point maximum acceleration in pitch axis (degrees/second). [Multirotor only]
// Range: 0 1800
// Values: 0:Disabled, 300:VerySlow, 720:Slow, 1080:Medium, 1620:Fast
uint16_t accel_pitch_max = 1080;

// Param: LOITER_BRK_ACCEL
// Description: Loiter braking acceleration in cm/s. Higher values stop the copter more quickly when the stick is centered.
// Range: 25 250
uint16_t brake_accel = 250;

// @Param: LOITER_BRK_DELAY
// @Description: Loiter brake start delay (in seconds)
// @Range: 0 2
uint8_t brake_delay = 1;

// Param: LOITER_BRK_JERK
// Description: Loiter braking jerk in cm/s. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver.
// Range: 500 5000
uint16_t brake_jerk_max = 500;

// Param: wp_accel_xy
// Description: Defines the horizontal acceleration in cm/s/s used during missions. [Multirotor only]
// Range: 50 500
float _wp_accel_cmss = 250.0f;

// @Param: wp_accel_z
// @DisplayName: Waypoint Vertical Acceleration
// @Description: Defines the Way-Point vertical acceleration in cm/s/s used during missions. [Multirotor only]
// @Range: 50 500
float _wp_accel_z_cmss = 100.0f;

// Param: wp_jerk
// Description: Defines the horizontal jerk in m/s/s used during missions
// Range: 1 20
float _wp_jerk = 1.0f;

// @Param: wp_shap_jerk_xy
// @Description: Way-Pont jerk limit in m/s/s/s of the horizontal kinematic path generation used to determine how quickly the UAV varies the acceleration target. [Multirotor only]
// @Range: 1 20
float wp_shaping_jerk_xy = 5.0f;

// wrap an angle defined in radians to -PI ~ PI
static inline float wrap_2PI(const float radian)
{
  float res = fmodf(radian, 2.0f * M_PIf);

  if (res < 0)
  {
    res += (2.0f * M_PIf);
  }

  return res;
}

// wrap an angle defined in radians to -PI ~ PI
static inline float wrap_PI(const float radian)
{
  float res = wrap_2PI(radian);

  if (res > M_PIf)
  {
    res -= (2.0f * M_PIf);
  }

  return res;
}

// update the pilot's filtered lean angle with the latest raw input received
static void posHoldUpdatePilotLeanAngle(float *lean_angle_filtered, float lean_angle_raw)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    if ((*lean_angle_filtered > 0 && lean_angle_raw < 0) || (*lean_angle_filtered < 0 && lean_angle_raw > 0) || (fabsf(lean_angle_raw) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        *lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        if (*lean_angle_filtered > 0) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            *lean_angle_filtered -= MAX((float)*lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1, (float)poshold_brake_rate / (float)LOOP_RATE_FACTOR));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            *lean_angle_filtered = MAX(*lean_angle_filtered, lean_angle_raw);
        } else {
            *lean_angle_filtered += MAX(-(float)*lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1, (float)poshold_brake_rate / (float)LOOP_RATE_FACTOR));
            *lean_angle_filtered = MIN(*lean_angle_filtered, lean_angle_raw);
        }
    }
}

// mixes two controls based on the mix_ratio
// mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
static float posHoldMixControls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrainf(mix_ratio, 0.0f, 1.0f);

    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control;
}

// updates the brake_angle based on the vehicle's velocity and brake_gain
// brake_angle is slewed with the poshold_brake_rate and constrained by the poshold_braking_angle_max
// velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
static void posHoldUpdateBrakeAngleFromVelocity(float *brake_angle, float velocity)
{
    float lean_angle;
    float brake_rate = poshold_brake_rate;

    brake_rate /= (float)LOOP_RATE_FACTOR;

    if (brake_rate <= 1.0f) {
        brake_rate = 1.0f;
    }

    // calculate velocity-only based lean angle
    if (velocity >= 0.0f) {
        lean_angle = -poshold.brake_gain * velocity * (1.0f + 500.0f / (velocity + 60.0f));
    } else {
        lean_angle = -poshold.brake_gain * velocity * (1.0f + 500.0f / (-velocity + 60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    *brake_angle = constrainf(lean_angle, *brake_angle - brake_rate, *brake_angle + brake_rate);

    // constrain final brake_angle
    *brake_angle = constrainf(*brake_angle, -(float)poshold_brake_angle_max, (float)poshold_brake_angle_max);
}

// updates wind compensation estimate
static void posHoldUpdateWindCompEstimate(void)
{
    // check wind estimate start has not been delayed
    if (poshold.wind_comp_start_timer > 0) {
        poshold.wind_comp_start_timer--;
        return;
    }

    // check horizontal velocity is low
    if (calc_length_pythagorean_2D(navGetCurrentActualPositionAndVelocity()->vel.x, navGetCurrentActualPositionAndVelocity()->vel.y) > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // update wind compensation in earth-frame lean angles
    if (poshold.wind_comp_ef.x == 0.0f) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        poshold.wind_comp_ef.x = _accel_target.x;
    } else {
        // low pass filter the position controller's lean angle output
        poshold.wind_comp_ef.x = (1.0f - TC_WIND_COMP) * poshold.wind_comp_ef.x + TC_WIND_COMP * _accel_target.x;
    }

    if (poshold.wind_comp_ef.y == 0.0f) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        poshold.wind_comp_ef.y = _accel_target.y;
    } else {
        // low pass filter the position controller's lean angle output
        poshold.wind_comp_ef.y = (1.0f - TC_WIND_COMP) * poshold.wind_comp_ef.y + TC_WIND_COMP * _accel_target.y;
    }

    // limit acceleration
    const float accel_lim_cmss = tanf(DEGREES_TO_RADIANS(POSHOLD_WIND_COMP_LEAN_PCT_MAX * (float)navConfig()->mc.max_bank_angle)) * GRAVITY_CMSS;
    const float wind_comp_ef_len = calc_length_pythagorean_2D(poshold.wind_comp_ef.x, poshold.wind_comp_ef.y);

    if (accel_lim_cmss != 0.0f && (wind_comp_ef_len > accel_lim_cmss)) {
        poshold.wind_comp_ef.x *= accel_lim_cmss / wind_comp_ef_len;
        poshold.wind_comp_ef.y *= accel_lim_cmss / wind_comp_ef_len;
    }
}

// retrieve wind compensation angles in body frame roll and pitch angles
static void posHoldGetWindCompLeanAngles(float *roll_angle, float *pitch_angle)
{
    // reduce rate to 10hz
    poshold.wind_comp_timer++;

    if (poshold.wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }

    poshold.wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    *roll_angle = atanf((-poshold.wind_comp_ef.x * posControl.actualState.sinYaw + poshold.wind_comp_ef.y * posControl.actualState.cosYaw) / GRAVITY_CMSS) * (1800.0f / M_PI);
    *pitch_angle = atanf(-(poshold.wind_comp_ef.x * posControl.actualState.cosYaw + poshold.wind_comp_ef.y * posControl.actualState.sinYaw) / GRAVITY_CMSS) * (1800.0f / M_PI);
}

// initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
static void posHoldRollControllerToPilotOverride(void)
{
    poshold.roll_mode = POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE;
    poshold.controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

    // initialise pilot_roll to 0, wind_comp will be updated to compensate and posHoldUpdatePilotLeanAngle function shall not smooth this transition at next iteration. so 0 is the right value
    poshold.pilot_roll = 0.0f;

    // store final controller output for mixing with pilot input
    poshold.controller_final_roll = poshold.roll;
}

// initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
static void posHoldPitchControllerToPilotOverride(void)
{
    poshold.pitch_mode = POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE;
    poshold.controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and posHoldUpdatePilotLeanAngle function shall not smooth this transition at next iteration. so 0 is the right value
    poshold.pilot_pitch = 0.0f;

    // store final loiter outputs for mixing with pilot input
    poshold.controller_final_pitch = poshold.pitch;
}

// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
static float getAltHoldLeanAngleMax(float dt)
{
    // calc maximum tilt angle based on throttle
    const float thr_max = (float)(getMaxThrottle() - 1000) / 1000.0f;
    
    // filtered Alt-Hold lean angle max - used to limit lean angle when throttle is saturated using Alt-Hold
    static float althold_lean_angle_max;

    // divide by zero check
    if (thr_max == 0.0f) {
        althold_lean_angle_max = 0.0f;
        return 0.0f;
    }

    #define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX 0.8f

    const float throttle_corrected = constrainf((float)rcCommand[THROTTLE], 1000, 1999);

    const float throttle_in = (throttle_corrected - 1000.0f) / 1000.0f;

    const float _althold_lean_angle_max = acos(constrainf(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    althold_lean_angle_max = althold_lean_angle_max + (dt / (dt + ((float)angle_limit_tc))) * (_althold_lean_angle_max - althold_lean_angle_max);

    return RADIANS_TO_DEGREES(althold_lean_angle_max);
}

// convert roll, pitch lean target angles to lat/lon frame accelerations in cm/s/s
static fpVector3_t leanAnglesToAccel(const fpVector3_t att_target_euler)
{
    // rotate our roll, pitch angles into lat/lon frame
    const float sin_roll = sinf(att_target_euler.x);
    const float cos_roll = cosf(att_target_euler.x);
    const float sin_pitch = sinf(att_target_euler.y);
    const float cos_pitch = cosf(att_target_euler.y);
    const float sin_yaw = sinf(att_target_euler.z);
    const float cos_yaw = cosf(att_target_euler.z);

    fpVector3_t v_ret = { .v = { GRAVITY_CMSS * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f),
                                 GRAVITY_CMSS * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f),
                                 0.0f
    }};

    return v_ret;
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using input_tc
static float inputShapingAngle(float error_angle, float _input_tc, float accel_max, float target_ang_vel, float dt)
{
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max
    sqrt_controller_shapping_angle.kp = 1.0f / MAX(_input_tc, 0.01f);
    sqrt_controller_shapping_angle.derivative_max = accel_max;
    const float desired_ang_vel = sqrtControllerApply(&sqrt_controller_shapping_angle, error_angle, 0.0f, dt, SQRT_CONTROLLER_POS_XY);

    // Acceleration is limited directly to smooth the beginning of the curve.
    if (accel_max > 0.0f) {
        float delta_ang_vel = accel_max * dt;
        return constrainf(desired_ang_vel, target_ang_vel - delta_ang_vel, target_ang_vel + delta_ang_vel);
    }

    return desired_ang_vel;
}

static void setPilotDesiredAcceleration(float euler_roll_angle_dd, float euler_pitch_angle_dd, float dt)
{
    // Convert from centidegrees on public interface to radians
    const float euler_roll_angle = DECIDEGREES_TO_RADIANS(euler_roll_angle_dd);
    const float euler_pitch_angle = DECIDEGREES_TO_RADIANS(euler_pitch_angle_dd);

    // convert our desired attitude to an acceleration vector assuming we are not accelerating vertically
    const fpVector3_t desired_euler = { .v = { euler_roll_angle, euler_pitch_angle, DECIDEGREES_TO_RADIANS(attitude.values.yaw) }};
    const fpVector3_t desired_accel = leanAnglesToAccel(desired_euler);

    _desired_accel.x = desired_accel.x;
    _desired_accel.y = desired_accel.y;

    // difference between where we think we should be and where we want to be
    fpVector3_t angle_error = { .v = { wrap_PI(euler_roll_angle - _predicted_euler_angle.x), wrap_PI(euler_pitch_angle - _predicted_euler_angle.y), 0.0f }};

    // calculate the angular velocity that we would expect given our desired and predicted attitude
    _predicted_euler_rate.x = inputShapingAngle(wrap_PI(angle_error.x), ((float)input_tc / 100.0f), DEGREES_TO_RADIANS((float)accel_roll_max), _predicted_euler_rate.x, dt);
    _predicted_euler_rate.y = inputShapingAngle(wrap_PI(angle_error.y), ((float)input_tc / 100.0f), DEGREES_TO_RADIANS((float)accel_pitch_max), _predicted_euler_rate.y, dt);

    // update our predicted attitude based on our predicted angular velocity
    _predicted_euler_angle.x += _predicted_euler_rate.x * dt;
    _predicted_euler_angle.y += _predicted_euler_rate.y * dt;

    // convert our predicted attitude to an acceleration vector assuming we are not accelerating vertically
    const fpVector3_t predicted_euler = { .v = { _predicted_euler_angle.x, _predicted_euler_angle.y, DECIDEGREES_TO_RADIANS(attitude.values.yaw)}};
    const fpVector3_t predicted_accel = leanAnglesToAccel(predicted_euler);

    _predicted_accel.x = predicted_accel.x;
    _predicted_accel.y = predicted_accel.y;
}

// get maximum lean angle when using loiter
static float getLoiterAngleMax(void)
{
    return navConfig()->mc.max_bank_angle * (2.0f / 3.0f);
}

static void updateLoiterDesiredVelocity(float dt)
{
    // calculate a loiter speed limit which is the minimum of the value set by the max_manual_speed parameter
    const float gnd_speed_limit_cms = MAX(loiter_speed_cms, 20.0f);

    const float pilot_acceleration_max = GRAVITY_CMSS * tanf(DEGREES_TO_RADIANS(getLoiterAngleMax()));

    // get loiters desired velocity from the position controller where it is being stored.
    fpVector3_t desired_vel = { .v = { posControl.desiredState.vel.x, posControl.desiredState.vel.y, 0.0f }};

    // update the desired velocity using our predicted acceleration
    desired_vel.x += _predicted_accel.x * dt;
    desired_vel.y += _predicted_accel.y * dt;

    fpVector3_t loiter_accel_brake;

    float desired_speed = calc_length_pythagorean_2D(desired_vel.x, desired_vel.y);

    if (desired_speed != 0.0f) {
        fpVector3_t desired_vel_norm = { .v = { desired_vel.x / desired_speed, desired_vel.y / desired_speed, 0.0f }};

        // calculate a drag acceleration based on the desired speed.
        const float drag_decel = pilot_acceleration_max * desired_speed / gnd_speed_limit_cms;

        // calculate a braking acceleration if sticks are at zero
        float loiter_brake_accel = 0.0f;

        if (_desired_accel.x == 0.0f && _desired_accel.y == 0.0f) {
            if ((millis() - brake_timer) > S2MS(brake_delay)) {
                float brake_gain = ((posControl.pids.vel[X].param.kP + posControl.pids.vel[Y].param.kP) / 2.0f) * 0.5f;
                sqrt_controller_loiter_brake.kp = brake_gain;
                sqrt_controller_loiter_brake.derivative_max = (float)brake_jerk_max;
                loiter_brake_accel = constrainf(sqrtControllerApply(&sqrt_controller_loiter_brake, desired_speed, 0.0f, dt, SQRT_CONTROLLER_POS_XY), 0.0f, (float)brake_accel);
            }
        } else {
            loiter_brake_accel = 0.0f;
            brake_timer = millis();
        }

        _brake_accel += constrainf(loiter_brake_accel - _brake_accel, -((float)brake_jerk_max) * dt, (float)brake_jerk_max * dt);
        loiter_accel_brake.x = desired_vel_norm.x * _brake_accel;
        loiter_accel_brake.y = desired_vel_norm.y * _brake_accel;

        // update the desired velocity using the drag and braking accelerations
        desired_speed = MAX(desired_speed - (drag_decel + _brake_accel) * dt, 0.0f);
        desired_vel.x = desired_vel_norm.x * desired_speed;
        desired_vel.y = desired_vel_norm.y * desired_speed;
    }

    // add braking to the desired acceleration
    _desired_accel.x -= loiter_accel_brake.x;
    _desired_accel.y -= loiter_accel_brake.y;

    // Apply limit to desired velocity
    const float horizSpdDem = calc_length_pythagorean_2D(desired_vel.x, desired_vel.y);
    if (horizSpdDem > gnd_speed_limit_cms) {
        desired_vel.x = desired_vel.x * gnd_speed_limit_cms / horizSpdDem;
        desired_vel.y = desired_vel.y * gnd_speed_limit_cms / horizSpdDem;
    }

    // get loiters desired velocity from the position controller where it is being stored.
    fpVector3_t target_pos = _pos_target;

    // update the target position using our predicted velocity
    target_pos.x += (desired_vel.x * dt);
    target_pos.y += (desired_vel.y * dt);

    // send adjusted feed forward acceleration and velocity back to the Position Controller
    _pos_target.x = target_pos.x;
    _pos_target.y = target_pos.y;
    _vel_desired.x = desired_vel.x;
    _vel_desired.y = desired_vel.y;
   _accel_desired.x = _desired_accel.x;
   _accel_desired.y = _desired_accel.y;

    debug[6] = _vel_desired.x;
    debug[7] = _vel_desired.y;
}

/* limitAccelXY limits the acceleration to prioritise acceleration perpendicular to the provided velocity vector.
 Input parameters are:
    vel is the velocity vector used to define the direction acceleration limit is biased in.
    accel is the acceleration vector to be limited.
    accel_max is the maximum length of the acceleration vector after being limited.
 Returns true when accel vector has been limited.
*/
static void limitAccelXY(const fpVector3_t vel, fpVector3_t *accel, float accel_max)
{
    // check accel_max is defined
    if (accel_max <= 0.0f) {
        return;
    }

    // limit acceleration to accel_max while prioritizing cross track acceleration
    if ((sq(accel->x) + sq(accel->y)) > sq(accel_max)) {
        if (vel.x == 0.0f && vel.y == 0.0f) {
            // We do not have a direction of travel so do a simple vector length limit
            const float len = calc_length_pythagorean_2D(accel->x, accel->y);
            if ((len > accel_max) && len > 0.0f) {
                accel->x *= (accel_max / len);
                accel->y *= (accel_max / len);
            }
        } else {
            // calculate acceleration in the direction of and perpendicular to the velocity input
            const float vel_len = calc_length_pythagorean_2D(vel.x, vel.y);
            const fpVector3_t vel_input_unit = { .v = { vel.x / vel_len, vel.y / vel_len, 0.0f }};

            // acceleration in the direction of travel
            float accel_dir = vel_input_unit.x * accel->x + vel_input_unit.y * accel->y;

            // cross track acceleration
            fpVector3_t accel_cross = { .v = { accel->x - (vel_input_unit.x * accel_dir), accel->y - (vel_input_unit.x * accel_dir), 0.0f }};

            const float len = calc_length_pythagorean_2D(accel_cross.x, accel_cross.y);
            if ((len > accel_max) && len > 0.0f) {
                accel_cross.x *= (accel_max / len);
                accel_cross.y *= (accel_max / len);
                accel_dir = 0.0f;
            } else {
                float accel_max_dir = fast_fsqrtf(sq(accel_max) - (sq(accel_cross.x) + sq(accel_cross.y)));
                accel_dir = constrainf(accel_dir, -accel_max_dir, accel_max_dir);
            }

            accel->x = accel_cross.x + vel_input_unit.x * accel_dir;
            accel->y = accel_cross.y + vel_input_unit.y * accel_dir;
        }
    }
}

// returns true if the xy position controller has been run in the previous loop times
static bool isActiveXY(void)
{
    const uint32_t dt_ticks = getSchedulerTicks32() - last_update_xy_ticks;

    return dt_ticks <= 1;
}

// initialise the position controller to the current position, velocity, acceleration and attitude.
// This function is the default initialisation for any position control that provides position, velocity and acceleration.
static void initXYController(float dt)
{
    _pos_target.x = navGetCurrentActualPositionAndVelocity()->pos.x;
    _pos_target.y = navGetCurrentActualPositionAndVelocity()->pos.y;
    _vel_desired.x = navGetCurrentActualPositionAndVelocity()->vel.x;
    _vel_desired.y = navGetCurrentActualPositionAndVelocity()->vel.y;
    _vel_target.x = navGetCurrentActualPositionAndVelocity()->vel.x;
    _vel_target.y = navGetCurrentActualPositionAndVelocity()->vel.y;

    // Set desired accel to zero
    _accel_desired.x = 0.0f;
    _accel_desired.y = 0.0f;

    if (!isActiveXY()) {
        const fpVector3_t predicted_euler = { .v = { DECIDEGREES_TO_RADIANS(attitude.values.roll), DECIDEGREES_TO_RADIANS(attitude.values.pitch), 0.0f }};
        const fpVector3_t predicted_accel_target = leanAnglesToAccel(predicted_euler);
        _accel_target.x = predicted_accel_target.x;
        _accel_target.y = predicted_accel_target.y;
    }

    // limit acceleration using maximum lean angles
    float angle_max = MIN((getAltHoldLeanAngleMax(dt)), navConfig()->mc.max_bank_angle);
    float accel_max = GRAVITY_CMSS * tanf(DEGREES_TO_RADIANS(angle_max));
    
    const float accel_len = calc_length_pythagorean_2D(_accel_target.x, _accel_target.y);
    if ((accel_len > accel_max) && accel_len > 0.0f) {
        _accel_target.x *= (accel_max / accel_len);
        _accel_target.y *= (accel_max / accel_len);
    }

    // initialise I terms from lean angles
    reset_pid_vel_xy_filter = true;

    // initialise the I term to _accel_target
    pid_vel_xy_integrator.x = _accel_target.x;
    pid_vel_xy_integrator.y = _accel_target.y;
    const float len = calc_length_pythagorean_2D(pid_vel_xy_integrator.x, pid_vel_xy_integrator.y);
    if ((len > pid_vel_xy_kimax) && len > 0.0f) {
        pid_vel_xy_integrator.x *= (pid_vel_xy_kimax / len);
        pid_vel_xy_integrator.y *= (pid_vel_xy_kimax / len);
    }

    // initialise xy_controller time out
    last_update_xy_ticks = getSchedulerTicks32();
}

static void getStoppingPointXY(fpVector3_t *stopping_point)
{
    stopping_point->x = navGetCurrentActualPositionAndVelocity()->pos.x;
    stopping_point->y = navGetCurrentActualPositionAndVelocity()->pos.y;

    const fpVector3_t curr_vel = navGetCurrentActualPositionAndVelocity()->vel;

    // calculate current velocity
    const float vel_total = calc_length_pythagorean_2D(curr_vel.x, curr_vel.y);

    if (vel_total <= 0.0f) {
        return;
    }
    
    const float kP = (posControl.pids.pos[X].param.kP + posControl.pids.pos[Y].param.kP) / 2.0f;
    const float max_speed_xy = (navGetCurrentStateFlags() & NAV_AUTO_RTH) || (navGetCurrentStateFlags() & NAV_AUTO_WP) ? wp_desired_speed_xy_cms : loiter_speed_cms;
    const float max_accel_xy = (navGetCurrentStateFlags() & NAV_AUTO_RTH) || (navGetCurrentStateFlags() & NAV_AUTO_WP) ? _wp_accel_cmss : loiter_max_accel_xy;

    const float stopping_dist = sqrtControllerInverse(kP, max_accel_xy, constrainf(vel_total, 0.0f, max_speed_xy));

    if (stopping_dist <= 0.0f) {
        return;
    }

    // convert the stopping distance into a stopping point using velocity vector
    const float t = stopping_dist / vel_total;
    stopping_point->x += (curr_vel.x * t);
    stopping_point->y += (curr_vel.y * t);
}

// initialise the position controller to the stopping point with zero velocity and acceleration.
// This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
void initXYControllerStoppingPoint(float dt)
{
    initXYController(dt);

    getStoppingPointXY(&_pos_target);

    _vel_desired.x = 0.0f;
    _vel_desired.y = 0.0f;
    _accel_desired.x = 0.0f;
    _accel_desired.y = 0.0f;
}

// initialise the position controller to the current position and velocity with decaying acceleration.
// This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
void relaxVelocityXYController(float dt)
{
    #define POSCONTROL_RELAX_TC 0.16f // This is used to decay the I term to 5% in half a second.

    // decay acceleration and therefore current attitude target to zero
    // this will be reset by initXYController() if !isActiveXY()
    if (dt > 0.0f) {
        float decay = 1.0 - dt / (dt + POSCONTROL_RELAX_TC);
        _accel_target.x *= decay;
        _accel_target.y *= decay;
    }

    initXYController(dt);
}

// reduce response for landing
static void softenForLandingXY(float dt)
{
    // decay position error to zero
    if (dt > 0.0f) {
        _pos_target.x += (navGetCurrentActualPositionAndVelocity()->pos.x - _pos_target.x) * (dt / (dt + POSCONTROL_RELAX_TC));
        _pos_target.y += (navGetCurrentActualPositionAndVelocity()->pos.y - _pos_target.y) * (dt / (dt + POSCONTROL_RELAX_TC));
    }
}

// runs the horizontal position controller correcting position, velocity and acceleration errors.
//     Position and velocity errors are converted to velocity and acceleration targets using PID objects
//     Desired velocity and accelerations are added to these corrections as they are calculated
//     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
static void updateXYController(float dt)
{
    float ekfNavVelGainScaler = 1.0f; // TODO: verify if works better with this value or 0.5f
    // Position Controller
    const fpVector3_t curr_pos = navGetCurrentActualPositionAndVelocity()->pos;
    fpVector3_t vel_target;

    // Check for position control time out
    if (!isActiveXY()) {
        initXYController(dt);
    }

    last_update_xy_ticks = getSchedulerTicks32();

    // calculate distance _error
    fpVector3_t _error = { .v = { _pos_target.x - curr_pos.x, _pos_target.y - curr_pos.y, 0.0f }};

    // Constrain _error and target position
    // Constrain the maximum length of _vel_target to the maximum position correction velocity
    const float error_length = calc_length_pythagorean_2D(_error.x, _error.y);
    if (sqrt_controller_pos_xy.error_max > 0.0f && (error_length > sqrt_controller_pos_xy.error_max) && error_length > 0.0f) {
        _error.x *= (sqrt_controller_pos_xy.error_max / error_length);
        _error.y *= (sqrt_controller_pos_xy.error_max / error_length);
        _pos_target.x = curr_pos.x + _error.x;
        _pos_target.y = curr_pos.y + _error.y;
    }

    const float _error_length = calc_length_pythagorean_2D(_error.x, _error.y);

    if (_error_length <= 0.0f) {
        vel_target.x = 0.0f;
        vel_target.y = 0.0f;
    }

    const float correction_length = sqrtControllerApply(&sqrt_controller_pos_xy, _error_length, 0.0f, 0.0f, SQRT_CONTROLLER_POS_XY);

    vel_target.x = _error.x * (correction_length / _error_length);
    vel_target.y = _error.y * (correction_length / _error_length);

    // add velocity feed-forward
    vel_target.x *= ekfNavVelGainScaler;
    vel_target.y *= ekfNavVelGainScaler;
    _vel_target.x = vel_target.x;
    _vel_target.y = vel_target.y;
    _vel_target.x += _vel_desired.x;
    _vel_target.y += _vel_desired.y;

    // Velocity Controller
    const fpVector3_t curr_vel = navGetCurrentActualPositionAndVelocity()->vel;

    // reset input filter to value received
    if (reset_pid_vel_xy_filter) {
        reset_pid_vel_xy_filter = false;
        vel_xy_error.x = _vel_target.x - curr_vel.x;
        vel_xy_error.y = _vel_target.y - curr_vel.y;
        vel_xy_derivative.x = 0.0f;
        vel_xy_derivative.y = 0.0f;
    } else {
        #define VEL_XY_ERROR_LPF_HZ 5.0f
        fpVector3_t error_last = vel_xy_error;
        vel_xy_error.x = pt1FilterApply4(&pid_vel_xy_error_filter[X], _vel_target.x - curr_vel.x, VEL_XY_ERROR_LPF_HZ, dt);
        vel_xy_error.y = pt1FilterApply4(&pid_vel_xy_error_filter[Y], _vel_target.y - curr_vel.y, VEL_XY_ERROR_LPF_HZ, dt);
        // calculate and filter derivative
        if (dt > 0.0f) {
            const fpVector3_t derivative = { .v = { (vel_xy_error.x - error_last.x) / dt, (vel_xy_error.y - error_last.y) / dt, 0.0f }};
            vel_xy_derivative.x = pt1FilterApply4(&pid_vel_xy_derivative_filter[X], derivative.x, pidProfile()->navVelXyDTermLpfHz, dt);
            vel_xy_derivative.y = pt1FilterApply4(&pid_vel_xy_derivative_filter[Y], derivative.y, pidProfile()->navVelXyDTermLpfHz, dt);
        }
    }

    // update I term
    fpVector3_t delta_integrator = { .v = { (vel_xy_error.x * posControl.pids.vel[X].param.kI) * dt, (vel_xy_error.y * posControl.pids.vel[Y].param.kI) * dt, 0.0f }};
    float integrator_length = calc_length_pythagorean_2D(pid_vel_xy_integrator.x, pid_vel_xy_integrator.y);
    pid_vel_xy_integrator.x += delta_integrator.x;
    pid_vel_xy_integrator.y += delta_integrator.y;

    const float len = calc_length_pythagorean_2D(pid_vel_xy_integrator.x, pid_vel_xy_integrator.y);

    if ((len > integrator_length) && len > 0.0f) {
        pid_vel_xy_integrator.x *= (integrator_length / len);
        pid_vel_xy_integrator.y *= (integrator_length / len);
    }

    const float i_len = calc_length_pythagorean_2D(pid_vel_xy_integrator.x, pid_vel_xy_integrator.y);

    if ((i_len > pid_vel_xy_kimax) && i_len > 0.0f) {
        pid_vel_xy_integrator.x *= (pid_vel_xy_kimax / i_len);
        pid_vel_xy_integrator.y *= (pid_vel_xy_kimax / i_len);
    }

    fpVector3_t accel_target;
    accel_target.x = vel_xy_error.x * posControl.pids.vel[X].param.kP + pid_vel_xy_integrator.x + vel_xy_derivative.x * posControl.pids.vel[X].param.kD;
    accel_target.y = vel_xy_error.y * posControl.pids.vel[Y].param.kP + pid_vel_xy_integrator.y + vel_xy_derivative.y * posControl.pids.vel[Y].param.kD;

    // acceleration to correct for velocity error
    accel_target.x *= ekfNavVelGainScaler;
    accel_target.y *= ekfNavVelGainScaler;

    // pass the correction acceleration to the target acceleration output
    _accel_target.x = accel_target.x;
    _accel_target.y = accel_target.y;

    // Add feed forward into the target acceleration output
    _accel_target.x += _accel_desired.y;
    _accel_target.y += _accel_desired.y;

    // Acceleration Controller

    // limit acceleration using maximum lean angles
    float angle_max = MIN((getAltHoldLeanAngleMax(dt)), navConfig()->mc.max_bank_angle);
    float accel_max = GRAVITY_CMSS * tanf(DEGREES_TO_RADIANS(angle_max));

    limitAccelXY(_vel_desired, &_accel_target, accel_max);

    // rotate accelerations into body forward-right frame
    const float accel_forward = _accel_target.x * posControl.actualState.cosYaw + _accel_target.y * posControl.actualState.sinYaw;
    const float accel_right = -_accel_target.x * posControl.actualState.sinYaw + _accel_target.y * posControl.actualState.cosYaw;

    // update angle targets that will be passed to stabilize controller
    _pitch_target = RADIANS_TO_DECIDEGREES(atanf((-accel_forward / GRAVITY_CMSS)));
    _roll_target = RADIANS_TO_DECIDEGREES(atanf((accel_right * cosf(DECIDEGREES_TO_RADIANS(_pitch_target)) / GRAVITY_CMSS)));
}

static void loiterInitTarget(bool resetPosition, float dt)
{
    #define LOITER_VEL_CORRECTION_MAX 200.0f
    #define LOITER_POS_CORRECTION_MAX 200.0f
    // initialise position controller speed and acceleration
    const float kP = (posControl.pids.pos[X].param.kP + posControl.pids.pos[Y].param.kP) / 2.0f;
    sqrtControllerInit(&sqrt_controller_pos_xy, kP, 0.0f, LOITER_VEL_CORRECTION_MAX, loiter_max_accel_xy, SQRT_CONTROLLER_POS_XY);
    // reduce maximum position error to LOITER_POS_CORRECTION_MAX
    if (sqrt_controller_pos_xy.error_max != 0.0f) {
        sqrt_controller_pos_xy.error_max = MIN(sqrt_controller_pos_xy.error_max, LOITER_POS_CORRECTION_MAX);
    } else {
        sqrt_controller_pos_xy.error_max = LOITER_POS_CORRECTION_MAX;
    }
    
    if (resetPosition) {
        // initialise position controller
        initXYControllerStoppingPoint(dt);
        // initialise desired acceleration and angles to zero to remain on station
        _predicted_accel.x = 0.0f;
        _predicted_accel.y = 0.0f;
        _desired_accel.x = 0.0f;
        _desired_accel.y = 0.0f;
        _predicted_euler_angle.x = 0.0f;
        _predicted_euler_angle.y = 0.0f;

        // set target position
        _pos_target.x = navGetCurrentActualPositionAndVelocity()->pos.x;
        _pos_target.y = navGetCurrentActualPositionAndVelocity()->pos.y;
    } else {
        // initialise position controller and move target accelerations smoothly towards zero
        relaxVelocityXYController(dt);
        // initialise predicted acceleration and angles from the position controller
        _predicted_accel.x = _accel_target.x;
        _predicted_accel.y = _accel_target.y;
        _predicted_euler_angle.x = DECIDEGREES_TO_RADIANS(_roll_target);
        _predicted_euler_angle.y = DECIDEGREES_TO_RADIANS(_pitch_target);
    }

    _brake_accel = 0.0f;
}

static void resetMulticopterXYPosControl(void)
{
    poshold.loop_rate_factor = (uint8_t)((1e6 / getLooptime()) / 100);

    // initialise lean angles to current attitude
    poshold.pilot_roll = 0.0f;
    poshold.pilot_pitch = 0.0f;

    // compute brake_gain
    poshold.brake_gain = (15.0f * (float)poshold_brake_rate + 95.0f) / 100.0f;

    if (isMulticopterLandingDetected()) {
        // if landed begin in loiter mode
        poshold.roll_mode = POSHOLD_LOITER;
        poshold.pitch_mode = POSHOLD_LOITER;
    } else {
        // if not landed start in pilot override to avoid hard twitch
        poshold.roll_mode = POSHOLD_PILOT_OVERRIDE;
        poshold.pitch_mode = POSHOLD_PILOT_OVERRIDE;
    }
    
    // initialise loiter
    _desired_accel.x = 0.0f;
    _desired_accel.y = 0.0f;
    loiterInitTarget(false, US2S(getLooptime()));

    // initialise wind_comp each time Pos-Hold is switched on
    vectorZero(&poshold.wind_comp_ef);
    poshold.wind_comp_roll = 0.0f;
    poshold.wind_comp_pitch = 0.0f;
    poshold.wind_comp_timer = 0;
}

// transform pilot's roll or pitch input into a desired lean angle
static void getPilotDesiredLeanAngles(float *roll_out, float *pitch_out, float angle_max, float angle_limit)
{
    // throttle fail-safe check
    if (FLIGHT_MODE(FAILSAFE_MODE)) {
        *roll_out = 0.0f;
        *pitch_out = 0.0f;
        return;
    }

    const float roll_in_unit = (float)applyDeadbandRescaled(rcCommand[ROLL], rcControlsConfig()->pos_hold_deadband, -500, 500) * (1.0f / 500.0f);
    const float pitch_in_unit = (float)applyDeadbandRescaled(rcCommand[PITCH], rcControlsConfig()->pos_hold_deadband, -500, 500) * (1.0f / 500.0f);

    const float angle_max_deg = MIN(angle_max, 85.0f);
    const float rc_2_rad = DEGREES_TO_RADIANS(angle_max_deg);

    // fetch roll and pitch stick positions and convert them to normalised horizontal thrust
    fpVector3_t thrust;
    thrust.x = -tanf(rc_2_rad * pitch_in_unit);
    thrust.y = tanf(rc_2_rad * roll_in_unit);

    // calculate the horizontal thrust limit based on the angle limit
    const float angle_limit_deg = constrainf(angle_limit, 10.0f, angle_max_deg);
    const float thrust_limit = tanf(DEGREES_TO_RADIANS(angle_limit_deg));

    // apply horizontal thrust limit
    const float len = calc_length_pythagorean_2D(thrust.x, thrust.y);
    if ((len > thrust_limit) && len > 0.0f) {
        thrust.x *= (thrust_limit / len);
        thrust.y *= (thrust_limit / len);
    }

    // Conversion from angular thrust vector to euler angles.
    const float pitch_rad = -atanf(thrust.x);
    const float roll_rad = atanf(cosf(pitch_rad) * thrust.y);

    *roll_out = RADIANS_TO_DECIDEGREES(roll_rad);
    *pitch_out = RADIANS_TO_DECIDEGREES(pitch_rad);
}

static void updateMulticopterLoiterControl(float dt)
{
    updateLoiterDesiredVelocity(dt);
    updateXYController(dt);
}

// should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void wpSplineInit(float speed_cms)
{    
    // check wp radius is reasonable
    wp_radius_cm = MAX(wp_radius_cm, 5.0f);

    // check wp speed
    wp_speed_cms = MAX(wp_speed_cms, 20.0f);

    // initialise position controller
    initXYControllerStoppingPoint(US2S(getLooptime()));

    // initialize the desired wp speed
    wp_desired_speed_xy_cms = speed_cms > 0.0f ? speed_cms : wp_speed_cms;
    wp_desired_speed_xy_cms = MAX(wp_desired_speed_xy_cms, 20.0f);

    // initialise position controller speed and acceleration
    const float kP = (posControl.pids.pos[X].param.kP + posControl.pids.pos[Y].param.kP) / 2.0f;
    sqrtControllerInit(&sqrt_controller_pos_xy, kP, 0.0f, wp_desired_speed_xy_cms, _wp_accel_cmss, SQRT_CONTROLLER_POS_XY);

    // calculate scurve jerk and jerk time
    if (_wp_jerk <= 0.0f) {
        _wp_jerk = _wp_accel_cmss;
    }

    // calculate maximum snap
    // Snap (the rate of change of jerk) uses the attitude control input time constant because multicopters lean to accelerate.
    // This means the change in angle is equivalent to the change in acceleration
    wp_scurve_snap = (_wp_jerk * M_PI) / (2.0 * MAX((float)input_tc / 100.0f, 0.1f));
    const float snap = MIN(DEGREES_TO_RADIANS(accel_roll_max), DEGREES_TO_RADIANS(accel_pitch_max)) * GRAVITY_MSS;
    if (snap > 0.0f) {
        wp_scurve_snap = MIN(wp_scurve_snap, snap);
    }

    // reduce maximum snap by a factor of two from what the aircraft is capable of
    wp_scurve_snap *= 0.5f;

    scurveInit(&scurve_prev_leg);
    scurveInit(&scurve_this_leg);
    scurveInit(&scurve_next_leg);
    wp_track_scalar_dt = 1.0f;

    vectorZero(&wp_origin);
    vectorZero(&wp_destination);

    wp_reached_destination = true;
    fast_waypoint = false;

    wp_this_leg_is_spline = false;

    // initialise the terrain velocity to the current maximum velocity
    wp_offset_vel = wp_desired_speed_xy_cms;
    wp_offset_accel = 0.0;

    // mark as active
    wp_last_update = millis();
}

// returns true if update_wpnav has been run very recently
bool wpIsActive(void)
{
    return (millis() - wp_last_update) < 200;
}

void setWPDestination(const fpVector3_t destination, float wp_manual_speed)
{
    // init way-point navigation
    if (!wpIsActive()) {
        const float des_speed_xy_cm = wp_manual_speed > 0.0f ? wp_manual_speed : 0.0f;
        wpSplineInit(des_speed_xy_cm);
    }

    // re-initialise if previous destination has been interrupted
    if (!wp_reached_destination) {
        wpSplineInit(wp_desired_speed_xy_cms);
    }

    scurveInit(&scurve_prev_leg);

    float origin_speed = 0.0f;

    // use previous destination as origin
    wp_origin = wp_destination;

    if (wp_this_leg_is_spline) {
        // if previous leg was a spline we can use current target velocity vector for origin velocity vector
        origin_speed = calc_length_pythagorean_3D(_vel_desired.x, _vel_desired.y, _vel_desired.z);
    } else {
        // store previous leg
        scurve_prev_leg = scurve_this_leg;
    }

    // update destination
    wp_destination = destination;

    if (fast_waypoint && !wp_this_leg_is_spline && !wp_next_leg_is_spline && !scurveFinished(&scurve_next_leg)) {
        scurve_this_leg = scurve_next_leg;
    } else {
        scurveCalculateTrack(&scurve_this_leg, wp_origin, wp_destination,
                                wp_speed_cms, navConfig()->mc.max_auto_climb_rate, -fabsf((float)navConfig()->mc.max_auto_climb_rate),
                                _wp_accel_cmss, _wp_accel_z_cmss,
                                wp_scurve_snap * 100.0f, _wp_jerk * 100.0f);

        if (origin_speed != 0.0f) {
            // rebuild start of scurve if we have a non-zero origin speed
            scurveSetOriginSpeeMax(&scurve_this_leg, origin_speed);
        }
    }

    wp_this_leg_is_spline = false;
    scurveInit(&scurve_next_leg);
    fast_waypoint = false; // default waypoint back to slow
    wp_reached_destination = false;
}

void setWPDestinationNext(const fpVector3_t destination)
{
    if (destination.x == 0.0f && destination.y == 0.0f) {
        return;
    }

    scurveCalculateTrack(&scurve_next_leg, wp_destination, destination,
                            wp_speed_cms, navConfig()->mc.max_auto_climb_rate, -fabsf((float)navConfig()->mc.max_auto_climb_rate),
                            _wp_accel_cmss, _wp_accel_z_cmss,
                            wp_scurve_snap * 100.0f, _wp_jerk * 100.0f);

    if (wp_this_leg_is_spline) {
        const float this_leg_dest_speed_max = spline_this_leg._destination_speed_max;
        const float next_leg_origin_speed_max = scurveSetOriginSpeeMax(&scurve_next_leg, this_leg_dest_speed_max);
        spline_this_leg._destination_speed_max = MIN(spline_this_leg._destination_speed_max, next_leg_origin_speed_max);
    }
    
    wp_next_leg_is_spline = false;

    // next destination provided so fast waypoint
    fast_waypoint = true;
}

// initiate move to next waypoint
// call this function inside of navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION in NAV_WP_ACTION_WAYPOINT switch case
void startMulticopterWPNavigation(const fpVector3_t dest_loc, const fpVector3_t next_dest_loc)
{
    const float wp_manual_speed = getActiveSpeed();

    setWPDestination(dest_loc, wp_manual_speed);

    // range check target speed and protect against divide by zero
    if (wp_manual_speed >= 20.0f && wp_desired_speed_xy_cms > 0.0f) {
        // update horizontal velocity speed offset scalar
        wp_offset_vel = wp_manual_speed * wp_offset_vel / wp_desired_speed_xy_cms;

        // initialize the desired wp speed
        wp_desired_speed_xy_cms = wp_manual_speed;

        // initialise position controller speed and acceleration
        const float kP = (posControl.pids.pos[X].param.kP + posControl.pids.pos[Y].param.kP) / 2.0f;
        sqrtControllerInit(&sqrt_controller_pos_xy, kP, 0.0f, wp_desired_speed_xy_cms, _wp_accel_cmss, SQRT_CONTROLLER_POS_XY);

        // change track speed - update this leg
        if (wp_this_leg_is_spline) {
            splineCurveSetSpeedAccel(&spline_this_leg, wp_desired_speed_xy_cms, navConfig()->mc.max_auto_climb_rate, -fabsf((float)navConfig()->mc.max_auto_climb_rate), _wp_accel_cmss, _wp_accel_z_cmss);
        } else {
            scurveSetSpeedMax(&scurve_this_leg, wp_desired_speed_xy_cms, navConfig()->mc.max_auto_climb_rate, -fabsf((float)navConfig()->mc.max_auto_climb_rate));
        }

        // change track speed - update next leg
        if (wp_next_leg_is_spline) {
            splineCurveSetSpeedAccel(&spline_next_leg, wp_desired_speed_xy_cms, navConfig()->mc.max_auto_climb_rate, -fabsf((float)navConfig()->mc.max_auto_climb_rate), _wp_accel_cmss, _wp_accel_z_cmss);
        } else {
            scurveSetSpeedMax(&scurve_next_leg, wp_desired_speed_xy_cms, navConfig()->mc.max_auto_climb_rate, -fabsf((float)navConfig()->mc.max_auto_climb_rate));
        }
    }

    // set next destination if necessary
    setWPDestinationNext(next_dest_loc);
}

// move target location along track from origin to destination
void advanceWPTargetAlongTrack(float dt)
{
    // get current position
    const fpVector3_t curr_pos = navGetCurrentActualPositionAndVelocity()->pos;
    const fpVector3_t curr_vel = navGetCurrentActualPositionAndVelocity()->vel;
    const fpVector3_t curr_target_vel = _vel_desired;

    // Use wp_track_scalar_dt to slow down progression of the position target moving too far in front of aircraft wp_track_scalar_dt does not scale the velocity or acceleration
    float track_scaler_dt = 1.0f;

    // check target velocity is non-zero
    if (sq(curr_target_vel.x) + sq(curr_target_vel.y) + sq(curr_target_vel.z) > 0.0f) {
        const float curr_vel_len = calc_length_pythagorean_3D(curr_target_vel.x, curr_target_vel.y, curr_target_vel.z);
        fpVector3_t track_direction = { .v = { curr_target_vel.x / curr_vel_len, curr_target_vel.y / curr_vel_len, curr_target_vel.z / curr_vel_len }};
        const float track_error = ((_pos_target.x - curr_pos.x) * track_direction.x) + ((_pos_target.y - curr_pos.y) * track_direction.y) + ((_pos_target.z - curr_pos.z) * track_direction.z);
        const float track_velocity = curr_vel.x * track_direction.x + curr_vel.y * track_direction.y + curr_vel.z * track_direction.z;
        // set time scaler to be consistent with the achievable aircraft speed with a 5% buffer for short term variation.
        const float kP = (posControl.pids.pos[X].param.kP + posControl.pids.pos[Y].param.kP) / 2.0f;
        track_scaler_dt = constrainf(0.05f + (track_velocity - kP * track_error) / curr_vel_len, 0.0f, 1.0f);
    }

    // Use vel_scaler_dt to slow down the trajectory time vel_scaler_dt scales the velocity and acceleration to be kinematically consistent
    float vel_scaler_dt = 1.0f;
    if (wp_desired_speed_xy_cms > 0.0f) {
        const float delta_vel = wp_offset_accel * dt;
        wp_offset_vel += delta_vel;
        
        const float accel_min = -_wp_accel_cmss;

        // sanity check accel_min, accel_max and jerk_max.
        if (accel_min < 0.0f || _wp_accel_cmss > 0.0f || wp_shaping_jerk_xy > 0.0f) {
            // velocity error to be corrected
            const float vel_error = wp_desired_speed_xy_cms - wp_offset_vel;

            // Calculate time constants and limits to ensure stable operation
            // The direction of acceleration limit is the same as the velocity error.
            // This is because the velocity error is negative when slowing down while
            // closing a positive position error.
            float KPa;
            if (vel_error > 0.0f) {
                KPa = wp_shaping_jerk_xy / _wp_accel_cmss;
            } else {
                KPa = wp_shaping_jerk_xy / (-accel_min);
            }

            // acceleration to correct velocity
            sqrt_controller_waypoint.derivative_max = wp_shaping_jerk_xy;
            sqrt_controller_waypoint.kp = KPa;
            float accel_target = sqrtControllerApply(&sqrt_controller_waypoint, vel_error, 0.0f, dt, SQRT_CONTROLLER_POS_XY);

            // constrain correction acceleration from accel_min to _wp_accel_cmss
            accel_target = constrainf(accel_target, accel_min, _wp_accel_cmss);

            // constrain total acceleration from accel_min to _wp_accel_cmss
            accel_target = constrainf(accel_target, accel_min, _wp_accel_cmss);

            // jerk limit acceleration change
            if (dt > 0.0f && wp_shaping_jerk_xy > 0.0f) {
                float accel_delta = accel_target - wp_offset_accel;
                accel_delta = constrainf(accel_delta, -wp_shaping_jerk_xy * dt, wp_shaping_jerk_xy * dt);
                wp_offset_accel += accel_delta;
            }
        }

        vel_scaler_dt = wp_offset_vel / wp_desired_speed_xy_cms;
    }

    // change s-curve time speed with a time constant of maximum acceleration / maximum jerk
    float track_scaler_tc = 1.0f;
    if (_wp_jerk != 0.0f) {
        track_scaler_tc = 0.01f * _wp_accel_cmss / _wp_jerk;
    }

    wp_track_scalar_dt += (track_scaler_dt - wp_track_scalar_dt) * (dt / track_scaler_tc);

    // target position, velocity and acceleration from straight line or spline calculators
    fpVector3_t target_pos;
    fpVector3_t target_vel;
    fpVector3_t target_accel;

    bool s_finished;

    if (!wp_this_leg_is_spline) {
        // update target position, velocity and acceleration
        target_pos = wp_origin;
        s_finished = scurveAdvanceTargetAlongTrack(&scurve_this_leg, &scurve_prev_leg, &scurve_next_leg, wp_radius_cm, _wp_accel_cmss, fast_waypoint, wp_track_scalar_dt * vel_scaler_dt * dt, &target_pos, &target_vel, &target_accel);
    } else {
        // spline target_vel
        target_vel = curr_target_vel;
        splineCurveAdvanceTargetAlongTrack(&spline_this_leg, wp_track_scalar_dt * vel_scaler_dt * dt, &target_pos, &target_vel);
        s_finished = spline_this_leg._reached_destination;
    }

    fpVector3_t accel_offset;
    if (sq(target_vel.x) + sq(target_vel.y) + sq(target_vel.z) > 0.0f) {
        const float target_vel_len = calc_length_pythagorean_3D(target_vel.x, target_vel.y, target_vel.z);
        fpVector3_t track_direction = { .v = { target_vel.x / target_vel_len, target_vel.y / target_vel_len, target_vel.z / target_vel_len }};
        accel_offset.x = track_direction.x * wp_offset_accel * target_vel_len / wp_desired_speed_xy_cms;
        accel_offset.y = track_direction.y * wp_offset_accel * target_vel_len / wp_desired_speed_xy_cms;
        accel_offset.z = track_direction.z * wp_offset_accel * target_vel_len / wp_desired_speed_xy_cms;
    }

    target_vel.x *= vel_scaler_dt;
    target_vel.y *= vel_scaler_dt;
    target_vel.z *= vel_scaler_dt;
    target_accel.x *= sq(vel_scaler_dt);
    target_accel.y *= sq(vel_scaler_dt);
    target_accel.z *= sq(vel_scaler_dt);
    target_accel.x += accel_offset.x;
    target_accel.y += accel_offset.y;
    target_accel.z += accel_offset.z;

    // pass new target to the position controller
    // TODO: For now we cannot change the Z axis target because it still uses the old INAV altitude controller.
    _pos_target.x = target_pos.x;
    _pos_target.y = target_pos.y;
    _vel_desired.x = target_vel.x;
    _vel_desired.y = target_vel.y;
    _accel_desired.x = target_accel.x;
    _accel_desired.y = target_accel.y;

    debug[6] = _pos_target.x;
    debug[7] = _pos_target.y;

    // check if we've reached the waypoint
    if (!wp_reached_destination) {
        if (s_finished) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (fast_waypoint) {
                wp_reached_destination = true;
            } else {
                // regular waypoints also require the copter to be within the waypoint radius
                const fpVector3_t dist_to_dest = { .v = { curr_pos.x - wp_destination.x, curr_pos.y - wp_destination.y, curr_pos.z - wp_destination.z }};
                if ((sq(dist_to_dest.x) + sq(dist_to_dest.y) + sq(dist_to_dest.z)) <= sq(wp_radius_cm)) {
                    wp_reached_destination = true;
                }
            }
        }
    }
}

bool isMulticopterWPReached(void)
{
    return wp_reached_destination;
}

static void updateMulticopterXYPosControl(float dt)
{
    float target_roll;  // pilot's roll angle input
    float target_pitch; // pilot's pitch angle input
    const float lean_angle_max = DEGREES_TO_DECIDEGREES(navConfig()->mc.max_bank_angle);

    // Way-Point and RTH mode
    if ((navGetCurrentStateFlags() & NAV_AUTO_WP) || (navGetCurrentStateFlags() & NAV_AUTO_RTH)) {
        advanceWPTargetAlongTrack(dt);
        
        updateXYController(dt);
        
        wp_last_update = millis();
        
        int16_t angleOut[2];

        // constrain target pitch/roll angles
        angleOut[ROLL] = constrainf(_roll_target, -lean_angle_max, lean_angle_max);
        angleOut[PITCH] = constrainf(_pitch_target, -lean_angle_max, lean_angle_max);

        debug[0] = angleOut[ROLL];
        debug[1] = angleOut[PITCH];

        //posControl.rcAdjustment[ROLL] = angleOut[ROLL];
        //posControl.rcAdjustment[PITCH] = angleOut[PITCH];
    }
    
    // Loiter mode
    /*if (navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE) {
        float roll_out;
        float pitch_out;

        if (!FLIGHT_MODE(FAILSAFE_MODE)) {
            // convert pilot input to lean angles
            getPilotDesiredLeanAngles(&roll_out, &pitch_out, getLoiterAngleMax(), getAltHoldLeanAngleMax(dt));
            // process pilot's roll and pitch input
            setPilotDesiredAcceleration(roll_out, pitch_out, dt);
        } else {
            // clear out pilot desired acceleration in case radio fail-safe event occurs
            _desired_accel.x = 0.0f;
            _desired_accel.y = 0.0f;
        }

        //if (isMulticopterLandingDetected()) // TODO: I'm still not sure if Land's detector is reliable enough to use here.
        //{
            //loiterInitTarget(false, dt);
            //softenForLandingXY(dt);
        //}

        updateMulticopterLoiterControl(dt);

        int16_t angleOut[2];

        // constrain target pitch/roll angles
        angleOut[ROLL] = constrainf(_roll_target, -lean_angle_max, lean_angle_max);
        angleOut[PITCH] = constrainf(_pitch_target, -lean_angle_max, lean_angle_max);

        debug[0] = angleOut[ROLL];
        debug[1] = angleOut[PITCH];

        //posControl.rcAdjustment[ROLL] = angleOut[ROLL];
        //posControl.rcAdjustment[PITCH] = angleOut[PITCH];
    }*/
    
    // Pos-Hold mode
    if (FLIGHT_MODE(NAV_POSHOLD_MODE)) {
        _desired_accel.x = 0.0f;
        _desired_accel.y = 0.0f;

        // convert pilot input to lean angles
        getPilotDesiredLeanAngles(&target_roll, &target_pitch, navConfig()->mc.max_bank_angle, getAltHoldLeanAngleMax(dt));

        //if (isMulticopterLandingDetected()) // TODO: I'm still not sure if Land's detector is reliable enough to use here.
        //{
            //loiterInitTarget(false, dt);
            //softenForLandingXY(dt);
        //}

        // convert inertial nav earth-frame velocities to body-frame
        const float vel_fw = navGetCurrentActualPositionAndVelocity()->vel.x * posControl.actualState.cosYaw + navGetCurrentActualPositionAndVelocity()->vel.y * posControl.actualState.sinYaw;
        const float vel_right = -navGetCurrentActualPositionAndVelocity()->vel.x * posControl.actualState.sinYaw + navGetCurrentActualPositionAndVelocity()->vel.y * posControl.actualState.cosYaw;
        
        // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
        if (poshold.roll_mode != POSHOLD_LOITER || poshold.pitch_mode != POSHOLD_LOITER) {
            posHoldGetWindCompLeanAngles(&poshold.wind_comp_roll, &poshold.wind_comp_pitch);
        }

        // Roll state machine
        //  Each state (aka mode) is responsible for:
        //      1. dealing with pilot input
        //      2. calculating the final roll output to the attitude controller
        //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
        switch (poshold.roll_mode) {

            case POSHOLD_PILOT_OVERRIDE:
                // update pilot desired roll angle using latest radio input
                // this filters the input so that it returns to zero no faster than the brake-rate
                posHoldUpdatePilotLeanAngle(&poshold.pilot_roll, target_roll);

                // switch to BRAKE mode for next iteration if no pilot input
                if (target_roll == 0.0f && (fabsf(poshold.pilot_roll) < 2 * poshold_brake_rate)) {
                    // initialise BRAKE mode
                    poshold.roll_mode = POSHOLD_BRAKE;                            // set brake roll mode
                    poshold.brake_roll = 0.0f;                                    // initialise braking angle to zero
                    poshold.brake_angle_max_roll = 0.0f;                          // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                    poshold.brake_timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                    poshold.braking_time_updated_roll = false;                    // flag the braking time can be re-estimated
                }

                // final lean angle should be pilot input plus wind compensation
                poshold.roll = poshold.pilot_roll + poshold.wind_comp_roll;
                break;

            case POSHOLD_BRAKE:
            case POSHOLD_BRAKE_READY_TO_LOITER:
                // calculate brake_roll angle to counter-act velocity
                posHoldUpdateBrakeAngleFromVelocity(&poshold.brake_roll, vel_right);

                // update braking time estimate
                if (!poshold.braking_time_updated_roll) {
                    // check if brake angle is increasing
                    if (fabsf(poshold.brake_roll) >= poshold.brake_angle_max_roll) {
                        poshold.brake_angle_max_roll = fabsf(poshold.brake_roll);
                    } else {
                        // braking angle has started decreasing so re-estimate braking time
                        poshold.brake_timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(poshold.brake_roll))/(10L*(int32_t)poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                        poshold.braking_time_updated_roll = true;
                    }
                }

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (poshold.brake_timeout_roll > 50*LOOP_RATE_FACTOR)) {
                    poshold.brake_timeout_roll = 50*LOOP_RATE_FACTOR;
                }

                // reduce braking timer
                if (poshold.brake_timeout_roll > 0) {
                    poshold.brake_timeout_roll--;
                } else {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to POSHOLD_BRAKE_READY_TO_LOITER
                    // logic for engaging loiter is handled below the roll and pitch mode switch statements
                    poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                }

                // final lean angle is braking angle + wind compensation angle
                poshold.roll = poshold.brake_roll + poshold.wind_comp_roll;

                // check for pilot input
                if (target_roll != 0.0f) {
                    // init transition to pilot override
                    posHoldRollControllerToPilotOverride();
                }
                break;

            case POSHOLD_BRAKE_TO_LOITER:
            case POSHOLD_LOITER:
                // these modes are combined roll-pitch modes and are handled below
                break;

            case POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE:
                // update pilot desired roll angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                posHoldUpdatePilotLeanAngle(&poshold.pilot_roll, target_roll);

                // count-down loiter to pilot timer
                if (poshold.controller_to_pilot_timer_roll > 0) {
                    poshold.controller_to_pilot_timer_roll--;
                } else {
                    // when timer runs out switch to full pilot override for next iteration
                    poshold.roll_mode = POSHOLD_PILOT_OVERRIDE;
                }

                // mix of controller and pilot controls. 0 = fully last controller controls, 1 = fully pilot controls
                const float controller_to_pilot_roll_mix = (float)poshold.controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

                // mix final loiter lean angle and pilot desired lean angles
                poshold.roll = posHoldMixControls(controller_to_pilot_roll_mix, poshold.controller_final_roll, poshold.pilot_roll + poshold.wind_comp_roll);
                break;
        }

        // Pitch state machine
        //  Each state (aka mode) is responsible for:
        //      1. dealing with pilot input
        //      2. calculating the final pitch output to the attitude contpitcher
        //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
        switch (poshold.pitch_mode) {

            case POSHOLD_PILOT_OVERRIDE:
                // update pilot desired pitch angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                posHoldUpdatePilotLeanAngle(&poshold.pilot_pitch, target_pitch);

                // switch to BRAKE mode for next iteration if no pilot input
                if (target_pitch == 0.0f && (fabsf(poshold.pilot_pitch) < 2 * poshold_brake_rate)) {
                    // initialise BRAKE mode
                    poshold.pitch_mode = POSHOLD_BRAKE;                            // set brake pitch mode
                    poshold.brake_pitch = 0.0f;                                    // initialise braking angle to zero
                    poshold.brake_angle_max_pitch = 0.0f;                          // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                    poshold.brake_timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                    poshold.braking_time_updated_pitch = false;                    // flag the braking time can be re-estimated
                }

                // final lean angle should be pilot input plus wind compensation
                poshold.pitch = poshold.pilot_pitch + poshold.wind_comp_pitch;
                break;

            case POSHOLD_BRAKE:
            case POSHOLD_BRAKE_READY_TO_LOITER:
                // calculate brake_pitch angle to counter-act velocity
                posHoldUpdateBrakeAngleFromVelocity(&poshold.brake_pitch, -vel_fw);

                // update braking time estimate
                if (!poshold.braking_time_updated_pitch) {
                    // check if brake angle is increasing
                    if (fabsf(poshold.brake_pitch) >= poshold.brake_angle_max_pitch) {
                        poshold.brake_angle_max_pitch = fabsf(poshold.brake_pitch);
                    } else {
                        // braking angle has started decreasing so re-estimate braking time
                        poshold.brake_timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(poshold.brake_pitch))/(10L*(int32_t)poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                        poshold.braking_time_updated_pitch = true;
                    }
                }

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (poshold.brake_timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                    poshold.brake_timeout_pitch = 50*LOOP_RATE_FACTOR;
                }

                // reduce braking timer
                if (poshold.brake_timeout_pitch > 0) {
                    poshold.brake_timeout_pitch--;
                } else {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to POSHOLD_BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                    poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                }

                // final lean angle is braking angle + wind compensation angle
                poshold.pitch = poshold.brake_pitch + poshold.wind_comp_pitch;

                // check for pilot input
                if (target_pitch != 0.0f) {
                    // init transition to pilot override
                    posHoldPitchControllerToPilotOverride();
                }
                break;

            case POSHOLD_BRAKE_TO_LOITER:
            case POSHOLD_LOITER:
                // these modes are combined pitch-pitch modes and are handled below
                break;

            case POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE:
                // update pilot desired pitch angle using latest radio input
                // this filters the input so that it returns to zero no faster than the brake-rate
                posHoldUpdatePilotLeanAngle(&poshold.pilot_pitch, target_pitch);

                // count-down loiter to pilot timer
                if (poshold.controller_to_pilot_timer_pitch > 0) {
                    poshold.controller_to_pilot_timer_pitch--;
                } else {
                    // when timer runs out switch to full pilot override for next iteration
                    poshold.pitch_mode = POSHOLD_PILOT_OVERRIDE;
                }

                // mix of controller and pilot controls. 0 = fully last controller controls, 1 = fully pilot controls
                const float controller_to_pilot_pitch_mix = (float)poshold.controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

                // mix final loiter lean angle and pilot desired lean angles
                poshold.pitch = posHoldMixControls(controller_to_pilot_pitch_mix, poshold.controller_final_pitch, poshold.pilot_pitch + poshold.wind_comp_pitch);
                break;
        }

        // switch into LOITER mode when both roll and pitch are ready
        if (poshold.roll_mode == POSHOLD_BRAKE_READY_TO_LOITER && poshold.pitch_mode == POSHOLD_BRAKE_READY_TO_LOITER) {
            poshold.roll_mode = POSHOLD_BRAKE_TO_LOITER;
            poshold.pitch_mode = POSHOLD_BRAKE_TO_LOITER;
            poshold.brake_to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
            // init loiter controller
            loiterInitTarget(true, dt);
            // set delay to start of wind compensation estimate updates
            poshold.wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
        }

        // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
        if (poshold.roll_mode == POSHOLD_BRAKE_TO_LOITER || poshold.roll_mode == POSHOLD_LOITER) {
            // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
            poshold.pitch_mode = poshold.roll_mode;

            // handle combined roll+pitch mode
            switch (poshold.roll_mode) {
                case POSHOLD_BRAKE_TO_LOITER:
                    // reduce brake_to_loiter timer
                    if (poshold.brake_to_loiter_timer > 0) {
                        poshold.brake_to_loiter_timer--;
                    } else {
                        // progress to full loiter on next iteration
                        poshold.roll_mode = POSHOLD_LOITER;
                        poshold.pitch_mode = POSHOLD_LOITER;
                    }

                    // mix of brake and loiter controls.  0 = fully brake controls, 1 = fully loiter controls
                    const float brake_to_loiter_mix = (float)poshold.brake_to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                    // calculate brake_roll and pitch angles to counter-act velocity
                    posHoldUpdateBrakeAngleFromVelocity(&poshold.brake_roll, vel_right);
                    posHoldUpdateBrakeAngleFromVelocity(&poshold.brake_pitch, -vel_fw);
                    
                    updateMulticopterLoiterControl(dt);

                    // calculate final roll and pitch output by mixing loiter and brake controls
                    poshold.roll = posHoldMixControls(brake_to_loiter_mix, poshold.brake_roll + poshold.wind_comp_roll, _roll_target);
                    poshold.pitch = posHoldMixControls(brake_to_loiter_mix, poshold.brake_pitch + poshold.wind_comp_pitch, _pitch_target);

                    // check for pilot input
                    if (target_roll != 0.0f || target_pitch != 0.0f) {
                        // if roll input switch to pilot override for roll
                        if (target_roll != 0.0f) {
                            // init transition to pilot override
                            posHoldRollControllerToPilotOverride();
                            // switch pitch-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset poshold.brake_pitch here as wind comp has not been updated since last brake_pitch computation
                            poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                        }
                        // if pitch input switch to pilot override for pitch
                        if (target_pitch != 0.0f) {
                            // init transition to pilot override
                            posHoldPitchControllerToPilotOverride();
                            if (target_roll == 0.0f) {
                                // switch roll-mode to brake (but ready to go back to loiter anytime)
                                // no need to reset poshold.brake_roll here as wind comp has not been updated since last brake_roll computation
                                poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                            }
                        }
                    }
                    break;

                case POSHOLD_LOITER:
                    updateMulticopterLoiterControl(dt);

                    poshold.roll = _roll_target;
                    poshold.pitch = _pitch_target;

                    // update wind compensation estimate
                    posHoldUpdateWindCompEstimate();

                    // check for pilot input
                    if (target_roll != 0.0f || target_pitch != 0.0f) {
                        // if roll input switch to pilot override for roll
                        if (target_roll != 0.0f) {
                            // init transition to pilot override
                            posHoldRollControllerToPilotOverride();
                            // switch pitch-mode to brake (but ready to go back to loiter anytime)
                            poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                            // reset brake_pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                            poshold.brake_pitch = 0.0f;
                        }
                        // if pitch input switch to pilot override for pitch
                        if (target_pitch != 0.0f) {
                            // init transition to pilot override
                            posHoldPitchControllerToPilotOverride();
                            // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                            if (target_roll == 0.0f) {
                                poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                                poshold.brake_roll = 0.0f;
                            }
                        }
                    }
                    break;

                default:
                    // do nothing for uncombined roll and pitch modes
                    break;
            }
        }

        int16_t angleOut[2];

        // constrain target pitch/roll angles
        angleOut[ROLL] = constrainf(poshold.roll, -lean_angle_max, lean_angle_max);
        angleOut[PITCH] = constrainf(poshold.pitch, -lean_angle_max, lean_angle_max);

        debug[2] = angleOut[ROLL];
        debug[3] = angleOut[PITCH];

        //posControl.rcAdjustment[ROLL] = angleOut[ROLL];
        //posControl.rcAdjustment[PITCH] = angleOut[PITCH];
    }
}

static void updatePositionAccelController_MC(timeDelta_t deltaMicros, float maxAccelLimit, const float maxSpeed)
{
    const float measurementX = navGetCurrentActualPositionAndVelocity()->vel.x;
    const float measurementY = navGetCurrentActualPositionAndVelocity()->vel.y;

    const float setpointX = posControl.desiredState.vel.x;
    const float setpointY = posControl.desiredState.vel.y;
    const float setpointXY = calc_length_pythagorean_2D(setpointX, setpointY);

    // Calculate velocity error
    const float velErrorX = setpointX - measurementX;
    const float velErrorY = setpointY - measurementY;

    // Calculate XY-acceleration limit according to velocity error limit
    float accelLimitX, accelLimitY;
    const float velErrorMagnitude = calc_length_pythagorean_2D(velErrorX, velErrorY);

    if (velErrorMagnitude > 0.1f) {
        accelLimitX = maxAccelLimit / velErrorMagnitude * fabsf(velErrorX);
        accelLimitY = maxAccelLimit / velErrorMagnitude * fabsf(velErrorY);
    } else {
        accelLimitX = maxAccelLimit / 1.414213f;
        accelLimitY = accelLimitX;
    }

    // Apply additional jerk limiting of 1700 cm/s^3 (~100 deg/s), almost any copter should be able to achieve this rate
    // This will assure that we wont't saturate out LEVEL and RATE PID controller

    float maxAccelChange = US2S(deltaMicros) * MC_POS_CONTROL_JERK_LIMIT_CMSSS;
    //When braking, raise jerk limit even if we are not boosting acceleration
#ifdef USE_MR_BRAKING_MODE
    if (STATE(NAV_CRUISE_BRAKING)) {
        maxAccelChange = maxAccelChange * 2;
    }
#endif

    const float accelLimitXMin = constrainf(lastAccelTargetX - maxAccelChange, -accelLimitX, +accelLimitX);
    const float accelLimitXMax = constrainf(lastAccelTargetX + maxAccelChange, -accelLimitX, +accelLimitX);
    const float accelLimitYMin = constrainf(lastAccelTargetY - maxAccelChange, -accelLimitY, +accelLimitY);
    const float accelLimitYMax = constrainf(lastAccelTargetY + maxAccelChange, -accelLimitY, +accelLimitY);

    // TODO: Verify if we need jerk limiting after all

    /*
     * This PID controller has dynamic dTerm scale. It's less active when controller
     * is tracking setpoint at high speed. Full dTerm is required only for position hold,
     * acceleration and deceleration
     * Scale down dTerm with 2D speed
     */
    const float setpointScale = computeVelocityScale(
        setpointXY,
        maxSpeed,
        multicopterPosXyCoefficients.dTermAttenuation,
        multicopterPosXyCoefficients.dTermAttenuationStart,
        multicopterPosXyCoefficients.dTermAttenuationEnd
    );
    
    const float measurementScale = computeVelocityScale(
        posControl.actualState.velXY,
        maxSpeed,
        multicopterPosXyCoefficients.dTermAttenuation,
        multicopterPosXyCoefficients.dTermAttenuationStart,
        multicopterPosXyCoefficients.dTermAttenuationEnd
    );

    // Choose smaller attenuation factor and convert from attenuation to scale
    const float dtermScale = 1.0f - MIN(setpointScale, measurementScale);

    // Apply PID with output limiting and I-term anti-windup
    // Pre-calculated accelLimit and the logic of navPidApply2 function guarantee that our newAccel won't exceed maxAccelLimit
    // Thus we don't need to do anything else with calculated acceleration
    float newAccelX = navPidApply3(
        &posControl.pids.vel[X],
        setpointX,
        measurementX,
        US2S(deltaMicros),
        accelLimitXMin,
        accelLimitXMax,
        0,      // Flags
        1.0f,   // Total gain scale
        dtermScale    // Additional dTerm scale
    );

    float newAccelY = navPidApply3(
        &posControl.pids.vel[Y],
        setpointY,
        measurementY,
        US2S(deltaMicros),
        accelLimitYMin,
        accelLimitYMax,
        0,      // Flags
        1.0f,   // Total gain scale
        dtermScale    // Additional dTerm scale
    );

    int32_t maxBankAngle = DEGREES_TO_DECIDEGREES(navConfig()->mc.max_bank_angle);

#ifdef USE_MR_BRAKING_MODE
    //Boost required accelerations
    if (STATE(NAV_CRUISE_BRAKING_BOOST) && multicopterPosXyCoefficients.breakingBoostFactor > 0.0f) {

        //Scale boost factor according to speed
        const float boostFactor = constrainf(
            scaleRangef(
                posControl.actualState.velXY,
                navConfig()->mc.braking_boost_speed_threshold,
                navConfig()->general.max_manual_speed,
                0.0f,
                multicopterPosXyCoefficients.breakingBoostFactor
            ),
            0.0f,
            multicopterPosXyCoefficients.breakingBoostFactor
        );

        //Boost required acceleration for harder braking
        newAccelX = newAccelX * (1.0f + boostFactor);
        newAccelY = newAccelY * (1.0f + boostFactor);

        maxBankAngle = DEGREES_TO_DECIDEGREES(navConfig()->mc.braking_bank_angle);
    }
#endif

    // Save last acceleration target
    lastAccelTargetX = newAccelX;
    lastAccelTargetY = newAccelY;

    // Rotate acceleration target into forward-right frame (aircraft)
    const float accelForward = newAccelX * posControl.actualState.cosYaw + newAccelY * posControl.actualState.sinYaw;
    const float accelRight = -newAccelX * posControl.actualState.sinYaw + newAccelY * posControl.actualState.cosYaw;

    // Calculate banking angles
    const float desiredPitch = atan2_approx(accelForward, GRAVITY_CMSS);
    const float desiredRoll = atan2_approx(accelRight * cos_approx(desiredPitch), GRAVITY_CMSS);

    posControl.rcAdjustment[ROLL] = constrain(RADIANS_TO_DECIDEGREES(desiredRoll), -maxBankAngle, maxBankAngle);
    posControl.rcAdjustment[PITCH] = constrain(RADIANS_TO_DECIDEGREES(desiredPitch), -maxBankAngle, maxBankAngle);

    debug[4] = posControl.rcAdjustment[ROLL];
    debug[5] = posControl.rcAdjustment[PITCH];
}

static void applyMulticopterPositionController(timeUs_t currentTimeUs)
{
    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    // and pilots input would be passed thru to PID controller
    if (posControl.flags.estPosStatus < EST_USABLE) {
        /* No position data, disable automatic adjustment, rcCommand passthrough */
        posControl.rcAdjustment[PITCH] = 0;
        posControl.rcAdjustment[ROLL] = 0;
        return;
    }

    // Passthrough rcCommand if adjusting position in GPS_ATTI mode except when Course Hold active
    bool bypassPositionController = !FLIGHT_MODE(NAV_COURSE_HOLD_MODE) &&
                                    navConfig()->general.flags.user_control_mode == NAV_GPS_ATTI &&
                                    posControl.flags.isAdjustingPosition;

    if (posControl.flags.horizontalPositionDataNew) {
        // Indicate that information is no longer usable
        posControl.flags.horizontalPositionDataConsumed = true;

        static timeUs_t previousTimePositionUpdate = 0;     // Occurs @ GPS update rate
        const timeDeltaLarge_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTimeUs;

        if (bypassPositionController) {
            return;
        }

        // If we have new position data - update velocity and acceleration controllers
        if (deltaMicrosPositionUpdate < MAX_POSITION_UPDATE_INTERVAL_US) {
            // Get max speed for current NAV mode
            float maxSpeed = getActiveSpeed();
            updatePositionVelocityController_MC(maxSpeed);
            updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCELERATION_XY_MAX, maxSpeed);
            updateMulticopterXYPosControl(US2S(deltaMicrosPositionUpdate));

            navDesiredVelocity[X] = constrain(lrintf(posControl.desiredState.vel.x), -32678, 32767);
            navDesiredVelocity[Y] = constrain(lrintf(posControl.desiredState.vel.y), -32678, 32767);
        }
        else {
            // Position update has not occurred in time (first start or glitch), reset position controller
            resetMulticopterPositionController();
        }
    } else if (bypassPositionController) {
        return;
    }

    rcCommand[PITCH] = pidAngleToRcCommand(posControl.rcAdjustment[PITCH], pidProfile()->max_angle_inclination[FD_PITCH]);
    rcCommand[ROLL] = pidAngleToRcCommand(posControl.rcAdjustment[ROLL], pidProfile()->max_angle_inclination[FD_ROLL]);
}

bool isMulticopterFlying(void)
{
    bool throttleCondition = rcCommand[THROTTLE] > currentBatteryProfile->nav.mc.hover_throttle;
    bool gyroCondition = averageAbsGyroRates() > 7.0f;

    return throttleCondition && gyroCondition;
}

/*-----------------------------------------------------------
 * Multicopter land detector
 *-----------------------------------------------------------*/
#if defined(USE_BARO)
static float baroAltRate;

void updateBaroAltitudeRate(float newBaroAltRate)
{
    baroAltRate = newBaroAltRate;
}

static bool isLandingGbumpDetected(timeMs_t currentTimeMs)
{
    /* Detection based on G bump at touchdown, falling Baro altitude and throttle below hover.
     * G bump trigger: > 2g then falling back below 1g in < 0.1s.
     * Baro trigger: rate must be -ve at initial trigger g and < -2 m/s when g falls back below 1g
     * Throttle trigger: must be below hover throttle with lower threshold for manual throttle control */

    static timeMs_t gSpikeDetectTimeMs = 0;

    if (!gSpikeDetectTimeMs && acc.accADCf[Z] > 2.0f && baroAltRate < 0.0f) {
        gSpikeDetectTimeMs = currentTimeMs;
    } else if (gSpikeDetectTimeMs) {
        if (currentTimeMs < gSpikeDetectTimeMs + 100) {
            if (acc.accADCf[Z] < 1.0f && baroAltRate < -200.0f) {
                const uint16_t idleThrottle = getThrottleIdleValue();
                const uint16_t hoverThrottleRange = currentBatteryProfile->nav.mc.hover_throttle - idleThrottle;
                return rcCommand[THROTTLE] < idleThrottle + ((navigationInAutomaticThrottleMode() ? 0.8 : 0.5) * hoverThrottleRange);
            }
        } else if (acc.accADCf[Z] <= 1.0f) {
            gSpikeDetectTimeMs = 0;
        }
    }

    return false;
}

bool isMulticopterCrashedInverted(timeMs_t currentTimeMs)
{
    /* Disarms MR if inverted on the ground. Checks vertical velocity is low based on Baro rate below 2 m/s */

    static timeMs_t startTime = 0;

    if ((ABS(attitude.values.roll) > 1000 || ABS(attitude.values.pitch) > 700) && fabsf(baroAltRate) < 200.0f) {
        if (startTime == 0) {
            startTime = currentTimeMs;
        }

        /* Minimum 3s disarm delay + extra user set delay time (min overall delay of 4s) */
        uint16_t disarmTimeDelay = 3000 + S2MS(navConfig()->mc.inverted_crash_detection);
        return currentTimeMs - startTime > disarmTimeDelay;
    }

    startTime = 0;
    return false;
}
#endif

bool isMulticopterLandingDetected(void)
{
    DEBUG_SET(DEBUG_LANDING, 4, 0);
    DEBUG_SET(DEBUG_LANDING, 3, averageAbsGyroRates() * 100);

    const timeMs_t currentTimeMs = millis();

#if defined(USE_BARO)
    if (sensors(SENSOR_BARO)) {
        /* Inverted crash landing detection - immediate disarm */
        if (navConfig()->mc.inverted_crash_detection && !FLIGHT_MODE(TURTLE_MODE) && isMulticopterCrashedInverted(currentTimeMs)) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_LANDING_DETECTED);
            disarm(DISARM_LANDING);
        }

        /* G bump landing detection *
         * Only used when xy velocity is low or failsafe is active */
        bool gBumpDetectionUsable = navConfig()->general.flags.landing_bump_detection &&
                                    ((posControl.flags.estPosStatus >= EST_USABLE && posControl.actualState.velXY < MC_LAND_CHECK_VEL_XY_MOVING) ||
                                    FLIGHT_MODE(FAILSAFE_MODE));

        if (gBumpDetectionUsable && isLandingGbumpDetected(currentTimeMs)) {
            return true;    // Landing flagged immediately if landing bump detected
        }
    }
#endif

    bool throttleIsBelowMidHover = rcCommand[THROTTLE] < (0.5 * (currentBatteryProfile->nav.mc.hover_throttle + getThrottleIdleValue()));

    /* Basic condition to start looking for landing
     * Detection active during Failsafe only if throttle below mid hover throttle
     * and WP mission not active (except landing states).
     * Also active in non autonomous flight modes but only when thottle low */
    bool startCondition = (navGetCurrentStateFlags() & (NAV_CTL_LAND | NAV_CTL_EMERG))
                          || (FLIGHT_MODE(FAILSAFE_MODE) && !FLIGHT_MODE(NAV_WP_MODE) && throttleIsBelowMidHover)
                          || (!navigationIsFlyingAutonomousMode() && throttleStickIsLow());

    static timeMs_t landingDetectorStartedAt;

    if (!startCondition || posControl.flags.resetLandingDetector) {
        landingDetectorStartedAt = 0;
        return posControl.flags.resetLandingDetector = false;
    }

    const float sensitivity = navConfig()->general.land_detect_sensitivity / 5.0f;

    // check vertical and horizontal velocities are low (cm/s)
    bool velCondition = fabsf(navGetCurrentActualPositionAndVelocity()->vel.z) < (MC_LAND_CHECK_VEL_Z_MOVING * sensitivity) &&
                        posControl.actualState.velXY < (MC_LAND_CHECK_VEL_XY_MOVING * sensitivity);
    // check gyro rates are low (degs/s)
    bool gyroCondition = averageAbsGyroRates() < (4.0f * sensitivity);
    DEBUG_SET(DEBUG_LANDING, 2, velCondition);
    DEBUG_SET(DEBUG_LANDING, 3, gyroCondition);

    bool possibleLandingDetected = false;

    if (navGetCurrentStateFlags() & NAV_CTL_LAND) {
        // We have likely landed if throttle is 40 units below average descend throttle
        // We use rcCommandAdjustedThrottle to keep track of NAV corrected throttle (isLandingDetected is executed
        // from processRx() and rcCommand at that moment holds rc input, not adjusted values from NAV core)
        DEBUG_SET(DEBUG_LANDING, 4, 1);

        static int32_t landingThrSum;
        static int32_t landingThrSamples;
        bool isAtMinimalThrust = false;

        if (!landingDetectorStartedAt) {
            landingThrSum = landingThrSamples = 0;
            landingDetectorStartedAt = currentTimeMs;
        }
        if (!landingThrSamples) {
            if (currentTimeMs - landingDetectorStartedAt < S2MS(MC_LAND_THR_STABILISE_DELAY)) {   // Wait for 1 second so throttle has stabilized.
                return false;
            } else {
                landingDetectorStartedAt = currentTimeMs;
            }
        }
        landingThrSamples += 1;
        landingThrSum += rcCommandAdjustedThrottle;
        isAtMinimalThrust = rcCommandAdjustedThrottle < (landingThrSum / landingThrSamples - MC_LAND_DESCEND_THROTTLE);

        possibleLandingDetected = isAtMinimalThrust && velCondition;

        DEBUG_SET(DEBUG_LANDING, 6, rcCommandAdjustedThrottle);
        DEBUG_SET(DEBUG_LANDING, 7, landingThrSum / landingThrSamples - MC_LAND_DESCEND_THROTTLE);
    } else {    // non autonomous and emergency landing
        DEBUG_SET(DEBUG_LANDING, 4, 2);
        if (landingDetectorStartedAt) {
            possibleLandingDetected = velCondition && gyroCondition;
        } else {
            landingDetectorStartedAt = currentTimeMs;
            return false;
        }
    }

    // If we have surface sensor (for example sonar) - use it to detect touchdown
    if ((posControl.flags.estAglStatus == EST_TRUSTED) && (posControl.actualState.agl.pos.z >= 0)) {
        // TODO: Come up with a clever way to let sonar increase detection performance, not just add extra safety.
        // TODO: Out of range sonar may give reading that looks like we landed, find a way to check if sonar is healthy.
        // surfaceMin is our ground reference. If we are less than 5cm above the ground - we are likely landed
        possibleLandingDetected = possibleLandingDetected && (posControl.actualState.agl.pos.z <= (posControl.actualState.surfaceMin + MC_LAND_SAFE_SURFACE));
    }
    DEBUG_SET(DEBUG_LANDING, 5, possibleLandingDetected);

    if (possibleLandingDetected) {
        /* Conditions need to be held for fixed safety time + optional extra delay.
         * Fixed time increased if Z velocity invalid to provide extra safety margin against false triggers */
        const uint16_t safetyTime = posControl.flags.estAltStatus == EST_NONE ? 5000 : 1000;
        timeMs_t safetyTimeDelay = safetyTime + navConfig()->general.auto_disarm_delay;
        return currentTimeMs - landingDetectorStartedAt > safetyTimeDelay;
    } else {
        landingDetectorStartedAt = currentTimeMs;
        return false;
    }
}

/*-----------------------------------------------------------
 * Multicopter emergency landing
 *-----------------------------------------------------------*/
static void applyMulticopterEmergencyLandingController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate = 0;

    /* Attempt to stabilise */
    rcCommand[YAW] = 0;
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;

    /* Altitude sensors gone haywire, attempt to land regardless */
    if (posControl.flags.estAltStatus < EST_USABLE) {
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_DROP_IT) {
            rcCommand[THROTTLE] = getThrottleIdleValue();
            return;
        }
        rcCommand[THROTTLE] = setDesiredThrottle(currentBatteryProfile->failsafe_throttle, true);
        return;
    }

    // Normal sensor data available, use controlled landing descent
    if (posControl.flags.verticalPositionDataNew) {
        const timeDeltaLarge_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTimeUs;

        // Check if last correction was not too long ago
        if (deltaMicrosPositionUpdate < MAX_POSITION_UPDATE_INTERVAL_US) {
            // target min descent rate at distance 2 x emerg descent rate above takeoff altitude
            updateClimbRateToAltitudeController(0, 2.0f * navConfig()->general.emerg_descent_rate, ROC_TO_ALT_TARGET);
            updateAltitudeVelocityController_MC(deltaMicrosPositionUpdate);
            updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
        }
        else {
            // due to some glitch position update has not occurred in time, reset altitude controller
            resetMulticopterAltitudeController();
        }

        // Indicate that information is no longer usable
        posControl.flags.verticalPositionDataConsumed = true;
    }

    // Update throttle
    rcCommand[THROTTLE] = posControl.rcAdjustment[THROTTLE];

    // Hold position if possible
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        applyMulticopterPositionController(currentTimeUs);
    }
}

/*-----------------------------------------------------------
 * Calculate loiter target based on current position and velocity
 *-----------------------------------------------------------*/
void calculateMulticopterInitialHoldPosition(fpVector3_t * pos)
{
    const float stoppingDistanceX = navGetCurrentActualPositionAndVelocity()->vel.x * posControl.posDecelerationTime;
    const float stoppingDistanceY = navGetCurrentActualPositionAndVelocity()->vel.y * posControl.posDecelerationTime;

    pos->x = navGetCurrentActualPositionAndVelocity()->pos.x + stoppingDistanceX;
    pos->y = navGetCurrentActualPositionAndVelocity()->pos.y + stoppingDistanceY;

    //getStoppingPointXY(pos); // TODO:
}

void resetMulticopterHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

static void applyMulticopterHeadingController(void)
{
    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {    // heading set by Nav during Course Hold so disable yaw stick input
        rcCommand[YAW] = 0;
    }

    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw));
}

void applyMulticopterNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    if (navStateFlags & NAV_CTL_EMERG) {
        applyMulticopterEmergencyLandingController(currentTimeUs);
    }
    else {
        if (navStateFlags & NAV_CTL_ALT)
            applyMulticopterAltitudeController(currentTimeUs);

        if (navStateFlags & NAV_CTL_POS)
            applyMulticopterPositionController(currentTimeUs);

        if (navStateFlags & NAV_CTL_YAW)
            applyMulticopterHeadingController();
    }
}

void resetMulticopterPositionController(void)
{
    for (int axis = 0; axis < 2; axis++) {
        navPidReset(&posControl.pids.vel[axis]);
        posControl.rcAdjustment[axis] = 0;
        lastAccelTargetX = 0.0f;
        lastAccelTargetY = 0.0f;
    }

    resetMulticopterXYPosControl();
}