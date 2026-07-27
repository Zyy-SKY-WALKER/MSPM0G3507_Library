/**
 * @file    chassis_motion.c
 * @brief   Non-blocking high-level differential-drive motion implementation.
 */

#include "chassis_motion.h"

#include <float.h>

#include "drive_geometry.h"
#include "drive_motor_config.h"
#include "speed_pid.h"
#include "zf_common_interrupt.h"

#define CHASSIS_MOTION_GAIN_LIMIT           (1000.0F)

/** @brief Reason retained while a controlled stop is in progress. */
typedef enum
{
    /** No controlled stop is active. */
    CHASSIS_MOTION_STOP_REASON_NONE = 0,
    /** The active command reached its completion condition. */
    CHASSIS_MOTION_STOP_REASON_COMPLETE,
    /** The caller cancelled the active command. */
    CHASSIS_MOTION_STOP_REASON_CANCEL,
    /** A pending command requires opposite wheel polarity. */
    CHASSIS_MOTION_STOP_REASON_REVERSE,
} chassis_motion_stop_reason_enum;

/** @brief Validated private representation of one motion request. */
typedef struct
{
    /** Command interpretation. */
    chassis_motion_command_enum command;
    /** Signed distance or relative angle. */
    float value;
    /** Signed linear speed or positive linear/angular speed limit. */
    float speed;
    /** Duration used only by a timed command. */
    uint32 duration_ms;
} chassis_motion_command_struct;

/** @brief Runtime state for one Smoothstep base-speed transition. */
typedef struct
{
    /** Base speed captured at transition start. */
    float start;
    /** Requested base speed at transition end. */
    float target;
    /** Current interpolated base speed. */
    float value;
    /** Configured transition length in scheduler ticks. */
    uint16 total_ticks;
    /** Number of completed transition ticks. */
    uint16 elapsed_ticks;
    /** Nonzero until the target endpoint is reached. */
    uint8 active;
} chassis_motion_transition_struct;

/** @brief Private heading PID gains and dynamic state. */
typedef struct
{
    float kp;
    float ki;
    float kd;
    /** Bounded accumulated integral contribution. */
    float integral;
    /** Error retained for the discrete derivative. */
    float previous_error;
    /** Latest wheel-speed correction. */
    float output;
} chassis_motion_heading_pid_struct;

static volatile chassis_motion_pid_profile_struct
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_COUNT];
static volatile uint8
    chassis_motion_pid_profile_configured[CHASSIS_MOTION_PID_PROFILE_COUNT];
static volatile chassis_motion_status_struct chassis_motion_status;
static chassis_motion_command_struct chassis_motion_command;
static chassis_motion_command_struct chassis_motion_pending_command;
static chassis_motion_transition_struct chassis_motion_speed_transition;
static chassis_motion_heading_pid_struct chassis_motion_heading_pid;
static chassis_motion_config_struct chassis_motion_config;
static chassis_motion_stop_reason_enum chassis_motion_stop_reason;
static uint16 chassis_motion_stopped_ticks;
static uint16 chassis_motion_stop_wait_ticks;
static uint8 chassis_motion_pending_valid;
static uint8 chassis_motion_activation_pending;
static float chassis_motion_turn_previous_heading_deg;

/**
 * @brief Return the absolute magnitude of a floating-point value.
 * @param value Input value.
 * @return Nonnegative magnitude.
 */
static float chassis_motion_abs(float value)
{
    if (value < 0.0F)
    {
        return -value;
    }

    return value;
}

/**
 * @brief Return the sign of a floating-point value.
 * @param value Input value.
 * @return 1 for positive, -1 for negative and 0 for zero.
 */
static int8 chassis_motion_get_sign(float value)
{
    if (value > 0.0F)
    {
        return 1;
    }

    if (value < 0.0F)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief Return whether one floating-point value is finite.
 * @param value Value to validate.
 * @return Nonzero when valid.
 */
static uint8 chassis_motion_value_is_valid(float value)
{
    return (uint8)((value == value)
        && (value >= -FLT_MAX)
        && (value <= FLT_MAX));
}

/**
 * @brief Return whether one PID gain is inside the accepted range.
 * @param gain Gain to validate.
 * @return Nonzero when valid.
 */
static uint8 chassis_motion_gain_is_valid(float gain)
{
    return (uint8)((chassis_motion_value_is_valid(gain) != 0U)
        && (gain >= -CHASSIS_MOTION_GAIN_LIMIT)
        && (gain <= CHASSIS_MOTION_GAIN_LIMIT));
}

/**
 * @brief Clamp one wheel speed to the speed PID target limit.
 * @param target_mm_s Requested wheel target.
 * @return Clamped wheel target.
 */
static float chassis_motion_clamp_wheel_target(float target_mm_s)
{
    if (target_mm_s > SPEED_PID_TARGET_LIMIT_MM_S)
    {
        return SPEED_PID_TARGET_LIMIT_MM_S;
    }

    if (target_mm_s < -SPEED_PID_TARGET_LIMIT_MM_S)
    {
        return -SPEED_PID_TARGET_LIMIT_MM_S;
    }

    return target_mm_s;
}

/**
 * @brief Wrap an angle into the inclusive range from -180 to 180 degrees.
 * @param angle_deg Input angle in degrees.
 * @return Wrapped angle in degrees.
 */
static float chassis_motion_wrap_angle_deg(float angle_deg)
{
    if (angle_deg > 180.0F)
    {
        angle_deg -= 360.0F;
    }
    else if (angle_deg < -180.0F)
    {
        angle_deg += 360.0F;
    }

    return angle_deg;
}

/**
 * @brief Calculate heading error while preserving a requested 180-degree turn.
 * @param target_heading_deg Desired heading in degrees.
 * @param current_heading_deg Current heading in degrees.
 * @return Signed shortest-path heading error in degrees.
 */
static float chassis_motion_get_heading_error(
    float target_heading_deg,
    float current_heading_deg)
{
    float error;

    error = chassis_motion_wrap_angle_deg(
        target_heading_deg - current_heading_deg);
    if ((error == -180.0F)
        && (chassis_motion_status.command_value > 0.0F))
    {
        error = 180.0F;
    }
    else if ((error == 180.0F)
        && (chassis_motion_status.command_value < 0.0F))
    {
        error = -180.0F;
    }

    return error;
}

/**
 * @brief Return the cubic Smoothstep progress for a normalized input.
 * @param progress Normalized input from 0.0 to 1.0.
 * @return Smoothed normalized output.
 */
static float chassis_motion_smoothstep(float progress)
{
    if (progress <= 0.0F)
    {
        return 0.0F;
    }

    if (progress >= 1.0F)
    {
        return 1.0F;
    }

    return progress * progress * (3.0F - (2.0F * progress));
}

/**
 * @brief Calculate the number of 10 ms ticks for one speed transition.
 * @return Positive transition tick count.
 */
static uint16 chassis_motion_get_transition_ticks(void)
{
    uint32 ticks;

    ticks = (uint32)(chassis_motion_config.speed_transition_ms
        + CHASSIS_MOTION_UPDATE_PERIOD_MS - 1U);
    ticks /= CHASSIS_MOTION_UPDATE_PERIOD_MS;
    if (ticks == 0U)
    {
        ticks = 1U;
    }
    else if (ticks > 0xFFFFU)
    {
        ticks = 0xFFFFU;
    }

    return (uint16)ticks;
}

/**
 * @brief Start a Smoothstep transition from the current base speed.
 * @param target Target base speed.
 */
static void chassis_motion_start_speed_transition(float target)
{
    chassis_motion_speed_transition.start =
        chassis_motion_speed_transition.value;
    chassis_motion_speed_transition.target = target;
    chassis_motion_speed_transition.total_ticks =
        chassis_motion_get_transition_ticks();
    chassis_motion_speed_transition.elapsed_ticks = 0U;
    chassis_motion_speed_transition.active = (uint8)(
        chassis_motion_speed_transition.start
        != chassis_motion_speed_transition.target);

    if (chassis_motion_speed_transition.active == 0U)
    {
        chassis_motion_speed_transition.value = target;
    }
}

/**
 * @brief Advance and return the current base speed transition.
 * @return Current interpolated base speed.
 */
static float chassis_motion_update_speed_transition(void)
{
    float progress;

    if (chassis_motion_speed_transition.active == 0U)
    {
        return chassis_motion_speed_transition.value;
    }

    if (chassis_motion_speed_transition.elapsed_ticks
        < chassis_motion_speed_transition.total_ticks)
    {
        chassis_motion_speed_transition.elapsed_ticks++;
    }

    progress = (float)chassis_motion_speed_transition.elapsed_ticks
        / (float)chassis_motion_speed_transition.total_ticks;
    chassis_motion_speed_transition.value =
        chassis_motion_speed_transition.start
        + ((chassis_motion_speed_transition.target
            - chassis_motion_speed_transition.start)
            * chassis_motion_smoothstep(progress));

    if (chassis_motion_speed_transition.elapsed_ticks
        >= chassis_motion_speed_transition.total_ticks)
    {
        chassis_motion_speed_transition.value =
            chassis_motion_speed_transition.target;
        chassis_motion_speed_transition.active = 0U;
    }

    return chassis_motion_speed_transition.value;
}

/**
 * @brief Clear heading PID runtime while preserving selected gains.
 */
static void chassis_motion_reset_heading_pid(void)
{
    chassis_motion_heading_pid.integral = 0.0F;
    chassis_motion_heading_pid.previous_error = 0.0F;
    chassis_motion_heading_pid.output = 0.0F;
}

/**
 * @brief Calculate heading PID correction from an explicit signed error.
 * @param error Signed heading error in degrees.
 * @param output_limit Nonnegative correction limit in millimeters per second.
 * @return Signed left-turn correction in millimeters per second.
 */
static float chassis_motion_update_heading_pid_error(
    float error,
    float output_limit)
{
    chassis_motion_heading_pid.integral +=
        chassis_motion_heading_pid.ki * error;
    if (chassis_motion_heading_pid.integral > output_limit)
    {
        chassis_motion_heading_pid.integral = output_limit;
    }
    else if (chassis_motion_heading_pid.integral < -output_limit)
    {
        chassis_motion_heading_pid.integral = -output_limit;
    }
    chassis_motion_heading_pid.output =
        (chassis_motion_heading_pid.kp * error)
        + chassis_motion_heading_pid.integral
        + (chassis_motion_heading_pid.kd
            * (error - chassis_motion_heading_pid.previous_error));
    chassis_motion_heading_pid.previous_error = error;

    return chassis_motion_heading_pid.output;
}

/**
 * @brief Calculate heading PID correction from wrapped headings.
 * @param target_heading_deg Desired heading in degrees.
 * @param current_heading_deg Current heading in degrees.
 * @param output_limit Nonnegative correction limit in millimeters per second.
 * @return Signed left-turn correction in millimeters per second.
 */
static float chassis_motion_update_heading_pid(
    float target_heading_deg,
    float current_heading_deg,
    float output_limit)
{
    return chassis_motion_update_heading_pid_error(
        chassis_motion_get_heading_error(
            target_heading_deg,
            current_heading_deg),
        output_limit);
}

/**
 * @brief Return whether one PID profile has valid gain values.
 * @param profile PID profile to validate.
 * @return Nonzero when valid.
 */
static uint8 chassis_motion_pid_profile_is_valid(
    const chassis_motion_pid_profile_struct *profile)
{
    if (profile == NULL)
    {
        return 0U;
    }

    return (uint8)(
        (chassis_motion_gain_is_valid(profile->left_speed_kp) != 0U)
        && (chassis_motion_gain_is_valid(profile->left_speed_ki) != 0U)
        && (chassis_motion_gain_is_valid(profile->left_speed_kd) != 0U)
        && (chassis_motion_gain_is_valid(profile->right_speed_kp) != 0U)
        && (chassis_motion_gain_is_valid(profile->right_speed_ki) != 0U)
        && (chassis_motion_gain_is_valid(profile->right_speed_kd) != 0U)
        && (chassis_motion_gain_is_valid(profile->heading_kp) != 0U)
        && (chassis_motion_gain_is_valid(profile->heading_ki) != 0U)
        && (chassis_motion_gain_is_valid(profile->heading_kd) != 0U));
}

/**
 * @brief Calculate the signed wheel targets implied by one command.
 * @param command Command to inspect.
 * @param left_target Destination left target.
 * @param right_target Destination right target.
 */
static void chassis_motion_get_command_targets(
    const chassis_motion_command_struct *command,
    float *left_target,
    float *right_target)
{
    float wheel_speed;

    *left_target = 0.0F;
    *right_target = 0.0F;

    if (command->command == CHASSIS_MOTION_COMMAND_DISTANCE)
    {
        wheel_speed = command->speed;
        if (command->value < 0.0F)
        {
            wheel_speed = -wheel_speed;
        }
        *left_target = wheel_speed;
        *right_target = wheel_speed;
    }
    else if (command->command == CHASSIS_MOTION_COMMAND_TIMED)
    {
        *left_target = command->speed;
        *right_target = command->speed;
    }
    else if (command->command == CHASSIS_MOTION_COMMAND_TURN_RELATIVE)
    {
        wheel_speed = command->speed
            * (DRIVE_PI / 180.0F)
            * (DRIVE_TRACK_WIDTH_MM * 0.5F);
        if (command->value < 0.0F)
        {
            wheel_speed = -wheel_speed;
        }
        *left_target = -wheel_speed;
        *right_target = wheel_speed;
    }
}

/**
 * @brief Return whether a new command requires a full stopped reversal.
 * @param command New command to inspect.
 * @return Nonzero when either wheel target changes sign.
 */
static uint8 chassis_motion_command_requires_stop(
    const chassis_motion_command_struct *command)
{
    float left_target;
    float right_target;
    int8 previous_left_sign;
    int8 previous_right_sign;
    int8 new_left_sign;
    int8 new_right_sign;

    chassis_motion_get_command_targets(command, &left_target, &right_target);
    previous_left_sign = chassis_motion_get_sign(
        chassis_motion_status.left_target_mm_s);
    previous_right_sign = chassis_motion_get_sign(
        chassis_motion_status.right_target_mm_s);
    new_left_sign = chassis_motion_get_sign(left_target);
    new_right_sign = chassis_motion_get_sign(right_target);

    return (uint8)(((previous_left_sign != 0)
            && (new_left_sign != 0)
            && (previous_left_sign != new_left_sign))
        || ((previous_right_sign != 0)
            && (new_right_sign != 0)
            && (previous_right_sign != new_right_sign)));
}

/**
 * @brief Start the controlled transition to zero for one stop reason.
 * @param reason Reason the current command is stopping.
 */
static void chassis_motion_start_stop(
    chassis_motion_stop_reason_enum reason)
{
    chassis_motion_stop_reason = reason;
    chassis_motion_stopped_ticks = 0U;
    chassis_motion_stop_wait_ticks = 0U;
    chassis_motion_status.phase = CHASSIS_MOTION_PHASE_DECELERATING;
    chassis_motion_start_speed_transition(0.0F);
}

/**
 * @brief Activate the stored command using the latest measured pose.
 * @param odometry Latest odometry state.
 * @param yaw_deg Latest IMU yaw in degrees.
 */
static void chassis_motion_activate_command(
    const odometry_state_struct *odometry,
    float yaw_deg)
{
    float base_target;

    chassis_motion_activation_pending = 0U;
    chassis_motion_status.command = chassis_motion_command.command;
    chassis_motion_status.command_value = chassis_motion_command.value;
    chassis_motion_status.command_speed = chassis_motion_command.speed;
    chassis_motion_status.start_center_displacement_mm =
        odometry->center_displacement_mm;
    chassis_motion_status.current_center_displacement_mm =
        odometry->center_displacement_mm;
    chassis_motion_status.start_heading_deg = yaw_deg;
    chassis_motion_status.current_heading_deg = yaw_deg;
    chassis_motion_status.turn_progress_deg = 0.0F;
    chassis_motion_status.turn_remaining_deg = 0.0F;
    chassis_motion_turn_previous_heading_deg = yaw_deg;
    chassis_motion_status.elapsed_ms = 0U;
    chassis_motion_status.result = CHASSIS_MOTION_RESULT_NONE;
    chassis_motion_status.active = 1U;
    chassis_motion_status.reversing = 0U;
    chassis_motion_stop_reason = CHASSIS_MOTION_STOP_REASON_NONE;
    chassis_motion_reset_heading_pid();

    if (chassis_motion_command.command == CHASSIS_MOTION_COMMAND_DISTANCE)
    {
        chassis_motion_status.target_heading_deg = yaw_deg;
        base_target = chassis_motion_command.speed;
        if (chassis_motion_command.value < 0.0F)
        {
            base_target = -base_target;
        }
    }
    else if (chassis_motion_command.command == CHASSIS_MOTION_COMMAND_TIMED)
    {
        chassis_motion_status.target_heading_deg = yaw_deg;
        base_target = chassis_motion_command.speed;
    }
    else
    {
        chassis_motion_status.target_heading_deg =
            chassis_motion_wrap_angle_deg(
                yaw_deg + chassis_motion_command.value);
        chassis_motion_status.turn_remaining_deg =
            chassis_motion_command.value;
        base_target = chassis_motion_command.speed
            * (DRIVE_PI / 180.0F)
            * (DRIVE_TRACK_WIDTH_MM * 0.5F);
    }

    chassis_motion_start_speed_transition(base_target);
    chassis_motion_status.phase = CHASSIS_MOTION_PHASE_TRANSITION;
}

/**
 * @brief Accumulate one wrapped IMU increment for a relative turn.
 * @param yaw_deg Latest wrapped IMU yaw in degrees.
 */
static void chassis_motion_update_turn_progress(float yaw_deg)
{
    float heading_delta;

    if (chassis_motion_status.command
        != CHASSIS_MOTION_COMMAND_TURN_RELATIVE)
    {
        return;
    }

    heading_delta = chassis_motion_wrap_angle_deg(
        yaw_deg - chassis_motion_turn_previous_heading_deg);
    chassis_motion_turn_previous_heading_deg = yaw_deg;
    chassis_motion_status.turn_progress_deg += heading_delta;
    chassis_motion_status.turn_remaining_deg =
        chassis_motion_status.command_value
        - chassis_motion_status.turn_progress_deg;
}

/**
 * @brief Check whether the current command has reached its completion condition.
 * @return Nonzero when complete.
 */
static uint8 chassis_motion_command_is_complete(void)
{
    float distance;

    if (chassis_motion_status.command == CHASSIS_MOTION_COMMAND_DISTANCE)
    {
        distance = chassis_motion_status.current_center_displacement_mm
            - chassis_motion_status.start_center_displacement_mm;
        if (chassis_motion_status.command_value > 0.0F)
        {
            return (uint8)(distance
                >= chassis_motion_status.command_value);
        }

        return (uint8)(distance <= chassis_motion_status.command_value);
    }

    if (chassis_motion_status.command == CHASSIS_MOTION_COMMAND_TIMED)
    {
        return (uint8)(chassis_motion_status.elapsed_ms
            >= chassis_motion_command.duration_ms);
    }

    if (chassis_motion_status.command
        == CHASSIS_MOTION_COMMAND_TURN_RELATIVE)
    {
        if (chassis_motion_status.command_value > 0.0F)
        {
            return (uint8)(chassis_motion_status.turn_progress_deg
                >= (chassis_motion_status.command_value
                    - CHASSIS_MOTION_TURN_TOLERANCE_DEG));
        }

        return (uint8)(chassis_motion_status.turn_progress_deg
            <= (chassis_motion_status.command_value
                + CHASSIS_MOTION_TURN_TOLERANCE_DEG));
    }

    return 1U;
}

/**
 * @brief Generate target speeds for the current active command.
 * @param yaw_deg Latest IMU yaw in degrees.
 * @param left_target Destination left wheel target.
 * @param right_target Destination right wheel target.
 */
static void chassis_motion_generate_targets(
    float yaw_deg,
    float *left_target,
    float *right_target)
{
    float base_speed;
    float correction;
    float correction_limit;
    float heading_error;
    float minimum_correction;
    float available_speed;

    base_speed = chassis_motion_update_speed_transition();
    if (chassis_motion_status.command
        == CHASSIS_MOTION_COMMAND_TURN_RELATIVE)
    {
        correction_limit = chassis_motion_abs(base_speed);
        heading_error = chassis_motion_status.turn_remaining_deg;
        correction = chassis_motion_update_heading_pid_error(
            heading_error,
            correction_limit);
        if (correction > correction_limit)
        {
            correction = correction_limit;
        }
        else if (correction < -correction_limit)
        {
            correction = -correction_limit;
        }

        minimum_correction = correction_limit;
        if (minimum_correction
            > CHASSIS_MOTION_TURN_MIN_WHEEL_SPEED_MM_S)
        {
            minimum_correction =
                CHASSIS_MOTION_TURN_MIN_WHEEL_SPEED_MM_S;
        }
        if ((chassis_motion_abs(heading_error)
                > CHASSIS_MOTION_TURN_TOLERANCE_DEG)
            && (chassis_motion_abs(correction) < minimum_correction))
        {
            correction = heading_error > 0.0F
                ? minimum_correction
                : -minimum_correction;
        }
        if (chassis_motion_status.phase
            == CHASSIS_MOTION_PHASE_DECELERATING)
        {
            if ((chassis_motion_status.command_value > 0.0F)
                && (correction < 0.0F))
            {
                correction = 0.0F;
            }
            else if ((chassis_motion_status.command_value < 0.0F)
                && (correction > 0.0F))
            {
                correction = 0.0F;
            }
        }

        *left_target = -correction;
        *right_target = correction;
    }
    else
    {
        correction_limit = chassis_motion_abs(base_speed) * 0.5F;
        available_speed = SPEED_PID_TARGET_LIMIT_MM_S
            - chassis_motion_abs(base_speed);
        if (available_speed < correction_limit)
        {
            correction_limit = available_speed;
        }

        correction = chassis_motion_update_heading_pid(
            chassis_motion_status.target_heading_deg,
            yaw_deg,
            correction_limit);

        if (correction > correction_limit)
        {
            correction = correction_limit;
        }
        else if (correction < -correction_limit)
        {
            correction = -correction_limit;
        }

        *left_target = base_speed - correction;
        *right_target = base_speed + correction;
    }

    *left_target = chassis_motion_clamp_wheel_target(*left_target);
    *right_target = chassis_motion_clamp_wheel_target(*right_target);
}

/**
 * @brief Return whether both encoder deltas are inside the stopped threshold.
 * @param left_count Latest signed left encoder delta.
 * @param right_count Latest signed right encoder delta.
 * @return Nonzero when both wheels are effectively stopped.
 */
static uint8 chassis_motion_wheels_are_stopped(
    int16 left_count,
    int16 right_count)
{
    int32 left_magnitude = left_count;
    int32 right_magnitude = right_count;

    if (left_magnitude < 0)
    {
        left_magnitude = -left_magnitude;
    }
    if (right_magnitude < 0)
    {
        right_magnitude = -right_magnitude;
    }

    return (uint8)((left_magnitude <= CHASSIS_MOTION_STOPPED_COUNT_LIMIT)
        && (right_magnitude <= CHASSIS_MOTION_STOPPED_COUNT_LIMIT));
}

/**
 * @brief Complete the controlled stop after confirming both wheels are slow.
 */
static void chassis_motion_finish_stop(void)
{
    chassis_motion_status.left_target_mm_s = 0.0F;
    chassis_motion_status.right_target_mm_s = 0.0F;
    chassis_motion_speed_transition.value = 0.0F;
    chassis_motion_speed_transition.active = 0U;
    chassis_motion_stopped_ticks = 0U;

    if ((chassis_motion_stop_reason == CHASSIS_MOTION_STOP_REASON_REVERSE)
        && (chassis_motion_pending_valid != 0U))
    {
        /* A confirmed reversal stop transfers ownership to the new command. */
        chassis_motion_command = chassis_motion_pending_command;
        chassis_motion_pending_valid = 0U;
        chassis_motion_activation_pending = 1U;
        chassis_motion_status.reversing = 0U;
        chassis_motion_status.phase = CHASSIS_MOTION_PHASE_TRANSITION;
        return;
    }

    chassis_motion_status.active = 0U;
    if (chassis_motion_stop_reason == CHASSIS_MOTION_STOP_REASON_CANCEL)
    {
        chassis_motion_status.phase = CHASSIS_MOTION_PHASE_CANCELLED;
        chassis_motion_status.result = CHASSIS_MOTION_RESULT_CANCELLED;
    }
    else
    {
        chassis_motion_status.phase = CHASSIS_MOTION_PHASE_COMPLETED;
        chassis_motion_status.result = CHASSIS_MOTION_RESULT_COMPLETED;
    }

    chassis_motion_stop_reason = CHASSIS_MOTION_STOP_REASON_NONE;
}

/**
 * @brief Update stopped-wheel confirmation after a controlled zero target.
 * @param left_count Latest signed left encoder delta.
 * @param right_count Latest signed right encoder delta.
 */
static void chassis_motion_update_stop_confirmation(
    int16 left_count,
    int16 right_count)
{
    if (chassis_motion_stop_wait_ticks < 0xFFFFU)
    {
        chassis_motion_stop_wait_ticks++;
    }

    if (chassis_motion_wheels_are_stopped(left_count, right_count) != 0U)
    {
        if (chassis_motion_stopped_ticks < 0xFFFFU)
        {
            chassis_motion_stopped_ticks++;
        }
    }
    else
    {
        chassis_motion_stopped_ticks = 0U;
    }

    if (chassis_motion_stopped_ticks
        >= CHASSIS_MOTION_STOPPED_CONFIRM_TICKS)
    {
        chassis_motion_finish_stop();
    }
    else if (chassis_motion_stop_wait_ticks
        >= CHASSIS_MOTION_STOP_TIMEOUT_TICKS)
    {
        if (chassis_motion_stop_reason == CHASSIS_MOTION_STOP_REASON_REVERSE)
        {
            chassis_motion_pending_valid = 0U;
            chassis_motion_stop_reason = CHASSIS_MOTION_STOP_REASON_CANCEL;
        }
        chassis_motion_finish_stop();
    }
}

/**
 * @brief Submit one validated command for immediate execution or reversal stop.
 * @param command New motion command.
 * @return ZF_TRUE when accepted.
 */
static uint8 chassis_motion_submit_command(
    const chassis_motion_command_struct *command)
{
    if ((chassis_motion_status.active_profile_id
            >= CHASSIS_MOTION_PID_PROFILE_COUNT)
        || (chassis_motion_pid_profile_configured[
                chassis_motion_status.active_profile_id] == 0U))
    {
        return ZF_FALSE;
    }

    if ((chassis_motion_status.active != 0U)
        && (chassis_motion_command_requires_stop(command) != 0U))
    {
        /* Opposite wheel polarity must decelerate to a confirmed stop first. */
        chassis_motion_pending_command = *command;
        chassis_motion_pending_valid = 1U;
        chassis_motion_status.reversing = 1U;
        chassis_motion_start_stop(CHASSIS_MOTION_STOP_REASON_REVERSE);
        return ZF_TRUE;
    }

    chassis_motion_command = *command;
    chassis_motion_pending_valid = 0U;
    chassis_motion_activation_pending = 1U;
    chassis_motion_status.active = 1U;
    chassis_motion_status.result = CHASSIS_MOTION_RESULT_NONE;
    chassis_motion_status.reversing = 0U;
    chassis_motion_status.phase = CHASSIS_MOTION_PHASE_TRANSITION;
    return ZF_TRUE;
}

/**
 * @brief Initialize motion state and load active-motor PID profiles.
 */
void chassis_motion_init(void)
{
    uint8 index;

    for (index = 0U;
        index < CHASSIS_MOTION_PID_PROFILE_COUNT;
        index++)
    {
        chassis_motion_pid_profiles[index].left_speed_kp = 0.0F;
        chassis_motion_pid_profiles[index].left_speed_ki = 0.0F;
        chassis_motion_pid_profiles[index].left_speed_kd = 0.0F;
        chassis_motion_pid_profiles[index].right_speed_kp = 0.0F;
        chassis_motion_pid_profiles[index].right_speed_ki = 0.0F;
        chassis_motion_pid_profiles[index].right_speed_kd = 0.0F;
        chassis_motion_pid_profiles[index].heading_kp = 0.0F;
        chassis_motion_pid_profiles[index].heading_ki = 0.0F;
        chassis_motion_pid_profiles[index].heading_kd = 0.0F;
        chassis_motion_pid_profile_configured[index] = 0U;
    }

    chassis_motion_config.speed_transition_ms =
        CHASSIS_MOTION_DEFAULT_SPEED_TRANSITION_MS;
    chassis_motion_status.command = CHASSIS_MOTION_COMMAND_NONE;
    chassis_motion_status.phase = CHASSIS_MOTION_PHASE_IDLE;
    chassis_motion_status.result = CHASSIS_MOTION_RESULT_NONE;
    chassis_motion_status.command_value = 0.0F;
    chassis_motion_status.command_speed = 0.0F;
    chassis_motion_status.start_center_displacement_mm = 0.0F;
    chassis_motion_status.current_center_displacement_mm = 0.0F;
    chassis_motion_status.start_heading_deg = 0.0F;
    chassis_motion_status.current_heading_deg = 0.0F;
    chassis_motion_status.target_heading_deg = 0.0F;
    chassis_motion_status.turn_progress_deg = 0.0F;
    chassis_motion_status.turn_remaining_deg = 0.0F;
    chassis_motion_status.left_target_mm_s = 0.0F;
    chassis_motion_status.right_target_mm_s = 0.0F;
    chassis_motion_status.elapsed_ms = 0U;
    chassis_motion_status.active_profile_id =
        CHASSIS_MOTION_PID_PROFILE_INVALID;
    chassis_motion_status.active = 0U;
    chassis_motion_status.reversing = 0U;

    chassis_motion_command.command = CHASSIS_MOTION_COMMAND_NONE;
    chassis_motion_command.value = 0.0F;
    chassis_motion_command.speed = 0.0F;
    chassis_motion_command.duration_ms = 0U;
    chassis_motion_pending_command = chassis_motion_command;
    chassis_motion_speed_transition.start = 0.0F;
    chassis_motion_speed_transition.target = 0.0F;
    chassis_motion_speed_transition.value = 0.0F;
    chassis_motion_speed_transition.total_ticks = 0U;
    chassis_motion_speed_transition.elapsed_ticks = 0U;
    chassis_motion_speed_transition.active = 0U;
    chassis_motion_heading_pid.kp = 0.0F;
    chassis_motion_heading_pid.ki = 0.0F;
    chassis_motion_heading_pid.kd = 0.0F;
    chassis_motion_reset_heading_pid();
    chassis_motion_stop_reason = CHASSIS_MOTION_STOP_REASON_NONE;
    chassis_motion_stopped_ticks = 0U;
    chassis_motion_stop_wait_ticks = 0U;
    chassis_motion_pending_valid = 0U;
    chassis_motion_activation_pending = 0U;
    chassis_motion_turn_previous_heading_deg = 0.0F;

    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .left_speed_kp = DRIVE_PROFILE_STRAIGHT_LEFT_KP;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .left_speed_ki = DRIVE_PROFILE_STRAIGHT_LEFT_KI;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .left_speed_kd = DRIVE_PROFILE_STRAIGHT_LEFT_KD;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .right_speed_kp = DRIVE_PROFILE_STRAIGHT_RIGHT_KP;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .right_speed_ki = DRIVE_PROFILE_STRAIGHT_RIGHT_KI;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .right_speed_kd = DRIVE_PROFILE_STRAIGHT_RIGHT_KD;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .heading_kp = DRIVE_PROFILE_HEADING_KP;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .heading_ki = DRIVE_PROFILE_HEADING_KI;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        .heading_kd = DRIVE_PROFILE_HEADING_KD;
    chassis_motion_pid_profile_configured[CHASSIS_MOTION_PID_PROFILE_STRAIGHT]
        = 1U;

    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .left_speed_kp = DRIVE_PROFILE_TURN_LEFT_KP;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .left_speed_ki = DRIVE_PROFILE_TURN_LEFT_KI;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .left_speed_kd = DRIVE_PROFILE_TURN_LEFT_KD;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .right_speed_kp = DRIVE_PROFILE_TURN_RIGHT_KP;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .right_speed_ki = DRIVE_PROFILE_TURN_RIGHT_KI;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .right_speed_kd = DRIVE_PROFILE_TURN_RIGHT_KD;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .heading_kp = DRIVE_PROFILE_HEADING_KP;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .heading_ki = DRIVE_PROFILE_HEADING_KI;
    chassis_motion_pid_profiles[CHASSIS_MOTION_PID_PROFILE_TURN]
        .heading_kd = DRIVE_PROFILE_HEADING_KD;
    chassis_motion_pid_profile_configured[CHASSIS_MOTION_PID_PROFILE_TURN]
        = 1U;
}

/**
 * @brief Configure one reusable speed and heading PID parameter group.
 * @param profile_id Profile identifier from 0 to 3.
 * @param profile Independent wheel-speed and heading PID gains.
 * @return ZF_TRUE when the profile was accepted.
 */
uint8 chassis_motion_pid_profile_configure(
    uint8 profile_id,
    const chassis_motion_pid_profile_struct *profile)
{
    uint32 primask;

    if ((profile_id >= CHASSIS_MOTION_PID_PROFILE_COUNT)
        || (chassis_motion_pid_profile_is_valid(profile) == 0U))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    chassis_motion_pid_profiles[profile_id] = *profile;
    chassis_motion_pid_profile_configured[profile_id] = 1U;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Apply one configured PID parameter group without resetting runtime state.
 * @param profile_id Profile identifier from 0 to 3.
 * @return ZF_TRUE when the configured profile was selected.
 */
uint8 chassis_motion_pid_profile_select(uint8 profile_id)
{
    chassis_motion_pid_profile_struct profile;

    if ((profile_id >= CHASSIS_MOTION_PID_PROFILE_COUNT)
        || (chassis_motion_pid_profile_configured[profile_id] == 0U))
    {
        return ZF_FALSE;
    }

    profile = chassis_motion_pid_profiles[profile_id];
    speed_pid_set_left_gains(
        profile.left_speed_kp,
        profile.left_speed_ki,
        profile.left_speed_kd);
    speed_pid_set_right_gains(
        profile.right_speed_kp,
        profile.right_speed_ki,
        profile.right_speed_kd);

    chassis_motion_heading_pid.kp = profile.heading_kp;
    chassis_motion_heading_pid.ki = profile.heading_ki;
    chassis_motion_heading_pid.kd = profile.heading_kd;
    chassis_motion_status.active_profile_id = profile_id;

    return ZF_TRUE;
}

/**
 * @brief Return the currently selected PID parameter group.
 * @return Profile identifier, or CHASSIS_MOTION_PID_PROFILE_INVALID.
 */
uint8 chassis_motion_pid_profile_get_active(void)
{
    uint8 profile_id;
    uint32 primask;

    primask = interrupt_global_disable();
    profile_id = chassis_motion_status.active_profile_id;
    interrupt_global_enable(primask);

    return profile_id;
}

/**
 * @brief Update the motion configuration used by subsequent transitions.
 * @param config Motion configuration.
 * @return ZF_TRUE when the configuration was accepted.
 */
uint8 chassis_motion_set_config(const chassis_motion_config_struct *config)
{
    uint32 primask;

    if ((config == NULL)
        || (config->speed_transition_ms
            < CHASSIS_MOTION_UPDATE_PERIOD_MS))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    chassis_motion_config = *config;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Start or smoothly replan a signed distance command.
 * @param distance_mm Positive for forward and negative for reverse travel.
 * @param max_speed_mm_s Positive maximum center speed in millimeters per second.
 * @return ZF_TRUE when the command was accepted.
 */
uint8 chassis_motion_start_distance(
    float distance_mm,
    float max_speed_mm_s)
{
    chassis_motion_command_struct command;

    if ((chassis_motion_value_is_valid(distance_mm) == 0U)
        || (chassis_motion_value_is_valid(max_speed_mm_s) == 0U)
        || (distance_mm == 0.0F)
        || (max_speed_mm_s <= 0.0F)
        || (max_speed_mm_s > SPEED_PID_TARGET_LIMIT_MM_S))
    {
        return ZF_FALSE;
    }

    command.command = CHASSIS_MOTION_COMMAND_DISTANCE;
    command.value = distance_mm;
    command.speed = max_speed_mm_s;
    command.duration_ms = 0U;
    return chassis_motion_submit_command(&command);
}

/**
 * @brief Start or smoothly replan a timed signed-speed command.
 * @param speed_mm_s Positive for forward and negative for reverse speed.
 * @param duration_ms Command duration in milliseconds.
 * @return ZF_TRUE when the command was accepted.
 */
uint8 chassis_motion_start_timed(
    float speed_mm_s,
    uint32 duration_ms)
{
    chassis_motion_command_struct command;

    if ((chassis_motion_value_is_valid(speed_mm_s) == 0U)
        || (speed_mm_s == 0.0F)
        || (chassis_motion_abs(speed_mm_s) > SPEED_PID_TARGET_LIMIT_MM_S)
        || (duration_ms == 0U))
    {
        return ZF_FALSE;
    }

    command.command = CHASSIS_MOTION_COMMAND_TIMED;
    command.value = 0.0F;
    command.speed = speed_mm_s;
    command.duration_ms = duration_ms;
    return chassis_motion_submit_command(&command);
}

/**
 * @brief Start or smoothly replan a relative IMU-heading turn.
 * @param angle_deg Relative angle from -360 to 360 degrees, excluding zero.
 *                  Positive turns left and negative turns right.
 * @param max_angular_speed_deg_s Positive angular-speed limit in degrees per second.
 * @return ZF_TRUE when the command was accepted.
 */
uint8 chassis_motion_start_turn_relative(
    float angle_deg,
    float max_angular_speed_deg_s)
{
    chassis_motion_command_struct command;
    float wheel_speed;

    if ((chassis_motion_value_is_valid(angle_deg) == 0U)
        || (chassis_motion_value_is_valid(max_angular_speed_deg_s) == 0U)
        || (angle_deg == 0.0F)
        || (angle_deg < -CHASSIS_MOTION_TURN_MAX_ANGLE_DEG)
        || (angle_deg > CHASSIS_MOTION_TURN_MAX_ANGLE_DEG)
        || (max_angular_speed_deg_s <= 0.0F))
    {
        return ZF_FALSE;
    }

    wheel_speed = max_angular_speed_deg_s
        * (DRIVE_PI / 180.0F)
        * (DRIVE_TRACK_WIDTH_MM * 0.5F);
    if (wheel_speed > SPEED_PID_TARGET_LIMIT_MM_S)
    {
        return ZF_FALSE;
    }

    command.command = CHASSIS_MOTION_COMMAND_TURN_RELATIVE;
    command.value = angle_deg;
    command.speed = max_angular_speed_deg_s;
    command.duration_ms = 0U;
    return chassis_motion_submit_command(&command);
}

/**
 * @brief Smoothly reduce the current motion target to zero and cancel it.
 */
void chassis_motion_cancel(void)
{
    if (chassis_motion_status.active == 0U)
    {
        return;
    }

    chassis_motion_pending_valid = 0U;
    chassis_motion_status.reversing = 0U;
    chassis_motion_start_stop(CHASSIS_MOTION_STOP_REASON_CANCEL);
}

/**
 * @brief Immediately clear motion state and force this module's target to zero.
 */
void chassis_motion_reset(void)
{
    chassis_motion_command.command = CHASSIS_MOTION_COMMAND_NONE;
    chassis_motion_command.value = 0.0F;
    chassis_motion_command.speed = 0.0F;
    chassis_motion_command.duration_ms = 0U;
    chassis_motion_pending_command = chassis_motion_command;
    chassis_motion_pending_valid = 0U;
    chassis_motion_activation_pending = 0U;
    chassis_motion_stop_reason = CHASSIS_MOTION_STOP_REASON_NONE;
    chassis_motion_stopped_ticks = 0U;
    chassis_motion_stop_wait_ticks = 0U;
    chassis_motion_speed_transition.start = 0.0F;
    chassis_motion_speed_transition.target = 0.0F;
    chassis_motion_speed_transition.value = 0.0F;
    chassis_motion_speed_transition.total_ticks = 0U;
    chassis_motion_speed_transition.elapsed_ticks = 0U;
    chassis_motion_speed_transition.active = 0U;
    chassis_motion_reset_heading_pid();
    chassis_motion_status.command = CHASSIS_MOTION_COMMAND_NONE;
    chassis_motion_status.phase = CHASSIS_MOTION_PHASE_IDLE;
    chassis_motion_status.result = CHASSIS_MOTION_RESULT_NONE;
    chassis_motion_status.command_value = 0.0F;
    chassis_motion_status.command_speed = 0.0F;
    chassis_motion_status.turn_progress_deg = 0.0F;
    chassis_motion_status.turn_remaining_deg = 0.0F;
    chassis_motion_status.left_target_mm_s = 0.0F;
    chassis_motion_status.right_target_mm_s = 0.0F;
    chassis_motion_status.elapsed_ms = 0U;
    chassis_motion_status.active = 0U;
    chassis_motion_status.reversing = 0U;
    chassis_motion_turn_previous_heading_deg = 0.0F;
}

/**
 * @brief Advance one non-blocking motion command by one 10 ms period.
 * @param odometry Latest odometry state from this scheduler period.
 * @param yaw_deg Latest IMU yaw angle in degrees.
 * @param left_count Latest signed left encoder delta.
 * @param right_count Latest signed right encoder delta.
 * @param left_target_mm_s Destination left wheel speed target.
 * @param right_target_mm_s Destination right wheel speed target.
 */
void chassis_motion_update_10ms(
    const odometry_state_struct *odometry,
    float yaw_deg,
    int16 left_count,
    int16 right_count,
    float *left_target_mm_s,
    float *right_target_mm_s)
{
    float left_target = 0.0F;
    float right_target = 0.0F;

    if ((odometry == NULL)
        || (left_target_mm_s == NULL)
        || (right_target_mm_s == NULL))
    {
        return;
    }

    chassis_motion_status.current_center_displacement_mm =
        odometry->center_displacement_mm;
    chassis_motion_status.current_heading_deg = yaw_deg;

    if (chassis_motion_status.active == 0U)
    {
        chassis_motion_status.left_target_mm_s = 0.0F;
        chassis_motion_status.right_target_mm_s = 0.0F;
        *left_target_mm_s = 0.0F;
        *right_target_mm_s = 0.0F;
        return;
    }

    if (chassis_motion_activation_pending != 0U)
    {
        /* Activation snapshots the latest pose before execution timing
         * starts. */
        chassis_motion_activate_command(odometry, yaw_deg);
    }

    chassis_motion_update_turn_progress(yaw_deg);

    if ((chassis_motion_status.phase == CHASSIS_MOTION_PHASE_EXECUTING)
        || ((chassis_motion_status.phase == CHASSIS_MOTION_PHASE_TRANSITION)
            && (chassis_motion_status.command
                != CHASSIS_MOTION_COMMAND_TIMED)))
    {
        if (chassis_motion_status.elapsed_ms
            <= (0xFFFFFFFFU - CHASSIS_MOTION_UPDATE_PERIOD_MS))
        {
            chassis_motion_status.elapsed_ms +=
                CHASSIS_MOTION_UPDATE_PERIOD_MS;
        }

    }

    if (((chassis_motion_status.phase == CHASSIS_MOTION_PHASE_TRANSITION)
            || (chassis_motion_status.phase
                == CHASSIS_MOTION_PHASE_EXECUTING))
        && (chassis_motion_command_is_complete() != 0U))
    {
        /* Command completion enters the same controlled-stop path. */
        chassis_motion_start_stop(CHASSIS_MOTION_STOP_REASON_COMPLETE);
    }

    if ((chassis_motion_status.phase == CHASSIS_MOTION_PHASE_TRANSITION)
        || (chassis_motion_status.phase == CHASSIS_MOTION_PHASE_EXECUTING)
        || (chassis_motion_status.phase
            == CHASSIS_MOTION_PHASE_DECELERATING))
    {
        chassis_motion_generate_targets(
            yaw_deg,
            &left_target,
            &right_target);

        if ((chassis_motion_status.phase
                == CHASSIS_MOTION_PHASE_TRANSITION)
            && (chassis_motion_speed_transition.active == 0U))
        {
            /* Ramp-up completion advances normal command execution. */
            chassis_motion_status.phase = CHASSIS_MOTION_PHASE_EXECUTING;
        }
        else if ((chassis_motion_status.phase
                == CHASSIS_MOTION_PHASE_DECELERATING)
            && (chassis_motion_speed_transition.active == 0U))
        {
            /* Ramp-down completion begins encoder-based stop confirmation. */
            chassis_motion_status.phase = CHASSIS_MOTION_PHASE_WAIT_STOPPED;
        }
    }

    if (chassis_motion_status.phase == CHASSIS_MOTION_PHASE_WAIT_STOPPED)
    {
        /* Hold zero until both encoder deltas remain below the threshold. */
        left_target = 0.0F;
        right_target = 0.0F;
        chassis_motion_update_stop_confirmation(left_count, right_count);
    }

    chassis_motion_status.left_target_mm_s = left_target;
    chassis_motion_status.right_target_mm_s = right_target;
    *left_target_mm_s = left_target;
    *right_target_mm_s = right_target;
}

/**
 * @brief Return whether a command or controlled stop is still active.
 * @return Nonzero while the module owns chassis wheel targets.
 */
uint8 chassis_motion_is_busy(void)
{
    uint8 active;
    uint32 primask;

    primask = interrupt_global_disable();
    active = chassis_motion_status.active;
    interrupt_global_enable(primask);

    return active;
}

/**
 * @brief Copy the latest chassis motion status.
 * @param status Destination status structure.
 */
void chassis_motion_get_status(chassis_motion_status_struct *status)
{
    uint32 primask;

    if (status == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *status = chassis_motion_status;
    interrupt_global_enable(primask);
}
