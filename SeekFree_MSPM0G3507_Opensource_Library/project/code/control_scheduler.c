/**
 * @file    control_scheduler.c
 * @brief   Unified bare-metal 10 ms vehicle control scheduler.
 */

#include "control_scheduler.h"

#include <float.h>
#include <math.h>

#include "drive_geometry.h"
#include "imu_uart.h"
#include "my_lib_encoder.h"
#include "zf_common_interrupt.h"
#include "zf_device_key.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"

#define CONTROL_SCHEDULER_PIT               (PIT_TIM_G12)
#define CONTROL_EMERGENCY_KEY_PIN            (A31)
#define CONTROL_ENCODER_COUNT_LIMIT          (2500)
#define CONTROL_STOPPED_COUNT_LIMIT          (1)
#define CONTROL_IMU_FRESH_LIMIT_TICKS        (10U)
#define CONTROL_SCHEDULER_IRQ_PRIORITY       (1U)
#define CONTROL_ENCODER_IRQ_PRIORITY         (2U)
#define CONTROL_GIMBAL_ENABLED                (GIMBAL_CONFIG_INSTALLED)
#define CONTROL_GIMBAL_FEEDFORWARD_DIVIDER   (2U)
#define CONTROL_GIMBAL_MAX_SEQUENCE_LAG      (1U)
#define CONTROL_IMU_WARMUP_TICKS             (200U)
#define CONTROL_IMU_STABLE_TICKS             (200U)
#define CONTROL_IMU_MIN_STABLE_SAMPLES       (25U)
#define CONTROL_IMU_STABLE_ANGLE_STEP_DEG    (0.30F)
#define CONTROL_IMU_ATTITUDE_FILTER_ALPHA    (0.15F)
#define CONTROL_IMU_STATIONARY_FRAMES        (30U)
#define CONTROL_IMU_STATIONARY_ANGLE_STEP_DEG (0.05F)
#define CONTROL_IMU_REFERENCE_ADAPT_ALPHA    (0.002F)
#define CONTROL_IMU_DEADBAND_ENTER_DEG       (0.12F)
#define CONTROL_IMU_DEADBAND_EXIT_DEG        (0.20F)
#define CONTROL_MANUAL_TIMEOUT_TICKS         \
    (CONTROL_SCHEDULER_MANUAL_TIMEOUT_MS \
        / CONTROL_SCHEDULER_PERIOD_MS)

#define CONTROL_REQUEST_ARM                          (0x0001U)
#define CONTROL_REQUEST_DISARM                       (0x0002U)
#define CONTROL_REQUEST_LINE_START                   (0x0004U)
#define CONTROL_REQUEST_LINE_STOP                    (0x0008U)
#define CONTROL_REQUEST_FAULT_CLEAR                  (0x0010U)
#define CONTROL_REQUEST_MANUAL_TARGET                (0x0020U)
#define CONTROL_REQUEST_CHASSIS_COMMAND              (0x0040U)
#define CONTROL_REQUEST_CHASSIS_CANCEL               (0x0080U)
#define CONTROL_REQUEST_CHASSIS_PID_PROFILE          (0x0100U)

/** @brief Deferred high-level chassis command encoded in the mailbox. */
typedef enum
{
    /** No chassis command is pending. */
    CONTROL_CHASSIS_REQUEST_NONE = 0,
    /** Signed distance command with a positive speed limit. */
    CONTROL_CHASSIS_REQUEST_DISTANCE,
    /** Signed speed command with a finite duration. */
    CONTROL_CHASSIS_REQUEST_TIMED,
    /** Relative heading command with a positive angular-speed limit. */
    CONTROL_CHASSIS_REQUEST_TURN,
} control_chassis_request_enum;

/** @brief Foreground-to-scheduler request payload consumed once per tick. */
typedef struct
{
    /** OR of CONTROL_REQUEST_* bits identifying pending fields. */
    uint16 flags;
    /** Manual left-wheel target associated with its request bit. */
    float left_target_mm_s;
    /** Manual right-wheel target associated with its request bit. */
    float right_target_mm_s;
    /** Distance or relative angle selected by chassis_command. */
    float chassis_value;
    /** Linear or angular speed selected by chassis_command. */
    float chassis_speed;
    /** Duration used only by a timed chassis command. */
    uint32 chassis_duration_ms;
    /** Interpretation of the chassis command payload. */
    control_chassis_request_enum chassis_command;
    /** PID profile selected by the profile request bit. */
    uint8 chassis_profile_id;
} control_request_mailbox_struct;

static volatile control_scheduler_status_struct control_status;
static volatile control_request_mailbox_struct control_mailbox;
static volatile uint8 control_initialized;
static volatile uint8 control_started;
static volatile uint8 control_update_busy;

static uint32 control_last_imu_frame_count;
static uint16 control_manual_target_age_ticks;
static uint8 control_manual_target_active;
static uint8 control_key4_long_handled;
static uint8 control_gimbal_calibration_complete;
static uint8 control_gimbal_feedforward_divider;
static volatile gimbal_feedforward_pose_struct control_gimbal_pose_mailbox;
static volatile uint32 control_gimbal_pose_sequence;
static volatile uint8 control_gimbal_pose_valid;
static uint32 control_gimbal_pose_consumed_sequence;
static volatile gimbal_feedforward_solution_struct
    control_gimbal_last_solution;
static uint16 control_imu_warmup_ticks;
static uint16 control_imu_stable_ticks;
static uint16 control_imu_stable_samples;
static uint8 control_imu_unwrap_valid;
static uint8 control_imu_previous_sample_valid;
static float control_imu_last_raw_yaw_deg;
static float control_imu_unwrapped_yaw_deg;
static float control_imu_previous_roll_deg;
static float control_imu_previous_pitch_deg;
static float control_imu_previous_yaw_deg;
static float control_imu_sum_roll_deg;
static float control_imu_sum_pitch_deg;
static float control_imu_sum_time;
static float control_imu_sum_time_squared;
static float control_imu_sum_yaw_deg;
static float control_imu_sum_time_yaw;
static float control_imu_reference_roll_deg;
static float control_imu_reference_pitch_deg;
static float control_imu_reference_yaw_deg;
static float control_imu_yaw_drift_deg_per_tick;
static float control_imu_filtered_roll_deg;
static float control_imu_filtered_pitch_deg;
static uint32 control_imu_reference_tick;
static uint16 control_imu_stationary_frames;
static uint8 control_imu_roll_deadband_active;
static uint8 control_imu_pitch_deadband_active;
static uint8 control_imu_latest_angle_stationary;

static uint16 control_count_magnitude(int16 count);

/**
 * @brief Wrap one degree angle to the interval [-180, 180].
 */
static float control_wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0F)
    {
        angle_deg -= 360.0F;
    }
    while (angle_deg < -180.0F)
    {
        angle_deg += 360.0F;
    }
    return angle_deg;
}

/**
 * @brief Apply a hysteretic zero deadband to one corrected attitude axis.
 */
static float control_imu_apply_deadband(
    float angle_deg,
    uint8 *deadband_active)
{
    float magnitude = fabsf(angle_deg);

    if (*deadband_active != 0U)
    {
        if (magnitude < CONTROL_IMU_DEADBAND_EXIT_DEG)
        {
            return 0.0F;
        }
        *deadband_active = 0U;
    }
    else if (magnitude <= CONTROL_IMU_DEADBAND_ENTER_DEG)
    {
        *deadband_active = 1U;
        return 0.0F;
    }

    return angle_deg;
}

/**
 * @brief Clear one interrupted stationary-sample window.
 */
static void control_imu_clear_stability_window(void)
{
    control_imu_warmup_ticks = 0U;
    control_imu_stable_ticks = 0U;
    control_imu_stable_samples = 0U;
    control_imu_previous_sample_valid = 0U;
    control_imu_sum_roll_deg = 0.0F;
    control_imu_sum_pitch_deg = 0.0F;
    control_imu_sum_time = 0.0F;
    control_imu_sum_time_squared = 0.0F;
    control_imu_sum_yaw_deg = 0.0F;
    control_imu_sum_time_yaw = 0.0F;
}

/**
 * @brief Restart automatic startup attitude referencing.
 */
static void control_imu_begin_stability(void)
{
    control_imu_clear_stability_window();
    control_imu_unwrap_valid = 0U;
    control_imu_reference_roll_deg = 0.0F;
    control_imu_reference_pitch_deg = 0.0F;
    control_imu_reference_yaw_deg = 0.0F;
    control_imu_yaw_drift_deg_per_tick = 0.0F;
    control_imu_filtered_roll_deg = 0.0F;
    control_imu_filtered_pitch_deg = 0.0F;
    control_imu_stationary_frames = 0U;
    control_imu_roll_deadband_active = 1U;
    control_imu_pitch_deadband_active = 1U;
    control_imu_latest_angle_stationary = 0U;
    control_imu_reference_tick = control_status.tick_count;
    control_status.imu_ready = 0U;
    control_status.imu_stability_progress = 0U;
    control_status.imu_stability_state =
        CONTROL_IMU_STABILITY_WAIT_STREAM;
    control_status.imu_yaw_drift_deg_min = 0.0F;
}

/**
 * @brief Install the completed startup reference and measured yaw drift.
 */
static void control_imu_finish_stability(void)
{
    float sample_count = (float)control_imu_stable_samples;
    float denominator = (sample_count
            * control_imu_sum_time_squared)
        - (control_imu_sum_time * control_imu_sum_time);

    control_imu_reference_roll_deg =
        control_imu_sum_roll_deg / sample_count;
    control_imu_reference_pitch_deg =
        control_imu_sum_pitch_deg / sample_count;
    control_imu_reference_yaw_deg = control_imu_unwrapped_yaw_deg;
    control_imu_yaw_drift_deg_per_tick = 0.0F;
    if (fabsf(denominator) > 0.001F)
    {
        control_imu_yaw_drift_deg_per_tick =
            ((sample_count * control_imu_sum_time_yaw)
                - (control_imu_sum_time
                    * control_imu_sum_yaw_deg))
            / denominator;
    }
    control_imu_reference_tick = control_status.tick_count;
    control_imu_filtered_roll_deg = 0.0F;
    control_imu_filtered_pitch_deg = 0.0F;
    control_imu_stationary_frames = 0U;
    control_imu_latest_angle_stationary = 1U;
    control_status.imu_yaw_drift_deg_min =
        control_imu_yaw_drift_deg_per_tick * 6000.0F;
    control_status.imu_ready = 1U;
    control_status.imu_stability_progress = 100U;
    control_status.imu_stability_state = CONTROL_IMU_STABILITY_READY;
    my_encoder_clear_count();
    odometry_reset_pose(0.0F, 0.0F, 0.0F);
}

/**
 * @brief Update startup stability and publish corrected body attitude.
 */
static void control_update_imu_stability(
    const imu_uart_data_struct *imu_data,
    uint8 new_angle_frame)
{
    float raw_roll_deg = -imu_data->angle_deg[1];
    float raw_pitch_deg = -imu_data->angle_deg[0];
    float raw_yaw_deg = -imu_data->angle_deg[2];
    uint8 sample_stable = 1U;

    if (new_angle_frame != 0U)
    {
        if (control_imu_unwrap_valid == 0U)
        {
            control_imu_last_raw_yaw_deg = raw_yaw_deg;
            control_imu_unwrapped_yaw_deg = raw_yaw_deg;
            control_imu_unwrap_valid = 1U;
        }
        else
        {
            control_imu_unwrapped_yaw_deg += control_wrap_angle_deg(
                raw_yaw_deg - control_imu_last_raw_yaw_deg);
            control_imu_last_raw_yaw_deg = raw_yaw_deg;
        }

        if (control_imu_previous_sample_valid != 0U)
        {
            float roll_step_deg = fabsf(control_wrap_angle_deg(
                raw_roll_deg - control_imu_previous_roll_deg));
            float pitch_step_deg = fabsf(control_wrap_angle_deg(
                raw_pitch_deg - control_imu_previous_pitch_deg));
            float yaw_step_deg = fabsf(control_wrap_angle_deg(
                raw_yaw_deg - control_imu_previous_yaw_deg));

            control_imu_latest_angle_stationary = (uint8)(
                (roll_step_deg <= CONTROL_IMU_STATIONARY_ANGLE_STEP_DEG)
                && (pitch_step_deg
                    <= CONTROL_IMU_STATIONARY_ANGLE_STEP_DEG)
                && (yaw_step_deg
                    <= CONTROL_IMU_STATIONARY_ANGLE_STEP_DEG));
            if ((roll_step_deg > CONTROL_IMU_STABLE_ANGLE_STEP_DEG)
                || (pitch_step_deg > CONTROL_IMU_STABLE_ANGLE_STEP_DEG)
                || (yaw_step_deg > CONTROL_IMU_STABLE_ANGLE_STEP_DEG))
            {
                sample_stable = 0U;
            }
        }
        else
        {
            control_imu_latest_angle_stationary = 0U;
        }
        control_imu_previous_roll_deg = raw_roll_deg;
        control_imu_previous_pitch_deg = raw_pitch_deg;
        control_imu_previous_yaw_deg = raw_yaw_deg;
        control_imu_previous_sample_valid = 1U;
    }

    if ((control_status.imu_ready != 0U)
        && (control_status.imu_fresh == 0U))
    {
        control_imu_begin_stability();
        control_status.imu_roll_deg = raw_roll_deg;
        control_status.imu_pitch_deg = raw_pitch_deg;
        control_status.imu_yaw_deg = raw_yaw_deg;
        return;
    }

    if (control_status.imu_ready != 0U)
    {
        float corrected_roll_deg = control_wrap_angle_deg(
            raw_roll_deg - control_imu_reference_roll_deg);
        float corrected_pitch_deg = control_wrap_angle_deg(
            raw_pitch_deg - control_imu_reference_pitch_deg);
        float corrected_yaw_deg = control_imu_unwrapped_yaw_deg
            - control_imu_reference_yaw_deg
            - (control_imu_yaw_drift_deg_per_tick
                * (float)(control_status.tick_count
                    - control_imu_reference_tick));
        float corrected_yaw_wrapped_deg = control_wrap_angle_deg(
            corrected_yaw_deg);
        uint8 stationary_mode = (uint8)(
            (control_status.mode == CONTROL_MODE_DISARMED)
            || (control_status.mode == CONTROL_MODE_MANUAL_ARMED));
        if (stationary_mode == 0U)
        {
            control_imu_stationary_frames = 0U;
        }
        else if (new_angle_frame != 0U)
        {
            if (control_imu_latest_angle_stationary != 0U)
            {
                if (control_imu_stationary_frames
                    < CONTROL_IMU_STATIONARY_FRAMES)
                {
                    control_imu_stationary_frames++;
                }
            }
            else
            {
                control_imu_stationary_frames = 0U;
            }
        }

        if (control_imu_stationary_frames
            >= CONTROL_IMU_STATIONARY_FRAMES)
        {
            if (new_angle_frame != 0U)
            {
                control_imu_reference_roll_deg = control_wrap_angle_deg(
                    control_imu_reference_roll_deg
                    + (CONTROL_IMU_REFERENCE_ADAPT_ALPHA
                        * corrected_roll_deg));
                control_imu_reference_pitch_deg = control_wrap_angle_deg(
                    control_imu_reference_pitch_deg
                    + (CONTROL_IMU_REFERENCE_ADAPT_ALPHA
                        * corrected_pitch_deg));
                control_imu_reference_yaw_deg +=
                    CONTROL_IMU_REFERENCE_ADAPT_ALPHA
                    * corrected_yaw_wrapped_deg;
            }
            control_imu_filtered_roll_deg = 0.0F;
            control_imu_filtered_pitch_deg = 0.0F;
            control_imu_roll_deadband_active = 1U;
            control_imu_pitch_deadband_active = 1U;
            control_status.imu_roll_deg = 0.0F;
            control_status.imu_pitch_deg = 0.0F;
            control_status.imu_yaw_deg = 0.0F;
            return;
        }

        if (new_angle_frame != 0U)
        {
            corrected_roll_deg = control_imu_apply_deadband(
                corrected_roll_deg,
                &control_imu_roll_deadband_active);
            corrected_pitch_deg = control_imu_apply_deadband(
                corrected_pitch_deg,
                &control_imu_pitch_deadband_active);
            if (control_imu_roll_deadband_active != 0U)
            {
                control_imu_filtered_roll_deg = 0.0F;
            }
            else
            {
                control_imu_filtered_roll_deg +=
                    CONTROL_IMU_ATTITUDE_FILTER_ALPHA
                    * (corrected_roll_deg
                        - control_imu_filtered_roll_deg);
            }
            if (control_imu_pitch_deadband_active != 0U)
            {
                control_imu_filtered_pitch_deg = 0.0F;
            }
            else
            {
                control_imu_filtered_pitch_deg +=
                    CONTROL_IMU_ATTITUDE_FILTER_ALPHA
                    * (corrected_pitch_deg
                        - control_imu_filtered_pitch_deg);
            }
        }
        control_status.imu_roll_deg = control_imu_filtered_roll_deg;
        control_status.imu_pitch_deg = control_imu_filtered_pitch_deg;
        control_status.imu_yaw_deg = corrected_yaw_wrapped_deg;
        return;
    }

    control_status.imu_roll_deg = raw_roll_deg;
    control_status.imu_pitch_deg = raw_pitch_deg;
    control_status.imu_yaw_deg = raw_yaw_deg;
    if ((control_gimbal_calibration_complete == 0U)
        || (control_status.imu_fresh == 0U))
    {
        control_status.imu_stability_state =
            CONTROL_IMU_STABILITY_WAIT_STREAM;
        control_status.imu_stability_progress = 0U;
        control_imu_clear_stability_window();
        return;
    }
    if (sample_stable == 0U)
    {
        control_status.imu_stability_state =
            CONTROL_IMU_STABILITY_WARMUP;
        control_status.imu_stability_progress = 0U;
        control_imu_clear_stability_window();
        return;
    }

    if (control_status.imu_stability_state
        == CONTROL_IMU_STABILITY_WAIT_STREAM)
    {
        control_status.imu_stability_state =
            CONTROL_IMU_STABILITY_WARMUP;
    }
    if (control_status.imu_stability_state
        == CONTROL_IMU_STABILITY_WARMUP)
    {
        if (control_imu_warmup_ticks < CONTROL_IMU_WARMUP_TICKS)
        {
            control_imu_warmup_ticks++;
        }
        control_status.imu_stability_progress = (uint8)(
            ((uint32)control_imu_warmup_ticks * 50U)
            / CONTROL_IMU_WARMUP_TICKS);
        if (control_imu_warmup_ticks >= CONTROL_IMU_WARMUP_TICKS)
        {
            control_status.imu_stability_state =
                CONTROL_IMU_STABILITY_COLLECTING;
            control_imu_stable_ticks = 0U;
            control_imu_stable_samples = 0U;
        }
        return;
    }

    if (control_imu_stable_ticks < CONTROL_IMU_STABLE_TICKS)
    {
        control_imu_stable_ticks++;
    }
    control_status.imu_stability_progress = (uint8)(50U
        + (((uint32)control_imu_stable_ticks * 50U)
            / CONTROL_IMU_STABLE_TICKS));
    if ((new_angle_frame != 0U)
        && (control_imu_stable_samples < 0xFFFFU))
    {
        float time = (float)control_imu_stable_ticks;

        control_imu_stable_samples++;
        control_imu_sum_roll_deg += raw_roll_deg;
        control_imu_sum_pitch_deg += raw_pitch_deg;
        control_imu_sum_time += time;
        control_imu_sum_time_squared += time * time;
        control_imu_sum_yaw_deg += control_imu_unwrapped_yaw_deg;
        control_imu_sum_time_yaw +=
            time * control_imu_unwrapped_yaw_deg;
    }
    if ((control_imu_stable_ticks >= CONTROL_IMU_STABLE_TICKS)
        && (control_imu_stable_samples
            >= CONTROL_IMU_MIN_STABLE_SAMPLES))
    {
        control_imu_finish_stability();
        control_status.imu_roll_deg = 0.0F;
        control_status.imu_pitch_deg = 0.0F;
        control_status.imu_yaw_deg = 0.0F;
    }
}

/**
 * @brief Check that all shared non-emergency keys are physically released.
 */
static uint8 control_gimbal_keys_released(void)
{
    return (uint8)(
        (gpio_get_level(A30) == GPIO_HIGH)
        && (gpio_get_level(B0) == GPIO_HIGH)
        && (gpio_get_level(B1) == GPIO_HIGH));
}

/**
 * @brief Transfer shared-key ownership after both gimbal zeros are valid.
 */
static void control_update_gimbal_calibration(
    const gimbal_stepper_status_struct *gimbal_status)
{
    if ((CONTROL_GIMBAL_ENABLED == 0U)
        || (control_gimbal_calibration_complete != 0U)
        || (gimbal_status == NULL))
    {
        return;
    }

    if ((gimbal_status->relative_ready != 0U)
        && (control_gimbal_keys_released() != 0U))
    {
        gimbal_stepper_set_manual_control_enabled(0U);
        key_clear_all_state();
        my_encoder_clear_count();
        odometry_reset_pose(0.0F, 0.0F, 0.0F);
        line_tracker_reset();
        speed_pid_reset();
        control_gimbal_pose_valid = 0U;
        control_gimbal_feedforward_divider = 0U;
        control_gimbal_calibration_complete = 1U;
        control_imu_begin_stability();
    }
}

/**
 * @brief Clear the stored manual command and its timeout state.
 */
static void control_clear_manual_target(void)
{
    control_manual_target_active = 0U;
    control_manual_target_age_ticks = 0U;
    control_status.manual_left_target_mm_s = 0.0F;
    control_status.manual_right_target_mm_s = 0.0F;
}

/**
 * @brief Return the absolute magnitude of a signed encoder count.
 * @param count Signed count.
 * @return Unsigned magnitude.
 */
static uint16 control_count_magnitude(int16 count)
{
    int32 magnitude = count;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }

    return (uint16)magnitude;
}

/**
 * @brief Check that a target is finite and in speed PID range.
 * @param target_mm_s Target speed.
 * @return Nonzero when valid.
 */
static uint8 control_target_is_valid(float target_mm_s)
{
    return (uint8)((target_mm_s == target_mm_s)
        && (target_mm_s >= -SPEED_PID_TARGET_LIMIT_MM_S)
        && (target_mm_s <= SPEED_PID_TARGET_LIMIT_MM_S)
        && (target_mm_s >= -FLT_MAX)
        && (target_mm_s <= FLT_MAX));
}

/**
 * @brief Check that one floating-point control request value is finite.
 * @param value Request value.
 * @return Nonzero when valid.
 */
static uint8 control_value_is_valid(float value)
{
    return (uint8)((value == value)
        && (value >= -FLT_MAX)
        && (value <= FLT_MAX));
}

/**
 * @brief Validate one bounded relative-turn request.
 * @param angle_deg Requested relative angle in degrees.
 * @param angular_speed_deg_s Positive angular-speed limit in degrees per second.
 * @return Nonzero when the request can map to valid wheel targets.
 */
static uint8 control_turn_request_is_valid(
    float angle_deg,
    float angular_speed_deg_s)
{
    float wheel_speed_mm_s;

    if ((control_value_is_valid(angle_deg) == 0U)
        || (control_value_is_valid(angular_speed_deg_s) == 0U)
        || (angle_deg == 0.0F)
        || (angle_deg < -CHASSIS_MOTION_TURN_MAX_ANGLE_DEG)
        || (angle_deg > CHASSIS_MOTION_TURN_MAX_ANGLE_DEG)
        || (angular_speed_deg_s <= 0.0F))
    {
        return 0U;
    }

    wheel_speed_mm_s = angular_speed_deg_s
        * (DRIVE_PI / 180.0F)
        * (DRIVE_TRACK_WIDTH_MM * 0.5F);
    return (uint8)(wheel_speed_mm_s <= SPEED_PID_TARGET_LIMIT_MM_S);
}

/**
 * @brief Force zero output and latch one or more fault flags.
 * @param fault_flags Fault bits to latch.
 */
static void control_latch_fault(uint32 fault_flags)
{
    control_status.fault_flags |= fault_flags;
    control_status.mode = CONTROL_MODE_FAULT_LATCHED;
    control_clear_manual_target();
    chassis_motion_reset();
    speed_pid_stop();
    (void)gimbal_stepper_set_laser(0U);
}

/**
 * @brief Enter the safe disarmed state and clear motion targets.
 */
static void control_enter_disarmed(void)
{
    control_status.mode = CONTROL_MODE_DISARMED;
    control_clear_manual_target();
    chassis_motion_reset();
    line_tracker_reset();
    speed_pid_stop();
    (void)gimbal_stepper_set_laser(0U);
}

/**
 * @brief Return whether both sampled wheels are effectively stopped.
 * @return Nonzero when both counts are inside the stopped threshold.
 */
static uint8 control_wheels_are_stopped(void)
{
    return (uint8)((control_count_magnitude(control_status.left_count)
            <= CONTROL_STOPPED_COUNT_LIMIT)
        && (control_count_magnitude(control_status.right_count)
            <= CONTROL_STOPPED_COUNT_LIMIT));
}

/**
 * @brief Clear a latched fault after release and stopped-wheel checks.
 * @param emergency_active Nonzero when the raw emergency key is pressed.
 */
static void control_try_clear_fault(uint8 emergency_active)
{
    if ((control_status.mode != CONTROL_MODE_FAULT_LATCHED)
        || (emergency_active != 0U)
        || (control_wheels_are_stopped() == 0U))
    {
        return;
    }

    control_status.fault_flags = CONTROL_FAULT_NONE;
    my_encoder_clear_count();
    odometry_reset();
    control_imu_begin_stability();
    control_enter_disarmed();
}

/**
 * @brief Consume all foreground requests into a local immutable snapshot.
 * @return Request snapshot.
 */
static control_request_mailbox_struct control_take_requests(void)
{
    control_request_mailbox_struct requests;

    requests.flags = control_mailbox.flags;
    requests.left_target_mm_s = control_mailbox.left_target_mm_s;
    requests.right_target_mm_s = control_mailbox.right_target_mm_s;
    requests.chassis_value = control_mailbox.chassis_value;
    requests.chassis_speed = control_mailbox.chassis_speed;
    requests.chassis_duration_ms = control_mailbox.chassis_duration_ms;
    requests.chassis_command = control_mailbox.chassis_command;
    requests.chassis_profile_id = control_mailbox.chassis_profile_id;
    control_mailbox.flags = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_NONE;

    return requests;
}

/**
 * @brief Merge physical key events into one scheduler request snapshot.
 * @param requests Request snapshot to extend.
 */
static void control_add_key_requests(
    control_request_mailbox_struct *requests)
{
    key_state_enum key1_state = key_get_state(KEY_1);
    key_state_enum key3_state = key_get_state(KEY_3);
    key_state_enum key4_state = key_get_state(KEY_4);

    if (key1_state == KEY_SHORT_PRESS)
    {
        if ((control_status.mode == CONTROL_MODE_MANUAL_ARMED)
            || (control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
            || (control_status.mode == CONTROL_MODE_LINE_FOLLOW))
        {
            requests->flags |= CONTROL_REQUEST_DISARM;
        }
        else if (control_status.mode == CONTROL_MODE_DISARMED)
        {
            requests->flags |= CONTROL_REQUEST_ARM;
        }
        key_clear_state(KEY_1);
    }

    if (key3_state == KEY_SHORT_PRESS)
    {
        if (control_status.mode == CONTROL_MODE_LINE_FOLLOW)
        {
            requests->flags |= CONTROL_REQUEST_LINE_STOP;
        }
        else if (control_status.mode == CONTROL_MODE_MANUAL_ARMED)
        {
            requests->flags |= CONTROL_REQUEST_LINE_START;
        }
        key_clear_state(KEY_3);
    }

    if (key4_state == KEY_LONG_PRESS)
    {
        if (control_key4_long_handled == 0U)
        {
            requests->flags |= CONTROL_REQUEST_FAULT_CLEAR;
            control_key4_long_handled = 1U;
        }
    }
    else
    {
        control_key4_long_handled = 0U;
    }
}

/**
 * @brief Apply requests according to scheduler mode and safety priority.
 * @param requests Coherent request snapshot.
 * @param emergency_active Nonzero when the raw emergency key is pressed.
 */
static void control_apply_requests(
    const control_request_mailbox_struct *requests,
    uint8 emergency_active)
{
    uint8 chassis_started = 0U;

    if ((requests->flags & CONTROL_REQUEST_FAULT_CLEAR) != 0U)
    {
        uint8 was_faulted =
            (uint8)(control_status.mode == CONTROL_MODE_FAULT_LATCHED);

        control_try_clear_fault(emergency_active);
        if (was_faulted != 0U)
        {
            return;
        }
    }

    if (control_status.mode == CONTROL_MODE_FAULT_LATCHED)
    {
        return;
    }

    if ((requests->flags & CONTROL_REQUEST_DISARM) != 0U)
    {
        control_enter_disarmed();
        return;
    }

    if ((requests->flags & CONTROL_REQUEST_ARM) != 0U)
    {
        if ((control_status.mode == CONTROL_MODE_DISARMED)
            && (control_status.imu_ready != 0U)
            && (control_status.imu_fresh != 0U))
        {
            control_status.mode = CONTROL_MODE_MANUAL_ARMED;
            speed_pid_reset();
        }
    }

    if ((requests->flags & CONTROL_REQUEST_LINE_STOP) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_LINE_FOLLOW)
        {
            line_tracker_reset();
            speed_pid_reset();
            control_clear_manual_target();
            control_status.mode = CONTROL_MODE_MANUAL_ARMED;
        }
    }

    if ((requests->flags & CONTROL_REQUEST_LINE_START) != 0U)
    {
        if ((control_status.mode == CONTROL_MODE_MANUAL_ARMED)
            && (control_status.gray.status == GRAY_SENSOR_STATUS_VALID)
            && (control_status.imu_ready != 0U)
            && (control_status.imu_fresh != 0U)
            && (control_wheels_are_stopped() != 0U))
        {
            control_clear_manual_target();
            line_tracker_reset();
            control_status.mode = CONTROL_MODE_LINE_FOLLOW;
        }
    }

    if ((requests->flags & CONTROL_REQUEST_MANUAL_TARGET) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_MANUAL_ARMED)
        {
            control_status.manual_left_target_mm_s =
                requests->left_target_mm_s;
            control_status.manual_right_target_mm_s =
                requests->right_target_mm_s;
            control_manual_target_active =
                (uint8)((requests->left_target_mm_s != 0.0F)
                    || (requests->right_target_mm_s != 0.0F));
            control_manual_target_age_ticks = 0U;
        }
    }

    if ((requests->flags & CONTROL_REQUEST_CHASSIS_PID_PROFILE) != 0U)
    {
        (void)chassis_motion_pid_profile_select(
            requests->chassis_profile_id);
    }

    if ((requests->flags & CONTROL_REQUEST_CHASSIS_CANCEL) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
        {
            chassis_motion_cancel();
        }
    }

    if ((control_status.mode == CONTROL_MODE_MANUAL_ARMED)
        || (control_status.mode == CONTROL_MODE_CHASSIS_MOTION))
    {
        if ((requests->flags & CONTROL_REQUEST_CHASSIS_COMMAND) != 0U)
        {
            if ((control_status.imu_fresh == 0U)
                || (control_status.imu_ready == 0U))
            {
                control_latch_fault(CONTROL_FAULT_IMU_STALE);
            }
            else if (requests->chassis_command
                == CONTROL_CHASSIS_REQUEST_DISTANCE)
            {
                chassis_started = chassis_motion_start_distance(
                    requests->chassis_value,
                    requests->chassis_speed);
            }
            else if (requests->chassis_command
                == CONTROL_CHASSIS_REQUEST_TIMED)
            {
                chassis_started = chassis_motion_start_timed(
                    requests->chassis_speed,
                    requests->chassis_duration_ms);
            }
            else if (requests->chassis_command
                == CONTROL_CHASSIS_REQUEST_TURN)
            {
                chassis_started = chassis_motion_start_turn_relative(
                    requests->chassis_value,
                    requests->chassis_speed);
            }
        }

        if (chassis_started != 0U)
        {
            control_clear_manual_target();
            control_status.mode = CONTROL_MODE_CHASSIS_MOTION;
        }
    }
}

/**
 * @brief Update manual command timeout and disarm stale motion targets.
 */
static void control_update_manual_timeout(void)
{
    if ((control_status.mode != CONTROL_MODE_MANUAL_ARMED)
        || (control_manual_target_active == 0U))
    {
        return;
    }

    if (control_manual_target_age_ticks < 0xFFFFU)
    {
        control_manual_target_age_ticks++;
    }

    if (control_manual_target_age_ticks >= CONTROL_MANUAL_TIMEOUT_TICKS)
    {
        control_enter_disarmed();
    }
}

/**
 * @brief Select one target pair from the current scheduler mode.
 * @param odometry Latest odometry state.
 * @param yaw_deg Latest IMU yaw in degrees.
 * @param left_count Latest signed left encoder delta.
 * @param right_count Latest signed right encoder delta.
 * @param left_target Destination left target.
 * @param right_target Destination right target.
 */
static void control_select_targets(
    const gray_sensor_result_struct *gray,
    const odometry_state_struct *odometry,
    float yaw_deg,
    int16 left_count,
    int16 right_count,
    line_tracker_output_struct *line_output,
    float *left_target,
    float *right_target)
{
    *left_target = 0.0F;
    *right_target = 0.0F;

    if (control_status.mode == CONTROL_MODE_MANUAL_ARMED)
    {
        *left_target = control_status.manual_left_target_mm_s;
        *right_target = control_status.manual_right_target_mm_s;
    }
    else if (control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
    {
        chassis_motion_update_10ms(
            odometry,
            yaw_deg,
            left_count,
            right_count,
            left_target,
            right_target);
        if (chassis_motion_is_busy() == 0U)
        {
            control_status.mode = CONTROL_MODE_MANUAL_ARMED;
        }
    }
    else if (control_status.mode == CONTROL_MODE_LINE_FOLLOW)
    {
        if (line_tracker_update(
                gray,
                line_output) == ZF_FALSE)
        {
            control_latch_fault(CONTROL_FAULT_LINE_TRACKER);
            return;
        }

        *left_target = line_output->left_target_mm_s;
        *right_target = line_output->right_target_mm_s;
    }
}

/**
 * @brief Publish all module status snapshots after one completed tick.
 */
static void control_publish_status(void)
{
    line_tracker_status_struct line_status;
    chassis_motion_status_struct chassis_status;
    speed_pid_status_struct speed_status;
    odometry_state_struct odometry_status;
    gimbal_stepper_status_struct gimbal_status;
    gimbal_feedforward_solution_struct gimbal_solution;

    line_tracker_get_status(&line_status);
    chassis_motion_get_status(&chassis_status);
    speed_pid_get_status(&speed_status);
    odometry_get_state(&odometry_status);
    gimbal_stepper_get_status(&gimbal_status);
    gimbal_solution = control_gimbal_last_solution;
    control_status.line_status = line_status;
    control_status.chassis_motion = chassis_status;
    control_status.speed = speed_status;
    control_status.odometry = odometry_status;
    control_status.gimbal = gimbal_status;
    control_status.gimbal_feedforward = gimbal_solution;
    control_status.gimbal_calibrated =
        control_gimbal_calibration_complete;
    control_status.initialized = control_initialized;
    control_status.started = control_started;
}

/**
 * @brief PIT callback that owns the fixed-period control pipeline.
 * @param event PIT callback event value.
 * @param user_data Optional callback context.
 */
static void control_pit_callback(uint32 event, void *user_data)
{
    (void)event;
    (void)user_data;

    control_scheduler_update_10ms();
}

/**
 * @brief Initialize every control dependency in a safe stopped state.
 * @return ZF_TRUE when mandatory hardware initialized successfully.
 */
uint8 control_scheduler_init(void)
{
    uint8 gray_ready;

    control_initialized = 0U;
    control_started = 0U;
    control_update_busy = 0U;
    control_last_imu_frame_count = 0U;
    control_manual_target_age_ticks = 0U;
    control_manual_target_active = 0U;
    control_key4_long_handled = 0U;
    control_gimbal_calibration_complete =
        CONTROL_GIMBAL_ENABLED == 0U ? 1U : 0U;
    control_gimbal_pose_sequence = 0U;
    control_gimbal_pose_valid = 0U;
    control_gimbal_pose_consumed_sequence = 0U;
    control_gimbal_feedforward_divider = 0U;
    control_gimbal_last_solution.target_x_mm =
        GIMBAL_CONFIG_TARGET_CENTER_X_MM;
    control_gimbal_last_solution.target_y_mm =
        GIMBAL_CONFIG_TARGET_CENTER_Y_MM;
    control_gimbal_last_solution.target_z_mm =
        GIMBAL_CONFIG_TARGET_CENTER_Z_MM;
    control_gimbal_last_solution.yaw_deg = 0.0F;
    control_gimbal_last_solution.pitch_deg = 0.0F;
    control_gimbal_last_solution.residual_deg = 0.0F;
    control_gimbal_last_solution.valid = 0U;
    control_gimbal_last_solution.singular = 0U;

    control_status.mode = CONTROL_MODE_BOOT;
    control_status.fault_flags = CONTROL_FAULT_NONE;
    control_status.tick_count = 0U;
    control_status.overrun_count = 0U;
    control_status.imu_angle_frame_count = 0U;
    control_status.imu_age_ticks = 0U;
    control_status.imu_valid = 0U;
    control_status.imu_fresh = 0U;
    control_status.imu_yaw_deg = 0.0F;
    control_status.imu_roll_deg = 0.0F;
    control_status.imu_pitch_deg = 0.0F;
    control_status.imu_yaw_drift_deg_min = 0.0F;
    control_status.gimbal_feedforward_count = 0U;
    control_status.gimbal_feedforward_reject_count = 0U;
    control_status.gimbal_feedforward_stale_count = 0U;
    control_status.gimbal_feedforward_solve_ticks = 0U;
    control_status.gimbal_feedforward_reject_reason =
        CONTROL_GIMBAL_REJECT_NOT_READY;
    control_status.gimbal_calibrated =
        CONTROL_GIMBAL_ENABLED == 0U ? 1U : 0U;
    control_status.gimbal_feedforward_valid = 0U;
    control_imu_begin_stability();
    control_mailbox.flags = 0U;
    control_mailbox.left_target_mm_s = 0.0F;
    control_mailbox.right_target_mm_s = 0.0F;
    control_mailbox.chassis_value = 0.0F;
    control_mailbox.chassis_speed = 0.0F;
    control_mailbox.chassis_duration_ms = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_NONE;
    control_mailbox.chassis_profile_id =
        CHASSIS_MOTION_PID_PROFILE_INVALID;

    interrupt_set_priority(
        TIMG12_INT_IRQn,
        CONTROL_SCHEDULER_IRQ_PRIORITY);
    interrupt_set_priority(
        GPIOA_INT_IRQn,
        CONTROL_ENCODER_IRQ_PRIORITY);

    speed_pid_init();
    chassis_motion_init();
    my_encoder_init();
    gray_ready = gray_sensor_init();
    imu_uart_init();
    odometry_init();
    line_tracker_init(NULL);
    key_init(CONTROL_SCHEDULER_PERIOD_MS);
    if (CONTROL_GIMBAL_ENABLED != 0U)
    {
        gimbal_stepper_init();
        gimbal_stepper_set_manual_control_enabled(1U);
    }
    my_encoder_clear_count();

    control_initialized = 1U;
    control_status.initialized = 1U;
    control_status.started = 0U;
    if (gray_ready == ZF_FALSE)
    {
        control_latch_fault(CONTROL_FAULT_GRAY_INIT);
        control_publish_status();
        return ZF_FALSE;
    }

    control_enter_disarmed();
    control_publish_status();
    return ZF_TRUE;
}

/**
 * @brief Start the unique 10 ms PIT control source.
 * @return ZF_TRUE when the scheduler was started.
 */
uint8 control_scheduler_start(void)
{
    if ((control_initialized == 0U) || (control_started != 0U))
    {
        return ZF_FALSE;
    }

    control_started = 1U;
    control_status.started = 1U;
    pit_ms_init(
        CONTROL_SCHEDULER_PIT,
        CONTROL_SCHEDULER_PERIOD_MS,
        control_pit_callback,
        NULL);

    return ZF_TRUE;
}

/**
 * @brief Execute the complete fixed-period control pipeline once.
 */
void control_scheduler_update_10ms(void)
{
    control_request_mailbox_struct requests;
    gray_sensor_result_struct gray;
    line_tracker_output_struct line_output;
    float left_target = 0.0F;
    float right_target = 0.0F;
    uint32 yaw_frame_count = 0U;
    float yaw_deg = 0.0F;
    odometry_state_struct odometry;
    imu_uart_data_struct imu_data;
    gimbal_stepper_status_struct gimbal_status;
    gimbal_feedforward_pose_struct gimbal_pose;
    int16 left_count;
    int16 right_count;
    int16 odometry_left_count;
    int16 odometry_right_count;
    uint8 emergency_active;
    uint8 encoder_valid;
    uint8 imu_data_valid;
    uint8 yaw_valid;
    uint8 odometry_yaw_valid;
    uint8 gray_valid;
    uint8 new_angle_frame;
    uint8 calibration_complete_at_tick_start;

    if ((control_initialized == 0U) || (control_started == 0U))
    {
        return;
    }

    if (control_update_busy != 0U)
    {
        control_status.overrun_count++;
        control_latch_fault(CONTROL_FAULT_REENTRY);
        return;
    }
    control_update_busy = 1U;
    control_status.tick_count++;

    /* Phase 1: latch safety input and collect one request snapshot. */
    emergency_active = (uint8)(
        gpio_get_level(CONTROL_EMERGENCY_KEY_PIN) == GPIO_LOW);
    if ((emergency_active != 0U)
        && (control_gimbal_calibration_complete != 0U))
    {
        control_latch_fault(CONTROL_FAULT_EMERGENCY_KEY);
    }

    calibration_complete_at_tick_start =
        control_gimbal_calibration_complete;
    requests = control_take_requests();
    gimbal_stepper_get_status(&gimbal_status);
    control_update_gimbal_calibration(&gimbal_status);
    if ((CONTROL_GIMBAL_ENABLED == 0U)
        || (control_gimbal_calibration_complete != 0U))
    {
        key_scanner();
        if (calibration_complete_at_tick_start != 0U)
        {
            control_add_key_requests(&requests);
        }
        else
        {
            requests.flags = 0U;
            requests.chassis_command = CONTROL_CHASSIS_REQUEST_NONE;
            key_clear_all_state();
        }
    }
    else
    {
        requests.flags = 0U;
        requests.chassis_command = CONTROL_CHASSIS_REQUEST_NONE;
        key_clear_all_state();
    }

    /* Phase 2: acquire this tick's encoder, IMU and grayscale samples. */
    my_encoder_get_delta(&left_count, &right_count);
    imu_uart_update();
    imu_data.angle_deg[0] = 0.0F;
    imu_data.angle_deg[1] = 0.0F;
    imu_data.angle_deg[2] = 0.0F;
    imu_data.angle_frame_count = 0U;
    imu_data.checksum_error_count = 0U;
    imu_data.angle_valid = 0U;
    imu_data_valid = imu_uart_get_data(&imu_data);
    yaw_valid = imu_data_valid;
    if (imu_data_valid != 0U)
    {
        yaw_frame_count = imu_data.angle_frame_count;
    }
    gray = control_status.gray;
    gray_valid = gray_sensor_sample(&gray);

    control_status.left_count = left_count;
    control_status.right_count = right_count;
    if (gray_valid != 0U)
    {
        control_status.gray = gray;
    }

    control_status.imu_angle_frame_count = yaw_frame_count;
    control_status.imu_valid = yaw_valid;
    new_angle_frame = (uint8)((yaw_valid != 0U)
        && (yaw_frame_count != control_last_imu_frame_count));
    if (new_angle_frame != 0U)
    {
        control_last_imu_frame_count = yaw_frame_count;
        control_status.imu_age_ticks = 0U;
    }
    else if (control_status.imu_age_ticks < 0xFFFFU)
    {
        control_status.imu_age_ticks++;
    }
    control_status.imu_fresh =
        (uint8)((yaw_valid != 0U)
            && (control_status.imu_age_ticks
                <= CONTROL_IMU_FRESH_LIMIT_TICKS));
    control_update_imu_stability(
        &imu_data,
        new_angle_frame);
    yaw_deg = control_status.imu_yaw_deg;

    /* Phase 3: validate inputs and gate corrected IMU use. */
    if (gray_valid == ZF_FALSE)
    {
        control_latch_fault(CONTROL_FAULT_GRAY_SAMPLE);
    }
    encoder_valid = (uint8)(
        (control_count_magnitude(control_status.left_count)
            <= CONTROL_ENCODER_COUNT_LIMIT)
        && (control_count_magnitude(control_status.right_count)
            <= CONTROL_ENCODER_COUNT_LIMIT));
    if (encoder_valid == 0U)
    {
        control_latch_fault(CONTROL_FAULT_ENCODER_RANGE);
    }

    odometry_yaw_valid = (uint8)(
        (control_status.imu_ready != 0U)
        && (control_status.imu_fresh != 0U));
    if ((control_status.mode == CONTROL_MODE_LINE_FOLLOW)
        && ((control_status.imu_fresh == 0U)
            || (control_status.imu_ready == 0U)))
    {
        control_enter_disarmed();
    }
    if ((control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
        && ((control_status.imu_fresh == 0U)
            || (control_status.imu_ready == 0U)))
    {
        control_latch_fault(CONTROL_FAULT_IMU_STALE);
    }

    /* Phase 4: suppress stopped-wheel jitter before pose integration. */
    odometry_left_count = encoder_valid != 0U ? left_count : 0;
    odometry_right_count = encoder_valid != 0U ? right_count : 0;
    if ((control_count_magnitude(odometry_left_count)
            <= CONTROL_STOPPED_COUNT_LIMIT)
        && (control_count_magnitude(odometry_right_count)
            <= CONTROL_STOPPED_COUNT_LIMIT))
    {
        odometry_left_count = 0;
        odometry_right_count = 0;
    }
    odometry_update(
        odometry_left_count,
        odometry_right_count,
        odometry_yaw_valid,
        yaw_deg,
        yaw_frame_count);
    control_apply_requests(&requests, emergency_active);
    odometry_get_state(&odometry);

    if ((CONTROL_GIMBAL_ENABLED != 0U)
        && (control_gimbal_calibration_complete != 0U)
        && (control_status.imu_fresh != 0U)
        && (control_status.imu_ready != 0U)
        && (control_status.mode != CONTROL_MODE_FAULT_LATCHED))
    {
        control_gimbal_feedforward_divider++;
        if (control_gimbal_feedforward_divider
            >= CONTROL_GIMBAL_FEEDFORWARD_DIVIDER)
        {
            control_gimbal_feedforward_divider = 0U;
            gimbal_pose.x_mm = odometry.x_mm;
            gimbal_pose.y_mm = odometry.y_mm;
            gimbal_pose.z_mm = 0.0F;
            gimbal_pose.roll_deg = GIMBAL_CONFIG_USE_BODY_ROLL != 0U
                ? control_status.imu_roll_deg
                : 0.0F;
            gimbal_pose.pitch_deg = GIMBAL_CONFIG_USE_BODY_PITCH != 0U
                ? control_status.imu_pitch_deg
                : 0.0F;
            gimbal_pose.heading_rad = odometry.theta_rad;
            gimbal_pose.valid = 1U;
            control_gimbal_pose_mailbox = gimbal_pose;
            control_gimbal_pose_sequence++;
            control_gimbal_pose_valid = 1U;
        }
    }
    else
    {
        control_gimbal_feedforward_divider = 0U;
        control_gimbal_pose_valid = 0U;
        control_status.gimbal_feedforward_valid = 0U;
        control_status.gimbal_feedforward_reject_reason =
            CONTROL_GIMBAL_REJECT_NOT_READY;
        if (CONTROL_GIMBAL_ENABLED != 0U)
        {
            (void)gimbal_stepper_set_laser(0U);
        }
    }

    /* Phase 5: update mode timeouts and select the owning target source. */
    control_update_manual_timeout();
    line_output.left_target_mm_s = 0.0F;
    line_output.right_target_mm_s = 0.0F;
    control_select_targets(
        &gray,
        &odometry,
        yaw_deg,
        left_count,
        right_count,
        &line_output,
        &left_target,
        &right_target);
    control_status.line_output = line_output;

    if (control_status.mode == CONTROL_MODE_FAULT_LATCHED)
    {
        left_target = 0.0F;
        right_target = 0.0F;
    }

    /* Phase 6: drive wheel control, then publish coherent status. */
    speed_pid_set_target(left_target, right_target);
    speed_pid_update_10ms(
        left_count,
        right_count);
    control_publish_status();
    control_update_busy = 0U;
}

/**
 * @brief Run deferred non-real-time control actions in foreground context.
 */
void control_scheduler_process_foreground(void)
{
    gimbal_feedforward_pose_struct gimbal_pose;
    gimbal_feedforward_solution_struct gimbal_solution;
    uint32 gimbal_pose_sequence = 0U;
    uint32 solve_start_tick;
    uint32 solve_end_tick;
    uint32 sequence_lag;
    uint32 primask;
    uint8 gimbal_pose_pending = 0U;
    uint8 gimbal_update_accepted;
    uint8 gimbal_solution_valid;

    if (CONTROL_GIMBAL_ENABLED == 0U)
    {
        return;
    }

    (void)gimbal_stepper_service();

    primask = interrupt_global_disable();
    if ((control_gimbal_pose_valid != 0U)
        && (control_gimbal_pose_sequence
            != control_gimbal_pose_consumed_sequence))
    {
        gimbal_pose = control_gimbal_pose_mailbox;
        gimbal_pose_sequence = control_gimbal_pose_sequence;
        gimbal_pose_pending = 1U;
    }
    interrupt_global_enable(primask);

    if (gimbal_pose_pending != 0U)
    {
        solve_start_tick = control_status.tick_count;
        gimbal_solution_valid = gimbal_stepper_compute_feedforward(
            &gimbal_pose,
            &gimbal_solution);
        solve_end_tick = control_status.tick_count;

        interrupt_disable(TIMG12_INT_IRQn);
        sequence_lag = (uint32)(control_gimbal_pose_sequence
            - gimbal_pose_sequence);
        control_gimbal_last_solution = gimbal_solution;
        control_status.gimbal_feedforward_solve_ticks = (uint16)(
            solve_end_tick - solve_start_tick);
        gimbal_update_accepted = 0U;
        if (gimbal_solution_valid == 0U)
        {
            control_status.gimbal_feedforward_reject_reason =
                CONTROL_GIMBAL_REJECT_SOLVER;
        }
        else if ((control_gimbal_pose_valid == 0U)
            || (control_gimbal_calibration_complete == 0U)
            || (control_status.imu_ready == 0U)
            || (control_status.mode == CONTROL_MODE_FAULT_LATCHED))
        {
            control_status.gimbal_feedforward_reject_reason =
                CONTROL_GIMBAL_REJECT_NOT_READY;
        }
        else if (sequence_lag > CONTROL_GIMBAL_MAX_SEQUENCE_LAG)
        {
            control_status.gimbal_feedforward_reject_reason =
                CONTROL_GIMBAL_REJECT_STALE;
            control_status.gimbal_feedforward_stale_count++;
        }
        else
        {
            gimbal_update_accepted =
                gimbal_stepper_apply_feedforward_solution(
                    &gimbal_solution);
            if (gimbal_update_accepted == 0U)
            {
                control_status.gimbal_feedforward_reject_reason =
                    CONTROL_GIMBAL_REJECT_APPLY;
            }
        }
        control_gimbal_pose_consumed_sequence = gimbal_pose_sequence;
        if (gimbal_update_accepted != 0U)
        {
            control_status.gimbal_feedforward_valid = 1U;
            control_status.gimbal_feedforward_count++;
            control_status.gimbal_feedforward_reject_reason =
                CONTROL_GIMBAL_REJECT_NONE;
        }
        else
        {
            control_status.gimbal_feedforward_valid = 0U;
            control_status.gimbal_feedforward_reject_count++;
        }
        interrupt_enable(TIMG12_INT_IRQn);

        if (gimbal_update_accepted == 0U)
        {
            (void)gimbal_stepper_set_laser(0U);
        }
    }

}

/**
 * @brief Submit an explicit arm request.
 */
void control_scheduler_request_arm(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_ARM;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit an explicit disarm request.
 */
void control_scheduler_request_disarm(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_DISARM;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a line-follow start request.
 */
void control_scheduler_request_line_start(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_LINE_START;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a line-follow stop request.
 */
void control_scheduler_request_line_stop(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_LINE_STOP;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a latched-fault clear request.
 */
void control_scheduler_request_fault_clear(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_FAULT_CLEAR;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit one bounded manual target pair.
 * @param left_mm_s Left target speed.
 * @param right_mm_s Right target speed.
 * @return ZF_TRUE when the request values are valid.
 */
uint8 control_scheduler_request_manual_target(
    float left_mm_s,
    float right_mm_s)
{
    uint32 primask;

    if ((control_target_is_valid(left_mm_s) == 0U)
        || (control_target_is_valid(right_mm_s) == 0U))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.left_target_mm_s = left_mm_s;
    control_mailbox.right_target_mm_s = right_mm_s;
    control_mailbox.flags |= CONTROL_REQUEST_MANUAL_TARGET;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a high-level signed distance command.
 * @param distance_mm Positive for forward and negative for reverse distance.
 * @param max_speed_mm_s Positive maximum center speed.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_distance(
    float distance_mm,
    float max_speed_mm_s)
{
    uint32 primask;

    if ((control_value_is_valid(distance_mm) == 0U)
        || (control_target_is_valid(max_speed_mm_s) == 0U)
        || (distance_mm == 0.0F)
        || (max_speed_mm_s <= 0.0F))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_value = distance_mm;
    control_mailbox.chassis_speed = max_speed_mm_s;
    control_mailbox.chassis_duration_ms = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_DISTANCE;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_COMMAND;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a high-level timed signed-speed command.
 * @param speed_mm_s Positive for forward and negative for reverse speed.
 * @param duration_ms Command duration.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_timed(
    float speed_mm_s,
    uint32 duration_ms)
{
    uint32 primask;

    if ((control_target_is_valid(speed_mm_s) == 0U)
        || (speed_mm_s == 0.0F)
        || (duration_ms == 0U))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_speed = speed_mm_s;
    control_mailbox.chassis_duration_ms = duration_ms;
    control_mailbox.chassis_value = 0.0F;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_TIMED;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_COMMAND;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a high-level relative heading-turn command.
 * @param angle_deg Relative angle from -360 to 360 degrees, excluding zero.
 *                  Positive turns left and negative turns right.
 * @param max_angular_speed_deg_s Positive maximum angular speed.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_turn_relative(
    float angle_deg,
    float max_angular_speed_deg_s)
{
    uint32 primask;

    if (control_turn_request_is_valid(
            angle_deg,
            max_angular_speed_deg_s) == 0U)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_value = angle_deg;
    control_mailbox.chassis_speed = max_angular_speed_deg_s;
    control_mailbox.chassis_duration_ms = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_TURN;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_COMMAND;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a request to smoothly cancel the active chassis command.
 */
void control_scheduler_request_chassis_motion_cancel(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_CANCEL;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a request to select one configured chassis PID parameter group.
 * @param profile_id Profile identifier from 0 to 3.
 * @return ZF_TRUE when the profile identifier is valid.
 */
uint8 control_scheduler_request_chassis_motion_pid_profile(uint8 profile_id)
{
    uint32 primask;

    if (profile_id >= CHASSIS_MOTION_PID_PROFILE_COUNT)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_profile_id = profile_id;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_PID_PROFILE;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Copy one coherent scheduler telemetry snapshot.
 * @param status Destination status structure.
 */
void control_scheduler_get_status(control_scheduler_status_struct *status)
{
    uint32 primask;

    if (status == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *status = control_status;
    interrupt_global_enable(primask);
}

/**
 * @brief Return the current scheduler tick without copying full telemetry.
 */
uint32 control_scheduler_get_tick_count(void)
{
    uint32 primask = interrupt_global_disable();
    uint32 tick_count = control_status.tick_count;

    interrupt_global_enable(primask);
    return tick_count;
}
