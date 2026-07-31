/**
 * @file    test_line_follow_task_5.c
 * @author  Project team
 * @version V1.0
 * @date    2026-08-01
 * @brief   Ball-groove acceleration feedforward with line-follow timing.
 *
 * @attention
 * - A30 captures the manually levelled lift zero before driving.
 * - The stopwatch starts when the scheduler accepts line-follow mode.
 * - OLED reports timing, startup stages, blocking errors and MPU freshness.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_TASK_5)

#include "control_scheduler.h"
#include "gimbal_stepper.h"
#include "my_lib_mpu6500.h"
#include "ml_oled.h"
#include "test_line_follow_task_5.h"
#include "zf_driver_delay.h"

#if (GIMBAL_CONFIG_INSTALLED != 0U)
#error Task 5 owns the lift and requires GIMBAL_CONFIG_INSTALLED to be zero.
#endif

#define TASK_5_STOP_MASK                    (0x78U)
#define TASK_5_STOP_DELAY_TICKS             \
    (280U / CONTROL_SCHEDULER_PERIOD_MS)
#define TASK_5_SOFT_START_TICKS             \
    (2000U / CONTROL_SCHEDULER_PERIOD_MS)
#define TASK_5_STOPPED_COUNT_LIMIT          (1)

typedef enum
{
    TASK_5_STOP_MONITOR = 0,
    TASK_5_STOP_DELAY_TRACKING,
    TASK_5_STOP_WAIT_BRAKE,
    TASK_5_STOP_DESCENDING,
    TASK_5_STOP_RISING,
    TASK_5_STOP_WAIT_WHEELS,
    TASK_5_STOP_FINISHED,
} task_5_stop_state_enum;

typedef enum
{
    TASK_5_START_WAIT_ARM = 0,
    TASK_5_START_CALIBRATING,
    TASK_5_START_WAIT_LINE,
    TASK_5_START_SOFT_START,
    TASK_5_START_RUNNING,
} task_5_start_state_enum;

#define BALL_GROOVE_TRAVEL_LIMIT_STEPS       (124272)
#define BALL_GROOVE_JOG_RATE_STEPS_S         (777U)
#define BALL_GROOVE_LOOP_PERIOD_MS           (1U)
#define BALL_GROOVE_BASELINE_SAMPLES         (50U)
#define BALL_GROOVE_MPU_FRESH_LIMIT_TICKS    (2U)
#define BALL_GROOVE_ACCEL_DEADBAND_ENTER_G   (0.006F)
#define BALL_GROOVE_ACCEL_DEADBAND_EXIT_G    (0.003F)
#define BALL_GROOVE_ACCEL_TO_LIFT_GAIN_MM_PER_G (250.0F)
#define BALL_GROOVE_ACCEL_FILTER_ALPHA       (0.3F)
#define BALL_GROOVE_TRAVEL_LIMIT_MM          (40.0F)
#define BALL_GROOVE_STEPS_PER_REVOLUTION     (6400.0F)
#define BALL_GROOVE_LIFT_MM_PER_REVOLUTION   (2.06F)
#define BALL_GROOVE_PARK_DESCENT_MM          (10.0F)
#define BALL_GROOVE_PARK_RATE_MM_S           (10.0F)
#define BALL_GROOVE_PARK_SETTLE_TOLERANCE_STEPS (2)

#define TASK_5_CURVE_ENTER_DEVIATION          (0.70F)
#define TASK_5_STRAIGHT_EXIT_DEVIATION        (0.25F)
#define TASK_5_STRAIGHT_EXIT_SAMPLES          (15U)
#define TASK_5_SOFT_START_ACCEL_MM_S2         (119.0F)

#define TASK_5_ERR_GIMBAL_CFG               (1U)
#define TASK_5_ERR_SCHED_INIT               (2U)
#define TASK_5_ERR_SCHED_START              (3U)
#define TASK_5_ERR_LINE_TRACKER_CFG         (4U)
#define TASK_5_ERR_MPU_INIT                 (5U)

#define TASK_5_TIME_TEXT_LENGTH              (13U)
#define TASK_5_STAGE_TEXT_LENGTH             (6U)
#define TASK_5_ERROR_TEXT_LENGTH             (8U)
#define TASK_5_IMU_TEXT_LENGTH               (14U)

static const line_tracker_config_struct task_5_straight_line_config =
{
    .base_speed_mm_s = 238.0F,
    .pid_kp = 15.4F,
    .pid_ki = 0.0F,
    .pid_kd = 0.0F,
    .pid_integral_limit_mm_s = 91.40F,
    .pid_derivative_filter_alpha = 0.2F,
    .max_target_mm_s = 560.0F,
    .max_correction_mm_s = 63.0F,
    .max_target_accel_mm_s2 = 2558.997F,
    .arc_outer_speed_mm_s = 383.852F,
    .arc_inner_speed_mm_s = 76.769F,
    .pivot_speed_mm_s = 383.852F,
    .lost_debounce_samples = 3U,
    .reacquire_samples = 3U,
    .arc_duration_samples = 100U,
    .search_timeout_samples = 500U,
    .default_search_direction = LINE_TRACKER_DIRECTION_RIGHT,
    .track_all_active_as_center = 1U,
};

/**
 * @brief Return whether all center D4-D7 stop sensors are active.
 */
static uint8 task_5_stop_pattern_detected(uint8 active_mask)
{
    return (uint8)(
        (active_mask & TASK_5_STOP_MASK)
            == TASK_5_STOP_MASK);
}

/**
 * @brief Return whether both wheel encoder intervals indicate a stopped vehicle.
 */
static uint8 task_5_wheels_stopped(
    const control_scheduler_status_struct *status)
{
    return (uint8)(
        (status->left_count >= -TASK_5_STOPPED_COUNT_LIMIT)
        && (status->left_count <= TASK_5_STOPPED_COUNT_LIMIT)
        && (status->right_count >= -TASK_5_STOPPED_COUNT_LIMIT)
        && (status->right_count <= TASK_5_STOPPED_COUNT_LIMIT));
}

/**
 * @brief Return whether the current gray sample is allowed to start tracking.
 */
static uint8 task_5_gray_allows_line_start(
    const control_scheduler_status_struct *status)
{
    return (uint8)(
        (status->gray.status == GRAY_SENSOR_STATUS_VALID)
        || ((status->gray.status == GRAY_SENSOR_STATUS_ALL_ACTIVE)
            && (line_tracker_tracks_all_active_as_center() != 0U)));
}

/**
 * @brief Convert one signed lift travel distance to bounded step units.
 */
static int32 task_5_lift_mm_to_steps(float lift_mm)
{
    float steps;

    if(lift_mm > BALL_GROOVE_TRAVEL_LIMIT_MM)
    {
        lift_mm = BALL_GROOVE_TRAVEL_LIMIT_MM;
    }
    else if(lift_mm < -BALL_GROOVE_TRAVEL_LIMIT_MM)
    {
        lift_mm = -BALL_GROOVE_TRAVEL_LIMIT_MM;
    }
    steps = lift_mm * BALL_GROOVE_STEPS_PER_REVOLUTION
        / BALL_GROOVE_LIFT_MM_PER_REVOLUTION;
    return steps >= 0.0F
        ? (int32)(steps + 0.5F)
        : (int32)(steps - 0.5F);
}

/**
 * @brief Move one commanded lift target toward its destination by one step.
 */
static int32 task_5_step_target_toward(
    int32 current_steps,
    int32 target_steps,
    int32 maximum_delta_steps)
{
    if(current_steps < target_steps)
    {
        if((target_steps - current_steps) > maximum_delta_steps)
        {
            return current_steps + maximum_delta_steps;
        }
    }
    else if(current_steps > target_steps)
    {
        if((current_steps - target_steps) > maximum_delta_steps)
        {
            return current_steps - maximum_delta_steps;
        }
    }
    return target_steps;
}

/**
 * @brief Return whether the lift has stopped at a requested target.
 */
static uint8 task_5_lift_at_target(int32 target_steps)
{
    gimbal_stepper_status_struct gimbal_status;
    const gimbal_stepper_axis_status_struct *lift;
    int32 position_error;

    gimbal_stepper_get_status(&gimbal_status);
    lift = &gimbal_status.axis[GIMBAL_STEPPER_AXIS_YAW];
    position_error = lift->position_steps - target_steps;
    if(position_error < 0)
    {
        position_error = -position_error;
    }
    return (uint8)(
        (lift->enabled != 0U)
        && (lift->zero_valid != 0U)
        && (position_error <= BALL_GROOVE_PARK_SETTLE_TOLERANCE_STEPS)
        && (lift->current_rate_steps_s == 0));
}

/**
 * @brief Advance the D4-D7 final-stop and lift-settle state machine.
 */
static void task_5_update_stop_state(
    const control_scheduler_status_struct *status,
    task_5_stop_state_enum *stop_state,
    uint32 *stop_trigger_tick,
    int32 *park_command_steps)
{
    int32 descent_target_steps = task_5_lift_mm_to_steps(
        -BALL_GROOVE_PARK_DESCENT_MM);
    int32 maximum_delta_steps = task_5_lift_mm_to_steps(
        BALL_GROOVE_PARK_RATE_MM_S
        * ((float)CONTROL_SCHEDULER_PERIOD_MS / 1000.0F));

    if(maximum_delta_steps <= 0)
    {
        maximum_delta_steps = 1;
    }
    if(*stop_state == TASK_5_STOP_MONITOR)
    {
        if((status->mode == CONTROL_MODE_LINE_FOLLOW)
            && (task_5_stop_pattern_detected(
                status->gray.active_mask) != 0U))
        {
            *stop_trigger_tick = status->tick_count;
            *stop_state = TASK_5_STOP_DELAY_TRACKING;
        }
    }
    else if(*stop_state == TASK_5_STOP_DELAY_TRACKING)
    {
        if(status->mode != CONTROL_MODE_LINE_FOLLOW)
        {
            *stop_state = TASK_5_STOP_MONITOR;
        }
        else if((uint32)(status->tick_count - *stop_trigger_tick)
            >= TASK_5_STOP_DELAY_TICKS)
        {
            control_scheduler_request_line_stop();
            *stop_state = TASK_5_STOP_WAIT_BRAKE;
        }
    }
    else if(*stop_state == TASK_5_STOP_WAIT_BRAKE)
    {
        if(status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            gimbal_stepper_status_struct gimbal_status;

            gimbal_stepper_get_status(&gimbal_status);
            *park_command_steps = gimbal_status.axis[
                GIMBAL_STEPPER_AXIS_YAW].target_position_steps;
            *stop_state = TASK_5_STOP_DESCENDING;
        }
        else if((status->mode == CONTROL_MODE_FAULT_LATCHED)
            || (status->mode == CONTROL_MODE_DISARMED))
        {
            *stop_state = TASK_5_STOP_FINISHED;
        }
    }
    else if(*stop_state == TASK_5_STOP_DESCENDING)
    {
        if(status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            *park_command_steps = task_5_step_target_toward(
                *park_command_steps,
                descent_target_steps,
                maximum_delta_steps);
            (void)gimbal_stepper_set_axis_absolute_target_steps(
                GIMBAL_STEPPER_AXIS_YAW,
                *park_command_steps);
            if((*park_command_steps == descent_target_steps)
                && (task_5_lift_at_target(descent_target_steps) != 0U))
            {
                *stop_state = TASK_5_STOP_RISING;
            }
        }
        else
        {
            *stop_state = TASK_5_STOP_FINISHED;
        }
    }
    else if(*stop_state == TASK_5_STOP_RISING)
    {
        if(status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            *park_command_steps = task_5_step_target_toward(
                *park_command_steps,
                0,
                maximum_delta_steps);
            (void)gimbal_stepper_set_axis_absolute_target_steps(
                GIMBAL_STEPPER_AXIS_YAW,
                *park_command_steps);
            if((*park_command_steps == 0)
                && (task_5_lift_at_target(0) != 0U))
            {
                *stop_state = TASK_5_STOP_WAIT_WHEELS;
            }
        }
        else
        {
            *stop_state = TASK_5_STOP_FINISHED;
        }
    }
    else if(*stop_state == TASK_5_STOP_WAIT_WHEELS)
    {
        if(task_5_wheels_stopped(status) != 0U)
        {
            *stop_state = TASK_5_STOP_FINISHED;
        }
    }
}

/**
 * @brief Apply one of Task 5's straight/curve line-tracking PID profiles.
 */
static uint8 task_5_apply_line_profile(uint8 curve_profile)
{
    line_tracker_config_struct config = task_5_straight_line_config;

    if(curve_profile != 0U)
    {
        config.pid_kp = 33.6F;
        config.max_correction_mm_s = 126.0F;
    }
    return line_tracker_set_config(&config);
}

/**
 * @brief Apply the two-second low-acceleration launch profile.
 */
static uint8 task_5_apply_soft_start_profile(void)
{
    line_tracker_config_struct config = task_5_straight_line_config;

    config.max_target_accel_mm_s2 = TASK_5_SOFT_START_ACCEL_MM_S2;
    return line_tracker_set_config(&config);
}

/**
 * @brief Select the active line-tracking PID profile from deviation hysteresis.
 */
static void task_5_update_line_profile(
    const control_scheduler_status_struct *status,
    uint8 *curve_profile,
    uint8 *straight_exit_samples)
{
    float absolute_deviation = status->gray.deviation;

    if(absolute_deviation < 0.0F)
    {
        absolute_deviation = -absolute_deviation;
    }

    if(*curve_profile == 0U)
    {
        *straight_exit_samples = 0U;
        if(absolute_deviation >= TASK_5_CURVE_ENTER_DEVIATION)
        {
            if(task_5_apply_line_profile(1U) != ZF_FALSE)
            {
                *curve_profile = 1U;
            }
        }
    }
    else if(absolute_deviation <= TASK_5_STRAIGHT_EXIT_DEVIATION)
    {
        if(*straight_exit_samples < TASK_5_STRAIGHT_EXIT_SAMPLES)
        {
            (*straight_exit_samples)++;
        }
        if(*straight_exit_samples >= TASK_5_STRAIGHT_EXIT_SAMPLES)
        {
            if(task_5_apply_line_profile(0U) != ZF_FALSE)
            {
                *curve_profile = 0U;
                *straight_exit_samples = 0U;
            }
        }
    }
    else
    {
        *straight_exit_samples = 0U;
    }
}

/**
 * @brief Read Task 5's MPU acceleration source once per scheduler tick.
 */
static void task_5_sample_mpu(
    uint32 scheduler_tick,
    float *accel_x_g,
    uint32 *last_success_tick,
    uint16 *age_ticks,
    uint8 *sample_valid,
    uint8 *sample_seen,
    uint8 *sample_fresh)
{
    mpu6500_data_struct sensor_data;
    uint32 elapsed_ticks;

    *sample_valid = 0U;
    if(mpu6500_read(&sensor_data) == 0U)
    {
        *accel_x_g = sensor_data.accel_g[0];
        *last_success_tick = scheduler_tick;
        *age_ticks = 0U;
        *sample_valid = 1U;
        *sample_seen = 1U;
        *sample_fresh = 1U;
        return;
    }

    if(*sample_seen == 0U)
    {
        *age_ticks = 0xFFFFU;
        *sample_fresh = 0U;
        return;
    }

    elapsed_ticks = scheduler_tick - *last_success_tick;
    if(elapsed_ticks > 0xFFFFU)
    {
        *age_ticks = 0xFFFFU;
    }
    else
    {
        *age_ticks = (uint16)elapsed_ticks;
    }
    *sample_fresh = (uint8)(
        *age_ticks <= BALL_GROOVE_MPU_FRESH_LIMIT_TICKS);
}

/**
 * @brief Return the current Task 5 line-start gate error code.
 */
static uint8 task_5_get_start_error(
    task_5_start_state_enum start_state,
    const control_scheduler_status_struct *status,
    uint8 mpu_sample_valid)
{
    if(status->mode == CONTROL_MODE_FAULT_LATCHED)
    {
        return 14U;
    }
    if(start_state == TASK_5_START_WAIT_ARM)
    {
        return mpu_sample_valid != 0U ? 0U : 13U;
    }
    if(start_state == TASK_5_START_RUNNING)
    {
        return 0U;
    }
    if(start_state == TASK_5_START_SOFT_START)
    {
        return 0U;
    }
    if(status->mode == CONTROL_MODE_DISARMED)
    {
        return 15U;
    }
    if(start_state == TASK_5_START_CALIBRATING)
    {
        if(task_5_wheels_stopped(status) == 0U)
        {
            return 12U;
        }
        return mpu_sample_valid != 0U ? 0U : 13U;
    }
    if(task_5_gray_allows_line_start(status) == 0U)
    {
        return 11U;
    }
    if(task_5_wheels_stopped(status) == 0U)
    {
        return 12U;
    }
    return 0U;
}

/**
 * @brief Advance the initial Task 5 line-start request state machine.
 */
static void task_5_update_start_state(
    task_5_start_state_enum *start_state,
    const control_scheduler_status_struct *status,
    float *baseline_sum,
    float *baseline_g,
    uint16 *baseline_samples,
    uint8 *baseline_ready,
    float accel_x_g,
    uint8 mpu_sample_valid,
    uint32 scheduler_tick,
    uint32 *soft_start_tick)
{
    uint8 start_error = task_5_get_start_error(
        *start_state,
        status,
        mpu_sample_valid);

    if(*start_state == TASK_5_START_WAIT_ARM)
    {
        if(status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            *baseline_sum = 0.0F;
            *baseline_g = 0.0F;
            *baseline_samples = 0U;
            *baseline_ready = 0U;
            *start_state = TASK_5_START_CALIBRATING;
        }
    }
    else if(*start_state == TASK_5_START_CALIBRATING)
    {
        if(status->mode == CONTROL_MODE_DISARMED)
        {
            *start_state = TASK_5_START_WAIT_ARM;
            *baseline_sum = 0.0F;
            *baseline_g = 0.0F;
            *baseline_samples = 0U;
            *baseline_ready = 0U;
        }
        else if((status->mode == CONTROL_MODE_MANUAL_ARMED)
            && (task_5_wheels_stopped(status) != 0U)
            && (mpu_sample_valid != 0U))
        {
            /* A30 arm explicitly starts this 500 ms horizontal-plane sample. */
            *baseline_sum += accel_x_g;
            (*baseline_samples)++;
            if(*baseline_samples >= BALL_GROOVE_BASELINE_SAMPLES)
            {
                *baseline_g = *baseline_sum / (float)*baseline_samples;
                *baseline_ready = 1U;
                *start_state = TASK_5_START_WAIT_LINE;
            }
        }
        else if(status->mode == CONTROL_MODE_LINE_FOLLOW)
        {
            control_scheduler_request_line_stop();
        }
        else
        {
            *baseline_sum = 0.0F;
            *baseline_samples = 0U;
        }
    }
    else if(*start_state == TASK_5_START_WAIT_LINE)
    {
        if(status->mode == CONTROL_MODE_LINE_FOLLOW)
        {
            if(task_5_apply_soft_start_profile() != ZF_FALSE)
            {
                *soft_start_tick = scheduler_tick;
                *start_state = TASK_5_START_SOFT_START;
            }
        }
        else if(status->mode == CONTROL_MODE_DISARMED)
        {
            *start_state = TASK_5_START_WAIT_ARM;
            *baseline_sum = 0.0F;
            *baseline_g = 0.0F;
            *baseline_samples = 0U;
            *baseline_ready = 0U;
        }
        else if((status->mode == CONTROL_MODE_MANUAL_ARMED)
            && (start_error == 0U))
        {
            control_scheduler_request_line_start();
        }
    }
    else if(*start_state == TASK_5_START_SOFT_START)
    {
        if(status->mode == CONTROL_MODE_LINE_FOLLOW)
        {
            if((uint32)(scheduler_tick - *soft_start_tick)
                >= TASK_5_SOFT_START_TICKS)
            {
                (void)task_5_apply_line_profile(0U);
                *start_state = TASK_5_START_RUNNING;
            }
        }
        else if(status->mode == CONTROL_MODE_DISARMED)
        {
            *start_state = TASK_5_START_WAIT_ARM;
            *baseline_sum = 0.0F;
            *baseline_g = 0.0F;
            *baseline_samples = 0U;
            *baseline_ready = 0U;
        }
        else
        {
            (void)task_5_apply_line_profile(0U);
            *start_state = TASK_5_START_RUNNING;
        }
    }
    else if(status->mode == CONTROL_MODE_DISARMED)
    {
        *start_state = TASK_5_START_WAIT_ARM;
        *baseline_sum = 0.0F;
        *baseline_g = 0.0F;
        *baseline_samples = 0U;
        *baseline_ready = 0U;
    }
}

/**
 * @brief Return the OLED stage code for startup and stop-state diagnostics.
 */
static uint8 task_5_get_stage_code(
    task_5_start_state_enum start_state,
    task_5_stop_state_enum stop_state,
    const control_scheduler_status_struct *status)
{
    if(status->mode == CONTROL_MODE_FAULT_LATCHED)
    {
        return 90U;
    }
    if(start_state == TASK_5_START_WAIT_ARM)
    {
        return 10U;
    }
    if(start_state == TASK_5_START_CALIBRATING)
    {
        return 11U;
    }
    if(start_state == TASK_5_START_WAIT_LINE)
    {
        return 12U;
    }
    if(stop_state == TASK_5_STOP_DELAY_TRACKING)
    {
        return 30U;
    }
    if(stop_state == TASK_5_STOP_WAIT_BRAKE)
    {
        return 31U;
    }
    if(stop_state == TASK_5_STOP_DESCENDING)
    {
        return 35U;
    }
    if(stop_state == TASK_5_STOP_RISING)
    {
        return 36U;
    }
    if((stop_state == TASK_5_STOP_WAIT_WHEELS)
        || (stop_state == TASK_5_STOP_FINISHED))
    {
        return 37U;
    }
    if(start_state == TASK_5_START_SOFT_START)
    {
        return 13U;
    }
    return status->mode == CONTROL_MODE_LINE_FOLLOW ? 20U : 21U;
}

/**
 * @brief Limit one acceleration-derived linear lift command to the mechanism.
 */
static int32 task_5_accel_target_steps(float acceleration_g)
{
    float lift_mm = acceleration_g * BALL_GROOVE_ACCEL_TO_LIFT_GAIN_MM_PER_G;

    return task_5_lift_mm_to_steps(lift_mm);
}

/**
 * @brief Apply a hysteretic acceleration deadband without filter delay.
 */
static float task_5_accel_apply_deadband(
    float acceleration_g,
    uint8 *deadband_active)
{
    if(*deadband_active != 0U)
    {
        if((acceleration_g >= -BALL_GROOVE_ACCEL_DEADBAND_EXIT_G)
            && (acceleration_g <= BALL_GROOVE_ACCEL_DEADBAND_EXIT_G))
        {
            *deadband_active = 0U;
            return 0.0F;
        }
        return acceleration_g;
    }
    if((acceleration_g > -BALL_GROOVE_ACCEL_DEADBAND_ENTER_G)
        && (acceleration_g < BALL_GROOVE_ACCEL_DEADBAND_ENTER_G))
    {
        return 0.0F;
    }
    *deadband_active = 1U;
    return acceleration_g;
}

/**
 * @brief Report an initialization error on the OLED and halt.
 * @param error_code One of the TASK_5_ERR_* identifiers.
 * @note This function never returns.
 */
static void task_5_report_error(uint8 error_code)
{
    static const char hex_digits[] = "0123456789ABCDEF";

    (void)ml_oled_init();
    (void)ml_oled_show_string(1U, 1U, "TIME:WAIT");
    (void)ml_oled_show_string(2U, 1U, "STG:99");
    (void)ml_oled_show_string(3U, 1U, "ERR:0x");
    (void)ml_oled_show_char(
        3U,
        7U,
        hex_digits[(error_code >> 4U) & 0x0FU]);
    (void)ml_oled_show_char(3U, 8U, hex_digits[error_code & 0x0FU]);
    while(true)
    {
    }
}

/**
 * @brief Build the fixed-width OLED line-one stopwatch text.
 */
static void task_5_build_time_text(
    char text[TASK_5_TIME_TEXT_LENGTH],
    uint8 started,
    uint32 elapsed_tenths)
{
    uint32 seconds = elapsed_tenths / 10U;

    text[0] = 'T';
    text[1] = 'I';
    text[2] = 'M';
    text[3] = 'E';
    text[4] = ':';
    if(started == 0U)
    {
        text[5] = 'W';
        text[6] = 'A';
        text[7] = 'I';
        text[8] = 'T';
        text[9] = ' ';
        text[10] = ' ';
        text[11] = ' ';
        text[12] = ' ';
        return;
    }

    seconds %= 100000U;
    text[5] = (char)('0' + ((seconds / 10000U) % 10U));
    text[6] = (char)('0' + ((seconds / 1000U) % 10U));
    text[7] = (char)('0' + ((seconds / 100U) % 10U));
    text[8] = (char)('0' + ((seconds / 10U) % 10U));
    text[9] = (char)('0' + (seconds % 10U));
    text[10] = '.';
    text[11] = (char)('0' + (elapsed_tenths % 10U));
    text[12] = 's';
}

/**
 * @brief Dirty-refresh only changed characters in OLED line one.
 */
static void task_5_render_time(
    uint8 started,
    uint32 elapsed_tenths,
    char cache[TASK_5_TIME_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[TASK_5_TIME_TEXT_LENGTH];
    uint8 index;

    task_5_build_time_text(text, started, elapsed_tenths);
    for(index = 0U; index < TASK_5_TIME_TEXT_LENGTH; index++)
    {
        if((*cache_valid == 0U) || (text[index] != cache[index]))
        {
            (void)ml_oled_show_char(1U, (uint8)(index + 1U), text[index]);
            cache[index] = text[index];
        }
    }
    *cache_valid = 1U;
}

/**
 * @brief Dirty-refresh the OLED stage line.
 */
static void task_5_render_stage(
    uint8 stage,
    char cache[TASK_5_STAGE_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[TASK_5_STAGE_TEXT_LENGTH];
    uint8 index;

    text[0] = 'S';
    text[1] = 'T';
    text[2] = 'G';
    text[3] = ':';
    text[4] = (char)('0' + ((stage / 10U) % 10U));
    text[5] = (char)('0' + (stage % 10U));
    for(index = 0U; index < TASK_5_STAGE_TEXT_LENGTH; index++)
    {
        if((*cache_valid == 0U) || (text[index] != cache[index]))
        {
            (void)ml_oled_show_char(2U, (uint8)(index + 1U), text[index]);
            cache[index] = text[index];
        }
    }
    *cache_valid = 1U;
}

/**
 * @brief Dirty-refresh the OLED line-start gate or scheduler fault line.
 */
static void task_5_render_error(
    uint8 error_code,
    uint32 fault_flags,
    uint8 fault_latched,
    char cache[TASK_5_ERROR_TEXT_LENGTH],
    uint8 *cache_valid)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    char text[TASK_5_ERROR_TEXT_LENGTH];
    uint8 index;

    if(fault_latched != 0U)
    {
        text[0] = 'F';
        text[1] = 'L';
        text[2] = 'T';
        text[3] = ':';
        text[4] = '0';
        text[5] = 'x';
        text[6] = hex_digits[(fault_flags >> 4U) & 0x0FU];
        text[7] = hex_digits[fault_flags & 0x0FU];
    }
    else
    {
        text[0] = 'E';
        text[1] = 'R';
        text[2] = 'R';
        text[3] = ':';
        text[4] = (char)('0' + ((error_code / 10U) % 10U));
        text[5] = (char)('0' + (error_code % 10U));
        text[6] = ' ';
        text[7] = ' ';
    }
    for(index = 0U; index < TASK_5_ERROR_TEXT_LENGTH; index++)
    {
        if((*cache_valid == 0U) || (text[index] != cache[index]))
        {
            (void)ml_oled_show_char(3U, (uint8)(index + 1U), text[index]);
            cache[index] = text[index];
        }
    }
    *cache_valid = 1U;
}

/**
 * @brief Dirty-refresh the Task 5 MPU read and frame-age diagnostic.
 */
static void task_5_render_imu_status(
    uint8 imu_ready,
    uint8 imu_fresh,
    uint16 imu_age_ticks,
    char cache[TASK_5_IMU_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[TASK_5_IMU_TEXT_LENGTH];
    uint16 displayed_age = imu_age_ticks;
    uint8 index;

    if(displayed_age > 999U)
    {
        displayed_age = 999U;
    }
    text[0] = 'I';
    text[1] = 'M';
    text[2] = 'U';
    text[3] = ':';
    text[4] = 'R';
    text[5] = imu_ready != 0U ? '1' : '0';
    text[6] = ' ';
    text[7] = 'F';
    text[8] = imu_fresh != 0U ? '1' : '0';
    text[9] = ' ';
    text[10] = 'A';
    text[11] = (char)('0' + (displayed_age / 100U));
    text[12] = (char)('0' + ((displayed_age / 10U) % 10U));
    text[13] = (char)('0' + (displayed_age % 10U));
    for(index = 0U; index < TASK_5_IMU_TEXT_LENGTH; index++)
    {
        if((*cache_valid == 0U) || (text[index] != cache[index]))
        {
            (void)ml_oled_show_char(4U, (uint8)(index + 1U), text[index]);
            cache[index] = text[index];
        }
    }
    *cache_valid = 1U;
}

/**
 * @brief Run ball-groove acceleration feedforward with line-follow timing.
 */
void test_line_follow_task_5_run(void)
{
    control_scheduler_status_struct status;
    task_5_stop_state_enum stop_state = TASK_5_STOP_MONITOR;
    uint32 stop_trigger_tick = 0U;
    uint32 last_control_tick = 0xFFFFFFFFU;
    uint32 mpu_last_success_tick = 0U;
    uint32 start_tick = 0U;
    uint32 soft_start_tick = 0U;
    uint32 last_profile_tick = 0xFFFFFFFFU;
    int32 park_command_steps = 0;
    float accel_baseline_sum = 0.0F;
    float accel_baseline_g = 0.0F;
    float filtered_accel_g = 0.0F;
    uint16 accel_baseline_samples = 0U;
    uint16 mpu_age_ticks = 0xFFFFU;
    uint8 accel_baseline_ready = 0U;
    uint8 accel_deadband_active = 0U;
    uint8 filtered_valid = 0U;
    uint8 stopwatch_running = 0U;
    uint8 lift_center_commanded = 0U;
    uint8 curve_profile = 0U;
    uint8 straight_exit_samples = 0U;
    uint8 mpu_sample_valid = 0U;
    uint8 mpu_sample_seen = 0U;
    uint8 mpu_sample_fresh = 0U;
    float mpu_accel_x_g = 0.0F;
    control_mode_enum previous_mode = CONTROL_MODE_BOOT;
    task_5_start_state_enum start_state = TASK_5_START_WAIT_ARM;
    char display_cache[TASK_5_TIME_TEXT_LENGTH] = {0};
    char stage_display_cache[TASK_5_STAGE_TEXT_LENGTH] = {0};
    char error_display_cache[TASK_5_ERROR_TEXT_LENGTH] = {0};
    char imu_display_cache[TASK_5_IMU_TEXT_LENGTH] = {0};
    uint8 display_cache_valid = 0U;
    uint8 stage_display_cache_valid = 0U;
    uint8 error_display_cache_valid = 0U;
    uint8 imu_display_cache_valid = 0U;

    if(ml_oled_init() == false)
    {
        while(true)
        {
        }
    }
    task_5_render_time(0U, 0U, display_cache, &display_cache_valid);
    task_5_render_stage(1U, stage_display_cache, &stage_display_cache_valid);
    task_5_render_error(
        0U,
        0U,
        0U,
        error_display_cache,
        &error_display_cache_valid);
    task_5_render_imu_status(
        0U,
        0U,
        0U,
        imu_display_cache,
        &imu_display_cache_valid);

    gimbal_stepper_init();
    if(gimbal_stepper_configure_single_axis(
            GIMBAL_STEPPER_AXIS_YAW,
            -BALL_GROOVE_TRAVEL_LIMIT_STEPS,
            BALL_GROOVE_TRAVEL_LIMIT_STEPS,
            BALL_GROOVE_JOG_RATE_STEPS_S) == 0U)
    {
        task_5_report_error(TASK_5_ERR_GIMBAL_CFG);
    }

    /* A30 captures the manually levelled ball-center position before driving. */
    while(gimbal_stepper_relative_ready() == 0U)
    {
        (void)gimbal_stepper_service();
        system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
    }
    gimbal_stepper_set_manual_control_enabled(0U);

    if(mpu6500_init() != 0U)
    {
        task_5_report_error(TASK_5_ERR_MPU_INIT);
    }
    control_scheduler_set_imu_bypass(1U);
    control_scheduler_set_imu_acceleration_only(0U);
    if(control_scheduler_init() == ZF_FALSE)
    {
        task_5_report_error(TASK_5_ERR_SCHED_INIT);
    }
    if(task_5_apply_line_profile(0U) == ZF_FALSE)
    {
        task_5_report_error(TASK_5_ERR_LINE_TRACKER_CFG);
    }
    if(control_scheduler_start() == ZF_FALSE)
    {
        task_5_report_error(TASK_5_ERR_SCHED_START);
    }
    while(true)
    {
        float command_accel_g = 0.0F;
        int32 target_steps = 0;
        uint32 elapsed_tenths = 0U;

        control_scheduler_process_foreground();
        (void)gimbal_stepper_service();
        control_scheduler_get_status(&status);

        if(status.tick_count == last_control_tick)
        {
            system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
            continue;
        }
        last_control_tick = status.tick_count;
        task_5_sample_mpu(
            status.tick_count,
            &mpu_accel_x_g,
            &mpu_last_success_tick,
            &mpu_age_ticks,
            &mpu_sample_valid,
            &mpu_sample_seen,
            &mpu_sample_fresh);
        if((status.mode == CONTROL_MODE_LINE_FOLLOW)
            && (start_state == TASK_5_START_RUNNING)
            && (status.tick_count != last_profile_tick))
        {
            task_5_update_line_profile(
                &status,
                &curve_profile,
                &straight_exit_samples);
            last_profile_tick = status.tick_count;
        }
        task_5_update_stop_state(
            &status,
            &stop_state,
            &stop_trigger_tick,
            &park_command_steps);

        if(status.mode != CONTROL_MODE_LINE_FOLLOW)
        {
            if(previous_mode == CONTROL_MODE_LINE_FOLLOW)
            {
                filtered_valid = 0U;
                curve_profile = 0U;
                straight_exit_samples = 0U;
                (void)task_5_apply_line_profile(0U);
            }
            previous_mode = status.mode;
            accel_deadband_active = 0U;
            if((stop_state != TASK_5_STOP_DESCENDING)
                && (stop_state != TASK_5_STOP_RISING)
                && (lift_center_commanded == 0U))
            {
                if(gimbal_stepper_set_axis_absolute_target_steps(
                        GIMBAL_STEPPER_AXIS_YAW,
                        0) != 0U)
                {
                    lift_center_commanded = 1U;
                }
            }
            filtered_accel_g = 0.0F;
        }
        else
        {
            previous_mode = status.mode;
            lift_center_commanded = 0U;
            if((accel_baseline_ready != 0U)
                && (mpu_sample_fresh != 0U))
            {
                /* +X acceleration commands positive steps: B1 raises the rear. */
                command_accel_g = mpu_accel_x_g - accel_baseline_g;
                command_accel_g = task_5_accel_apply_deadband(
                    command_accel_g,
                    &accel_deadband_active);
                if(filtered_valid == 0U)
                {
                    filtered_accel_g = command_accel_g;
                    filtered_valid = 1U;
                }
                else
                {
                    filtered_accel_g = BALL_GROOVE_ACCEL_FILTER_ALPHA
                            * command_accel_g
                        + (1.0F - BALL_GROOVE_ACCEL_FILTER_ALPHA)
                            * filtered_accel_g;
                }
                target_steps = task_5_accel_target_steps(filtered_accel_g);
                (void)gimbal_stepper_set_axis_absolute_target_steps(
                    GIMBAL_STEPPER_AXIS_YAW,
                    target_steps);
            }
            else
            {
                accel_deadband_active = 0U;
                filtered_valid = 0U;
                filtered_accel_g = 0.0F;
                (void)gimbal_stepper_set_axis_absolute_target_steps(
                    GIMBAL_STEPPER_AXIS_YAW,
                    0);
            }
        }

        task_5_update_start_state(
            &start_state,
            &status,
            &accel_baseline_sum,
            &accel_baseline_g,
            &accel_baseline_samples,
            &accel_baseline_ready,
            mpu_accel_x_g,
            mpu_sample_valid,
            status.tick_count,
            &soft_start_tick);
        if((stopwatch_running == 0U)
            && ((start_state == TASK_5_START_SOFT_START)
                || (start_state == TASK_5_START_RUNNING))
            && (status.mode == CONTROL_MODE_LINE_FOLLOW))
        {
            start_tick = status.tick_count;
            stopwatch_running = 1U;
        }
        if(stopwatch_running != 0U)
        {
            elapsed_tenths = (status.tick_count - start_tick) / 10U;
        }
        task_5_render_time(
            stopwatch_running,
            elapsed_tenths,
            display_cache,
            &display_cache_valid);
        task_5_render_stage(
            task_5_get_stage_code(start_state, stop_state, &status),
            stage_display_cache,
            &stage_display_cache_valid);
        task_5_render_error(
            task_5_get_start_error(
                start_state,
                &status,
                mpu_sample_valid),
            status.fault_flags,
            (uint8)(status.mode == CONTROL_MODE_FAULT_LATCHED),
            error_display_cache,
            &error_display_cache_valid);
        task_5_render_imu_status(
            mpu_sample_valid,
            mpu_sample_fresh,
            mpu_age_ticks,
            imu_display_cache,
            &imu_display_cache_valid);

        system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
    }
}

#endif
