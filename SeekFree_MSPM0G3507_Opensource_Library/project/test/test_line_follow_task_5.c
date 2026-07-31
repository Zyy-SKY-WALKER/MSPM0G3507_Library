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
 * - OLED reports timing, startup stages and blocking errors.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_TASK_5)

#include "control_scheduler.h"
#include "gimbal_stepper.h"
#include "ml_oled.h"
#include "test_line_follow_task_5.h"
#include "zf_driver_delay.h"

#if (GIMBAL_CONFIG_INSTALLED != 0U)
#error Task 5 owns the lift and requires GIMBAL_CONFIG_INSTALLED to be zero.
#endif

#define TASK_5_STOP_MASK                    (0x7CU)
#define TASK_5_STOP_DELAY_TICKS             \
    (500U / CONTROL_SCHEDULER_PERIOD_MS)
#define TASK_5_STOP_HOLD_TICKS              \
    (2000U / CONTROL_SCHEDULER_PERIOD_MS)
#define TASK_5_RESUME_RETRY_TICKS           (10U)
#define TASK_5_STOPPED_COUNT_LIMIT          (1)

typedef enum
{
    TASK_5_STOP_MONITOR = 0,
    TASK_5_STOP_DELAY_TRACKING,
    TASK_5_STOP_WAIT_WHEELS,
    TASK_5_STOP_HOLDING,
    TASK_5_STOP_WAIT_RESUME,
    TASK_5_STOP_WAIT_PATTERN_CLEAR,
} task_5_stop_state_enum;

typedef enum
{
    TASK_5_START_WAIT_ARM = 0,
    TASK_5_START_CALIBRATING,
    TASK_5_START_WAIT_LINE,
    TASK_5_START_RUNNING,
} task_5_start_state_enum;

#define BALL_GROOVE_TRAVEL_LIMIT_STEPS       (124272)
#define BALL_GROOVE_JOG_RATE_STEPS_S         (777U)
#define BALL_GROOVE_LOOP_PERIOD_MS           (1U)
#define BALL_GROOVE_BASELINE_SAMPLES         (50U)
#define BALL_GROOVE_ACCEL_DEADBAND_ENTER_G   (0.006F)
#define BALL_GROOVE_ACCEL_DEADBAND_EXIT_G    (0.003F)
#define BALL_GROOVE_ACCEL_TO_LIFT_GAIN_MM_PER_G (250.0F)
#define BALL_GROOVE_ACCEL_FILTER_ALPHA       (0.3F)
#define BALL_GROOVE_TRAVEL_LIMIT_MM          (40.0F)
#define BALL_GROOVE_STEPS_PER_REVOLUTION     (6400.0F)
#define BALL_GROOVE_LIFT_MM_PER_REVOLUTION   (2.06F)

#define TASK_5_CURVE_ENTER_DEVIATION          (0.70F)
#define TASK_5_STRAIGHT_EXIT_DEVIATION        (0.25F)
#define TASK_5_STRAIGHT_EXIT_SAMPLES          (15U)

#define TASK_5_ERR_GIMBAL_CFG               (1U)
#define TASK_5_ERR_SCHED_INIT               (2U)
#define TASK_5_ERR_SCHED_START              (3U)
#define TASK_5_ERR_LINE_TRACKER_CFG         (4U)

#define TASK_5_TIME_TEXT_LENGTH              (13U)
#define TASK_5_STAGE_TEXT_LENGTH             (6U)
#define TASK_5_ERROR_TEXT_LENGTH             (8U)

static const line_tracker_config_struct task_5_straight_line_config =
{
    .base_speed_mm_s = 340.0F,
    .pid_kp = 22.0F,
    .pid_ki = 0.0F,
    .pid_kd = 0.0F,
    .pid_integral_limit_mm_s = 91.40F,
    .pid_derivative_filter_alpha = 0.2F,
    .max_target_mm_s = 800.0F,
    .max_correction_mm_s = 90.0F,
    .max_target_accel_mm_s2 = 3655.71F,
    .arc_outer_speed_mm_s = 548.36F,
    .arc_inner_speed_mm_s = 109.67F,
    .pivot_speed_mm_s = 548.36F,
    .lost_debounce_samples = 3U,
    .reacquire_samples = 3U,
    .arc_duration_samples = 100U,
    .search_timeout_samples = 500U,
    .default_search_direction = LINE_TRACKER_DIRECTION_RIGHT,
    .track_all_active_as_center = 1U,
};

/**
 * @brief Return whether all center D3-D7 stop sensors are active.
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
 * @brief Advance the shared D3-D7 delayed stop and restart state machine.
 */
static void task_5_update_stop_state(
    const control_scheduler_status_struct *status,
    task_5_stop_state_enum *stop_state,
    uint32 *stop_trigger_tick,
    uint32 *stop_hold_start_tick,
    uint32 *last_resume_request_tick)
{
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
            *stop_state = TASK_5_STOP_WAIT_WHEELS;
        }
    }
    else if(*stop_state == TASK_5_STOP_WAIT_WHEELS)
    {
        if((status->mode == CONTROL_MODE_MANUAL_ARMED)
            && (task_5_wheels_stopped(status) != 0U))
        {
            *stop_hold_start_tick = status->tick_count;
            *stop_state = TASK_5_STOP_HOLDING;
        }
        else if((status->mode != CONTROL_MODE_LINE_FOLLOW)
            && (status->mode != CONTROL_MODE_MANUAL_ARMED))
        {
            *stop_state = TASK_5_STOP_MONITOR;
        }
    }
    else if(*stop_state == TASK_5_STOP_HOLDING)
    {
        if(status->mode != CONTROL_MODE_MANUAL_ARMED)
        {
            *stop_state = TASK_5_STOP_MONITOR;
        }
        else if(task_5_wheels_stopped(status) == 0U)
        {
            *stop_hold_start_tick = status->tick_count;
        }
        else if((uint32)(status->tick_count - *stop_hold_start_tick)
            >= TASK_5_STOP_HOLD_TICKS)
        {
            if(task_5_gray_allows_line_start(status) != 0U)
            {
                control_scheduler_request_line_start();
                *last_resume_request_tick = status->tick_count;
                *stop_state = TASK_5_STOP_WAIT_RESUME;
            }
        }
    }
    else if(*stop_state == TASK_5_STOP_WAIT_RESUME)
    {
        if(status->mode == CONTROL_MODE_LINE_FOLLOW)
        {
            *stop_state = TASK_5_STOP_WAIT_PATTERN_CLEAR;
        }
        else if(status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            if(task_5_wheels_stopped(status) == 0U)
            {
                *stop_hold_start_tick = status->tick_count;
                *stop_state = TASK_5_STOP_HOLDING;
            }
            else if((task_5_gray_allows_line_start(status) != 0U)
                && ((uint32)(status->tick_count - *last_resume_request_tick)
                    >= TASK_5_RESUME_RETRY_TICKS))
            {
                control_scheduler_request_line_start();
                *last_resume_request_tick = status->tick_count;
            }
        }
        else if((status->mode == CONTROL_MODE_FAULT_LATCHED)
            || (status->mode == CONTROL_MODE_DISARMED))
        {
            *stop_state = TASK_5_STOP_MONITOR;
        }
    }
    else if((status->mode == CONTROL_MODE_LINE_FOLLOW)
        && (task_5_stop_pattern_detected(
            status->gray.active_mask) == 0U))
    {
        *stop_state = TASK_5_STOP_MONITOR;
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
        config.pid_kp = 48.0F;
        config.max_correction_mm_s = 180.0F;
    }
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
 * @brief Return the current Task 5 line-start gate error code.
 */
static uint8 task_5_get_start_error(
    task_5_start_state_enum start_state,
    const control_scheduler_status_struct *status)
{
    if(status->mode == CONTROL_MODE_FAULT_LATCHED)
    {
        return 14U;
    }
    if(start_state == TASK_5_START_WAIT_ARM)
    {
        return ((status->imu_ready == 0U) || (status->imu_fresh == 0U))
            ? 13U : 0U;
    }
    if(start_state == TASK_5_START_RUNNING)
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
        return ((status->imu_ready == 0U) || (status->imu_fresh == 0U))
            ? 13U : 0U;
    }
    if(task_5_gray_allows_line_start(status) == 0U)
    {
        return 11U;
    }
    if(task_5_wheels_stopped(status) == 0U)
    {
        return 12U;
    }
    if((status->imu_ready == 0U) || (status->imu_fresh == 0U))
    {
        return 13U;
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
    uint8 *baseline_ready)
{
    uint8 start_error = task_5_get_start_error(
        *start_state,
        status);

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
            && (status->imu_ready != 0U)
            && (status->imu_fresh != 0U))
        {
            /* A30 arm explicitly starts this 500 ms horizontal-plane sample. */
            *baseline_sum += status->imu_accel_x_g;
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
            *start_state = TASK_5_START_RUNNING;
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
    if(stop_state == TASK_5_STOP_WAIT_WHEELS)
    {
        return 31U;
    }
    if(stop_state == TASK_5_STOP_HOLDING)
    {
        return 32U;
    }
    if(stop_state == TASK_5_STOP_WAIT_RESUME)
    {
        return 33U;
    }
    if(stop_state == TASK_5_STOP_WAIT_PATTERN_CLEAR)
    {
        return 34U;
    }
    return status->mode == CONTROL_MODE_LINE_FOLLOW ? 20U : 21U;
}

/**
 * @brief Limit one acceleration-derived linear lift command to the mechanism.
 */
static int32 task_5_accel_target_steps(float acceleration_g)
{
    float lift_mm = acceleration_g * BALL_GROOVE_ACCEL_TO_LIFT_GAIN_MM_PER_G;
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
 * @brief Run ball-groove acceleration feedforward with line-follow timing.
 */
void test_line_follow_task_5_run(void)
{
    control_scheduler_status_struct status;
    task_5_stop_state_enum stop_state = TASK_5_STOP_MONITOR;
    uint32 stop_trigger_tick = 0U;
    uint32 stop_hold_start_tick = 0U;
    uint32 last_resume_request_tick = 0U;
    uint32 last_control_tick = 0xFFFFFFFFU;
    uint32 start_tick = 0U;
    uint32 last_profile_tick = 0xFFFFFFFFU;
    float accel_baseline_sum = 0.0F;
    float accel_baseline_g = 0.0F;
    float filtered_accel_g = 0.0F;
    uint16 accel_baseline_samples = 0U;
    uint8 accel_baseline_ready = 0U;
    uint8 accel_deadband_active = 0U;
    uint8 filtered_valid = 0U;
    uint8 stopwatch_running = 0U;
    uint8 lift_center_commanded = 0U;
    uint8 curve_profile = 0U;
    uint8 straight_exit_samples = 0U;
    control_mode_enum previous_mode = CONTROL_MODE_BOOT;
    task_5_start_state_enum start_state = TASK_5_START_WAIT_ARM;
    char display_cache[TASK_5_TIME_TEXT_LENGTH] = {0};
    char stage_display_cache[TASK_5_STAGE_TEXT_LENGTH] = {0};
    char error_display_cache[TASK_5_ERROR_TEXT_LENGTH] = {0};
    uint8 display_cache_valid = 0U;
    uint8 stage_display_cache_valid = 0U;
    uint8 error_display_cache_valid = 0U;

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

    control_scheduler_set_imu_bypass(0U);
    control_scheduler_set_imu_acceleration_only(1U);
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
        if((status.mode == CONTROL_MODE_LINE_FOLLOW)
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
            &stop_hold_start_tick,
            &last_resume_request_tick);

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
            if(lift_center_commanded == 0U)
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
                && (status.imu_ready != 0U)
                && (status.imu_fresh != 0U))
            {
                /* +X acceleration commands positive steps: B1 raises the rear. */
                command_accel_g = status.imu_accel_x_g - accel_baseline_g;
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
        }

        task_5_update_start_state(
            &start_state,
            &status,
            &accel_baseline_sum,
            &accel_baseline_g,
            &accel_baseline_samples,
            &accel_baseline_ready);
        if((stopwatch_running == 0U)
            && (start_state == TASK_5_START_RUNNING)
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
                &status),
            status.fault_flags,
            (uint8)(status.mode == CONTROL_MODE_FAULT_LATCHED),
            error_display_cache,
            &error_display_cache_valid);

        system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
    }
}

#endif
