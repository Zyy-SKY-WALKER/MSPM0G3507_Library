/**
 * @file    test_line_follow_real.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Real line-follow test with nonblocking VOFA speed telemetry.
 */

#include "test_config.h"

#if ((TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL) \
    || (TEST_MODE == TEST_MODE_LINE_FOLLOW_BALL_ACCEL_OPEN_LOOP) \
    || (TEST_MODE == TEST_MODE_LINE_FOLLOW_TASK_2))

#include "control_scheduler.h"
#include "test_line_follow_real.h"

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)
#include <string.h>
#include "vofa.h"
#endif

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_BALL_ACCEL_OPEN_LOOP)
#include "zf_driver_delay.h"
#endif

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_TASK_2)
#include "ml_oled.h"
#endif

#define LINE_FOLLOW_REAL_VOFA_PERIOD_TICKS  (2U)
#define LINE_FOLLOW_REAL_STOP_MASK          (0x7CU)
#define LINE_FOLLOW_REAL_STOP_DELAY_TICKS   \
    (500U / CONTROL_SCHEDULER_PERIOD_MS)
#define LINE_FOLLOW_REAL_STOP_HOLD_TICKS    \
    (2000U / CONTROL_SCHEDULER_PERIOD_MS)
#define LINE_FOLLOW_REAL_RESUME_RETRY_TICKS (10U)
#define LINE_FOLLOW_REAL_STOPPED_COUNT_LIMIT (1)

typedef enum
{
    LINE_FOLLOW_REAL_STOP_MONITOR = 0,
    LINE_FOLLOW_REAL_STOP_DELAY_TRACKING,
    LINE_FOLLOW_REAL_STOP_WAIT_WHEELS,
    LINE_FOLLOW_REAL_STOP_HOLDING,
    LINE_FOLLOW_REAL_STOP_WAIT_RESUME,
    LINE_FOLLOW_REAL_STOP_WAIT_PATTERN_CLEAR,
} line_follow_real_stop_state_enum;

/**
 * @brief Return whether all center D3-D7 stop sensors are active.
 */
static uint8 line_follow_real_stop_pattern_detected(uint8 active_mask)
{
    return (uint8)(
        (active_mask & LINE_FOLLOW_REAL_STOP_MASK)
            == LINE_FOLLOW_REAL_STOP_MASK);
}

/**
 * @brief Return whether both wheel encoder intervals indicate a stopped vehicle.
 */
static uint8 line_follow_real_wheels_stopped(
    const control_scheduler_status_struct *status)
{
    return (uint8)(
        (status->left_count >= -LINE_FOLLOW_REAL_STOPPED_COUNT_LIMIT)
        && (status->left_count <= LINE_FOLLOW_REAL_STOPPED_COUNT_LIMIT)
        && (status->right_count >= -LINE_FOLLOW_REAL_STOPPED_COUNT_LIMIT)
        && (status->right_count <= LINE_FOLLOW_REAL_STOPPED_COUNT_LIMIT));
}

/**
 * @brief Advance the shared D3-D7 delayed stop and restart state machine.
 */
static void line_follow_real_update_stop_state(
    const control_scheduler_status_struct *status,
    line_follow_real_stop_state_enum *stop_state,
    uint32 *stop_trigger_tick,
    uint32 *stop_hold_start_tick,
    uint32 *last_resume_request_tick)
{
    if(*stop_state == LINE_FOLLOW_REAL_STOP_MONITOR)
    {
        if((status->mode == CONTROL_MODE_LINE_FOLLOW)
            && (line_follow_real_stop_pattern_detected(
                status->gray.active_mask) != 0U))
        {
            *stop_trigger_tick = status->tick_count;
            *stop_state = LINE_FOLLOW_REAL_STOP_DELAY_TRACKING;
        }
    }
    else if(*stop_state == LINE_FOLLOW_REAL_STOP_DELAY_TRACKING)
    {
        if(status->mode != CONTROL_MODE_LINE_FOLLOW)
        {
            *stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
        }
        else if((uint32)(status->tick_count - *stop_trigger_tick)
            >= LINE_FOLLOW_REAL_STOP_DELAY_TICKS)
        {
            control_scheduler_request_line_stop();
            *stop_state = LINE_FOLLOW_REAL_STOP_WAIT_WHEELS;
        }
    }
    else if(*stop_state == LINE_FOLLOW_REAL_STOP_WAIT_WHEELS)
    {
        if((status->mode == CONTROL_MODE_MANUAL_ARMED)
            && (line_follow_real_wheels_stopped(status) != 0U))
        {
            *stop_hold_start_tick = status->tick_count;
            *stop_state = LINE_FOLLOW_REAL_STOP_HOLDING;
        }
        else if((status->mode != CONTROL_MODE_LINE_FOLLOW)
            && (status->mode != CONTROL_MODE_MANUAL_ARMED))
        {
            *stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
        }
    }
    else if(*stop_state == LINE_FOLLOW_REAL_STOP_HOLDING)
    {
        if(status->mode != CONTROL_MODE_MANUAL_ARMED)
        {
            *stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
        }
        else if(line_follow_real_wheels_stopped(status) == 0U)
        {
            *stop_hold_start_tick = status->tick_count;
        }
        else if((uint32)(status->tick_count - *stop_hold_start_tick)
            >= LINE_FOLLOW_REAL_STOP_HOLD_TICKS)
        {
            if(status->gray.status == GRAY_SENSOR_STATUS_VALID)
            {
                control_scheduler_request_line_start();
                *last_resume_request_tick = status->tick_count;
                *stop_state = LINE_FOLLOW_REAL_STOP_WAIT_RESUME;
            }
        }
    }
    else if(*stop_state == LINE_FOLLOW_REAL_STOP_WAIT_RESUME)
    {
        if(status->mode == CONTROL_MODE_LINE_FOLLOW)
        {
            *stop_state = LINE_FOLLOW_REAL_STOP_WAIT_PATTERN_CLEAR;
        }
        else if(status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            if(line_follow_real_wheels_stopped(status) == 0U)
            {
                *stop_hold_start_tick = status->tick_count;
                *stop_state = LINE_FOLLOW_REAL_STOP_HOLDING;
            }
            else if((status->gray.status == GRAY_SENSOR_STATUS_VALID)
                && ((uint32)(status->tick_count - *last_resume_request_tick)
                    >= LINE_FOLLOW_REAL_RESUME_RETRY_TICKS))
            {
                control_scheduler_request_line_start();
                *last_resume_request_tick = status->tick_count;
            }
        }
        else if((status->mode == CONTROL_MODE_FAULT_LATCHED)
            || (status->mode == CONTROL_MODE_DISARMED))
        {
            *stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
        }
    }
    else if((status->mode == CONTROL_MODE_LINE_FOLLOW)
        && (line_follow_real_stop_pattern_detected(
            status->gray.active_mask) == 0U))
    {
        *stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
    }
}

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)

/**
 * @brief Send wheel targets and measured speeds through VOFA JustFloat.
 */
static void line_follow_real_send_vofa(
    const control_scheduler_status_struct *status)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float channels[4];
    uint8 frame[20];

    channels[0] = status->speed.left_target_mm_s;
    channels[1] = status->speed.right_target_mm_s;
    channels[2] = status->speed.left_speed_mm_s;
    channels[3] = status->speed.right_speed_mm_s;
    memcpy(frame, channels, sizeof(channels));
    memcpy(&frame[sizeof(channels)], tail, sizeof(tail));
    uart_write_buffer(VOFA_UART_INDEX, frame, sizeof(frame));
}

/**
 * @brief Initialize and run real line following with VOFA telemetry.
 */
void test_line_follow_real_run(void)
{
    control_scheduler_status_struct status;
    line_follow_real_stop_state_enum stop_state =
        LINE_FOLLOW_REAL_STOP_MONITOR;
    uint32 last_vofa_tick = 0U;
    uint32 stop_trigger_tick = 0U;
    uint32 stop_hold_start_tick = 0U;
    uint32 last_resume_request_tick = 0U;

    control_scheduler_set_imu_bypass(1U);
    if((vofa_init_tx_only() == ZF_FALSE)
        || (control_scheduler_init() == ZF_FALSE)
        || (control_scheduler_start() == ZF_FALSE))
    {
        while(true)
        {
        }
    }

    while (true)
    {
        control_scheduler_process_foreground();
        control_scheduler_get_status(&status);
        line_follow_real_update_stop_state(
            &status,
            &stop_state,
            &stop_trigger_tick,
            &stop_hold_start_tick,
            &last_resume_request_tick);
        if((uint32)(status.tick_count - last_vofa_tick)
            >= LINE_FOLLOW_REAL_VOFA_PERIOD_TICKS)
        {
            line_follow_real_send_vofa(&status);
            last_vofa_tick = status.tick_count;
        }
    }
}

#endif

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_BALL_ACCEL_OPEN_LOOP)

#define BALL_GROOVE_TRAVEL_LIMIT_STEPS       (124272)
#define BALL_GROOVE_JOG_RATE_STEPS_S         (777U)
#define BALL_GROOVE_LOOP_PERIOD_MS           (1U)
#define BALL_GROOVE_BASELINE_SAMPLES         (50U)
#define BALL_GROOVE_ACCEL_DEADBAND_ENTER_G   (0.006F)
#define BALL_GROOVE_ACCEL_DEADBAND_EXIT_G    (0.003F)
#define BALL_GROOVE_LENGTH_MM                (250.0F)
#define BALL_GROOVE_TRAVEL_LIMIT_MM          (40.0F)
#define BALL_GROOVE_STEPS_PER_REVOLUTION     (6400.0F)
#define BALL_GROOVE_LIFT_MM_PER_REVOLUTION   (2.06F)

/**
 * @brief Limit one acceleration-derived linear lift command to the mechanism.
 */
static int32 line_follow_ball_accel_target_steps(float acceleration_g)
{
    float lift_mm = acceleration_g * BALL_GROOVE_LENGTH_MM;
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
static float line_follow_ball_accel_apply_deadband(
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
 * @brief Run real MPU line following with acceleration-only groove feedforward.
 */
void test_line_follow_ball_accel_open_loop_run(void)
{
    control_scheduler_status_struct status;
    line_follow_real_stop_state_enum stop_state =
        LINE_FOLLOW_REAL_STOP_MONITOR;
    uint32 stop_trigger_tick = 0U;
    uint32 stop_hold_start_tick = 0U;
    uint32 last_resume_request_tick = 0U;
    uint32 last_control_tick = 0xFFFFFFFFU;
    float accel_baseline_sum = 0.0F;
    float accel_baseline_g = 0.0F;
    uint16 accel_baseline_samples = 0U;
    uint8 accel_baseline_ready = 0U;
    uint8 accel_deadband_active = 0U;

    gimbal_stepper_init();
    if(gimbal_stepper_configure_single_axis(
            GIMBAL_STEPPER_AXIS_YAW,
            -BALL_GROOVE_TRAVEL_LIMIT_STEPS,
            BALL_GROOVE_TRAVEL_LIMIT_STEPS,
            BALL_GROOVE_JOG_RATE_STEPS_S) == 0U)
    {
        while(true)
        {
        }
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
    if((control_scheduler_init() == ZF_FALSE)
        || (control_scheduler_start() == ZF_FALSE))
    {
        while(true)
        {
        }
    }

    while(true)
    {
        float command_accel_g;

        control_scheduler_process_foreground();
        (void)gimbal_stepper_service();
        control_scheduler_get_status(&status);
        if(status.tick_count == last_control_tick)
        {
            system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
            continue;
        }
        last_control_tick = status.tick_count;
        line_follow_real_update_stop_state(
            &status,
            &stop_state,
            &stop_trigger_tick,
            &stop_hold_start_tick,
            &last_resume_request_tick);

        if(status.mode != CONTROL_MODE_LINE_FOLLOW)
        {
            accel_deadband_active = 0U;
            if((accel_baseline_ready == 0U)
                && (status.imu_ready != 0U)
                && (status.imu_fresh != 0U)
                && (line_follow_real_wheels_stopped(&status) != 0U))
            {
                accel_baseline_sum += status.imu_accel_x_g;
                accel_baseline_samples++;
                if(accel_baseline_samples >= BALL_GROOVE_BASELINE_SAMPLES)
                {
                    accel_baseline_g = accel_baseline_sum
                        / (float)accel_baseline_samples;
                    accel_baseline_ready = 1U;
                }
            }
            else if(accel_baseline_ready == 0U)
            {
                accel_baseline_sum = 0.0F;
                accel_baseline_samples = 0U;
            }
            system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
            continue;
        }

        if((accel_baseline_ready != 0U)
            && (status.imu_ready != 0U)
            && (status.imu_fresh != 0U))
        {
            /* +X acceleration commands positive steps: B1 raises the rear. */
            command_accel_g = status.imu_accel_x_g - accel_baseline_g;
            command_accel_g = line_follow_ball_accel_apply_deadband(
                command_accel_g,
                &accel_deadband_active);
            (void)gimbal_stepper_set_axis_absolute_target_steps(
                GIMBAL_STEPPER_AXIS_YAW,
                line_follow_ball_accel_target_steps(command_accel_g));
        }
        system_delay_ms(BALL_GROOVE_LOOP_PERIOD_MS);
    }
}

#endif

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_TASK_2)

#define TASK_2_FINISH_MASK                    (0x78U)
#define TASK_2_START_CLEAR_SAMPLES            (3U)
#define TASK_2_STOP_CONFIRM_SAMPLES           (3U)
#define TASK_2_TIME_TEXT_LENGTH               (13U)
#define TASK_2_MASK_TEXT_LENGTH               (14U)
#define TASK_2_START_TEXT_LENGTH              (8U)

typedef enum
{
    TASK_2_WAIT_ARM = 0,
    TASK_2_WAIT_LINE_START,
    TASK_2_RUNNING,
    TASK_2_WAIT_STOP,
    TASK_2_FINISHED,
} task_2_state_enum;

static const line_tracker_config_struct task_2_line_config =
{
    .base_speed_mm_s = 360.0F,
    .pid_kp = 29.0F,
    .pid_ki = 0.3F,
    .pid_kd = 0.0F,
    .pid_integral_limit_mm_s = 91.40F,
    .pid_derivative_filter_alpha = 0.2F,
    .max_target_mm_s = 800.0F,
    .max_correction_mm_s = 146.23F,
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
 * @brief Build the fixed-width OLED first-line time text.
 */
static void task_2_build_time_text(
    char text[TASK_2_TIME_TEXT_LENGTH],
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
static void task_2_render_time(
    uint8 started,
    uint32 elapsed_tenths,
    char cache[TASK_2_TIME_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[TASK_2_TIME_TEXT_LENGTH];
    uint8 index;

    task_2_build_time_text(text, started, elapsed_tenths);
    for(index = 0U; index < TASK_2_TIME_TEXT_LENGTH; index++)
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
 * @brief Build the OLED third-line D1-D8 binary gray-sensor status text.
 */
static void task_2_build_mask_text(
    char text[TASK_2_MASK_TEXT_LENGTH],
    uint8 active_mask)
{
    uint8 index;

    text[0] = 'D';
    text[1] = '1';
    text[2] = '-';
    text[3] = 'D';
    text[4] = '8';
    text[5] = ':';
    for(index = 0U; index < 8U; index++)
    {
        text[6U + index] = (active_mask & (uint8)(1U << index)) != 0U
            ? '1' : '0';
    }
}

/**
 * @brief Dirty-refresh only changed characters in OLED line three.
 */
static void task_2_render_mask(
    uint8 active_mask,
    char cache[TASK_2_MASK_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[TASK_2_MASK_TEXT_LENGTH];
    uint8 index;

    task_2_build_mask_text(text, active_mask);
    for(index = 0U; index < TASK_2_MASK_TEXT_LENGTH; index++)
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
 * @brief Return the startup gate diagnostic code for the current task state.
 *
 * 00 running or finished, 01 waiting for A30 arm, 11 gray mask/status,
 * 12 wheel motion, 13 IMU source, 14 fault latch, 15 disarmed unexpectedly,
 * and 16 means all gates are ready and a start request is being retried.
 */
static uint8 task_2_get_start_code(
    task_2_state_enum state,
    const control_scheduler_status_struct *status)
{
    if((state == TASK_2_RUNNING)
        || (state == TASK_2_WAIT_STOP)
        || (state == TASK_2_FINISHED))
    {
        return 0U;
    }
    if(status->mode == CONTROL_MODE_FAULT_LATCHED)
    {
        return 14U;
    }
    if(state == TASK_2_WAIT_ARM)
    {
        return status->mode == CONTROL_MODE_MANUAL_ARMED ? 16U : 1U;
    }
    if(status->mode != CONTROL_MODE_MANUAL_ARMED)
    {
        return 15U;
    }
    if((status->gray.status != GRAY_SENSOR_STATUS_VALID)
        && !((status->gray.status == GRAY_SENSOR_STATUS_ALL_ACTIVE)
            && (line_tracker_tracks_all_active_as_center() != 0U)))
    {
        return 11U;
    }
    if(line_follow_real_wheels_stopped(status) == 0U)
    {
        return 12U;
    }
    if((status->imu_ready == 0U) || (status->imu_fresh == 0U))
    {
        return 13U;
    }
    return 16U;
}

/**
 * @brief Dirty-refresh the startup diagnostic code on OLED line four.
 */
static void task_2_render_start_code(
    uint8 code,
    uint32 fault_flags,
    uint8 fault_latched,
    char cache[TASK_2_START_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[TASK_2_START_TEXT_LENGTH];
    uint8 index;

    if(fault_latched != 0U)
    {
        static const char hex_digits[] = "0123456789ABCDEF";

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
        text[0] = 'S';
        text[1] = 'T';
        text[2] = 'A';
        text[3] = 'R';
        text[4] = 'T';
        text[5] = ':';
        text[6] = (char)('0' + ((code / 10U) % 10U));
        text[7] = (char)('0' + (code % 10U));
    }
    for(index = 0U; index < TASK_2_START_TEXT_LENGTH; index++)
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
 * @brief Run one clockwise 6141 mm line-follow lap from A back to A.
 */
void test_line_follow_task_2_run(void)
{
    control_scheduler_status_struct status;
    task_2_state_enum state = TASK_2_WAIT_ARM;
    char display_cache[TASK_2_TIME_TEXT_LENGTH] = {0};
    char mask_display_cache[TASK_2_MASK_TEXT_LENGTH] = {0};
    char start_display_cache[TASK_2_START_TEXT_LENGTH] = {0};
    uint32 start_tick = 0U;
    uint32 first_stopped_tick = 0U;
    uint32 final_elapsed_tenths = 0U;
    uint8 display_cache_valid = 0U;
    uint8 mask_display_cache_valid = 0U;
    uint8 start_display_cache_valid = 0U;
    uint8 start_marker_cleared = 0U;
    uint8 start_clear_samples = 0U;
    uint8 stopped_samples = 0U;

    control_scheduler_set_imu_bypass(1U);
    control_scheduler_set_imu_acceleration_only(0U);
    if((ml_oled_init() == false)
        || (control_scheduler_init() == ZF_FALSE)
        || (line_tracker_set_config(&task_2_line_config) == ZF_FALSE)
        || (control_scheduler_start() == ZF_FALSE))
    {
        while(true)
        {
        }
    }
    task_2_render_time(0U, 0U, display_cache, &display_cache_valid);

    while(true)
    {
        uint32 elapsed_tenths = final_elapsed_tenths;

        control_scheduler_process_foreground();
        control_scheduler_get_status(&status);
        task_2_render_mask(
            status.gray.active_mask,
            mask_display_cache,
            &mask_display_cache_valid);
        task_2_render_start_code(
            task_2_get_start_code(state, &status),
            status.fault_flags,
            (uint8)(status.mode == CONTROL_MODE_FAULT_LATCHED),
            start_display_cache,
            &start_display_cache_valid);

        if(state == TASK_2_WAIT_ARM)
        {
            if(status.mode == CONTROL_MODE_MANUAL_ARMED)
            {
                control_scheduler_request_line_start();
                state = TASK_2_WAIT_LINE_START;
            }
        }
        else if(state == TASK_2_WAIT_LINE_START)
        {
            if(status.mode == CONTROL_MODE_LINE_FOLLOW)
            {
                start_tick = status.tick_count;
                start_marker_cleared = 0U;
                start_clear_samples = 0U;
                state = TASK_2_RUNNING;
            }
            else
            {
                if(task_2_get_start_code(state, &status) == 16U)
                {
                    control_scheduler_request_line_start();
                }
                else if(status.mode == CONTROL_MODE_DISARMED)
                {
                    state = TASK_2_WAIT_ARM;
                }
            }
        }
        else if(state == TASK_2_RUNNING)
        {
            uint8 finish_active = (uint8)((status.gray.active_mask
                & TASK_2_FINISH_MASK) == TASK_2_FINISH_MASK);

            elapsed_tenths = (status.tick_count - start_tick) / 10U;
            if(start_marker_cleared == 0U)
            {
                if(finish_active == 0U)
                {
                    if(start_clear_samples < TASK_2_START_CLEAR_SAMPLES)
                    {
                        start_clear_samples++;
                    }
                    if(start_clear_samples >= TASK_2_START_CLEAR_SAMPLES)
                    {
                        start_marker_cleared = 1U;
                    }
                }
                else
                {
                    start_clear_samples = 0U;
                }
            }
            else if(finish_active != 0U)
            {
                control_scheduler_request_line_stop();
                stopped_samples = 0U;
                state = TASK_2_WAIT_STOP;
            }
        }
        else if(state == TASK_2_WAIT_STOP)
        {
            elapsed_tenths = (status.tick_count - start_tick) / 10U;
            if((status.mode == CONTROL_MODE_MANUAL_ARMED)
                && (line_follow_real_wheels_stopped(&status) != 0U))
            {
                if(stopped_samples == 0U)
                {
                    first_stopped_tick = status.tick_count;
                }
                if(stopped_samples < TASK_2_STOP_CONFIRM_SAMPLES)
                {
                    stopped_samples++;
                }
                if(stopped_samples >= TASK_2_STOP_CONFIRM_SAMPLES)
                {
                    final_elapsed_tenths =
                        (first_stopped_tick - start_tick) / 10U;
                    state = TASK_2_FINISHED;
                }
            }
            else
            {
                stopped_samples = 0U;
            }
        }

        if((state == TASK_2_RUNNING) || (state == TASK_2_WAIT_STOP))
        {
            task_2_render_time(
                1U,
                elapsed_tenths,
                display_cache,
                &display_cache_valid);
        }
        else if(state == TASK_2_FINISHED)
        {
            task_2_render_time(
                1U,
                final_elapsed_tenths,
                display_cache,
                &display_cache_valid);
        }
        else
        {
            task_2_render_time(0U, 0U, display_cache, &display_cache_valid);
        }
    }
}

#endif

#endif
