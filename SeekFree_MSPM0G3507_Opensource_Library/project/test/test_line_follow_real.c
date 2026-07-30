/**
 * @file    test_line_follow_real.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Real line-follow test with nonblocking VOFA speed telemetry.
 */

#include "test_config.h"

#if ((TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL) \
    || (TEST_MODE == TEST_MODE_LINE_FOLLOW_BALL_ACCEL_OPEN_LOOP))

#include "control_scheduler.h"
#include "test_line_follow_real.h"

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)
#include <string.h>
#include "vofa.h"
#endif

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_BALL_ACCEL_OPEN_LOOP)
#include "zf_driver_delay.h"
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

#endif
