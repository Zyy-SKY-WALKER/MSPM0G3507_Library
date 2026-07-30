/**
 * @file    test_line_follow_real.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Real line-follow test with nonblocking VOFA speed telemetry.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)

#include "control_scheduler.h"
#include "test_line_follow_real.h"

#include <string.h>

#include "vofa.h"

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
        if(stop_state == LINE_FOLLOW_REAL_STOP_MONITOR)
        {
            if((status.mode == CONTROL_MODE_LINE_FOLLOW)
                && (line_follow_real_stop_pattern_detected(
                    status.gray.active_mask) != 0U))
            {
                stop_trigger_tick = status.tick_count;
                stop_state = LINE_FOLLOW_REAL_STOP_DELAY_TRACKING;
            }
        }
        else if(stop_state == LINE_FOLLOW_REAL_STOP_DELAY_TRACKING)
        {
            if(status.mode != CONTROL_MODE_LINE_FOLLOW)
            {
                stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
            }
            else if((uint32)(status.tick_count - stop_trigger_tick)
                >= LINE_FOLLOW_REAL_STOP_DELAY_TICKS)
            {
                control_scheduler_request_line_stop();
                stop_state = LINE_FOLLOW_REAL_STOP_WAIT_WHEELS;
            }
        }
        else if(stop_state == LINE_FOLLOW_REAL_STOP_WAIT_WHEELS)
        {
            if((status.mode == CONTROL_MODE_MANUAL_ARMED)
                && (line_follow_real_wheels_stopped(&status) != 0U))
            {
                stop_hold_start_tick = status.tick_count;
                stop_state = LINE_FOLLOW_REAL_STOP_HOLDING;
            }
            else if((status.mode != CONTROL_MODE_LINE_FOLLOW)
                && (status.mode != CONTROL_MODE_MANUAL_ARMED))
            {
                stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
            }
        }
        else if(stop_state == LINE_FOLLOW_REAL_STOP_HOLDING)
        {
            if(status.mode != CONTROL_MODE_MANUAL_ARMED)
            {
                stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
            }
            else if(line_follow_real_wheels_stopped(&status) == 0U)
            {
                stop_hold_start_tick = status.tick_count;
            }
            else if((uint32)(status.tick_count - stop_hold_start_tick)
                >= LINE_FOLLOW_REAL_STOP_HOLD_TICKS)
            {
                if(status.gray.status == GRAY_SENSOR_STATUS_VALID)
                {
                    control_scheduler_request_line_start();
                    last_resume_request_tick = status.tick_count;
                    stop_state = LINE_FOLLOW_REAL_STOP_WAIT_RESUME;
                }
            }
        }
        else if(stop_state == LINE_FOLLOW_REAL_STOP_WAIT_RESUME)
        {
            if(status.mode == CONTROL_MODE_LINE_FOLLOW)
            {
                stop_state = LINE_FOLLOW_REAL_STOP_WAIT_PATTERN_CLEAR;
            }
            else if(status.mode == CONTROL_MODE_MANUAL_ARMED)
            {
                if(line_follow_real_wheels_stopped(&status) == 0U)
                {
                    stop_hold_start_tick = status.tick_count;
                    stop_state = LINE_FOLLOW_REAL_STOP_HOLDING;
                }
                else if((status.gray.status == GRAY_SENSOR_STATUS_VALID)
                    && ((uint32)(status.tick_count - last_resume_request_tick)
                        >= LINE_FOLLOW_REAL_RESUME_RETRY_TICKS))
                {
                    control_scheduler_request_line_start();
                    last_resume_request_tick = status.tick_count;
                }
            }
            else if((status.mode == CONTROL_MODE_FAULT_LATCHED)
                || (status.mode == CONTROL_MODE_DISARMED))
            {
                stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
            }
        }
        else
        {
            if((status.mode == CONTROL_MODE_LINE_FOLLOW)
                && (line_follow_real_stop_pattern_detected(
                    status.gray.active_mask) == 0U))
            {
                stop_state = LINE_FOLLOW_REAL_STOP_MONITOR;
            }
        }
        if((uint32)(status.tick_count - last_vofa_tick)
            >= LINE_FOLLOW_REAL_VOFA_PERIOD_TICKS)
        {
            line_follow_real_send_vofa(&status);
            last_vofa_tick = status.tick_count;
        }
    }
}

#endif
