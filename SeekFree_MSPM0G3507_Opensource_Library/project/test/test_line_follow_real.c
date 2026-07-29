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
    uint32 last_vofa_tick = 0U;

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
        if((uint32)(status.tick_count - last_vofa_tick)
            >= LINE_FOLLOW_REAL_VOFA_PERIOD_TICKS)
        {
            line_follow_real_send_vofa(&status);
            last_vofa_tick = status.tick_count;
        }
    }
}

#endif
