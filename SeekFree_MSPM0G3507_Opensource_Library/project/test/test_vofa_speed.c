/**
 * @file    test_vofa_speed.c
 * @brief   VOFA UART telemetry and target-command loopback test.
 *
 * Commands:
 *   ARM
 *   STOP
 *   TARGET,100,100
 *
 * JustFloat channels:
 *   0 parsed left target, 1 simulated left speed,
 *   2 parsed right target, 3 simulated right speed,
 *   4 valid commands, 5 invalid commands,
 *   6 received lines, 7 armed state.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_VOFA_SPEED)

#include "test_vofa_speed.h"

#include "speed_pid.h"
#include "vofa.h"
#include "zf_driver_delay.h"

#define VOFA_SPEED_TEST_SEND_MS             (20U)
#define VOFA_SPEED_TEST_LEFT_MM_S            (123.0F)
#define VOFA_SPEED_TEST_RIGHT_MM_S           (-123.0F)

/**
 * @brief Send parsed targets with fixed simulated wheel speeds.
 */
static void vofa_speed_test_send(void)
{
    speed_pid_status_struct status = {0};
    vofa_stats_struct stats;

    vofa_get_stats(&stats);
    status.left_target_mm_s = stats.left_target_mm_s;
    status.left_speed_mm_s = VOFA_SPEED_TEST_LEFT_MM_S;
    status.right_target_mm_s = stats.right_target_mm_s;
    status.right_speed_mm_s = VOFA_SPEED_TEST_RIGHT_MM_S;
    status.left_duty = (int16)stats.valid_command_count;
    status.right_duty = (int16)stats.invalid_command_count;
    status.left_count = (int16)stats.received_line_count;
    status.right_count = (int16)stats.armed;
    vofa_send_speed(&status);
}

/**
 * @brief Run the foreground VOFA UART loopback test.
 * @note The test never executes the speed control update, so PWM stays zero.
 */
void test_vofa_speed_run(void)
{
    if (VOFA_UART_CONFIGURED == 0U)
    {
        printf("VOFA UART pins are not configured.\r\n");

        while (true)
        {
        }
    }

    speed_pid_init();

    if (vofa_init() == ZF_FALSE)
    {
        speed_pid_stop();
        printf("VOFA UART configuration is invalid.\r\n");

        while (true)
        {
        }
    }

    vofa_set_stream_enabled(0U);
    printf("VOFA UART test ready. Send ARM, then TARGET,100,100.\r\n");

    while (true)
    {
        vofa_process();
        vofa_speed_test_send();
        system_delay_ms(VOFA_SPEED_TEST_SEND_MS);
    }
}

#endif
