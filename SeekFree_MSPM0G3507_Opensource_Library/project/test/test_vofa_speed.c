/**
 * @file    test_vofa_speed.c
 * @brief   VOFA speed PID telemetry and safe command test.
 *
 * Commands:
 *   ARM
 *   STOP
 *   TARGET,100,100
 *   KP,L,1.0
 *   KI,R,0.2
 *   KD,B,0.0
 *   STREAM,0
 *   RATE,50
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_VOFA_SPEED)

#include "test_vofa_speed.h"

#include "my_lib_encoder.h"
#include "speed_pid.h"
#include "vofa.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"

#define VOFA_SPEED_TEST_PIT           (PIT_TIM_G12)
#define VOFA_SPEED_TEST_IDLE_MS       (1U)

/**
 * @brief Update encoder sampling, speed PID and VOFA timeout timing.
 * @param event PIT callback event value.
 * @param user_data Optional callback context.
 */
static void vofa_speed_test_pit_callback(uint32 event, void *user_data)
{
    int16 left_count;
    int16 right_count;

    (void)event;
    (void)user_data;

    my_encoder_get_delta(&left_count, &right_count);
    speed_pid_update_10ms(left_count, right_count);
    vofa_tick_10ms();
}

/**
 * @brief Run the foreground VOFA speed tuning test.
 * @note Raise both wheels before enabling nonzero TARGET commands.
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

    my_encoder_init();
    speed_pid_init();

    if (vofa_init() == ZF_FALSE)
    {
        speed_pid_stop();
        printf("VOFA UART configuration is invalid.\r\n");

        while (true)
        {
        }
    }

    pit_ms_init(
        VOFA_SPEED_TEST_PIT,
        SPEED_PID_SAMPLE_PERIOD_MS,
        vofa_speed_test_pit_callback,
        NULL);

    printf("VOFA speed test ready. Send ARM before TARGET.\r\n");

    while (true)
    {
        vofa_process();
        system_delay_ms(VOFA_SPEED_TEST_IDLE_MS);
    }
}

#endif
