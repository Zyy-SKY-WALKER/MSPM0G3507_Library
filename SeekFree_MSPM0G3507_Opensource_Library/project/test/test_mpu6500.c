/**
 * @file    test_mpu6500.c
 * @brief   MPU6500 attitude and VOFA angle-output test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_MPU6500)

#include "test_mpu6500.h"

#include <string.h>

#include "my_lib_mpu6500.h"
#include "my_lib_mpu6500_attitude.h"
#include "vofa.h"
#include "zf_driver_pit.h"

#define MPU6500_TEST_TIMER               (PIT_TIM_G12)
#define MPU6500_TEST_SAMPLE_PERIOD_MS    (10U)
#define MPU6500_TEST_RATE_PERIOD_MS      (1000U)
#define MPU6500_TEST_VOFA_PERIOD_MS      (20U)

static volatile uint32 mpu6500_test_time_ms;

/**
 * @brief Advance the standalone test clock by one millisecond.
 */
static void mpu6500_test_timer_callback(uint32 event, void *context)
{
    (void)event;
    (void)context;
    mpu6500_test_time_ms++;
}

/**
 * @brief Send roll, pitch, yaw, and successful solve rate through VOFA.
 */
static void mpu6500_test_send_attitude(
    const mpu6500_attitude_data_struct *attitude_data,
    float solve_rate_hz)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float channels[4];
    uint8 frame[20];

    channels[0] = attitude_data->roll_deg;
    channels[1] = attitude_data->pitch_deg;
    channels[2] = attitude_data->yaw_deg;
    channels[3] = solve_rate_hz;
    memcpy(frame, channels, sizeof(channels));
    memcpy(&frame[sizeof(channels)], tail, sizeof(tail));

    uart_write_buffer(VOFA_UART_INDEX, frame, sizeof(frame));
}

/**
 * @brief Run MPU6500 attitude estimation and VOFA angle telemetry.
 * @note Keep the vehicle stationary until the 1000-sample calibration finishes.
 */
void test_mpu6500_run(void)
{
    mpu6500_data_struct sensor_data;
    mpu6500_attitude_data_struct attitude_data;
    uint32 last_sample_attempt_ms = 0U;
    uint32 last_successful_sample_ms = 0U;
    uint32 last_rate_ms = 0U;
    uint32 last_rate_solve_count = 0U;
    uint32 last_vofa_ms = 0U;
    uint32 successful_solve_count = 0U;
    float solve_rate_hz = 0.0F;
    uint8 successful_sample_seen = 0U;

    if((vofa_init_tx_only() == ZF_FALSE) || (mpu6500_init() != 0U))
    {
        while(true)
        {
        }
    }

    mpu6500_attitude_init();
    mpu6500_test_time_ms = 0U;
    pit_ms_init(
        MPU6500_TEST_TIMER,
        1U,
        mpu6500_test_timer_callback,
        NULL);

    while(true)
    {
        uint32 now_ms = mpu6500_test_time_ms;

        if((uint32)(now_ms - last_sample_attempt_ms)
            >= MPU6500_TEST_SAMPLE_PERIOD_MS)
        {
            if(mpu6500_read(&sensor_data) == 0U)
            {
                uint32 dt_ms = MPU6500_TEST_SAMPLE_PERIOD_MS;

                if(successful_sample_seen != 0U)
                {
                    dt_ms = now_ms - last_successful_sample_ms;
                }
                if(mpu6500_attitude_update(&sensor_data, dt_ms) == 0U)
                {
                    successful_solve_count++;
                    last_successful_sample_ms = now_ms;
                    successful_sample_seen = 1U;
                }
            }
            last_sample_attempt_ms = now_ms;
        }

        if((uint32)(now_ms - last_rate_ms) >= MPU6500_TEST_RATE_PERIOD_MS)
        {
            uint32 elapsed_ms = now_ms - last_rate_ms;

            solve_rate_hz = ((float)(successful_solve_count
                - last_rate_solve_count) * 1000.0F) / (float)elapsed_ms;
            last_rate_solve_count = successful_solve_count;
            last_rate_ms = now_ms;
        }

        if((uint32)(now_ms - last_vofa_ms) >= MPU6500_TEST_VOFA_PERIOD_MS)
        {
            if((mpu6500_attitude_get_data(&attitude_data) == 0U)
                && (attitude_data.ready != 0U))
            {
                mpu6500_test_send_attitude(&attitude_data, solve_rate_hz);
            }
            last_vofa_ms = now_ms;
        }
    }
}

#endif
