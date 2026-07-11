/**
 * @file    test_motor.c
 * @brief   Low-speed TB6612FNG motor direction test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_MOTOR)

#include "test_motor.h"

#include "motor.h"
#include "zf_driver_delay.h"

#define MOTOR_TEST_DUTY              (2000)
#define MOTOR_TEST_RUN_TIME_MS       (1000U)
#define MOTOR_TEST_STOP_TIME_MS      (500U)

/**
 * @brief Stop both motors for the configured safety interval.
 */
static void test_motor_stop_wait(void)
{
    motor_stop();
    system_delay_ms(MOTOR_TEST_STOP_TIME_MS);
}

/**
 * @brief Run one motor in both directions at low speed.
 * @param is_left Select left motor when nonzero; otherwise select right.
 */
static void test_motor_run_channel(uint8 is_left)
{
    if(is_left != 0U)
    {
        motor_left_set_duty(MOTOR_TEST_DUTY);
    }
    else
    {
        motor_right_set_duty(MOTOR_TEST_DUTY);
    }
    system_delay_ms(MOTOR_TEST_RUN_TIME_MS);
    test_motor_stop_wait();

    if(is_left != 0U)
    {
        motor_left_set_duty(-MOTOR_TEST_DUTY);
    }
    else
    {
        motor_right_set_duty(-MOTOR_TEST_DUTY);
    }
    system_delay_ms(MOTOR_TEST_RUN_TIME_MS);
    test_motor_stop_wait();
}

/**
 * @brief Run the TB6612FNG low-speed direction test once.
 * @note Keep both wheels raised before running this test.
 */
void test_motor_run(void)
{
    motor_init();
    test_motor_stop_wait();

    test_motor_run_channel(1U);
    test_motor_run_channel(0U);

    motor_left_set_duty(MOTOR_TEST_DUTY);
    motor_right_set_duty(MOTOR_TEST_DUTY);
    system_delay_ms(MOTOR_TEST_RUN_TIME_MS);
    test_motor_stop_wait();

    motor_left_set_duty(-MOTOR_TEST_DUTY);
    motor_right_set_duty(-MOTOR_TEST_DUTY);
    system_delay_ms(MOTOR_TEST_RUN_TIME_MS);
    motor_stop();
}

#endif
