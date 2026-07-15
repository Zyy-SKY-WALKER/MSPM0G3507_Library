/**
 * @file    test_servo.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Smoothstep 0-to-270-degree dual servo sweep test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_SERVO)

#include "servo.h"
#include "test_servo.h"
#include "zf_common_typedef.h"
#include "zf_driver_delay.h"

#define SERVO_TEST_STARTUP_HOLD_MS                 (500U)
#define SERVO_TEST_UPDATE_PERIOD_MS                (10U)
#define SERVO_TEST_FULL_SWEEP_DURATION_MS          (5400U)
#define SERVO_TEST_STARTUP_SWEEP_DURATION_MS       (2700U)
#define SERVO_TEST_ENDPOINT_HOLD_MS                (50U)
#define SERVO_TEST_FULL_SWEEP_TICK_COUNT           \
    (SERVO_TEST_FULL_SWEEP_DURATION_MS / SERVO_TEST_UPDATE_PERIOD_MS)
#define SERVO_TEST_STARTUP_SWEEP_TICK_COUNT        \
    (SERVO_TEST_STARTUP_SWEEP_DURATION_MS / SERVO_TEST_UPDATE_PERIOD_MS)

/**
 * @brief Set both servos and stop the test if the request is invalid.
 * @param angle_tenth_deg Requested angle from 0 to 2700 tenths of a degree.
 */
static void test_servo_set_angle(uint16 angle_tenth_deg)
{
    if (servo_set_all_angle_tenth_deg(angle_tenth_deg) == 0U)
    {
        while (1)
        {
        }
    }
}

/**
 * @brief Calculate a cubic Smoothstep angle for one trajectory time point.
 * @param start_angle_tenth_deg Start angle in tenths of a degree.
 * @param end_angle_tenth_deg End angle in tenths of a degree.
 * @param elapsed_tick Elapsed 10 ms update count.
 * @param total_tick_count Total 10 ms update count.
 * @return Interpolated angle in tenths of a degree.
 */
static uint16 test_servo_smoothstep_angle(
    uint16 start_angle_tenth_deg,
    uint16 end_angle_tenth_deg,
    uint16 elapsed_tick,
    uint16 total_tick_count)
{
    uint64 elapsed;
    uint64 total;
    uint64 smoothstep_numerator;
    uint64 smoothstep_denominator;
    uint64 angle_delta;
    uint64 angle_offset;

    if ((total_tick_count == 0U)
        || (elapsed_tick >= total_tick_count))
    {
        return end_angle_tenth_deg;
    }

    elapsed = elapsed_tick;
    total = total_tick_count;
    smoothstep_numerator = elapsed * elapsed
        * ((3U * total) - (2U * elapsed));
    smoothstep_denominator = total * total * total;

    if (start_angle_tenth_deg <= end_angle_tenth_deg)
    {
        angle_delta = end_angle_tenth_deg - start_angle_tenth_deg;
        angle_offset = ((angle_delta * smoothstep_numerator)
            + (smoothstep_denominator / 2U)) / smoothstep_denominator;
        return start_angle_tenth_deg + (uint16)angle_offset;
    }

    angle_delta = start_angle_tenth_deg - end_angle_tenth_deg;
    angle_offset = ((angle_delta * smoothstep_numerator)
        + (smoothstep_denominator / 2U)) / smoothstep_denominator;
    return start_angle_tenth_deg - (uint16)angle_offset;
}

/**
 * @brief Move both servos with cubic Smoothstep interpolation.
 * @param start_angle_tenth_deg Start angle in tenths of a degree.
 * @param end_angle_tenth_deg End angle in tenths of a degree.
 * @param total_tick_count Total 10 ms update count.
 */
static void test_servo_sweep(
    uint16 start_angle_tenth_deg,
    uint16 end_angle_tenth_deg,
    uint16 total_tick_count)
{
    uint16 elapsed_tick;
    uint16 angle_tenth_deg;

    for (elapsed_tick = 0U;
        elapsed_tick < total_tick_count;
        elapsed_tick++)
    {
        angle_tenth_deg = test_servo_smoothstep_angle(
            start_angle_tenth_deg,
            end_angle_tenth_deg,
            elapsed_tick,
            total_tick_count);
        test_servo_set_angle(angle_tenth_deg);
        system_delay_ms(SERVO_TEST_UPDATE_PERIOD_MS);
    }

    test_servo_set_angle(end_angle_tenth_deg);
    system_delay_ms(SERVO_TEST_ENDPOINT_HOLD_MS);
}

/**
 * @brief Run the synchronized dual 270-degree servo sweep test.
 * @note Power the servos externally and connect all grounds together.
 */
void test_servo_run(void)
{
    servo_init();
    system_delay_ms(SERVO_TEST_STARTUP_HOLD_MS);
    test_servo_sweep(
        SERVO_MID_ANGLE_TENTH_DEG,
        0U,
        SERVO_TEST_STARTUP_SWEEP_TICK_COUNT);

    while (1)
    {
        test_servo_sweep(
            0U,
            SERVO_MAX_ANGLE_TENTH_DEG,
            SERVO_TEST_FULL_SWEEP_TICK_COUNT);
        test_servo_sweep(
            SERVO_MAX_ANGLE_TENTH_DEG,
            0U,
            SERVO_TEST_FULL_SWEEP_TICK_COUNT);
    }
}

#endif
