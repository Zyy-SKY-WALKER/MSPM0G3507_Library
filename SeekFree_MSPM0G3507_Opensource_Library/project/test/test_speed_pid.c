/**
 * @file    test_speed_pid.c
 * @brief   PIT-driven dual-wheel speed PID verification test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_SPEED_PID)

#include "test_speed_pid.h"

#include "my_lib_encoder.h"
#include "speed_pid.h"
#include "vofa.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"

#define SPEED_PID_TEST_PIT               (PIT_TIM_G12)
#define SPEED_PID_TEST_ARM_TIME_MS       (3000U)
#define SPEED_PID_TEST_IDLE_TIME_MS      (1U)

typedef struct
{
    float left_target_mm_s;
    float right_target_mm_s;
    uint32 duration_ms;
} speed_pid_test_step_struct;

static const speed_pid_test_step_struct speed_pid_test_steps[] =
{
    {300.0F, 300.0F, 3000U},
    {550.0F, 550.0F, 3000U},
    {800.0F, 800.0F, 3000U},
    {0.0F, 0.0F, 1000U},
};

static volatile uint32 speed_pid_test_elapsed_ms;
static volatile uint8 speed_pid_test_vofa_due;

/**
 * @brief Execute one speed control update from the 10 ms PIT interrupt.
 * @param event PIT callback event value.
 * @param user_data Optional callback context.
 */
static void speed_pid_test_pit_callback(uint32 event, void *user_data)
{
    int16 left_count;
    int16 right_count;

    (void)event;
    (void)user_data;

    my_encoder_get_delta(&left_count, &right_count);
    speed_pid_update_10ms(left_count, right_count);
    speed_pid_test_elapsed_ms += SPEED_PID_SAMPLE_PERIOD_MS;
    speed_pid_test_vofa_due = 1U;
}

/**
 * @brief Send the latest speed status when one VOFA frame is due.
 */
static void speed_pid_test_process_vofa(void)
{
    speed_pid_status_struct status;
    uint32 primask;
    uint8 send_due;

    primask = interrupt_global_disable();
    send_due = speed_pid_test_vofa_due;
    speed_pid_test_vofa_due = 0U;
    interrupt_global_enable(primask);

    if (send_due != 0U)
    {
        speed_pid_get_status(&status);
        vofa_send_right_speed(&status);
    }
}

/**
 * @brief Start one step atomically against the 10 ms control callback.
 * @param step Test sequence step.
 * @return Step start time from the 10 ms control clock.
 */
static uint32 speed_pid_test_start_step(
    const speed_pid_test_step_struct *step)
{
    uint32 primask;
    uint32 start_ms;

    primask = interrupt_global_disable();
    start_ms = speed_pid_test_elapsed_ms;
    speed_pid_set_target(
        step->left_target_mm_s,
        step->right_target_mm_s);
    speed_pid_test_vofa_due = 0U;
    interrupt_global_enable(primask);

    return start_ms;
}

/**
 * @brief Run foreground telemetry for a fixed test interval.
 * @param start_ms Interval start time from the 10 ms control clock.
 * @param duration_ms Foreground service interval in milliseconds.
 */
static void speed_pid_test_run_foreground(
    uint32 start_ms,
    uint32 duration_ms)
{
    while ((uint32)(speed_pid_test_elapsed_ms - start_ms) < duration_ms)
    {
        speed_pid_test_process_vofa();
        system_delay_ms(SPEED_PID_TEST_IDLE_TIME_MS);
    }
}

/**
 * @brief Hold one target pair while servicing foreground telemetry.
 * @param step Test sequence step.
 */
static void speed_pid_test_run_step(
    const speed_pid_test_step_struct *step)
{
    uint32 start_ms = speed_pid_test_start_step(step);

    speed_pid_test_run_foreground(start_ms, step->duration_ms);
}

/**
 * @brief Run the cyclic dual-wheel speed PID sequence.
 * @note Raise both wheels before selecting TEST_MODE_SPEED_PID.
 */
void test_speed_pid_run(void)
{
    uint32 index;

    my_encoder_init();
    speed_pid_init();
    if (vofa_init_tx_only() == ZF_FALSE)
    {
        speed_pid_stop();
        while (true)
        {
        }
    }
    speed_pid_test_elapsed_ms = 0U;
    speed_pid_test_vofa_due = 0U;
    pit_ms_init(
        SPEED_PID_TEST_PIT,
        SPEED_PID_SAMPLE_PERIOD_MS,
        speed_pid_test_pit_callback,
        NULL);

    speed_pid_test_run_foreground(0U, SPEED_PID_TEST_ARM_TIME_MS);

    while (true)
    {
        for (index = 0U;
            index < (sizeof(speed_pid_test_steps)
                / sizeof(speed_pid_test_steps[0]));
            index++)
        {
            speed_pid_test_run_step(&speed_pid_test_steps[index]);
        }
    }
}

#endif
