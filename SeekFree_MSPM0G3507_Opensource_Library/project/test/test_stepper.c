/**
 * @file    test_stepper.c
 * @brief   Dual-axis STEP/DIR manual jog and zero test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_STEPPER)

#include "test_stepper.h"

#include <stdio.h>

#include "gimbal_stepper.h"
#include "ml_stepper_debug.h"
#include "zf_driver_delay.h"

#define TEST_STEPPER_LOOP_PERIOD_MS       (1U)
#define TEST_STEPPER_STATUS_PERIOD_MS     (500U)

static void test_stepper_print_status(
    const gimbal_stepper_status_struct *status)
{
    const gimbal_stepper_axis_status_struct *yaw =
        &status->axis[GIMBAL_STEPPER_AXIS_YAW];
    const gimbal_stepper_axis_status_struct *pitch =
        &status->axis[GIMBAL_STEPPER_AXIS_PITCH];

    printf(
        "Axis=%s Yaw=%ld>%ld/%ldpps/Z%u "
        "Pitch=%ld>%ld/%ldpps/Z%u Stop=%u\r\n",
        status->selected_axis == GIMBAL_STEPPER_AXIS_YAW
            ? "YAW" : "PITCH",
        (long)yaw->position_steps,
        (long)yaw->target_position_steps,
        (long)yaw->current_rate_steps_s,
        (unsigned int)yaw->zero_valid,
        (long)pitch->position_steps,
        (long)pitch->target_position_steps,
        (long)pitch->current_rate_steps_s,
        (unsigned int)pitch->zero_valid,
        (unsigned int)status->stop_latched);
}

void test_stepper_run(void)
{
    gimbal_stepper_status_struct status;
    uint16 status_elapsed_ms = 0U;

    gimbal_stepper_init();
    ml_stepper_debug_init();
    printf("Dual-axis STEP/DIR test ready.\r\n");
    printf("A30 short: select; A30 long: zero selected axis.\r\n");
    printf("B0/B1: negative/positive jog; A31: immediate stop.\r\n");

    while(1)
    {
        uint16 elapsed_ms = gimbal_stepper_service();

        status_elapsed_ms += elapsed_ms;
        if(status_elapsed_ms >= TEST_STEPPER_STATUS_PERIOD_MS)
        {
            gimbal_stepper_get_status(&status);
            test_stepper_print_status(&status);
            ml_stepper_debug_update(
                status.selected_axis,
                status.axis[GIMBAL_STEPPER_AXIS_YAW].position_steps,
                status.axis[GIMBAL_STEPPER_AXIS_YAW]
                    .current_rate_steps_s,
                status.axis[GIMBAL_STEPPER_AXIS_YAW].zero_valid,
                status.axis[GIMBAL_STEPPER_AXIS_PITCH].position_steps,
                status.axis[GIMBAL_STEPPER_AXIS_PITCH]
                    .current_rate_steps_s,
                status.axis[GIMBAL_STEPPER_AXIS_PITCH].zero_valid,
                status.stop_latched,
                status.negative_key_pressed,
                status.positive_key_pressed,
                status.select_key_pressed);
            status_elapsed_ms %= TEST_STEPPER_STATUS_PERIOD_MS;
        }

        system_delay_ms(TEST_STEPPER_LOOP_PERIOD_MS);
    }
}

#endif
