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
#include "zf_driver_uart.h"

#define TEST_STEPPER_LOOP_PERIOD_MS       (1U)
#define TEST_STEPPER_STATUS_PERIOD_MS     (500U)
#define TEST_STEPPER_UART_INDEX           (UART_1)
#define TEST_STEPPER_UART_BAUDRATE        (115200U)
#define TEST_STEPPER_UART_TX_PIN          (UART1_TX_B6)
#define TEST_STEPPER_UART_RX_PIN          (UART1_RX_B7)
#define TEST_STEPPER_TEXT_BUFFER_SIZE     (192U)
#define TEST_STEPPER_ARRIVAL_HOLD_MS      (200U)
#define TEST_STEPPER_POSITION_TOLERANCE   (2)

/**
 * @brief Write one zero-terminated test message through UART1 B6.
 */
static void test_stepper_uart_write(const char *message)
{
    if(message != NULL)
    {
        uart_write_string(TEST_STEPPER_UART_INDEX, message);
    }
}

/**
 * @brief Round one float to a signed scaled integer.
 */
static int32 test_stepper_scale_float(float value, float scale)
{
    float scaled = value * scale;

    if(scaled >= 0.0F)
    {
        return (int32)(scaled + 0.5F);
    }
    return (int32)(scaled - 0.5F);
}

/**
 * @brief Return the magnitude of one signed step error.
 */
static int32 test_stepper_step_error_magnitude(
    int32 position,
    int32 target)
{
    int32 error = target - position;

    return error < 0 ? -error : error;
}

/**
 * @brief Check that both software positions have settled at their targets.
 */
static uint8 test_stepper_axes_arrived(
    const gimbal_stepper_status_struct *status)
{
    const gimbal_stepper_axis_status_struct *yaw =
        &status->axis[GIMBAL_STEPPER_AXIS_YAW];
    const gimbal_stepper_axis_status_struct *pitch =
        &status->axis[GIMBAL_STEPPER_AXIS_PITCH];

    return (uint8)(
        (test_stepper_step_error_magnitude(
            yaw->position_steps,
            yaw->target_position_steps) <= TEST_STEPPER_POSITION_TOLERANCE)
        && (test_stepper_step_error_magnitude(
            pitch->position_steps,
            pitch->target_position_steps) <= TEST_STEPPER_POSITION_TOLERANCE)
        && (yaw->current_rate_steps_s == 0)
        && (pitch->current_rate_steps_s == 0));
}

static void test_stepper_print_status(
    const gimbal_stepper_status_struct *status)
{
    char buffer[TEST_STEPPER_TEXT_BUFFER_SIZE];
    const gimbal_stepper_axis_status_struct *yaw =
        &status->axis[GIMBAL_STEPPER_AXIS_YAW];
    const gimbal_stepper_axis_status_struct *pitch =
        &status->axis[GIMBAL_STEPPER_AXIS_PITCH];

    (void)snprintf(
        buffer,
        sizeof(buffer),
        "Axis=%s Yaw=%ld>%ld/%ldpps/Z%u "
        "Pitch=%ld>%ld/%ldpps/Z%u Stop=%u Laser=%u\r\n",
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
        (unsigned int)status->stop_latched,
        (unsigned int)status->laser_enabled);
    test_stepper_uart_write(buffer);
}

/**
 * @brief Print the latest nominal feedforward result.
 */
static void test_stepper_print_feedforward(
    const gimbal_feedforward_solution_struct *solution)
{
    char buffer[TEST_STEPPER_TEXT_BUFFER_SIZE];

    (void)snprintf(
        buffer,
        sizeof(buffer),
        "FF=(%ld,%ld,%ld)mm Y=%ldcdeg P=%ldcdeg "
        "R=%ldmdeg V=%u\r\n",
        (long)test_stepper_scale_float(solution->target_x_mm, 1.0F),
        (long)test_stepper_scale_float(solution->target_y_mm, 1.0F),
        (long)test_stepper_scale_float(solution->target_z_mm, 1.0F),
        (long)test_stepper_scale_float(solution->yaw_deg, 100.0F),
        (long)test_stepper_scale_float(
            solution->pitch_deg,
            100.0F),
        (long)test_stepper_scale_float(solution->residual_deg, 1000.0F),
        (unsigned int)solution->valid);
    test_stepper_uart_write(buffer);
}

void test_stepper_run(void)
{
    gimbal_stepper_status_struct status;
    gimbal_feedforward_pose_struct pose;
    gimbal_feedforward_solution_struct solution;
    uint16 status_elapsed_ms = 0U;
    uint16 arrival_hold_ms = 0U;
    uint8 feedforward_started = 0U;

    pose.x_mm = 0.0F;
    pose.y_mm = 0.0F;
    pose.z_mm = 0.0F;
    pose.roll_deg = 0.0F;
    pose.pitch_deg = 0.0F;
    pose.heading_rad = 0.0F;
    pose.valid = 1U;

    gimbal_stepper_laser_init();
    uart_init(
        TEST_STEPPER_UART_INDEX,
        TEST_STEPPER_UART_BAUDRATE,
        TEST_STEPPER_UART_TX_PIN,
        TEST_STEPPER_UART_RX_PIN);
    test_stepper_uart_write("Stepper UART1 B6 output ready.\r\n");
    gimbal_stepper_init();
    gimbal_stepper_set_log_callback(test_stepper_uart_write);
    ml_stepper_debug_init();
    test_stepper_uart_write("Dual-axis STEP/DIR test ready.\r\n");
    test_stepper_uart_write(
        "A30 short: select; A30 long: zero selected axis.\r\n");
    test_stepper_uart_write(
        "Yaw zero: minimum stop, laser points right.\r\n");
    test_stepper_uart_write(
        "Pitch zero: midpoint, laser points horizontally forward.\r\n");
    test_stepper_uart_write(
        "B0/B1: negative/positive jog; A31: immediate stop.\r\n");
    test_stepper_uart_write(
        "After zeroing, feedforward moves automatically; laser turns on "
        "after 200 ms settled.\r\n");

    while(1)
    {
        uint16 elapsed_ms = gimbal_stepper_service();
        uint8 manual_active;

        gimbal_stepper_get_status(&status);
        manual_active = (uint8)(
            (status.negative_key_pressed != 0U)
            || (status.positive_key_pressed != 0U)
            || (status.select_key_pressed != 0U));

        if((status.stop_latched != 0U)
            || (status.axis[GIMBAL_STEPPER_AXIS_YAW].zero_valid == 0U)
            || (status.axis[GIMBAL_STEPPER_AXIS_PITCH].zero_valid == 0U)
            || (manual_active != 0U))
        {
            arrival_hold_ms = 0U;
            feedforward_started = 0U;
        }

        if((feedforward_started == 0U)
            && (status.relative_ready != 0U)
            && (manual_active == 0U)
            && (gimbal_stepper_compute_feedforward(
                &pose,
                &solution) != 0U))
        {
            test_stepper_print_feedforward(&solution);
            if(gimbal_stepper_update_feedforward(&pose) != 0U)
            {
                feedforward_started = 1U;
                arrival_hold_ms = 0U;
                (void)gimbal_stepper_set_laser(0U);
                test_stepper_uart_write("Feedforward motion started.\r\n");
            }
        }

        if((feedforward_started != 0U)
            && (status.stop_latched == 0U)
            && (manual_active == 0U)
            && (test_stepper_axes_arrived(&status) != 0U))
        {
            if(arrival_hold_ms
                < TEST_STEPPER_ARRIVAL_HOLD_MS)
            {
                arrival_hold_ms += elapsed_ms;
            }
            if((arrival_hold_ms >= TEST_STEPPER_ARRIVAL_HOLD_MS)
                && (status.laser_enabled == 0U))
            {
                if(gimbal_stepper_set_laser(1U) != 0U)
                {
                    test_stepper_uart_write(
                        "Feedforward arrived; laser on.\r\n");
                }
            }
        }
        else
        {
            arrival_hold_ms = 0U;
            if(status.laser_enabled != 0U)
            {
                (void)gimbal_stepper_set_laser(0U);
                test_stepper_uart_write("Laser off: target moving.\r\n");
            }
        }

        status_elapsed_ms += elapsed_ms;
        if(status_elapsed_ms >= TEST_STEPPER_STATUS_PERIOD_MS)
        {
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
