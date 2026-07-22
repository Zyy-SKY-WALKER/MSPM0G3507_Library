/**
 * @file    test_control_scheduler.c
 * @brief   Unified control scheduler TFT status and key test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_CONTROL_SCHEDULER)

#include "test_control_scheduler.h"

#include "control_scheduler.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define CONTROL_TEST_DISPLAY_TIME_MS    (100U)
#define CONTROL_TEST_LOOP_TIME_MS       (1U)

/** @brief Conservative line profile used to validate interrupt load. */
static const line_tracker_config_struct control_test_line_config =
{
    .base_speed_mm_s = {100.0F, 90.0F, 80.0F, 70.0F, 60.0F},
    .pid_kp = {30.0F, 38.0F, 43.0F, 55.0F, 68.0F},
    .pid_ki = 0.0F,
    .pid_kd = 0.0F,
    .pid_integral_limit_mm_s = 50.0F,
    .pid_derivative_filter_alpha = 0.2F,
    .max_target_mm_s = 200.0F,
    .max_correction_mm_s = 100.0F,
    .arc_outer_speed_mm_s = 100.0F,
    .arc_inner_speed_mm_s = -25.0F,
    .pivot_speed_mm_s = 80.0F,
    .lost_debounce_samples = 3U,
    .reacquire_samples = 3U,
    .arc_duration_samples = 300U,
    .search_timeout_samples = 500U,
    .default_search_direction = LINE_TRACKER_DIRECTION_RIGHT,
};

/**
 * @brief Display the operation expected in the current scheduler state.
 * @param status Scheduler status.
 */
static void control_test_show_instruction(
    const control_scheduler_status_struct *status)
{
    const char *instruction;

    if (status->gimbal_calibrated == 0U)
    {
        instruction = "CAL: HOLD A30 / SHORT SEL";
    }
    else if (status->mode == CONTROL_MODE_DISARMED)
    {
        instruction = "READY: SHORT A30 TO ARM";
    }
    else if (status->mode == CONTROL_MODE_MANUAL_ARMED)
    {
        instruction = "ARMED: B0 START LINE";
    }
    else if (status->mode == CONTROL_MODE_LINE_FOLLOW)
    {
        instruction = "LINE RUN: A31 STOP";
    }
    else if (status->mode == CONTROL_MODE_FAULT_LATCHED)
    {
        instruction = "FAULT: RELEASE A31,HOLD B1";
    }
    else
    {
        instruction = "CONTROL SCHEDULER";
    }

    ili9341_fill_rect(
        0U,
        8U,
        239U,
        23U,
        ILI9341_COLOR_BLACK);
    ili9341_show_string(0U, 8U, instruction);
}

/**
 * @brief Display an eight-bit mask with D1 at the left side.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param mask Mask to display.
 */
static void control_test_show_mask(uint16 x, uint16 y, uint8 mask)
{
    uint8 index;

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        ili9341_show_char(
            (uint16)(x + ((uint16)index * 8U)),
            y,
            (mask & (uint8)(1U << index)) != 0U ? '1' : '0');
    }
}

/**
 * @brief Clear and display one signed status value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed value.
 * @param digits Magnitude field width.
 */
static void control_test_show_int(
    uint16 x,
    uint16 y,
    int32 value,
    uint8 digits)
{
    uint16 field_width = (uint16)((digits + 1U) * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, value, digits);
}

/**
 * @brief Clear and display one unsigned status value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Unsigned value.
 * @param digits Field width.
 */
static void control_test_show_uint(
    uint16 x,
    uint16 y,
    uint32 value,
    uint8 digits)
{
    uint16 field_width = (uint16)(digits * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_uint(x, y, value, digits);
}

/**
 * @brief Draw the latest coherent scheduler status snapshot.
 * @param status Scheduler status.
 */
static void control_test_show_status(
    const control_scheduler_status_struct *status)
{
    control_test_show_instruction(status);
    control_test_show_uint(96U, 32U, status->mode, 2U);
    control_test_show_uint(96U, 56U, status->fault_flags, 4U);
    control_test_show_uint(96U, 80U, status->tick_count, 8U);
    control_test_show_mask(96U, 104U, status->gray.active_mask);
    control_test_show_int(
        96U,
        128U,
        (int32)(status->gray.deviation * 10.0F),
        3U);
    control_test_show_int(96U, 152U, status->left_count, 5U);
    control_test_show_int(160U, 152U, status->right_count, 5U);
    control_test_show_int(
        96U,
        176U,
        (int32)status->speed.left_target_mm_s,
        4U);
    control_test_show_int(
        160U,
        176U,
        (int32)status->speed.right_target_mm_s,
        4U);
    control_test_show_int(96U, 200U, status->speed.left_duty, 4U);
    control_test_show_int(160U, 200U, status->speed.right_duty, 4U);
    control_test_show_uint(96U, 224U, status->line_status.state, 2U);
    control_test_show_uint(160U, 224U, status->imu_fresh, 1U);
    control_test_show_int(
        96U,
        248U,
        (int32)status->imu_yaw_deg,
        4U);
    control_test_show_uint(160U, 248U, status->imu_age_ticks, 4U);
    control_test_show_uint(
        96U,
        280U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_YAW].zero_valid,
        1U);
    control_test_show_uint(
        112U,
        280U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_PITCH].zero_valid,
        1U);
    control_test_show_uint(144U, 280U, status->gimbal_calibrated, 1U);
    control_test_show_uint(
        160U,
        280U,
        status->gimbal_feedforward_valid,
        1U);
    control_test_show_int(
        96U,
        296U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_YAW]
            .target_position_steps,
        5U);
    control_test_show_int(
        160U,
        296U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_PITCH]
            .target_position_steps,
        5U);
}

/**
 * @brief Initialize and run the scheduler status and key test.
 */
void test_control_scheduler_run(void)
{
    control_scheduler_status_struct status;
    uint16 display_elapsed_ms = 0U;

    gimbal_stepper_laser_init();
    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "CONTROL SCHEDULER");
    ili9341_show_string(8U, 32U, "MODE   :");
    ili9341_show_string(8U, 56U, "FAULT  :");
    ili9341_show_string(8U, 80U, "TICK   :");
    ili9341_show_string(8U, 104U, "GRAY   :");
    ili9341_show_string(8U, 128U, "DEVx10 :");
    ili9341_show_string(8U, 152U, "ENC L/R:");
    ili9341_show_string(8U, 176U, "TGT L/R:");
    ili9341_show_string(8U, 200U, "PWM L/R:");
    ili9341_show_string(8U, 224U, "LINE/IM:");
    ili9341_show_string(8U, 248U, "YAW/AGE:");
    ili9341_show_string(8U, 280U, "GIM YP/CF:");
    ili9341_show_string(8U, 296U, "GIM TGT:");

    if ((control_scheduler_init() == ZF_FALSE)
        || (line_tracker_set_config(&control_test_line_config) == ZF_FALSE)
        || (control_scheduler_start() == ZF_FALSE))
    {
        ili9341_fill_rect(
            0U,
            8U,
            239U,
            23U,
            ILI9341_COLOR_BLACK);
        ili9341_show_string(0U, 8U, "CONTROL INIT FAILED");
        while (true)
        {
            system_delay_ms(CONTROL_TEST_DISPLAY_TIME_MS);
        }
    }

    while (true)
    {
        control_scheduler_process_foreground();
        display_elapsed_ms += CONTROL_TEST_LOOP_TIME_MS;
        if (display_elapsed_ms >= CONTROL_TEST_DISPLAY_TIME_MS)
        {
            control_scheduler_get_status(&status);
            control_test_show_status(&status);
            display_elapsed_ms = 0U;
        }
        system_delay_ms(CONTROL_TEST_LOOP_TIME_MS);
    }
}

#endif
