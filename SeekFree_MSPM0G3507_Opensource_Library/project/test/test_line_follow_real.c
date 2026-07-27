/**
 * @file    test_line_follow_real.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Real grayscale, line PID, and wheel-speed PID integration test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)

#include "control_scheduler.h"
#include "my_lib_ili9341.h"
#include "test_line_follow_real.h"
#include "zf_driver_delay.h"

#define LINE_FOLLOW_REAL_LOOP_TIME_MS       (1U)
#define LINE_FOLLOW_REAL_DISPLAY_TICKS      (20U)

/**
 * @brief Round one float for integer TFT display.
 * @param value Value to round.
 * @return Rounded signed value.
 */
static int32 line_follow_real_round(float value)
{
    if (value >= 0.0F)
    {
        return (int32)(value + 0.5F);
    }

    return (int32)(value - 0.5F);
}

/**
 * @brief Display one eight-bit mask with D1 at the left side.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param mask Input mask.
 */
static void line_follow_real_show_mask(uint16 x, uint16 y, uint8 mask)
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
 * @brief Clear and display one signed value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed value.
 * @param digits Magnitude width.
 */
static void line_follow_real_show_int(
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
 * @brief Clear and display one unsigned value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Unsigned value.
 * @param digits Field width.
 */
static void line_follow_real_show_uint(
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
 * @brief Draw one coherent real line-follow status snapshot.
 * @param status Scheduler status snapshot.
 */
static void line_follow_real_show_status(
    const control_scheduler_status_struct *status)
{
    line_follow_real_show_uint(96U, 32U, status->mode, 2U);
    line_follow_real_show_uint(96U, 56U, status->fault_flags, 4U);
    line_follow_real_show_mask(96U, 80U, status->gray.active_mask);
    line_follow_real_show_int(
        96U,
        104U,
        line_follow_real_round(status->gray.deviation * 10.0F),
        3U);
    line_follow_real_show_int(
        160U,
        104U,
        line_follow_real_round(status->line_status.correction_mm_s),
        4U);
    line_follow_real_show_int(
        96U,
        128U,
        line_follow_real_round(status->speed.left_target_mm_s),
        4U);
    line_follow_real_show_int(
        160U,
        128U,
        line_follow_real_round(status->speed.right_target_mm_s),
        4U);
    line_follow_real_show_int(
        96U,
        152U,
        line_follow_real_round(status->speed.left_speed_mm_s),
        4U);
    line_follow_real_show_int(
        160U,
        152U,
        line_follow_real_round(status->speed.right_speed_mm_s),
        4U);
    line_follow_real_show_int(96U, 176U, status->speed.left_duty, 4U);
    line_follow_real_show_int(160U, 176U, status->speed.right_duty, 4U);
    line_follow_real_show_uint(96U, 200U, status->line_status.state, 2U);
    line_follow_real_show_uint(160U, 200U, status->line_status.speed_band, 1U);
    line_follow_real_show_int(
        96U,
        224U,
        line_follow_real_round(status->line_status.pid_integral_mm_s),
        4U);
    line_follow_real_show_int(
        160U,
        224U,
        line_follow_real_round(
            status->line_status.pid_filtered_derivative),
        4U);
    line_follow_real_show_int(96U, 248U, status->left_count, 5U);
    line_follow_real_show_int(160U, 248U, status->right_count, 5U);
    line_follow_real_show_uint(
        96U,
        280U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_YAW].zero_valid,
        1U);
    line_follow_real_show_uint(
        112U,
        280U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_PITCH].zero_valid,
        1U);
    line_follow_real_show_uint(
        144U,
        280U,
        status->gimbal_calibrated,
        1U);
    line_follow_real_show_uint(
        160U,
        280U,
        status->gimbal_feedforward_valid,
        1U);
    line_follow_real_show_int(
        96U,
        296U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_YAW]
            .target_position_steps,
        5U);
    line_follow_real_show_int(
        160U,
        296U,
        status->gimbal.axis[GIMBAL_STEPPER_AXIS_PITCH]
            .target_position_steps,
        5U);
}

/**
 * @brief Initialize and run the real closed-loop line-follow test.
 */
void test_line_follow_real_run(void)
{
    control_scheduler_status_struct status;
    uint32 last_display_tick = 0U;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "REAL LINE FOLLOW");
    ili9341_show_string(8U, 32U, "MODE   :");
    ili9341_show_string(8U, 56U, "FAULT  :");
    ili9341_show_string(8U, 80U, "GRAY   :");
    ili9341_show_string(8U, 104U, "DEV/COR:");
    ili9341_show_string(8U, 128U, "TGT L/R:");
    ili9341_show_string(8U, 152U, "SPD L/R:");
    ili9341_show_string(8U, 176U, "PWM L/R:");
    ili9341_show_string(8U, 200U, "STATE/B:");
    ili9341_show_string(8U, 224U, "I/D    :");
    ili9341_show_string(8U, 248U, "ENC L/R:");
    ili9341_show_string(8U, 280U, "GIM YP/CF:");
    ili9341_show_string(8U, 296U, "GIM TGT:");

    control_scheduler_init();
    control_scheduler_start();

    while (true)
    {
        uint32 current_tick;

        control_scheduler_process_foreground();
        current_tick = control_scheduler_get_tick_count();
        if ((uint32)(current_tick - last_display_tick)
            >= LINE_FOLLOW_REAL_DISPLAY_TICKS)
        {
            control_scheduler_get_status(&status);
            line_follow_real_show_status(&status);
            last_display_tick = status.tick_count;
        }
        system_delay_ms(LINE_FOLLOW_REAL_LOOP_TIME_MS);
    }
}

#endif
