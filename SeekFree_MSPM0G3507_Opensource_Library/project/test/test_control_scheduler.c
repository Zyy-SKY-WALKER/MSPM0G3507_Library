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
}

/**
 * @brief Initialize and run the scheduler status and key test.
 */
void test_control_scheduler_run(void)
{
    control_scheduler_status_struct status;

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
    ili9341_show_string(8U, 280U, "A30 ARM B0 LINE");
    ili9341_show_string(8U, 296U, "A31 STOP B1 CLEAR");

    control_scheduler_init();
    control_scheduler_start();

    while (true)
    {
        control_scheduler_process_foreground();
        control_scheduler_get_status(&status);
        control_test_show_status(&status);
        system_delay_ms(CONTROL_TEST_DISPLAY_TIME_MS);
    }
}

#endif
