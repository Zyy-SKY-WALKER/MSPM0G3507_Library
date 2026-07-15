/**
 * @file    test_chassis_motion.c
 * @brief   Chassis motion scheduler status display test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_CHASSIS_MOTION)

#include "test_chassis_motion.h"

#include "control_scheduler.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define CHASSIS_MOTION_TEST_DISPLAY_PERIOD_MS    (100U)

/**
 * @brief Clear and display one signed status value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed value.
 * @param digits Magnitude field width.
 */
static void chassis_motion_test_show_int(
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
static void chassis_motion_test_show_uint(
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
 * @brief Draw one chassis motion scheduler status snapshot.
 * @param status Scheduler status.
 */
static void chassis_motion_test_show_status(
    const control_scheduler_status_struct *status)
{
    chassis_motion_test_show_uint(104U, 32U, status->mode, 2U);
    chassis_motion_test_show_uint(
        104U,
        56U,
        status->chassis_motion.command,
        2U);
    chassis_motion_test_show_uint(
        104U,
        80U,
        status->chassis_motion.phase,
        2U);
    chassis_motion_test_show_uint(
        104U,
        104U,
        status->chassis_motion.result,
        2U);
    chassis_motion_test_show_uint(
        104U,
        128U,
        status->chassis_motion.active_profile_id,
        3U);
    chassis_motion_test_show_uint(
        104U,
        152U,
        status->chassis_motion.elapsed_ms,
        6U);
    chassis_motion_test_show_int(
        104U,
        176U,
        (int32)status->chassis_motion.left_target_mm_s,
        4U);
    chassis_motion_test_show_int(
        168U,
        176U,
        (int32)status->chassis_motion.right_target_mm_s,
        4U);
    chassis_motion_test_show_int(
        104U,
        200U,
        (int32)status->chassis_motion.current_center_displacement_mm,
        5U);
    chassis_motion_test_show_int(
        104U,
        224U,
        (int32)status->chassis_motion.current_heading_deg,
        4U);
    chassis_motion_test_show_uint(
        104U,
        248U,
        status->chassis_motion.reversing,
        1U);
}

/**
 * @brief Initialize and run the chassis motion status test.
 * @note Configure and select a PID profile before submitting motion commands.
 */
void test_chassis_motion_run(void)
{
    control_scheduler_status_struct status;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "CHASSIS MOTION");
    ili9341_show_string(8U, 32U, "MODE    :");
    ili9341_show_string(8U, 56U, "COMMAND :");
    ili9341_show_string(8U, 80U, "PHASE   :");
    ili9341_show_string(8U, 104U, "RESULT  :");
    ili9341_show_string(8U, 128U, "PROFILE :");
    ili9341_show_string(8U, 152U, "TIME MS :");
    ili9341_show_string(8U, 176U, "TGT L/R :");
    ili9341_show_string(8U, 200U, "DIST MM :");
    ili9341_show_string(8U, 224U, "YAW DEG :");
    ili9341_show_string(8U, 248U, "REVERSING:");
    ili9341_show_string(8U, 280U, "A30 ARM A31 STOP");

    control_scheduler_init();
    control_scheduler_start();

    while (true)
    {
        control_scheduler_process_foreground();
        control_scheduler_get_status(&status);
        chassis_motion_test_show_status(&status);
        system_delay_ms(CHASSIS_MOTION_TEST_DISPLAY_PERIOD_MS);
    }
}

#endif
