/**
 * @file    test_encoder.c
 * @brief   Dual motor encoder count display test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ENCODER)

#include "test_encoder.h"

#include "my_lib_encoder.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#define ENCODER_TEST_UPDATE_TIME_MS    (100U)

/**
 * @brief Display one GPIO phase level as H or L.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param level GPIO direction level.
 */
static void test_encoder_show_level(uint16 x, uint16 y, uint8 level)
{
    if (level == GPIO_HIGH)
    {
        ili9341_show_char(x, y, 'H');
    }
    else
    {
        ili9341_show_char(x, y, 'L');
    }
}

/**
 * @brief Clear and display one signed encoder count.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param count Signed encoder count.
 */
static void test_encoder_show_count(
    uint16 x,
    uint16 y,
    int32 count,
    uint8 digits)
{
    uint16 field_width = (uint16)((digits + 1U) * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, count, digits);
}

/**
 * @brief Initialize the display and show live dual encoder counts.
 * @note Rotate each raised wheel by hand. Record count signs when the vehicle
 *       moves forward before integrating PID or odometry code.
 */
void test_encoder_run(void)
{
    int32 left_total = 0;
    int32 right_total = 0;
    int16 left_step;
    int16 right_step;

    ili9341_init();
    my_encoder_init();

    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "ENCODER TEST");
    ili9341_show_string(8U, 40U, "L TOTAL:");
    ili9341_show_string(8U, 72U, "R TOTAL:");
    ili9341_show_string(8U, 104U, "L STEP :");
    ili9341_show_string(8U, 136U, "R STEP :");
    ili9341_show_string(8U, 168U, "L A/B  :");
    ili9341_show_string(8U, 200U, "R A/B  :");
    ili9341_show_string(8U, 240U, "TURN ONE WHEEL");
    ili9341_show_string(8U, 264U, "ONE REV ~= 10250");
    ili9341_show_string(8U, 288U, "CHECK FORWARD SIGN");

    while (true)
    {
        my_encoder_get_delta(&left_step, &right_step);
        left_total += left_step;
        right_total += right_step;

        test_encoder_show_count(
            96U,
            40U,
            left_total,
            8U);
        test_encoder_show_count(
            96U,
            72U,
            right_total,
            8U);
        test_encoder_show_count(96U, 104U, left_step, 6U);
        test_encoder_show_count(96U, 136U, right_step, 6U);
        test_encoder_show_level(
            96U,
            168U,
            my_encoder_get_left_phase_a());
        test_encoder_show_level(
            120U,
            168U,
            my_encoder_get_left_phase_b());
        test_encoder_show_level(
            96U,
            200U,
            my_encoder_get_right_phase_a());
        test_encoder_show_level(
            120U,
            200U,
            my_encoder_get_right_phase_b());
        system_delay_ms(ENCODER_TEST_UPDATE_TIME_MS);
    }
}

#endif
