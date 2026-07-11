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
 * @brief Display one GPIO direction level as H or L.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param level GPIO direction level.
 */
static void test_encoder_show_direction(uint16 x, uint16 y, uint8 level)
{
    if(level == GPIO_HIGH)
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
static void test_encoder_show_count(uint16 x, uint16 y, int16 count)
{
    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + 63U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, count, 6U);
}

/**
 * @brief Initialize the display and show live dual encoder counts.
 * @note Rotate each raised wheel by hand. Record count signs when the vehicle
 *       moves forward before integrating PID or odometry code.
 */
void test_encoder_run(void)
{
    ili9341_init();
    my_encoder_init();

    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "ENCODER TEST");
    ili9341_show_string(8U, 40U, "LEFT :");
    ili9341_show_string(8U, 72U, "RIGHT:");
    ili9341_show_string(8U, 104U, "L DIR:");
    ili9341_show_string(8U, 136U, "R DIR:");
    ili9341_show_string(8U, 184U, "TURN WHEELS");
    ili9341_show_string(8U, 208U, "FORWARD THEN BACK");

    while(true)
    {
        test_encoder_show_count(
            120U,
            40U,
            my_encoder_get_left_count());
        test_encoder_show_count(
            120U,
            72U,
            my_encoder_get_right_count());
        test_encoder_show_direction(
            120U,
            104U,
            my_encoder_get_left_direction());
        test_encoder_show_direction(
            120U,
            136U,
            my_encoder_get_right_direction());
        system_delay_ms(ENCODER_TEST_UPDATE_TIME_MS);
    }
}

#endif
