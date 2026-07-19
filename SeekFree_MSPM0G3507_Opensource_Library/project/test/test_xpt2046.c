/**
 * @file    test_xpt2046.c
 * @brief   Independent XPT2046 touch controller verification test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_XPT2046)

#include "test_xpt2046.h"

#include "my_lib_ili9341.h"
#include "my_lib_xpt2046.h"
#include "zf_driver_delay.h"

#define TEST_XPT2046_LOOP_DELAY_MS       (10U)
#define TEST_XPT2046_POINT_RADIUS        (2U)

/**
 * @brief Draw the static touch diagnostic labels.
 */
static void test_xpt2046_draw_layout(void)
{
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "XPT2046 TOUCH");
    ili9341_show_string(8U, 32U, "RAW X:");
    ili9341_show_string(8U, 56U, "RAW Y:");
    ili9341_show_string(8U, 80U, "PIX X:");
    ili9341_show_string(8U, 104U, "PIX Y:");
    ili9341_show_string(8U, 128U, "STATUS:");
    ili9341_set_color(ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
    ili9341_show_string(72U, 128U, "RELEASED");
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
}

/**
 * @brief Draw a small marker at a calibrated touch coordinate.
 * @param x Horizontal pixel coordinate.
 * @param y Vertical pixel coordinate.
 */
static void test_xpt2046_draw_point(uint16 x, uint16 y)
{
    uint16 x_start;
    uint16 y_start;
    uint16 x_end;
    uint16 y_end;

    x_start = (x > TEST_XPT2046_POINT_RADIUS)
        ? (uint16)(x - TEST_XPT2046_POINT_RADIUS)
        : 0U;
    y_start = (y > TEST_XPT2046_POINT_RADIUS)
        ? (uint16)(y - TEST_XPT2046_POINT_RADIUS)
        : 0U;
    x_end = (uint16)(x + TEST_XPT2046_POINT_RADIUS);
    y_end = (uint16)(y + TEST_XPT2046_POINT_RADIUS);

    if(x_end >= XPT2046_SCREEN_WIDTH)
    {
        x_end = XPT2046_SCREEN_WIDTH - 1U;
    }
    if(y_end >= XPT2046_SCREEN_HEIGHT)
    {
        y_end = XPT2046_SCREEN_HEIGHT - 1U;
    }

    ili9341_fill_rect(
        x_start,
        y_start,
        x_end,
        y_end,
        ILI9341_COLOR_YELLOW);
}

/**
 * @brief Update the displayed raw and calibrated coordinate values.
 * @param raw_x Filtered X ADC value.
 * @param raw_y Filtered Y ADC value.
 * @param x Calibrated horizontal pixel coordinate.
 * @param y Calibrated vertical pixel coordinate.
 */
static void test_xpt2046_show_point(
    uint16 raw_x,
    uint16 raw_y,
    uint16 x,
    uint16 y)
{
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_uint(72U, 32U, raw_x, 4U);
    ili9341_show_uint(72U, 56U, raw_y, 4U);
    ili9341_show_uint(72U, 80U, x, 3U);
    ili9341_show_uint(72U, 104U, y, 3U);
    test_xpt2046_draw_point(x, y);
}

/**
 * @brief Initialize the display and touch drivers, then poll touch data.
 */
void test_xpt2046_run(void)
{
    uint16 raw_x;
    uint16 raw_y;
    uint16 x;
    uint16 y;
    uint8 was_pressed = 0U;

    ili9341_init();
    xpt2046_init();
    test_xpt2046_draw_layout();

    while(1)
    {
        if(xpt2046_is_pressed())
        {
            if(was_pressed == 0U)
            {
                ili9341_set_color(
                    ILI9341_COLOR_GREEN,
                    ILI9341_COLOR_BLACK);
                ili9341_show_string(72U, 128U, "PRESSED ");
                was_pressed = 1U;
            }

            if(xpt2046_read_raw(&raw_x, &raw_y))
            {
                xpt2046_convert_point(raw_x, raw_y, &x, &y);
                test_xpt2046_show_point(raw_x, raw_y, x, y);
            }
        }
        else if(was_pressed != 0U)
        {
            ili9341_set_color(
                ILI9341_COLOR_RED,
                ILI9341_COLOR_BLACK);
            ili9341_show_string(72U, 128U, "RELEASED");
            was_pressed = 0U;
        }

        system_delay_ms(TEST_XPT2046_LOOP_DELAY_MS);
    }
}

#endif
