/**
 * @file    test_ili9341.c
 * @brief   ILI9341 display verification test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341)

#include "test_ili9341.h"

#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

/**
 * @brief Show a full-screen RGB color sequence.
 */
static void test_ili9341_color_sequence(void)
{
    ili9341_full(ILI9341_COLOR_RED);
    system_delay_ms(500U);
    ili9341_full(ILI9341_COLOR_GREEN);
    system_delay_ms(500U);
    ili9341_full(ILI9341_COLOR_BLUE);
    system_delay_ms(500U);
    ili9341_full(ILI9341_COLOR_WHITE);
    system_delay_ms(500U);
}

/**
 * @brief Show text, numbers, color blocks and diagonal lines.
 */
static void test_ili9341_pattern(void)
{
    uint16 width = ili9341_get_width();
    uint16 height = ili9341_get_height();

    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "MSPM0G3507");
    ili9341_show_string(8U, 32U, "ILI9341 240x320");
    ili9341_show_string(8U, 56U, "UINT:");
    ili9341_show_uint(72U, 56U, 3507U, 4U);
    ili9341_show_string(8U, 80U, "INT:");
    ili9341_show_int(72U, 80U, -123, 3U);

    ili9341_fill_rect(8U, 112U, 68U, 151U, ILI9341_COLOR_RED);
    ili9341_fill_rect(88U, 112U, 148U, 151U, ILI9341_COLOR_GREEN);
    ili9341_fill_rect(168U, 112U, 228U, 151U, ILI9341_COLOR_BLUE);

    ili9341_draw_line(
        0U,
        176U,
        (uint16)(width - 1U),
        (uint16)(height - 1U),
        ILI9341_COLOR_YELLOW);
    ili9341_draw_line(
        (uint16)(width - 1U),
        176U,
        0U,
        (uint16)(height - 1U),
        ILI9341_COLOR_CYAN);
}

/**
 * @brief Initialize and run the ILI9341 display verification test.
 */
void test_ili9341_run(void)
{
    ili9341_init();
    test_ili9341_color_sequence();
    test_ili9341_pattern();
}

#endif
