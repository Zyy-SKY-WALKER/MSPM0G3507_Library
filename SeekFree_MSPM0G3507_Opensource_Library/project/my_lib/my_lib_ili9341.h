/**
 * @file    my_lib_ili9341.h
 * @brief   ILI9341 SPI display driver for MSPM0G3507.
 * @note    The driver supports display functions only. XPT2046 touch
 *          functions are intentionally excluded.
 */

#ifndef MY_LIB_ILI9341_H
#define MY_LIB_ILI9341_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#define ILI9341_SPI                    (SPI_0)
#define ILI9341_SPI_SPEED              (30U * 1000U * 1000U)
#define ILI9341_SPI_MODE               (SPI_MODE0)

#define ILI9341_SCK_PIN                (SPI0_SCK_A12)
#define ILI9341_MOSI_PIN               (SPI0_MOSI_A9)
#define ILI9341_MISO_PIN               (SPI0_MISO_A13)

#define ILI9341_RST_PIN                (A7)
#define ILI9341_DC_PIN                 (A15)
#define ILI9341_CS_PIN                 (A8)

#define ILI9341_PORTRAIT_WIDTH         (240U)
#define ILI9341_PORTRAIT_HEIGHT        (320U)

typedef enum
{
    ILI9341_DIR_PORTRAIT = 0,
    ILI9341_DIR_PORTRAIT_180,
    ILI9341_DIR_LANDSCAPE,
    ILI9341_DIR_LANDSCAPE_180,
} ili9341_dir_enum;

typedef enum
{
    ILI9341_FONT_6X8 = 0,
    ILI9341_FONT_8X16,
} ili9341_font_enum;

typedef enum
{
    ILI9341_COLOR_BLACK = 0x0000,
    ILI9341_COLOR_BLUE = 0x001F,
    ILI9341_COLOR_GREEN = 0x07E0,
    ILI9341_COLOR_CYAN = 0x07FF,
    ILI9341_COLOR_RED = 0xF800,
    ILI9341_COLOR_MAGENTA = 0xF81F,
    ILI9341_COLOR_YELLOW = 0xFFE0,
    ILI9341_COLOR_WHITE = 0xFFFF,
} ili9341_color_enum;

void ili9341_init(void);

void ili9341_set_dir(ili9341_dir_enum dir);
void ili9341_set_font(ili9341_font_enum font);
void ili9341_set_color(uint16 pen_color, uint16 background_color);

uint16 ili9341_get_width(void);
uint16 ili9341_get_height(void);

void ili9341_clear(void);
void ili9341_full(uint16 color);
void ili9341_draw_point(uint16 x, uint16 y, uint16 color);
void ili9341_draw_line(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end,
    uint16 color);
void ili9341_fill_rect(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end,
    uint16 color);

void ili9341_show_char(uint16 x, uint16 y, char character);
void ili9341_show_string(uint16 x, uint16 y, const char string[]);
void ili9341_show_uint(uint16 x, uint16 y, uint32 value, uint8 digits);
void ili9341_show_int(uint16 x, uint16 y, int32 value, uint8 digits);
void ili9341_show_rgb565_image(
    uint16 x,
    uint16 y,
    const uint16 image[],
    uint16 width,
    uint16 height);

#endif
