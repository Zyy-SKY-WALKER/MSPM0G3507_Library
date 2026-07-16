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

/** @brief SPI peripheral used by the display. */
#define ILI9341_SPI                    (SPI_0)
/** @brief SPI clock frequency in hertz. */
#define ILI9341_SPI_SPEED              (30U * 1000U * 1000U)
/** @brief SPI clock polarity and phase mode. */
#define ILI9341_SPI_MODE               (SPI_MODE0)

#define ILI9341_SCK_PIN                (SPI0_SCK_A12)
#define ILI9341_MOSI_PIN               (SPI0_MOSI_A9)
#define ILI9341_MISO_PIN               (SPI0_MISO_A13)

#define ILI9341_RST_PIN                (A7)
#define ILI9341_DC_PIN                 (A15)
#define ILI9341_CS_PIN                 (A8)

/** @brief Native portrait width in pixels. */
#define ILI9341_PORTRAIT_WIDTH         (240U)
/** @brief Native portrait height in pixels. */
#define ILI9341_PORTRAIT_HEIGHT        (320U)

/**
 * @brief Display orientation relative to the panel's portrait mounting.
 * @note Logical coordinates retain a top-left origin after rotation.
 */
typedef enum
{
    /** Portrait, 240 x 320 pixels. */
    ILI9341_DIR_PORTRAIT = 0,
    /** Portrait rotated by 180 degrees, 240 x 320 pixels. */
    ILI9341_DIR_PORTRAIT_180,
    /** Landscape, 320 x 240 pixels. */
    ILI9341_DIR_LANDSCAPE,
    /** Landscape rotated by 180 degrees, 320 x 240 pixels. */
    ILI9341_DIR_LANDSCAPE_180,
} ili9341_dir_enum;

/** @brief Built-in fixed-width ASCII font selection. */
typedef enum
{
    /** 6-pixel-wide by 8-pixel-high glyphs. */
    ILI9341_FONT_6X8 = 0,
    /** 8-pixel-wide by 16-pixel-high glyphs. */
    ILI9341_FONT_8X16,
} ili9341_font_enum;

/** @brief Standard 16-bit RGB565 color values. */
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

/**
 * @brief Initialize SPI, control GPIOs, and the ILI9341 controller.
 * @note Selects portrait orientation, 8x16 text, black on white, and clears
 *       the display. Call before any other display API.
 */
void ili9341_init(void);

/**
 * @brief Set the display orientation and active pixel dimensions.
 * @param dir One of ili9341_dir_enum; other values are ignored.
 * @pre ili9341_init() has completed.
 */
void ili9341_set_dir(ili9341_dir_enum dir);
/**
 * @brief Select the fixed-width font used by subsequent text operations.
 * @param font ILI9341_FONT_6X8 or ILI9341_FONT_8X16; other values are ignored.
 */
void ili9341_set_font(ili9341_font_enum font);
/**
 * @brief Set text foreground and background colors.
 * @param pen_color Foreground color in 16-bit RGB565 format.
 * @param background_color Background color in 16-bit RGB565 format.
 */
void ili9341_set_color(uint16 pen_color, uint16 background_color);

/**
 * @brief Return the active display width.
 * @return Width in pixels for the selected orientation.
 */
uint16 ili9341_get_width(void);
/**
 * @brief Return the active display height.
 * @return Height in pixels for the selected orientation.
 */
uint16 ili9341_get_height(void);

/** @brief Fill the display with the configured text background color. */
void ili9341_clear(void);
/**
 * @brief Fill the complete display with one color.
 * @param color Fill color in 16-bit RGB565 format.
 */
void ili9341_full(uint16 color);
/**
 * @brief Draw one pixel using zero-based logical coordinates.
 * @param x Horizontal pixel in the range 0 through width - 1.
 * @param y Vertical pixel in the range 0 through height - 1.
 * @param color Pixel color in 16-bit RGB565 format.
 * @note The origin is top-left; out-of-range coordinates are ignored.
 */
void ili9341_draw_point(uint16 x, uint16 y, uint16 color);
/**
 * @brief Draw a line between two inclusive pixel endpoints.
 * @param x_start Start X coordinate, zero-based.
 * @param y_start Start Y coordinate, zero-based.
 * @param x_end End X coordinate, zero-based.
 * @param y_end End Y coordinate, zero-based.
 * @param color Line color in 16-bit RGB565 format.
 * @note No clipping is performed; all endpoints must be within active bounds.
 */
void ili9341_draw_line(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end,
    uint16 color);
/**
 * @brief Fill a rectangle with inclusive, zero-based pixel bounds.
 * @param x_start Left edge in the range 0 through x_end.
 * @param y_start Top edge in the range 0 through y_end.
 * @param x_end Right edge, not greater than width - 1.
 * @param y_end Bottom edge, not greater than height - 1.
 * @param color Fill color in 16-bit RGB565 format.
 * @note Invalid or reversed bounds are ignored; no clipping is performed.
 */
void ili9341_fill_rect(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end,
    uint16 color);

/**
 * @brief Draw one printable ASCII glyph at a zero-based pixel position.
 * @param x Left pixel of the glyph.
 * @param y Top pixel of the glyph.
 * @param character Printable ASCII value from 32 through 126.
 * @note The complete active-font glyph must fit within the display.
 */
void ili9341_show_char(uint16 x, uint16 y, char character);
/**
 * @brief Draw a null-terminated printable ASCII string without wrapping.
 * @param x Left pixel of the first glyph.
 * @param y Top pixel of the glyph row.
 * @param string Non-NULL string to display.
 * @note Glyphs beyond the right edge are omitted; the glyph height must fit.
 */
void ili9341_show_string(uint16 x, uint16 y, const char string[]);
/**
 * @brief Draw an unsigned decimal value in a fixed-width field.
 * @param x Left pixel of the field.
 * @param y Top pixel of the field.
 * @param value Value to display.
 * @param digits Field width from 1 through 10 glyphs.
 * @note The field is left-padded with spaces; overflow is shown as '#'.
 */
void ili9341_show_uint(uint16 x, uint16 y, uint32 value, uint8 digits);
/**
 * @brief Draw a signed decimal value in a fixed-width magnitude field.
 * @param x Left pixel of the optional sign and magnitude field.
 * @param y Top pixel of the field.
 * @param value Value to display.
 * @param digits Magnitude width from 1 through 10 glyphs.
 * @note Negative values add a leading '-' glyph; positive values have no sign.
 *       Magnitude overflow is shown as '#'. The complete field must fit.
 */
void ili9341_show_int(uint16 x, uint16 y, int32 value, uint8 digits);
/**
 * @brief Draw an unscaled RGB565 image at a zero-based pixel position.
 * @param x Left pixel of the image.
 * @param y Top pixel of the image.
 * @param image Non-NULL row-major buffer containing width * height pixels.
 * @param width Image width in pixels; must be greater than zero.
 * @param height Image height in pixels; must be greater than zero.
 * @note The complete image must fit within the active display bounds.
 */
void ili9341_show_rgb565_image(
    uint16 x,
    uint16 y,
    const uint16 image[],
    uint16 width,
    uint16 height);

#endif
