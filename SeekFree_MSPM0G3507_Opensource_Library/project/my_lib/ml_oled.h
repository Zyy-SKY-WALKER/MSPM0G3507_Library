/**
 * @file        ml_oled.h
 * @author      xxs13
 * @version     V1.0
 * @date        2026-07-11
 * @brief       SSD1306 OLED software I2C driver interface
 * @attention   This driver is intended for the MSPM0G3507 platform.
 * @note        Text APIs use one-based 8x16 cell coordinates. Cursor and
 *              bitmap APIs use zero-based horizontal pixels and 8-pixel pages.
 * @note        An I2C failure may leave a partially updated display.
 */

#ifndef ML_OLED_H
#define ML_OLED_H

#include <stdbool.h>
#include <stdint.h>

#include "zf_driver_gpio.h"

/** @brief Seven-bit SSD1306 I2C slave address. */
#define ML_OLED_I2C_ADDRESS          (0x3CU)
/** @brief Software I2C busy-wait delay in loop iterations. */
#define ML_OLED_I2C_DELAY            (100U)
#define ML_OLED_SCL_PIN              (B8)
#define ML_OLED_SDA_PIN              (B9)

/** @brief Display width in pixels. */
#define ML_OLED_WIDTH                (128U)
/** @brief Display height in pixels. */
#define ML_OLED_HEIGHT               (64U)
/** @brief Number of eight-pixel-high display pages. */
#define ML_OLED_PAGE_COUNT           (8U)
/** @brief Number of one-based 8x16 text lines. */
#define ML_OLED_TEXT_LINE_COUNT      (4U)
/** @brief Number of one-based 8x16 text columns. */
#define ML_OLED_TEXT_COLUMN_COUNT    (16U)

/**
 * @brief       Initialize and clear the SSD1306 display.
 * @retval      true Initialization and clearing completed successfully.
 * @retval      false An I2C transaction was not acknowledged.
 */
bool ml_oled_init(void);
/**
 * @brief       Clear all 128 x 64 pixels.
 * @retval      true The complete display was cleared.
 * @retval      false I2C communication failed; clearing may be partial.
 * @pre         ml_oled_init() has configured the software I2C interface.
 */
bool ml_oled_clear(void);
/**
 * @brief       Set the zero-based page and horizontal pixel write position.
 * @param       page Page index from 0 through 7; each page is 8 pixels high.
 * @param       x Horizontal pixel coordinate from 0 through 127.
 * @retval      true All cursor commands were acknowledged.
 * @retval      false A coordinate is out of range or I2C communication failed.
 */
bool ml_oled_set_cursor(uint8_t page, uint8_t x);
/**
 * @brief       Display one 8x16 printable ASCII character.
 * @param       line One-based text line from 1 through 4.
 * @param       column One-based text column from 1 through 16.
 * @param       character Printable ASCII value from 32 through 126.
 * @retval      true The complete character was transmitted.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_char(uint8_t line, uint8_t column, char character);
/**
 * @brief       Display a null-terminated ASCII string without wrapping.
 * @param       line One-based starting text line from 1 through 4.
 * @param       column One-based starting text column from 1 through 16.
 * @param       string Non-NULL printable ASCII string.
 * @retval      true Valid characters through column 16 were transmitted.
 * @retval      false A parameter is invalid or I2C communication failed.
 * @note        Characters beyond column 16 are silently omitted.
 */
bool ml_oled_show_string(uint8_t line, uint8_t column,
                         const char *string);
/**
 * @brief       Display a zero-padded unsigned decimal field.
 * @param       line One-based starting text line from 1 through 4.
 * @param       column One-based starting text column from 1 through 16.
 * @param       number Value whose low-order decimal digits are displayed.
 * @param       length Field width from 1 through 10 text cells.
 * @retval      true The complete field was transmitted.
 * @retval      false The field does not fit, a parameter is invalid, or I2C
 *                    communication failed.
 */
bool ml_oled_show_uint(uint8_t line, uint8_t column, uint32_t number,
                       uint8_t length);
/**
 * @brief       Display an explicitly signed, zero-padded decimal field.
 * @param       line One-based starting text line from 1 through 4.
 * @param       column One-based starting text column from 1 through 16.
 * @param       number Signed value whose magnitude is displayed.
 * @param       length Magnitude width from 1 through 10 cells, excluding sign.
 * @retval      true The sign and complete magnitude field were transmitted.
 * @retval      false The field does not fit, a parameter is invalid, or I2C
 *                    communication failed.
 * @note        Both nonnegative and negative values consume one sign cell.
 * @note        Magnitudes wider than length keep only low-order digits.
 */
bool ml_oled_show_int(uint8_t line, uint8_t column, int32_t number,
                      uint8_t length);
/**
 * @brief       Display a zero-padded uppercase hexadecimal field.
 * @param       line One-based starting text line from 1 through 4.
 * @param       column One-based starting text column from 1 through 16.
 * @param       number Value whose low-order hexadecimal digits are displayed.
 * @param       length Field width from 1 through 8 text cells.
 * @retval      true The complete field was transmitted.
 * @retval      false The field does not fit, a parameter is invalid, or I2C
 *                    communication failed.
 */
bool ml_oled_show_hex(uint8_t line, uint8_t column, uint32_t number,
                      uint8_t length);
/**
 * @brief       Display a zero-padded binary field.
 * @param       line One-based starting text line from 1 through 4.
 * @param       column One-based starting text column from 1 through 16.
 * @param       number Value whose low-order bits are displayed.
 * @param       length Field width from 1 through 16 text cells.
 * @retval      true The complete field was transmitted.
 * @retval      false The field does not fit, a parameter is invalid, or I2C
 *                    communication failed.
 */
bool ml_oled_show_binary(uint8_t line, uint8_t column, uint32_t number,
                         uint8_t length);
/**
 * @brief       Display a signed, rounded fixed-point decimal field.
 * @param       line One-based starting text line from 1 through 4.
 * @param       column One-based starting text column from 1 through 16.
 * @param       number Finite value with magnitude representable by uint32_t.
 * @param       integer_length Integer field width from 1 through 10 cells.
 * @param       fractional_length Fractional width from 1 through 9 cells.
 * @retval      true The sign, integer, decimal point, and fraction were sent.
 * @retval      false The field does not fit, the value or a length is invalid,
 *                    or I2C communication failed.
 * @note        Total width is integer_length + fractional_length + 2 cells.
 * @note        An integer part wider than integer_length keeps low-order
 *              digits after fractional rounding.
 */
bool ml_oled_show_float(uint8_t line, uint8_t column, float number,
                        uint8_t integer_length,
                        uint8_t fractional_length);
/**
 * @brief       Draw a monochrome bitmap using zero-based half-open bounds.
 * @param       x_start First horizontal pixel, included; range 0 through 127.
 * @param       page_start First eight-pixel page, included; range 0 through 7.
 * @param       x_end Horizontal pixel bound, excluded; range 1 through 128.
 * @param       page_end Page bound, excluded; range 1 through 8.
 * @param       bitmap Non-NULL page-major buffer. Each page contains
 *                    x_end - x_start bytes, one vertical 8-pixel column per
 *                    byte with bits 0 through 7 selecting its page pixels,
 *                    for a total of width * page count bytes.
 * @retval      true The complete bitmap was transmitted.
 * @retval      false Bounds are empty or invalid, bitmap is NULL, or I2C
 *                    communication failed.
 */
bool ml_oled_draw_bitmap(uint8_t x_start, uint8_t page_start,
                         uint8_t x_end, uint8_t page_end,
                         const uint8_t *bitmap);

#endif
