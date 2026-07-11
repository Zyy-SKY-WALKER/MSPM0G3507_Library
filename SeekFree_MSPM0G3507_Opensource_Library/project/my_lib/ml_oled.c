/**
 * @file        ml_oled.c
 * @author      xxs13
 * @version     V1.0
 * @date        2026-07-11
 * @brief       SSD1306 OLED software I2C driver
 * @attention   The display uses B8 as SCL and B9 as SDA by default.
 *
 * This driver is based on the OLED example from Jiangxie Technology.
 */

#include "ml_oled.h"

#include <stddef.h>

#include "ml_oled_font.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"

#define ML_OLED_CONTROL_COMMAND      (0x00U)
#define ML_OLED_CONTROL_DATA         (0x40U)
#define ML_OLED_ASCII_FIRST          (' ')
#define ML_OLED_ASCII_LAST           ('~')
#define ML_OLED_FONT_WIDTH           (8U)
#define ML_OLED_CLEAR_BLOCK_SIZE     (16U)

static soft_iic_info_struct ml_oled_iic;

/**
 * @brief       Send one SSD1306 I2C transaction.
 * @param       control OLED control byte.
 * @param       data Data buffer to send.
 * @param       length Number of bytes in the data buffer.
 * @retval      true All transmitted bytes were acknowledged.
 * @retval      false A parameter is invalid or a byte was not acknowledged.
 */
static bool ml_oled_send(uint8_t control, const uint8_t *data,
                         uint16_t length)
{
    uint16_t index;
    bool result = true;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    soft_iic_start(&ml_oled_iic);
    if (soft_iic_send_data(&ml_oled_iic,
                           ML_OLED_I2C_ADDRESS << 1U) == 0U)
    {
        result = false;
    }

    if (result &&
        (soft_iic_send_data(&ml_oled_iic, control) == 0U))
    {
        result = false;
    }

    for (index = 0U; result && (index < length); index++)
    {
        if (soft_iic_send_data(&ml_oled_iic, data[index]) == 0U)
        {
            result = false;
        }
    }

    soft_iic_stop(&ml_oled_iic);
    return result;
}

/**
 * @brief       Calculate an unsigned integer power.
 * @param       base Base value.
 * @param       exponent Exponent value.
 * @return      Calculated power value.
 */
static uint32_t ml_oled_power(uint32_t base, uint8_t exponent)
{
    uint32_t result = 1U;

    while (exponent > 0U)
    {
        result *= base;
        exponent--;
    }

    return result;
}

/**
 * @brief       Check a text cell range.
 * @param       line Text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       width Number of text cells required.
 * @return      Whether the requested cells are on the display.
 */
static bool ml_oled_text_range_valid(uint8_t line, uint8_t column,
                                     uint8_t width)
{
    if ((line == 0U) || (line > ML_OLED_TEXT_LINE_COUNT))
    {
        return false;
    }

    if ((column == 0U) || (column > ML_OLED_TEXT_COLUMN_COUNT))
    {
        return false;
    }

    return (width > 0U) &&
           (width <= (ML_OLED_TEXT_COLUMN_COUNT - column + 1U));
}

/**
 * @brief       Set the OLED write position.
 * @param       page Display page, from 0 to 7.
 * @param       x Horizontal pixel coordinate, from 0 to 127.
 * @retval      true The position commands were acknowledged.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_set_cursor(uint8_t page, uint8_t x)
{
    uint8_t commands[3];

    if ((page >= ML_OLED_PAGE_COUNT) || (x >= ML_OLED_WIDTH))
    {
        return false;
    }

    commands[0] = (uint8_t)(0xB0U | page);
    commands[1] = (uint8_t)(0x10U | ((x & 0xF0U) >> 4U));
    commands[2] = (uint8_t)(x & 0x0FU);

    return ml_oled_send(ML_OLED_CONTROL_COMMAND,
                        commands,
                        (uint16_t)sizeof(commands));
}

/**
 * @brief       Clear the complete OLED display.
 * @retval      true The display was cleared successfully.
 * @retval      false I2C communication failed.
 */
bool ml_oled_clear(void)
{
    static const uint8_t clear_data[ML_OLED_CLEAR_BLOCK_SIZE] = {0U};
    uint8_t page;
    uint8_t block;

    for (page = 0U; page < ML_OLED_PAGE_COUNT; page++)
    {
        if (!ml_oled_set_cursor(page, 0U))
        {
            return false;
        }

        for (block = 0U;
             block < (ML_OLED_WIDTH / ML_OLED_CLEAR_BLOCK_SIZE);
             block++)
        {
            if (!ml_oled_send(ML_OLED_CONTROL_DATA,
                              clear_data,
                              ML_OLED_CLEAR_BLOCK_SIZE))
            {
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief       Display one 8x16 printable ASCII character.
 * @param       line Text line, from 1 to 4.
 * @param       column Text column, from 1 to 16.
 * @param       character Printable ASCII character.
 * @retval      true The character was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_char(uint8_t line, uint8_t column, char character)
{
    const uint8_t *font_data;
    uint8_t page;
    uint8_t x;

    if (!ml_oled_text_range_valid(line, column, 1U))
    {
        return false;
    }

    if ((character < ML_OLED_ASCII_FIRST) ||
        (character > ML_OLED_ASCII_LAST))
    {
        return false;
    }

    page = (uint8_t)((line - 1U) * 2U);
    x = (uint8_t)((column - 1U) * ML_OLED_FONT_WIDTH);
    font_data = ML_OLED_FONT_8X16[character - ML_OLED_ASCII_FIRST];

    if (!ml_oled_set_cursor(page, x))
    {
        return false;
    }

    if (!ml_oled_send(ML_OLED_CONTROL_DATA,
                      font_data,
                      ML_OLED_FONT_WIDTH))
    {
        return false;
    }

    if (!ml_oled_set_cursor((uint8_t)(page + 1U), x))
    {
        return false;
    }

    return ml_oled_send(ML_OLED_CONTROL_DATA,
                        &font_data[ML_OLED_FONT_WIDTH],
                        ML_OLED_FONT_WIDTH);
}

/**
 * @brief       Display an ASCII string.
 * @param       line Starting text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       string Null-terminated ASCII string.
 * @retval      true The string was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_string(uint8_t line, uint8_t column,
                         const char *string)
{
    uint8_t current_column = column;

    if ((string == NULL) ||
        !ml_oled_text_range_valid(line, column, 1U))
    {
        return false;
    }

    while ((*string != '\0') &&
           (current_column <= ML_OLED_TEXT_COLUMN_COUNT))
    {
        if (!ml_oled_show_char(line, current_column, *string))
        {
            return false;
        }

        current_column++;
        string++;
    }

    return true;
}

/**
 * @brief       Display a zero-padded unsigned decimal number.
 * @param       line Starting text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       number Number to display.
 * @param       length Number of digits, from 1 to 10.
 * @retval      true The number was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_uint(uint8_t line, uint8_t column, uint32_t number,
                       uint8_t length)
{
    uint8_t index;
    uint32_t divisor;
    char character;

    if ((length > 10U) ||
        !ml_oled_text_range_valid(line, column, length))
    {
        return false;
    }

    for (index = 0U; index < length; index++)
    {
        divisor = ml_oled_power(10U, (uint8_t)(length - index - 1U));
        character = (char)(((number / divisor) % 10U) + '0');

        if (!ml_oled_show_char(line,
                               (uint8_t)(column + index),
                               character))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief       Display a signed decimal number with an explicit sign.
 * @param       line Starting text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       number Number to display.
 * @param       length Number of digits excluding the sign, from 1 to 10.
 * @retval      true The number was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_int(uint8_t line, uint8_t column, int32_t number,
                      uint8_t length)
{
    uint32_t magnitude;
    char sign;

    if ((length == 0U) || (length > 10U) ||
        !ml_oled_text_range_valid(line,
                                  column,
                                  (uint8_t)(length + 1U)))
    {
        return false;
    }

    if (number < 0)
    {
        sign = '-';
        magnitude = (uint32_t)(-(number + 1)) + 1U;
    }
    else
    {
        sign = '+';
        magnitude = (uint32_t)number;
    }

    if (!ml_oled_show_char(line, column, sign))
    {
        return false;
    }

    return ml_oled_show_uint(line,
                             (uint8_t)(column + 1U),
                             magnitude,
                             length);
}

/**
 * @brief       Display an uppercase hexadecimal number.
 * @param       line Starting text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       number Number to display.
 * @param       length Number of hexadecimal digits, from 1 to 8.
 * @retval      true The number was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_hex(uint8_t line, uint8_t column, uint32_t number,
                      uint8_t length)
{
    uint8_t index;
    uint8_t digit;
    uint8_t shift;
    char character;

    if ((length > 8U) ||
        !ml_oled_text_range_valid(line, column, length))
    {
        return false;
    }

    for (index = 0U; index < length; index++)
    {
        shift = (uint8_t)((length - index - 1U) * 4U);
        digit = (uint8_t)((number >> shift) & 0x0FU);
        character = (digit < 10U) ?
                    (char)(digit + '0') :
                    (char)(digit - 10U + 'A');

        if (!ml_oled_show_char(line,
                               (uint8_t)(column + index),
                               character))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief       Display a zero-padded binary number.
 * @param       line Starting text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       number Number to display.
 * @param       length Number of binary digits, from 1 to 16.
 * @retval      true The number was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_binary(uint8_t line, uint8_t column, uint32_t number,
                         uint8_t length)
{
    uint8_t index;
    uint8_t shift;
    char character;

    if ((length > 16U) ||
        !ml_oled_text_range_valid(line, column, length))
    {
        return false;
    }

    for (index = 0U; index < length; index++)
    {
        shift = (uint8_t)(length - index - 1U);
        character = (char)(((number >> shift) & 0x01U) + '0');

        if (!ml_oled_show_char(line,
                               (uint8_t)(column + index),
                               character))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief       Display a floating-point number with an explicit sign.
 * @param       line Starting text line, from 1 to 4.
 * @param       column Starting text column, from 1 to 16.
 * @param       number Number to display.
 * @param       integer_length Number of integer digits, from 1 to 10.
 * @param       fractional_length Number of fractional digits, from 1 to 9.
 * @retval      true The number was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_show_float(uint8_t line, uint8_t column, float number,
                        uint8_t integer_length,
                        uint8_t fractional_length)
{
    double absolute_number;
    double fraction;
    uint32_t integer_part;
    uint32_t fractional_part;
    uint32_t scale;
    uint8_t display_width;
    char sign;

    display_width = (uint8_t)(integer_length + fractional_length + 2U);
    if ((number != number) ||
        (integer_length == 0U) || (integer_length > 10U) ||
        (fractional_length == 0U) || (fractional_length > 9U) ||
        !ml_oled_text_range_valid(line, column, display_width))
    {
        return false;
    }

    sign = (number < 0.0F) ? '-' : '+';
    absolute_number = (number < 0.0F) ? -(double)number : (double)number;
    if (absolute_number > (double)UINT32_MAX)
    {
        return false;
    }

    scale = ml_oled_power(10U, fractional_length);
    integer_part = (uint32_t)absolute_number;
    fraction = absolute_number - (double)integer_part;
    fractional_part = (uint32_t)(fraction * (double)scale + 0.5);

    if (fractional_part >= scale)
    {
        if (integer_part == UINT32_MAX)
        {
            return false;
        }

        integer_part++;
        fractional_part = 0U;
    }

    if (!ml_oled_show_char(line, column, sign))
    {
        return false;
    }

    if (!ml_oled_show_uint(line,
                           (uint8_t)(column + 1U),
                           integer_part,
                           integer_length))
    {
        return false;
    }

    if (!ml_oled_show_char(line,
                           (uint8_t)(column + integer_length + 1U),
                           '.'))
    {
        return false;
    }

    return ml_oled_show_uint(
        line,
        (uint8_t)(column + integer_length + 2U),
        fractional_part,
        fractional_length);
}

/**
 * @brief       Draw a page-organized monochrome bitmap.
 * @param       x_start First horizontal pixel coordinate.
 * @param       page_start First display page.
 * @param       x_end Horizontal end coordinate, excluded.
 * @param       page_end Page end coordinate, excluded.
 * @param       bitmap Page-major bitmap data.
 * @retval      true The bitmap was displayed successfully.
 * @retval      false A parameter is invalid or I2C communication failed.
 */
bool ml_oled_draw_bitmap(uint8_t x_start, uint8_t page_start,
                         uint8_t x_end, uint8_t page_end,
                         const uint8_t *bitmap)
{
    uint16_t offset = 0U;
    uint8_t page;
    uint8_t width;

    if ((bitmap == NULL) || (x_start >= x_end) ||
        (x_end > ML_OLED_WIDTH) || (page_start >= page_end) ||
        (page_end > ML_OLED_PAGE_COUNT))
    {
        return false;
    }

    width = (uint8_t)(x_end - x_start);
    for (page = page_start; page < page_end; page++)
    {
        if (!ml_oled_set_cursor(page, x_start))
        {
            return false;
        }

        if (!ml_oled_send(ML_OLED_CONTROL_DATA,
                          &bitmap[offset],
                          width))
        {
            return false;
        }

        offset += width;
    }

    return true;
}

/**
 * @brief       Initialize the SSD1306 OLED display.
 * @retval      true Initialization and display clearing succeeded.
 * @retval      false The OLED did not acknowledge an I2C transaction.
 */
bool ml_oled_init(void)
{
    static const uint8_t init_commands[] =
    {
        0xAEU,
        0xD5U, 0x80U,
        0xA8U, 0x3FU,
        0xD3U, 0x00U,
        0x40U,
        0xA1U,
        0xC8U,
        0xDAU, 0x12U,
        0x81U, 0xCFU,
        0xD9U, 0xF1U,
        0xDBU, 0x30U,
        0xA4U,
        0xA6U,
        0x8DU, 0x14U,
        0xAFU
    };

    soft_iic_init(&ml_oled_iic,
                  ML_OLED_I2C_ADDRESS,
                  ML_OLED_I2C_DELAY,
                  ML_OLED_SCL_PIN,
                  ML_OLED_SDA_PIN);
    system_delay_ms(100U);

    if (!ml_oled_send(ML_OLED_CONTROL_COMMAND,
                      init_commands,
                      (uint16_t)sizeof(init_commands)))
    {
        return false;
    }

    return ml_oled_clear();
}
