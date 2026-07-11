
/**
 * @file    my_lib_ili9341.c
 * @brief   ILI9341 SPI display driver for MSPM0G3507.
 * @note    The controller initialization values are based on the provided
 *          STM32 ILI9341 reference driver. The transport layer is rewritten
 *          for the SeekFree MSPM0G3507 SPI and GPIO APIs.
 */

#include "my_lib_ili9341.h"

#include "zf_common_font.h"
#include "zf_driver_delay.h"

#define ILI9341_COMMAND_COLUMN_ADDRESS  (0x2AU)
#define ILI9341_COMMAND_PAGE_ADDRESS    (0x2BU)
#define ILI9341_COMMAND_MEMORY_WRITE    (0x2CU)
#define ILI9341_COMMAND_MEMORY_ACCESS   (0x36U)
#define ILI9341_COMMAND_SLEEP_OUT       (0x11U)
#define ILI9341_COMMAND_DISPLAY_ON      (0x29U)

#define ILI9341_MADCTL_BGR              (0x08U)
#define ILI9341_MADCTL_MV               (0x20U)
#define ILI9341_MADCTL_MX               (0x40U)
#define ILI9341_MADCTL_MY               (0x80U)

#define ILI9341_FILL_BUFFER_SIZE        (64U)
#define ILI9341_ASCII_FIRST             (32)
#define ILI9341_ASCII_LAST              (126)

static ili9341_font_enum ili9341_display_font = ILI9341_FONT_8X16;
static uint16 ili9341_width = ILI9341_PORTRAIT_WIDTH;
static uint16 ili9341_height = ILI9341_PORTRAIT_HEIGHT;
static uint16 ili9341_pen_color = ILI9341_COLOR_BLACK;
static uint16 ili9341_background_color = ILI9341_COLOR_WHITE;

/**
 * @brief Set the display chip-select level.
 * @param level GPIO_LOW selects the display; GPIO_HIGH releases it.
 */
static void ili9341_set_cs(uint8 level)
{
    gpio_set_level(ILI9341_CS_PIN, level);
}

/**
 * @brief Set the display data/command level.
 * @param level GPIO_LOW selects command; GPIO_HIGH selects data.
 */
static void ili9341_set_dc(uint8 level)
{
    gpio_set_level(ILI9341_DC_PIN, level);
}

/**
 * @brief Write one command byte.
 * @param command ILI9341 command value.
 */
static void ili9341_write_command(uint8 command)
{
    ili9341_set_cs(GPIO_LOW);
    ili9341_set_dc(GPIO_LOW);
    spi_write_8bit(ILI9341_SPI, command);
    ili9341_set_cs(GPIO_HIGH);
}

/**
 * @brief Write a command followed by parameter bytes.
 * @param command ILI9341 command value.
 * @param data Parameter buffer, or NULL when length is zero.
 * @param length Number of parameter bytes.
 */
static void ili9341_write_command_data(
    uint8 command,
    const uint8 data[],
    uint32 length)
{
    if((length > 0U) && (data == NULL))
    {
        return;
    }

    ili9341_set_cs(GPIO_LOW);
    ili9341_set_dc(GPIO_LOW);
    spi_write_8bit(ILI9341_SPI, command);

    if((data != NULL) && (length > 0U))
    {
        ili9341_set_dc(GPIO_HIGH);
        spi_write_8bit_array(ILI9341_SPI, data, length);
    }

    ili9341_set_cs(GPIO_HIGH);
}

/**
 * @brief Reset the ILI9341 controller through the RST pin.
 */
static void ili9341_hardware_reset(void)
{
    gpio_set_level(ILI9341_RST_PIN, GPIO_HIGH);
    system_delay_ms(10U);
    gpio_set_level(ILI9341_RST_PIN, GPIO_LOW);
    system_delay_ms(20U);
    gpio_set_level(ILI9341_RST_PIN, GPIO_HIGH);
    system_delay_ms(120U);
}

/**
 * @brief Configure ILI9341 power, timing, pixel format and gamma values.
 */
static void ili9341_configure_controller(void)
{
    static const uint8 power_control_b[] = {0x00U, 0xC9U, 0x30U};
    static const uint8 power_on_sequence[] = {
        0x64U, 0x03U, 0x12U, 0x81U};
    static const uint8 driver_timing_a[] = {0x85U, 0x10U, 0x7AU};
    static const uint8 power_control_a[] = {
        0x39U, 0x2CU, 0x00U, 0x34U, 0x02U};
    static const uint8 driver_timing_b[] = {0x00U, 0x00U};
    static const uint8 vcom_control[] = {0x30U, 0x30U};
    static const uint8 frame_rate[] = {0x00U, 0x1AU};
    static const uint8 display_function[] = {0x0AU, 0xA2U};
    static const uint8 positive_gamma[] = {
        0x0FU, 0x2AU, 0x28U, 0x08U, 0x0EU,
        0x08U, 0x54U, 0xA9U, 0x43U, 0x0AU,
        0x0FU, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8 negative_gamma[] = {
        0x00U, 0x15U, 0x17U, 0x07U, 0x11U,
        0x06U, 0x2BU, 0x56U, 0x3CU, 0x05U,
        0x10U, 0x0FU, 0x3FU, 0x3FU, 0x0FU};
    static const uint8 pump_ratio = 0x20U;
    static const uint8 power_control_1 = 0x1BU;
    static const uint8 power_control_2 = 0x00U;
    static const uint8 vcom_control_2 = 0xB7U;
    static const uint8 pixel_format = 0x55U;
    static const uint8 gamma_disable = 0x00U;
    static const uint8 gamma_curve = 0x01U;

    ili9341_write_command_data(0xCFU, power_control_b,
        sizeof(power_control_b));
    ili9341_write_command_data(0xEDU, power_on_sequence,
        sizeof(power_on_sequence));
    ili9341_write_command_data(0xE8U, driver_timing_a,
        sizeof(driver_timing_a));
    ili9341_write_command_data(0xCBU, power_control_a,
        sizeof(power_control_a));
    ili9341_write_command_data(0xF7U, &pump_ratio, 1U);
    ili9341_write_command_data(0xEAU, driver_timing_b,
        sizeof(driver_timing_b));
    ili9341_write_command_data(0xC0U, &power_control_1, 1U);
    ili9341_write_command_data(0xC1U, &power_control_2, 1U);
    ili9341_write_command_data(0xC5U, vcom_control,
        sizeof(vcom_control));
    ili9341_write_command_data(0xC7U, &vcom_control_2, 1U);
    ili9341_write_command_data(0x3AU, &pixel_format, 1U);
    ili9341_write_command_data(0xB1U, frame_rate,
        sizeof(frame_rate));
    ili9341_write_command_data(0xB6U, display_function,
        sizeof(display_function));
    ili9341_write_command_data(0xF2U, &gamma_disable, 1U);
    ili9341_write_command_data(0x26U, &gamma_curve, 1U);
    ili9341_write_command_data(0xE0U, positive_gamma,
        sizeof(positive_gamma));
    ili9341_write_command_data(0xE1U, negative_gamma,
        sizeof(negative_gamma));
}

/**
 * @brief Select an inclusive drawing window and enter memory-write mode.
 * @param x_start Left coordinate.
 * @param y_start Top coordinate.
 * @param x_end Right coordinate.
 * @param y_end Bottom coordinate.
 */
static void ili9341_set_window(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end)
{
    uint8 column_data[4];
    uint8 page_data[4];

    column_data[0] = (uint8)(x_start >> 8);
    column_data[1] = (uint8)(x_start & 0x00FFU);
    column_data[2] = (uint8)(x_end >> 8);
    column_data[3] = (uint8)(x_end & 0x00FFU);

    page_data[0] = (uint8)(y_start >> 8);
    page_data[1] = (uint8)(y_start & 0x00FFU);
    page_data[2] = (uint8)(y_end >> 8);
    page_data[3] = (uint8)(y_end & 0x00FFU);

    ili9341_write_command_data(
        ILI9341_COMMAND_COLUMN_ADDRESS,
        column_data,
        sizeof(column_data));
    ili9341_write_command_data(
        ILI9341_COMMAND_PAGE_ADDRESS,
        page_data,
        sizeof(page_data));
    ili9341_write_command(ILI9341_COMMAND_MEMORY_WRITE);
}

/**
 * @brief Return the active font width in pixels.
 * @return Font width.
 */
static uint8 ili9341_get_font_width(void)
{
    return (ili9341_display_font == ILI9341_FONT_6X8) ? 6U : 8U;
}

/**
 * @brief Return the active font height in pixels.
 * @return Font height.
 */
static uint8 ili9341_get_font_height(void)
{
    return (ili9341_display_font == ILI9341_FONT_6X8) ? 8U : 16U;
}

/**
 * @brief Convert an unsigned value to a fixed-width decimal string.
 * @param buffer Destination buffer. It must hold digits plus a terminator.
 * @param value Value to convert.
 * @param digits Number of displayed digits.
 */
static void ili9341_format_uint(char buffer[], uint32 value, uint8 digits)
{
    uint8 index;

    buffer[digits] = '\0';
    for(index = digits; index > 0U; index--)
    {
        buffer[index - 1U] = (char)('0' + (value % 10U));
        value /= 10U;

        if(value == 0U)
        {
            index--;
            while(index > 0U)
            {
                buffer[index - 1U] = ' ';
                index--;
            }
            break;
        }
    }

    if(value > 0U)
    {
        for(index = 0U; index < digits; index++)
        {
            buffer[index] = '#';
        }
    }
}

/**
 * @brief Initialize the SPI interface, GPIO control pins and ILI9341.
 */
void ili9341_init(void)
{
    spi_init(
        ILI9341_SPI,
        ILI9341_SPI_MODE,
        ILI9341_SPI_SPEED,
        ILI9341_SCK_PIN,
        ILI9341_MOSI_PIN,
        ILI9341_MISO_PIN,
        SPI_CS_NULL);

    gpio_init(ILI9341_RST_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(ILI9341_DC_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(ILI9341_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    ili9341_hardware_reset();
    ili9341_configure_controller();
    ili9341_write_command(ILI9341_COMMAND_SLEEP_OUT);
    system_delay_ms(120U);
    ili9341_write_command(ILI9341_COMMAND_DISPLAY_ON);
    system_delay_ms(20U);

    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
    ili9341_clear();
}

/**
 * @brief Set display rotation and update the active dimensions.
 * @param dir Display direction.
 */
void ili9341_set_dir(ili9341_dir_enum dir)
{
    uint8 madctl;

    switch(dir)
    {
        case ILI9341_DIR_PORTRAIT:
        {
            madctl = ILI9341_MADCTL_BGR;
            ili9341_width = ILI9341_PORTRAIT_WIDTH;
            ili9341_height = ILI9341_PORTRAIT_HEIGHT;
            break;
        }

        case ILI9341_DIR_PORTRAIT_180:
        {
            madctl = ILI9341_MADCTL_MY
                | ILI9341_MADCTL_MX
                | ILI9341_MADCTL_BGR;
            ili9341_width = ILI9341_PORTRAIT_WIDTH;
            ili9341_height = ILI9341_PORTRAIT_HEIGHT;
            break;
        }

        case ILI9341_DIR_LANDSCAPE:
        {
            madctl = ILI9341_MADCTL_MY
                | ILI9341_MADCTL_MV
                | ILI9341_MADCTL_BGR;
            ili9341_width = ILI9341_PORTRAIT_HEIGHT;
            ili9341_height = ILI9341_PORTRAIT_WIDTH;
            break;
        }

        case ILI9341_DIR_LANDSCAPE_180:
        {
            madctl = ILI9341_MADCTL_MX
                | ILI9341_MADCTL_MV
                | ILI9341_MADCTL_BGR;
            ili9341_width = ILI9341_PORTRAIT_HEIGHT;
            ili9341_height = ILI9341_PORTRAIT_WIDTH;
            break;
        }

        default:
        {
            return;
        }
    }

    ili9341_write_command_data(
        ILI9341_COMMAND_MEMORY_ACCESS,
        &madctl,
        1U);
}

/**
 * @brief Select the font used by subsequent text calls.
 * @param font Supported font size.
 */
void ili9341_set_font(ili9341_font_enum font)
{
    if((font == ILI9341_FONT_6X8) || (font == ILI9341_FONT_8X16))
    {
        ili9341_display_font = font;
    }
}

/**
 * @brief Set text foreground and background colors.
 * @param pen_color Foreground RGB565 color.
 * @param background_color Background RGB565 color.
 */
void ili9341_set_color(uint16 pen_color, uint16 background_color)
{
    ili9341_pen_color = pen_color;
    ili9341_background_color = background_color;
}

/**
 * @brief Return the active display width.
 * @return Width in pixels.
 */
uint16 ili9341_get_width(void)
{
    return ili9341_width;
}

/**
 * @brief Return the active display height.
 * @return Height in pixels.
 */
uint16 ili9341_get_height(void)
{
    return ili9341_height;
}

/**
 * @brief Clear the display with the configured background color.
 */
void ili9341_clear(void)
{
    ili9341_full(ili9341_background_color);
}

/**
 * @brief Fill the complete display with one RGB565 color.
 * @param color Fill color.
 */
void ili9341_full(uint16 color)
{
    ili9341_fill_rect(
        0U,
        0U,
        (uint16)(ili9341_width - 1U),
        (uint16)(ili9341_height - 1U),
        color);
}

/**
 * @brief Draw one pixel.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param color RGB565 color.
 */
void ili9341_draw_point(uint16 x, uint16 y, uint16 color)
{
    if((x >= ili9341_width) || (y >= ili9341_height))
    {
        return;
    }

    ili9341_set_window(x, y, x, y);
    ili9341_set_cs(GPIO_LOW);
    ili9341_set_dc(GPIO_HIGH);
    spi_write_16bit(ILI9341_SPI, color);
    ili9341_set_cs(GPIO_HIGH);
}

/**
 * @brief Draw a line using the integer Bresenham algorithm.
 * @param x_start Start X coordinate.
 * @param y_start Start Y coordinate.
 * @param x_end End X coordinate.
 * @param y_end End Y coordinate.
 * @param color RGB565 color.
 */
void ili9341_draw_line(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end,
    uint16 color)
{
    int32 x = x_start;
    int32 y = y_start;
    int32 target_x = x_end;
    int32 target_y = y_end;
    int32 delta_x;
    int32 delta_y;
    int32 step_x;
    int32 step_y;
    int32 error;

    if((x_start >= ili9341_width) || (x_end >= ili9341_width)
        || (y_start >= ili9341_height) || (y_end >= ili9341_height))
    {
        return;
    }

    if(y_start == y_end)
    {
        if(x_start > x_end)
        {
            uint16 temporary = x_start;
            x_start = x_end;
            x_end = temporary;
        }
        ili9341_fill_rect(x_start, y_start, x_end, y_end, color);
        return;
    }

    if(x_start == x_end)
    {
        if(y_start > y_end)
        {
            uint16 temporary = y_start;
            y_start = y_end;
            y_end = temporary;
        }
        ili9341_fill_rect(x_start, y_start, x_end, y_end, color);
        return;
    }

    delta_x = (target_x >= x) ? (target_x - x) : (x - target_x);
    delta_y = (target_y >= y) ? (target_y - y) : (y - target_y);
    step_x = (x < target_x) ? 1 : -1;
    step_y = (y < target_y) ? 1 : -1;
    error = delta_x - delta_y;

    while(1)
    {
        int32 error_double;

        ili9341_draw_point((uint16)x, (uint16)y, color);
        if((x == target_x) && (y == target_y))
        {
            break;
        }

        error_double = error * 2;
        if(error_double > -delta_y)
        {
            error -= delta_y;
            x += step_x;
        }
        if(error_double < delta_x)
        {
            error += delta_x;
            y += step_y;
        }
    }
}

/**
 * @brief Fill an inclusive rectangle.
 * @param x_start Left coordinate.
 * @param y_start Top coordinate.
 * @param x_end Right coordinate.
 * @param y_end Bottom coordinate.
 * @param color RGB565 color.
 */
void ili9341_fill_rect(
    uint16 x_start,
    uint16 y_start,
    uint16 x_end,
    uint16 y_end,
    uint16 color)
{
    uint16 color_buffer[ILI9341_FILL_BUFFER_SIZE];
    uint32 pixel_count;
    uint32 transfer_count;
    uint16 index;

    if((x_start > x_end) || (y_start > y_end)
        || (x_end >= ili9341_width) || (y_end >= ili9341_height))
    {
        return;
    }

    for(index = 0U; index < ILI9341_FILL_BUFFER_SIZE; index++)
    {
        color_buffer[index] = color;
    }

    pixel_count = ((uint32)x_end - x_start + 1U)
        * ((uint32)y_end - y_start + 1U);
    ili9341_set_window(x_start, y_start, x_end, y_end);
    ili9341_set_cs(GPIO_LOW);
    ili9341_set_dc(GPIO_HIGH);

    while(pixel_count > 0U)
    {
        transfer_count = pixel_count;
        if(transfer_count > ILI9341_FILL_BUFFER_SIZE)
        {
            transfer_count = ILI9341_FILL_BUFFER_SIZE;
        }

        spi_write_16bit_array(
            ILI9341_SPI,
            color_buffer,
            transfer_count);
        pixel_count -= transfer_count;
    }

    ili9341_set_cs(GPIO_HIGH);
}

/**
 * @brief Draw one printable ASCII character.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param character Printable ASCII character.
 */
void ili9341_show_char(uint16 x, uint16 y, char character)
{
    uint16 display_buffer[8U * 16U];
    uint8 font_width = ili9341_get_font_width();
    uint8 font_height = ili9341_get_font_height();
    uint8 column;
    uint8 row;
    uint8 character_index;

    if((character < ILI9341_ASCII_FIRST)
        || (character > ILI9341_ASCII_LAST)
        || ((uint32)x + font_width > ili9341_width)
        || ((uint32)y + font_height > ili9341_height))
    {
        return;
    }

    character_index = (uint8)(character - ILI9341_ASCII_FIRST);
    if(ili9341_display_font == ILI9341_FONT_6X8)
    {
        for(column = 0U; column < 6U; column++)
        {
            uint8 font_data = ascii_font_6x8[character_index][column];

            for(row = 0U; row < 8U; row++)
            {
                display_buffer[column + ((uint16)row * 6U)] =
                    ((font_data & 0x01U) != 0U)
                    ? ili9341_pen_color
                    : ili9341_background_color;
                font_data >>= 1;
            }
        }
    }
    else
    {
        for(column = 0U; column < 8U; column++)
        {
            uint8 font_top = ascii_font_8x16[character_index][column];
            uint8 font_bottom =
                ascii_font_8x16[character_index][column + 8U];

            for(row = 0U; row < 8U; row++)
            {
                display_buffer[column + ((uint16)row * 8U)] =
                    ((font_top & 0x01U) != 0U)
                    ? ili9341_pen_color
                    : ili9341_background_color;
                display_buffer[column + ((uint16)(row + 8U) * 8U)] =
                    ((font_bottom & 0x01U) != 0U)
                    ? ili9341_pen_color
                    : ili9341_background_color;
                font_top >>= 1;
                font_bottom >>= 1;
            }
        }
    }

    ili9341_set_window(
        x,
        y,
        (uint16)(x + font_width - 1U),
        (uint16)(y + font_height - 1U));
    ili9341_set_cs(GPIO_LOW);
    ili9341_set_dc(GPIO_HIGH);
    spi_write_16bit_array(
        ILI9341_SPI,
        display_buffer,
        (uint32)font_width * font_height);
    ili9341_set_cs(GPIO_HIGH);
}

/**
 * @brief Draw a null-terminated printable ASCII string.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param string String to display.
 */
void ili9341_show_string(uint16 x, uint16 y, const char string[])
{
    uint8 font_width = ili9341_get_font_width();
    uint8 font_height = ili9341_get_font_height();

    if((string == NULL) || ((uint32)y + font_height > ili9341_height))
    {
        return;
    }

    while((*string != '\0') && ((uint32)x + font_width <= ili9341_width))
    {
        ili9341_show_char(x, y, *string);
        x = (uint16)(x + font_width);
        string++;
    }
}

/**
 * @brief Draw an unsigned decimal value in a fixed-width field.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Value to display.
 * @param digits Field width from 1 to 10 digits.
 */
void ili9341_show_uint(uint16 x, uint16 y, uint32 value, uint8 digits)
{
    char buffer[11];

    if((digits == 0U) || (digits > 10U))
    {
        return;
    }

    ili9341_format_uint(buffer, value, digits);
    ili9341_show_string(x, y, buffer);
}

/**
 * @brief Draw a signed decimal value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Value to display.
 * @param digits Magnitude field width from 1 to 10 digits.
 */
void ili9341_show_int(uint16 x, uint16 y, int32 value, uint8 digits)
{
    uint32 magnitude;
    uint32 character_count;
    uint8 font_width = ili9341_get_font_width();
    uint8 font_height = ili9341_get_font_height();

    if((digits == 0U) || (digits > 10U))
    {
        return;
    }

    character_count = digits;
    if(value < 0)
    {
        character_count++;
    }

    if(((uint32)x + (character_count * font_width) > ili9341_width)
        || ((uint32)y + font_height > ili9341_height))
    {
        return;
    }

    if(value < 0)
    {
        magnitude = (uint32)(-(value + 1)) + 1U;
        ili9341_show_char(x, y, '-');
        x = (uint16)(x + font_width);
    }
    else
    {
        magnitude = (uint32)value;
    }

    ili9341_show_uint(x, y, magnitude, digits);
}

/**
 * @brief Draw an RGB565 image without scaling.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param image RGB565 pixel buffer in row-major order.
 * @param width Image width.
 * @param height Image height.
 */
void ili9341_show_rgb565_image(
    uint16 x,
    uint16 y,
    const uint16 image[],
    uint16 width,
    uint16 height)
{
    if((image == NULL) || (width == 0U) || (height == 0U)
        || ((uint32)x + width > ili9341_width)
        || ((uint32)y + height > ili9341_height))
    {
        return;
    }

    ili9341_set_window(
        x,
        y,
        (uint16)(x + width - 1U),
        (uint16)(y + height - 1U));
    ili9341_set_cs(GPIO_LOW);
    ili9341_set_dc(GPIO_HIGH);
    spi_write_16bit_array(
        ILI9341_SPI,
        image,
        (uint32)width * height);
    ili9341_set_cs(GPIO_HIGH);
}
