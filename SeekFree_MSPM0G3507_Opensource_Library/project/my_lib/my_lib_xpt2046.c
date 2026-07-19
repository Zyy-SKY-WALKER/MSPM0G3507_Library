/**
 * @file    my_lib_xpt2046.c
 * @brief   Independent XPT2046 resistive touch controller driver.
 */

#include "my_lib_xpt2046.h"

#define XPT2046_COMMAND_X                 (0xD0U)
#define XPT2046_COMMAND_Y                 (0x90U)
#define XPT2046_SAMPLE_COUNT              (5U)
#define XPT2046_FILTER_FIRST_INDEX        (1U)
#define XPT2046_FILTER_LAST_INDEX         (3U)
#define XPT2046_FILTER_DIVISOR            (3U)

static uint8 xpt2046_initialized;

/**
 * @brief Read one 12-bit conversion result from the selected channel.
 * @param command XPT2046 control byte.
 * @return Raw 12-bit ADC value.
 */
static uint16 xpt2046_read_channel(uint8 command)
{
    uint8 transmit_buffer[3] = {command, 0U, 0U};
    uint8 receive_buffer[3] = {0U, 0U, 0U};
    uint16 value;

    gpio_low(XPT2046_CS_PIN);
    spi_transfer_8bit(
        XPT2046_SPI,
        transmit_buffer,
        receive_buffer,
        3U);
    gpio_high(XPT2046_CS_PIN);

    value = ((uint16)receive_buffer[1] << 8)
        | (uint16)receive_buffer[2];
    return (uint16)((value >> 3) & 0x0FFFU);
}

/**
 * @brief Sort a five-element sample buffer in ascending order.
 * @param samples Sample buffer.
 */
static void xpt2046_sort_samples(uint16 samples[])
{
    uint8 outer_index;
    uint8 inner_index;

    for(outer_index = 0U;
        outer_index < (XPT2046_SAMPLE_COUNT - 1U);
        outer_index++)
    {
        for(inner_index = 0U;
            inner_index < (XPT2046_SAMPLE_COUNT - 1U - outer_index);
            inner_index++)
        {
            if(samples[inner_index] > samples[inner_index + 1U])
            {
                uint16 temporary = samples[inner_index];

                samples[inner_index] = samples[inner_index + 1U];
                samples[inner_index + 1U] = temporary;
            }
        }
    }
}

/**
 * @brief Average the middle three values of a sorted sample buffer.
 * @param samples Sorted sample buffer.
 * @return Filtered sample value.
 */
static uint16 xpt2046_filter_samples(const uint16 samples[])
{
    uint32 sum = 0U;
    uint8 index;

    for(index = XPT2046_FILTER_FIRST_INDEX;
        index <= XPT2046_FILTER_LAST_INDEX;
        index++)
    {
        sum += samples[index];
    }

    return (uint16)(sum / XPT2046_FILTER_DIVISOR);
}

/**
 * @brief Apply a two-point linear calibration and clamp the result.
 * @param raw Raw ADC value.
 * @param raw_1 First calibration ADC value.
 * @param pixel_1 First calibration pixel coordinate.
 * @param raw_2 Second calibration ADC value.
 * @param pixel_2 Second calibration pixel coordinate.
 * @param pixel_limit Exclusive coordinate limit.
 * @return Clamped pixel coordinate.
 */
static uint16 xpt2046_map_axis(
    uint16 raw,
    int32 raw_1,
    int32 pixel_1,
    int32 raw_2,
    int32 pixel_2,
    uint16 pixel_limit)
{
    int32 denominator = raw_2 - raw_1;
    int32 coordinate;

    if((denominator == 0) || (pixel_limit == 0U))
    {
        return 0U;
    }

    coordinate = pixel_1
        + (((int32)raw - raw_1) * (pixel_2 - pixel_1)) / denominator;

    if(coordinate < 0)
    {
        coordinate = 0;
    }
    else if(coordinate >= (int32)pixel_limit)
    {
        coordinate = (int32)pixel_limit - 1;
    }

    return (uint16)coordinate;
}

/**
 * @brief Initialize the independent XPT2046 interface.
 */
void xpt2046_init(void)
{
    gpio_init(XPT2046_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(XPT2046_IRQ_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    spi_init(
        XPT2046_SPI,
        XPT2046_SPI_MODE,
        XPT2046_SPI_SPEED,
        XPT2046_SCK_PIN,
        XPT2046_MOSI_PIN,
        XPT2046_MISO_PIN,
        SPI_CS_NULL);

    xpt2046_initialized = 1U;
}

/**
 * @brief Check whether the touch panel is pressed.
 * @return 1 when pressed, otherwise 0.
 */
uint8 xpt2046_is_pressed(void)
{
    if(xpt2046_initialized == 0U)
    {
        return 0U;
    }

    return (gpio_get_level(XPT2046_IRQ_PIN) == GPIO_LOW) ? 1U : 0U;
}

/**
 * @brief Read and filter raw touch coordinates.
 * @param raw_x Destination for X ADC data.
 * @param raw_y Destination for Y ADC data.
 * @return 1 when a valid pressed sample is returned, otherwise 0.
 */
uint8 xpt2046_read_raw(uint16 *raw_x, uint16 *raw_y)
{
    uint16 x_samples[XPT2046_SAMPLE_COUNT];
    uint16 y_samples[XPT2046_SAMPLE_COUNT];
    uint8 index;

    if((raw_x == NULL) || (raw_y == NULL) || !xpt2046_is_pressed())
    {
        return 0U;
    }

    for(index = 0U; index < XPT2046_SAMPLE_COUNT; index++)
    {
        x_samples[index] = xpt2046_read_channel(XPT2046_COMMAND_X);
        y_samples[index] = xpt2046_read_channel(XPT2046_COMMAND_Y);
    }

    xpt2046_sort_samples(x_samples);
    xpt2046_sort_samples(y_samples);
    *raw_x = xpt2046_filter_samples(x_samples);
    *raw_y = xpt2046_filter_samples(y_samples);

    return 1U;
}

/**
 * @brief Convert raw values to calibrated portrait coordinates.
 * @param raw_x Filtered X ADC value.
 * @param raw_y Filtered Y ADC value.
 * @param x Destination for horizontal position.
 * @param y Destination for vertical position.
 */
void xpt2046_convert_point(
    uint16 raw_x,
    uint16 raw_y,
    uint16 *x,
    uint16 *y)
{
    if((x == NULL) || (y == NULL))
    {
        return;
    }

    *x = xpt2046_map_axis(
        raw_x,
        XPT2046_CAL_X1_RAW,
        XPT2046_CAL_X1_PIXEL,
        XPT2046_CAL_X2_RAW,
        XPT2046_CAL_X2_PIXEL,
        XPT2046_SCREEN_WIDTH);
    *y = xpt2046_map_axis(
        raw_y,
        XPT2046_CAL_Y1_RAW,
        XPT2046_CAL_Y1_PIXEL,
        XPT2046_CAL_Y2_RAW,
        XPT2046_CAL_Y2_PIXEL,
        XPT2046_SCREEN_HEIGHT);
}

/**
 * @brief Read one calibrated portrait touch point.
 * @param x Destination for horizontal position.
 * @param y Destination for vertical position.
 * @return 1 when a valid pressed point is returned, otherwise 0.
 */
uint8 xpt2046_read_point(uint16 *x, uint16 *y)
{
    uint16 raw_x;
    uint16 raw_y;

    if((x == NULL) || (y == NULL)
        || !xpt2046_read_raw(&raw_x, &raw_y))
    {
        return 0U;
    }

    xpt2046_convert_point(raw_x, raw_y, x, y);
    return 1U;
}
