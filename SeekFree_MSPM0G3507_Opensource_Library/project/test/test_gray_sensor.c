/**
 * @file    test_gray_sensor.c
 * @brief   Eight-channel grayscale sensor TFT test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_GRAY_SENSOR)

#include "test_gray_sensor.h"

#include "gray_sensor.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define GRAY_SENSOR_TEST_UPDATE_TIME_MS  (100U)

/**
 * @brief Round a floating-point display value to a signed integer.
 * @param value Value to round.
 * @return Rounded signed integer.
 */
static int32 gray_sensor_test_round(float value)
{
    if (value >= 0.0F)
    {
        return (int32)(value + 0.5F);
    }

    return (int32)(value - 0.5F);
}

/**
 * @brief Display an eight-bit mask with D1 at the left side.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param mask Mask to display.
 */
static void gray_sensor_test_show_mask(uint16 x, uint16 y, uint8 mask)
{
    uint8 index;

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        ili9341_show_char(
            (uint16)(x + ((uint16)index * 8U)),
            y,
            (mask & (uint8)(1U << index)) != 0U ? '1' : '0');
    }
}

/**
 * @brief Clear and display one signed result value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed value.
 * @param digits Magnitude field width.
 */
static void gray_sensor_test_show_value(
    uint16 x,
    uint16 y,
    int32 value,
    uint8 digits)
{
    uint16 field_width = (uint16)((digits + 1U) * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, value, digits);
}

/**
 * @brief Display the current grayscale classification.
 * @param status Current sensor status.
 */
static void gray_sensor_test_show_status(
    gray_sensor_status_enum status)
{
    ili9341_fill_rect(
        104U,
        232U,
        231U,
        247U,
        ILI9341_COLOR_BLACK);

    switch (status)
    {
        case GRAY_SENSOR_STATUS_LOST:
        {
            ili9341_show_string(104U, 232U, "LOST");
            break;
        }

        case GRAY_SENSOR_STATUS_VALID:
        {
            ili9341_show_string(104U, 232U, "VALID");
            break;
        }

        case GRAY_SENSOR_STATUS_ALL_ACTIVE:
        {
            ili9341_show_string(104U, 232U, "ALL ACTIVE");
            break;
        }

        default:
        {
            ili9341_show_string(104U, 232U, "UNKNOWN");
            break;
        }
    }
}

/**
 * @brief Initialize and run the eight-channel grayscale sensor test.
 */
void test_gray_sensor_run(void)
{
    gray_sensor_result_struct result;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "GRAY SENSOR TEST");

    if (gray_sensor_init() == ZF_FALSE)
    {
        ili9341_show_string(8U, 48U, "CONFIG D1-D8 PINS");
        ili9341_show_string(8U, 80U, "SET CONFIGURED=1");

        while (true)
        {
        }
    }

    ili9341_show_string(8U, 40U, "RAW    :");
    ili9341_show_string(8U, 72U, "ACTIVE :");
    ili9341_show_string(8U, 104U, "COUNT  :");
    ili9341_show_string(8U, 136U, "POS x10:");
    ili9341_show_string(8U, 168U, "DEV x10:");
    ili9341_show_string(8U, 200U, "WEIGHT :");
    ili9341_show_string(8U, 232U, "STATUS :");
    ili9341_show_string(8U, 280U, "D1 LEFT, D8 RIGHT");

    while (true)
    {
        if (gray_sensor_sample(&result) != 0U)
        {
            gray_sensor_test_show_mask(104U, 40U, result.raw_mask);
            gray_sensor_test_show_mask(104U, 72U, result.active_mask);
            gray_sensor_test_show_value(
                104U,
                104U,
                result.active_count,
                2U);
            gray_sensor_test_show_value(
                104U,
                136U,
                gray_sensor_test_round(result.position * 10.0F),
                3U);
            gray_sensor_test_show_value(
                104U,
                168U,
                gray_sensor_test_round(result.deviation * 10.0F),
                3U);
            gray_sensor_test_show_value(
                104U,
                200U,
                result.weighted_sum,
                2U);
            gray_sensor_test_show_status(result.status);
        }

        system_delay_ms(GRAY_SENSOR_TEST_UPDATE_TIME_MS);
    }
}

#endif
