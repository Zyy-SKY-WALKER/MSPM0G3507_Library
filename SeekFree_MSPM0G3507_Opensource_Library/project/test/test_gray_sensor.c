/**
 * @file    test_gray_sensor.c
 * @brief   Eight-channel analog grayscale sensor OLED test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_GRAY_SENSOR)

#include "test_gray_sensor.h"

#include "gray_sensor.h"
#include "ml_oled.h"
#include "zf_driver_delay.h"

#define GRAY_SENSOR_TEST_UPDATE_TIME_MS  (500U)
#define GRAY_SENSOR_TEST_PAGE_COUNT      (3U)

/**
 * @brief Display one ADC sample and its software comparator result.
 * @param line OLED text line.
 * @param sensor_index Zero-based sensor index.
 * @param adc_raw 12-bit ADC sample.
 */
static void gray_sensor_test_show_sample(
    uint8 line,
    uint8 sensor_index,
    const gray_sensor_result_struct *result)
{
    uint16 adc_raw = result->analog_raw[sensor_index];
    uint8 digital_value = (uint8)((result->active_mask
        & (uint8)(1U << sensor_index)) != 0U);

    (void)ml_oled_show_string(line, 1U, "D");
    (void)ml_oled_show_uint(line, 2U, (uint32)(sensor_index + 1U), 1U);
    (void)ml_oled_show_string(line, 3U, " A:");
    (void)ml_oled_show_uint(line, 6U, adc_raw, 4U);
    (void)ml_oled_show_string(line, 10U, " B:");
    (void)ml_oled_show_uint(line, 13U, digital_value, 1U);
}

/**
 * @brief Display the latest analog algorithm status.
 * @param result Latest analog grayscale result.
 */
static void gray_sensor_test_show_status(
    const gray_sensor_result_struct *result)
{
    (void)ml_oled_show_string(2U, 1U, "POS:");
    (void)ml_oled_show_float(2U, 5U, result->position, 1U, 2U);
    (void)ml_oled_show_string(3U, 1U, "DEV:");
    (void)ml_oled_show_float(3U, 5U, result->deviation, 1U, 2U);
    (void)ml_oled_show_string(4U, 1U, "STATE:");

    switch (result->status)
    {
        case GRAY_SENSOR_STATUS_VALID:
        {
            (void)ml_oled_show_string(4U, 7U, "VALID");
            break;
        }

        case GRAY_SENSOR_STATUS_ALL_ACTIVE:
        {
            (void)ml_oled_show_string(4U, 7U, "ALL");
            break;
        }

        case GRAY_SENSOR_STATUS_LOST:
        default:
        {
            (void)ml_oled_show_string(4U, 7U, "LOST");
            break;
        }
    }
}

/**
 * @brief Display the software comparator mask with D1 at the left.
 * @param active_mask D1 in bit 0 and D8 in bit 7.
 */
static void gray_sensor_test_show_mask(uint8 active_mask)
{
    uint8 index;

    (void)ml_oled_show_string(1U, 1U, "MASK:");
    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        (void)ml_oled_show_char(
            1U,
            (uint8)(index + 6U),
            (active_mask & (uint8)(1U << index)) != 0U ? '1' : '0');
    }
}

/**
 * @brief Display one page of four-channel analog diagnostics.
 * @param page Zero-based page number.
 * @param adc_raw Latest samples for all eight sensors.
 * @param active_mask Software comparator result mask.
 */
static void gray_sensor_test_show_page(
    uint8 page,
    const gray_sensor_result_struct *result)
{
    uint8 first_sensor = (uint8)(page * 4U);
    uint8 line;
    uint8 sensor_index;

    if (page < 2U)
    {
        for (line = 1U, sensor_index = first_sensor;
             (line <= 4U) && (sensor_index < GRAY_SENSOR_CHANNEL_COUNT);
             line++, sensor_index++)
        {
            gray_sensor_test_show_sample(
                line,
                sensor_index,
                result);
        }
    }
    else
    {
        gray_sensor_test_show_mask(result->active_mask);
        gray_sensor_test_show_status(result);
    }
}

/**
 * @brief Initialize and run the eight-channel grayscale sensor test.
 */
void test_gray_sensor_run(void)
{
    gray_sensor_result_struct result;
    uint8 page = 0U;

    if ((ml_oled_init() == false)
        || (gray_sensor_init() == ZF_FALSE))
    {
        while (true)
        {
        }
    }

    while (true)
    {
        if (gray_sensor_sample(&result) != ZF_FALSE)
        {
            gray_sensor_test_show_page(page, &result);
        }

        page++;
        if (page >= GRAY_SENSOR_TEST_PAGE_COUNT)
        {
            page = 0U;
        }
        system_delay_ms(GRAY_SENSOR_TEST_UPDATE_TIME_MS);
    }
}

#endif
