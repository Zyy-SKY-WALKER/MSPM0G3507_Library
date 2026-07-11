/**
 * @file    test_line_tracker.c
 * @brief   Pure line tracker exhaustive and sequence tests.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_LINE_TRACKER)

#include "test_line_tracker.h"

#include "gray_sensor.h"
#include "line_tracker.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define LINE_TRACKER_TEST_MAX_TARGET   (300.0F)
#define LINE_TRACKER_TEST_STEP_TIME_MS (1000U)
#define LINE_TRACKER_TEST_FLOAT_EPSILON (0.01F)
#define LINE_TRACKER_TEST_NORMALIZE_SCALE (5.0F / 3.5F)

static const uint8 line_tracker_test_masks[] =
{
    0x18U,
    0x08U,
    0x04U,
    0x01U,
    0x80U,
    0x00U,
    0xFFU,
};

/**
 * @brief Check that one output is finite and in configured bounds.
 * @param output Tracker output to validate.
 * @return Nonzero when valid.
 */
static uint8 line_tracker_test_output_is_valid(
    const line_tracker_output_struct *output)
{
    if ((output->left_target_mm_s != output->left_target_mm_s)
        || (output->right_target_mm_s != output->right_target_mm_s)
        || (output->left_target_mm_s < -LINE_TRACKER_TEST_MAX_TARGET)
        || (output->left_target_mm_s > LINE_TRACKER_TEST_MAX_TARGET)
        || (output->right_target_mm_s < -LINE_TRACKER_TEST_MAX_TARGET)
        || (output->right_target_mm_s > LINE_TRACKER_TEST_MAX_TARGET))
    {
        return ZF_FALSE;
    }

    return ZF_TRUE;
}

/**
 * @brief Return the expected speed band for normalized deviation.
 * @param deviation Normalized signed deviation.
 * @return Expected band index.
 */
static uint8 line_tracker_test_expected_band(float deviation)
{
    float absolute = deviation >= 0.0F ? deviation : -deviation;

    if (absolute <= 1.0F)
    {
        return 0U;
    }
    if (absolute <= 2.0F)
    {
        return 1U;
    }
    if (absolute <= 3.0F)
    {
        return 2U;
    }
    if (absolute <= 4.0F)
    {
        return 3U;
    }

    return 4U;
}

/**
 * @brief Run all 256 masks and representative state transitions.
 * @return Failure count.
 */
static uint32 line_tracker_test_self_check(void)
{
    gray_sensor_result_struct sensor;
    line_tracker_output_struct output;
    line_tracker_status_struct status;
    uint32 failures = 0U;
    uint16 mask;
    uint16 index;

    for (mask = 0U; mask <= 0xFFU; mask++)
    {
        float expected_normalized;
        float normalized_error;

        gray_sensor_calculate((uint8)mask, &sensor);
        line_tracker_reset();
        if ((line_tracker_update(&sensor, &output) == ZF_FALSE)
            || (line_tracker_test_output_is_valid(&output) == ZF_FALSE))
        {
            failures++;
        }

        line_tracker_get_status(&status);
        if (sensor.status == GRAY_SENSOR_STATUS_VALID)
        {
            expected_normalized =
                sensor.deviation * LINE_TRACKER_TEST_NORMALIZE_SCALE;
            normalized_error =
                status.normalized_deviation - expected_normalized;
            if (normalized_error < 0.0F)
            {
                normalized_error = -normalized_error;
            }

            if ((status.state != LINE_TRACKER_STATE_TRACKING)
                || ((output.left_target_mm_s == 0.0F)
                    && (output.right_target_mm_s == 0.0F))
                || (output.left_target_mm_s < 0.0F)
                || (output.right_target_mm_s < 0.0F)
                || (normalized_error > LINE_TRACKER_TEST_FLOAT_EPSILON)
                || (status.speed_band
                    != line_tracker_test_expected_band(expected_normalized)))
            {
                failures++;
            }

            if (((sensor.deviation < 0.0F)
                    && (output.left_target_mm_s
                        >= output.right_target_mm_s))
                || ((sensor.deviation > 0.0F)
                    && (output.left_target_mm_s
                        <= output.right_target_mm_s))
                || ((sensor.deviation == 0.0F)
                    && (output.left_target_mm_s
                        != output.right_target_mm_s)))
            {
                failures++;
            }
        }
        else if ((output.left_target_mm_s != 0.0F)
            || (output.right_target_mm_s != 0.0F))
        {
            failures++;
        }
    }

    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_reset();
    line_tracker_update(&sensor, &output);
    if (output.left_target_mm_s != output.right_target_mm_s)
    {
        failures++;
    }

    gray_sensor_calculate(0x01U, &sensor);
    line_tracker_reset();
    line_tracker_update(&sensor, &output);
    if (output.left_target_mm_s >= output.right_target_mm_s)
    {
        failures++;
    }

    gray_sensor_calculate(0x80U, &sensor);
    line_tracker_reset();
    line_tracker_update(&sensor, &output);
    if (output.left_target_mm_s <= output.right_target_mm_s)
    {
        failures++;
    }

    gray_sensor_calculate(0xFFU, &sensor);
    line_tracker_reset();
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_ALL_ACTIVE)
        || (output.left_target_mm_s != 0.0F)
        || (output.right_target_mm_s != 0.0F))
    {
        failures++;
    }

    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_reset();
    line_tracker_update(&sensor, &output);
    gray_sensor_calculate(0x00U, &sensor);
    for (index = 0U; index < 3U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if (status.state != LINE_TRACKER_STATE_LOST_ARC)
    {
        failures++;
    }

    for (index = 0U; index < 50U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if (status.state != LINE_TRACKER_STATE_LOST_PIVOT)
    {
        failures++;
    }

    gray_sensor_calculate(0x18U, &sensor);
    for (index = 0U; index < 3U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if (status.state != LINE_TRACKER_STATE_TRACKING)
    {
        failures++;
    }

    line_tracker_reset();
    gray_sensor_calculate(0x00U, &sensor);
    for (index = 0U; index < 3U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    for (index = 0U; index < 499U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_FAULT)
        || (output.left_target_mm_s != 0.0F)
        || (output.right_target_mm_s != 0.0F))
    {
        failures++;
    }

    return failures;
}

/**
 * @brief Display one eight-bit mask with D1 at the left side.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param mask Mask to display.
 */
static void line_tracker_test_show_mask(uint16 x, uint16 y, uint8 mask)
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
 * @brief Clear and display one signed integer field.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed display value.
 */
static void line_tracker_test_show_value(
    uint16 x,
    uint16 y,
    int32 value)
{
    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + 55U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, value, 6U);
}

/**
 * @brief Initialize and run the synthetic line tracker test.
 */
void test_line_tracker_run(void)
{
    gray_sensor_result_struct sensor;
    line_tracker_output_struct output;
    line_tracker_status_struct status;
    uint32 failures;
    uint8 mask_index = 0U;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "LINE TRACKER TEST");
    ili9341_show_string(8U, 40U, "FAIL   :");
    ili9341_show_string(8U, 72U, "MASK   :");
    ili9341_show_string(8U, 104U, "DEVx10 :");
    ili9341_show_string(8U, 136U, "LEFT   :");
    ili9341_show_string(8U, 168U, "RIGHT  :");
    ili9341_show_string(8U, 200U, "STATE  :");
    ili9341_show_string(8U, 232U, "BAND   :");
    ili9341_show_string(8U, 280U, "SYNTHETIC MASKS");

    line_tracker_init(NULL);
    failures = line_tracker_test_self_check();
    ili9341_show_uint(104U, 40U, failures, 4U);

    while (true)
    {
        uint8 mask = line_tracker_test_masks[mask_index];

        gray_sensor_calculate(mask, &sensor);
        line_tracker_reset();
        line_tracker_update(&sensor, &output);
        line_tracker_get_status(&status);

        line_tracker_test_show_mask(104U, 72U, mask);
        line_tracker_test_show_value(
            104U,
            104U,
            (int32)(sensor.deviation * 10.0F));
        line_tracker_test_show_value(
            104U,
            136U,
            (int32)output.left_target_mm_s);
        line_tracker_test_show_value(
            104U,
            168U,
            (int32)output.right_target_mm_s);
        line_tracker_test_show_value(104U, 200U, status.state);
        line_tracker_test_show_value(104U, 232U, status.speed_band);

        mask_index++;
        if (mask_index >= (sizeof(line_tracker_test_masks)
            / sizeof(line_tracker_test_masks[0])))
        {
            mask_index = 0U;
        }

        system_delay_ms(LINE_TRACKER_TEST_STEP_TIME_MS);
    }
}

#endif
