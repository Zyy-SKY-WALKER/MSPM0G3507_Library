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

#define LINE_TRACKER_TEST_MAX_TARGET   (800.0F)
#define LINE_TRACKER_TEST_MAX_CORRECTION (400.0F)
#define LINE_TRACKER_TEST_DEFAULT_ARC_SAMPLES (300U)
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

static const float line_tracker_test_default_base_speed[] =
{
    450.0F,
    400.0F,
    380.0F,
    340.0F,
    320.0F,
};

static const float line_tracker_test_default_kp[] =
{
    30.0F,
    38.0F,
    43.0F,
    55.0F,
    68.0F,
};

/**
 * @brief Compare two floats with the test tolerance.
 * @param actual Actual value.
 * @param expected Expected value.
 * @return Nonzero when the values are sufficiently close.
 */
static uint8 line_tracker_test_float_is_near(
    float actual,
    float expected)
{
    float difference = actual - expected;

    if (difference < 0.0F)
    {
        difference = -difference;
    }

    return (uint8)(difference <= LINE_TRACKER_TEST_FLOAT_EPSILON);
}

/**
 * @brief Clamp one expected forward wheel target.
 * @param value Unbounded target value.
 * @return Target limited to the default zero-to-maximum range.
 */
static float line_tracker_test_clamp_target(float value)
{
    if (value > LINE_TRACKER_TEST_MAX_TARGET)
    {
        return LINE_TRACKER_TEST_MAX_TARGET;
    }
    if (value < 0.0F)
    {
        return 0.0F;
    }

    return value;
}

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
 * @brief Fill a deterministic configuration for PID behavior checks.
 * @param config Destination configuration.
 */
static void line_tracker_test_prepare_pid_config(
    line_tracker_config_struct *config)
{
    uint8 index;

    for (index = 0U; index < LINE_TRACKER_SPEED_BAND_COUNT; index++)
    {
        config->base_speed_mm_s[index] = 100.0F;
        config->pid_kp[index] = 0.0F;
    }

    config->pid_ki = 100.0F;
    config->pid_kd = 0.0F;
    config->pid_integral_limit_mm_s = 50.0F;
    config->pid_derivative_filter_alpha = 0.25F;
    config->max_target_mm_s = 300.0F;
    config->max_correction_mm_s = 20.0F;
    config->max_target_accel_mm_s2 = 0.0F;
    config->arc_outer_speed_mm_s = 160.0F;
    config->arc_inner_speed_mm_s = -60.0F;
    config->pivot_speed_mm_s = 100.0F;
    config->lost_debounce_samples = 3U;
    config->reacquire_samples = 3U;
    config->arc_duration_samples = 50U;
    config->search_timeout_samples = 500U;
    config->default_search_direction = LINE_TRACKER_DIRECTION_RIGHT;
}

/**
 * @brief Fill the fixed configuration used by exhaustive output checks.
 * @param config Destination configuration.
 */
static void line_tracker_test_prepare_tracking_config(
    line_tracker_config_struct *config)
{
    uint8 index;

    line_tracker_test_prepare_pid_config(config);
    for (index = 0U; index < LINE_TRACKER_SPEED_BAND_COUNT; index++)
    {
        config->base_speed_mm_s[index] =
            line_tracker_test_default_base_speed[index];
        config->pid_kp[index] = line_tracker_test_default_kp[index];
    }

    config->pid_ki = 0.0F;
    config->pid_kd = 0.0F;
    config->max_target_mm_s = LINE_TRACKER_TEST_MAX_TARGET;
    config->max_correction_mm_s = LINE_TRACKER_TEST_MAX_CORRECTION;
    config->max_target_accel_mm_s2 = 0.0F;
    config->arc_outer_speed_mm_s = 400.0F;
    config->arc_inner_speed_mm_s = -100.0F;
    config->pivot_speed_mm_s = 300.0F;
    config->arc_duration_samples = LINE_TRACKER_TEST_DEFAULT_ARC_SAMPLES;
}

/**
 * @brief Check whether every exposed PID state value is reset.
 * @param status Tracker status snapshot.
 * @return Nonzero when PID state is fully cleared.
 */
static uint8 line_tracker_test_pid_is_reset(
    const line_tracker_status_struct *status)
{
    return (uint8)(
        (status->pid_integral_mm_s == 0.0F)
        && (status->pid_filtered_derivative == 0.0F)
        && (status->correction_mm_s == 0.0F));
}

/**
 * @brief Create nonzero integral and derivative state.
 * @param config PID test configuration.
 * @param sensor Working sensor value.
 * @param output Working tracker output.
 */
static void line_tracker_test_prime_pid(
    const line_tracker_config_struct *config,
    gray_sensor_result_struct *sensor,
    line_tracker_output_struct *output)
{
    line_tracker_init(config);
    gray_sensor_calculate(0x80U, sensor);
    line_tracker_update(sensor, output);
    gray_sensor_calculate(0x18U, sensor);
    line_tracker_update(sensor, output);
}

/**
 * @brief Check PID math, anti-windup, filtering, and reset paths.
 * @return Failure count.
 */
static uint32 line_tracker_test_pid_self_check(void)
{
    gray_sensor_result_struct sensor;
    line_tracker_config_struct config;
    line_tracker_output_struct held_output;
    line_tracker_output_struct output;
    line_tracker_status_struct status;
    uint32 failures = 0U;
    uint16 index;

    /* Integral limiting alone must not report output saturation. */
    line_tracker_test_prepare_pid_config(&config);
    config.max_target_mm_s = 1000.0F;
    config.max_correction_mm_s = 140.0F;
    line_tracker_init(&config);
    gray_sensor_calculate(0x80U, &sensor);
    for (index = 0U; index < 20U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if ((line_tracker_test_float_is_near(
            status.pid_integral_mm_s,
            50.0F) == 0U)
        || (status.output_limited != 0U))
    {
        failures++;
    }

    /* Saturation freezes same-direction integration but permits unwind. */
    line_tracker_test_prepare_pid_config(&config);
    line_tracker_init(&config);
    gray_sensor_calculate(0x80U, &sensor);
    for (index = 0U; index < 10U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if ((line_tracker_test_float_is_near(
            status.pid_integral_mm_s,
            20.0F) == 0U)
        || (status.output_limited == 0U))
    {
        failures++;
    }
    gray_sensor_calculate(0x01U, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if (line_tracker_test_float_is_near(
            status.pid_integral_mm_s,
            15.0F) == 0U)
    {
        failures++;
    }

    /* The first D sample is kick-free; later samples follow the filter. */
    line_tracker_test_prepare_pid_config(&config);
    for (index = 0U; index < LINE_TRACKER_SPEED_BAND_COUNT; index++)
    {
        config.base_speed_mm_s[index] = 500.0F;
    }
    config.pid_ki = 0.0F;
    config.pid_kd = 1.0F;
    config.max_target_mm_s = 1000.0F;
    config.max_correction_mm_s = 500.0F;
    line_tracker_init(&config);
    gray_sensor_calculate(0x80U, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((status.pid_filtered_derivative != 0.0F)
        || (status.correction_mm_s != 0.0F))
    {
        failures++;
    }
    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((line_tracker_test_float_is_near(
            status.pid_filtered_derivative,
            -125.0F) == 0U)
        || (line_tracker_test_float_is_near(
            status.correction_mm_s,
            -125.0F) == 0U))
    {
        failures++;
    }
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if (line_tracker_test_float_is_near(
            status.pid_filtered_derivative,
            -93.75F) == 0U)
    {
        failures++;
    }

    /* All-active input must clear every PID state value. */
    line_tracker_test_prepare_pid_config(&config);
    config.pid_kd = 1.0F;
    config.max_correction_mm_s = 140.0F;
    line_tracker_test_prime_pid(&config, &sensor, &output);
    gray_sensor_calculate(0xFFU, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if (line_tracker_test_pid_is_reset(&status) == 0U)
    {
        failures++;
    }

    /* Explicit tracker reset must clear every PID state value. */
    line_tracker_test_prime_pid(&config, &sensor, &output);
    line_tracker_reset();
    line_tracker_get_status(&status);
    if (line_tracker_test_pid_is_reset(&status) == 0U)
    {
        failures++;
    }

    /* Lost-line debounce retains wheel targets while clearing PID history. */
    line_tracker_test_prime_pid(&config, &sensor, &output);
    held_output = output;
    gray_sensor_calculate(0x00U, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((line_tracker_test_pid_is_reset(&status) == 0U)
        || (line_tracker_test_float_is_near(
                output.left_target_mm_s,
                held_output.left_target_mm_s) == 0U)
        || (line_tracker_test_float_is_near(
                output.right_target_mm_s,
                held_output.right_target_mm_s) == 0U))
    {
        failures++;
    }

    /* Search and pivot outputs remain independent of the tracking PID. */
    line_tracker_update(&sensor, &output);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_LOST_ARC)
        || (line_tracker_test_pid_is_reset(&status) == 0U))
    {
        failures++;
    }
    if ((line_tracker_test_float_is_near(
            output.left_target_mm_s,
            160.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            -60.0F) == 0U))
    {
        failures++;
    }
    for (index = 0U; index < 50U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_LOST_PIVOT)
        || (line_tracker_test_pid_is_reset(&status) == 0U)
        || (line_tracker_test_float_is_near(
            output.left_target_mm_s,
            100.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            -100.0F) == 0U))
    {
        failures++;
    }

    /* Reacquisition waits for valid samples and starts D without a kick. */
    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((output.left_target_mm_s != 0.0F)
        || (output.right_target_mm_s != 0.0F)
        || (line_tracker_test_pid_is_reset(&status) == 0U))
    {
        failures++;
    }
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_TRACKING)
        || (status.pid_filtered_derivative != 0.0F))
    {
        failures++;
    }

    /* Invalid sensor data latches fault and clears all controller history. */
    line_tracker_test_prime_pid(&config, &sensor, &output);
    sensor.active_count++;
    if (line_tracker_update(&sensor, &output) != ZF_FALSE)
    {
        failures++;
    }
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_FAULT)
        || (line_tracker_test_pid_is_reset(&status) == 0U))
    {
        failures++;
    }

    /* Signed inner speed accepts the lower bound and rejects overflow. */
    line_tracker_test_prepare_pid_config(&config);
    config.arc_inner_speed_mm_s = -301.0F;
    if (line_tracker_set_config(&config) != ZF_FALSE)
    {
        failures++;
    }
    config.arc_inner_speed_mm_s = -300.0F;
    if (line_tracker_set_config(&config) == ZF_FALSE)
    {
        failures++;
    }

    line_tracker_init(NULL);
    return failures;
}

/**
 * @brief Check normal-tracking target slew and immediate safety outputs.
 * @return Failure count.
 */
static uint32 line_tracker_test_target_slew_self_check(void)
{
    gray_sensor_result_struct sensor;
    line_tracker_config_struct config;
    line_tracker_output_struct output;
    line_tracker_status_struct status;
    uint32 failures = 0U;
    uint8 index;

    line_tracker_test_prepare_pid_config(&config);
    for (index = 0U; index < LINE_TRACKER_SPEED_BAND_COUNT; index++)
    {
        config.base_speed_mm_s[index] = 100.0F;
        config.pid_kp[index] = 0.0F;
    }
    config.max_target_accel_mm_s2 = 2000.0F;
    config.lost_debounce_samples = 1U;
    config.reacquire_samples = 1U;
    line_tracker_init(&config);

    /* At 2,000 mm/s squared, each 10 ms tracking update moves 20 mm/s. */
    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_update(&sensor, &output);
    line_tracker_get_status(&status);
    if ((line_tracker_test_float_is_near(
            output.left_target_mm_s,
            20.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            20.0F) == 0U)
        || (status.output_limited == 0U))
    {
        failures++;
    }
    line_tracker_update(&sensor, &output);
    if ((line_tracker_test_float_is_near(
            output.left_target_mm_s,
            40.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            40.0F) == 0U))
    {
        failures++;
    }

    /* Stop, search, and fault targets remain immediate. */
    gray_sensor_calculate(0xFFU, &sensor);
    line_tracker_update(&sensor, &output);
    if ((output.left_target_mm_s != 0.0F)
        || (output.right_target_mm_s != 0.0F))
    {
        failures++;
    }

    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_update(&sensor, &output);
    if ((line_tracker_test_float_is_near(
            output.left_target_mm_s,
            20.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            20.0F) == 0U))
    {
        failures++;
    }

    gray_sensor_calculate(0x00U, &sensor);
    line_tracker_update(&sensor, &output);
    if ((line_tracker_test_float_is_near(
            output.left_target_mm_s,
            160.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            -60.0F) == 0U))
    {
        failures++;
    }

    gray_sensor_calculate(0x18U, &sensor);
    line_tracker_update(&sensor, &output);
    sensor.active_count++;
    if ((line_tracker_update(&sensor, &output) != ZF_FALSE)
        || (output.left_target_mm_s != 0.0F)
        || (output.right_target_mm_s != 0.0F))
    {
        failures++;
    }

    line_tracker_init(NULL);
    return failures;
}

/**
 * @brief Run all 256 masks and representative state transitions.
 * @return Failure count.
 */
static uint32 line_tracker_test_self_check(void)
{
    gray_sensor_result_struct sensor;
    line_tracker_config_struct config;
    line_tracker_output_struct output;
    line_tracker_status_struct status;
    uint32 failures = 0U;
    uint16 mask;
    uint16 index;

    line_tracker_test_prepare_tracking_config(&config);
    line_tracker_init(&config);

    for (mask = 0U; mask <= 0xFFU; mask++)
    {
        float expected_correction;
        float expected_left;
        float expected_normalized;
        float expected_right;
        float normalized_error;
        uint8 expected_band;

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
            expected_band =
                line_tracker_test_expected_band(expected_normalized);
            expected_correction = expected_normalized
                * line_tracker_test_default_kp[expected_band];
            if (expected_correction > LINE_TRACKER_TEST_MAX_CORRECTION)
            {
                expected_correction = LINE_TRACKER_TEST_MAX_CORRECTION;
            }
            else if (expected_correction
                < -LINE_TRACKER_TEST_MAX_CORRECTION)
            {
                expected_correction = -LINE_TRACKER_TEST_MAX_CORRECTION;
            }
            expected_left = line_tracker_test_clamp_target(
                line_tracker_test_default_base_speed[expected_band]
                + expected_correction);
            expected_right = line_tracker_test_clamp_target(
                line_tracker_test_default_base_speed[expected_band]
                - expected_correction);
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
                || (status.speed_band != expected_band)
                || (line_tracker_test_float_is_near(
                        status.correction_mm_s,
                        expected_correction) == 0U)
                || (line_tracker_test_float_is_near(
                        output.left_target_mm_s,
                        expected_left) == 0U)
                || (line_tracker_test_float_is_near(
                        output.right_target_mm_s,
                        expected_right) == 0U))
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
    if ((status.state != LINE_TRACKER_STATE_LOST_ARC)
        || (line_tracker_test_float_is_near(
            output.left_target_mm_s,
            400.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            -100.0F) == 0U))
    {
        failures++;
    }

    for (index = 0U;
        index < LINE_TRACKER_TEST_DEFAULT_ARC_SAMPLES;
        index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_LOST_PIVOT)
        || (line_tracker_test_float_is_near(
            output.left_target_mm_s,
            300.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            -300.0F) == 0U))
    {
        failures++;
    }

    gray_sensor_calculate(0x01U, &sensor);
    line_tracker_reset();
    line_tracker_update(&sensor, &output);
    gray_sensor_calculate(0x00U, &sensor);
    for (index = 0U; index < 3U; index++)
    {
        line_tracker_update(&sensor, &output);
    }
    line_tracker_get_status(&status);
    if ((status.state != LINE_TRACKER_STATE_LOST_ARC)
        || (line_tracker_test_float_is_near(
            output.left_target_mm_s,
            -100.0F) == 0U)
        || (line_tracker_test_float_is_near(
            output.right_target_mm_s,
            400.0F) == 0U))
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
    failures += line_tracker_test_pid_self_check();
    failures += line_tracker_test_target_slew_self_check();
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
