/**
 * @file    gray_sensor.c
 * @brief   Eight-channel analog grayscale sensor acquisition.
 */

#include "gray_sensor.h"

static const adc_pin_enum gray_sensor_adc_pins[
    GRAY_SENSOR_CHANNEL_COUNT] =
{
    GRAY_SENSOR_D1_ADC_PIN,
    GRAY_SENSOR_D2_ADC_PIN,
    GRAY_SENSOR_D3_ADC_PIN,
    GRAY_SENSOR_D4_ADC_PIN,
    GRAY_SENSOR_D5_ADC_PIN,
    GRAY_SENSOR_D6_ADC_PIN,
    GRAY_SENSOR_D7_ADC_PIN,
    GRAY_SENSOR_D8_ADC_PIN,
};

static uint8 gray_sensor_initialized;

/**
 * @brief Validate all configured ADC pins and analog parameters.
 * @return ZF_TRUE when every pin and parameter is valid.
 */
static uint8 gray_sensor_adc_config_is_valid(void)
{
    uint8 left_index;
    uint8 right_index;
    gpio_pin_enum left_gpio_pin;
    gpio_pin_enum right_gpio_pin;

    if (GRAY_SENSOR_ADC_PINS_CONFIGURED == 0U)
    {
        return ZF_FALSE;
    }

    if ((GRAY_SENSOR_ADC_MIN_VALUE >= GRAY_SENSOR_ADC_MAX_VALUE)
        || (GRAY_SENSOR_ADC_COMPARE_RAW > GRAY_SENSOR_ADC_MAX_VALUE))
    {
        return ZF_FALSE;
    }

    for (left_index = 0U;
        left_index < GRAY_SENSOR_CHANNEL_COUNT;
        left_index++)
    {
        left_gpio_pin = (gpio_pin_enum)(gray_sensor_adc_pins[left_index]
            & ADC_PIN_INDEX_MASK);
        if ((left_gpio_pin < A0) || (left_gpio_pin >= GPIO_MAX))
        {
            return ZF_FALSE;
        }

        for (right_index = (uint8)(left_index + 1U);
            right_index < GRAY_SENSOR_CHANNEL_COUNT;
            right_index++)
        {
            right_gpio_pin = (gpio_pin_enum)(gray_sensor_adc_pins[right_index]
                & ADC_PIN_INDEX_MASK);
            if (left_gpio_pin == right_gpio_pin)
            {
                return ZF_FALSE;
            }
        }
    }

    return ZF_TRUE;
}

/**
 * @brief Initialize all configured analog grayscale inputs.
 * @return ZF_TRUE when the ADC pin configuration is valid.
 */
uint8 gray_sensor_init(void)
{
    uint8 index;

    gray_sensor_initialized = 0U;
    if (gray_sensor_adc_config_is_valid() == ZF_FALSE)
    {
        return ZF_FALSE;
    }

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        adc_init(gray_sensor_adc_pins[index], ADC_12BIT);
    }

    gray_sensor_initialized = 1U;
    return ZF_TRUE;
}

/**
 * @brief Clear one result before calculating a new sample.
 * @param result Result to clear.
 * @param mode Calculation mode for the new result.
 */
static void gray_sensor_result_reset(
    gray_sensor_result_struct *result,
    gray_sensor_result_mode_enum mode)
{
    uint8 index;

    result->raw_mask = 0U;
    result->active_mask = 0U;
    result->active_count = 0U;
    result->weighted_sum = 0U;
    result->position = 0.0F;
    result->deviation = 0.0F;
    result->status = GRAY_SENSOR_STATUS_LOST;
    result->calculation_mode = mode;
    result->analog_weight_sum = 0U;
    result->analog_weighted_sum = 0U;
    result->analog_max_weight = 0U;
    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        result->analog_raw[index] = 0U;
    }
}

/**
 * @brief Calculate position and deviation from a normalized active mask.
 * @param active_mask D1 in bit 0 and D8 in bit 7.
 * @param result Destination result structure.
 */
void gray_sensor_calculate(
    uint8 active_mask,
    gray_sensor_result_struct *result)
{
    uint8 index;

    if (result == NULL)
    {
        return;
    }

    gray_sensor_result_reset(
        result,
        GRAY_SENSOR_RESULT_MODE_DIGITAL);
    result->active_mask = active_mask;
    result->raw_mask = active_mask;

    /* The mean active channel index is the binary line centroid. */
    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        if ((active_mask & (uint8)(1U << index)) != 0U)
        {
            result->active_count++;
            result->weighted_sum =
                (uint8)(result->weighted_sum + index);
        }
    }

    if (result->active_count == 0U)
    {
        return;
    }

    if (result->active_count == GRAY_SENSOR_CHANNEL_COUNT)
    {
        result->position = 3.5F;
        result->status = GRAY_SENSOR_STATUS_ALL_ACTIVE;
        return;
    }

    result->position =
        (float)result->weighted_sum / (float)result->active_count;
    result->deviation = result->position - 3.5F;
    result->status = GRAY_SENSOR_STATUS_VALID;
}

/**
 * @brief Calculate position and deviation from eight ADC samples.
 * @param adc_raw Eight 12-bit ADC samples in D1-D8 order.
 * @param result Destination result structure.
 */
void gray_sensor_calculate_analog(
    const uint16 adc_raw[GRAY_SENSOR_CHANNEL_COUNT],
    gray_sensor_result_struct *result)
{
    uint8 index;
    uint16 weight;
    uint32 weight_sum = 0U;
    uint32 weighted_position_sum = 0U;

    if ((adc_raw == NULL) || (result == NULL))
    {
        return;
    }

    gray_sensor_result_reset(
        result,
        GRAY_SENSOR_RESULT_MODE_ANALOG);

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        weight = adc_raw[index];

        result->analog_raw[index] = adc_raw[index];

        if (adc_raw[index] >= GRAY_SENSOR_ADC_COMPARE_RAW)
        {
            result->active_mask |= (uint8)(1U << index);
            result->active_count++;
            result->weighted_sum =
                (uint8)(result->weighted_sum + index);
        }

        if (weight > result->analog_max_weight)
        {
            result->analog_max_weight = weight;
        }
        weight_sum += weight;
        weighted_position_sum += (uint32)weight * (uint32)index;
    }

    result->raw_mask = result->active_mask;
    result->analog_weight_sum = weight_sum;
    result->analog_weighted_sum = weighted_position_sum;

    if (result->active_count == GRAY_SENSOR_CHANNEL_COUNT)
    {
        result->position = 3.5F;
        result->deviation = 0.0F;
        result->status = GRAY_SENSOR_STATUS_ALL_ACTIVE;
        return;
    }

    if ((result->active_count == 0U) || (weight_sum == 0U))
    {
        return;
    }

    result->position =
        (float)weighted_position_sum / (float)weight_sum;
    result->deviation = result->position - 3.5F;
    result->status = GRAY_SENSOR_STATUS_VALID;
}

/**
 * @brief Sample all eight ADC inputs and calculate analog line position.
 * @param result Destination result structure.
 * @return ZF_TRUE when acquisition is initialized.
 */
uint8 gray_sensor_sample(gray_sensor_result_struct *result)
{
    uint16 adc_raw[GRAY_SENSOR_CHANNEL_COUNT];
    uint8 index;

    if ((result == NULL) || (gray_sensor_initialized == 0U))
    {
        return ZF_FALSE;
    }

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        adc_raw[index] = adc_convert(gray_sensor_adc_pins[index]);
    }

    gray_sensor_calculate_analog(adc_raw, result);
    return ZF_TRUE;
}
