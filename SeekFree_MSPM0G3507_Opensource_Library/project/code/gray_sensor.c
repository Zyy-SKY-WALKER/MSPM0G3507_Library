/**
 * @file    gray_sensor.c
 * @brief   Eight-channel digital grayscale sensor acquisition.
 */

#include "gray_sensor.h"

static const gpio_pin_enum gray_sensor_pins[GRAY_SENSOR_CHANNEL_COUNT] =
{
    GRAY_SENSOR_D1_PIN,
    GRAY_SENSOR_D2_PIN,
    GRAY_SENSOR_D3_PIN,
    GRAY_SENSOR_D4_PIN,
    GRAY_SENSOR_D5_PIN,
    GRAY_SENSOR_D6_PIN,
    GRAY_SENSOR_D7_PIN,
    GRAY_SENSOR_D8_PIN,
};

static uint8 gray_sensor_initialized;

/**
 * @brief Validate all configured sensor pins.
 * @return ZF_TRUE when every pin is valid and unique.
 */
static uint8 gray_sensor_pins_are_valid(void)
{
    uint8 left_index;
    uint8 right_index;

    if (GRAY_SENSOR_PINS_CONFIGURED == 0U)
    {
        return ZF_FALSE;
    }

    if ((GRAY_SENSOR_ACTIVE_LEVEL != GPIO_LOW)
        && (GRAY_SENSOR_ACTIVE_LEVEL != GPIO_HIGH))
    {
        return ZF_FALSE;
    }

    if ((GRAY_SENSOR_INPUT_MODE != GPI_FLOATING_IN)
        && (GRAY_SENSOR_INPUT_MODE != GPI_PULL_DOWN)
        && (GRAY_SENSOR_INPUT_MODE != GPI_PULL_UP))
    {
        return ZF_FALSE;
    }

    for (left_index = 0U;
        left_index < GRAY_SENSOR_CHANNEL_COUNT;
        left_index++)
    {
        if ((gray_sensor_pins[left_index] < A0)
            || (gray_sensor_pins[left_index] >= GPIO_MAX))
        {
            return ZF_FALSE;
        }

        for (right_index = (uint8)(left_index + 1U);
            right_index < GRAY_SENSOR_CHANNEL_COUNT;
            right_index++)
        {
            if (gray_sensor_pins[left_index]
                == gray_sensor_pins[right_index])
            {
                return ZF_FALSE;
            }
        }
    }

    return ZF_TRUE;
}

/**
 * @brief Initialize all configured digital grayscale inputs.
 * @return ZF_TRUE when the pin configuration is valid.
 */
uint8 gray_sensor_init(void)
{
    uint8 index;

    gray_sensor_initialized = 0U;
    if (gray_sensor_pins_are_valid() == ZF_FALSE)
    {
        return ZF_FALSE;
    }

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        gpio_init(
            gray_sensor_pins[index],
            GPI,
            GPIO_LOW,
            GRAY_SENSOR_INPUT_MODE);
    }

    gray_sensor_initialized = 1U;
    return ZF_TRUE;
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

    result->active_mask = active_mask;
    result->raw_mask = active_mask;
    result->active_count = 0U;
    result->weighted_sum = 0U;
    result->position = 0.0F;
    result->deviation = 0.0F;

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
        result->status = GRAY_SENSOR_STATUS_LOST;
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
 * @brief Sample all eight inputs and calculate normalized line position.
 * @param result Destination result structure.
 * @return ZF_TRUE when acquisition is initialized.
 */
uint8 gray_sensor_sample(gray_sensor_result_struct *result)
{
    uint8 raw_mask = 0U;
    uint8 active_mask;
    uint8 index;

    if ((result == NULL) || (gray_sensor_initialized == 0U))
    {
        return ZF_FALSE;
    }

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        if (gpio_get_level(gray_sensor_pins[index]) != GPIO_LOW)
        {
            raw_mask |= (uint8)(1U << index);
        }
    }

    if (GRAY_SENSOR_ACTIVE_LEVEL == GPIO_HIGH)
    {
        active_mask = raw_mask;
    }
    else
    {
        active_mask = (uint8)(~raw_mask);
    }
    active_mask &= GRAY_SENSOR_ALL_ACTIVE_MASK;

    gray_sensor_calculate(active_mask, result);
    result->raw_mask = raw_mask;

    return ZF_TRUE;
}
