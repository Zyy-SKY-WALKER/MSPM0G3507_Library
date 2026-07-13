/**
 * @file    test_gpio_output.c
 * @brief   Expansion-board GPIO push-pull output test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_GPIO_OUTPUT)

#include "test_gpio_output.h"

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#define GPIO_OUTPUT_TEST_PIN_COUNT       (4U)
#define GPIO_OUTPUT_TEST_START_TIME_MS   (1000U)
#define GPIO_OUTPUT_TEST_LEVEL_TIME_MS   (500U)

static const gpio_pin_enum gpio_output_test_pins[
    GPIO_OUTPUT_TEST_PIN_COUNT] =
{
    B8,
    B9,
    B12,
    B13,
};

/**
 * @brief Set all test pins to the same output level.
 * @param level GPIO_LOW or GPIO_HIGH.
 */
static void gpio_output_test_set_all(uint8 level)
{
    uint8 index;

    for (index = 0U; index < GPIO_OUTPUT_TEST_PIN_COUNT; index++)
    {
        gpio_set_level(gpio_output_test_pins[index], level);
    }
}

/**
 * @brief Initialize and continuously toggle all expansion GPIO pins.
 */
void test_gpio_output_run(void)
{
    uint8 index;

    for (index = 0U; index < GPIO_OUTPUT_TEST_PIN_COUNT; index++)
    {
        gpio_init(
            gpio_output_test_pins[index],
            GPO,
            GPIO_LOW,
            GPO_PUSH_PULL);
    }

    gpio_output_test_set_all(GPIO_LOW);
    system_delay_ms(GPIO_OUTPUT_TEST_START_TIME_MS);

    while (true)
    {
        gpio_output_test_set_all(GPIO_HIGH);
        system_delay_ms(GPIO_OUTPUT_TEST_LEVEL_TIME_MS);
        gpio_output_test_set_all(GPIO_LOW);
        system_delay_ms(GPIO_OUTPUT_TEST_LEVEL_TIME_MS);
    }
}

#endif
