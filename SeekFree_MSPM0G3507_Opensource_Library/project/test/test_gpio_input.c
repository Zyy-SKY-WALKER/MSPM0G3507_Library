/**
 * @file    test_gpio_input.c
 * @brief   Eight-channel pull-up GPIO input TFT test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_GPIO_INPUT)

#include "test_gpio_input.h"

#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#define GPIO_INPUT_TEST_PIN_COUNT       (8U)
#define GPIO_INPUT_TEST_UPDATE_TIME_MS  (100U)

static const gpio_pin_enum gpio_input_test_pins[
    GPIO_INPUT_TEST_PIN_COUNT] =
{
    B26,
    B19,
    A29,
    A28,
    B27,
    A16,
    A17,
    B20,
};

static const char *const gpio_input_test_labels[
    GPIO_INPUT_TEST_PIN_COUNT] =
{
    "D1 B26 :",
    "D2 B19 :",
    "D3 A29 :",
    "D4 A28 :",
    "D5 B27 :",
    "D6 A16 :",
    "D7 A17 :",
    "D8 B20 :",
};

static const uint16 gpio_input_test_rows[
    GPIO_INPUT_TEST_PIN_COUNT] =
{
    40U,
    64U,
    88U,
    112U,
    136U,
    160U,
    184U,
    208U,
};

/**
 * @brief Display an eight-bit input mask with D1 at the left side.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param mask Input level mask.
 */
static void gpio_input_test_show_mask(uint16 x, uint16 y, uint8 mask)
{
    uint8 index;

    for (index = 0U; index < GPIO_INPUT_TEST_PIN_COUNT; index++)
    {
        ili9341_show_char(
            (uint16)(x + ((uint16)index * 8U)),
            y,
            (mask & (uint8)(1U << index)) != 0U ? '1' : '0');
    }
}

/**
 * @brief Initialize and display all pull-up GPIO input levels.
 */
void test_gpio_input_run(void)
{
    uint8 index;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "GPIO INPUT TEST");

    for (index = 0U; index < GPIO_INPUT_TEST_PIN_COUNT; index++)
    {
        gpio_init(
            gpio_input_test_pins[index],
            GPI,
            GPIO_HIGH,
            GPI_PULL_UP);
        ili9341_show_string(
            8U,
            gpio_input_test_rows[index],
            gpio_input_test_labels[index]);
    }

    ili9341_show_string(8U, 248U, "MASK   :");
    ili9341_show_string(8U, 280U, "OPEN=1  GND=0");

    while (true)
    {
        uint8 input_mask = 0U;

        for (index = 0U; index < GPIO_INPUT_TEST_PIN_COUNT; index++)
        {
            uint8 level = gpio_get_level(gpio_input_test_pins[index]);

            if (level != GPIO_LOW)
            {
                input_mask |= (uint8)(1U << index);
            }

            ili9341_show_char(
                104U,
                gpio_input_test_rows[index],
                level != GPIO_LOW ? '1' : '0');
        }

        gpio_input_test_show_mask(104U, 248U, input_mask);
        system_delay_ms(GPIO_INPUT_TEST_UPDATE_TIME_MS);
    }
}

#endif
