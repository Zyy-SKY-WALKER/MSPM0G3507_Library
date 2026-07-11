/**
 * @file    my_lib_encoder.c
 * @brief   Dual single-edge motor encoder driver for MSPM0G3507.
 * @note    Left encoder uses the extended TIMG8 CCP1 wrapper. Right encoder
 *          uses the existing TIMG6 CCP0 wrapper.
 */

#include "my_lib_encoder.h"

#include "zf_common_interrupt.h"
#include "zf_driver_encoder.h"
#include "zf_driver_timer.h"

#define MY_ENCODER_LEFT_TIMER             (TIM_G8)
#define MY_ENCODER_LEFT_CHANNEL           (TIMG8_ENCODER1_CH2_B22)

#define MY_ENCODER_RIGHT_TIMER            (TIM_G6)
#define MY_ENCODER_RIGHT_CHANNEL          (TIMG6_ENCODER1_CH1_B26)

static uint16 my_encoder_left_previous;
static uint16 my_encoder_right_previous;

/**
 * @brief Apply a sampled direction level to an interval count.
 * @param count Unsigned interval pulse count.
 * @param direction GPIO direction level sampled with the counter.
 * @return Signed interval count. High direction level is positive.
 * @note Each sample interval must contain fewer than 32768 pulses.
 */
static int16 my_encoder_apply_direction(uint16 count, uint8 direction)
{
    int16 signed_count = (int16)count;

    if(direction == GPIO_LOW)
    {
        signed_count = -signed_count;
    }

    return signed_count;
}

/**
 * @brief Initialize left and right hardware single-edge encoder counters.
 */
void my_encoder_init(void)
{
    encoder_dir_timg8_ch2_init(
        MY_ENCODER_LEFT_CHANNEL,
        MY_ENCODER_LEFT_DIRECTION_PIN);
    encoder_dir_init(
        MY_ENCODER_RIGHT_TIMER,
        MY_ENCODER_RIGHT_CHANNEL,
        MY_ENCODER_RIGHT_DIRECTION_PIN);
    my_encoder_clear_count();
}

/**
 * @brief Take a near-synchronous snapshot of both encoder counters.
 * @param left_count Destination for the signed left interval count.
 * @param right_count Destination for the signed right interval count.
 * @note Hardware counters continue running while the snapshot is taken.
 */
void my_encoder_get_delta(int16 *left_count, int16 *right_count)
{
    uint32 primask;
    uint16 left_current;
    uint16 right_current;
    uint16 left_delta;
    uint16 right_delta;
    uint8 left_direction;
    uint8 right_direction;

    if((left_count == NULL) || (right_count == NULL))
    {
        return;
    }

    primask = interrupt_global_disable();

    left_current = timer_get(MY_ENCODER_LEFT_TIMER);
    right_current = timer_get(MY_ENCODER_RIGHT_TIMER);
    left_direction = gpio_get_level(MY_ENCODER_LEFT_DIRECTION_PIN);
    right_direction = gpio_get_level(MY_ENCODER_RIGHT_DIRECTION_PIN);

    left_delta = (uint16)(left_current - my_encoder_left_previous);
    right_delta = (uint16)(right_current - my_encoder_right_previous);
    my_encoder_left_previous = left_current;
    my_encoder_right_previous = right_current;

    interrupt_global_enable(primask);

    *left_count = my_encoder_apply_direction(left_delta, left_direction);
    *right_count = my_encoder_apply_direction(right_delta, right_direction);
}

/**
 * @brief Read the left encoder direction input level.
 * @return GPIO_HIGH or GPIO_LOW.
 */
uint8 my_encoder_get_left_direction(void)
{
    return gpio_get_level(MY_ENCODER_LEFT_DIRECTION_PIN);
}

/**
 * @brief Read the right encoder direction input level.
 * @return GPIO_HIGH or GPIO_LOW.
 */
uint8 my_encoder_get_right_direction(void)
{
    return gpio_get_level(MY_ENCODER_RIGHT_DIRECTION_PIN);
}

/**
 * @brief Reset the left software sampling baseline.
 */
void my_encoder_clear_left_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_previous = timer_get(MY_ENCODER_LEFT_TIMER);

    interrupt_global_enable(primask);
}

/**
 * @brief Reset the right software sampling baseline.
 */
void my_encoder_clear_right_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_right_previous = timer_get(MY_ENCODER_RIGHT_TIMER);

    interrupt_global_enable(primask);
}

/**
 * @brief Reset both software sampling baselines near-synchronously.
 */
void my_encoder_clear_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_previous = timer_get(MY_ENCODER_LEFT_TIMER);
    my_encoder_right_previous = timer_get(MY_ENCODER_RIGHT_TIMER);

    interrupt_global_enable(primask);
}
