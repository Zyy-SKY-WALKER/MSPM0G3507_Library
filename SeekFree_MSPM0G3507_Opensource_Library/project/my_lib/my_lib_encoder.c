/**
 * @file    my_lib_encoder.c
 * @brief   Dual single-edge quadrature encoder driver for MSPM0G3507.
 * @note    Each phase-A rising edge samples phase B to determine direction.
 */

#include "my_lib_encoder.h"

#include "zf_common_interrupt.h"
#include "zf_driver_exti.h"

#define MY_ENCODER_DELTA_MAX              (32767)
#define MY_ENCODER_DELTA_MIN              (-32768)

static volatile int32 my_encoder_left_count;
static volatile int32 my_encoder_right_count;

/**
 * @brief Clamp one interval count to the public signed 16-bit range.
 * @param count Signed software count accumulated since the last sample.
 * @return Clamped signed interval count.
 */
static int16 my_encoder_clamp_delta(int32 count)
{
    if (count > MY_ENCODER_DELTA_MAX)
    {
        count = MY_ENCODER_DELTA_MAX;
    }
    else if (count < MY_ENCODER_DELTA_MIN)
    {
        count = MY_ENCODER_DELTA_MIN;
    }

    return (int16)count;
}

/**
 * @brief Accumulate one phase-A rising edge using the sampled phase B.
 * @param count Destination interval accumulator.
 * @param phase_b_pin Encoder phase-B GPIO pin.
 * @param positive_level Phase-B level representing positive rotation.
 */
static void my_encoder_accumulate_edge(
    volatile int32 *count,
    gpio_pin_enum phase_b_pin,
    uint8 positive_level)
{
    if (gpio_get_level(phase_b_pin) == positive_level)
    {
        (*count)++;
    }
    else
    {
        (*count)--;
    }
}

/**
 * @brief Decode one left encoder phase-A rising edge.
 * @param event EXTI trigger event.
 * @param user_data Optional callback context.
 */
static void my_encoder_left_callback(uint32 event, void *user_data)
{
    (void)event;
    (void)user_data;

    my_encoder_accumulate_edge(
        &my_encoder_left_count,
        MY_ENCODER_LEFT_PHASE_B_PIN,
        MY_ENCODER_LEFT_POSITIVE_B_LEVEL);
}

/**
 * @brief Decode one right encoder phase-A rising edge.
 * @param event EXTI trigger event.
 * @param user_data Optional callback context.
 */
static void my_encoder_right_callback(uint32 event, void *user_data)
{
    (void)event;
    (void)user_data;

    my_encoder_accumulate_edge(
        &my_encoder_right_count,
        MY_ENCODER_RIGHT_PHASE_B_PIN,
        MY_ENCODER_RIGHT_POSITIVE_B_LEVEL);
}

/**
 * @brief Initialize both single-edge quadrature encoder inputs.
 */
void my_encoder_init(void)
{
    gpio_init(
        MY_ENCODER_LEFT_PHASE_B_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        MY_ENCODER_RIGHT_PHASE_B_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);

    my_encoder_clear_count();
    exti_init(
        MY_ENCODER_LEFT_PHASE_A_PIN,
        EXTI_TRIGGER_RISING,
        my_encoder_left_callback,
        NULL);
    exti_init(
        MY_ENCODER_RIGHT_PHASE_A_PIN,
        EXTI_TRIGGER_RISING,
        my_encoder_right_callback,
        NULL);
}

/**
 * @brief Take and clear a near-synchronous interval count snapshot.
 * @param left_count Destination for the signed left interval count.
 * @param right_count Destination for the signed right interval count.
 * @note Call exactly once per sampling interval in closed-loop control.
 */
void my_encoder_get_delta(int16 *left_count, int16 *right_count)
{
    uint32 primask;
    int32 left_delta;
    int32 right_delta;

    if ((left_count == NULL) || (right_count == NULL))
    {
        return;
    }

    primask = interrupt_global_disable();

    left_delta = my_encoder_left_count;
    right_delta = my_encoder_right_count;
    my_encoder_left_count = 0;
    my_encoder_right_count = 0;

    interrupt_global_enable(primask);

    *left_count = my_encoder_clamp_delta(left_delta);
    *right_count = my_encoder_clamp_delta(right_delta);
}

/**
 * @brief Read the left encoder phase-A input level.
 * @return GPIO_HIGH or GPIO_LOW.
 */
uint8 my_encoder_get_left_phase_a(void)
{
    return gpio_get_level(MY_ENCODER_LEFT_PHASE_A_PIN);
}

/**
 * @brief Read the left encoder phase-B input level.
 * @return GPIO_HIGH or GPIO_LOW.
 */
uint8 my_encoder_get_left_phase_b(void)
{
    return gpio_get_level(MY_ENCODER_LEFT_PHASE_B_PIN);
}

/**
 * @brief Read the right encoder phase-A input level.
 * @return GPIO_HIGH or GPIO_LOW.
 */
uint8 my_encoder_get_right_phase_a(void)
{
    return gpio_get_level(MY_ENCODER_RIGHT_PHASE_A_PIN);
}

/**
 * @brief Read the right encoder phase-B input level.
 * @return GPIO_HIGH or GPIO_LOW.
 */
uint8 my_encoder_get_right_phase_b(void)
{
    return gpio_get_level(MY_ENCODER_RIGHT_PHASE_B_PIN);
}

/**
 * @brief Clear the left software interval accumulator.
 */
void my_encoder_clear_left_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_count = 0;

    interrupt_global_enable(primask);
}

/**
 * @brief Clear the right software interval accumulator.
 */
void my_encoder_clear_right_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_right_count = 0;

    interrupt_global_enable(primask);
}

/**
 * @brief Clear both software interval accumulators near-synchronously.
 */
void my_encoder_clear_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_count = 0;
    my_encoder_right_count = 0;

    interrupt_global_enable(primask);
}
