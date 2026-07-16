/**
 * @file    my_lib_encoder.c
 * @brief   Hybrid hardware/software x2 quadrature encoder driver.
 * @note    Left uses TIMG8 QEI divided by two. Right uses phase-A dual-edge
 *          interrupts and samples both phase levels.
 */

#include "my_lib_encoder.h"

#include "zf_common_interrupt.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_timer.h"

#define MY_ENCODER_DELTA_MAX              (32767)
#define MY_ENCODER_DELTA_MIN              (-32768)
#define MY_ENCODER_LEFT_TIMER             (TIM_G8)
#define MY_ENCODER_LEFT_QEI_PHASE_A       (TIMG8_ENCODER1_CH1_B21)
#define MY_ENCODER_LEFT_QEI_PHASE_B       (TIMG8_ENCODER1_CH2_B22)
#define MY_ENCODER_QEI_X4_TO_X2_DIVISOR   (2)

static uint16 my_encoder_left_previous;
static int32 my_encoder_left_remainder;
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
 * @brief Decode one right encoder phase-A edge at x2 resolution.
 * @param event EXTI trigger event.
 * @param user_data Optional callback context.
 */
static void my_encoder_right_callback(uint32 event, void *user_data)
{
    uint8 phase_a;
    uint8 phase_b;
    uint8 phases_are_equal;

    (void)event;
    (void)user_data;

    phase_a = gpio_get_level(MY_ENCODER_RIGHT_PHASE_A_PIN);
    phase_b = gpio_get_level(MY_ENCODER_RIGHT_PHASE_B_PIN);
    phases_are_equal = (uint8)(phase_a == phase_b);

    if (phases_are_equal == MY_ENCODER_RIGHT_POSITIVE_AB_EQUAL)
    {
        my_encoder_right_count++;
    }
    else
    {
        my_encoder_right_count--;
    }
}

/**
 * @brief Initialize left hardware QEI and right software x2 decoding.
 */
void my_encoder_init(void)
{
    encoder_quad_init(
        MY_ENCODER_LEFT_TIMER,
        MY_ENCODER_LEFT_QEI_PHASE_A,
        MY_ENCODER_LEFT_QEI_PHASE_B);

    gpio_init(
        MY_ENCODER_RIGHT_PHASE_B_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);

    exti_init(
        MY_ENCODER_RIGHT_PHASE_A_PIN,
        EXTI_TRIGGER_BOTH,
        my_encoder_right_callback,
        NULL);
    my_encoder_clear_count();
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
    uint16 left_current;
    int32 left_raw_delta;
    int32 left_delta;
    int32 right_delta;

    if ((left_count == NULL) || (right_count == NULL))
    {
        return;
    }

    primask = interrupt_global_disable();

    left_current = timer_get(MY_ENCODER_LEFT_TIMER);
    left_raw_delta = (int32)(int16)(uint16)(
        left_current - my_encoder_left_previous);
    my_encoder_left_previous = left_current;
    left_raw_delta *= MY_ENCODER_LEFT_COUNT_SIGN;
    left_raw_delta += my_encoder_left_remainder;
    left_delta = left_raw_delta / MY_ENCODER_QEI_X4_TO_X2_DIVISOR;
    my_encoder_left_remainder =
        left_raw_delta % MY_ENCODER_QEI_X4_TO_X2_DIVISOR;

    right_delta = my_encoder_right_count;
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
 * @brief Reset the left QEI sampling baseline and scaling remainder.
 */
void my_encoder_clear_left_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_previous = timer_get(MY_ENCODER_LEFT_TIMER);
    my_encoder_left_remainder = 0;

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
 * @brief Reset both interval accumulators near-synchronously.
 */
void my_encoder_clear_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_previous = timer_get(MY_ENCODER_LEFT_TIMER);
    my_encoder_left_remainder = 0;
    my_encoder_right_count = 0;

    interrupt_global_enable(primask);
}
