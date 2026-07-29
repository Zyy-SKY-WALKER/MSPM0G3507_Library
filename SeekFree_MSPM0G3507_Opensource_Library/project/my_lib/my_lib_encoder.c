/**
 * @file    my_lib_encoder.c
 * @brief   Profile-configured hardware/software quadrature encoder driver.
 * @note    The 520 profile uses x4 decoding. Left uses TIMG8 QEI directly and
 *          right uses both GPIO edge inputs with a four-phase state table.
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
static uint16 my_encoder_left_previous;
#if (DRIVE_PROFILE_ENCODER_USE_X4 == 0U)
static int32 my_encoder_left_remainder;
#endif
static volatile int32 my_encoder_right_count;
#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
static volatile uint32 my_encoder_right_invalid_transition_count;
static uint8 my_encoder_right_previous_state;

/**
 * @brief Return the current right encoder A/B phase state.
 */
static uint8 my_encoder_right_get_state(void)
{
    return (uint8)((gpio_get_level(MY_ENCODER_RIGHT_PHASE_A_PIN) << 1U)
        | gpio_get_level(MY_ENCODER_RIGHT_PHASE_B_PIN));
}
#endif

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
 * @brief Decode one right encoder edge using the active profile resolution.
 * @param event EXTI trigger event.
 * @param user_data Optional callback context.
 */
static void my_encoder_right_callback(uint32 event, void *user_data)
{
#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    static const int8 transition_delta[16] =
    {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0,
    };
    uint8 current_state;
    int8 delta;
#else
    uint8 phase_b;
#endif

    (void)event;
    (void)user_data;

#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    current_state = my_encoder_right_get_state();
    delta = transition_delta[(my_encoder_right_previous_state << 2U)
        | current_state];
    if((delta == 0) && (current_state != my_encoder_right_previous_state))
    {
        my_encoder_right_invalid_transition_count++;
    }
    else if(delta != 0)
    {
        if(MY_ENCODER_RIGHT_POSITIVE_B_LEVEL == GPIO_HIGH)
        {
            delta = -delta;
        }
        my_encoder_right_count += delta;
    }
    my_encoder_right_previous_state = current_state;
#else
    phase_b = gpio_get_level(MY_ENCODER_RIGHT_PHASE_B_PIN);

    if (phase_b == MY_ENCODER_RIGHT_POSITIVE_B_LEVEL)
    {
        my_encoder_right_count++;
    }
    else
    {
        my_encoder_right_count--;
    }
#endif
}

/**
 * @brief Initialize left hardware QEI and right software quadrature decoding.
 */
void my_encoder_init(void)
{
    encoder_quad_init(
        MY_ENCODER_LEFT_TIMER,
        MY_ENCODER_LEFT_QEI_PHASE_A,
        MY_ENCODER_LEFT_QEI_PHASE_B);

#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    exti_init(
        MY_ENCODER_RIGHT_PHASE_A_PIN,
        EXTI_TRIGGER_BOTH,
        my_encoder_right_callback,
        NULL);
    exti_init(
        MY_ENCODER_RIGHT_PHASE_B_PIN,
        EXTI_TRIGGER_BOTH,
        my_encoder_right_callback,
        NULL);
    my_encoder_right_previous_state = my_encoder_right_get_state();
#else
    exti_init(
        MY_ENCODER_RIGHT_PHASE_A_PIN,
        EXTI_TRIGGER_RISING,
        my_encoder_right_callback,
        NULL);
#endif
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
    /* Subtract the QEI counter modulo 2^16 and apply profile polarity. */
    left_raw_delta = (int32)(int16)(uint16)(
        left_current - my_encoder_left_previous);
    my_encoder_left_previous = left_current;
    left_raw_delta *= MY_ENCODER_LEFT_COUNT_SIGN;
#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    left_delta = left_raw_delta;
#else
    left_raw_delta += my_encoder_left_remainder;
    left_delta = left_raw_delta / 4;
    my_encoder_left_remainder = left_raw_delta % 4;
#endif

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
 * @brief Reset the left QEI sampling baseline and optional x1 remainder.
 */
void my_encoder_clear_left_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_previous = timer_get(MY_ENCODER_LEFT_TIMER);
#if (DRIVE_PROFILE_ENCODER_USE_X4 == 0U)
    my_encoder_left_remainder = 0;
#endif

    interrupt_global_enable(primask);
}

/**
 * @brief Clear the right software encoder interval accumulator.
 */
void my_encoder_clear_right_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_right_count = 0;
#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    my_encoder_right_invalid_transition_count = 0U;
    my_encoder_right_previous_state = my_encoder_right_get_state();
#endif

    interrupt_global_enable(primask);
}

/**
 * @brief Reset both interval accumulators near-synchronously.
 */
void my_encoder_clear_count(void)
{
    uint32 primask = interrupt_global_disable();

    my_encoder_left_previous = timer_get(MY_ENCODER_LEFT_TIMER);
#if (DRIVE_PROFILE_ENCODER_USE_X4 == 0U)
    my_encoder_left_remainder = 0;
#endif
    my_encoder_right_count = 0;
#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    my_encoder_right_invalid_transition_count = 0U;
    my_encoder_right_previous_state = my_encoder_right_get_state();
#endif

    interrupt_global_enable(primask);
}

/**
 * @brief Read the number of rejected right-wheel quadrature transitions.
 * @return Invalid phase-transition count since the most recent clear.
 */
uint32 my_encoder_get_right_invalid_transition_count(void)
{
    uint32 primask;
    uint32 count;

    primask = interrupt_global_disable();
#if (DRIVE_PROFILE_ENCODER_USE_X4 != 0U)
    count = my_encoder_right_invalid_transition_count;
#else
    count = 0U;
#endif
    interrupt_global_enable(primask);

    return count;
}
