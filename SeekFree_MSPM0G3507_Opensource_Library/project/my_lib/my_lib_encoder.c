/**
 * @file    my_lib_encoder.c
 * @brief   Dual single-edge motor encoder driver for MSPM0G3507.
 * @note    Left encoder uses TIMG8 CCP1 because B22 is not exposed by the
 *          SeekFree single-edge encoder wrapper. Right encoder reuses the
 *          existing TIMG6 CCP0 wrapper.
 */

#include "my_lib_encoder.h"

#include "zf_driver_encoder.h"
#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"

#define MY_ENCODER_LEFT_TIMER             (TIMG8)
#define MY_ENCODER_LEFT_CC_INDEX          (DL_TIMER_CC_1_INDEX)
#define MY_ENCODER_LEFT_CC_DIRECTION      (DL_TIMER_CC1_INPUT)

#define MY_ENCODER_RIGHT_TIMER            (TIM_G6)
#define MY_ENCODER_RIGHT_CHANNEL          (TIMG6_ENCODER1_CH1_B26)

static const DL_TimerG_ClockConfig my_encoder_clock_config =
{
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U,
};

/**
 * @brief Configure TIMG8 CCP1 as a hardware rising-edge counter.
 */
static void my_encoder_left_counter_init(void)
{
    afio_init(
        MY_ENCODER_LEFT_PULSE_PIN,
        GPI,
        GPIO_AF3,
        GPI_PULL_UP);
    gpio_init(
        MY_ENCODER_LEFT_DIRECTION_PIN,
        GPI,
        GPIO_LOW,
        GPI_PULL_UP);

    DL_Timer_setClockConfig(
        MY_ENCODER_LEFT_TIMER,
        &my_encoder_clock_config);
    DL_Timer_setCaptureCompareInput(
        MY_ENCODER_LEFT_TIMER,
        DL_TIMER_CC_INPUT_INV_NOINVERT,
        DL_TIMER_CC_IN_SEL_CCPX,
        MY_ENCODER_LEFT_CC_INDEX);
    DL_Timer_setLoadValue(MY_ENCODER_LEFT_TIMER, 0xFFFFU);
    DL_Timer_setCaptureCompareCtl(
        MY_ENCODER_LEFT_TIMER,
        DL_TIMER_CC_MODE_CAPTURE,
        DL_TIMER_CC_ZCOND_NONE
            | DL_TIMER_CC_ACOND_TRIG_RISE
            | DL_TIMER_CC_LCOND_NONE
            | DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
        MY_ENCODER_LEFT_CC_INDEX);
    DL_Timer_setCCPDirection(
        MY_ENCODER_LEFT_TIMER,
        MY_ENCODER_LEFT_CC_DIRECTION);
    DL_Timer_setCounterControl(
        MY_ENCODER_LEFT_TIMER,
        DL_TIMER_CZC_CCCTL1_ZCOND,
        DL_TIMER_CAC_CCCTL1_ACOND,
        DL_TIMER_CLC_CCCTL1_LCOND);
    DL_Timer_setCounterMode(
        MY_ENCODER_LEFT_TIMER,
        DL_TIMER_COUNT_MODE_UP);
    DL_Timer_setCounterValueAfterEnable(
        MY_ENCODER_LEFT_TIMER,
        DL_TIMER_COUNT_AFTER_EN_NO_CHANGE);
    DL_Timer_setCounterRepeatMode(
        MY_ENCODER_LEFT_TIMER,
        DL_TIMER_REPEAT_MODE_ENABLED);
    DL_Timer_setTimerCount(MY_ENCODER_LEFT_TIMER, 0U);
    DL_Timer_enableClock(MY_ENCODER_LEFT_TIMER);
    DL_Timer_startCounter(MY_ENCODER_LEFT_TIMER);
}

/**
 * @brief Convert an unsigned hardware counter to a signed direction count.
 * @param count Hardware counter value.
 * @param direction_pin GPIO sampled to determine direction.
 * @return Signed encoder count. High direction level is positive.
 */
static int16 my_encoder_get_signed_count(
    uint16 count,
    gpio_pin_enum direction_pin)
{
    int16 signed_count = (int16)count;

    if(gpio_get_level(direction_pin) == GPIO_LOW)
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
    my_encoder_left_counter_init();
    encoder_dir_init(
        MY_ENCODER_RIGHT_TIMER,
        MY_ENCODER_RIGHT_CHANNEL,
        MY_ENCODER_RIGHT_DIRECTION_PIN);
    DL_Timer_setCounterValueAfterEnable(
        TIMG6,
        DL_TIMER_COUNT_AFTER_EN_NO_CHANGE);
    my_encoder_clear_count();
}

/**
 * @brief Read the signed left encoder count.
 * @return Count since the last left counter clear.
 */
int16 my_encoder_get_left_count(void)
{
    return my_encoder_get_signed_count(
        (uint16)DL_Timer_getTimerCount(MY_ENCODER_LEFT_TIMER),
        MY_ENCODER_LEFT_DIRECTION_PIN);
}

/**
 * @brief Read the signed right encoder count.
 * @return Count since the last right counter clear.
 */
int16 my_encoder_get_right_count(void)
{
    return encoder_get_count(MY_ENCODER_RIGHT_TIMER);
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
 * @brief Clear the left hardware counter.
 */
void my_encoder_clear_left_count(void)
{
    DL_Timer_stopCounter(MY_ENCODER_LEFT_TIMER);
    DL_Timer_setTimerCount(MY_ENCODER_LEFT_TIMER, 0U);
    DL_Timer_startCounter(MY_ENCODER_LEFT_TIMER);
}

/**
 * @brief Clear the right hardware counter.
 */
void my_encoder_clear_right_count(void)
{
    encoder_clear_count(MY_ENCODER_RIGHT_TIMER);
}

/**
 * @brief Clear both hardware counters.
 */
void my_encoder_clear_count(void)
{
    my_encoder_clear_left_count();
    my_encoder_clear_right_count();
}
