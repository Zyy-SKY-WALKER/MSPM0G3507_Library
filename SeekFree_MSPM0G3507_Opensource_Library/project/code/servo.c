/**
 * @file    servo.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Dual 270-degree servo control implementation.
 */

#include "servo.h"

#define SERVO_MICROSECONDS_PER_SECOND    (1000000UL)

static uint8 servo_initialized;

/**
 * @brief Convert a servo pulse width to PWM duty units.
 * @param pulse_us Pulse width in microseconds.
 * @return PWM duty in the range supported by the PWM driver.
 */
static uint32 servo_pulse_to_duty(uint32 pulse_us)
{
    uint32 duty;

    duty = pulse_us * SERVO_PWM_FREQUENCY_HZ * PWM_DUTY_MAX;
    duty /= SERVO_MICROSECONDS_PER_SECOND;

    return duty;
}

/**
 * @brief Convert a valid servo angle in tenths of a degree to PWM duty units.
 * @param angle_tenth_deg Servo angle from 0 to 2700 tenths of a degree.
 * @return PWM duty in the range supported by the PWM driver.
 */
static uint32 servo_angle_tenth_deg_to_duty(uint16 angle_tenth_deg)
{
    uint32 minimum_duty;
    uint32 maximum_duty;
    uint32 duty_range;
    uint32 scaled_duty;

    minimum_duty = servo_pulse_to_duty(SERVO_MIN_PULSE_US);
    maximum_duty = servo_pulse_to_duty(SERVO_MAX_PULSE_US);
    duty_range = maximum_duty - minimum_duty;
    /* Round the linear angle-to-duty interpolation to the nearest unit. */
    scaled_duty = ((uint32)angle_tenth_deg * duty_range)
        + (SERVO_MAX_ANGLE_TENTH_DEG / 2U);

    return minimum_duty + (scaled_duty / SERVO_MAX_ANGLE_TENTH_DEG);
}

/**
 * @brief Initialize both servo PWM channels at the middle position.
 */
void servo_init(void)
{
    uint32 middle_duty;

    servo_initialized = 0U;
    middle_duty = servo_pulse_to_duty(SERVO_MID_PULSE_US);

    pwm_init(
        SERVO_B4_PWM_PIN,
        SERVO_PWM_FREQUENCY_HZ,
        middle_duty);
    pwm_init(
        SERVO_B5_PWM_PIN,
        SERVO_PWM_FREQUENCY_HZ,
        middle_duty);

    servo_initialized = 1U;
    (void)servo_set_all_angle_tenth_deg(SERVO_MID_ANGLE_TENTH_DEG);
}

/**
 * @brief Set one servo angle.
 * @param channel Servo PWM channel.
 * @param angle_deg Requested angle from 0 to 270 degrees.
 * @return 1 when accepted; otherwise 0.
 */
uint8 servo_set_angle(
    servo_channel_enum channel,
    uint16 angle_deg)
{
    if (angle_deg > SERVO_MAX_ANGLE_DEG)
    {
        return 0U;
    }

    return servo_set_angle_tenth_deg(
        channel,
        angle_deg * SERVO_ANGLE_TENTH_DEGREE_SCALE);
}

/**
 * @brief Set one servo angle in tenths of a degree.
 * @param channel Servo PWM channel.
 * @param angle_tenth_deg Requested angle from 0 to 2700 tenths of a degree.
 * @return 1 when accepted; otherwise 0.
 */
uint8 servo_set_angle_tenth_deg(
    servo_channel_enum channel,
    uint16 angle_tenth_deg)
{
    pwm_channel_enum pwm_pin;

    if ((servo_initialized == 0U)
        || (angle_tenth_deg > SERVO_MAX_ANGLE_TENTH_DEG))
    {
        return 0U;
    }

    if (channel == SERVO_CHANNEL_B4)
    {
        pwm_pin = SERVO_B4_PWM_PIN;
    }
    else if (channel == SERVO_CHANNEL_B5)
    {
        pwm_pin = SERVO_B5_PWM_PIN;
    }
    else
    {
        return 0U;
    }

    pwm_set_duty(pwm_pin, servo_angle_tenth_deg_to_duty(angle_tenth_deg));
    return 1U;
}

/**
 * @brief Set both servo channels to the same angle.
 * @param angle_deg Requested angle from 0 to 270 degrees.
 * @return 1 when accepted; otherwise 0.
 */
uint8 servo_set_all_angle(uint16 angle_deg)
{
    if (angle_deg > SERVO_MAX_ANGLE_DEG)
    {
        return 0U;
    }

    return servo_set_all_angle_tenth_deg(
        angle_deg * SERVO_ANGLE_TENTH_DEGREE_SCALE);
}

/**
 * @brief Set both servo angles in tenths of a degree.
 * @param angle_tenth_deg Requested angle from 0 to 2700 tenths of a degree.
 * @return 1 when accepted; otherwise 0.
 */
uint8 servo_set_all_angle_tenth_deg(uint16 angle_tenth_deg)
{
    uint32 duty;

    if ((servo_initialized == 0U)
        || (angle_tenth_deg > SERVO_MAX_ANGLE_TENTH_DEG))
    {
        return 0U;
    }

    duty = servo_angle_tenth_deg_to_duty(angle_tenth_deg);
    pwm_set_duty(SERVO_B4_PWM_PIN, duty);
    pwm_set_duty(SERVO_B5_PWM_PIN, duty);

    return 1U;
}
