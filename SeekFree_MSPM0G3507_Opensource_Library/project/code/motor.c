/**
 * @file    motor.c
 * @brief   TB6612FNG dual DC motor driver implementation.
 */

#include "motor.h"

#include "zf_driver_gpio.h"

/**
 * @brief Set one TB6612 channel direction.
 * @param in1_pin Channel IN1 GPIO pin.
 * @param in2_pin Channel IN2 GPIO pin.
 * @param duty Signed motor duty.
 */
static void motor_set_direction(
    gpio_pin_enum in1_pin,
    gpio_pin_enum in2_pin,
    int16 duty)
{
    if(duty > 0)
    {
        gpio_high(in1_pin);
        gpio_low(in2_pin);
    }
    else if(duty < 0)
    {
        gpio_low(in1_pin);
        gpio_high(in2_pin);
    }
    else
    {
        gpio_low(in1_pin);
        gpio_low(in2_pin);
    }
}

/**
 * @brief Convert a signed duty to a clamped PWM duty magnitude.
 * @param duty Signed motor duty.
 * @return PWM duty from 0 to MOTOR_DUTY_MAX.
 */
static uint32 motor_get_duty_magnitude(int16 duty)
{
    int32 magnitude = duty;

    if(magnitude < 0)
    {
        magnitude = -magnitude;
    }

    if(magnitude > MOTOR_DUTY_MAX)
    {
        magnitude = MOTOR_DUTY_MAX;
    }

    return (uint32)magnitude;
}

/**
 * @brief Set one TB6612 channel output.
 * @param pwm_pin PWM output pin.
 * @param in1_pin Channel IN1 GPIO pin.
 * @param in2_pin Channel IN2 GPIO pin.
 * @param duty Signed motor duty.
 */
static void motor_set_duty(
    pwm_channel_enum pwm_pin,
    gpio_pin_enum in1_pin,
    gpio_pin_enum in2_pin,
    int16 duty)
{
    pwm_set_duty(pwm_pin, 0U);
    motor_set_direction(in1_pin, in2_pin, duty);
    pwm_set_duty(pwm_pin, motor_get_duty_magnitude(duty));
}

/**
 * @brief Initialize TB6612 PWM and direction outputs in a stopped state.
 */
void motor_init(void)
{
    gpio_init(MOTOR_LEFT_IN1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_LEFT_IN2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_IN1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_IN2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(MOTOR_LEFT_PWM_PIN, MOTOR_PWM_FREQUENCY, 0U);
    pwm_init(MOTOR_RIGHT_PWM_PIN, MOTOR_PWM_FREQUENCY, 0U);
}

/**
 * @brief Set left motor signed duty.
 * @param duty Positive is forward and negative is reverse.
 */
void motor_left_set_duty(int16 duty)
{
    motor_set_duty(
        MOTOR_LEFT_PWM_PIN,
        MOTOR_LEFT_IN1_PIN,
        MOTOR_LEFT_IN2_PIN,
        duty);
}

/**
 * @brief Set right motor signed duty.
 * @param duty Positive is forward and negative is reverse.
 */
void motor_right_set_duty(int16 duty)
{
    motor_set_duty(
        MOTOR_RIGHT_PWM_PIN,
        MOTOR_RIGHT_IN1_PIN,
        MOTOR_RIGHT_IN2_PIN,
        duty);
}

/**
 * @brief Stop both motors with PWM disabled and direction inputs low.
 */
void motor_stop(void)
{
    motor_left_set_duty(0);
    motor_right_set_duty(0);
}
