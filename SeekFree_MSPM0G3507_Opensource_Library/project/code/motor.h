/**
 * @file    motor.h
 * @brief   TB6612FNG dual DC motor driver interface.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

/** Motor PWM carrier frequency in hertz. */
#define MOTOR_PWM_FREQUENCY          (1000U)
/** Maximum accepted PWM duty in driver-native duty units. */
#define MOTOR_DUTY_MAX               (PWM_DUTY_MAX)

#define MOTOR_LEFT_PWM_PIN           (PWM_TIM_G7_CH1_A27)
#define MOTOR_LEFT_IN1_PIN           (B14)
#define MOTOR_LEFT_IN2_PIN           (B10)

#define MOTOR_RIGHT_PWM_PIN          (PWM_TIM_G0_CH1_B11)
#define MOTOR_RIGHT_IN1_PIN          (B25)
#define MOTOR_RIGHT_IN2_PIN          (B19)

/**
 * @brief Initialize both motor PWM channels and direction GPIO outputs.
 * @note Both motor channels are left stopped with direction inputs low.
 */
void motor_init(void);

/**
 * @brief Set the signed left motor duty command.
 * @param duty Positive drives forward, negative drives reverse, and zero stops.
 * @note Magnitude is clamped to MOTOR_DUTY_MAX.
 */
void motor_left_set_duty(int16 duty);

/**
 * @brief Set the signed right motor duty command.
 * @param duty Positive drives forward, negative drives reverse, and zero stops.
 * @note Magnitude is clamped to MOTOR_DUTY_MAX.
 */
void motor_right_set_duty(int16 duty);

/**
 * @brief Stop both motors by clearing PWM and both direction inputs.
 */
void motor_stop(void);

#endif
