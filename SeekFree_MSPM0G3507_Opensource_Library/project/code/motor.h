/**
 * @file    motor.h
 * @brief   TB6612FNG dual DC motor driver interface.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

#define MOTOR_PWM_FREQUENCY          (1000U)
#define MOTOR_DUTY_MAX               (PWM_DUTY_MAX)

#define MOTOR_LEFT_PWM_PIN           (PWM_TIM_G7_CH1_A27)
#define MOTOR_LEFT_IN1_PIN           (A25)
#define MOTOR_LEFT_IN2_PIN           (A24)

#define MOTOR_RIGHT_PWM_PIN          (PWM_TIM_G0_CH1_B11)
#define MOTOR_RIGHT_IN1_PIN          (B25)
#define MOTOR_RIGHT_IN2_PIN          (B24)

void motor_init(void);
void motor_left_set_duty(int16 duty);
void motor_right_set_duty(int16 duty);
void motor_stop(void);

#endif
