/**
 * @file    servo.h
 * @author  Project team
 * @version V1.0
 * @date    2026-07-14
 * @brief   Dual 270-degree servo control interface.
 */

#ifndef SERVO_H
#define SERVO_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

#define SERVO_PWM_FREQUENCY_HZ             (50U)
#define SERVO_MIN_PULSE_US                 (500U)
#define SERVO_MID_PULSE_US                 (1500U)
#define SERVO_MAX_PULSE_US                 (2500U)
#define SERVO_MAX_ANGLE_DEG                (270U)
#define SERVO_MID_ANGLE_DEG                (135U)
#define SERVO_ANGLE_TENTH_DEGREE_SCALE     (10U)
#define SERVO_MAX_ANGLE_TENTH_DEG          \
    (SERVO_MAX_ANGLE_DEG * SERVO_ANGLE_TENTH_DEGREE_SCALE)
#define SERVO_MID_ANGLE_TENTH_DEG          \
    (SERVO_MID_ANGLE_DEG * SERVO_ANGLE_TENTH_DEGREE_SCALE)

#define SERVO_B4_PWM_PIN                   (PWM_TIM_A1_CH0_B4)
#define SERVO_B5_PWM_PIN                   (PWM_TIM_A1_CH1_B5)

/** @brief PWM output selection for one physical servo connector. */
typedef enum
{
    /** Servo connected to the timer output on pin B4. */
    SERVO_CHANNEL_B4 = 0,
    /** Servo connected to the timer output on pin B5. */
    SERVO_CHANNEL_B5
} servo_channel_enum;

/**
 * @brief Initialize both servo PWM channels at the middle position.
 * @note Configures both outputs for 50 Hz PWM and commands 135 degrees.
 */
void servo_init(void);

/**
 * @brief Set one servo angle in whole degrees.
 * @param channel Servo PWM channel.
 * @param angle_deg Requested angle from 0 to 270 degrees.
 * @return 1 when accepted; otherwise 0.
 * @note servo_init() must complete before setting an angle.
 */
uint8 servo_set_angle(
    servo_channel_enum channel,
    uint16 angle_deg);

/**
 * @brief Set both servo angles in whole degrees.
 * @param angle_deg Requested angle from 0 to 270 degrees.
 * @return 1 when accepted; otherwise 0.
 * @note servo_init() must complete before setting an angle.
 */
uint8 servo_set_all_angle(uint16 angle_deg);

/**
 * @brief Set one servo angle in tenths of a degree.
 * @param channel Servo PWM channel.
 * @param angle_tenth_deg Requested angle from 0 to 2700 tenths of a degree.
 * @return 1 when accepted; otherwise 0.
 * @note servo_init() must complete before setting an angle.
 */
uint8 servo_set_angle_tenth_deg(
    servo_channel_enum channel,
    uint16 angle_tenth_deg);

/**
 * @brief Set both servo angles in tenths of a degree.
 * @param angle_tenth_deg Requested angle from 0 to 2700 tenths of a degree.
 * @return 1 when accepted; otherwise 0.
 * @note servo_init() must complete before setting an angle.
 */
uint8 servo_set_all_angle_tenth_deg(uint16 angle_tenth_deg);

#endif
