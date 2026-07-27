/**
 * @file    drive_motor_config.h
 * @brief   Selectable motor, encoder and chassis calibration profiles.
 */

#ifndef DRIVE_MOTOR_CONFIG_H
#define DRIVE_MOTOR_CONFIG_H

/** @brief Original motor and chassis calibration profile. */
#define DRIVE_MOTOR_PROFILE_ORIGINAL                 (0U)
/** @brief 520 motor, 30:1 gearbox and 65 mm wheel calibration profile. */
#define DRIVE_MOTOR_PROFILE_520                      (1U)

/** @brief Profile selected for this firmware build. */
#ifndef DRIVE_ACTIVE_MOTOR_PROFILE
#define DRIVE_ACTIVE_MOTOR_PROFILE                   \
    (DRIVE_MOTOR_PROFILE_520)
#endif

/* Original profile retained from the existing MSPM0G3507 vehicle setup. */
#define DRIVE_ORIGINAL_ENCODER_COUNTS_PER_REV        (10250U)
#define DRIVE_ORIGINAL_LEFT_ENCODER_COUNT_SIGN       (1)
#define DRIVE_ORIGINAL_RIGHT_ENCODER_POSITIVE_B_LEVEL \
    (1U)
#define DRIVE_ORIGINAL_LEFT_MOTOR_DUTY_SIGN          (1)
#define DRIVE_ORIGINAL_RIGHT_MOTOR_DUTY_SIGN         (1)
#define DRIVE_ORIGINAL_LEFT_WHEEL_DIAMETER_MM        (48.0F)
#define DRIVE_ORIGINAL_RIGHT_WHEEL_DIAMETER_MM       (48.0F)
#define DRIVE_ORIGINAL_TRACK_WIDTH_MM                (129.0F)
#define DRIVE_ORIGINAL_SPEED_PID_OUTPUT_LIMIT        (8000)
#define DRIVE_ORIGINAL_STRAIGHT_LEFT_KP              (18.0F)
#define DRIVE_ORIGINAL_STRAIGHT_LEFT_KI              (2.6F)
#define DRIVE_ORIGINAL_STRAIGHT_LEFT_KD              (1.0F)
#define DRIVE_ORIGINAL_STRAIGHT_RIGHT_KP             (18.0F)
#define DRIVE_ORIGINAL_STRAIGHT_RIGHT_KI             (2.6F)
#define DRIVE_ORIGINAL_STRAIGHT_RIGHT_KD             (1.0F)
#define DRIVE_ORIGINAL_TURN_LEFT_KP                  (18.0F)
#define DRIVE_ORIGINAL_TURN_LEFT_KI                  (2.6F)
#define DRIVE_ORIGINAL_TURN_LEFT_KD                  (1.0F)
#define DRIVE_ORIGINAL_TURN_RIGHT_KP                 (18.0F)
#define DRIVE_ORIGINAL_TURN_RIGHT_KI                 (2.6F)
#define DRIVE_ORIGINAL_TURN_RIGHT_KD                 (1.0F)
#define DRIVE_ORIGINAL_HEADING_KP                    (2.0F)
#define DRIVE_ORIGINAL_HEADING_KI                    (0.0F)
#define DRIVE_ORIGINAL_HEADING_KD                    (0.0F)

/* 520 profile: 13 PPR, x1 decoding and a 30:1 gearbox yield 390 counts/rev. */
#define DRIVE_520_ENCODER_PPR                        (13U)
#define DRIVE_520_ENCODER_DECODE_MULTIPLIER          (1U)
#define DRIVE_520_GEAR_RATIO                         (30U)
#define DRIVE_520_ENCODER_COUNTS_PER_REV             \
    (DRIVE_520_ENCODER_PPR * DRIVE_520_ENCODER_DECODE_MULTIPLIER \
        * DRIVE_520_GEAR_RATIO)
/* The installed 520 wiring reports forward travel with opposite sign. */
#define DRIVE_520_LEFT_ENCODER_COUNT_SIGN            (-1)
#define DRIVE_520_RIGHT_ENCODER_POSITIVE_B_LEVEL     (0U)
/* Both motor channels need inverse bridge polarity for physical forward. */
#define DRIVE_520_LEFT_MOTOR_DUTY_SIGN               (-1)
#define DRIVE_520_RIGHT_MOTOR_DUTY_SIGN              (-1)
#define DRIVE_520_LEFT_WHEEL_DIAMETER_MM             (65.0F)
#define DRIVE_520_RIGHT_WHEEL_DIAMETER_MM            (65.0F)
#define DRIVE_520_TRACK_WIDTH_MM                     (230.0F)
/* Start at 40% duty while validating the new motor installation. */
#define DRIVE_520_SPEED_PID_OUTPUT_LIMIT             (4000)
/* Values are legacy 50000-duty gains scaled to the 10000-duty driver. */
#define DRIVE_520_STRAIGHT_LEFT_KP                   (45.0F)
#define DRIVE_520_STRAIGHT_LEFT_KI                   (51.0F)
#define DRIVE_520_STRAIGHT_LEFT_KD                   (5.6F)
#define DRIVE_520_STRAIGHT_RIGHT_KP                  (44.0F)
#define DRIVE_520_STRAIGHT_RIGHT_KI                  (50.0F)
#define DRIVE_520_STRAIGHT_RIGHT_KD                  (6.0F)
#define DRIVE_520_TURN_LEFT_KP                       (42.0F)
#define DRIVE_520_TURN_LEFT_KI                       (0.0F)
#define DRIVE_520_TURN_LEFT_KD                       (2.0F)
#define DRIVE_520_TURN_RIGHT_KP                      (42.0F)
#define DRIVE_520_TURN_RIGHT_KI                      (0.0F)
#define DRIVE_520_TURN_RIGHT_KD                      (2.0F)
#define DRIVE_520_HEADING_KP                         (28.0F)
#define DRIVE_520_HEADING_KI                         (0.8F)
#define DRIVE_520_HEADING_KD                         (0.0F)

#if (DRIVE_ACTIVE_MOTOR_PROFILE == DRIVE_MOTOR_PROFILE_ORIGINAL)

#define DRIVE_PROFILE_ENCODER_COUNTS_PER_REV         \
    (DRIVE_ORIGINAL_ENCODER_COUNTS_PER_REV)
#define DRIVE_PROFILE_LEFT_ENCODER_COUNT_SIGN        \
    (DRIVE_ORIGINAL_LEFT_ENCODER_COUNT_SIGN)
#define DRIVE_PROFILE_RIGHT_ENCODER_POSITIVE_B_LEVEL \
    (DRIVE_ORIGINAL_RIGHT_ENCODER_POSITIVE_B_LEVEL)
#define DRIVE_PROFILE_LEFT_MOTOR_DUTY_SIGN           \
    (DRIVE_ORIGINAL_LEFT_MOTOR_DUTY_SIGN)
#define DRIVE_PROFILE_RIGHT_MOTOR_DUTY_SIGN          \
    (DRIVE_ORIGINAL_RIGHT_MOTOR_DUTY_SIGN)
#define DRIVE_PROFILE_LEFT_WHEEL_DIAMETER_MM          \
    (DRIVE_ORIGINAL_LEFT_WHEEL_DIAMETER_MM)
#define DRIVE_PROFILE_RIGHT_WHEEL_DIAMETER_MM         \
    (DRIVE_ORIGINAL_RIGHT_WHEEL_DIAMETER_MM)
#define DRIVE_PROFILE_TRACK_WIDTH_MM                  \
    (DRIVE_ORIGINAL_TRACK_WIDTH_MM)
#define DRIVE_PROFILE_SPEED_PID_OUTPUT_LIMIT          \
    (DRIVE_ORIGINAL_SPEED_PID_OUTPUT_LIMIT)
#define DRIVE_PROFILE_STRAIGHT_LEFT_KP                \
    (DRIVE_ORIGINAL_STRAIGHT_LEFT_KP)
#define DRIVE_PROFILE_STRAIGHT_LEFT_KI                \
    (DRIVE_ORIGINAL_STRAIGHT_LEFT_KI)
#define DRIVE_PROFILE_STRAIGHT_LEFT_KD                \
    (DRIVE_ORIGINAL_STRAIGHT_LEFT_KD)
#define DRIVE_PROFILE_STRAIGHT_RIGHT_KP               \
    (DRIVE_ORIGINAL_STRAIGHT_RIGHT_KP)
#define DRIVE_PROFILE_STRAIGHT_RIGHT_KI               \
    (DRIVE_ORIGINAL_STRAIGHT_RIGHT_KI)
#define DRIVE_PROFILE_STRAIGHT_RIGHT_KD               \
    (DRIVE_ORIGINAL_STRAIGHT_RIGHT_KD)
#define DRIVE_PROFILE_TURN_LEFT_KP                    \
    (DRIVE_ORIGINAL_TURN_LEFT_KP)
#define DRIVE_PROFILE_TURN_LEFT_KI                    \
    (DRIVE_ORIGINAL_TURN_LEFT_KI)
#define DRIVE_PROFILE_TURN_LEFT_KD                    \
    (DRIVE_ORIGINAL_TURN_LEFT_KD)
#define DRIVE_PROFILE_TURN_RIGHT_KP                   \
    (DRIVE_ORIGINAL_TURN_RIGHT_KP)
#define DRIVE_PROFILE_TURN_RIGHT_KI                   \
    (DRIVE_ORIGINAL_TURN_RIGHT_KI)
#define DRIVE_PROFILE_TURN_RIGHT_KD                   \
    (DRIVE_ORIGINAL_TURN_RIGHT_KD)
#define DRIVE_PROFILE_HEADING_KP                      \
    (DRIVE_ORIGINAL_HEADING_KP)
#define DRIVE_PROFILE_HEADING_KI                      \
    (DRIVE_ORIGINAL_HEADING_KI)
#define DRIVE_PROFILE_HEADING_KD                      \
    (DRIVE_ORIGINAL_HEADING_KD)

#elif (DRIVE_ACTIVE_MOTOR_PROFILE == DRIVE_MOTOR_PROFILE_520)

#define DRIVE_PROFILE_ENCODER_COUNTS_PER_REV         \
    (DRIVE_520_ENCODER_COUNTS_PER_REV)
#define DRIVE_PROFILE_LEFT_ENCODER_COUNT_SIGN        \
    (DRIVE_520_LEFT_ENCODER_COUNT_SIGN)
#define DRIVE_PROFILE_RIGHT_ENCODER_POSITIVE_B_LEVEL \
    (DRIVE_520_RIGHT_ENCODER_POSITIVE_B_LEVEL)
#define DRIVE_PROFILE_LEFT_MOTOR_DUTY_SIGN           \
    (DRIVE_520_LEFT_MOTOR_DUTY_SIGN)
#define DRIVE_PROFILE_RIGHT_MOTOR_DUTY_SIGN          \
    (DRIVE_520_RIGHT_MOTOR_DUTY_SIGN)
#define DRIVE_PROFILE_LEFT_WHEEL_DIAMETER_MM          \
    (DRIVE_520_LEFT_WHEEL_DIAMETER_MM)
#define DRIVE_PROFILE_RIGHT_WHEEL_DIAMETER_MM         \
    (DRIVE_520_RIGHT_WHEEL_DIAMETER_MM)
#define DRIVE_PROFILE_TRACK_WIDTH_MM                  \
    (DRIVE_520_TRACK_WIDTH_MM)
#define DRIVE_PROFILE_SPEED_PID_OUTPUT_LIMIT          \
    (DRIVE_520_SPEED_PID_OUTPUT_LIMIT)
#define DRIVE_PROFILE_STRAIGHT_LEFT_KP                \
    (DRIVE_520_STRAIGHT_LEFT_KP)
#define DRIVE_PROFILE_STRAIGHT_LEFT_KI                \
    (DRIVE_520_STRAIGHT_LEFT_KI)
#define DRIVE_PROFILE_STRAIGHT_LEFT_KD                \
    (DRIVE_520_STRAIGHT_LEFT_KD)
#define DRIVE_PROFILE_STRAIGHT_RIGHT_KP               \
    (DRIVE_520_STRAIGHT_RIGHT_KP)
#define DRIVE_PROFILE_STRAIGHT_RIGHT_KI               \
    (DRIVE_520_STRAIGHT_RIGHT_KI)
#define DRIVE_PROFILE_STRAIGHT_RIGHT_KD               \
    (DRIVE_520_STRAIGHT_RIGHT_KD)
#define DRIVE_PROFILE_TURN_LEFT_KP                    \
    (DRIVE_520_TURN_LEFT_KP)
#define DRIVE_PROFILE_TURN_LEFT_KI                    \
    (DRIVE_520_TURN_LEFT_KI)
#define DRIVE_PROFILE_TURN_LEFT_KD                    \
    (DRIVE_520_TURN_LEFT_KD)
#define DRIVE_PROFILE_TURN_RIGHT_KP                   \
    (DRIVE_520_TURN_RIGHT_KP)
#define DRIVE_PROFILE_TURN_RIGHT_KI                   \
    (DRIVE_520_TURN_RIGHT_KI)
#define DRIVE_PROFILE_TURN_RIGHT_KD                   \
    (DRIVE_520_TURN_RIGHT_KD)
#define DRIVE_PROFILE_HEADING_KP                      (DRIVE_520_HEADING_KP)
#define DRIVE_PROFILE_HEADING_KI                      (DRIVE_520_HEADING_KI)
#define DRIVE_PROFILE_HEADING_KD                      (DRIVE_520_HEADING_KD)

#else
#error "DRIVE_ACTIVE_MOTOR_PROFILE is invalid"
#endif

#endif
