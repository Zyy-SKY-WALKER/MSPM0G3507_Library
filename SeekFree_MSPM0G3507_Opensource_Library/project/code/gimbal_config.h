/**
 * @file    gimbal_config.h
 * @brief   Default configuration for the two-axis gimbal framework.
 *
 * Values marked nominal are placeholders for bench validation. Replace them
 * after the mechanical and target measurements are available.
 */

#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/** Nonzero only when the physical gimbal is installed on the vehicle. */
#define GIMBAL_CONFIG_INSTALLED                (0U)

#define GIMBAL_CONFIG_STEPS_PER_REVOLUTION       (12800U)
#define GIMBAL_CONFIG_PULSE_ENGINE_HZ           (10000U)
#define GIMBAL_CONFIG_PULSE_TICK_US             (100U)
#define GIMBAL_CONFIG_CONTROL_PERIOD_MS         (10U)

#define GIMBAL_CONFIG_YAW_MIN_STEPS             (0)
#define GIMBAL_CONFIG_YAW_MAX_STEPS             (13511)
#define GIMBAL_CONFIG_PITCH_LOW_ANGLE_STEPS     (-1067)
#define GIMBAL_CONFIG_PITCH_HIGH_ANGLE_STEPS    (1067)
#define GIMBAL_CONFIG_YAW_CALIBRATE_STEPS       (0)
#define GIMBAL_CONFIG_PITCH_CALIBRATE_STEPS     (0)
#define GIMBAL_CONFIG_ALLOW_PREZERO_YAW_JOG     (0U)
#define GIMBAL_CONFIG_ALLOW_PREZERO_PITCH_JOG   (0U)

#define GIMBAL_CONFIG_YAW_JOG_SPEED_DEG_S       (90U)
#define GIMBAL_CONFIG_PITCH_JOG_SPEED_DEG_S     (90U)
#define GIMBAL_CONFIG_CALIBRATE_SPEED_DEG_S     (5U)
#define GIMBAL_CONFIG_POSITION_SPEED_DEG_S      (90U)
#define GIMBAL_CONFIG_ACCELERATION_DEG_S2       (540U)
#define GIMBAL_CONFIG_POSITION_GAIN_MILLI_RATE  (10000)

#define GIMBAL_CONFIG_YAW_POSITION_SIGN         (1)
#define GIMBAL_CONFIG_PITCH_POSITION_SIGN       (1)
#define GIMBAL_CONFIG_YAW_POSITIVE_DIR_LEVEL    (1U)
#define GIMBAL_CONFIG_PITCH_POSITIVE_DIR_LEVEL  (1U)
#define GIMBAL_CONFIG_LASER_POLARITY_VALID      (1U)
#define GIMBAL_CONFIG_LASER_ACTIVE_LEVEL        (0U)
#define GIMBAL_CONFIG_LASER_SETTLE_MS           (200U)

#define GIMBAL_CONFIG_YAW_MIN_DEG               (0.0F)
#define GIMBAL_CONFIG_YAW_MAX_DEG               (380.0F)
#define GIMBAL_CONFIG_YAW_KINEMATIC_SIGN        (-1.0F)
#define GIMBAL_CONFIG_PITCH_ZERO_DEG             (0.0F)
#define GIMBAL_CONFIG_PITCH_MIN_DEG              (-30.0F)
#define GIMBAL_CONFIG_PITCH_MAX_DEG              (30.0F)

/* Nominal world target: AB is +X and the field is +Y. */
#define GIMBAL_CONFIG_TARGET_CENTER_X_MM        (0.0F)
#define GIMBAL_CONFIG_TARGET_CENTER_Y_MM        (-500.0F)
#define GIMBAL_CONFIG_TARGET_CENTER_Z_MM        (250.0F)
#define GIMBAL_CONFIG_TARGET_AXIS_X_X           (1.0F)
#define GIMBAL_CONFIG_TARGET_AXIS_X_Y           (0.0F)
#define GIMBAL_CONFIG_TARGET_AXIS_X_Z           (0.0F)
#define GIMBAL_CONFIG_TARGET_AXIS_Y_X           (0.0F)
#define GIMBAL_CONFIG_TARGET_AXIS_Y_Y           (0.0F)
#define GIMBAL_CONFIG_TARGET_AXIS_Y_Z           (1.0F)
#define GIMBAL_CONFIG_TARGET_CIRCLE_RADIUS_MM   (60.0F)

/* Nominal body-frame installation geometry. */
#define GIMBAL_CONFIG_PAN_ORIGIN_X_MM           (96.0F)
#define GIMBAL_CONFIG_PAN_ORIGIN_Y_MM           (0.0F)
#define GIMBAL_CONFIG_PAN_ORIGIN_Z_MM           (147.0F)
#define GIMBAL_CONFIG_PAN_MOUNT_ROLL_DEG        (0.0F)
#define GIMBAL_CONFIG_PAN_MOUNT_PITCH_DEG       (0.0F)
#define GIMBAL_CONFIG_PAN_MOUNT_YAW_DEG         (-90.0F)
#define GIMBAL_CONFIG_TILT_ORIGIN_X_MM          (0.0F)
#define GIMBAL_CONFIG_TILT_ORIGIN_Y_MM          (0.0F)
#define GIMBAL_CONFIG_TILT_ORIGIN_Z_MM          (0.0F)
#define GIMBAL_CONFIG_TILT_MOUNT_ROLL_DEG       (0.0F)
#define GIMBAL_CONFIG_TILT_MOUNT_PITCH_DEG      (0.0F)
#define GIMBAL_CONFIG_TILT_MOUNT_YAW_DEG        (0.0F)
#define GIMBAL_CONFIG_LASER_ORIGIN_X_MM         (22.5F)
#define GIMBAL_CONFIG_LASER_ORIGIN_Y_MM         (0.0F)
#define GIMBAL_CONFIG_LASER_ORIGIN_Z_MM         (60.0F)
#define GIMBAL_CONFIG_LASER_ROLL_OFFSET_DEG     (0.0F)
#define GIMBAL_CONFIG_LASER_PITCH_OFFSET_DEG    (0.0F)
#define GIMBAL_CONFIG_LASER_YAW_OFFSET_DEG      (0.0F)

#define GIMBAL_CONFIG_FEEDFORWARD_ITERATIONS    (6U)
#define GIMBAL_CONFIG_FEEDFORWARD_JACOBIAN_DEG  (0.5F)
#define GIMBAL_CONFIG_FEEDFORWARD_TOLERANCE_DEG (1.0F)
#define GIMBAL_CONFIG_FEEDFORWARD_MAX_STEP_DEG  (10.0F)

/* The flat-course feedforward ignores drifting body tilt estimates. */
#define GIMBAL_CONFIG_USE_BODY_ROLL              (0U)
#define GIMBAL_CONFIG_USE_BODY_PITCH             (0U)

#define GIMBAL_CONFIG_CAMERA_TIMEOUT_MS         (120U)

#endif
