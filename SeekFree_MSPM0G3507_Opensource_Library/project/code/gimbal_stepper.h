/**
 * @file    gimbal_stepper.h
 * @brief   Reusable dual-axis STEP/DIR gimbal controller.
 */

#ifndef GIMBAL_STEPPER_H
#define GIMBAL_STEPPER_H

#include "zf_common_typedef.h"

#include "gimbal_config.h"

#define GIMBAL_STEPPER_STEPS_PER_REVOLUTION     \
    (GIMBAL_CONFIG_STEPS_PER_REVOLUTION)
#define GIMBAL_STEPPER_YAW_MIN_STEPS            \
    (GIMBAL_CONFIG_YAW_MIN_STEPS)
#define GIMBAL_STEPPER_YAW_MAX_STEPS            \
    (GIMBAL_CONFIG_YAW_MAX_STEPS)
#define GIMBAL_STEPPER_PITCH_MIN_STEPS          \
    ((GIMBAL_CONFIG_PITCH_POSITION_SIGN > 0) \
        ? GIMBAL_CONFIG_PITCH_LOW_ANGLE_STEPS \
        : -GIMBAL_CONFIG_PITCH_HIGH_ANGLE_STEPS)
#define GIMBAL_STEPPER_PITCH_MAX_STEPS          \
    ((GIMBAL_CONFIG_PITCH_POSITION_SIGN > 0) \
        ? GIMBAL_CONFIG_PITCH_HIGH_ANGLE_STEPS \
        : -GIMBAL_CONFIG_PITCH_LOW_ANGLE_STEPS)

typedef enum
{
    GIMBAL_STEPPER_AXIS_YAW = 0,
    GIMBAL_STEPPER_AXIS_PITCH,
    GIMBAL_STEPPER_AXIS_COUNT,
} gimbal_stepper_axis_enum;

typedef struct
{
    int32 position_steps;
    int32 target_position_steps;
    int32 current_rate_steps_s;
    uint8 zero_valid;
} gimbal_stepper_axis_status_struct;

typedef struct
{
    gimbal_stepper_axis_status_struct axis[
        GIMBAL_STEPPER_AXIS_COUNT];
    gimbal_stepper_axis_enum selected_axis;
    uint8 stop_latched;
    uint8 relative_ready;
    uint8 negative_key_pressed;
    uint8 positive_key_pressed;
    uint8 select_key_pressed;
    uint8 laser_enabled;
    uint8 manual_control_enabled;
} gimbal_stepper_status_struct;

/** @brief Feedforward target selection. */
typedef enum
{
    /** Aim at the configured target center. */
    GIMBAL_TARGET_CENTER = 0,
    /** Aim at the configured target circle phase. */
    GIMBAL_TARGET_CIRCLE,
} gimbal_target_mode_enum;

/** @brief Vehicle pose supplied to the gimbal feedforward solver. */
typedef struct
{
    /** Vehicle origin in world millimeters. */
    float x_mm;
    float y_mm;
    float z_mm;
    /** Vehicle attitude in degrees. */
    float roll_deg;
    float pitch_deg;
    /** Continuous world yaw in radians. */
    float heading_rad;
    /** Nonzero when the pose is usable. */
    uint8 valid;
} gimbal_feedforward_pose_struct;

/** @brief Result returned by the geometric feedforward inverse solver. */
typedef struct
{
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float yaw_deg;
    float pitch_deg;
    float residual_deg;
    uint8 valid;
    uint8 singular;
} gimbal_feedforward_solution_struct;

/** @brief Camera-to-MSPM0 visual error placeholder until the UART protocol is fixed. */
typedef struct
{
    int16 error_x_mm;
    int16 error_y_mm;
    uint16 sequence;
    uint16 age_ms;
    uint8 target_valid;
    uint8 spot_valid;
    uint8 mode;
} gimbal_camera_error_struct;

/** @brief Optional foreground log sink used by tests and applications. */
typedef void (*gimbal_stepper_log_callback)(const char *message);

/**
 * @brief Initialize the gimbal GPIOs, keys and configured pulse timer.
 */
void gimbal_stepper_init(void);

/**
 * @brief Process elapsed milliseconds reported by the pulse timer.
 * @return Number of milliseconds processed by this service call.
 * @note Call repeatedly from the foreground application loop.
 */
uint16 gimbal_stepper_service(void);

/**
 * @brief Add one relative target movement to both axes.
 * @param yaw_delta_steps Signed yaw target increment.
 * @param pitch_delta_steps Signed pitch target increment.
 * @return 1 when accepted, otherwise 0.
 */
uint8 gimbal_stepper_move_relative_steps(
    int32 yaw_delta_steps,
    int32 pitch_delta_steps);

/**
 * @brief Check whether relative position commands may be accepted.
 * @return 1 when both axes are zeroed and manual controls are inactive.
 */
uint8 gimbal_stepper_relative_ready(void);

/**
 * @brief Copy one atomic runtime status snapshot.
 * @param status Destination status structure.
 */
void gimbal_stepper_get_status(
    gimbal_stepper_status_struct *status);

/**
 * @brief Replace the foreground text log destination.
 * @param callback Log sink, or NULL to restore debug printf output.
 */
void gimbal_stepper_set_log_callback(
    gimbal_stepper_log_callback callback);

/**
 * @brief Enable or disable direct A30/B0/B1 gimbal calibration controls.
 * @param enabled Nonzero while the gimbal owns the shared physical keys.
 */
void gimbal_stepper_set_manual_control_enabled(uint8 enabled);

/**
 * @brief Select the target point used by the feedforward solver.
 * @param mode Center or circle target mode.
 */
void gimbal_stepper_set_target_mode(gimbal_target_mode_enum mode);

/**
 * @brief Set the target-circle phase in radians.
 * @param phase_rad Desired phase; it is wrapped internally.
 */
void gimbal_stepper_set_target_phase(float phase_rad);

/**
 * @brief Convert a world pose and target configuration into axis targets.
 * @param pose Vehicle pose snapshot.
 * @return 1 when a valid target was accepted.
 */
uint8 gimbal_stepper_update_feedforward(
    const gimbal_feedforward_pose_struct *pose);

/**
 * @brief Calculate feedforward without changing motor targets.
 * @param pose Vehicle pose snapshot.
 * @param solution Destination solution.
 * @return 1 when the configured target is reachable.
 */
uint8 gimbal_stepper_compute_feedforward(
    const gimbal_feedforward_pose_struct *pose,
    gimbal_feedforward_solution_struct *solution);

/**
 * @brief Apply one previously computed feedforward solution.
 * @param solution Valid geometric inverse solution.
 * @return 1 when the bounded motor targets were accepted.
 */
uint8 gimbal_stepper_apply_feedforward_solution(
    const gimbal_feedforward_solution_struct *solution);

/**
 * @brief Set absolute step targets after a valid manual zero.
 * @param yaw_steps Signed yaw target relative to the manual zero.
 * @param pitch_steps Signed pitch target relative to the manual zero.
 * @return 1 when accepted.
 */
uint8 gimbal_stepper_set_absolute_target_steps(
    int32 yaw_steps,
    int32 pitch_steps);

/**
 * @brief Copy the latest feedforward solution.
 * @param solution Destination snapshot.
 */
void gimbal_stepper_get_feedforward_solution(
    gimbal_feedforward_solution_struct *solution);

/**
 * @brief Initialize the laser output to its configured safe-off level.
 */
void gimbal_stepper_laser_init(void);

/**
 * @brief Set the laser output using the configured active level.
 * @param enabled Nonzero to request laser on.
 * @return 1 when the requested state was accepted.
 */
uint8 gimbal_stepper_set_laser(uint8 enabled);

#endif
