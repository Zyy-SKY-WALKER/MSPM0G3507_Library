/**
 * @file    chassis_motion.h
 * @brief   Non-blocking high-level differential-drive motion commands.
 */

#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include "odometry.h"
#include "zf_common_typedef.h"

#define CHASSIS_MOTION_UPDATE_PERIOD_MS               (10U)
#define CHASSIS_MOTION_DEFAULT_SPEED_TRANSITION_MS    (300U)
#define CHASSIS_MOTION_STOPPED_COUNT_LIMIT            (3)
#define CHASSIS_MOTION_STOPPED_CONFIRM_TICKS          (3U)
#define CHASSIS_MOTION_TURN_TOLERANCE_DEG             (2.0F)
#define CHASSIS_MOTION_PID_PROFILE_COUNT              (4U)
#define CHASSIS_MOTION_PID_PROFILE_INVALID            (0xFFU)

typedef enum
{
    CHASSIS_MOTION_PID_PROFILE_STRAIGHT = 0,
    CHASSIS_MOTION_PID_PROFILE_TURN,
    CHASSIS_MOTION_PID_PROFILE_LINE,
    CHASSIS_MOTION_PID_PROFILE_CUSTOM,
} chassis_motion_pid_profile_id_enum;

typedef enum
{
    CHASSIS_MOTION_COMMAND_NONE = 0,
    CHASSIS_MOTION_COMMAND_DISTANCE,
    CHASSIS_MOTION_COMMAND_TIMED,
    CHASSIS_MOTION_COMMAND_TURN_RELATIVE,
} chassis_motion_command_enum;

typedef enum
{
    CHASSIS_MOTION_PHASE_IDLE = 0,
    CHASSIS_MOTION_PHASE_TRANSITION,
    CHASSIS_MOTION_PHASE_EXECUTING,
    CHASSIS_MOTION_PHASE_DECELERATING,
    CHASSIS_MOTION_PHASE_WAIT_STOPPED,
    CHASSIS_MOTION_PHASE_COMPLETED,
    CHASSIS_MOTION_PHASE_CANCELLED,
} chassis_motion_phase_enum;

typedef enum
{
    CHASSIS_MOTION_RESULT_NONE = 0,
    CHASSIS_MOTION_RESULT_COMPLETED,
    CHASSIS_MOTION_RESULT_CANCELLED,
} chassis_motion_result_enum;

typedef struct
{
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float heading_kp;
    float heading_ki;
    float heading_kd;
} chassis_motion_pid_profile_struct;

typedef struct
{
    uint16 speed_transition_ms;
} chassis_motion_config_struct;

typedef struct
{
    chassis_motion_command_enum command;
    chassis_motion_phase_enum phase;
    chassis_motion_result_enum result;
    float command_value;
    float command_speed;
    float start_center_displacement_mm;
    float current_center_displacement_mm;
    float start_heading_deg;
    float current_heading_deg;
    float target_heading_deg;
    float left_target_mm_s;
    float right_target_mm_s;
    uint32 elapsed_ms;
    uint8 active_profile_id;
    uint8 active;
    uint8 reversing;
} chassis_motion_status_struct;

/**
 * @brief Initialize motion state and clear all PID profile selections.
 * @note Call after speed_pid_init() and before the 10 ms scheduler starts.
 */
void chassis_motion_init(void);

/**
 * @brief Configure one reusable speed and heading PID parameter group.
 * @param profile_id Profile identifier from 0 to 3.
 * @param profile Shared wheel-speed and heading PID gains.
 * @return ZF_TRUE when the profile was accepted.
 */
uint8 chassis_motion_pid_profile_configure(
    uint8 profile_id,
    const chassis_motion_pid_profile_struct *profile);

/**
 * @brief Apply one configured PID parameter group without resetting runtime state.
 * @param profile_id Profile identifier from 0 to 3.
 * @return ZF_TRUE when the configured profile was selected.
 * @note Call from the unique 10 ms scheduler context.
 */
uint8 chassis_motion_pid_profile_select(uint8 profile_id);

/**
 * @brief Return the currently selected PID parameter group.
 * @return Profile identifier, or CHASSIS_MOTION_PID_PROFILE_INVALID.
 */
uint8 chassis_motion_pid_profile_get_active(void);

/**
 * @brief Update the motion configuration used by subsequent transitions.
 * @param config Motion configuration.
 * @return ZF_TRUE when the configuration was accepted.
 */
uint8 chassis_motion_set_config(const chassis_motion_config_struct *config);

/**
 * @brief Start or smoothly replan a signed distance command.
 * @param distance_mm Positive for forward and negative for reverse travel.
 * @param max_speed_mm_s Positive maximum center speed in millimeters per second.
 * @return ZF_TRUE when the command was accepted.
 * @note Call from the unique 10 ms scheduler context.
 */
uint8 chassis_motion_start_distance(
    float distance_mm,
    float max_speed_mm_s);

/**
 * @brief Start or smoothly replan a timed signed-speed command.
 * @param speed_mm_s Positive for forward and negative for reverse speed.
 * @param duration_ms Command duration in milliseconds.
 * @return ZF_TRUE when the command was accepted.
 * @note Call from the unique 10 ms scheduler context.
 */
uint8 chassis_motion_start_timed(
    float speed_mm_s,
    uint32 duration_ms);

/**
 * @brief Start or smoothly replan a relative IMU-heading turn.
 * @param angle_deg Relative angle from -180 to 180 degrees, excluding zero.
 *                  Positive turns left and negative turns right.
 * @param max_angular_speed_deg_s Positive angular-speed limit in degrees per second.
 * @return ZF_TRUE when the command was accepted.
 * @note Call from the unique 10 ms scheduler context.
 */
uint8 chassis_motion_start_turn_relative(
    float angle_deg,
    float max_angular_speed_deg_s);

/**
 * @brief Smoothly reduce the current motion target to zero and cancel it.
 * @note Call from the unique 10 ms scheduler context.
 */
void chassis_motion_cancel(void);

/**
 * @brief Immediately clear motion state and force this module's target to zero.
 * @note Use for scheduler disarm and fault paths.
 */
void chassis_motion_reset(void);

/**
 * @brief Advance one non-blocking motion command by one 10 ms period.
 * @param odometry Latest odometry state from this scheduler period.
 * @param yaw_deg Latest IMU yaw angle in degrees.
 * @param left_count Latest signed left encoder delta.
 * @param right_count Latest signed right encoder delta.
 * @param left_target_mm_s Destination left wheel speed target.
 * @param right_target_mm_s Destination right wheel speed target.
 * @note Call from exactly one 10 ms periodic context.
 */
void chassis_motion_update_10ms(
    const odometry_state_struct *odometry,
    float yaw_deg,
    int16 left_count,
    int16 right_count,
    float *left_target_mm_s,
    float *right_target_mm_s);

/**
 * @brief Return whether a command or controlled stop is still active.
 * @return Nonzero while the module owns chassis wheel targets.
 */
uint8 chassis_motion_is_busy(void);

/**
 * @brief Copy the latest chassis motion status.
 * @param status Destination status structure.
 */
void chassis_motion_get_status(chassis_motion_status_struct *status);

#endif
