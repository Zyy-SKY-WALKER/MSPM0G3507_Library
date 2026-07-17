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

/** @brief Identifiers for reusable speed and heading PID gain groups. */
typedef enum
{
    /** Straight-distance motion gains. */
    CHASSIS_MOTION_PID_PROFILE_STRAIGHT = 0,
    /** In-place heading-turn gains. */
    CHASSIS_MOTION_PID_PROFILE_TURN,
    /** Line-following support gains. */
    CHASSIS_MOTION_PID_PROFILE_LINE,
    /** Application-defined gain group. */
    CHASSIS_MOTION_PID_PROFILE_CUSTOM,
} chassis_motion_pid_profile_id_enum;

/** @brief High-level command interpreted by the chassis state machine. */
typedef enum
{
    /** No motion command is selected. */
    CHASSIS_MOTION_COMMAND_NONE = 0,
    /** Travel a signed center distance. */
    CHASSIS_MOTION_COMMAND_DISTANCE,
    /** Hold a signed center speed for a duration. */
    CHASSIS_MOTION_COMMAND_TIMED,
    /** Turn through a signed relative IMU heading. */
    CHASSIS_MOTION_COMMAND_TURN_RELATIVE,
} chassis_motion_command_enum;

/** @brief Observable phases of the non-blocking motion state machine. */
typedef enum
{
    /** No command owns wheel targets. */
    CHASSIS_MOTION_PHASE_IDLE = 0,
    /** Base speed is ramping toward the command target. */
    CHASSIS_MOTION_PHASE_TRANSITION,
    /** The command is running at its generated target. */
    CHASSIS_MOTION_PHASE_EXECUTING,
    /** Base speed is ramping down to zero. */
    CHASSIS_MOTION_PHASE_DECELERATING,
    /** Zero target is held while encoder stop is confirmed. */
    CHASSIS_MOTION_PHASE_WAIT_STOPPED,
    /** The command and controlled stop completed. */
    CHASSIS_MOTION_PHASE_COMPLETED,
    /** A cancellation and controlled stop completed. */
    CHASSIS_MOTION_PHASE_CANCELLED,
} chassis_motion_phase_enum;

/** @brief Terminal outcome retained after motion ownership ends. */
typedef enum
{
    /** No terminal result is available. */
    CHASSIS_MOTION_RESULT_NONE = 0,
    /** The command reached its completion condition. */
    CHASSIS_MOTION_RESULT_COMPLETED,
    /** The command was cancelled before completion. */
    CHASSIS_MOTION_RESULT_CANCELLED,
} chassis_motion_result_enum;

/** @brief Reusable wheel-speed and heading-controller gains. */
typedef struct
{
    /** Left wheel-speed proportional gain. */
    float left_speed_kp;
    /** Left wheel-speed integral gain. */
    float left_speed_ki;
    /** Left wheel-speed derivative gain. */
    float left_speed_kd;
    /** Right wheel-speed proportional gain. */
    float right_speed_kp;
    /** Right wheel-speed integral gain. */
    float right_speed_ki;
    /** Right wheel-speed derivative gain. */
    float right_speed_kd;
    /** Heading-correction proportional gain. */
    float heading_kp;
    /** Heading-correction integral gain. */
    float heading_ki;
    /** Heading-correction derivative gain. */
    float heading_kd;
} chassis_motion_pid_profile_struct;

/** @brief Runtime configuration for command speed transitions. */
typedef struct
{
    /** Duration of each Smoothstep speed ramp in milliseconds. */
    uint16 speed_transition_ms;
} chassis_motion_config_struct;

/** @brief Observable command, phase, target and pose state. */
typedef struct
{
    /** Current or most recently completed high-level command kind. */
    chassis_motion_command_enum command;
    /** Current state-machine phase. */
    chassis_motion_phase_enum phase;
    /** Retained terminal command outcome. */
    chassis_motion_result_enum result;
    /** Signed distance in mm or turn angle in degrees; zero for timed speed. */
    float command_value;
    /** Linear speed in mm/s or angular speed limit in degrees per second. */
    float command_speed;
    /** Center displacement captured when the command activates. */
    float start_center_displacement_mm;
    /** Latest center displacement from odometry. */
    float current_center_displacement_mm;
    /** IMU heading captured when the command activates. */
    float start_heading_deg;
    /** Latest IMU heading. */
    float current_heading_deg;
    /** Heading held by straight motion or targeted by a turn. */
    float target_heading_deg;
    /** Latest generated left-wheel speed target. */
    float left_target_mm_s;
    /** Latest generated right-wheel speed target. */
    float right_target_mm_s;
    /** Command execution time accumulated in 10 ms periods. */
    uint32 elapsed_ms;
    /** Currently selected configured PID profile. */
    uint8 active_profile_id;
    /** Nonzero while this module owns wheel targets. */
    uint8 active;
    /** Nonzero while stopping before an opposite-direction command. */
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
 * @param profile Independent wheel-speed and heading PID gains.
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
