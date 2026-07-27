/**
 * @file    speed_pid.h
 * @brief   Dual-wheel incremental speed PID controller.
 */

#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "drive_motor_config.h"
#include "zf_common_typedef.h"

#define SPEED_PID_SAMPLE_PERIOD_MS        (10U)
#define SPEED_PID_TARGET_LIMIT_MM_S       (800.0F)
#define SPEED_PID_OUTPUT_LIMIT            \
    (DRIVE_PROFILE_SPEED_PID_OUTPUT_LIMIT)
#define SPEED_PID_REVERSE_STOP_COUNT      (1)

#define SPEED_PID_LEFT_KP                 (DRIVE_PROFILE_STRAIGHT_LEFT_KP)
#define SPEED_PID_LEFT_KI                 (DRIVE_PROFILE_STRAIGHT_LEFT_KI)
#define SPEED_PID_LEFT_KD                 (DRIVE_PROFILE_STRAIGHT_LEFT_KD)

#define SPEED_PID_RIGHT_KP                (DRIVE_PROFILE_STRAIGHT_RIGHT_KP)
#define SPEED_PID_RIGHT_KI                (DRIVE_PROFILE_STRAIGHT_RIGHT_KI)
#define SPEED_PID_RIGHT_KD                (DRIVE_PROFILE_STRAIGHT_RIGHT_KD)

/** @brief Latest dual-wheel command, measurement and controller state. */
typedef struct
{
    /** Applied signed left target in millimeters per second. */
    float left_target_mm_s;
    /** Applied signed right target in millimeters per second. */
    float right_target_mm_s;
    /** Latest measured left speed in millimeters per second. */
    float left_speed_mm_s;
    /** Latest measured right speed in millimeters per second. */
    float right_speed_mm_s;
    /** Signed left encoder count from the latest 10 ms interval. */
    int16 left_count;
    /** Signed right encoder count from the latest 10 ms interval. */
    int16 right_count;
    /** Signed left motor duty bounded by SPEED_PID_OUTPUT_LIMIT. */
    int16 left_duty;
    /** Signed right motor duty bounded by SPEED_PID_OUTPUT_LIMIT. */
    int16 right_duty;
    /** Nonzero when the latest left PID output was clamped. */
    uint8 left_saturated;
    /** Nonzero when the latest right PID output was clamped. */
    uint8 right_saturated;
    /** Nonzero while the left motor waits to stop before reversing. */
    uint8 left_reversing;
    /** Nonzero while the right motor waits to stop before reversing. */
    uint8 right_reversing;
} speed_pid_status_struct;

/**
 * @brief Initialize both incremental controllers and stopped motor outputs.
 */
void speed_pid_init(void);

/**
 * @brief Set signed left and right wheel speed targets.
 * @param left_mm_s Left target in millimeters per second.
 * @param right_mm_s Right target in millimeters per second.
 * @note Each target is clamped to +/-800 mm/s; NaN becomes zero.
 * @note A direct sign reversal holds duty at zero until the wheel stops.
 */
void speed_pid_set_target(float left_mm_s, float right_mm_s);

/**
 * @brief Apply one shared PID gain set to both wheel controllers.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @return ZF_TRUE when the gains were accepted.
 * @note Each gain must be finite and in the range -1000 through 1000.
 * @note Runtime state and current output are preserved during the update.
 */
uint8 speed_pid_set_shared_gains(float kp, float ki, float kd);

/**
 * @brief Execute one fixed-period speed control update.
 * @param left_count Signed left encoder count from the latest 10 ms interval.
 * @param right_count Signed right encoder count from the latest 10 ms interval.
 * @note Call from exactly one 10 ms periodic context.
 * @note Updates both motor outputs and the observable status snapshot.
 */
void speed_pid_update_10ms(int16 left_count, int16 right_count);

/**
 * @brief Clear both controllers, targets and motor outputs.
 */
void speed_pid_stop(void);

/**
 * @brief Reset both controller histories, status and motor outputs.
 */
void speed_pid_reset(void);

/**
 * @brief Set left-wheel gains and clear its error and output history.
 * @param kp Finite proportional gain from -1000 through 1000.
 * @param ki Finite integral gain from -1000 through 1000.
 * @param kd Finite derivative gain from -1000 through 1000.
 * @note Invalid gains are ignored; motor duty changes on the next update.
 * @note An active direction-reversal wait is preserved.
 */
void speed_pid_set_left_gains(float kp, float ki, float kd);

/**
 * @brief Set right-wheel gains and clear its error and output history.
 * @param kp Finite proportional gain from -1000 through 1000.
 * @param ki Finite integral gain from -1000 through 1000.
 * @param kd Finite derivative gain from -1000 through 1000.
 * @note Invalid gains are ignored; motor duty changes on the next update.
 * @note An active direction-reversal wait is preserved.
 */
void speed_pid_set_right_gains(float kp, float ki, float kd);

/**
 * @brief Copy one interrupt-coherent dual-wheel status snapshot.
 * @param status Destination status; NULL causes no operation.
 */
void speed_pid_get_status(speed_pid_status_struct *status);

#endif
