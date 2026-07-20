/**
 * @file    gimbal_stepper.h
 * @brief   Reusable dual-axis STEP/DIR gimbal controller.
 */

#ifndef GIMBAL_STEPPER_H
#define GIMBAL_STEPPER_H

#include "zf_common_typedef.h"

#define GIMBAL_STEPPER_STEPS_PER_REVOLUTION     (12800U)
#define GIMBAL_STEPPER_YAW_MIN_STEPS            (-6400)
#define GIMBAL_STEPPER_YAW_MAX_STEPS            (6400)
#define GIMBAL_STEPPER_PITCH_MIN_STEPS          (-3700)
#define GIMBAL_STEPPER_PITCH_MAX_STEPS          (3400)

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
} gimbal_stepper_status_struct;

/**
 * @brief Initialize the gimbal GPIOs, keys and 5 kHz pulse timer.
 */
void gimbal_stepper_init(void);

/**
 * @brief Process elapsed milliseconds reported by the 5 kHz pulse timer.
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

#endif
