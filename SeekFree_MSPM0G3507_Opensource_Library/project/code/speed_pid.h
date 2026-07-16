/**
 * @file    speed_pid.h
 * @brief   Dual-wheel incremental speed PID controller.
 */

#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "zf_common_typedef.h"

#define SPEED_PID_SAMPLE_PERIOD_MS        (10U)
#define SPEED_PID_TARGET_LIMIT_MM_S       (800.0F)
#define SPEED_PID_OUTPUT_LIMIT            (8000)
#define SPEED_PID_REVERSE_STOP_COUNT      (3)

#define SPEED_PID_LEFT_KP                 (9.0F)
#define SPEED_PID_LEFT_KI                 (1.3F)
#define SPEED_PID_LEFT_KD                 (0.5F)

#define SPEED_PID_RIGHT_KP                (9.0F)
#define SPEED_PID_RIGHT_KI                (1.5F)
#define SPEED_PID_RIGHT_KD                (0.8F)

typedef struct
{
    float left_target_mm_s;
    float right_target_mm_s;
    float left_speed_mm_s;
    float right_speed_mm_s;
    int16 left_count;
    int16 right_count;
    int16 left_duty;
    int16 right_duty;
    uint8 left_saturated;
    uint8 right_saturated;
    uint8 left_reversing;
    uint8 right_reversing;
} speed_pid_status_struct;

void speed_pid_init(void);
void speed_pid_set_target(float left_mm_s, float right_mm_s);

/**
 * @brief Apply one shared PID gain set to both wheel controllers.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @return ZF_TRUE when the gains were accepted.
 * @note Runtime state and current output are preserved during the update.
 */
uint8 speed_pid_set_shared_gains(float kp, float ki, float kd);

/**
 * @brief Execute one fixed-period speed control update.
 * @note Call from exactly one 10 ms periodic context.
 */
void speed_pid_update_10ms(int16 left_count, int16 right_count);
void speed_pid_stop(void);
void speed_pid_reset(void);

void speed_pid_set_left_gains(float kp, float ki, float kd);
void speed_pid_set_right_gains(float kp, float ki, float kd);
void speed_pid_get_status(speed_pid_status_struct *status);

#endif
