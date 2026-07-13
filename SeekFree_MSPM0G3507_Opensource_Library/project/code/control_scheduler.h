/**
 * @file    control_scheduler.h
 * @brief   Unified bare-metal 10 ms vehicle control scheduler.
 */

#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include "gray_sensor.h"
#include "line_tracker.h"
#include "odometry.h"
#include "speed_pid.h"
#include "zf_common_typedef.h"

#define CONTROL_SCHEDULER_PERIOD_MS          (10U)
#define CONTROL_SCHEDULER_MANUAL_TIMEOUT_MS  (3000U)

typedef enum
{
    CONTROL_MODE_BOOT = 0,
    CONTROL_MODE_DISARMED,
    CONTROL_MODE_MANUAL_ARMED,
    CONTROL_MODE_LINE_FOLLOW,
    CONTROL_MODE_FAULT_LATCHED,
} control_mode_enum;

typedef enum
{
    CONTROL_FAULT_NONE = 0x00000000U,
    CONTROL_FAULT_EMERGENCY_KEY = 0x00000001U,
    CONTROL_FAULT_GRAY_INIT = 0x00000002U,
    CONTROL_FAULT_GRAY_SAMPLE = 0x00000004U,
    CONTROL_FAULT_ENCODER_RANGE = 0x00000008U,
    CONTROL_FAULT_LINE_TRACKER = 0x00000010U,
    CONTROL_FAULT_REENTRY = 0x00000020U,
} control_fault_enum;

typedef struct
{
    control_mode_enum mode;
    uint32 fault_flags;
    uint32 tick_count;
    uint32 overrun_count;
    uint32 imu_angle_frame_count;
    uint16 imu_age_ticks;
    int16 left_count;
    int16 right_count;
    float imu_yaw_deg;
    float manual_left_target_mm_s;
    float manual_right_target_mm_s;
    gray_sensor_result_struct gray;
    line_tracker_output_struct line_output;
    line_tracker_status_struct line_status;
    speed_pid_status_struct speed;
    odometry_state_struct odometry;
    uint8 initialized;
    uint8 started;
    uint8 imu_valid;
    uint8 imu_fresh;
} control_scheduler_status_struct;

uint8 control_scheduler_init(void);
uint8 control_scheduler_start(void);
void control_scheduler_update_10ms(void);
void control_scheduler_process_foreground(void);

void control_scheduler_request_arm(void);
void control_scheduler_request_disarm(void);
void control_scheduler_request_line_start(void);
void control_scheduler_request_line_stop(void);
void control_scheduler_request_fault_clear(void);
uint8 control_scheduler_request_manual_target(
    float left_mm_s,
    float right_mm_s);

void control_scheduler_get_status(control_scheduler_status_struct *status);

#endif
