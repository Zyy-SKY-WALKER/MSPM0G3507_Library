/**
 * @file    control_scheduler.h
 * @brief   Unified bare-metal 10 ms vehicle control scheduler.
 */

#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include "chassis_motion.h"
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
    CONTROL_MODE_CHASSIS_MOTION,
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
    chassis_motion_status_struct chassis_motion;
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

/**
 * @brief Submit a high-level signed distance command.
 * @param distance_mm Positive for forward and negative for reverse distance.
 * @param max_speed_mm_s Positive maximum center speed.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_distance(
    float distance_mm,
    float max_speed_mm_s);

/**
 * @brief Submit a high-level timed signed-speed command.
 * @param speed_mm_s Positive for forward and negative for reverse speed.
 * @param duration_ms Command duration.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_timed(
    float speed_mm_s,
    uint32 duration_ms);

/**
 * @brief Submit a high-level relative heading-turn command.
 * @param angle_deg Positive for left and negative for right rotation.
 * @param max_angular_speed_deg_s Positive maximum angular speed.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_turn_relative(
    float angle_deg,
    float max_angular_speed_deg_s);

/**
 * @brief Submit a request to smoothly cancel the active chassis command.
 */
void control_scheduler_request_chassis_motion_cancel(void);

/**
 * @brief Submit a request to select one configured chassis PID parameter group.
 * @param profile_id Profile identifier from 0 to 3.
 * @return ZF_TRUE when the profile identifier is valid.
 */
uint8 control_scheduler_request_chassis_motion_pid_profile(uint8 profile_id);

void control_scheduler_get_status(control_scheduler_status_struct *status);

#endif
