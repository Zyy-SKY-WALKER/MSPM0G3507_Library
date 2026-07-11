/**
 * @file    odometry.h
 * @brief   Differential-drive encoder and IMU fusion odometry.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "zf_common_typedef.h"

typedef struct
{
    float x_mm;
    float y_mm;
    float theta_rad;
    float left_distance_mm;
    float right_distance_mm;
    float center_displacement_mm;
    float path_length_mm;
    float imu_yaw_deg;
    int16 left_count;
    int16 right_count;
    uint32 sample_count;
    uint32 imu_angle_frame_count;
    uint8 imu_valid;
} odometry_state_struct;

void odometry_init(void);
void odometry_update(
    int16 left_count,
    int16 right_count,
    uint8 yaw_valid,
    float yaw_deg,
    uint32 yaw_frame_count);
void odometry_reset(void);
void odometry_reset_pose(float x_mm, float y_mm, float theta_rad);
void odometry_get_state(odometry_state_struct *state);

#endif
