/**
 * @file    odometry.h
 * @brief   Differential-drive encoder and IMU fusion odometry.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "zf_common_typedef.h"

/** @brief Latest differential-drive pose, travel and input sample state. */
typedef struct
{
    /** Global X coordinate in millimeters. */
    float x_mm;
    /** Global Y coordinate in millimeters. */
    float y_mm;
    /** Continuous, unwrapped global heading in radians. */
    float theta_rad;
    /** Signed accumulated left wheel travel in millimeters. */
    float left_distance_mm;
    /** Signed accumulated right wheel travel in millimeters. */
    float right_distance_mm;
    /** Signed accumulated center displacement in millimeters. */
    float center_displacement_mm;
    /** Accumulated absolute center travel in millimeters. */
    float path_length_mm;
    /** Latest supplied wrapped IMU yaw in degrees. */
    float imu_yaw_deg;
    /** Latest signed left encoder interval count. */
    int16 left_count;
    /** Latest signed right encoder interval count. */
    int16 right_count;
    /** Number of integrated encoder samples since reset. */
    uint32 sample_count;
    /** Latest supplied IMU angle-frame sequence number. */
    uint32 imu_angle_frame_count;
    /** Nonzero when the latest supplied yaw is marked valid. */
    uint8 imu_valid;
} odometry_state_struct;

/**
 * @brief Initialize odometry with a zero pose and cleared accumulators.
 */
void odometry_init(void);

/**
 * @brief Integrate one encoder interval and an optional new IMU yaw frame.
 * @param left_count Signed left encoder count for this sample interval.
 * @param right_count Signed right encoder count for this sample interval.
 * @param yaw_valid Nonzero when yaw_deg and yaw_frame_count are valid.
 * @param yaw_deg Latest wrapped IMU yaw angle in degrees.
 * @param yaw_frame_count Monotonic valid angle-frame sequence number.
 * @note Supply each frame count only once; repeated counts are not fused again.
 */
void odometry_update(
    int16 left_count,
    int16 right_count,
    uint8 yaw_valid,
    float yaw_deg,
    uint32 yaw_frame_count);

/**
 * @brief Reset the pose, travel accumulators and IMU reference to zero.
 */
void odometry_reset(void);

/**
 * @brief Establish a new pose and clear travel and IMU history.
 * @param x_mm New global X coordinate in millimeters.
 * @param y_mm New global Y coordinate in millimeters.
 * @param theta_rad New continuous global heading in radians.
 */
void odometry_reset_pose(float x_mm, float y_mm, float theta_rad);

/**
 * @brief Copy one interrupt-coherent odometry state snapshot.
 * @param state Destination state; NULL causes no operation.
 */
void odometry_get_state(odometry_state_struct *state);

#endif
