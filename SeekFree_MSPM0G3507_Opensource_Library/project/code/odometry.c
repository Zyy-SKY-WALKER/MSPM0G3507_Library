/**
 * @file    odometry.c
 * @brief   Differential-drive encoder and IMU fusion odometry.
 */

#include "odometry.h"

#include <math.h>

#include "drive_geometry.h"
#include "zf_common_interrupt.h"

#define ODOMETRY_DEG_TO_RAD             (DRIVE_PI / 180.0F)
#define ODOMETRY_TWO_PI                 (2.0F * DRIVE_PI)

static volatile odometry_state_struct odometry_state;
/** @brief Wrapped-yaw history used to maintain a continuous IMU heading. */
static volatile float odometry_last_imu_yaw_rad;
static volatile float odometry_continuous_imu_theta_rad;
static volatile uint32 odometry_last_imu_frame_count;
static volatile uint8 odometry_imu_reference_valid;

/**
 * @brief Wrap an angle difference to the interval [-pi, pi].
 * @param angle_rad Angle in radians.
 * @return Wrapped angle in radians.
 */
static float odometry_wrap_angle(float angle_rad)
{
    while (angle_rad > DRIVE_PI)
    {
        angle_rad -= ODOMETRY_TWO_PI;
    }

    while (angle_rad < -DRIVE_PI)
    {
        angle_rad += ODOMETRY_TWO_PI;
    }

    return angle_rad;
}

/**
 * @brief Update the continuous IMU heading when a new yaw frame arrives.
 * @param yaw_valid Nonzero when a yaw frame has been received.
 * @param yaw_deg Latest wrapped yaw angle in degrees.
 * @param yaw_frame_count Angle-frame sequence number.
 * @param predicted_theta Encoder-predicted continuous heading.
 * @param imu_theta Destination continuous IMU heading.
 * @return Nonzero when a new IMU correction is available.
 */
static uint8 odometry_update_imu_heading(
    uint8 yaw_valid,
    float yaw_deg,
    uint32 yaw_frame_count,
    float predicted_theta,
    float *imu_theta)
{
    float yaw_rad;
    float yaw_delta;

    if ((yaw_valid == 0U) || (imu_theta == NULL))
    {
        return ZF_FALSE;
    }

    if ((odometry_imu_reference_valid != 0U)
        && (yaw_frame_count == odometry_last_imu_frame_count))
    {
        return ZF_FALSE;
    }

    yaw_rad = yaw_deg * ODOMETRY_DEG_TO_RAD;
    if (odometry_imu_reference_valid == 0U)
    {
        /* Anchor the first wrapped yaw to the encoder-predicted heading. */
        odometry_last_imu_yaw_rad = yaw_rad;
        odometry_continuous_imu_theta_rad = predicted_theta;
        odometry_last_imu_frame_count = yaw_frame_count;
        odometry_imu_reference_valid = 1U;
        return ZF_FALSE;
    }

    /* Accumulate the shortest signed delta to unwrap successive yaw frames. */
    yaw_delta = odometry_wrap_angle(
        yaw_rad - odometry_last_imu_yaw_rad);
    odometry_continuous_imu_theta_rad += yaw_delta;
    odometry_last_imu_yaw_rad = yaw_rad;
    odometry_last_imu_frame_count = yaw_frame_count;
    *imu_theta = odometry_continuous_imu_theta_rad;

    return ZF_TRUE;
}

/**
 * @brief Initialize and clear the odometry state.
 */
void odometry_init(void)
{
    odometry_reset();
}

/**
 * @brief Integrate one shared encoder sample and optional new IMU yaw frame.
 * @param left_count Signed left encoder interval count.
 * @param right_count Signed right encoder interval count.
 * @param yaw_valid Nonzero after at least one valid angle frame.
 * @param yaw_deg Latest yaw angle in degrees.
 * @param yaw_frame_count Angle-frame sequence number.
 */
void odometry_update(
    int16 left_count,
    int16 right_count,
    uint8 yaw_valid,
    float yaw_deg,
    uint32 yaw_frame_count)
{
    float left_delta_mm =
        (float)left_count * DRIVE_LEFT_MM_PER_COUNT;
    float right_delta_mm =
        (float)right_count * DRIVE_RIGHT_MM_PER_COUNT;
    float center_delta_mm =
        (left_delta_mm + right_delta_mm) * 0.5F;
    float encoder_delta_theta =
        (right_delta_mm - left_delta_mm) / DRIVE_TRACK_WIDTH_MM;
    float theta_before = odometry_state.theta_rad;
    float theta_predict = theta_before + encoder_delta_theta;
    float theta_fused = theta_predict;
    float imu_theta;
    float theta_mid;

    /* Correct the encoder prediction only when a fresh IMU frame arrives. */
    if (odometry_update_imu_heading(
            yaw_valid,
            yaw_deg,
            yaw_frame_count,
            theta_predict,
            &imu_theta) != 0U)
    {
        float heading_error = odometry_wrap_angle(
            imu_theta - theta_predict);

        theta_fused = theta_predict
            + (DRIVE_ODOMETRY_IMU_WEIGHT * heading_error);
    }

    /* Integrate translation at the midpoint of the fused heading change. */
    theta_mid = theta_before + ((theta_fused - theta_before) * 0.5F);
    odometry_state.x_mm += center_delta_mm * cosf(theta_mid);
    odometry_state.y_mm += center_delta_mm * sinf(theta_mid);
    odometry_state.theta_rad = theta_fused;
    odometry_state.left_distance_mm += left_delta_mm;
    odometry_state.right_distance_mm += right_delta_mm;
    odometry_state.center_displacement_mm += center_delta_mm;
    odometry_state.path_length_mm += fabsf(center_delta_mm);
    odometry_state.imu_yaw_deg = yaw_deg;
    odometry_state.left_count = left_count;
    odometry_state.right_count = right_count;
    odometry_state.sample_count++;
    odometry_state.imu_angle_frame_count = yaw_frame_count;
    odometry_state.imu_valid = yaw_valid != 0U ? 1U : 0U;
}

/**
 * @brief Reset pose and all accumulated distances to zero.
 */
void odometry_reset(void)
{
    odometry_reset_pose(0.0F, 0.0F, 0.0F);
}

/**
 * @brief Reset accumulated data and establish a new global pose.
 * @param x_mm New X coordinate in millimeters.
 * @param y_mm New Y coordinate in millimeters.
 * @param theta_rad New continuous heading in radians.
 */
void odometry_reset_pose(float x_mm, float y_mm, float theta_rad)
{
    uint32 primask = interrupt_global_disable();

    odometry_state.x_mm = x_mm;
    odometry_state.y_mm = y_mm;
    odometry_state.theta_rad = theta_rad;
    odometry_state.left_distance_mm = 0.0F;
    odometry_state.right_distance_mm = 0.0F;
    odometry_state.center_displacement_mm = 0.0F;
    odometry_state.path_length_mm = 0.0F;
    odometry_state.imu_yaw_deg = 0.0F;
    odometry_state.left_count = 0;
    odometry_state.right_count = 0;
    odometry_state.sample_count = 0U;
    odometry_state.imu_angle_frame_count = 0U;
    odometry_state.imu_valid = 0U;

    odometry_last_imu_yaw_rad = 0.0F;
    odometry_continuous_imu_theta_rad = theta_rad;
    odometry_last_imu_frame_count = 0U;
    odometry_imu_reference_valid = 0U;

    interrupt_global_enable(primask);
}

/**
 * @brief Copy one coherent odometry state snapshot.
 * @param state Destination state structure.
 */
void odometry_get_state(odometry_state_struct *state)
{
    uint32 primask;

    if (state == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *state = odometry_state;
    interrupt_global_enable(primask);
}
