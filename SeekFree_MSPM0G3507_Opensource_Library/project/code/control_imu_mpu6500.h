/**
 * @file    control_imu_mpu6500.h
 * @brief   Foreground MPU6500 source for the vehicle control scheduler.
 */

#ifndef CONTROL_IMU_MPU6500_H
#define CONTROL_IMU_MPU6500_H

#include "zf_common_typedef.h"

/** @brief Latest foreground-produced MPU6500 attitude snapshot. */
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_continuous_deg;
    float yaw_bias_deg_s;
    float temperature_c;
    uint32 update_count;
    uint16 calibration_sample_count;
    uint8 calibration_progress;
    uint8 valid;
    uint8 ready;
} control_imu_mpu6500_data_struct;

/**
 * @brief Initialize the MPU6500 and clear the foreground source state.
 * @return ZF_TRUE on success, otherwise ZF_FALSE.
 */
uint8 control_imu_mpu6500_init(void);

/**
 * @brief Read and solve at most once for the supplied scheduler tick.
 * @param scheduler_tick Latest 10 ms scheduler tick.
 * @note Call frequently from foreground, never from the control ISR.
 */
void control_imu_mpu6500_service(uint32 scheduler_tick);

/**
 * @brief Copy the latest coherent MPU6500 attitude snapshot.
 * @param data Destination snapshot.
 * @return ZF_TRUE after at least one successful sample, otherwise ZF_FALSE.
 */
uint8 control_imu_mpu6500_get_data(control_imu_mpu6500_data_struct *data);

#endif
