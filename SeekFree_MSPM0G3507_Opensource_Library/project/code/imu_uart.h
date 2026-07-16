/**
 * @file    imu_uart.h
 * @brief   UART attitude module driver for the 0x55 frame protocol.
 */

#ifndef IMU_UART_H
#define IMU_UART_H

#include "zf_common_typedef.h"

#define IMU_UART_BAUD_RATE              (115200U)
#define IMU_UART_FRAME_LENGTH           (11U)

#define IMU_UART_VALID_ACCEL            (0x01U)
#define IMU_UART_VALID_GYRO             (0x02U)
#define IMU_UART_VALID_ANGLE            (0x04U)

/** @brief Latest converted values and receive diagnostics from the IMU. */
typedef struct
{
    /** X, Y and Z acceleration in g, nominally -16 through 16. */
    float accel_g[3];
    /** X, Y and Z angular velocity in degrees per second. */
    float gyro_dps[3];
    /** X, Y and Z Euler angles in degrees, nominally -180 through 180. */
    float angle_deg[3];
    /** Temperature reported by the most recent supported frame, in Celsius. */
    float temperature_c;
    /** Bitwise OR of IMU_UART_VALID_* flags received at least once. */
    uint8 valid_flags;
    /** Number of valid supported frames received since initialization. */
    uint32 frame_count;
    /** Number of valid angle frames received since initialization. */
    uint32 angle_frame_count;
    /** Number of checksum failures detected since initialization. */
    uint32 checksum_error_count;
} imu_uart_data_struct;

/**
 * @brief Initialize UART2 reception and clear all IMU parser state.
 * @note Enables the receive interrupt and sends the yaw-zero command.
 */
void imu_uart_init(void);

/**
 * @brief Send the module command that initializes its Z-axis angle to zero.
 * @note imu_uart_init() must configure UART2 before this call.
 */
void imu_uart_angle_init(void);

/**
 * @brief Reset the module yaw reference to zero.
 * @note Sends the same UART command as imu_uart_angle_init().
 */
void imu_uart_reset_yaw(void);

/**
 * @brief Copy and convert the latest validated IMU frames atomically.
 * @param data Destination data structure; must not be NULL.
 * @return ZF_TRUE after any supported frame has been received.
 */
uint8 imu_uart_get_data(imu_uart_data_struct *data);

/**
 * @brief Copy the latest yaw angle and angle-frame sequence number.
 * @param yaw_deg Destination yaw in degrees, nominally -180 through 180.
 * @param angle_frame_count Destination valid angle-frame count.
 * @return ZF_TRUE after at least one valid angle frame has been received.
 */
uint8 imu_uart_get_yaw(float *yaw_deg, uint32 *angle_frame_count);

#endif
