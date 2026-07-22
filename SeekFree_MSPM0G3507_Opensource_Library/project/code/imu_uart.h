/**
 * @file    imu_uart.h
 * @brief   DMA UART angle-frame driver for the 0x55 frame protocol.
 */

#ifndef IMU_UART_H
#define IMU_UART_H

#include "zf_common_typedef.h"

#define IMU_UART_BAUD_RATE              (115200U)
#define IMU_UART_FRAME_LENGTH           (11U)

/** @brief Latest converted angle values and receive diagnostics. */
typedef struct
{
    /** X, Y and Z Euler angles in degrees, nominally -180 through 180. */
    float angle_deg[3];
    /** X, Y and Z angular rates in degrees per second. */
    float gyro_deg_s[3];
    /** Number of valid angle frames received since initialization. */
    uint32 angle_frame_count;
    /** Number of valid angular-rate frames received since initialization. */
    uint32 gyro_frame_count;
    /** Number of candidate angle frames rejected by checksum. */
    uint32 checksum_error_count;
    /** Nonzero after at least one valid angle frame is received. */
    uint8 angle_valid;
    /** Nonzero after at least one valid angular-rate frame is received. */
    uint8 gyro_valid;
} imu_uart_data_struct;

/**
 * @brief Initialize UART2 and its circular receive DMA channel.
 * @note Sends the yaw-zero command after DMA reception starts.
 */
void imu_uart_init(void);

/**
 * @brief Consume new DMA bytes and decode valid 0x55 0x53 angle frames.
 * @note Call from exactly one periodic context at least every 100 ms.
 */
void imu_uart_update(void);

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
 * @brief Copy and convert the latest validated angle data atomically.
 * @param data Destination data structure; must not be NULL.
 * @return ZF_TRUE after at least one valid angle frame has been received.
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
