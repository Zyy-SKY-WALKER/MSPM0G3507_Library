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

typedef struct
{
    float accel_g[3];
    float gyro_dps[3];
    float angle_deg[3];
    float temperature_c;
    uint8 valid_flags;
    uint32 frame_count;
    uint32 angle_frame_count;
    uint32 checksum_error_count;
} imu_uart_data_struct;

void imu_uart_init(void);
void imu_uart_reset_yaw(void);
uint8 imu_uart_get_data(imu_uart_data_struct *data);
uint8 imu_uart_get_yaw(float *yaw_deg, uint32 *angle_frame_count);

#endif
