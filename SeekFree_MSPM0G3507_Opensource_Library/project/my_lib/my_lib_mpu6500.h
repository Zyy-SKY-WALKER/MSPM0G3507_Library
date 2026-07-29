/**
 * @file    my_lib_mpu6500.h
 * @brief   MPU6500 software-IIC driver.
 */

#ifndef MY_LIB_MPU6500_H
#define MY_LIB_MPU6500_H

#include "zf_common_typedef.h"

/** @brief MPU6500 seven-bit IIC address when AD0 is low. */
#define MPU6500_IIC_ADDRESS              (0x68U)
/** @brief Software-IIC delay matching the existing IMU driver setting. */
#define MPU6500_SOFT_IIC_DELAY           (50U)

/** @brief Raw MPU6500 sample from one contiguous register burst. */
typedef struct
{
    int16 accel_x;
    int16 accel_y;
    int16 accel_z;
    int16 temperature;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
} mpu6500_raw_data_struct;

/** @brief MPU6500 sample converted to engineering units. */
typedef struct
{
    float accel_g[3];
    float gyro_deg_s[3];
    float temperature_c;
} mpu6500_data_struct;

/**
 * @brief Initialize MPU6500 on A1 SCL and A0 SDA.
 * @return 0 on success, otherwise 1.
 * @note CS/NCS must be held high and AD0 low in IIC mode.
 */
uint8 mpu6500_init(void);

/**
 * @brief Read the MPU6500 WHO_AM_I register.
 * @param device_id Destination for the device ID.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_read_who_am_i(uint8 *device_id);

/**
 * @brief Read one coherent accelerometer, temperature, and gyroscope frame.
 * @param data Destination for raw sensor values.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_read_raw(mpu6500_raw_data_struct *data);

/**
 * @brief Read one coherent frame and convert it to physical units.
 * @param data Destination for converted sensor values.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_read(mpu6500_data_struct *data);

/**
 * @brief Return the cumulative failed IIC-transaction count.
 * @return Number of failed transactions since initialization.
 */
uint32 mpu6500_get_error_count(void);

#endif
