/**
 * @file    my_lib_mpu6500.c
 * @brief   MPU6500 software-IIC driver.
 */

#include "my_lib_mpu6500.h"

#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"

#define MPU6500_SCL_PIN                  (A1)
#define MPU6500_SDA_PIN                  (A0)

#define MPU6500_REG_SMPLRT_DIV           (0x19U)
#define MPU6500_REG_CONFIG               (0x1AU)
#define MPU6500_REG_GYRO_CONFIG          (0x1BU)
#define MPU6500_REG_ACCEL_CONFIG         (0x1CU)
#define MPU6500_REG_ACCEL_CONFIG2        (0x1DU)
#define MPU6500_REG_ACCEL_XOUT_H         (0x3BU)
#define MPU6500_REG_PWR_MGMT_1           (0x6BU)
#define MPU6500_REG_PWR_MGMT_2           (0x6CU)
#define MPU6500_REG_WHO_AM_I             (0x75U)

#define MPU6500_WHO_AM_I_VALUE           (0x70U)
#define MPU6500_SAMPLE_RATE_DIVIDER      (0x09U)
#define MPU6500_GYRO_DLPF_41HZ           (0x03U)
#define MPU6500_GYRO_RANGE_2000DPS       (0x18U)
#define MPU6500_ACCEL_RANGE_8G           (0x10U)
#define MPU6500_ACCEL_DLPF_41HZ          (0x03U)
#define MPU6500_RESET_COMMAND             (0x80U)
#define MPU6500_CLOCK_PLL_XGYRO          (0x01U)
#define MPU6500_RAW_FRAME_SIZE            (14U)

static soft_iic_info_struct mpu6500_iic;
static uint8 mpu6500_initialized;
static uint32 mpu6500_error_count;

/**
 * @brief Send one byte and check the slave ACK result.
 * @return 1 when acknowledged, otherwise 0.
 */
static uint8 mpu6500_send_byte(uint8 data)
{
    return soft_iic_send_data(&mpu6500_iic, data);
}

/**
 * @brief Write one MPU6500 register.
 * @return 0 on success, otherwise 1.
 */
static uint8 mpu6500_write_register(uint8 reg, uint8 data)
{
    uint8 success = 0U;

    soft_iic_start(&mpu6500_iic);
    if((mpu6500_send_byte((uint8)(MPU6500_IIC_ADDRESS << 1U)) != 0U)
        && (mpu6500_send_byte(reg) != 0U)
        && (mpu6500_send_byte(data) != 0U))
    {
        success = 1U;
    }
    soft_iic_stop(&mpu6500_iic);

    if(success == 0U)
    {
        mpu6500_error_count++;
        return 1U;
    }

    return 0U;
}

/**
 * @brief Read a contiguous MPU6500 register range.
 * @return 0 on success, otherwise 1.
 */
static uint8 mpu6500_read_registers(uint8 reg, uint8 data[], uint8 length)
{
    uint8 index;
    uint8 success = 0U;

    if((data == NULL) || (length == 0U))
    {
        return 1U;
    }

    soft_iic_start(&mpu6500_iic);
    if((mpu6500_send_byte((uint8)(MPU6500_IIC_ADDRESS << 1U)) != 0U)
        && (mpu6500_send_byte(reg) != 0U))
    {
        soft_iic_start(&mpu6500_iic);
        if(mpu6500_send_byte((uint8)((MPU6500_IIC_ADDRESS << 1U) | 0x01U)) != 0U)
        {
            for(index = 0U; index < length; index++)
            {
                data[index] = soft_iic_read_data(
                    &mpu6500_iic,
                    (uint8)(index == (length - 1U)));
            }
            success = 1U;
        }
    }
    soft_iic_stop(&mpu6500_iic);

    if(success == 0U)
    {
        mpu6500_error_count++;
        return 1U;
    }

    return 0U;
}

/**
 * @brief Verify one critical configuration register.
 * @return 0 when the expected value was read, otherwise 1.
 */
static uint8 mpu6500_verify_register(uint8 reg, uint8 expected)
{
    uint8 value;

    if(mpu6500_read_registers(reg, &value, 1U) != 0U)
    {
        return 1U;
    }

    if(value != expected)
    {
        mpu6500_error_count++;
        return 1U;
    }

    return 0U;
}

/**
 * @brief Initialize the MPU6500 software-IIC interface and configuration.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_init(void)
{
    uint8 device_id = 0U;

    mpu6500_initialized = 0U;
    mpu6500_error_count = 0U;
    soft_iic_init(
        &mpu6500_iic,
        MPU6500_IIC_ADDRESS,
        MPU6500_SOFT_IIC_DELAY,
        MPU6500_SCL_PIN,
        MPU6500_SDA_PIN);
    system_delay_ms(100U);

    if(mpu6500_write_register(MPU6500_REG_PWR_MGMT_1, MPU6500_RESET_COMMAND) != 0U)
    {
        return 1U;
    }
    system_delay_ms(100U);

    if((mpu6500_read_registers(MPU6500_REG_WHO_AM_I, &device_id, 1U) != 0U)
        || (device_id != MPU6500_WHO_AM_I_VALUE)
        || (mpu6500_write_register(MPU6500_REG_PWR_MGMT_1, MPU6500_CLOCK_PLL_XGYRO) != 0U)
        || (mpu6500_write_register(MPU6500_REG_PWR_MGMT_2, 0x00U) != 0U))
    {
        if(device_id != MPU6500_WHO_AM_I_VALUE)
        {
            mpu6500_error_count++;
        }
        return 1U;
    }
    system_delay_ms(10U);

    if((mpu6500_write_register(MPU6500_REG_SMPLRT_DIV, MPU6500_SAMPLE_RATE_DIVIDER) != 0U)
        || (mpu6500_write_register(MPU6500_REG_CONFIG, MPU6500_GYRO_DLPF_41HZ) != 0U)
        || (mpu6500_write_register(MPU6500_REG_GYRO_CONFIG, MPU6500_GYRO_RANGE_2000DPS) != 0U)
        || (mpu6500_write_register(MPU6500_REG_ACCEL_CONFIG, MPU6500_ACCEL_RANGE_8G) != 0U)
        || (mpu6500_write_register(MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_41HZ) != 0U)
        || (mpu6500_verify_register(MPU6500_REG_SMPLRT_DIV, MPU6500_SAMPLE_RATE_DIVIDER) != 0U)
        || (mpu6500_verify_register(MPU6500_REG_CONFIG, MPU6500_GYRO_DLPF_41HZ) != 0U)
        || (mpu6500_verify_register(MPU6500_REG_GYRO_CONFIG, MPU6500_GYRO_RANGE_2000DPS) != 0U)
        || (mpu6500_verify_register(MPU6500_REG_ACCEL_CONFIG, MPU6500_ACCEL_RANGE_8G) != 0U)
        || (mpu6500_verify_register(MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_41HZ) != 0U))
    {
        return 1U;
    }

    mpu6500_initialized = 1U;
    return 0U;
}

/**
 * @brief Read the MPU6500 WHO_AM_I register.
 * @param device_id Destination for the device ID.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_read_who_am_i(uint8 *device_id)
{
    if((mpu6500_initialized == 0U) || (device_id == NULL))
    {
        return 1U;
    }

    return mpu6500_read_registers(MPU6500_REG_WHO_AM_I, device_id, 1U);
}

/**
 * @brief Read one coherent raw sensor frame.
 * @param data Destination for raw sensor values.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_read_raw(mpu6500_raw_data_struct *data)
{
    uint8 buffer[MPU6500_RAW_FRAME_SIZE];

    if((mpu6500_initialized == 0U) || (data == NULL)
        || (mpu6500_read_registers(
            MPU6500_REG_ACCEL_XOUT_H,
            buffer,
            MPU6500_RAW_FRAME_SIZE) != 0U))
    {
        return 1U;
    }

    data->accel_x = (int16)(((uint16)buffer[0] << 8U) | buffer[1]);
    data->accel_y = (int16)(((uint16)buffer[2] << 8U) | buffer[3]);
    data->accel_z = (int16)(((uint16)buffer[4] << 8U) | buffer[5]);
    data->temperature = (int16)(((uint16)buffer[6] << 8U) | buffer[7]);
    data->gyro_x = (int16)(((uint16)buffer[8] << 8U) | buffer[9]);
    data->gyro_y = (int16)(((uint16)buffer[10] << 8U) | buffer[11]);
    data->gyro_z = (int16)(((uint16)buffer[12] << 8U) | buffer[13]);

    return 0U;
}

/**
 * @brief Read one sensor frame converted to physical units.
 * @param data Destination for converted sensor values.
 * @return 0 on success, otherwise 1.
 */
uint8 mpu6500_read(mpu6500_data_struct *data)
{
    mpu6500_raw_data_struct raw_data;

    if((data == NULL) || (mpu6500_read_raw(&raw_data) != 0U))
    {
        return 1U;
    }

    data->accel_g[0] = (float)raw_data.accel_x / 4096.0F;
    data->accel_g[1] = (float)raw_data.accel_y / 4096.0F;
    data->accel_g[2] = (float)raw_data.accel_z / 4096.0F;
    data->gyro_deg_s[0] = (float)raw_data.gyro_x / 16.4F;
    data->gyro_deg_s[1] = (float)raw_data.gyro_y / 16.4F;
    data->gyro_deg_s[2] = (float)raw_data.gyro_z / 16.4F;
    data->temperature_c = ((float)raw_data.temperature / 333.87F) + 21.0F;

    return 0U;
}

/**
 * @brief Return the cumulative failed IIC-transaction count.
 * @return Number of failed transactions since initialization.
 */
uint32 mpu6500_get_error_count(void)
{
    return mpu6500_error_count;
}
