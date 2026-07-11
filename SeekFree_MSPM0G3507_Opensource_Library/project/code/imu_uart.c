/**
 * @file    imu_uart.c
 * @brief   UART attitude module driver for the 0x55 frame protocol.
 */

#include "imu_uart.h"

#include "zf_common_interrupt.h"
#include "zf_driver_uart.h"

#define IMU_UART_INDEX                  (UART_3)
#define IMU_UART_TX_PIN                 (UART3_TX_B12)
#define IMU_UART_RX_PIN                 (UART3_RX_B13)

#define IMU_UART_FRAME_HEADER           (0x55U)
#define IMU_UART_FRAME_ACCEL            (0x51U)
#define IMU_UART_FRAME_GYRO             (0x52U)
#define IMU_UART_FRAME_ANGLE            (0x53U)

#define IMU_UART_ACCEL_SCALE            (16.0F / 32768.0F)
#define IMU_UART_GYRO_SCALE             (2000.0F / 32768.0F)
#define IMU_UART_ANGLE_SCALE            (180.0F / 32768.0F)
#define IMU_UART_TEMPERATURE_SCALE      (1.0F / 100.0F)

typedef struct
{
    int16 accel[3];
    int16 gyro[3];
    int16 angle[3];
    int16 temperature;
    uint8 valid_flags;
    uint32 frame_count;
    uint32 angle_frame_count;
    uint32 checksum_error_count;
} imu_uart_raw_data_struct;

static volatile imu_uart_raw_data_struct imu_uart_raw_data;
static uint8 imu_uart_frame[IMU_UART_FRAME_LENGTH];
static uint8 imu_uart_frame_index;

/**
 * @brief Decode one little-endian signed 16-bit field.
 * @param data Address of the low byte.
 * @return Signed field value.
 */
static int16 imu_uart_decode_int16(const uint8 data[])
{
    uint16 value = (uint16)data[0]
        | ((uint16)data[1] << 8);

    return (int16)value;
}

/**
 * @brief Validate one complete 11-byte frame checksum.
 * @param frame Complete frame.
 * @return ZF_TRUE when the checksum is valid.
 */
static uint8 imu_uart_checksum_is_valid(const uint8 frame[])
{
    uint8 checksum = 0U;
    uint8 index;

    for (index = 0U; index < (IMU_UART_FRAME_LENGTH - 1U); index++)
    {
        checksum = (uint8)(checksum + frame[index]);
    }

    return (uint8)(checksum == frame[IMU_UART_FRAME_LENGTH - 1U]);
}

/**
 * @brief Save one validated raw sensor frame.
 * @param frame Complete validated frame.
 */
static void imu_uart_save_frame(const uint8 frame[])
{
    int16 x = imu_uart_decode_int16(&frame[2]);
    int16 y = imu_uart_decode_int16(&frame[4]);
    int16 z = imu_uart_decode_int16(&frame[6]);
    int16 temperature = imu_uart_decode_int16(&frame[8]);

    switch (frame[1])
    {
        case IMU_UART_FRAME_ACCEL:
        {
            imu_uart_raw_data.accel[0] = x;
            imu_uart_raw_data.accel[1] = y;
            imu_uart_raw_data.accel[2] = z;
            imu_uart_raw_data.temperature = temperature;
            imu_uart_raw_data.valid_flags |= IMU_UART_VALID_ACCEL;
            break;
        }

        case IMU_UART_FRAME_GYRO:
        {
            imu_uart_raw_data.gyro[0] = x;
            imu_uart_raw_data.gyro[1] = y;
            imu_uart_raw_data.gyro[2] = z;
            imu_uart_raw_data.temperature = temperature;
            imu_uart_raw_data.valid_flags |= IMU_UART_VALID_GYRO;
            break;
        }

        case IMU_UART_FRAME_ANGLE:
        {
            imu_uart_raw_data.angle[0] = x;
            imu_uart_raw_data.angle[1] = y;
            imu_uart_raw_data.angle[2] = z;
            imu_uart_raw_data.temperature = temperature;
            imu_uart_raw_data.valid_flags |= IMU_UART_VALID_ANGLE;
            imu_uart_raw_data.angle_frame_count++;
            break;
        }

        default:
        {
            return;
        }
    }

    imu_uart_raw_data.frame_count++;
}

/**
 * @brief Retain a possible new frame header after a checksum failure.
 */
static void imu_uart_resynchronize(void)
{
    uint8 header_index;
    uint8 remaining_length;
    uint8 index;

    for (header_index = 1U;
        header_index < IMU_UART_FRAME_LENGTH;
        header_index++)
    {
        if (imu_uart_frame[header_index] == IMU_UART_FRAME_HEADER)
        {
            break;
        }
    }

    if (header_index >= IMU_UART_FRAME_LENGTH)
    {
        imu_uart_frame_index = 0U;
        return;
    }

    remaining_length = (uint8)(IMU_UART_FRAME_LENGTH - header_index);
    for (index = 0U; index < remaining_length; index++)
    {
        imu_uart_frame[index] = imu_uart_frame[header_index + index];
    }
    imu_uart_frame_index = remaining_length;
}

/**
 * @brief Consume one received byte and advance the frame parser.
 * @param byte Received byte.
 */
static void imu_uart_process_byte(uint8 byte)
{
    if ((imu_uart_frame_index == 0U)
        && (byte != IMU_UART_FRAME_HEADER))
    {
        return;
    }

    imu_uart_frame[imu_uart_frame_index] = byte;
    imu_uart_frame_index++;

    if (imu_uart_frame_index < IMU_UART_FRAME_LENGTH)
    {
        return;
    }

    if (imu_uart_checksum_is_valid(imu_uart_frame) != 0U)
    {
        imu_uart_frame_index = 0U;
        imu_uart_save_frame(imu_uart_frame);
    }
    else
    {
        imu_uart_raw_data.checksum_error_count++;
        imu_uart_resynchronize();
    }
}

/**
 * @brief Drain received bytes from the UART3 interrupt callback.
 * @param state UART interrupt state.
 * @param user_data Optional callback context.
 */
static void imu_uart_callback(uint32 state, void *user_data)
{
    uint8 byte;

    (void)user_data;

    if (state != UART_INTERRUPT_STATE_RX)
    {
        return;
    }

    while (uart_query_byte(IMU_UART_INDEX, &byte) != 0U)
    {
        imu_uart_process_byte(byte);
    }
}

/**
 * @brief Initialize the UART3 attitude-module interface.
 */
void imu_uart_init(void)
{
    uint32 primask = interrupt_global_disable();
    uint8 index;

    imu_uart_frame_index = 0U;
    for (index = 0U; index < IMU_UART_FRAME_LENGTH; index++)
    {
        imu_uart_frame[index] = 0U;
    }
    imu_uart_raw_data.accel[0] = 0;
    imu_uart_raw_data.accel[1] = 0;
    imu_uart_raw_data.accel[2] = 0;
    imu_uart_raw_data.gyro[0] = 0;
    imu_uart_raw_data.gyro[1] = 0;
    imu_uart_raw_data.gyro[2] = 0;
    imu_uart_raw_data.angle[0] = 0;
    imu_uart_raw_data.angle[1] = 0;
    imu_uart_raw_data.angle[2] = 0;
    imu_uart_raw_data.temperature = 0;
    imu_uart_raw_data.valid_flags = 0U;
    imu_uart_raw_data.frame_count = 0U;
    imu_uart_raw_data.angle_frame_count = 0U;
    imu_uart_raw_data.checksum_error_count = 0U;

    interrupt_global_enable(primask);

    uart_init(
        IMU_UART_INDEX,
        IMU_UART_BAUD_RATE,
        IMU_UART_TX_PIN,
        IMU_UART_RX_PIN);
    uart_set_callback(IMU_UART_INDEX, imu_uart_callback, NULL);
    uart_set_interrupt_config(
        IMU_UART_INDEX,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
    imu_uart_reset_yaw();
}

/**
 * @brief Send the module command that resets the yaw reference.
 */
void imu_uart_reset_yaw(void)
{
    static const uint8 command[] = {0xFFU, 0xAAU, 0x52U};

    uart_write_buffer(
        IMU_UART_INDEX,
        command,
        sizeof(command));
}

/**
 * @brief Copy and convert the latest validated sensor data.
 * @param data Destination converted data structure.
 * @return ZF_TRUE after at least one supported frame is received.
 */
uint8 imu_uart_get_data(imu_uart_data_struct *data)
{
    imu_uart_raw_data_struct raw_data;
    uint32 primask;
    uint8 index;

    if (data == NULL)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    raw_data = imu_uart_raw_data;
    interrupt_global_enable(primask);

    for (index = 0U; index < 3U; index++)
    {
        data->accel_g[index] =
            (float)raw_data.accel[index] * IMU_UART_ACCEL_SCALE;
        data->gyro_dps[index] =
            (float)raw_data.gyro[index] * IMU_UART_GYRO_SCALE;
        data->angle_deg[index] =
            (float)raw_data.angle[index] * IMU_UART_ANGLE_SCALE;
    }
    data->temperature_c =
        (float)raw_data.temperature * IMU_UART_TEMPERATURE_SCALE;
    data->valid_flags = raw_data.valid_flags;
    data->frame_count = raw_data.frame_count;
    data->angle_frame_count = raw_data.angle_frame_count;
    data->checksum_error_count = raw_data.checksum_error_count;

    return raw_data.valid_flags != 0U ? ZF_TRUE : ZF_FALSE;
}

/**
 * @brief Get the latest validated yaw angle.
 * @param yaw_deg Destination yaw angle in degrees.
 * @param angle_frame_count Destination angle-frame sequence number.
 * @return ZF_TRUE when an angle frame has been received.
 */
uint8 imu_uart_get_yaw(float *yaw_deg, uint32 *angle_frame_count)
{
    uint32 primask;
    uint32 frame_count;
    int16 yaw_raw;
    uint8 valid_flags;

    if ((yaw_deg == NULL) || (angle_frame_count == NULL))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    yaw_raw = imu_uart_raw_data.angle[2];
    frame_count = imu_uart_raw_data.angle_frame_count;
    valid_flags = imu_uart_raw_data.valid_flags;
    interrupt_global_enable(primask);

    if ((valid_flags & IMU_UART_VALID_ANGLE) == 0U)
    {
        return ZF_FALSE;
    }

    *yaw_deg = (float)yaw_raw * IMU_UART_ANGLE_SCALE;
    *angle_frame_count = frame_count;
    return ZF_TRUE;
}
