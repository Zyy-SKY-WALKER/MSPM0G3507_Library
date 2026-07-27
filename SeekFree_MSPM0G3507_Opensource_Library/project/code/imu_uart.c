/**
 * @file    imu_uart.c
 * @brief   DMA UART angle-frame driver for the 0x55 frame protocol.
 */

#include "imu_uart.h"

#include <ti/driverlib/dl_dma.h>

#include "zf_common_interrupt.h"
#include "zf_driver_uart.h"

#define IMU_UART_INDEX                  (UART_2)
#define IMU_UART_TX_PIN                 (UART2_TX_B15)
#define IMU_UART_RX_PIN                 (UART2_RX_B16)

#define IMU_UART_DMA_CHANNEL            (0U)
#define IMU_UART_DMA_BUFFER_SIZE        (4096U)
#define IMU_UART_DMA_BUFFER_MASK        (IMU_UART_DMA_BUFFER_SIZE - 1U)

#define IMU_UART_FRAME_HEADER           (0x55U)
#define IMU_UART_FRAME_ANGLE            (0x53U)

#define IMU_UART_ANGLE_SCALE            (180.0F / 32768.0F)

typedef struct
{
    int16 angle[3];
    uint32 angle_frame_count;
    uint32 checksum_error_count;
    uint8 angle_valid;
} imu_uart_raw_data_struct;

static volatile imu_uart_raw_data_struct imu_uart_raw_data;
static volatile uint8 imu_uart_dma_buffer[IMU_UART_DMA_BUFFER_SIZE];
static uint8 imu_uart_frame[IMU_UART_FRAME_LENGTH];
static uint8 imu_uart_frame_index;
static uint16 imu_uart_dma_consumer_index;

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
 * @brief Save one validated raw angle frame.
 * @param frame Complete validated 0x53 frame.
 */
static void imu_uart_save_angle_frame(const uint8 frame[])
{
    uint8 index;

    for (index = 0U; index < 3U; index++)
    {
        imu_uart_raw_data.angle[index] = imu_uart_decode_int16(
            &frame[2U + ((uint8)index * 2U)]);
    }
    imu_uart_raw_data.angle_frame_count++;
    imu_uart_raw_data.angle_valid = 1U;
}

/**
 * @brief Retain a possible angle-frame prefix after checksum failure.
 */
static void imu_uart_resynchronize(void)
{
    uint8 header_index;
    uint8 remaining_length;
    uint8 index;

    for (header_index = 1U;
        header_index < (IMU_UART_FRAME_LENGTH - 1U);
        header_index++)
    {
        if ((imu_uart_frame[header_index] == IMU_UART_FRAME_HEADER)
            && (imu_uart_frame[header_index + 1U]
                == IMU_UART_FRAME_ANGLE))
        {
            break;
        }
    }

    if (header_index < (IMU_UART_FRAME_LENGTH - 1U))
    {
        remaining_length = (uint8)(
            IMU_UART_FRAME_LENGTH - header_index);
        for (index = 0U; index < remaining_length; index++)
        {
            imu_uart_frame[index] =
                imu_uart_frame[header_index + index];
        }
        imu_uart_frame_index = remaining_length;
    }
    else if (imu_uart_frame[IMU_UART_FRAME_LENGTH - 1U]
        == IMU_UART_FRAME_HEADER)
    {
        imu_uart_frame[0] = IMU_UART_FRAME_HEADER;
        imu_uart_frame_index = 1U;
    }
    else
    {
        imu_uart_frame_index = 0U;
    }
}

/**
 * @brief Consume one received byte and advance the frame parser.
 * @param byte Received byte.
 */
static void imu_uart_process_byte(uint8 byte)
{
    if (imu_uart_frame_index == 0U)
    {
        if (byte == IMU_UART_FRAME_HEADER)
        {
            imu_uart_frame[0] = byte;
            imu_uart_frame_index = 1U;
        }
        return;
    }

    if (imu_uart_frame_index == 1U)
    {
        if (byte == IMU_UART_FRAME_ANGLE)
        {
            imu_uart_frame[1] = byte;
            imu_uart_frame_index = 2U;
        }
        else if (byte != IMU_UART_FRAME_HEADER)
        {
            imu_uart_frame_index = 0U;
        }
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
        imu_uart_save_angle_frame(imu_uart_frame);
    }
    else
    {
        imu_uart_raw_data.checksum_error_count++;
        imu_uart_resynchronize();
    }
}

/**
 * @brief Return the next DMA destination index in the circular buffer.
 * @return Index at which DMA will write the next received byte.
 */
static uint16 imu_uart_dma_producer_index(void)
{
    uint16 remaining;

    remaining = DL_DMA_getTransferSize(DMA, IMU_UART_DMA_CHANNEL);
    return (uint16)(
        (IMU_UART_DMA_BUFFER_SIZE - remaining)
        & IMU_UART_DMA_BUFFER_MASK);
}

/**
 * @brief Configure repeated byte transfers from UART2 RX to memory.
 */
static void imu_uart_dma_init(void)
{
    static const DL_DMA_Config dma_config =
    {
        .trigger = DMA_UART2_RX_TRIG,
        .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
        .transferMode = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
        .extendedMode = DL_DMA_NORMAL_MODE,
        .srcWidth = DL_DMA_WIDTH_BYTE,
        .destWidth = DL_DMA_WIDTH_BYTE,
        .srcIncrement = DL_DMA_ADDR_UNCHANGED,
        .destIncrement = DL_DMA_ADDR_INCREMENT,
    };
    uint8 discarded_byte;

    uart_set_interrupt_config(
        IMU_UART_INDEX,
        UART_INTERRUPT_CONFIG_RX_DISABLE);
    DL_UART_Main_disableDMAReceiveEvent(
        UART2,
        DL_UART_MAIN_DMA_INTERRUPT_RX);
    DL_DMA_disableChannel(DMA, IMU_UART_DMA_CHANNEL);

    while (uart_query_byte(IMU_UART_INDEX, &discarded_byte) != 0U)
    {
    }

    DL_DMA_initChannel(DMA, IMU_UART_DMA_CHANNEL, &dma_config);
    DL_DMA_setSrcAddr(
        DMA,
        IMU_UART_DMA_CHANNEL,
        (uint32)&UART2->RXDATA);
    DL_DMA_setDestAddr(
        DMA,
        IMU_UART_DMA_CHANNEL,
        (uint32)imu_uart_dma_buffer);
    DL_DMA_setTransferSize(
        DMA,
        IMU_UART_DMA_CHANNEL,
        IMU_UART_DMA_BUFFER_SIZE);
    DL_UART_Main_setRXFIFOThreshold(
        UART2,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_DMA_enableChannel(DMA, IMU_UART_DMA_CHANNEL);
    DL_UART_Main_enableDMAReceiveEvent(
        UART2,
        DL_UART_MAIN_DMA_INTERRUPT_RX);
}

/**
 * @brief Initialize the UART2 angle-module DMA interface.
 */
void imu_uart_init(void)
{
    uint32 primask;
    uint8 index;

    uart_init(
        IMU_UART_INDEX,
        IMU_UART_BAUD_RATE,
        IMU_UART_TX_PIN,
        IMU_UART_RX_PIN);
    imu_uart_dma_init();

    primask = interrupt_global_disable();

    imu_uart_frame_index = 0U;
    imu_uart_dma_consumer_index = 0U;
    for (index = 0U; index < IMU_UART_FRAME_LENGTH; index++)
    {
        imu_uart_frame[index] = 0U;
    }
    imu_uart_raw_data.angle[0] = 0;
    imu_uart_raw_data.angle[1] = 0;
    imu_uart_raw_data.angle[2] = 0;
    imu_uart_raw_data.angle_frame_count = 0U;
    imu_uart_raw_data.checksum_error_count = 0U;
    imu_uart_raw_data.angle_valid = 0U;

    interrupt_global_enable(primask);
    imu_uart_angle_init();
}

/**
 * @brief Consume all UART2 bytes received by DMA since the previous call.
 */
void imu_uart_update(void)
{
    uint16 producer_index;

    producer_index = imu_uart_dma_producer_index();
    while (imu_uart_dma_consumer_index != producer_index)
    {
        imu_uart_process_byte(
            imu_uart_dma_buffer[imu_uart_dma_consumer_index]);
        imu_uart_dma_consumer_index = (uint16)(
            (imu_uart_dma_consumer_index + 1U)
            & IMU_UART_DMA_BUFFER_MASK);
    }
}

/**
 * @brief Send the module command that initializes the Z-axis angle to zero.
 */
void imu_uart_angle_init(void)
{
    static const uint8 command[] = {0xFFU, 0xAAU, 0x52U};

    uart_write_buffer(
        IMU_UART_INDEX,
        command,
        sizeof(command));
}

/**
 * @brief Reset the yaw reference through the module angle-init command.
 */
void imu_uart_reset_yaw(void)
{
    imu_uart_angle_init();
}

/**
 * @brief Copy and convert the latest validated angle data.
 * @param data Destination converted data structure.
 * @return ZF_TRUE after at least one valid angle frame is received.
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
        data->angle_deg[index] =
            (float)raw_data.angle[index] * IMU_UART_ANGLE_SCALE;
    }
    data->angle_frame_count = raw_data.angle_frame_count;
    data->checksum_error_count = raw_data.checksum_error_count;
    data->angle_valid = raw_data.angle_valid;

    return raw_data.angle_valid != 0U ? ZF_TRUE : ZF_FALSE;
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
    uint8 angle_valid;

    if ((yaw_deg == NULL) || (angle_frame_count == NULL))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    yaw_raw = imu_uart_raw_data.angle[2];
    frame_count = imu_uart_raw_data.angle_frame_count;
    angle_valid = imu_uart_raw_data.angle_valid;
    interrupt_global_enable(primask);

    if (angle_valid == 0U)
    {
        return ZF_FALSE;
    }

    *yaw_deg = (float)yaw_raw * IMU_UART_ANGLE_SCALE;
    *angle_frame_count = frame_count;
    return ZF_TRUE;
}
