/**
 * @file    vision_uart.c
 * @brief   UART1 DMA receiver for ASCII ball-position lines.
 */

#include "vision_uart.h"

#include <string.h>
#include <ti/driverlib/dl_dma.h>

#include "zf_common_interrupt.h"
#include "zf_driver_uart.h"

#define VISION_UART_INDEX              (UART_1)
#define VISION_UART_TX_PIN             (UART1_TX_B6)
#define VISION_UART_RX_PIN             (UART1_RX_B7)
#define VISION_UART_BAUD_RATE          (115200U)

#define VISION_UART_DMA_CHANNEL        (1U)
#define VISION_UART_DMA_BUFFER_SIZE    (4096U)
#define VISION_UART_DMA_BUFFER_MASK    (VISION_UART_DMA_BUFFER_SIZE - 1U)
static volatile uint8 vision_uart_dma_buffer[
    VISION_UART_DMA_BUFFER_SIZE];
static char vision_uart_line[VISION_UART_LINE_MAX_LENGTH];
static uint8 vision_uart_line_length;
static uint8 vision_uart_discard_line;
static uint16 vision_uart_dma_consumer_index;
static vision_uart_data_struct vision_uart_data;

/**
 * @brief Parse one fixed-width signed position in 0.01 cm.
 * @return ZF_TRUE for [+-]DD.DD within the physical ball range.
 */
static uint8 vision_uart_parse_position_centi_cm(int32 *position_centi_cm)
{
    int32 magnitude;

    if((vision_uart_line_length != VISION_UART_POSITION_TEXT_LENGTH)
        || ((vision_uart_line[0] != '+') && (vision_uart_line[0] != '-'))
        || (vision_uart_line[3] != '.')
        || (vision_uart_line[1] < '0')
        || (vision_uart_line[1] > '9')
        || (vision_uart_line[2] < '0')
        || (vision_uart_line[2] > '9')
        || (vision_uart_line[4] < '0')
        || (vision_uart_line[4] > '9')
        || (vision_uart_line[5] < '0')
        || (vision_uart_line[5] > '9'))
    {
        return ZF_FALSE;
    }

    magnitude = ((int32)(vision_uart_line[1] - '0') * 1000)
        + ((int32)(vision_uart_line[2] - '0') * 100)
        + ((int32)(vision_uart_line[4] - '0') * 10)
        + (int32)(vision_uart_line[5] - '0');
    if(magnitude > VISION_UART_POSITION_LIMIT_CENTI_CM)
    {
        return ZF_FALSE;
    }
    *position_centi_cm = vision_uart_line[0] == '-'
        ? -magnitude : magnitude;
    return ZF_TRUE;
}

/**
 * @brief Decode and publish one complete ball-position or invalid line.
 */
static void vision_uart_process_line(void)
{
    int32 position_centi_cm;

    if((vision_uart_line_length == 1U)
        && (vision_uart_line[0] == VISION_UART_INVALID_MARKER))
    {
        vision_uart_data.packet_count++;
        vision_uart_data.invalid_packet_count++;
        vision_uart_data.recognition_valid = 0U;
        return;
    }

    if(vision_uart_parse_position_centi_cm(&position_centi_cm) == 0U)
    {
        vision_uart_data.format_error_count++;
        vision_uart_data.recognition_valid = 0U;
        return;
    }

    vision_uart_data.packet_count++;
    vision_uart_data.position_centi_cm = position_centi_cm;
    vision_uart_data.valid_packet_count++;
    vision_uart_data.recognition_valid = 1U;
}

/**
 * @brief Advance the newline-delimited parser by one byte.
 */
static void vision_uart_process_byte(uint8 byte)
{
    if (byte == '\n')
    {
        if ((vision_uart_discard_line == 0U)
            && (vision_uart_line_length != 0U))
        {
            vision_uart_process_line();
        }
        vision_uart_line_length = 0U;
        vision_uart_discard_line = 0U;
        return;
    }
    if(vision_uart_discard_line != 0U)
    {
        return;
    }
    if (vision_uart_line_length
        >= (VISION_UART_LINE_MAX_LENGTH - 1U))
    {
        vision_uart_data.overflow_count++;
        vision_uart_data.recognition_valid = 0U;
        vision_uart_line_length = 0U;
        vision_uart_discard_line = 1U;
        return;
    }
    vision_uart_line[vision_uart_line_length] = (char)byte;
    vision_uart_line_length++;
}

/**
 * @brief Return the index at which DMA will write the next byte.
 */
static uint16 vision_uart_dma_producer_index(void)
{
    uint16 remaining = DL_DMA_getTransferSize(
        DMA,
        VISION_UART_DMA_CHANNEL);

    return (uint16)((VISION_UART_DMA_BUFFER_SIZE - remaining)
        & VISION_UART_DMA_BUFFER_MASK);
}

/**
 * @brief Configure repeated UART1 RX byte transfers into a ring buffer.
 */
static void vision_uart_dma_init(void)
{
    static const DL_DMA_Config dma_config =
    {
        .trigger = DMA_UART1_RX_TRIG,
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
        VISION_UART_INDEX,
        UART_INTERRUPT_CONFIG_ALL_DISABLE);
    DL_UART_Main_disableDMAReceiveEvent(
        UART1,
        DL_UART_MAIN_DMA_INTERRUPT_RX);
    DL_DMA_disableChannel(DMA, VISION_UART_DMA_CHANNEL);
    while (uart_query_byte(VISION_UART_INDEX, &discarded_byte) != 0U)
    {
    }

    DL_DMA_initChannel(DMA, VISION_UART_DMA_CHANNEL, &dma_config);
    DL_DMA_setSrcAddr(
        DMA,
        VISION_UART_DMA_CHANNEL,
        (uint32)&UART1->RXDATA);
    DL_DMA_setDestAddr(
        DMA,
        VISION_UART_DMA_CHANNEL,
        (uint32)vision_uart_dma_buffer);
    DL_DMA_setTransferSize(
        DMA,
        VISION_UART_DMA_CHANNEL,
        VISION_UART_DMA_BUFFER_SIZE);
    DL_UART_Main_setRXFIFOThreshold(
        UART1,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_DMA_enableChannel(DMA, VISION_UART_DMA_CHANNEL);
    DL_UART_Main_enableDMAReceiveEvent(
        UART1,
        DL_UART_MAIN_DMA_INTERRUPT_RX);
}

/**
 * @brief Initialize UART1 and its receive DMA channel.
 */
uint8 vision_uart_init(void)
{
    if (VISION_UART_DMA_BUFFER_SIZE
        != (VISION_UART_DMA_BUFFER_MASK + 1U))
    {
        return ZF_FALSE;
    }

    vision_uart_line_length = 0U;
    vision_uart_discard_line = 0U;
    vision_uart_dma_consumer_index = 0U;
    memset(vision_uart_line, 0, sizeof(vision_uart_line));
    memset(&vision_uart_data, 0, sizeof(vision_uart_data));

    uart_init(
        VISION_UART_INDEX,
        VISION_UART_BAUD_RATE,
        VISION_UART_TX_PIN,
        VISION_UART_RX_PIN);
    vision_uart_dma_init();
    return ZF_TRUE;
}

/**
 * @brief Consume every byte currently available in the DMA ring.
 */
void vision_uart_update(void)
{
    uint16 producer_index = vision_uart_dma_producer_index();

    while (vision_uart_dma_consumer_index != producer_index)
    {
        vision_uart_process_byte(
            vision_uart_dma_buffer[vision_uart_dma_consumer_index]);
        vision_uart_dma_consumer_index = (uint16)(
            (vision_uart_dma_consumer_index + 1U)
            & VISION_UART_DMA_BUFFER_MASK);
    }
}

/**
 * @brief Copy the latest parser snapshot.
 */
uint8 vision_uart_get_data(vision_uart_data_struct *data)
{
    if (data == NULL)
    {
        return ZF_FALSE;
    }

    *data = vision_uart_data;
    return vision_uart_data.recognition_valid;
}
