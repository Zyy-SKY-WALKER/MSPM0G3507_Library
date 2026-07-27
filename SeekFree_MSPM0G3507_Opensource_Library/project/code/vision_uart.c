/**
 * @file    vision_uart.c
 * @brief   UART1 DMA receiver for ASCII vision-error lines.
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
#define VISION_UART_INT32_MAX_VALUE    (2147483647U)
#define VISION_UART_INT32_MIN_MAGNITUDE (2147483648U)

static volatile uint8 vision_uart_dma_buffer[
    VISION_UART_DMA_BUFFER_SIZE];
static char vision_uart_line[VISION_UART_LINE_MAX_LENGTH];
static uint8 vision_uart_line_length;
static uint8 vision_uart_discard_line;
static uint16 vision_uart_dma_consumer_index;
static vision_uart_data_struct vision_uart_data;

/**
 * @brief Parse one strict signed decimal int32 field.
 * @param cursor Input and end position.
 * @param value Parsed result.
 * @return ZF_TRUE when at least one digit was parsed without overflow.
 */
static uint8 vision_uart_parse_int32(
    const char **cursor,
    int32 *value)
{
    const char *position = *cursor;
    uint32 magnitude = 0U;
    uint32 limit = VISION_UART_INT32_MAX_VALUE;
    uint8 negative = 0U;
    uint8 digit_count = 0U;

    if (*position == '-')
    {
        negative = 1U;
        limit = VISION_UART_INT32_MIN_MAGNITUDE;
        position++;
    }
    else if (*position == '+')
    {
        position++;
    }

    while ((*position >= '0') && (*position <= '9'))
    {
        uint32 digit = (uint32)(*position - '0');

        if (magnitude > ((limit - digit) / 10U))
        {
            return ZF_FALSE;
        }
        magnitude = (magnitude * 10U) + digit;
        digit_count++;
        position++;
    }
    if (digit_count == 0U)
    {
        return ZF_FALSE;
    }

    if (negative != 0U)
    {
        if (magnitude == VISION_UART_INT32_MIN_MAGNITUDE)
        {
            *value = (-2147483647 - 1);
        }
        else
        {
            *value = -(int32)magnitude;
        }
    }
    else
    {
        *value = (int32)magnitude;
    }
    *cursor = position;
    return ZF_TRUE;
}

/**
 * @brief Decode and publish one complete D,x,y line.
 */
static void vision_uart_process_line(void)
{
    const char *cursor;
    int32 error_x;
    int32 error_y;

    vision_uart_line[vision_uart_line_length] = '\0';
    if ((vision_uart_line_length == 0U)
        || (vision_uart_line[vision_uart_line_length - 1U] != '\r'))
    {
        vision_uart_data.format_error_count++;
        vision_uart_data.recognition_valid = 0U;
        return;
    }
    else
    {
        vision_uart_line_length--;
        vision_uart_line[vision_uart_line_length] = '\0';
    }

    cursor = vision_uart_line;
    if ((cursor[0] != VISION_UART_LINE_HEADER)
        || (cursor[1] != ','))
    {
        vision_uart_data.format_error_count++;
        vision_uart_data.recognition_valid = 0U;
        return;
    }
    cursor += 2;
    if ((vision_uart_parse_int32(&cursor, &error_x) == 0U)
        || (*cursor != ','))
    {
        vision_uart_data.format_error_count++;
        vision_uart_data.recognition_valid = 0U;
        return;
    }
    cursor++;
    if ((vision_uart_parse_int32(&cursor, &error_y) == 0U)
        || (*cursor != '\0'))
    {
        vision_uart_data.format_error_count++;
        vision_uart_data.recognition_valid = 0U;
        return;
    }

    vision_uart_data.packet_count++;
    if ((error_x == -1) && (error_y == -1))
    {
        vision_uart_data.invalid_packet_count++;
        vision_uart_data.recognition_valid = 0U;
    }
    else
    {
        vision_uart_data.error_x = error_x;
        vision_uart_data.error_y = error_y;
        vision_uart_data.valid_packet_count++;
        vision_uart_data.recognition_valid = 1U;
    }
}

/**
 * @brief Advance the newline-delimited parser by one byte.
 */
static void vision_uart_process_byte(uint8 byte)
{
    if (byte == (uint8)VISION_UART_LINE_HEADER)
    {
        if ((vision_uart_line_length != 0U)
            && (vision_uart_discard_line == 0U))
        {
            vision_uart_data.format_error_count++;
        }
        vision_uart_line[0] = VISION_UART_LINE_HEADER;
        vision_uart_line_length = 1U;
        vision_uart_discard_line = 0U;
        return;
    }

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
    if ((vision_uart_discard_line != 0U)
        || (vision_uart_line_length == 0U))
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
