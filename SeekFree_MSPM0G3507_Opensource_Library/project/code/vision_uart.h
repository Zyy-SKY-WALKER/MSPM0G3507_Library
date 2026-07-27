/**
 * @file    vision_uart.h
 * @brief   UART1 DMA receiver for ASCII vision-error lines.
 */

#ifndef VISION_UART_H
#define VISION_UART_H

#include "zf_common_typedef.h"

#define VISION_UART_LINE_HEADER        ('D')
#define VISION_UART_LINE_MAX_LENGTH    (32U)

/** @brief Latest accepted coordinates and parser diagnostics. */
typedef struct
{
    /** Latest target-X minus laser-X error from a valid line. */
    int32 error_x;
    /** Latest target-Y minus laser-Y error from a valid line. */
    int32 error_y;
    /** Number of strictly parsed D,x,y lines. */
    uint32 packet_count;
    /** Number of parsed lines other than D,-1,-1. */
    uint32 valid_packet_count;
    /** Number of parsed D,-1,-1 recognition-failure lines. */
    uint32 invalid_packet_count;
    /** Number of malformed D-prefixed lines rejected by the parser. */
    uint32 format_error_count;
    /** Number of overlength lines discarded before parsing. */
    uint32 overflow_count;
    /** Nonzero only when the latest parsed line contained usable errors. */
    uint8 recognition_valid;
} vision_uart_data_struct;

/**
 * @brief Initialize UART1 B6/B7 at 115200 baud with DMA channel 1 RX.
 * @return ZF_TRUE when the DMA ring configuration is supported.
 * @note UART1 cannot simultaneously serve VOFA or stepper debug output.
 */
uint8 vision_uart_init(void);

/**
 * @brief Consume all bytes received by DMA since the previous call.
 * @note Call repeatedly from foreground code; this function does not block.
 */
void vision_uart_update(void);

/**
 * @brief Copy the latest errors and parser diagnostics.
 * @param data Destination snapshot; must not be NULL.
 * @return ZF_TRUE only when the most recent parsed line was usable.
 * @note D,-1,-1 does not overwrite the last accepted error values.
 */
uint8 vision_uart_get_data(vision_uart_data_struct *data);

#endif
