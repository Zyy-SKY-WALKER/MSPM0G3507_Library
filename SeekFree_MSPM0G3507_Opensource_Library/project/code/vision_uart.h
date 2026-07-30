/**
 * @file    vision_uart.h
 * @brief   UART1 DMA receiver for ASCII ball-position lines.
 */

#ifndef VISION_UART_H
#define VISION_UART_H

#include "zf_common_typedef.h"

#define VISION_UART_LINE_MAX_LENGTH            (32U)
#define VISION_UART_POSITION_TEXT_LENGTH       (6U)
#define VISION_UART_POSITION_LIMIT_CENTI_CM    (1250)
#define VISION_UART_INVALID_MARKER             ('!')

/** @brief Latest ball position and parser diagnostics. */
typedef struct
{
    /** Latest ball position in 0.01 cm; positive points toward the vehicle front. */
    int32 position_centi_cm;
    /** Number of accepted position or explicit invalid frames. */
    uint32 packet_count;
    /** Number of strictly parsed signed-decimal position frames. */
    uint32 valid_packet_count;
    /** Number of explicit ! recognition-failure frames. */
    uint32 invalid_packet_count;
    /** Number of malformed frames rejected by the parser. */
    uint32 format_error_count;
    /** Number of overlength lines discarded before parsing. */
    uint32 overflow_count;
    /** Nonzero only when the latest parsed frame contained a ball position. */
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
 * @brief Copy the latest ball position and parser diagnostics.
 * @param data Destination snapshot; must not be NULL.
 * @return ZF_TRUE only when the most recent parsed line was usable.
 * @note Valid frames are +05.23\n; !\n marks a recognition failure and does
 *       not overwrite the last accepted position.
 */
uint8 vision_uart_get_data(vision_uart_data_struct *data);

#endif
