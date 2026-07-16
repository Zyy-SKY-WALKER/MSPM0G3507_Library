/**
 * @file    vofa.h
 * @brief   VOFA JustFloat telemetry and safe tuning command interface.
 */

#ifndef VOFA_H
#define VOFA_H

#include "speed_pid.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

#define VOFA_UART_CONFIGURED           (1U)
#define VOFA_UART_INDEX                (UART_1)
#define VOFA_UART_TX_PIN               (UART1_TX_B6)
#define VOFA_UART_RX_PIN               (UART1_RX_B7)
#define VOFA_UART_BAUD_RATE            (115200U)

#define VOFA_STREAM_DEFAULT_HZ         (50U)
#define VOFA_STREAM_MAX_HZ             (100U)
#define VOFA_COMMAND_TIMEOUT_MS        (3000U)
#define VOFA_CONTROL_PERIOD_MS         (10U)

typedef struct
{
    uint32 received_line_count;
    uint32 valid_command_count;
    uint32 invalid_command_count;
    uint32 rx_overflow_count;
    uint32 queue_drop_count;
    uint32 timeout_stop_count;
    float left_target_mm_s;
    float right_target_mm_s;
    uint8 armed;
    uint8 stream_enabled;
    uint8 stream_rate_hz;
} vofa_stats_struct;

/*
 * VOFA speed JustFloat channel order:
 * 0 left target, 1 left speed, 2 right target, 3 right speed,
 * 4 left duty, 5 right duty, 6 left count, 7 right count.
 *
 * VOFA left-wheel test channel order:
 * 0 left target, 1 left speed, 2 left duty.
 */

uint8 vofa_init(void);
uint8 vofa_init_tx_only(void);
void vofa_tick_10ms(void);
void vofa_process(void);
void vofa_send_speed(const speed_pid_status_struct *status);
void vofa_send_left_speed(const speed_pid_status_struct *status);
void vofa_set_stream_enabled(uint8 enabled);
void vofa_get_stats(vofa_stats_struct *stats);

#endif
