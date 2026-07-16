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

/** @brief VOFA command parser, safety and stream diagnostics. */
typedef struct
{
    /** Number of nonempty command lines queued by the receive ISR. */
    uint32 received_line_count;
    /** Number of accepted commands processed in the foreground. */
    uint32 valid_command_count;
    /** Number of rejected command lines processed in the foreground. */
    uint32 invalid_command_count;
    /** Number of overlength receive lines discarded by the ISR. */
    uint32 rx_overflow_count;
    /** Number of complete lines dropped because the queue was full. */
    uint32 queue_drop_count;
    /** Number of armed command timeouts that stopped the motors. */
    uint32 timeout_stop_count;
    /** Current retained left target in millimeters per second. */
    float left_target_mm_s;
    /** Current retained right target in millimeters per second. */
    float right_target_mm_s;
    /** Nonzero while TARGET commands are permitted. */
    uint8 armed;
    /** Nonzero while periodic JustFloat telemetry is enabled. */
    uint8 stream_enabled;
    /** Configured 10 ms-aligned telemetry rate in hertz. */
    uint8 stream_rate_hz;
} vofa_stats_struct;

/*
 * VOFA speed JustFloat channel order:
 * 0 left target, 1 left speed, 2 right target, 3 right speed,
 * 4 left duty, 5 right duty, 6 left count, 7 right count.
 *
 * VOFA right-wheel test channel order:
 * 0 right target, 1 right speed, 2 right duty.
 *
 * Accepted stream rates are 1, 2, 4, 5, 10, 20, 25, 50 and 100 Hz.
 */

/**
 * @brief Initialize VOFA UART reception, parser and disarmed safety state.
 * @return ZF_TRUE when the UART and float configuration are valid.
 * @note Enables receive interrupts and 50 Hz telemetry by default.
 */
uint8 vofa_init(void);

/**
 * @brief Initialize the VOFA UART for polling transmission only.
 * @return ZF_TRUE when the UART and float configuration are valid.
 * @note Disables all UART interrupts and does not initialize parser state.
 */
uint8 vofa_init_tx_only(void);

/**
 * @brief Advance safety timeout and stream timing by one 10 ms period.
 * @note Call from exactly one 10 ms control context after vofa_init().
 * @note An armed three-second timeout stops the speed controller.
 */
void vofa_tick_10ms(void);

/**
 * @brief Process queued commands and transmit due speed telemetry.
 * @note Call repeatedly from foreground context after vofa_init().
 */
void vofa_process(void);

/**
 * @brief Transmit the fixed eight-channel speed JustFloat frame.
 * @param status Coherent speed PID status; NULL causes no operation.
 * @note Requires successful VOFA initialization and writes to the UART.
 */
void vofa_send_speed(const speed_pid_status_struct *status);

/**
 * @brief Transmit right target, measured speed and duty as three channels.
 * @param status Coherent speed PID status; NULL causes no operation.
 * @note Requires successful VOFA initialization and writes to the UART.
 */
void vofa_send_right_speed(const speed_pid_status_struct *status);

/**
 * @brief Enable or disable periodic foreground telemetry.
 * @param enabled Nonzero enables streaming; zero disables it.
 */
void vofa_set_stream_enabled(uint8 enabled);

/**
 * @brief Copy one interrupt-coherent VOFA diagnostic snapshot.
 * @param stats Destination statistics; NULL causes no operation.
 */
void vofa_get_stats(vofa_stats_struct *stats);

#endif
