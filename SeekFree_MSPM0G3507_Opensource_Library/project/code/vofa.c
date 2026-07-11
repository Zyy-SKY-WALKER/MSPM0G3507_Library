/**
 * @file    vofa.c
 * @brief   VOFA JustFloat telemetry and safe tuning command interface.
 */

#include "vofa.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>

#include "zf_common_interrupt.h"

#define VOFA_RX_LINE_LENGTH            (64U)
#define VOFA_RX_QUEUE_LENGTH           (4U)
#define VOFA_MAX_TOKENS                (4U)
#define VOFA_FLOAT_CHANNEL_COUNT       (8U)
#define VOFA_FLOAT_SIZE                (4U)
#define VOFA_FRAME_SIZE                \
    ((VOFA_FLOAT_CHANNEL_COUNT * VOFA_FLOAT_SIZE) + 4U)
#define VOFA_GAIN_LIMIT                (1000.0F)
#define VOFA_TARGET_LIMIT_MM_S         (600.0F)

typedef struct
{
    float kp;
    float ki;
    float kd;
} vofa_gain_struct;

static char vofa_rx_current_line[VOFA_RX_LINE_LENGTH];
static char vofa_rx_queue[VOFA_RX_QUEUE_LENGTH][VOFA_RX_LINE_LENGTH];
static uint8 vofa_rx_current_length;
static uint8 vofa_rx_discarding;
static volatile uint8 vofa_rx_write_index;
static volatile uint8 vofa_rx_read_index;

static volatile uint32 vofa_uptime_ms;
static volatile uint32 vofa_last_command_ms;
static uint32 vofa_last_stream_ms;
static volatile uint32 vofa_received_line_count;
static volatile uint32 vofa_valid_command_count;
static volatile uint32 vofa_invalid_command_count;
static volatile uint32 vofa_rx_overflow_count;
static volatile uint32 vofa_queue_drop_count;
static volatile uint32 vofa_timeout_stop_count;

static volatile uint8 vofa_initialized;
static volatile uint8 vofa_armed;
static volatile uint8 vofa_stream_enabled;
static volatile uint8 vofa_stream_rate_hz;

static vofa_gain_struct vofa_left_gains;
static vofa_gain_struct vofa_right_gains;

/**
 * @brief Return the next ring-buffer index.
 * @param index Current index.
 * @return Wrapped next index.
 */
static uint8 vofa_next_queue_index(uint8 index)
{
    index++;
    if (index >= VOFA_RX_QUEUE_LENGTH)
    {
        index = 0U;
    }

    return index;
}

/**
 * @brief Publish one completed ISR line to the foreground queue.
 */
static void vofa_publish_current_line(void)
{
    uint8 next_index;
    uint8 index;

    if (vofa_rx_current_length == 0U)
    {
        return;
    }

    next_index = vofa_next_queue_index(vofa_rx_write_index);
    if (next_index == vofa_rx_read_index)
    {
        vofa_queue_drop_count++;
        vofa_rx_current_length = 0U;
        return;
    }

    for (index = 0U; index < vofa_rx_current_length; index++)
    {
        vofa_rx_queue[vofa_rx_write_index][index] =
            vofa_rx_current_line[index];
    }
    vofa_rx_queue[vofa_rx_write_index][vofa_rx_current_length] = '\0';
    vofa_rx_write_index = next_index;
    vofa_received_line_count++;
    vofa_rx_current_length = 0U;
}

/**
 * @brief Consume one command-stream byte in UART interrupt context.
 * @param byte Received byte.
 */
static void vofa_process_rx_byte(uint8 byte)
{
    if ((byte == '\r') || (byte == '\n'))
    {
        if (vofa_rx_discarding != 0U)
        {
            vofa_rx_discarding = 0U;
            vofa_rx_current_length = 0U;
            return;
        }

        vofa_publish_current_line();
        return;
    }

    if (vofa_rx_discarding != 0U)
    {
        return;
    }

    if (vofa_rx_current_length >= (VOFA_RX_LINE_LENGTH - 1U))
    {
        vofa_rx_discarding = 1U;
        vofa_rx_current_length = 0U;
        vofa_rx_overflow_count++;
        return;
    }

    vofa_rx_current_line[vofa_rx_current_length] = (char)byte;
    vofa_rx_current_length++;
}

/**
 * @brief Drain UART2 bytes in interrupt context.
 * @param state UART interrupt state.
 * @param user_data Optional callback context.
 */
static void vofa_uart_callback(uint32 state, void *user_data)
{
    uint8 byte;

    (void)user_data;

    if (state != UART_INTERRUPT_STATE_RX)
    {
        return;
    }

    while (uart_query_byte(VOFA_UART_INDEX, &byte) != 0U)
    {
        vofa_process_rx_byte(byte);
    }
}

/**
 * @brief Copy one queued command line for foreground processing.
 * @param line Destination line buffer.
 * @return ZF_TRUE when one line was copied.
 */
static uint8 vofa_get_line(char line[])
{
    uint32 primask;
    uint8 index;

    primask = interrupt_global_disable();
    if (vofa_rx_read_index == vofa_rx_write_index)
    {
        interrupt_global_enable(primask);
        return ZF_FALSE;
    }

    for (index = 0U; index < VOFA_RX_LINE_LENGTH; index++)
    {
        line[index] = vofa_rx_queue[vofa_rx_read_index][index];
        if (line[index] == '\0')
        {
            break;
        }
    }
    line[VOFA_RX_LINE_LENGTH - 1U] = '\0';
    vofa_rx_read_index = vofa_next_queue_index(vofa_rx_read_index);
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Convert one ASCII token to uppercase in place.
 * @param text Null-terminated token.
 */
static void vofa_uppercase(char text[])
{
    while (*text != '\0')
    {
        if ((*text >= 'a') && (*text <= 'z'))
        {
            *text = (char)(*text - ('a' - 'A'));
        }
        text++;
    }
}

/**
 * @brief Split one comma-separated command into nonempty tokens.
 * @param line Mutable command line.
 * @param tokens Destination token pointer array.
 * @return Token count, or zero for malformed input.
 */
static uint8 vofa_split_line(char line[], char *tokens[])
{
    uint8 token_count = 1U;
    char *cursor = line;

    if (*line == '\0')
    {
        return 0U;
    }

    tokens[0] = line;
    while (*cursor != '\0')
    {
        if (*cursor == ',')
        {
            if ((cursor == tokens[token_count - 1U])
                || (token_count >= VOFA_MAX_TOKENS))
            {
                return 0U;
            }

            *cursor = '\0';
            tokens[token_count] = cursor + 1;
            token_count++;
        }
        cursor++;
    }

    if (*tokens[token_count - 1U] == '\0')
    {
        return 0U;
    }

    return token_count;
}

/**
 * @brief Parse one complete finite floating-point token.
 * @param text Source token.
 * @param value Destination value.
 * @return ZF_TRUE when the complete token is valid.
 */
static uint8 vofa_parse_float(const char text[], float *value)
{
    char *end;
    float parsed_value;

    if ((text == NULL) || (value == NULL)
        || (*text == ' ') || (*text == '\t')
        || (*text == '\r') || (*text == '\n')
        || (*text == '+'))
    {
        return ZF_FALSE;
    }

    parsed_value = strtof(text, &end);
    if ((end == text) || (*end != '\0')
        || (parsed_value != parsed_value)
        || (parsed_value > FLT_MAX)
        || (parsed_value < -FLT_MAX))
    {
        return ZF_FALSE;
    }

    *value = parsed_value;
    return ZF_TRUE;
}

/**
 * @brief Parse one complete unsigned decimal token.
 * @param text Source token.
 * @param value Destination value.
 * @return ZF_TRUE when the complete token is valid.
 */
static uint8 vofa_parse_uint(const char text[], uint32 *value)
{
    char *end;
    unsigned long parsed_value;

    if ((text == NULL) || (value == NULL)
        || (*text == '-') || (*text == '+')
        || (*text == ' ') || (*text == '\t')
        || (*text == '\r') || (*text == '\n'))
    {
        return ZF_FALSE;
    }

    parsed_value = strtoul(text, &end, 10);
    if ((end == text) || (*end != '\0')
        || (parsed_value > 0xFFFFFFFFUL))
    {
        return ZF_FALSE;
    }

    *value = (uint32)parsed_value;
    return ZF_TRUE;
}

/**
 * @brief Mark one non-motion command as valid.
 */
static void vofa_mark_command_valid(void)
{
    uint32 primask = interrupt_global_disable();

    vofa_valid_command_count++;

    interrupt_global_enable(primask);
}

/**
 * @brief Arm target commands and refresh the command timeout atomically.
 */
static void vofa_arm(void)
{
    uint32 primask = interrupt_global_disable();

    vofa_armed = 1U;
    vofa_valid_command_count++;
    vofa_last_command_ms = vofa_uptime_ms;

    interrupt_global_enable(primask);
}

/**
 * @brief Stop both motors and disarm target commands atomically.
 */
static void vofa_stop_and_disarm(void)
{
    uint32 primask = interrupt_global_disable();

    vofa_armed = 0U;
    vofa_valid_command_count++;
    vofa_last_command_ms = vofa_uptime_ms;
    speed_pid_stop();

    interrupt_global_enable(primask);
}

/**
 * @brief Apply a target only while armed, without racing timeout stop.
 * @param left_mm_s Left wheel target speed.
 * @param right_mm_s Right wheel target speed.
 * @return ZF_TRUE when the target was accepted.
 */
static uint8 vofa_apply_target_if_armed(
    float left_mm_s,
    float right_mm_s)
{
    uint32 primask = interrupt_global_disable();

    if (vofa_armed == 0U)
    {
        interrupt_global_enable(primask);
        return ZF_FALSE;
    }

    vofa_valid_command_count++;
    vofa_last_command_ms = vofa_uptime_ms;
    speed_pid_set_target(left_mm_s, right_mm_s);

    interrupt_global_enable(primask);
    return ZF_TRUE;
}

/**
 * @brief Apply one gain field to selected wheel controllers.
 * @param command KP, KI or KD token.
 * @param selector L, R or B token.
 * @param value Validated gain value.
 */
static void vofa_apply_gain(
    const char command[],
    const char selector[],
    float value)
{
    if ((selector[0] == 'L') || (selector[0] == 'B'))
    {
        if (strcmp(command, "KP") == 0)
        {
            vofa_left_gains.kp = value;
        }
        else if (strcmp(command, "KI") == 0)
        {
            vofa_left_gains.ki = value;
        }
        else
        {
            vofa_left_gains.kd = value;
        }

        speed_pid_set_left_gains(
            vofa_left_gains.kp,
            vofa_left_gains.ki,
            vofa_left_gains.kd);
    }

    if ((selector[0] == 'R') || (selector[0] == 'B'))
    {
        if (strcmp(command, "KP") == 0)
        {
            vofa_right_gains.kp = value;
        }
        else if (strcmp(command, "KI") == 0)
        {
            vofa_right_gains.ki = value;
        }
        else
        {
            vofa_right_gains.kd = value;
        }

        speed_pid_set_right_gains(
            vofa_right_gains.kp,
            vofa_right_gains.ki,
            vofa_right_gains.kd);
    }
}

/**
 * @brief Process one validated, tokenized foreground command line.
 * @param line Mutable command line.
 * @return ZF_TRUE when the command was accepted.
 */
static uint8 vofa_process_line(char line[])
{
    char *tokens[VOFA_MAX_TOKENS];
    float first_value;
    float second_value;
    uint32 uint_value;
    uint32 primask;
    uint8 token_count;

    token_count = vofa_split_line(line, tokens);
    if (token_count == 0U)
    {
        return ZF_FALSE;
    }

    vofa_uppercase(tokens[0]);
    if (token_count > 1U)
    {
        vofa_uppercase(tokens[1]);
    }

    if ((strcmp(tokens[0], "ARM") == 0) && (token_count == 1U))
    {
        vofa_arm();
        return ZF_TRUE;
    }

    if (((strcmp(tokens[0], "STOP") == 0)
            || (strcmp(tokens[0], "DISARM") == 0))
        && (token_count == 1U))
    {
        vofa_stop_and_disarm();
        return ZF_TRUE;
    }

    if ((strcmp(tokens[0], "TARGET") == 0)
        && (token_count == 3U)
        && (vofa_parse_float(tokens[1], &first_value) != 0U)
        && (vofa_parse_float(tokens[2], &second_value) != 0U)
        && (first_value >= -VOFA_TARGET_LIMIT_MM_S)
        && (first_value <= VOFA_TARGET_LIMIT_MM_S)
        && (second_value >= -VOFA_TARGET_LIMIT_MM_S)
        && (second_value <= VOFA_TARGET_LIMIT_MM_S))
    {
        return vofa_apply_target_if_armed(first_value, second_value);
    }

    if (((strcmp(tokens[0], "KP") == 0)
            || (strcmp(tokens[0], "KI") == 0)
            || (strcmp(tokens[0], "KD") == 0))
        && (token_count == 3U)
        && ((strcmp(tokens[1], "L") == 0)
            || (strcmp(tokens[1], "R") == 0)
            || (strcmp(tokens[1], "B") == 0))
        && (vofa_parse_float(tokens[2], &first_value) != 0U)
        && (first_value >= -VOFA_GAIN_LIMIT)
        && (first_value <= VOFA_GAIN_LIMIT))
    {
        vofa_apply_gain(tokens[0], tokens[1], first_value);
        vofa_mark_command_valid();
        return ZF_TRUE;
    }

    if ((strcmp(tokens[0], "STREAM") == 0)
        && (token_count == 2U)
        && (vofa_parse_uint(tokens[1], &uint_value) != 0U)
        && (uint_value <= 1U))
    {
        vofa_set_stream_enabled((uint8)uint_value);
        vofa_mark_command_valid();
        return ZF_TRUE;
    }

    if ((strcmp(tokens[0], "RATE") == 0)
        && (token_count == 2U)
        && (vofa_parse_uint(tokens[1], &uint_value) != 0U)
        && (uint_value >= 1U)
        && (uint_value <= VOFA_STREAM_MAX_HZ)
        && ((1000U % uint_value) == 0U)
        && (((1000U / uint_value) % VOFA_CONTROL_PERIOD_MS) == 0U))
    {
        primask = interrupt_global_disable();
        vofa_stream_rate_hz = (uint8)uint_value;
        interrupt_global_enable(primask);
        vofa_mark_command_valid();
        return ZF_TRUE;
    }

    return ZF_FALSE;
}

/**
 * @brief Initialize VOFA UART, parser and safety state.
 * @return ZF_TRUE when UART configuration is enabled and valid.
 */
uint8 vofa_init(void)
{
    uint32 primask;

    vofa_initialized = 0U;
    if (VOFA_UART_CONFIGURED == 0U)
    {
        return ZF_FALSE;
    }

    if ((VOFA_UART_INDEX != UART_2)
        || (((VOFA_UART_TX_PIN >> UART_INDEX_OFFSET) & UART_INDEX_MASK)
            != VOFA_UART_INDEX)
        || (((VOFA_UART_RX_PIN >> UART_INDEX_OFFSET) & UART_INDEX_MASK)
            != VOFA_UART_INDEX)
        || ((VOFA_UART_TX_PIN & UART_PIN_INDEX_MASK)
            == (VOFA_UART_RX_PIN & UART_PIN_INDEX_MASK))
        || (sizeof(float) != VOFA_FLOAT_SIZE))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    vofa_rx_current_length = 0U;
    vofa_rx_discarding = 0U;
    vofa_rx_write_index = 0U;
    vofa_rx_read_index = 0U;
    vofa_uptime_ms = 0U;
    vofa_last_command_ms = 0U;
    vofa_last_stream_ms = 0U;
    vofa_received_line_count = 0U;
    vofa_valid_command_count = 0U;
    vofa_invalid_command_count = 0U;
    vofa_rx_overflow_count = 0U;
    vofa_queue_drop_count = 0U;
    vofa_timeout_stop_count = 0U;
    vofa_armed = 0U;
    vofa_stream_enabled = 1U;
    vofa_stream_rate_hz = VOFA_STREAM_DEFAULT_HZ;
    vofa_left_gains.kp = SPEED_PID_LEFT_KP;
    vofa_left_gains.ki = SPEED_PID_LEFT_KI;
    vofa_left_gains.kd = SPEED_PID_LEFT_KD;
    vofa_right_gains.kp = SPEED_PID_RIGHT_KP;
    vofa_right_gains.ki = SPEED_PID_RIGHT_KI;
    vofa_right_gains.kd = SPEED_PID_RIGHT_KD;
    interrupt_global_enable(primask);

    uart_init(
        VOFA_UART_INDEX,
        VOFA_UART_BAUD_RATE,
        VOFA_UART_TX_PIN,
        VOFA_UART_RX_PIN);
    uart_set_callback(VOFA_UART_INDEX, vofa_uart_callback, NULL);
    uart_set_interrupt_config(
        VOFA_UART_INDEX,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
    vofa_initialized = 1U;

    return ZF_TRUE;
}

/**
 * @brief Advance timeout timing from the fixed 10 ms control callback.
 */
void vofa_tick_10ms(void)
{
    if (vofa_initialized == 0U)
    {
        return;
    }

    vofa_uptime_ms += VOFA_CONTROL_PERIOD_MS;
    if ((vofa_armed != 0U)
        && ((uint32)(vofa_uptime_ms - vofa_last_command_ms)
            >= VOFA_COMMAND_TIMEOUT_MS))
    {
        vofa_armed = 0U;
        vofa_timeout_stop_count++;
        speed_pid_stop();
    }
}

/**
 * @brief Send one fixed eight-channel speed JustFloat frame.
 * @param status Coherent speed PID status snapshot.
 */
void vofa_send_speed(const speed_pid_status_struct *status)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float channels[VOFA_FLOAT_CHANNEL_COUNT];
    uint8 frame[VOFA_FRAME_SIZE];
    uint8 channel;

    if ((status == NULL) || (vofa_initialized == 0U))
    {
        return;
    }

    channels[0] = status->left_target_mm_s;
    channels[1] = status->left_speed_mm_s;
    channels[2] = status->right_target_mm_s;
    channels[3] = status->right_speed_mm_s;
    channels[4] = (float)status->left_duty;
    channels[5] = (float)status->right_duty;
    channels[6] = (float)status->left_count;
    channels[7] = (float)status->right_count;

    for (channel = 0U; channel < VOFA_FLOAT_CHANNEL_COUNT; channel++)
    {
        memcpy(
            &frame[channel * VOFA_FLOAT_SIZE],
            &channels[channel],
            VOFA_FLOAT_SIZE);
    }

    memcpy(
        &frame[VOFA_FLOAT_CHANNEL_COUNT * VOFA_FLOAT_SIZE],
        tail,
        sizeof(tail));

    uart_write_buffer(VOFA_UART_INDEX, frame, sizeof(frame));
}

/**
 * @brief Parse queued commands and send due foreground telemetry.
 */
void vofa_process(void)
{
    speed_pid_status_struct status;
    char line[VOFA_RX_LINE_LENGTH];
    uint32 primask;
    uint32 now_ms;
    uint32 interval_ms;
    uint8 stream_enabled;
    uint8 stream_rate_hz;

    if (vofa_initialized == 0U)
    {
        return;
    }

    while (vofa_get_line(line) != 0U)
    {
        if (vofa_process_line(line) == ZF_FALSE)
        {
            primask = interrupt_global_disable();
            vofa_invalid_command_count++;
            interrupt_global_enable(primask);
        }
    }

    primask = interrupt_global_disable();
    now_ms = vofa_uptime_ms;
    stream_enabled = vofa_stream_enabled;
    stream_rate_hz = vofa_stream_rate_hz;
    interrupt_global_enable(primask);

    if ((stream_enabled == 0U) || (stream_rate_hz == 0U))
    {
        return;
    }

    interval_ms = 1000U / stream_rate_hz;
    if ((uint32)(now_ms - vofa_last_stream_ms) < interval_ms)
    {
        return;
    }

    vofa_last_stream_ms = now_ms;
    speed_pid_get_status(&status);
    vofa_send_speed(&status);
}

/**
 * @brief Enable or disable foreground telemetry streaming.
 * @param enabled Nonzero enables streaming.
 */
void vofa_set_stream_enabled(uint8 enabled)
{
    uint32 primask = interrupt_global_disable();

    vofa_stream_enabled = enabled != 0U ? 1U : 0U;

    interrupt_global_enable(primask);
}

/**
 * @brief Copy one coherent VOFA parser and safety status snapshot.
 * @param stats Destination statistics structure.
 */
void vofa_get_stats(vofa_stats_struct *stats)
{
    uint32 primask;

    if (stats == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    stats->received_line_count = vofa_received_line_count;
    stats->valid_command_count = vofa_valid_command_count;
    stats->invalid_command_count = vofa_invalid_command_count;
    stats->rx_overflow_count = vofa_rx_overflow_count;
    stats->queue_drop_count = vofa_queue_drop_count;
    stats->timeout_stop_count = vofa_timeout_stop_count;
    stats->armed = vofa_armed;
    stats->stream_enabled = vofa_stream_enabled;
    stats->stream_rate_hz = vofa_stream_rate_hz;
    interrupt_global_enable(primask);
}
