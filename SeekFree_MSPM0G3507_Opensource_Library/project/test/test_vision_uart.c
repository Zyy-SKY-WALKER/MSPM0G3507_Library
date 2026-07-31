/**
 * @file    test_vision_uart.c
 * @brief   UART1 DMA ASCII vision and ball-position tests.
 */

#include "test_config.h"

#if ((TEST_MODE == TEST_MODE_VISION_UART) \
    || (TEST_MODE == TEST_MODE_BALL_VISION_OSCILLATION) \
    || (TEST_MODE == TEST_MODE_OLED_TASK_1))

#include "test_vision_uart.h"

#include "vision_uart.h"

#if (TEST_MODE == TEST_MODE_VISION_UART)
#include "my_lib_ili9341.h"
#include "zf_driver_pit.h"
#endif

#if (TEST_MODE == TEST_MODE_BALL_VISION_OSCILLATION)
#include "gimbal_stepper.h"
#include "ml_oled.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#endif

#if (TEST_MODE == TEST_MODE_OLED_TASK_1)
#include "ml_oled.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#endif

#if (TEST_MODE == TEST_MODE_VISION_UART)

#define VISION_TEST_TIMER             (PIT_TIM_G12)
#define VISION_TEST_DISPLAY_MS        (100U)
#define VISION_TEST_RATE_PERIOD_MS    (1000U)

/** @brief Values already written to the TFT. */
typedef struct
{
    int32 position_centi_cm;
    uint32 packet_count;
    uint32 valid_packet_count;
    uint32 invalid_packet_count;
    uint32 format_error_count;
    uint32 overflow_count;
    uint32 rate_hz;
    uint8 recognition_valid;
    uint8 initialized;
} vision_test_display_cache_struct;

static volatile uint32 vision_test_time_ms;

/**
 * @brief Advance the standalone test clock by one millisecond.
 */
static void vision_test_timer_callback(uint32 event, void *context)
{
    (void)event;
    (void)context;
    vision_test_time_ms++;
}

/**
 * @brief Clear and display one signed value.
 */
static void vision_test_show_int(
    uint16 x,
    uint16 y,
    int32 value,
    uint8 digits)
{
    uint16 width = (uint16)((digits + 1U) * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, value, digits);
}

/**
 * @brief Clear and display one unsigned value.
 */
static void vision_test_show_uint(
    uint16 x,
    uint16 y,
    uint32 value,
    uint8 digits)
{
    uint16 width = (uint16)(digits * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_uint(x, y, value, digits);
}

/**
 * @brief Refresh only TFT fields whose displayed value changed.
 */
static void vision_test_show_data(
    const vision_uart_data_struct *data,
    uint32 rate_hz,
    vision_test_display_cache_struct *cache)
{
    uint8 force = (uint8)(cache->initialized == 0U);

    if ((force != 0U)
        || (data->position_centi_cm != cache->position_centi_cm))
    {
        vision_test_show_int(112U, 40U, data->position_centi_cm, 10U);
        cache->position_centi_cm = data->position_centi_cm;
    }
    if ((force != 0U)
        || (data->recognition_valid != cache->recognition_valid))
    {
        vision_test_show_uint(
            112U,
            64U,
            data->recognition_valid,
            1U);
        cache->recognition_valid = data->recognition_valid;
    }
    if ((force != 0U) || (data->packet_count != cache->packet_count))
    {
        vision_test_show_uint(112U, 88U, data->packet_count, 8U);
        cache->packet_count = data->packet_count;
    }
    if ((force != 0U)
        || (data->valid_packet_count != cache->valid_packet_count))
    {
        vision_test_show_uint(
            112U,
            112U,
            data->valid_packet_count,
            8U);
        cache->valid_packet_count = data->valid_packet_count;
    }
    if ((force != 0U)
        || (data->invalid_packet_count != cache->invalid_packet_count))
    {
        vision_test_show_uint(
            112U,
            136U,
            data->invalid_packet_count,
            8U);
        cache->invalid_packet_count = data->invalid_packet_count;
    }
    if ((force != 0U)
        || (data->format_error_count != cache->format_error_count))
    {
        vision_test_show_uint(
            112U,
            160U,
            data->format_error_count,
            8U);
        cache->format_error_count = data->format_error_count;
    }
    if ((force != 0U)
        || (data->overflow_count != cache->overflow_count))
    {
        vision_test_show_uint(
            112U,
            184U,
            data->overflow_count,
            8U);
        cache->overflow_count = data->overflow_count;
    }
    if ((force != 0U) || (rate_hz != cache->rate_hz))
    {
        vision_test_show_uint(112U, 208U, rate_hz, 4U);
        cache->rate_hz = rate_hz;
    }
    cache->initialized = 1U;
}

/**
 * @brief Run the standalone vision receiver and TFT diagnostics.
 */
void test_vision_uart_run(void)
{
    static vision_test_display_cache_struct display_cache;
    vision_uart_data_struct data;
    uint32 last_display_ms = 0U;
    uint32 last_rate_ms = 0U;
    uint32 last_rate_packet_count = 0U;
    uint32 rate_hz = 0U;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "UART1 DMA VISION ASCII");
    ili9341_show_string(8U, 40U, "POS cCM    :");
    ili9341_show_string(8U, 64U, "VALID      :");
    ili9341_show_string(8U, 88U, "PACKETS    :");
    ili9341_show_string(8U, 112U, "VALID PKT  :");
    ili9341_show_string(8U, 136U, "INVALID PKT:");
    ili9341_show_string(8U, 160U, "FORMAT ERR :");
    ili9341_show_string(8U, 184U, "LINE OVR   :");
    ili9341_show_string(8U, 208U, "RATE HZ    :");
    ili9341_show_string(8U, 240U, "+05.23 LF / ! LF");
    ili9341_show_string(8U, 264U, "UART1 B7 RX / 115200");

    vision_test_time_ms = 0U;
    pit_ms_init(
        VISION_TEST_TIMER,
        1U,
        vision_test_timer_callback,
        NULL);
    if (vision_uart_init() == 0U)
    {
        ili9341_show_string(8U, 304U, "INIT FAILED");
        while (true)
        {
        }
    }

    while (true)
    {
        uint32 now_ms;

        vision_uart_update();
        now_ms = vision_test_time_ms;
        if ((uint32)(now_ms - last_rate_ms)
            >= VISION_TEST_RATE_PERIOD_MS)
        {
            uint32 elapsed_ms = now_ms - last_rate_ms;
            uint32 packet_delta;

            vision_uart_get_data(&data);
            packet_delta = data.packet_count - last_rate_packet_count;
            rate_hz = (uint32)(((uint64)packet_delta * 1000U)
                / elapsed_ms);
            last_rate_packet_count = data.packet_count;
            last_rate_ms = now_ms;
        }
        if ((uint32)(now_ms - last_display_ms)
            >= VISION_TEST_DISPLAY_MS)
        {
            vision_uart_get_data(&data);
            vision_test_show_data(&data, rate_hz, &display_cache);
            last_display_ms = now_ms;
        }
    }
}

#endif

#if (TEST_MODE == TEST_MODE_BALL_VISION_OSCILLATION)

#define BALL_VISION_MIN_POSITION_STEPS         (-51262)
#define BALL_VISION_MAX_POSITION_STEPS         (153786)
#define BALL_VISION_JOG_RATE_STEPS_S          (5000U)
#define BALL_VISION_LOOP_PERIOD_MS            (1U)
#define BALL_VISION_START_DEBOUNCE_MS         (10U)
#define BALL_VISION_START_LONG_PRESS_MS       (1000U)
#define BALL_VISION_TARGET_POS_CENTI_CM       (500)
#define BALL_VISION_TARGET_NEG_CENTI_CM       (-500)
#define BALL_VISION_TARGET_TOLERANCE_CENTI_CM (30)
#define BALL_VISION_FRAME_TIMEOUT_MS          (100U)
#define BALL_VISION_KP_MM_PER_CM              (20.0F)
#define BALL_VISION_KD_MM_PER_CM_S            (0.0F)
#define BALL_VISION_MAX_VELOCITY_CM_S         (100.0F)
#define BALL_VISION_VELOCITY_FILTER_ALPHA     (0.40F)
#define BALL_VISION_MIN_LIFT_MM                (-16.5F)
#define BALL_VISION_MAX_LIFT_MM                (49.5F)
#define BALL_VISION_LIFT_MM_PER_REVOLUTION    (2.06F)
#define BALL_VISION_TEXT_LENGTH               (13U)
#define BALL_VISION_STATUS_TEXT_LENGTH        (16U)

typedef enum
{
    BALL_VISION_WAIT_FIRST_ZERO = 0,
    BALL_VISION_WAIT_FINAL_ZERO,
    BALL_VISION_WAIT_START,
    BALL_VISION_RUNNING,
} ball_vision_state_enum;

typedef struct
{
    char position_characters[BALL_VISION_TEXT_LENGTH];
    char status_characters[BALL_VISION_STATUS_TEXT_LENGTH];
    uint8 position_initialized;
    uint8 status_initialized;
} ball_vision_display_cache_struct;

/**
 * @brief Clamp one scalar to a closed interval.
 */
static float ball_vision_clamp_float(float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

/**
 * @brief Build the fixed first-row text from the latest vision state.
 */
static void ball_vision_build_position_text(
    char text[BALL_VISION_TEXT_LENGTH],
    uint8 recognition_valid,
    int32 position_centi_cm)
{
    int32 magnitude = position_centi_cm;

    text[0] = 'B';
    text[1] = 'A';
    text[2] = 'L';
    text[3] = 'L';
    text[4] = ':';
    if(recognition_valid == 0U)
    {
        text[5] = '!';
        text[6] = ' ';
        text[7] = ' ';
        text[8] = ' ';
        text[9] = ' ';
        text[10] = ' ';
        text[11] = ' ';
        text[12] = ' ';
        return;
    }

    if(magnitude < 0)
    {
        magnitude = -magnitude;
        text[5] = '-';
    }
    else
    {
        text[5] = '+';
    }
    text[6] = (char)('0' + ((magnitude / 1000) % 10));
    text[7] = (char)('0' + ((magnitude / 100) % 10));
    text[8] = '.';
    text[9] = (char)('0' + ((magnitude / 10) % 10));
    text[10] = (char)('0' + (magnitude % 10));
    text[11] = 'c';
    text[12] = 'm';
}

/**
 * @brief Redraw only changed characters in one OLED text row.
 */
static void ball_vision_render_characters(
    const char text[],
    uint8 length,
    uint8 line,
    char cache[],
    uint8 *initialized)
{
    uint8 index;

    for(index = 0U; index < length; index++)
    {
        if((*initialized == 0U) || (text[index] != cache[index]))
        {
            (void)ml_oled_show_char(
                line,
                (uint8)(index + 1U),
                text[index]);
            cache[index] = text[index];
        }
    }
    *initialized = 1U;
}

/**
 * @brief Dirty-refresh OLED line one with the latest ball position.
 */
static void ball_vision_render_position(
    const vision_uart_data_struct *data,
    ball_vision_display_cache_struct *cache)
{
    char text[BALL_VISION_TEXT_LENGTH];

    ball_vision_build_position_text(
        text,
        data->recognition_valid,
        data->position_centi_cm);
    ball_vision_render_characters(
        text,
        BALL_VISION_TEXT_LENGTH,
        1U,
        cache->position_characters,
        &cache->position_initialized);
}

/**
 * @brief Dirty-refresh OLED line two with a space-padded task state.
 */
static void ball_vision_render_status(
    const char status[],
    ball_vision_display_cache_struct *cache)
{
    char text[BALL_VISION_STATUS_TEXT_LENGTH];
    uint8 index;
    uint8 finished = 0U;

    for(index = 0U; index < BALL_VISION_STATUS_TEXT_LENGTH; index++)
    {
        if(finished == 0U)
        {
            text[index] = status[index];
            if(text[index] == '\0')
            {
                text[index] = ' ';
                finished = 1U;
            }
        }
        else
        {
            text[index] = ' ';
        }
    }
    ball_vision_render_characters(
        text,
        BALL_VISION_STATUS_TEXT_LENGTH,
        2U,
        cache->status_characters,
        &cache->status_initialized);
}

/**
 * @brief Detect one debounced B0 short press after calibration ownership ends.
 */
static uint8 ball_vision_start_key_update(
    uint16 elapsed_ms,
    uint8 *pressed,
    uint16 *hold_ms)
{
    uint8 key_pressed = (uint8)(gpio_get_level(B0) == GPIO_LOW);

    if(key_pressed != 0U)
    {
        if(*pressed == 0U)
        {
            *pressed = 1U;
            *hold_ms = 0U;
        }
        if(*hold_ms < BALL_VISION_START_LONG_PRESS_MS)
        {
            *hold_ms = (uint16)(*hold_ms + elapsed_ms);
            if(*hold_ms > BALL_VISION_START_LONG_PRESS_MS)
            {
                *hold_ms = BALL_VISION_START_LONG_PRESS_MS;
            }
        }
        return 0U;
    }

    if(*pressed != 0U)
    {
        uint16 completed_hold_ms = *hold_ms;

        *pressed = 0U;
        *hold_ms = 0U;
        return (uint8)((completed_hold_ms >= BALL_VISION_START_DEBOUNCE_MS)
            && (completed_hold_ms < BALL_VISION_START_LONG_PRESS_MS));
    }
    return 0U;
}

/**
 * @brief Convert one bounded PD lift command into the single-axis target.
 */
static int32 ball_vision_lift_to_steps(float lift_mm)
{
    float steps;

    lift_mm = ball_vision_clamp_float(
        lift_mm,
        BALL_VISION_MIN_LIFT_MM,
        BALL_VISION_MAX_LIFT_MM);
    steps = lift_mm * (float)GIMBAL_STEPPER_STEPS_PER_REVOLUTION
        / BALL_VISION_LIFT_MM_PER_REVOLUTION;
    return steps >= 0.0F
        ? (int32)(steps + 0.5F)
        : (int32)(steps - 0.5F);
}

/**
 * @brief Select the concise status displayed in the second OLED row.
 */
static const char *ball_vision_get_status_text(
    ball_vision_state_enum state,
    int32 target_centi_cm,
    uint8 vision_available,
    uint8 rezero_required)
{
    if(state == BALL_VISION_WAIT_FIRST_ZERO)
    {
        return rezero_required != 0U
            ? "STATE:REZERO A30" : "STATE:ZERO TEMP";
    }
    if(state == BALL_VISION_WAIT_FINAL_ZERO)
    {
        return "STATE:LEVEL B0/B1";
    }
    if(state == BALL_VISION_WAIT_START)
    {
        return "STATE:PRESS B0";
    }
    if(vision_available == 0U)
    {
        return "STATE:V LOST";
    }
    return target_centi_cm > 0
        ? "STATE:RUN TO +5" : "STATE:RUN TO -5";
}

/**
 * @brief Run stationary visual PD control between +5 cm and -5 cm.
 */
void test_ball_vision_oscillation_run(void)
{
    ball_vision_display_cache_struct display_cache = {{0}, {0}, 0U, 0U};
    vision_uart_data_struct vision_data;
    gimbal_stepper_status_struct stepper_status;
    ball_vision_state_enum state = BALL_VISION_WAIT_FIRST_ZERO;
    uint32 time_ms = 0U;
    uint32 last_packet_count = 0U;
    uint32 last_valid_frame_ms = 0U;
    uint32 previous_frame_ms = 0U;
    int32 target_centi_cm = BALL_VISION_TARGET_POS_CENTI_CM;
    int32 previous_position_centi_cm = 0;
    float filtered_velocity_cm_s = 0.0F;
    uint16 start_hold_ms = 0U;
    uint8 start_key_pressed = 0U;
    uint8 previous_position_valid = 0U;
    uint8 rezero_required = 0U;

    if(ml_oled_init() == false)
    {
        while(true)
        {
        }
    }
    vision_data.position_centi_cm = 0;
    vision_data.recognition_valid = 0U;
    ball_vision_render_position(&vision_data, &display_cache);
    ball_vision_render_status("STATE:INIT", &display_cache);

    gimbal_stepper_init();
    if(vision_uart_init() == 0U)
    {
        ball_vision_render_status("STATE:UART ERROR", &display_cache);
        while(true)
        {
        }
    }
    if(gimbal_stepper_configure_single_axis(
            GIMBAL_STEPPER_AXIS_YAW,
            BALL_VISION_MIN_POSITION_STEPS,
            BALL_VISION_MAX_POSITION_STEPS,
            BALL_VISION_JOG_RATE_STEPS_S) == 0U)
    {
        ball_vision_render_status("STATE:PWM ERROR", &display_cache);
        while(true)
        {
            vision_uart_update();
            (void)vision_uart_get_data(&vision_data);
            ball_vision_render_position(&vision_data, &display_cache);
            system_delay_ms(BALL_VISION_LOOP_PERIOD_MS);
        }
    }

    while(true)
    {
        uint16 elapsed_ms = gimbal_stepper_service();

        time_ms += elapsed_ms;
        vision_uart_update();
        (void)vision_uart_get_data(&vision_data);
        ball_vision_render_position(&vision_data, &display_cache);
        gimbal_stepper_get_status(&stepper_status);

        if((stepper_status.axis[GIMBAL_STEPPER_AXIS_YAW]
            .zero_valid == 0U)
            && (state != BALL_VISION_WAIT_FIRST_ZERO))
        {
            gimbal_stepper_set_manual_control_enabled(1U);
            previous_position_valid = 0U;
            filtered_velocity_cm_s = 0.0F;
            start_key_pressed = 0U;
            start_hold_ms = 0U;
            rezero_required = 1U;
            state = BALL_VISION_WAIT_FIRST_ZERO;
        }

        if(state == BALL_VISION_WAIT_FIRST_ZERO)
        {
            if(stepper_status.axis[GIMBAL_STEPPER_AXIS_YAW]
                .zero_capture_count >= 1U)
            {
                rezero_required = 0U;
                state = BALL_VISION_WAIT_FINAL_ZERO;
            }
        }
        else if(state == BALL_VISION_WAIT_FINAL_ZERO)
        {
            if((stepper_status.axis[GIMBAL_STEPPER_AXIS_YAW]
                .zero_capture_count >= 2U)
                && (stepper_status.relative_ready != 0U))
            {
                gimbal_stepper_set_manual_control_enabled(0U);
                start_key_pressed = 0U;
                start_hold_ms = 0U;
                state = BALL_VISION_WAIT_START;
            }
        }
        else if(state == BALL_VISION_WAIT_START)
        {
            if(ball_vision_start_key_update(
                    elapsed_ms,
                    &start_key_pressed,
                    &start_hold_ms) != 0U)
            {
                target_centi_cm = BALL_VISION_TARGET_POS_CENTI_CM;
                previous_position_valid = 0U;
                filtered_velocity_cm_s = 0.0F;
                state = BALL_VISION_RUNNING;
            }
        }
        else if(vision_data.packet_count != last_packet_count)
        {
            last_packet_count = vision_data.packet_count;
            if(vision_data.recognition_valid != 0U)
            {
                float velocity_cm_s = 0.0F;
                float error_cm;
                float lift_mm;

                last_valid_frame_ms = time_ms;
                if(previous_position_valid != 0U)
                {
                    uint32 frame_period_ms = time_ms - previous_frame_ms;

                    if((frame_period_ms > 0U)
                        && (frame_period_ms <= BALL_VISION_FRAME_TIMEOUT_MS))
                    {
                        velocity_cm_s = ((float)(
                            vision_data.position_centi_cm
                            - previous_position_centi_cm) * 10.0F)
                            / (float)frame_period_ms;
                        velocity_cm_s = ball_vision_clamp_float(
                            velocity_cm_s,
                            -BALL_VISION_MAX_VELOCITY_CM_S,
                            BALL_VISION_MAX_VELOCITY_CM_S);
                        filtered_velocity_cm_s +=
                            BALL_VISION_VELOCITY_FILTER_ALPHA
                            * (velocity_cm_s - filtered_velocity_cm_s);
                    }
                    else
                    {
                        filtered_velocity_cm_s = 0.0F;
                    }
                }
                else
                {
                    filtered_velocity_cm_s = 0.0F;
                }

                if((target_centi_cm > 0)
                    && (vision_data.position_centi_cm >=
                        (BALL_VISION_TARGET_POS_CENTI_CM
                            - BALL_VISION_TARGET_TOLERANCE_CENTI_CM)))
                {
                    target_centi_cm = BALL_VISION_TARGET_NEG_CENTI_CM;
                }
                else if((target_centi_cm < 0)
                    && (vision_data.position_centi_cm <=
                        (BALL_VISION_TARGET_NEG_CENTI_CM
                            + BALL_VISION_TARGET_TOLERANCE_CENTI_CM)))
                {
                    target_centi_cm = BALL_VISION_TARGET_POS_CENTI_CM;
                }

                error_cm = (float)(target_centi_cm
                    - vision_data.position_centi_cm) / 100.0F;
                lift_mm = (BALL_VISION_KP_MM_PER_CM * error_cm)
                    - (BALL_VISION_KD_MM_PER_CM_S
                        * filtered_velocity_cm_s);
                (void)gimbal_stepper_set_axis_absolute_target_steps(
                    GIMBAL_STEPPER_AXIS_YAW,
                    ball_vision_lift_to_steps(lift_mm));
                previous_position_centi_cm = vision_data.position_centi_cm;
                previous_frame_ms = time_ms;
                previous_position_valid = 1U;
            }
            else
            {
                previous_position_valid = 0U;
                filtered_velocity_cm_s = 0.0F;
            }
        }

        if((previous_position_valid != 0U)
            && ((uint32)(time_ms - last_valid_frame_ms)
                > BALL_VISION_FRAME_TIMEOUT_MS))
        {
            previous_position_valid = 0U;
            filtered_velocity_cm_s = 0.0F;
        }
        ball_vision_render_status(
            ball_vision_get_status_text(
                state,
                target_centi_cm,
                previous_position_valid,
                rezero_required),
            &display_cache);
        system_delay_ms(BALL_VISION_LOOP_PERIOD_MS);
    }
}

#endif

#if (TEST_MODE == TEST_MODE_OLED_TASK_1)

#define OLED_TASK_1_TIMER                    (PIT_TIM_G12)
#define OLED_TASK_1_TICK_US                  (10000U)
#define OLED_TASK_1_DEBOUNCE_MS              (20U)
#define OLED_TASK_1_TEXT_LENGTH              (13U)
#define OLED_TASK_1_DEBUG_FIRST_LINE         (2U)
#define OLED_TASK_1_DEBUG_LAST_LINE          (4U)

static volatile uint32 oled_task_1_elapsed_10ms;
static volatile uint8 oled_task_1_running;

/**
 * @brief Advance the stopwatch from a 10 ms hardware timer tick.
 */
static void oled_task_1_timer_callback(uint32 event, void *context)
{
    (void)event;
    (void)context;

    if(oled_task_1_running != 0U)
    {
        oled_task_1_elapsed_10ms++;
    }
}

/**
 * @brief Build the fixed-width first-row stopwatch text.
 */
static void oled_task_1_build_text(
    char text[OLED_TASK_1_TEXT_LENGTH],
    uint8 running,
    uint32 elapsed_tenths)
{
    uint32 seconds = elapsed_tenths / 10U;

    text[0] = 'T';
    text[1] = 'I';
    text[2] = 'M';
    text[3] = 'E';
    text[4] = ':';
    if(running == 0U)
    {
        text[5] = 'W';
        text[6] = 'A';
        text[7] = 'I';
        text[8] = 'T';
        text[9] = ' ';
        text[10] = ' ';
        text[11] = ' ';
        text[12] = ' ';
        return;
    }

    seconds %= 100000U;
    text[5] = (char)('0' + ((seconds / 10000U) % 10U));
    text[6] = (char)('0' + ((seconds / 1000U) % 10U));
    text[7] = (char)('0' + ((seconds / 100U) % 10U));
    text[8] = (char)('0' + ((seconds / 10U) % 10U));
    text[9] = (char)('0' + (seconds % 10U));
    text[10] = '.';
    text[11] = (char)('0' + (elapsed_tenths % 10U));
    text[12] = 's';
}

/**
 * @brief Redraw only changed characters in OLED line one.
 */
static void oled_task_1_render_time(
    uint8 running,
    uint32 elapsed_tenths,
    char cache[OLED_TASK_1_TEXT_LENGTH],
    uint8 *cache_valid)
{
    char text[OLED_TASK_1_TEXT_LENGTH];
    uint8 index;

    oled_task_1_build_text(text, running, elapsed_tenths);
    for(index = 0U; index < OLED_TASK_1_TEXT_LENGTH; index++)
    {
        if((*cache_valid == 0U) || (text[index] != cache[index]))
        {
            (void)ml_oled_show_char(1U, (uint8)(index + 1U), text[index]);
            cache[index] = text[index];
        }
    }
    *cache_valid = 1U;
}

/**
 * @brief Run OLED task one: A30 starts a 0.1-second stopwatch.
 * @note OLED lines 2 through 4 are intentionally reserved for diagnostics.
 */
void test_oled_task_1_run(void)
{
    char display_cache[OLED_TASK_1_TEXT_LENGTH] = {0};
    uint8 display_cache_valid = 0U;
    uint16 a30_low_ms = 0U;

    if(ml_oled_init() == false)
    {
        while(true)
        {
        }
    }
    gpio_init(A30, GPI, GPIO_HIGH, GPI_PULL_UP);
    oled_task_1_elapsed_10ms = 0U;
    oled_task_1_running = 0U;
    pit_us_init(
        OLED_TASK_1_TIMER,
        OLED_TASK_1_TICK_US,
        oled_task_1_timer_callback,
        NULL);

    while(true)
    {
        uint8 started_snapshot;
        uint32 elapsed_snapshot;
        uint32 primask;

        if((gpio_get_level(A30) == GPIO_LOW)
            && (oled_task_1_running == 0U))
        {
            if(a30_low_ms < OLED_TASK_1_DEBOUNCE_MS)
            {
                a30_low_ms++;
            }
            if(a30_low_ms >= OLED_TASK_1_DEBOUNCE_MS)
            {
                primask = interrupt_global_disable();
                oled_task_1_elapsed_10ms = 0U;
                oled_task_1_running = 1U;
                interrupt_global_enable(primask);
            }
        }
        else if(gpio_get_level(A30) != GPIO_LOW)
        {
            a30_low_ms = 0U;
        }

        primask = interrupt_global_disable();
        started_snapshot = oled_task_1_running;
        elapsed_snapshot = oled_task_1_elapsed_10ms / 10U;
        interrupt_global_enable(primask);
        oled_task_1_render_time(
            started_snapshot,
            elapsed_snapshot,
            display_cache,
            &display_cache_valid);
        system_delay_ms(1U);
    }
}

#endif

#endif
