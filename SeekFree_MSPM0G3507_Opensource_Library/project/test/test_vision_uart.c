/**
 * @file    test_vision_uart.c
 * @brief   UART1 DMA ASCII ball-position receiver TFT test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_VISION_UART)

#include "test_vision_uart.h"

#include "my_lib_ili9341.h"
#include "vision_uart.h"
#include "zf_driver_pit.h"

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
