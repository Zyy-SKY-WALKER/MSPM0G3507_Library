/**
 * @file    test_encoder.c
 * @brief   Dual motor encoder count display test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ENCODER)

#include <string.h>

#include "test_encoder.h"

#include "drive_geometry.h"
#include "my_lib_encoder.h"
#include "my_lib_ili9341.h"
#include "vofa.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#define ENCODER_TEST_REFRESH_DELAY_MS    (100U)
#define ENCODER_TEST_VOFA_CHANNEL_COUNT  (2U)
#define ENCODER_TEST_VOFA_FRAME_SIZE     (12U)

/* VOFA JustFloat channel order: 0 left total, 1 right total. */

/**
 * @brief Display one GPIO phase level as H or L.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param level GPIO direction level.
 */
static void test_encoder_show_level(uint16 x, uint16 y, uint8 level)
{
    if (level == GPIO_HIGH)
    {
        ili9341_show_char(x, y, 'H');
    }
    else
    {
        ili9341_show_char(x, y, 'L');
    }
}

/**
 * @brief Clear and display one signed encoder count.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param count Signed encoder count.
 */
static void test_encoder_show_count(
    uint16 x,
    uint16 y,
    int32 count,
    uint8 digits)
{
    uint16 field_width = (uint16)((digits + 1U) * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, count, digits);
}

/**
 * @brief Send cumulative left and right encoder counts through VOFA.
 * @param left_total Cumulative signed left encoder count.
 * @param right_total Cumulative signed right encoder count.
 */
static void test_encoder_send_vofa(int32 left_total, int32 right_total)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float channels[ENCODER_TEST_VOFA_CHANNEL_COUNT];
    uint8 frame[ENCODER_TEST_VOFA_FRAME_SIZE];

    channels[0] = (float)left_total;
    channels[1] = (float)right_total;
    memcpy(frame, channels, sizeof(channels));
    memcpy(&frame[sizeof(channels)], tail, sizeof(tail));
    uart_write_buffer(VOFA_UART_INDEX, frame, sizeof(frame));
}

/**
 * @brief Initialize the display and show live dual encoder counts.
 * @note Rotate each raised wheel by hand. Record count signs when the vehicle
 *       moves forward before integrating PID or odometry code.
 */
void test_encoder_run(void)
{
    int32 left_total = 0;
    int32 right_total = 0;
    int16 left_delta;
    int16 right_delta;
    uint8 left_phase_a;
    uint8 left_phase_b;
    uint8 right_phase_a;
    uint8 right_phase_b;
    uint32 right_invalid_transition_count;

    ili9341_init();
    my_encoder_init();

    if (vofa_init_tx_only() == ZF_FALSE)
    {
        while (true)
        {
        }
    }

    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "ENCODER TEST");
    ili9341_show_string(8U, 40U, "L TOTAL:");
    ili9341_show_string(8U, 72U, "R TOTAL:");
    ili9341_show_string(8U, 104U, "L DELTA:");
    ili9341_show_string(8U, 136U, "R DELTA:");
    ili9341_show_string(8U, 168U, "L A/B  :");
    ili9341_show_string(8U, 200U, "R A/B  :");
    ili9341_show_string(8U, 240U, "TURN ONE WHEEL");
    ili9341_show_string(8U, 264U, "ONE REV CNT:");
    ili9341_show_int(
        104U,
        264U,
        (int32)DRIVE_ENCODER_COUNTS_PER_REV,
        5U);
    ili9341_show_string(8U, 288U, "R INVALID:");

    my_encoder_clear_count();
    while (true)
    {
        my_encoder_get_delta(&left_delta, &right_delta);
        left_total += left_delta;
        right_total += right_delta;
        test_encoder_send_vofa(left_total, right_total);
        left_phase_a = my_encoder_get_left_phase_a();
        left_phase_b = my_encoder_get_left_phase_b();
        right_phase_a = my_encoder_get_right_phase_a();
        right_phase_b = my_encoder_get_right_phase_b();
        right_invalid_transition_count =
            my_encoder_get_right_invalid_transition_count();

        test_encoder_show_count(
            96U,
            40U,
            left_total,
            8U);
        test_encoder_show_count(
            96U,
            72U,
            right_total,
            8U);
        test_encoder_show_count(96U, 104U, left_delta, 6U);
        test_encoder_show_count(96U, 136U, right_delta, 6U);
        test_encoder_show_level(
            96U,
            168U,
            left_phase_a);
        test_encoder_show_level(
            120U,
            168U,
            left_phase_b);
        test_encoder_show_level(
            96U,
            200U,
            right_phase_a);
        test_encoder_show_level(
            120U,
            200U,
            right_phase_b);
        test_encoder_show_count(
            96U,
            288U,
            right_invalid_transition_count,
            8U);
        system_delay_ms(ENCODER_TEST_REFRESH_DELAY_MS);
    }
}

#endif
