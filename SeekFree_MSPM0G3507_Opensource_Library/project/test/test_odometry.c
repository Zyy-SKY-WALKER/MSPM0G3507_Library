/**
 * @file    test_odometry.c
 * @brief   Encoder and IMU fusion odometry TFT test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ODOMETRY)

#include "test_odometry.h"

#include "drive_geometry.h"
#include "imu_uart.h"
#include "my_lib_encoder.h"
#include "my_lib_ili9341.h"
#include "odometry.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"

#define ODOMETRY_TEST_PIT                (PIT_TIM_G12)
#define ODOMETRY_TEST_SAMPLE_TIME_MS     (10U)
#define ODOMETRY_TEST_DISPLAY_TIME_MS    (100U)
#define ODOMETRY_RAD_TO_DEG              (180.0F / DRIVE_PI)
#define ODOMETRY_TEST_PIT_PRIORITY       (1U)
#define ODOMETRY_TEST_ENCODER_PRIORITY   (0U)

/**
 * @brief Sample both encoders and update fused odometry every 10 ms.
 * @param event PIT callback event value.
 * @param user_data Optional callback context.
 */
static void odometry_test_pit_callback(uint32 event, void *user_data)
{
    uint32 yaw_frame_count = 0U;
    float yaw_deg = 0.0F;
    int16 left_count;
    int16 right_count;
    uint8 yaw_valid;

    (void)event;
    (void)user_data;

    my_encoder_get_delta(&left_count, &right_count);
    imu_uart_update();
    yaw_valid = imu_uart_get_yaw(&yaw_deg, &yaw_frame_count);
    odometry_update(
        left_count,
        right_count,
        yaw_valid,
        yaw_deg,
        yaw_frame_count);
}

/**
 * @brief Round a floating-point display value to a signed integer.
 * @param value Value to round.
 * @return Rounded signed integer.
 */
static int32 odometry_test_round(float value)
{
    if (value >= 0.0F)
    {
        return (int32)(value + 0.5F);
    }

    return (int32)(value - 0.5F);
}

/**
 * @brief Clear and display one signed state value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed display value.
 * @param digits Magnitude field width.
 */
static void odometry_test_show_value(
    uint16 x,
    uint16 y,
    int32 value,
    uint8 digits)
{
    uint16 field_width = (uint16)((digits + 1U) * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_int(x, y, value, digits);
}

/**
 * @brief Refresh the current fused odometry state.
 */
static void odometry_test_show_state(void)
{
    odometry_state_struct state;

    odometry_get_state(&state);
    odometry_test_show_value(
        104U,
        40U,
        odometry_test_round(state.x_mm),
        6U);
    odometry_test_show_value(
        104U,
        72U,
        odometry_test_round(state.y_mm),
        6U);
    odometry_test_show_value(
        104U,
        104U,
        odometry_test_round(state.theta_rad * ODOMETRY_RAD_TO_DEG),
        5U);
    odometry_test_show_value(
        104U,
        136U,
        odometry_test_round(state.left_distance_mm),
        6U);
    odometry_test_show_value(
        104U,
        168U,
        odometry_test_round(state.right_distance_mm),
        6U);
    odometry_test_show_value(
        104U,
        200U,
        odometry_test_round(state.center_displacement_mm),
        6U);
    odometry_test_show_value(
        104U,
        232U,
        odometry_test_round(state.path_length_mm),
        6U);
    odometry_test_show_value(
        104U,
        264U,
        odometry_test_round(state.imu_yaw_deg),
        5U);
    ili9341_show_char(
        104U,
        288U,
        state.imu_valid != 0U ? 'Y' : 'N');
}

/**
 * @brief Initialize and run the manual fused-odometry test.
 */
void test_odometry_run(void)
{
    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "ODOMETRY TEST");
    ili9341_show_string(8U, 40U, "X MM  :");
    ili9341_show_string(8U, 72U, "Y MM  :");
    ili9341_show_string(8U, 104U, "TH DEG:");
    ili9341_show_string(8U, 136U, "LEFT  :");
    ili9341_show_string(8U, 168U, "RIGHT :");
    ili9341_show_string(8U, 200U, "CENTER:");
    ili9341_show_string(8U, 232U, "PATH  :");
    ili9341_show_string(8U, 264U, "IMU Y :");
    ili9341_show_string(8U, 288U, "IMU OK:");

    interrupt_set_priority(
        TIMG12_INT_IRQn,
        ODOMETRY_TEST_PIT_PRIORITY);
    interrupt_set_priority(
        GPIOA_INT_IRQn,
        ODOMETRY_TEST_ENCODER_PRIORITY);
    my_encoder_init();
    imu_uart_init();
    odometry_init();
    pit_ms_init(
        ODOMETRY_TEST_PIT,
        ODOMETRY_TEST_SAMPLE_TIME_MS,
        odometry_test_pit_callback,
        NULL);

    while (true)
    {
        odometry_test_show_state();
        system_delay_ms(ODOMETRY_TEST_DISPLAY_TIME_MS);
    }
}

#endif
