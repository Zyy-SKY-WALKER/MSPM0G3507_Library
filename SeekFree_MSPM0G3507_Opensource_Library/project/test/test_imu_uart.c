/**
 * @file    test_imu_uart.c
 * @brief   DMA UART angle-frame TFT verification test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_IMU_UART)

#include "test_imu_uart.h"

#include "imu_uart.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define IMU_UART_TEST_UPDATE_TIME_MS     (100U)

/**
 * @brief Round a floating-point display value to a signed integer.
 * @param value Value to round.
 * @return Rounded signed integer.
 */
static int32 imu_uart_test_round(float value)
{
    if (value >= 0.0F)
    {
        return (int32)(value + 0.5F);
    }

    return (int32)(value - 0.5F);
}

/**
 * @brief Clear and display one signed status value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed value.
 * @param digits Magnitude field width.
 */
static void imu_uart_test_show_value(
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
 * @brief Clear and display one unsigned status value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Unsigned value.
 * @param digits Field width.
 */
static void imu_uart_test_show_uint(
    uint16 x,
    uint16 y,
    uint32 value,
    uint8 digits)
{
    uint16 field_width = (uint16)(digits * 8U);

    ili9341_fill_rect(
        x,
        y,
        (uint16)(x + field_width - 1U),
        (uint16)(y + 15U),
        ILI9341_COLOR_BLACK);
    ili9341_show_uint(x, y, value, digits);
}

/**
 * @brief Refresh the latest converted angle and DMA receive values.
 * @param data Latest IMU data snapshot.
 * @param frame_delta Valid angle frames decoded since the prior refresh.
 */
static void imu_uart_test_show_data(
    const imu_uart_data_struct *data,
    uint32 frame_delta)
{
    imu_uart_test_show_value(
        112U,
        40U,
        imu_uart_test_round(data->angle_deg[0]),
        4U);
    imu_uart_test_show_value(
        112U,
        72U,
        imu_uart_test_round(data->angle_deg[1]),
        4U);
    imu_uart_test_show_value(
        112U,
        104U,
        imu_uart_test_round(data->angle_deg[2]),
        4U);
    imu_uart_test_show_value(
        112U,
        136U,
        data->angle_valid,
        1U);
    imu_uart_test_show_uint(
        112U,
        168U,
        data->angle_frame_count,
        8U);
    imu_uart_test_show_uint(
        112U,
        200U,
        frame_delta,
        4U);
    imu_uart_test_show_uint(
        112U,
        232U,
        data->checksum_error_count,
        8U);
}

/**
 * @brief Initialize and run the UART attitude-module display test.
 */
void test_imu_uart_run(void)
{
    imu_uart_data_struct data;
    uint32 previous_frame_count = 0U;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "UART IMU DMA TEST");
    ili9341_show_string(8U, 40U, "ROLL   :");
    ili9341_show_string(8U, 72U, "PITCH  :");
    ili9341_show_string(8U, 104U, "YAW    :");
    ili9341_show_string(8U, 136U, "VALID  :");
    ili9341_show_string(8U, 168U, "ANG FRM:");
    ili9341_show_string(8U, 200U, "DELTA  :");
    ili9341_show_string(8U, 232U, "CRC ERR:");
    ili9341_show_string(8U, 264U, "DMA RX 0x55 0x53");
    ili9341_show_string(8U, 296U, "WAITING FOR DATA");

    imu_uart_init();

    while (true)
    {
        uint32 frame_delta;
        uint8 data_valid;

        imu_uart_update();
        data_valid = imu_uart_get_data(&data);
        frame_delta = data.angle_frame_count - previous_frame_count;
        previous_frame_count = data.angle_frame_count;
        imu_uart_test_show_data(&data, frame_delta);
        ili9341_fill_rect(
            8U,
            296U,
            231U,
            311U,
            ILI9341_COLOR_BLACK);

        if (data_valid != 0U)
        {
            ili9341_show_string(8U, 296U, "DATA OK");
        }
        else
        {
            ili9341_show_string(8U, 296U, "WAITING FOR DATA");
        }

        system_delay_ms(IMU_UART_TEST_UPDATE_TIME_MS);
    }
}

#endif
