/**
 * @file    test_speed_pid.c
 * @brief   PIT-driven dual-wheel speed PID verification test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_SPEED_PID)

#include "test_speed_pid.h"

#include "my_lib_encoder.h"
#include "my_lib_ili9341.h"
#include "speed_pid.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"

#define SPEED_PID_TEST_PIT               (PIT_TIM_G12)
#define SPEED_PID_TEST_DISPLAY_TIME_MS   (100U)
#define SPEED_PID_TEST_ARM_TIME_MS       (3000U)

typedef struct
{
    float left_target_mm_s;
    float right_target_mm_s;
    uint32 duration_ms;
} speed_pid_test_step_struct;

static const speed_pid_test_step_struct speed_pid_test_steps[] =
{
    {0.0F, 0.0F, 1000U},
    {100.0F, 100.0F, 3000U},
    {200.0F, 200.0F, 3000U},
    {0.0F, 0.0F, 1500U},
    {-100.0F, -100.0F, 3000U},
    {0.0F, 0.0F, 1000U},
};

static volatile uint32 speed_pid_test_elapsed_ms;

/**
 * @brief Execute one speed control update from the 10 ms PIT interrupt.
 * @param event PIT callback event value.
 * @param user_data Optional callback context.
 */
static void speed_pid_test_pit_callback(uint32 event, void *user_data)
{
    int16 left_count;
    int16 right_count;

    (void)event;
    (void)user_data;

    my_encoder_get_delta(&left_count, &right_count);
    speed_pid_update_10ms(left_count, right_count);
    speed_pid_test_elapsed_ms += SPEED_PID_SAMPLE_PERIOD_MS;
}

/**
 * @brief Round a floating-point display value to a signed integer.
 * @param value Value to round.
 * @return Rounded signed integer.
 */
static int32 speed_pid_test_round(float value)
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
static void speed_pid_test_show_value(
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
 * @brief Refresh the foreground PID status page.
 */
static void speed_pid_test_show_status(void)
{
    speed_pid_status_struct status;

    speed_pid_get_status(&status);
    speed_pid_test_show_value(
        96U,
        40U,
        speed_pid_test_round(status.left_target_mm_s),
        4U);
    speed_pid_test_show_value(
        96U,
        64U,
        speed_pid_test_round(status.right_target_mm_s),
        4U);
    speed_pid_test_show_value(
        96U,
        88U,
        speed_pid_test_round(status.left_speed_mm_s),
        4U);
    speed_pid_test_show_value(
        96U,
        112U,
        speed_pid_test_round(status.right_speed_mm_s),
        4U);
    speed_pid_test_show_value(96U, 136U, status.left_count, 5U);
    speed_pid_test_show_value(96U, 160U, status.right_count, 5U);
    speed_pid_test_show_value(96U, 184U, status.left_duty, 4U);
    speed_pid_test_show_value(96U, 208U, status.right_duty, 4U);

    ili9341_show_char(
        96U,
        232U,
        status.left_saturated != 0U ? 'Y' : 'N');
    ili9341_show_char(
        120U,
        232U,
        status.right_saturated != 0U ? 'Y' : 'N');
    ili9341_show_char(
        96U,
        256U,
        status.left_reversing != 0U ? 'Y' : 'N');
    ili9341_show_char(
        120U,
        256U,
        status.right_reversing != 0U ? 'Y' : 'N');
}

/**
 * @brief Hold one target pair while updating the foreground display.
 * @param step Test sequence step.
 */
static void speed_pid_test_run_step(
    const speed_pid_test_step_struct *step)
{
    uint32 start_ms = speed_pid_test_elapsed_ms;

    speed_pid_set_target(
        step->left_target_mm_s,
        step->right_target_mm_s);

    while ((uint32)(speed_pid_test_elapsed_ms - start_ms)
        < step->duration_ms)
    {
        speed_pid_test_show_status();
        system_delay_ms(SPEED_PID_TEST_DISPLAY_TIME_MS);
    }
}

/**
 * @brief Run the conservative dual-wheel speed PID sequence.
 * @note Raise both wheels before selecting TEST_MODE_SPEED_PID.
 */
void test_speed_pid_run(void)
{
    uint32 index;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "SPEED PID TEST");
    ili9341_show_string(8U, 40U, "L TGT :");
    ili9341_show_string(8U, 64U, "R TGT :");
    ili9341_show_string(8U, 88U, "L SPD :");
    ili9341_show_string(8U, 112U, "R SPD :");
    ili9341_show_string(8U, 136U, "L CNT :");
    ili9341_show_string(8U, 160U, "R CNT :");
    ili9341_show_string(8U, 184U, "L PWM :");
    ili9341_show_string(8U, 208U, "R PWM :");
    ili9341_show_string(8U, 232U, "SAT L/R:");
    ili9341_show_string(8U, 256U, "REV L/R:");
    ili9341_show_string(8U, 288U, "LIFT WHEELS");

    my_encoder_init();
    speed_pid_init();
    pit_ms_init(
        SPEED_PID_TEST_PIT,
        SPEED_PID_SAMPLE_PERIOD_MS,
        speed_pid_test_pit_callback,
        NULL);

    system_delay_ms(SPEED_PID_TEST_ARM_TIME_MS);
    ili9341_fill_rect(
        8U,
        288U,
        231U,
        303U,
        ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 288U, "RUNNING");

    for (index = 0U;
        index < (sizeof(speed_pid_test_steps)
            / sizeof(speed_pid_test_steps[0]));
        index++)
    {
        speed_pid_test_run_step(&speed_pid_test_steps[index]);
    }

    speed_pid_stop();
    ili9341_fill_rect(
        8U,
        288U,
        231U,
        303U,
        ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 288U, "TEST COMPLETE");

    while (true)
    {
        speed_pid_test_show_status();
        system_delay_ms(SPEED_PID_TEST_DISPLAY_TIME_MS);
    }
}

#endif
