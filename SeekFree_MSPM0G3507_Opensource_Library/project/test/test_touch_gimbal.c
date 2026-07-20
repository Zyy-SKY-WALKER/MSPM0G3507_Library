/**
 * @file    test_touch_gimbal.c
 * @brief   Relative-angle touch control for the dual-axis gimbal.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_TOUCH_GIMBAL)

#include "test_touch_gimbal.h"

#include "gimbal_stepper.h"
#include "my_lib_ili9341.h"
#include "my_lib_xpt2046.h"
#include "zf_driver_delay.h"

#define TOUCH_GIMBAL_LOOP_PERIOD_MS             (1U)
#define TOUCH_GIMBAL_SAMPLE_PERIOD_MS           (10U)
#define TOUCH_GIMBAL_UI_PERIOD_MS               (100U)
#define TOUCH_GIMBAL_POINT_RADIUS               (2U)
#define TOUCH_GIMBAL_PIXEL_DEAD_ZONE            (2)
#define TOUCH_GIMBAL_MAX_SAMPLE_DELTA            (40)

#define TOUCH_GIMBAL_YAW_SWIPE_STEPS            (6400)
#define TOUCH_GIMBAL_PITCH_SWIPE_STEPS          (3200)
#define TOUCH_GIMBAL_YAW_PIXEL_SPAN             \
    ((int32)XPT2046_SCREEN_HEIGHT - 1)
#define TOUCH_GIMBAL_PITCH_PIXEL_SPAN           \
    ((int32)XPT2046_SCREEN_WIDTH - 1)
#define TOUCH_GIMBAL_PITCH_DIRECTION_SIGN       (1)

typedef struct
{
    uint16 last_x;
    uint16 last_y;
    int16 pending_x;
    int16 pending_y;
    int32 yaw_remainder;
    int32 pitch_remainder;
    uint8 active;
} touch_gimbal_gesture_struct;

static touch_gimbal_gesture_struct touch_gimbal_gesture;
static uint16 touch_gimbal_last_x;
static uint16 touch_gimbal_last_y;
static uint8 touch_gimbal_touch_valid;

/**
 * @brief Return the unsigned magnitude of one signed pixel delta.
 */
static uint16 touch_gimbal_magnitude(int16 value)
{
    return value < 0
        ? (uint16)(-(int32)value)
        : (uint16)value;
}

/**
 * @brief End the current gesture and discard fractional movement.
 */
static void touch_gimbal_reset_gesture(void)
{
    touch_gimbal_gesture.last_x = 0U;
    touch_gimbal_gesture.last_y = 0U;
    touch_gimbal_gesture.pending_x = 0;
    touch_gimbal_gesture.pending_y = 0;
    touch_gimbal_gesture.yaw_remainder = 0;
    touch_gimbal_gesture.pitch_remainder = 0;
    touch_gimbal_gesture.active = 0U;
}

/**
 * @brief Convert signed pixels to steps while preserving division residue.
 */
static int32 touch_gimbal_pixels_to_steps(
    int16 pixels,
    int32 swipe_steps,
    int32 pixel_span,
    int32 *remainder)
{
    int32 numerator;
    int32 steps;

    numerator = *remainder + ((int32)pixels * swipe_steps);
    steps = numerator / pixel_span;
    *remainder = numerator - (steps * pixel_span);
    return steps;
}

/**
 * @brief Draw one bounded yellow touch marker.
 */
static void touch_gimbal_draw_point(uint16 x, uint16 y)
{
    uint16 x_start;
    uint16 y_start;
    uint16 x_end;
    uint16 y_end;

    x_start = x > TOUCH_GIMBAL_POINT_RADIUS
        ? (uint16)(x - TOUCH_GIMBAL_POINT_RADIUS) : 0U;
    y_start = y > TOUCH_GIMBAL_POINT_RADIUS
        ? (uint16)(y - TOUCH_GIMBAL_POINT_RADIUS) : 0U;
    x_end = (uint16)(x + TOUCH_GIMBAL_POINT_RADIUS);
    y_end = (uint16)(y + TOUCH_GIMBAL_POINT_RADIUS);

    if(x_end >= XPT2046_SCREEN_WIDTH)
    {
        x_end = XPT2046_SCREEN_WIDTH - 1U;
    }
    if(y_end >= XPT2046_SCREEN_HEIGHT)
    {
        y_end = XPT2046_SCREEN_HEIGHT - 1U;
    }

    ili9341_fill_rect(
        x_start,
        y_start,
        x_end,
        y_end,
        ILI9341_COLOR_YELLOW);
}

/**
 * @brief Draw one signed fixed-width value after clearing its field.
 */
static void touch_gimbal_show_signed(
    uint16 x,
    uint16 y,
    int32 value)
{
    ili9341_show_string(x, y, "      ");
    ili9341_show_int(x, y, value, 5U);
}

/**
 * @brief Draw static labels for the touch gimbal diagnostic screen.
 */
static void touch_gimbal_draw_layout(void)
{
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
}

/**
 * @brief Redraw labels that may be crossed by a yellow touch marker.
 */
static void touch_gimbal_draw_labels(void)
{
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "TOUCH ANGLE GIMBAL");
    ili9341_show_string(8U, 32U, "Y P:");
    ili9341_show_string(88U, 32U, "T:");
    ili9341_show_string(8U, 56U, "P P:");
    ili9341_show_string(88U, 56U, "T:");
    ili9341_show_string(8U, 80U, "ZERO Y:");
    ili9341_show_string(96U, 80U, "P:");
    ili9341_show_string(136U, 80U, "SEL:");
    ili9341_show_string(8U, 104U, "TOUCH:");
    ili9341_show_string(8U, 128U, "STATE:");
}

/**
 * @brief Refresh gimbal positions, zero state and touch status.
 */
static void touch_gimbal_update_ui(void)
{
    gimbal_stepper_status_struct status;

    gimbal_stepper_get_status(&status);
    touch_gimbal_draw_labels();
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    touch_gimbal_show_signed(
        40U,
        32U,
        status.axis[GIMBAL_STEPPER_AXIS_YAW].position_steps);
    touch_gimbal_show_signed(
        112U,
        32U,
        status.axis[GIMBAL_STEPPER_AXIS_YAW].target_position_steps);
    touch_gimbal_show_signed(
        40U,
        56U,
        status.axis[GIMBAL_STEPPER_AXIS_PITCH].position_steps);
    touch_gimbal_show_signed(
        112U,
        56U,
        status.axis[GIMBAL_STEPPER_AXIS_PITCH]
            .target_position_steps);
    ili9341_show_uint(
        72U,
        80U,
        status.axis[GIMBAL_STEPPER_AXIS_YAW].zero_valid,
        1U);
    ili9341_show_uint(
        112U,
        80U,
        status.axis[GIMBAL_STEPPER_AXIS_PITCH].zero_valid,
        1U);
    ili9341_show_string(
        176U,
        80U,
        status.selected_axis == GIMBAL_STEPPER_AXIS_YAW
            ? "YAW  " : "PITCH");

    if(touch_gimbal_touch_valid != 0U)
    {
        ili9341_show_uint(64U, 104U, touch_gimbal_last_x, 3U);
        ili9341_show_string(88U, 104U, ",");
        ili9341_show_uint(96U, 104U, touch_gimbal_last_y, 3U);
    }
    else
    {
        ili9341_show_string(64U, 104U, "---,---");
    }

    if(status.stop_latched != 0U)
    {
        ili9341_set_color(ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
        ili9341_show_string(64U, 128U, "STOPPED    ");
    }
    else if(status.relative_ready != 0U)
    {
        ili9341_set_color(ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
        ili9341_show_string(64U, 128U, "READY      ");
    }
    else if((status.axis[GIMBAL_STEPPER_AXIS_YAW].zero_valid != 0U)
        && (status.axis[GIMBAL_STEPPER_AXIS_PITCH].zero_valid != 0U))
    {
        ili9341_set_color(ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);
        ili9341_show_string(64U, 128U, "MANUAL     ");
    }
    else
    {
        ili9341_set_color(ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
        ili9341_show_string(64U, 128U, "ZERO FIRST ");
    }
}

/**
 * @brief Convert one accepted touch displacement to relative target steps.
 */
static void touch_gimbal_apply_delta(uint16 x, uint16 y)
{
    int16 delta_x = (int16)((int32)x
        - (int32)touch_gimbal_gesture.last_x);
    int16 delta_y = (int16)((int32)y
        - (int32)touch_gimbal_gesture.last_y);
    int32 yaw_steps = 0;
    int32 pitch_steps = 0;

    touch_gimbal_gesture.last_x = x;
    touch_gimbal_gesture.last_y = y;

    if((touch_gimbal_magnitude(delta_x)
            > TOUCH_GIMBAL_MAX_SAMPLE_DELTA)
        || (touch_gimbal_magnitude(delta_y)
            > TOUCH_GIMBAL_MAX_SAMPLE_DELTA))
    {
        touch_gimbal_gesture.pending_x = 0;
        touch_gimbal_gesture.pending_y = 0;
        touch_gimbal_gesture.yaw_remainder = 0;
        touch_gimbal_gesture.pitch_remainder = 0;
        return;
    }

    touch_gimbal_gesture.pending_x += delta_x;
    touch_gimbal_gesture.pending_y += delta_y;

    if(touch_gimbal_magnitude(touch_gimbal_gesture.pending_y)
        >= TOUCH_GIMBAL_PIXEL_DEAD_ZONE)
    {
        yaw_steps = touch_gimbal_pixels_to_steps(
            (int16)-touch_gimbal_gesture.pending_y,
            TOUCH_GIMBAL_YAW_SWIPE_STEPS,
            TOUCH_GIMBAL_YAW_PIXEL_SPAN,
            &touch_gimbal_gesture.yaw_remainder);
        touch_gimbal_gesture.pending_y = 0;
    }

    if(touch_gimbal_magnitude(touch_gimbal_gesture.pending_x)
        >= TOUCH_GIMBAL_PIXEL_DEAD_ZONE)
    {
        pitch_steps = touch_gimbal_pixels_to_steps(
            (int16)(touch_gimbal_gesture.pending_x
                * TOUCH_GIMBAL_PITCH_DIRECTION_SIGN),
            TOUCH_GIMBAL_PITCH_SWIPE_STEPS,
            TOUCH_GIMBAL_PITCH_PIXEL_SPAN,
            &touch_gimbal_gesture.pitch_remainder);
        touch_gimbal_gesture.pending_x = 0;
    }

    if((yaw_steps != 0) || (pitch_steps != 0))
    {
        if(gimbal_stepper_move_relative_steps(
                yaw_steps,
                pitch_steps) == 0U)
        {
            touch_gimbal_reset_gesture();
        }
    }
}

/**
 * @brief Poll one touch point and update the active drag gesture.
 */
static void touch_gimbal_poll(void)
{
    uint16 x;
    uint16 y;
    uint8 ready = gimbal_stepper_relative_ready();

    if(xpt2046_is_pressed() == 0U)
    {
        touch_gimbal_touch_valid = 0U;
        touch_gimbal_reset_gesture();
        return;
    }

    if(xpt2046_read_point(&x, &y) == 0U)
    {
        return;
    }

    touch_gimbal_last_x = x;
    touch_gimbal_last_y = y;
    touch_gimbal_touch_valid = 1U;
    touch_gimbal_draw_point(x, y);

    if(ready == 0U)
    {
        touch_gimbal_reset_gesture();
        return;
    }

    if(touch_gimbal_gesture.active == 0U)
    {
        touch_gimbal_gesture.last_x = x;
        touch_gimbal_gesture.last_y = y;
        touch_gimbal_gesture.pending_x = 0;
        touch_gimbal_gesture.pending_y = 0;
        touch_gimbal_gesture.yaw_remainder = 0;
        touch_gimbal_gesture.pitch_remainder = 0;
        touch_gimbal_gesture.active = 1U;
        return;
    }

    touch_gimbal_apply_delta(x, y);
}

/**
 * @brief Run the combined manual-zero and relative-angle touch test.
 */
void test_touch_gimbal_run(void)
{
    uint16 touch_elapsed_ms = 0U;
    uint16 ui_elapsed_ms = 0U;

    ili9341_init();
    xpt2046_init();
    touch_gimbal_reset_gesture();
    touch_gimbal_touch_valid = 0U;
    touch_gimbal_draw_layout();
    touch_gimbal_draw_labels();
    gimbal_stepper_init();
    touch_gimbal_update_ui();

    while(1)
    {
        uint16 elapsed_ms = gimbal_stepper_service();

        touch_elapsed_ms += elapsed_ms;
        if(touch_elapsed_ms >= TOUCH_GIMBAL_SAMPLE_PERIOD_MS)
        {
            touch_gimbal_poll();
            touch_elapsed_ms %= TOUCH_GIMBAL_SAMPLE_PERIOD_MS;
        }

        ui_elapsed_ms += elapsed_ms;
        if(ui_elapsed_ms >= TOUCH_GIMBAL_UI_PERIOD_MS)
        {
            touch_gimbal_update_ui();
            ui_elapsed_ms %= TOUCH_GIMBAL_UI_PERIOD_MS;
        }

        system_delay_ms(TOUCH_GIMBAL_LOOP_PERIOD_MS);
    }
}

#endif
