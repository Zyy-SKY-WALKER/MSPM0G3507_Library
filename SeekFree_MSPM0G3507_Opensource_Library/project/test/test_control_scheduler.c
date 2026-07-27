/**
 * @file    test_control_scheduler.c
 * @brief   Unified control scheduler TFT status and key test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_CONTROL_SCHEDULER)

#include "test_control_scheduler.h"

#include "control_scheduler.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define CONTROL_TEST_DISPLAY_TIME_MS    (100U)
#define CONTROL_TEST_LOOP_TIME_MS       (1U)
#define CONTROL_TEST_DISPLAY_TICKS      (20U)
#define CONTROL_TEST_DISPLAY_PHASES     (14U)

/** @brief Balanced profile for stable straights and decisive left corners. */
static const line_tracker_config_struct control_test_line_config =
{
    .base_speed_mm_s = {220.0F, 200.0F, 175.0F, 145.0F, 120.0F},
    .pid_kp = {30.0F, 38.0F, 43.0F, 55.0F, 63.0F},
    .pid_ki = 0.0F,
    .pid_kd = 0.30F,
    .pid_integral_limit_mm_s = 50.0F,
    .pid_derivative_filter_alpha = 0.2F,
    .max_target_mm_s = 800.0F,
    .max_correction_mm_s = 400.0F,
    .arc_outer_speed_mm_s = 280.0F,
    .arc_inner_speed_mm_s = 50.0F,
    .pivot_speed_mm_s = 300.0F,
    .lost_debounce_samples = 3U,
    .reacquire_samples = 3U,
    .arc_duration_samples = 120U,
    .search_timeout_samples = 500U,
    .default_search_direction = LINE_TRACKER_DIRECTION_LEFT,
};

/** @brief Last values physically written to the TFT. */
typedef struct
{
    uint32 fault_flags;
    uint32 tick_bucket;
    uint8 instruction_key;
    uint8 mode;
    uint8 gray_mask;
    int32 deviation_x10;
    int32 left_count;
    int32 right_count;
    int32 left_target;
    int32 right_target;
    int32 left_duty;
    int32 right_duty;
    uint8 line_state;
    uint8 imu_fresh;
    uint8 imu_ready;
    uint8 imu_progress;
    int32 imu_yaw;
    uint16 imu_age;
    int32 imu_drift_x10;
    int32 imu_roll_x10;
    int32 imu_pitch_x10;
    uint8 yaw_zero;
    uint8 pitch_zero;
    uint8 gimbal_calibrated;
    uint8 feedforward_valid;
    uint8 feedforward_reject;
    uint16 feedforward_solve_ticks;
    int32 pitch_position;
    int32 pitch_target;
    uint8 initialized;
} control_test_display_cache_struct;

/**
 * @brief Reduce scheduler state to one instruction-line identifier.
 */
static uint8 control_test_get_instruction_key(
    const control_scheduler_status_struct *status)
{
    uint8 key = 6U;

    if (status->gimbal_calibrated == 0U)
    {
        key = 0U;
    }
    else if (status->imu_ready == 0U)
    {
        key = 1U;
    }
    else if (status->mode == CONTROL_MODE_DISARMED)
    {
        key = 2U;
    }
    else if (status->mode == CONTROL_MODE_MANUAL_ARMED)
    {
        key = 3U;
    }
    else if (status->mode == CONTROL_MODE_LINE_FOLLOW)
    {
        key = 4U;
    }
    else if (status->mode == CONTROL_MODE_FAULT_LATCHED)
    {
        key = 5U;
    }

    return key;
}

/**
 * @brief Display the operation expected in the current scheduler state.
 * @param status Scheduler status.
 */
static void control_test_show_instruction(
    const control_scheduler_status_struct *status)
{
    const char *instruction;

    switch (control_test_get_instruction_key(status))
    {
        case 0U:
            instruction = "CAL: HOLD A30 / SHORT SEL";
            break;
        case 1U:
            instruction = "IMU SETTLING: KEEP STILL";
            break;
        case 2U:
            instruction = "READY: SHORT A30 TO ARM";
            break;
        case 3U:
            instruction = "ARMED: B0 START LINE";
            break;
        case 4U:
            instruction = "LINE RUN: A31 STOP";
            break;
        case 5U:
            instruction = "FAULT: RELEASE A31,HOLD B1";
            break;
        default:
            instruction = "CONTROL SCHEDULER";
            break;
    }

    ili9341_fill_rect(
        0U,
        8U,
        239U,
        23U,
        ILI9341_COLOR_BLACK);
    ili9341_show_string(0U, 8U, instruction);
}

/**
 * @brief Display an eight-bit mask with D1 at the left side.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param mask Mask to display.
 * @param previous_mask Previously displayed mask.
 * @param force Nonzero redraws all eight channels.
 */
static void control_test_show_mask(
    uint16 x,
    uint16 y,
    uint8 mask,
    uint8 previous_mask,
    uint8 force)
{
    uint8 changed_mask = force != 0U
        ? GRAY_SENSOR_ALL_ACTIVE_MASK
        : (uint8)(mask ^ previous_mask);
    uint8 index;

    for (index = 0U; index < GRAY_SENSOR_CHANNEL_COUNT; index++)
    {
        if ((changed_mask & (uint8)(1U << index)) != 0U)
        {
            ili9341_show_char(
                (uint16)(x + ((uint16)index * 8U)),
                y,
                (mask & (uint8)(1U << index)) != 0U ? '1' : '0');
        }
    }
}

/**
 * @brief Clear and display one signed status value.
 * @param x Horizontal coordinate.
 * @param y Vertical coordinate.
 * @param value Signed value.
 * @param digits Magnitude field width.
 */
static void control_test_show_int(
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
static void control_test_show_uint(
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
 * @brief Draw one row from the latest coherent scheduler status snapshot.
 * @param status Scheduler status.
 * @param cache Last values physically written to the display.
 * @param phase Row phase from zero through CONTROL_TEST_DISPLAY_PHASES - 1.
 */
static void control_test_show_status_phase(
    const control_scheduler_status_struct *status,
    control_test_display_cache_struct *cache,
    uint8 phase)
{
    uint8 force = (uint8)(cache->initialized == 0U);
    uint8 instruction_key = control_test_get_instruction_key(status);
    uint8 progress = status->imu_ready != 0U
        ? 100U
        : (uint8)((status->imu_stability_progress / 5U) * 5U);
    uint32 tick_bucket = status->tick_count / 100U;
    int32 deviation_x10 = (int32)(status->gray.deviation * 10.0F);
    int32 left_target = (int32)status->speed.left_target_mm_s;
    int32 right_target = (int32)status->speed.right_target_mm_s;
    int32 imu_yaw = (int32)status->imu_yaw_deg;
    int32 imu_drift_x10 = (int32)(
        status->imu_yaw_drift_deg_min * 10.0F);
    int32 imu_roll_x10 = (int32)(status->imu_roll_deg * 10.0F);
    int32 imu_pitch_x10 = (int32)(status->imu_pitch_deg * 10.0F);
    uint8 yaw_zero = status->gimbal.axis[
        GIMBAL_STEPPER_AXIS_YAW].zero_valid;
    uint8 pitch_zero = status->gimbal.axis[
        GIMBAL_STEPPER_AXIS_PITCH].zero_valid;
    int32 pitch_position = status->gimbal.axis[
        GIMBAL_STEPPER_AXIS_PITCH].position_steps;
    int32 pitch_target = status->gimbal.axis[
        GIMBAL_STEPPER_AXIS_PITCH].target_position_steps;

    switch (phase)
    {
        case 0U:
            if ((force != 0U)
                || (instruction_key != cache->instruction_key))
            {
                control_test_show_instruction(status);
                cache->instruction_key = instruction_key;
            }
            break;
        case 1U:
            if ((force != 0U) || ((uint8)status->mode != cache->mode))
            {
                control_test_show_uint(96U, 32U, status->mode, 2U);
                cache->mode = (uint8)status->mode;
            }
            break;
        case 2U:
            if ((force != 0U)
                || (status->fault_flags != cache->fault_flags))
            {
                control_test_show_uint(
                    96U,
                    56U,
                    status->fault_flags,
                    4U);
                cache->fault_flags = status->fault_flags;
            }
            break;
        case 3U:
            if ((force != 0U) || (tick_bucket != cache->tick_bucket))
            {
                control_test_show_uint(
                    96U,
                    80U,
                    status->tick_count,
                    8U);
                cache->tick_bucket = tick_bucket;
            }
            break;
        case 4U:
            if ((force != 0U)
                || (status->gray.active_mask != cache->gray_mask))
            {
                control_test_show_mask(
                    96U,
                    104U,
                    status->gray.active_mask,
                    cache->gray_mask,
                    force);
                cache->gray_mask = status->gray.active_mask;
            }
            break;
        case 5U:
            if ((force != 0U)
                || (deviation_x10 != cache->deviation_x10))
            {
                control_test_show_int(96U, 128U, deviation_x10, 3U);
                cache->deviation_x10 = deviation_x10;
            }
            break;
        case 6U:
            if ((force != 0U)
                || ((int32)status->left_count != cache->left_count))
            {
                control_test_show_int(
                    96U,
                    152U,
                    status->left_count,
                    5U);
                cache->left_count = status->left_count;
            }
            if ((force != 0U)
                || ((int32)status->right_count != cache->right_count))
            {
                control_test_show_int(
                    160U,
                    152U,
                    status->right_count,
                    5U);
                cache->right_count = status->right_count;
            }
            break;
        case 7U:
            if ((force != 0U) || (left_target != cache->left_target))
            {
                control_test_show_int(96U, 176U, left_target, 4U);
                cache->left_target = left_target;
            }
            if ((force != 0U) || (right_target != cache->right_target))
            {
                control_test_show_int(160U, 176U, right_target, 4U);
                cache->right_target = right_target;
            }
            break;
        case 8U:
            if ((force != 0U)
                || ((int32)status->speed.left_duty != cache->left_duty))
            {
                control_test_show_int(
                    96U,
                    200U,
                    status->speed.left_duty,
                    4U);
                cache->left_duty = status->speed.left_duty;
            }
            if ((force != 0U)
                || ((int32)status->speed.right_duty != cache->right_duty))
            {
                control_test_show_int(
                    160U,
                    200U,
                    status->speed.right_duty,
                    4U);
                cache->right_duty = status->speed.right_duty;
            }
            break;
        case 9U:
            if ((force != 0U)
                || ((uint8)status->line_status.state
                    != cache->line_state))
            {
                control_test_show_uint(
                    96U,
                    224U,
                    status->line_status.state,
                    2U);
                cache->line_state = (uint8)status->line_status.state;
            }
            if ((force != 0U)
                || (status->imu_fresh != cache->imu_fresh))
            {
                control_test_show_uint(
                    160U,
                    224U,
                    status->imu_fresh,
                    1U);
                cache->imu_fresh = status->imu_fresh;
            }
            if ((force != 0U)
                || (status->imu_ready != cache->imu_ready))
            {
                control_test_show_uint(
                    176U,
                    224U,
                    status->imu_ready,
                    1U);
                cache->imu_ready = status->imu_ready;
            }
            if ((force != 0U) || (progress != cache->imu_progress))
            {
                control_test_show_uint(192U, 224U, progress, 3U);
                cache->imu_progress = progress;
            }
            break;
        case 10U:
            if ((force != 0U) || (imu_yaw != cache->imu_yaw))
            {
                control_test_show_int(96U, 248U, imu_yaw, 4U);
                cache->imu_yaw = imu_yaw;
            }
            if ((force != 0U)
                || (status->imu_age_ticks != cache->imu_age))
            {
                control_test_show_uint(
                    160U,
                    248U,
                    status->imu_age_ticks,
                    4U);
                cache->imu_age = status->imu_age_ticks;
            }
            if ((force != 0U)
                || (imu_drift_x10 != cache->imu_drift_x10))
            {
                control_test_show_int(
                    192U,
                    248U,
                    imu_drift_x10,
                    4U);
                cache->imu_drift_x10 = imu_drift_x10;
            }
            break;
        case 11U:
            if ((force != 0U)
                || (imu_roll_x10 != cache->imu_roll_x10))
            {
                control_test_show_int(96U, 264U, imu_roll_x10, 4U);
                cache->imu_roll_x10 = imu_roll_x10;
            }
            if ((force != 0U)
                || (imu_pitch_x10 != cache->imu_pitch_x10))
            {
                control_test_show_int(160U, 264U, imu_pitch_x10, 4U);
                cache->imu_pitch_x10 = imu_pitch_x10;
            }
            break;
        case 12U:
            if ((force != 0U) || (yaw_zero != cache->yaw_zero))
            {
                control_test_show_uint(96U, 280U, yaw_zero, 1U);
                cache->yaw_zero = yaw_zero;
            }
            if ((force != 0U) || (pitch_zero != cache->pitch_zero))
            {
                control_test_show_uint(112U, 280U, pitch_zero, 1U);
                cache->pitch_zero = pitch_zero;
            }
            if ((force != 0U)
                || (status->gimbal_calibrated
                    != cache->gimbal_calibrated))
            {
                control_test_show_uint(
                    144U,
                    280U,
                    status->gimbal_calibrated,
                    1U);
                cache->gimbal_calibrated = status->gimbal_calibrated;
            }
            if ((force != 0U)
                || (status->gimbal_feedforward_valid
                    != cache->feedforward_valid))
            {
                control_test_show_uint(
                    160U,
                    280U,
                    status->gimbal_feedforward_valid,
                    1U);
                cache->feedforward_valid =
                    status->gimbal_feedforward_valid;
            }
            if ((force != 0U)
                || ((uint8)status->gimbal_feedforward_reject_reason
                    != cache->feedforward_reject))
            {
                control_test_show_uint(
                    176U,
                    280U,
                    status->gimbal_feedforward_reject_reason,
                    1U);
                cache->feedforward_reject = (uint8)
                    status->gimbal_feedforward_reject_reason;
            }
            if ((force != 0U)
                || (status->gimbal_feedforward_solve_ticks
                    != cache->feedforward_solve_ticks))
            {
                control_test_show_uint(
                    192U,
                    280U,
                    status->gimbal_feedforward_solve_ticks,
                    2U);
                cache->feedforward_solve_ticks =
                    status->gimbal_feedforward_solve_ticks;
            }
            break;
        case 13U:
            if ((force != 0U)
                || (pitch_position != cache->pitch_position))
            {
                control_test_show_int(96U, 296U, pitch_position, 5U);
                cache->pitch_position = pitch_position;
            }
            if ((force != 0U) || (pitch_target != cache->pitch_target))
            {
                control_test_show_int(160U, 296U, pitch_target, 5U);
                cache->pitch_target = pitch_target;
            }
            cache->initialized = 1U;
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize and run the scheduler status and key test.
 */
void test_control_scheduler_run(void)
{
    static control_test_display_cache_struct display_cache;
    control_scheduler_status_struct status;
    uint32 last_display_tick = 0U;
    uint8 display_phase = CONTROL_TEST_DISPLAY_PHASES;

    gimbal_stepper_laser_init();
    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "CONTROL SCHEDULER");
    ili9341_show_string(8U, 32U, "MODE   :");
    ili9341_show_string(8U, 56U, "FAULT  :");
    ili9341_show_string(8U, 80U, "TICK   :");
    ili9341_show_string(8U, 104U, "GRAY   :");
    ili9341_show_string(8U, 128U, "DEVx10 :");
    ili9341_show_string(8U, 152U, "ENC L/R:");
    ili9341_show_string(8U, 176U, "TGT L/R:");
    ili9341_show_string(8U, 200U, "PWM L/R:");
    ili9341_show_string(8U, 224U, "LINE/F/R/P:");
    ili9341_show_string(8U, 248U, "YAW/AGE/DR:");
    ili9341_show_string(8U, 264U, "ROLL/PIT:");
    ili9341_show_string(8U, 280U, "GIM FLAGS:");
    ili9341_show_string(8U, 296U, "P CUR/TGT:");

    if ((control_scheduler_init() == ZF_FALSE)
        || (line_tracker_set_config(&control_test_line_config) == ZF_FALSE)
        || (control_scheduler_start() == ZF_FALSE))
    {
        ili9341_fill_rect(
            0U,
            8U,
            239U,
            23U,
            ILI9341_COLOR_BLACK);
        ili9341_show_string(0U, 8U, "CONTROL INIT FAILED");
        while (true)
        {
            system_delay_ms(CONTROL_TEST_DISPLAY_TIME_MS);
        }
    }

    while (true)
    {
        uint32 current_tick;

        control_scheduler_process_foreground();
        current_tick = control_scheduler_get_tick_count();
        if ((display_phase >= CONTROL_TEST_DISPLAY_PHASES)
            && ((uint32)(current_tick - last_display_tick)
                >= CONTROL_TEST_DISPLAY_TICKS))
        {
            control_scheduler_get_status(&status);
            last_display_tick = status.tick_count;
            display_phase = 0U;
        }
        if (display_phase < CONTROL_TEST_DISPLAY_PHASES)
        {
            control_test_show_status_phase(
                &status,
                &display_cache,
                display_phase);
            display_phase++;
        }
        system_delay_ms(CONTROL_TEST_LOOP_TIME_MS);
    }
}

#endif
