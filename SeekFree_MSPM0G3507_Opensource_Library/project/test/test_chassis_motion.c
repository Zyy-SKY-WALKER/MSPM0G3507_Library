/**
 * @file    test_chassis_motion.c
 * @brief   Chassis motion scheduler status display test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_CHASSIS_MOTION)

#include "test_chassis_motion.h"

#include "control_scheduler.h"
#include "my_lib_ili9341.h"
#include "zf_driver_delay.h"

#define CHASSIS_MOTION_TEST_DISPLAY_PERIOD_MS    (100U)
#define CHASSIS_MOTION_TEST_PAUSE_MS             (1000U)
#define CHASSIS_MOTION_TEST_REQUEST_TIMEOUT_MS   (500U)
#define CHASSIS_MOTION_TEST_DIRECTION_LIMIT_MM   (20.0F)

typedef enum
{
    CHASSIS_MOTION_TEST_WAIT_ARM = 0,
    CHASSIS_MOTION_TEST_WAIT_PROFILE,
    CHASSIS_MOTION_TEST_WAIT_ACTIVE,
    CHASSIS_MOTION_TEST_WAIT_COMPLETE,
    CHASSIS_MOTION_TEST_PAUSE,
    CHASSIS_MOTION_TEST_DONE,
    CHASSIS_MOTION_TEST_FAILED,
} chassis_motion_test_state_enum;

typedef enum
{
    CHASSIS_MOTION_TEST_ERROR_NONE = 0,
    CHASSIS_MOTION_TEST_ERROR_PROFILE,
    CHASSIS_MOTION_TEST_ERROR_COMMAND,
    CHASSIS_MOTION_TEST_ERROR_START_TIMEOUT,
    CHASSIS_MOTION_TEST_ERROR_COMMAND_TIMEOUT,
    CHASSIS_MOTION_TEST_ERROR_IMU,
    CHASSIS_MOTION_TEST_ERROR_DIRECTION,
    CHASSIS_MOTION_TEST_ERROR_RESULT,
    CHASSIS_MOTION_TEST_ERROR_FAULT,
} chassis_motion_test_error_enum;

typedef struct
{
    chassis_motion_command_enum command;
    float value;
    float speed;
    uint32 duration_ms;
    uint32 timeout_ms;
    uint8 profile_id;
} chassis_motion_test_step_struct;

static const chassis_motion_test_step_struct chassis_motion_test_steps[] =
{
    {CHASSIS_MOTION_COMMAND_TIMED, 100.0F, 0.0F, 500U, 2000U,
        CHASSIS_MOTION_PID_PROFILE_STRAIGHT},
    {CHASSIS_MOTION_COMMAND_DISTANCE, 200.0F, 100.0F, 0U, 5000U,
        CHASSIS_MOTION_PID_PROFILE_STRAIGHT},
    {CHASSIS_MOTION_COMMAND_DISTANCE, -200.0F, 100.0F, 0U, 5000U,
        CHASSIS_MOTION_PID_PROFILE_STRAIGHT},
    {CHASSIS_MOTION_COMMAND_TURN_RELATIVE, 90.0F, 30.0F, 0U, 7000U,
        CHASSIS_MOTION_PID_PROFILE_TURN},
    {CHASSIS_MOTION_COMMAND_TURN_RELATIVE, -90.0F, 30.0F, 0U, 7000U,
        CHASSIS_MOTION_PID_PROFILE_TURN},
    {CHASSIS_MOTION_COMMAND_TIMED, 150.0F, 0.0F, 1000U, 3000U,
        CHASSIS_MOTION_PID_PROFILE_STRAIGHT},
    {CHASSIS_MOTION_COMMAND_TIMED, -150.0F, 0.0F, 1000U, 3000U,
        CHASSIS_MOTION_PID_PROFILE_STRAIGHT},
};

static volatile chassis_motion_test_state_enum chassis_motion_test_state;
static volatile chassis_motion_test_error_enum chassis_motion_test_error;
static volatile uint8 chassis_motion_test_step_index;
static uint32 chassis_motion_test_state_tick;

/**
 * @brief Return elapsed scheduler time from a retained tick.
 * @param status Scheduler status snapshot.
 * @param start_tick Retained scheduler tick.
 * @return Elapsed time in milliseconds.
 */
static uint32 chassis_motion_test_elapsed_ms(
    const control_scheduler_status_struct *status,
    uint32 start_tick)
{
    return (status->tick_count - start_tick)
        * CONTROL_SCHEDULER_PERIOD_MS;
}

/**
 * @brief Configure straight and turn profiles from tuned wheel gains.
 * @return ZF_TRUE when both profiles were accepted.
 */
static uint8 chassis_motion_test_configure_profiles(void)
{
    chassis_motion_pid_profile_struct straight_profile;
    chassis_motion_pid_profile_struct turn_profile;

    straight_profile.left_speed_kp = SPEED_PID_LEFT_KP;
    straight_profile.left_speed_ki = SPEED_PID_LEFT_KI;
    straight_profile.left_speed_kd = SPEED_PID_LEFT_KD;
    straight_profile.right_speed_kp = SPEED_PID_RIGHT_KP;
    straight_profile.right_speed_ki = SPEED_PID_RIGHT_KI;
    straight_profile.right_speed_kd = SPEED_PID_RIGHT_KD;
    straight_profile.heading_kp = 2.0F;
    straight_profile.heading_ki = 0.0F;
    straight_profile.heading_kd = 0.0F;

    turn_profile = straight_profile;
    turn_profile.heading_kp = 1.0F;

    if (chassis_motion_pid_profile_configure(
            CHASSIS_MOTION_PID_PROFILE_STRAIGHT,
            &straight_profile) == ZF_FALSE)
    {
        return ZF_FALSE;
    }
    if (chassis_motion_pid_profile_configure(
            CHASSIS_MOTION_PID_PROFILE_TURN,
            &turn_profile) == ZF_FALSE)
    {
        return ZF_FALSE;
    }

    return ZF_TRUE;
}

/**
 * @brief Enter the failed test state and cancel active chassis motion.
 * @param error Retained test error code.
 * @param status Latest scheduler status.
 */
static void chassis_motion_test_fail(
    chassis_motion_test_error_enum error,
    const control_scheduler_status_struct *status)
{
    if (status->mode == CONTROL_MODE_CHASSIS_MOTION)
    {
        control_scheduler_request_chassis_motion_cancel();
    }
    chassis_motion_test_error = error;
    chassis_motion_test_state = CHASSIS_MOTION_TEST_FAILED;
}

/**
 * @brief Queue the active sequence step.
 * @param status Latest scheduler status.
 */
static void chassis_motion_test_submit_step(
    const control_scheduler_status_struct *status)
{
    const chassis_motion_test_step_struct *step;
    uint8 accepted = ZF_FALSE;

    step = &chassis_motion_test_steps[chassis_motion_test_step_index];
    if (step->command == CHASSIS_MOTION_COMMAND_DISTANCE)
    {
        accepted = control_scheduler_request_chassis_motion_distance(
            step->value,
            step->speed);
    }
    else if (step->command == CHASSIS_MOTION_COMMAND_TIMED)
    {
        accepted = control_scheduler_request_chassis_motion_timed(
            step->value,
            step->duration_ms);
    }
    else if (step->command == CHASSIS_MOTION_COMMAND_TURN_RELATIVE)
    {
        accepted = control_scheduler_request_chassis_motion_turn_relative(
            step->value,
            step->speed);
    }

    if (accepted == ZF_FALSE)
    {
        chassis_motion_test_fail(
            CHASSIS_MOTION_TEST_ERROR_COMMAND,
            status);
        return;
    }

    chassis_motion_test_state_tick = status->tick_count;
    chassis_motion_test_state = CHASSIS_MOTION_TEST_WAIT_ACTIVE;
}

/**
 * @brief Select the profile for the active sequence step.
 * @param status Latest scheduler status.
 */
static void chassis_motion_test_prepare_step(
    const control_scheduler_status_struct *status)
{
    const chassis_motion_test_step_struct *step;

    step = &chassis_motion_test_steps[chassis_motion_test_step_index];
    if (status->imu_fresh == 0U)
    {
        chassis_motion_test_fail(CHASSIS_MOTION_TEST_ERROR_IMU, status);
        return;
    }

    if (status->chassis_motion.active_profile_id == step->profile_id)
    {
        chassis_motion_test_submit_step(status);
        return;
    }

    if (control_scheduler_request_chassis_motion_pid_profile(
            step->profile_id) == ZF_FALSE)
    {
        chassis_motion_test_fail(
            CHASSIS_MOTION_TEST_ERROR_PROFILE,
            status);
        return;
    }

    chassis_motion_test_state_tick = status->tick_count;
    chassis_motion_test_state = CHASSIS_MOTION_TEST_WAIT_PROFILE;
}

/**
 * @brief Return whether a distance step is moving in the wrong direction.
 * @param status Latest scheduler status.
 * @param step Active sequence step.
 * @return Nonzero after meaningful opposite displacement.
 */
static uint8 chassis_motion_test_direction_is_wrong(
    const control_scheduler_status_struct *status,
    const chassis_motion_test_step_struct *step)
{
    float progress;

    if (step->command != CHASSIS_MOTION_COMMAND_DISTANCE)
    {
        return 0U;
    }

    progress = status->chassis_motion.current_center_displacement_mm
        - status->chassis_motion.start_center_displacement_mm;
    return (uint8)(
        ((step->value > 0.0F)
            && (progress < -CHASSIS_MOTION_TEST_DIRECTION_LIMIT_MM))
        || ((step->value < 0.0F)
            && (progress > CHASSIS_MOTION_TEST_DIRECTION_LIMIT_MM)));
}

/**
 * @brief Advance the complete chassis-motion test state machine.
 * @param status Latest scheduler status.
 */
static void chassis_motion_test_update(
    const control_scheduler_status_struct *status)
{
    const chassis_motion_test_step_struct *step;
    uint32 elapsed_ms;

    if ((status->fault_flags != CONTROL_FAULT_NONE)
        || (status->mode == CONTROL_MODE_FAULT_LATCHED))
    {
        chassis_motion_test_error = CHASSIS_MOTION_TEST_ERROR_FAULT;
        chassis_motion_test_state = CHASSIS_MOTION_TEST_FAILED;
        return;
    }

    if (status->mode == CONTROL_MODE_DISARMED)
    {
        chassis_motion_test_step_index = 0U;
        chassis_motion_test_error = CHASSIS_MOTION_TEST_ERROR_NONE;
        chassis_motion_test_state = CHASSIS_MOTION_TEST_WAIT_ARM;
        return;
    }

    if (chassis_motion_test_state == CHASSIS_MOTION_TEST_WAIT_ARM)
    {
        if (status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            chassis_motion_test_prepare_step(status);
        }
        return;
    }

    if ((chassis_motion_test_state == CHASSIS_MOTION_TEST_DONE)
        || (chassis_motion_test_state == CHASSIS_MOTION_TEST_FAILED))
    {
        return;
    }

    step = &chassis_motion_test_steps[chassis_motion_test_step_index];
    elapsed_ms = chassis_motion_test_elapsed_ms(
        status,
        chassis_motion_test_state_tick);

    if (chassis_motion_test_state == CHASSIS_MOTION_TEST_WAIT_PROFILE)
    {
        if ((status->mode == CONTROL_MODE_MANUAL_ARMED)
            && (status->chassis_motion.active_profile_id
                == step->profile_id))
        {
            chassis_motion_test_submit_step(status);
        }
        else if (elapsed_ms >= CHASSIS_MOTION_TEST_REQUEST_TIMEOUT_MS)
        {
            chassis_motion_test_fail(
                CHASSIS_MOTION_TEST_ERROR_PROFILE,
                status);
        }
        return;
    }

    if (chassis_motion_test_state == CHASSIS_MOTION_TEST_WAIT_ACTIVE)
    {
        if (status->mode == CONTROL_MODE_CHASSIS_MOTION)
        {
            chassis_motion_test_state_tick = status->tick_count;
            chassis_motion_test_state = CHASSIS_MOTION_TEST_WAIT_COMPLETE;
        }
        else if (elapsed_ms >= CHASSIS_MOTION_TEST_REQUEST_TIMEOUT_MS)
        {
            chassis_motion_test_fail(
                CHASSIS_MOTION_TEST_ERROR_START_TIMEOUT,
                status);
        }
        return;
    }

    if (chassis_motion_test_state == CHASSIS_MOTION_TEST_WAIT_COMPLETE)
    {
        if (status->imu_fresh == 0U)
        {
            chassis_motion_test_fail(CHASSIS_MOTION_TEST_ERROR_IMU, status);
            return;
        }
        if (chassis_motion_test_direction_is_wrong(status, step) != 0U)
        {
            chassis_motion_test_fail(
                CHASSIS_MOTION_TEST_ERROR_DIRECTION,
                status);
            return;
        }
        if (elapsed_ms >= step->timeout_ms)
        {
            chassis_motion_test_fail(
                CHASSIS_MOTION_TEST_ERROR_COMMAND_TIMEOUT,
                status);
            return;
        }
        if (status->mode == CONTROL_MODE_MANUAL_ARMED)
        {
            if (status->chassis_motion.result
                != CHASSIS_MOTION_RESULT_COMPLETED)
            {
                chassis_motion_test_fail(
                    CHASSIS_MOTION_TEST_ERROR_RESULT,
                    status);
                return;
            }
            chassis_motion_test_state_tick = status->tick_count;
            chassis_motion_test_state = CHASSIS_MOTION_TEST_PAUSE;
        }
        return;
    }

    if ((chassis_motion_test_state == CHASSIS_MOTION_TEST_PAUSE)
        && (elapsed_ms >= CHASSIS_MOTION_TEST_PAUSE_MS))
    {
        chassis_motion_test_step_index++;
        if (chassis_motion_test_step_index
            >= (sizeof(chassis_motion_test_steps)
                / sizeof(chassis_motion_test_steps[0])))
        {
            chassis_motion_test_state = CHASSIS_MOTION_TEST_DONE;
        }
        else
        {
            chassis_motion_test_prepare_step(status);
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
static void chassis_motion_test_show_int(
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
static void chassis_motion_test_show_uint(
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
 * @brief Draw one chassis motion scheduler status snapshot.
 * @param status Scheduler status.
 */
static void chassis_motion_test_show_status(
    const control_scheduler_status_struct *status)
{
    chassis_motion_test_show_uint(104U, 32U, status->mode, 2U);
    chassis_motion_test_show_uint(
        104U,
        56U,
        status->fault_flags,
        4U);
    chassis_motion_test_show_uint(
        104U,
        80U,
        chassis_motion_test_step_index,
        2U);
    chassis_motion_test_show_uint(
        168U,
        80U,
        chassis_motion_test_state,
        2U);
    chassis_motion_test_show_uint(
        104U,
        104U,
        status->chassis_motion.command,
        2U);
    chassis_motion_test_show_uint(
        168U,
        104U,
        status->chassis_motion.phase,
        2U);
    chassis_motion_test_show_uint(
        104U,
        128U,
        status->chassis_motion.result,
        2U);
    chassis_motion_test_show_uint(
        168U,
        128U,
        chassis_motion_test_error,
        2U);
    chassis_motion_test_show_uint(
        104U,
        152U,
        status->chassis_motion.active_profile_id,
        3U);
    chassis_motion_test_show_uint(
        168U,
        152U,
        status->imu_fresh,
        1U);
    chassis_motion_test_show_int(
        104U,
        176U,
        (int32)status->chassis_motion.left_target_mm_s,
        4U);
    chassis_motion_test_show_int(
        168U,
        176U,
        (int32)status->chassis_motion.right_target_mm_s,
        4U);
    chassis_motion_test_show_int(
        104U,
        200U,
        (int32)status->speed.left_speed_mm_s,
        4U);
    chassis_motion_test_show_int(
        168U,
        200U,
        (int32)status->speed.right_speed_mm_s,
        4U);
    chassis_motion_test_show_int(
        104U,
        224U,
        (int32)status->chassis_motion.current_center_displacement_mm,
        5U);
    chassis_motion_test_show_int(
        104U,
        248U,
        (int32)status->chassis_motion.current_heading_deg,
        4U);
}

/**
 * @brief Initialize and run the complete chassis motion sequence.
 * @note Arming starts one finite sequence; disarm before restarting it.
 */
void test_chassis_motion_run(void)
{
    control_scheduler_status_struct status;

    ili9341_init();
    ili9341_full(ILI9341_COLOR_BLACK);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(8U, 8U, "CHASSIS MOTION");
    ili9341_show_string(8U, 32U, "MODE    :");
    ili9341_show_string(8U, 56U, "FAULT   :");
    ili9341_show_string(8U, 80U, "STEP/ST :");
    ili9341_show_string(8U, 104U, "CMD/PH  :");
    ili9341_show_string(8U, 128U, "RES/ERR :");
    ili9341_show_string(8U, 152U, "PROF/IM :");
    ili9341_show_string(8U, 176U, "TGT L/R :");
    ili9341_show_string(8U, 200U, "SPD L/R :");
    ili9341_show_string(8U, 224U, "DIST MM :");
    ili9341_show_string(8U, 248U, "YAW DEG :");
    ili9341_show_string(8U, 280U, "A30 ARM/START");
    ili9341_show_string(8U, 296U, "A31 STOP B1 CLEAR");

    control_scheduler_init();
    chassis_motion_test_step_index = 0U;
    chassis_motion_test_state = CHASSIS_MOTION_TEST_WAIT_ARM;
    chassis_motion_test_error = CHASSIS_MOTION_TEST_ERROR_NONE;
    chassis_motion_test_state_tick = 0U;
    if (chassis_motion_test_configure_profiles() == ZF_FALSE)
    {
        chassis_motion_test_state = CHASSIS_MOTION_TEST_FAILED;
        chassis_motion_test_error = CHASSIS_MOTION_TEST_ERROR_PROFILE;
    }
    control_scheduler_start();

    while (true)
    {
        control_scheduler_process_foreground();
        control_scheduler_get_status(&status);
        chassis_motion_test_update(&status);
        chassis_motion_test_show_status(&status);
        system_delay_ms(CHASSIS_MOTION_TEST_DISPLAY_PERIOD_MS);
    }
}

#endif
