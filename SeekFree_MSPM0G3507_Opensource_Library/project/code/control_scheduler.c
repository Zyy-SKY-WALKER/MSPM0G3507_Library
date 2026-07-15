/**
 * @file    control_scheduler.c
 * @brief   Unified bare-metal 10 ms vehicle control scheduler.
 */

#include "control_scheduler.h"

#include <float.h>

#include "drive_geometry.h"
#include "imu_uart.h"
#include "my_lib_encoder.h"
#include "zf_common_interrupt.h"
#include "zf_device_key.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"

#define CONTROL_SCHEDULER_PIT               (PIT_TIM_G12)
#define CONTROL_EMERGENCY_KEY_PIN            (A31)
#define CONTROL_ENCODER_COUNT_LIMIT          (5000)
#define CONTROL_STOPPED_COUNT_LIMIT          (3)
#define CONTROL_IMU_FRESH_LIMIT_TICKS        (50U)
#define CONTROL_MANUAL_TIMEOUT_TICKS         \
    (CONTROL_SCHEDULER_MANUAL_TIMEOUT_MS \
        / CONTROL_SCHEDULER_PERIOD_MS)

#define CONTROL_REQUEST_ARM                          (0x0001U)
#define CONTROL_REQUEST_DISARM                       (0x0002U)
#define CONTROL_REQUEST_LINE_START                   (0x0004U)
#define CONTROL_REQUEST_LINE_STOP                    (0x0008U)
#define CONTROL_REQUEST_FAULT_CLEAR                  (0x0010U)
#define CONTROL_REQUEST_MANUAL_TARGET                (0x0020U)
#define CONTROL_REQUEST_CHASSIS_COMMAND              (0x0040U)
#define CONTROL_REQUEST_CHASSIS_CANCEL               (0x0080U)
#define CONTROL_REQUEST_CHASSIS_PID_PROFILE          (0x0100U)

typedef enum
{
    CONTROL_CHASSIS_REQUEST_NONE = 0,
    CONTROL_CHASSIS_REQUEST_DISTANCE,
    CONTROL_CHASSIS_REQUEST_TIMED,
    CONTROL_CHASSIS_REQUEST_TURN,
} control_chassis_request_enum;

typedef struct
{
    uint16 flags;
    float left_target_mm_s;
    float right_target_mm_s;
    float chassis_value;
    float chassis_speed;
    uint32 chassis_duration_ms;
    control_chassis_request_enum chassis_command;
    uint8 chassis_profile_id;
} control_request_mailbox_struct;

static volatile control_scheduler_status_struct control_status;
static volatile control_request_mailbox_struct control_mailbox;
static volatile uint8 control_initialized;
static volatile uint8 control_started;
static volatile uint8 control_update_busy;
static volatile uint8 control_yaw_reset_pending;
static volatile uint8 control_yaw_reset_sent;
static volatile uint8 control_imu_rebase_pending;
static volatile uint32 control_imu_rebase_frame_count;

static uint32 control_last_imu_frame_count;
static uint16 control_manual_target_age_ticks;
static uint8 control_manual_target_active;
static uint8 control_key4_long_handled;

/**
 * @brief Clear the stored manual command and its timeout state.
 */
static void control_clear_manual_target(void)
{
    control_manual_target_active = 0U;
    control_manual_target_age_ticks = 0U;
    control_status.manual_left_target_mm_s = 0.0F;
    control_status.manual_right_target_mm_s = 0.0F;
}

/**
 * @brief Return the absolute magnitude of a signed encoder count.
 * @param count Signed count.
 * @return Unsigned magnitude.
 */
static uint16 control_count_magnitude(int16 count)
{
    int32 magnitude = count;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }

    return (uint16)magnitude;
}

/**
 * @brief Check that a target is finite and in speed PID range.
 * @param target_mm_s Target speed.
 * @return Nonzero when valid.
 */
static uint8 control_target_is_valid(float target_mm_s)
{
    return (uint8)((target_mm_s == target_mm_s)
        && (target_mm_s >= -SPEED_PID_TARGET_LIMIT_MM_S)
        && (target_mm_s <= SPEED_PID_TARGET_LIMIT_MM_S)
        && (target_mm_s >= -FLT_MAX)
        && (target_mm_s <= FLT_MAX));
}

/**
 * @brief Check that one floating-point control request value is finite.
 * @param value Request value.
 * @return Nonzero when valid.
 */
static uint8 control_value_is_valid(float value)
{
    return (uint8)((value == value)
        && (value >= -FLT_MAX)
        && (value <= FLT_MAX));
}

/**
 * @brief Validate one bounded relative-turn request.
 * @param angle_deg Requested relative angle in degrees.
 * @param angular_speed_deg_s Positive angular-speed limit in degrees per second.
 * @return Nonzero when the request can map to valid wheel targets.
 */
static uint8 control_turn_request_is_valid(
    float angle_deg,
    float angular_speed_deg_s)
{
    float wheel_speed_mm_s;

    if ((control_value_is_valid(angle_deg) == 0U)
        || (control_value_is_valid(angular_speed_deg_s) == 0U)
        || (angle_deg == 0.0F)
        || (angle_deg < -180.0F)
        || (angle_deg > 180.0F)
        || (angular_speed_deg_s <= 0.0F))
    {
        return 0U;
    }

    wheel_speed_mm_s = angular_speed_deg_s
        * (DRIVE_PI / 180.0F)
        * (DRIVE_TRACK_WIDTH_MM * 0.5F);
    return (uint8)(wheel_speed_mm_s <= SPEED_PID_TARGET_LIMIT_MM_S);
}

/**
 * @brief Force zero output and latch one or more fault flags.
 * @param fault_flags Fault bits to latch.
 */
static void control_latch_fault(uint32 fault_flags)
{
    control_status.fault_flags |= fault_flags;
    control_status.mode = CONTROL_MODE_FAULT_LATCHED;
    control_clear_manual_target();
    chassis_motion_reset();
    speed_pid_stop();
}

/**
 * @brief Enter the safe disarmed state and clear motion targets.
 */
static void control_enter_disarmed(void)
{
    control_status.mode = CONTROL_MODE_DISARMED;
    control_clear_manual_target();
    chassis_motion_reset();
    line_tracker_reset();
    speed_pid_stop();
}

/**
 * @brief Return whether both sampled wheels are effectively stopped.
 * @return Nonzero when both counts are inside the stopped threshold.
 */
static uint8 control_wheels_are_stopped(void)
{
    return (uint8)((control_count_magnitude(control_status.left_count)
            <= CONTROL_STOPPED_COUNT_LIMIT)
        && (control_count_magnitude(control_status.right_count)
            <= CONTROL_STOPPED_COUNT_LIMIT));
}

/**
 * @brief Clear a latched fault after release and stopped-wheel checks.
 * @param emergency_active Nonzero when the raw emergency key is pressed.
 */
static void control_try_clear_fault(uint8 emergency_active)
{
    if ((control_status.mode != CONTROL_MODE_FAULT_LATCHED)
        || (emergency_active != 0U)
        || (control_wheels_are_stopped() == 0U))
    {
        return;
    }

    control_status.fault_flags = CONTROL_FAULT_NONE;
    my_encoder_clear_count();
    odometry_reset();
    control_yaw_reset_pending = 1U;
    control_yaw_reset_sent = 0U;
    control_imu_rebase_pending = 1U;
    control_imu_rebase_frame_count =
        control_status.imu_angle_frame_count;
    control_enter_disarmed();
}

/**
 * @brief Consume all foreground requests into a local immutable snapshot.
 * @return Request snapshot.
 */
static control_request_mailbox_struct control_take_requests(void)
{
    control_request_mailbox_struct requests;

    requests.flags = control_mailbox.flags;
    requests.left_target_mm_s = control_mailbox.left_target_mm_s;
    requests.right_target_mm_s = control_mailbox.right_target_mm_s;
    requests.chassis_value = control_mailbox.chassis_value;
    requests.chassis_speed = control_mailbox.chassis_speed;
    requests.chassis_duration_ms = control_mailbox.chassis_duration_ms;
    requests.chassis_command = control_mailbox.chassis_command;
    requests.chassis_profile_id = control_mailbox.chassis_profile_id;
    control_mailbox.flags = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_NONE;

    return requests;
}

/**
 * @brief Merge physical key events into one scheduler request snapshot.
 * @param requests Request snapshot to extend.
 */
static void control_add_key_requests(
    control_request_mailbox_struct *requests)
{
    key_state_enum key1_state = key_get_state(KEY_1);
    key_state_enum key3_state = key_get_state(KEY_3);
    key_state_enum key4_state = key_get_state(KEY_4);

    if (key1_state == KEY_SHORT_PRESS)
    {
        if ((control_status.mode == CONTROL_MODE_MANUAL_ARMED)
            || (control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
            || (control_status.mode == CONTROL_MODE_LINE_FOLLOW))
        {
            requests->flags |= CONTROL_REQUEST_DISARM;
        }
        else if (control_status.mode == CONTROL_MODE_DISARMED)
        {
            requests->flags |= CONTROL_REQUEST_ARM;
        }
        key_clear_state(KEY_1);
    }

    if (key3_state == KEY_SHORT_PRESS)
    {
        if (control_status.mode == CONTROL_MODE_LINE_FOLLOW)
        {
            requests->flags |= CONTROL_REQUEST_LINE_STOP;
        }
        else if (control_status.mode == CONTROL_MODE_MANUAL_ARMED)
        {
            requests->flags |= CONTROL_REQUEST_LINE_START;
        }
        key_clear_state(KEY_3);
    }

    if (key4_state == KEY_LONG_PRESS)
    {
        if (control_key4_long_handled == 0U)
        {
            requests->flags |= CONTROL_REQUEST_FAULT_CLEAR;
            control_key4_long_handled = 1U;
        }
    }
    else
    {
        control_key4_long_handled = 0U;
    }
}

/**
 * @brief Apply requests according to scheduler mode and safety priority.
 * @param requests Coherent request snapshot.
 * @param emergency_active Nonzero when the raw emergency key is pressed.
 */
static void control_apply_requests(
    const control_request_mailbox_struct *requests,
    uint8 emergency_active)
{
    uint8 chassis_started = 0U;

    if ((requests->flags & CONTROL_REQUEST_FAULT_CLEAR) != 0U)
    {
        uint8 was_faulted =
            (uint8)(control_status.mode == CONTROL_MODE_FAULT_LATCHED);

        control_try_clear_fault(emergency_active);
        if (was_faulted != 0U)
        {
            return;
        }
    }

    if (control_status.mode == CONTROL_MODE_FAULT_LATCHED)
    {
        return;
    }

    if ((requests->flags & CONTROL_REQUEST_DISARM) != 0U)
    {
        control_enter_disarmed();
        return;
    }

    if ((requests->flags & CONTROL_REQUEST_ARM) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_DISARMED)
        {
            control_status.mode = CONTROL_MODE_MANUAL_ARMED;
            speed_pid_reset();
        }
    }

    if ((requests->flags & CONTROL_REQUEST_LINE_STOP) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_LINE_FOLLOW)
        {
            line_tracker_reset();
            speed_pid_reset();
            control_clear_manual_target();
            control_status.mode = CONTROL_MODE_MANUAL_ARMED;
        }
    }

    if ((requests->flags & CONTROL_REQUEST_LINE_START) != 0U)
    {
        if ((control_status.mode == CONTROL_MODE_MANUAL_ARMED)
            && (control_status.gray.status == GRAY_SENSOR_STATUS_VALID))
        {
            control_clear_manual_target();
            line_tracker_reset();
            control_status.mode = CONTROL_MODE_LINE_FOLLOW;
        }
    }

    if ((requests->flags & CONTROL_REQUEST_MANUAL_TARGET) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_MANUAL_ARMED)
        {
            control_status.manual_left_target_mm_s =
                requests->left_target_mm_s;
            control_status.manual_right_target_mm_s =
                requests->right_target_mm_s;
            control_manual_target_active =
                (uint8)((requests->left_target_mm_s != 0.0F)
                    || (requests->right_target_mm_s != 0.0F));
            control_manual_target_age_ticks = 0U;
        }
    }

    if ((requests->flags & CONTROL_REQUEST_CHASSIS_PID_PROFILE) != 0U)
    {
        (void)chassis_motion_pid_profile_select(
            requests->chassis_profile_id);
    }

    if ((requests->flags & CONTROL_REQUEST_CHASSIS_CANCEL) != 0U)
    {
        if (control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
        {
            chassis_motion_cancel();
        }
    }

    if ((control_status.mode == CONTROL_MODE_MANUAL_ARMED)
        || (control_status.mode == CONTROL_MODE_CHASSIS_MOTION))
    {
        if ((requests->flags & CONTROL_REQUEST_CHASSIS_COMMAND) != 0U)
        {
            if (requests->chassis_command
                == CONTROL_CHASSIS_REQUEST_DISTANCE)
            {
                chassis_started = chassis_motion_start_distance(
                    requests->chassis_value,
                    requests->chassis_speed);
            }
            else if (requests->chassis_command
                == CONTROL_CHASSIS_REQUEST_TIMED)
            {
                chassis_started = chassis_motion_start_timed(
                    requests->chassis_speed,
                    requests->chassis_duration_ms);
            }
            else if (requests->chassis_command
                == CONTROL_CHASSIS_REQUEST_TURN)
            {
                chassis_started = chassis_motion_start_turn_relative(
                    requests->chassis_value,
                    requests->chassis_speed);
            }
        }

        if (chassis_started != 0U)
        {
            control_clear_manual_target();
            control_status.mode = CONTROL_MODE_CHASSIS_MOTION;
        }
    }
}

/**
 * @brief Update manual command timeout and disarm stale motion targets.
 */
static void control_update_manual_timeout(void)
{
    if ((control_status.mode != CONTROL_MODE_MANUAL_ARMED)
        || (control_manual_target_active == 0U))
    {
        return;
    }

    if (control_manual_target_age_ticks < 0xFFFFU)
    {
        control_manual_target_age_ticks++;
    }

    if (control_manual_target_age_ticks >= CONTROL_MANUAL_TIMEOUT_TICKS)
    {
        control_enter_disarmed();
    }
}

/**
 * @brief Select one target pair from the current scheduler mode.
 * @param odometry Latest odometry state.
 * @param yaw_deg Latest IMU yaw in degrees.
 * @param left_count Latest signed left encoder delta.
 * @param right_count Latest signed right encoder delta.
 * @param left_target Destination left target.
 * @param right_target Destination right target.
 */
static void control_select_targets(
    const gray_sensor_result_struct *gray,
    const odometry_state_struct *odometry,
    float yaw_deg,
    int16 left_count,
    int16 right_count,
    line_tracker_output_struct *line_output,
    float *left_target,
    float *right_target)
{
    *left_target = 0.0F;
    *right_target = 0.0F;

    if (control_status.mode == CONTROL_MODE_MANUAL_ARMED)
    {
        *left_target = control_status.manual_left_target_mm_s;
        *right_target = control_status.manual_right_target_mm_s;
    }
    else if (control_status.mode == CONTROL_MODE_CHASSIS_MOTION)
    {
        chassis_motion_update_10ms(
            odometry,
            yaw_deg,
            left_count,
            right_count,
            left_target,
            right_target);
        if (chassis_motion_is_busy() == 0U)
        {
            control_status.mode = CONTROL_MODE_MANUAL_ARMED;
        }
    }
    else if (control_status.mode == CONTROL_MODE_LINE_FOLLOW)
    {
        if (line_tracker_update(
                gray,
                line_output) == ZF_FALSE)
        {
            control_latch_fault(CONTROL_FAULT_LINE_TRACKER);
            return;
        }

        *left_target = line_output->left_target_mm_s;
        *right_target = line_output->right_target_mm_s;
    }
}

/**
 * @brief Publish all module status snapshots after one completed tick.
 */
static void control_publish_status(void)
{
    line_tracker_status_struct line_status;
    chassis_motion_status_struct chassis_status;
    speed_pid_status_struct speed_status;
    odometry_state_struct odometry_status;

    line_tracker_get_status(&line_status);
    chassis_motion_get_status(&chassis_status);
    speed_pid_get_status(&speed_status);
    odometry_get_state(&odometry_status);
    control_status.line_status = line_status;
    control_status.chassis_motion = chassis_status;
    control_status.speed = speed_status;
    control_status.odometry = odometry_status;
    control_status.initialized = control_initialized;
    control_status.started = control_started;
}

/**
 * @brief PIT callback that owns the fixed-period control pipeline.
 * @param event PIT callback event value.
 * @param user_data Optional callback context.
 */
static void control_pit_callback(uint32 event, void *user_data)
{
    (void)event;
    (void)user_data;

    control_scheduler_update_10ms();
}

/**
 * @brief Initialize every control dependency in a safe stopped state.
 * @return ZF_TRUE when mandatory hardware initialized successfully.
 */
uint8 control_scheduler_init(void)
{
    uint8 gray_ready;

    control_initialized = 0U;
    control_started = 0U;
    control_update_busy = 0U;
    control_yaw_reset_pending = 0U;
    control_yaw_reset_sent = 0U;
    control_imu_rebase_pending = 0U;
    control_imu_rebase_frame_count = 0U;
    control_last_imu_frame_count = 0U;
    control_manual_target_age_ticks = 0U;
    control_manual_target_active = 0U;
    control_key4_long_handled = 0U;

    control_status.mode = CONTROL_MODE_BOOT;
    control_status.fault_flags = CONTROL_FAULT_NONE;
    control_status.tick_count = 0U;
    control_status.overrun_count = 0U;
    control_status.imu_age_ticks = 0U;
    control_status.imu_valid = 0U;
    control_status.imu_fresh = 0U;
    control_mailbox.flags = 0U;
    control_mailbox.left_target_mm_s = 0.0F;
    control_mailbox.right_target_mm_s = 0.0F;
    control_mailbox.chassis_value = 0.0F;
    control_mailbox.chassis_speed = 0.0F;
    control_mailbox.chassis_duration_ms = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_NONE;
    control_mailbox.chassis_profile_id =
        CHASSIS_MOTION_PID_PROFILE_INVALID;

    speed_pid_init();
    chassis_motion_init();
    my_encoder_init();
    gray_ready = gray_sensor_init();
    imu_uart_init();
    odometry_init();
    line_tracker_init(NULL);
    key_init(CONTROL_SCHEDULER_PERIOD_MS);
    my_encoder_clear_count();

    control_initialized = 1U;
    control_status.initialized = 1U;
    control_status.started = 0U;
    if (gray_ready == ZF_FALSE)
    {
        control_latch_fault(CONTROL_FAULT_GRAY_INIT);
        control_publish_status();
        return ZF_FALSE;
    }

    control_enter_disarmed();
    control_publish_status();
    return ZF_TRUE;
}

/**
 * @brief Start the unique 10 ms PIT control source.
 * @return ZF_TRUE when the scheduler was started.
 */
uint8 control_scheduler_start(void)
{
    if ((control_initialized == 0U) || (control_started != 0U))
    {
        return ZF_FALSE;
    }

    control_started = 1U;
    control_status.started = 1U;
    pit_ms_init(
        CONTROL_SCHEDULER_PIT,
        CONTROL_SCHEDULER_PERIOD_MS,
        control_pit_callback,
        NULL);

    return ZF_TRUE;
}

/**
 * @brief Execute the complete fixed-period control pipeline once.
 */
void control_scheduler_update_10ms(void)
{
    control_request_mailbox_struct requests;
    gray_sensor_result_struct gray;
    line_tracker_output_struct line_output;
    float left_target = 0.0F;
    float right_target = 0.0F;
    uint32 yaw_frame_count = 0U;
    float yaw_deg = 0.0F;
    odometry_state_struct odometry;
    int16 left_count;
    int16 right_count;
    uint8 emergency_active;
    uint8 encoder_valid;
    uint8 yaw_valid;
    uint8 odometry_yaw_valid;
    uint8 gray_valid;

    if ((control_initialized == 0U) || (control_started == 0U))
    {
        return;
    }

    if (control_update_busy != 0U)
    {
        control_status.overrun_count++;
        control_latch_fault(CONTROL_FAULT_REENTRY);
        return;
    }
    control_update_busy = 1U;
    control_status.tick_count++;

    emergency_active = (uint8)(
        gpio_get_level(CONTROL_EMERGENCY_KEY_PIN) == GPIO_LOW);
    if (emergency_active != 0U)
    {
        control_latch_fault(CONTROL_FAULT_EMERGENCY_KEY);
    }

    key_scanner();
    requests = control_take_requests();
    control_add_key_requests(&requests);
    control_apply_requests(&requests, emergency_active);

    my_encoder_get_delta(&left_count, &right_count);
    yaw_valid = imu_uart_get_yaw(&yaw_deg, &yaw_frame_count);
    gray = control_status.gray;
    gray_valid = gray_sensor_sample(&gray);

    control_status.left_count = left_count;
    control_status.right_count = right_count;
    if (gray_valid != 0U)
    {
        control_status.gray = gray;
    }

    control_status.imu_yaw_deg = yaw_deg;
    control_status.imu_angle_frame_count = yaw_frame_count;
    control_status.imu_valid = yaw_valid;
    if ((yaw_valid != 0U)
        && (yaw_frame_count != control_last_imu_frame_count))
    {
        control_last_imu_frame_count = yaw_frame_count;
        control_status.imu_age_ticks = 0U;
    }
    else if (control_status.imu_age_ticks < 0xFFFFU)
    {
        control_status.imu_age_ticks++;
    }
    control_status.imu_fresh =
        (uint8)((yaw_valid != 0U)
            && (control_status.imu_age_ticks
                <= CONTROL_IMU_FRESH_LIMIT_TICKS));

    if (gray_valid == ZF_FALSE)
    {
        control_latch_fault(CONTROL_FAULT_GRAY_SAMPLE);
    }
    encoder_valid = (uint8)(
        (control_count_magnitude(control_status.left_count)
            <= CONTROL_ENCODER_COUNT_LIMIT)
        && (control_count_magnitude(control_status.right_count)
            <= CONTROL_ENCODER_COUNT_LIMIT));
    if (encoder_valid == 0U)
    {
        control_latch_fault(CONTROL_FAULT_ENCODER_RANGE);
    }

    odometry_yaw_valid = yaw_valid;
    if (control_imu_rebase_pending != 0U)
    {
        if ((control_yaw_reset_sent != 0U)
            && (yaw_valid != 0U)
            && (yaw_frame_count != control_imu_rebase_frame_count))
        {
            control_yaw_reset_sent = 0U;
            control_imu_rebase_pending = 0U;
        }
        else
        {
            odometry_yaw_valid = ZF_FALSE;
            control_status.imu_fresh = 0U;
        }
    }

    if (encoder_valid != 0U)
    {
        odometry_update(
            left_count,
            right_count,
            odometry_yaw_valid,
            yaw_deg,
            yaw_frame_count);
    }
    else
    {
        odometry_update(
            0,
            0,
            odometry_yaw_valid,
            yaw_deg,
            yaw_frame_count);
    }
    odometry_get_state(&odometry);

    control_update_manual_timeout();
    line_output.left_target_mm_s = 0.0F;
    line_output.right_target_mm_s = 0.0F;
    control_select_targets(
        &gray,
        &odometry,
        yaw_deg,
        left_count,
        right_count,
        &line_output,
        &left_target,
        &right_target);
    control_status.line_output = line_output;

    if (control_status.mode == CONTROL_MODE_FAULT_LATCHED)
    {
        left_target = 0.0F;
        right_target = 0.0F;
    }

    speed_pid_set_target(left_target, right_target);
    speed_pid_update_10ms(
        left_count,
        right_count);
    control_publish_status();
    control_update_busy = 0U;
}

/**
 * @brief Run deferred non-real-time control actions in foreground context.
 */
void control_scheduler_process_foreground(void)
{
    float yaw_deg;
    uint32 yaw_frame_count = 0U;
    uint32 primask;
    uint8 reset_yaw;
    uint8 yaw_valid;

    primask = interrupt_global_disable();
    reset_yaw = control_yaw_reset_pending;
    control_yaw_reset_pending = 0U;
    interrupt_global_enable(primask);

    if (reset_yaw != 0U)
    {
        imu_uart_reset_yaw();
        yaw_valid = imu_uart_get_yaw(&yaw_deg, &yaw_frame_count);

        primask = interrupt_global_disable();
        if (control_imu_rebase_pending != 0U)
        {
            control_imu_rebase_frame_count =
                yaw_valid != 0U ? yaw_frame_count : 0U;
            control_yaw_reset_sent = 1U;
        }
        interrupt_global_enable(primask);
    }
}

/**
 * @brief Submit an explicit arm request.
 */
void control_scheduler_request_arm(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_ARM;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit an explicit disarm request.
 */
void control_scheduler_request_disarm(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_DISARM;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a line-follow start request.
 */
void control_scheduler_request_line_start(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_LINE_START;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a line-follow stop request.
 */
void control_scheduler_request_line_stop(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_LINE_STOP;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a latched-fault clear request.
 */
void control_scheduler_request_fault_clear(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_FAULT_CLEAR;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit one bounded manual target pair.
 * @param left_mm_s Left target speed.
 * @param right_mm_s Right target speed.
 * @return ZF_TRUE when the request values are valid.
 */
uint8 control_scheduler_request_manual_target(
    float left_mm_s,
    float right_mm_s)
{
    uint32 primask;

    if ((control_target_is_valid(left_mm_s) == 0U)
        || (control_target_is_valid(right_mm_s) == 0U))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.left_target_mm_s = left_mm_s;
    control_mailbox.right_target_mm_s = right_mm_s;
    control_mailbox.flags |= CONTROL_REQUEST_MANUAL_TARGET;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a high-level signed distance command.
 * @param distance_mm Positive for forward and negative for reverse distance.
 * @param max_speed_mm_s Positive maximum center speed.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_distance(
    float distance_mm,
    float max_speed_mm_s)
{
    uint32 primask;

    if ((control_value_is_valid(distance_mm) == 0U)
        || (control_target_is_valid(max_speed_mm_s) == 0U)
        || (distance_mm == 0.0F)
        || (max_speed_mm_s <= 0.0F))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_value = distance_mm;
    control_mailbox.chassis_speed = max_speed_mm_s;
    control_mailbox.chassis_duration_ms = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_DISTANCE;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_COMMAND;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a high-level timed signed-speed command.
 * @param speed_mm_s Positive for forward and negative for reverse speed.
 * @param duration_ms Command duration.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_timed(
    float speed_mm_s,
    uint32 duration_ms)
{
    uint32 primask;

    if ((control_target_is_valid(speed_mm_s) == 0U)
        || (speed_mm_s == 0.0F)
        || (duration_ms == 0U))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_speed = speed_mm_s;
    control_mailbox.chassis_duration_ms = duration_ms;
    control_mailbox.chassis_value = 0.0F;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_TIMED;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_COMMAND;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a high-level relative heading-turn command.
 * @param angle_deg Relative angle from -180 to 180 degrees, excluding zero.
 *                  Positive turns left and negative turns right.
 * @param max_angular_speed_deg_s Positive maximum angular speed.
 * @return ZF_TRUE when request values are valid.
 */
uint8 control_scheduler_request_chassis_motion_turn_relative(
    float angle_deg,
    float max_angular_speed_deg_s)
{
    uint32 primask;

    if (control_turn_request_is_valid(
            angle_deg,
            max_angular_speed_deg_s) == 0U)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_value = angle_deg;
    control_mailbox.chassis_speed = max_angular_speed_deg_s;
    control_mailbox.chassis_duration_ms = 0U;
    control_mailbox.chassis_command = CONTROL_CHASSIS_REQUEST_TURN;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_COMMAND;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Submit a request to smoothly cancel the active chassis command.
 */
void control_scheduler_request_chassis_motion_cancel(void)
{
    uint32 primask = interrupt_global_disable();

    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_CANCEL;

    interrupt_global_enable(primask);
}

/**
 * @brief Submit a request to select one configured chassis PID parameter group.
 * @param profile_id Profile identifier from 0 to 3.
 * @return ZF_TRUE when the profile identifier is valid.
 */
uint8 control_scheduler_request_chassis_motion_pid_profile(uint8 profile_id)
{
    uint32 primask;

    if (profile_id >= CHASSIS_MOTION_PID_PROFILE_COUNT)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    control_mailbox.chassis_profile_id = profile_id;
    control_mailbox.flags |= CONTROL_REQUEST_CHASSIS_PID_PROFILE;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Copy one coherent scheduler telemetry snapshot.
 * @param status Destination status structure.
 */
void control_scheduler_get_status(control_scheduler_status_struct *status)
{
    uint32 primask;

    if (status == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *status = control_status;
    interrupt_global_enable(primask);
}
