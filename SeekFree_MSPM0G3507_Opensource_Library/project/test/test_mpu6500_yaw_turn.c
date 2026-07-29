/**
 * @file    test_mpu6500_yaw_turn.c
 * @brief   MPU6500 speed-loop differential turn to positive 90 degrees.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_MPU6500_YAW_TURN)

#include "test_mpu6500_yaw_turn.h"

#include <string.h>

#include "control_scheduler.h"
#include "drive_geometry.h"
#include "vofa.h"

#define MPU6500_YAW_TURN_TARGET_DEG          (90.0F)
#define MPU6500_YAW_TURN_SPEED_DEG_S         (75.0F)
#define MPU6500_YAW_TURN_TIMEOUT_TICKS       (1000U)
#define MPU6500_YAW_TURN_WRONG_DIRECTION_DEG (-5.0F)
#define MPU6500_YAW_TURN_TARGET_REFRESH_TICKS (20U)
#define MPU6500_YAW_TURN_RATE_PERIOD_TICKS   (100U)
#define MPU6500_YAW_TURN_VOFA_PERIOD_TICKS   (2U)
#define MPU6500_YAW_TURN_SPEED_PROFILE       \
    (CHASSIS_MOTION_PID_PROFILE_STRAIGHT)

#define MPU6500_YAW_TURN_WHEEL_SPEED_MM_S    \
    (MPU6500_YAW_TURN_SPEED_DEG_S * (DRIVE_PI / 180.0F) \
        * (DRIVE_TRACK_WIDTH_MM * 0.5F))

typedef enum
{
    MPU6500_YAW_TURN_WAIT_READY = 0,
    MPU6500_YAW_TURN_WAIT_ARM,
    MPU6500_YAW_TURN_WAIT_PROFILE,
    MPU6500_YAW_TURN_RUNNING,
    MPU6500_YAW_TURN_STOPPED,
    MPU6500_YAW_TURN_FAILED,
} mpu6500_yaw_turn_state_enum;

/**
 * @brief Send roll, pitch, relative yaw, and solve rate through VOFA.
 */
static void mpu6500_yaw_turn_send_vofa(
    const control_scheduler_status_struct *status,
    float relative_yaw_deg,
    float solve_rate_hz)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float channels[4];
    uint8 frame[20];

    channels[0] = status->imu_roll_deg;
    channels[1] = status->imu_pitch_deg;
    channels[2] = relative_yaw_deg;
    channels[3] = solve_rate_hz;
    memcpy(frame, channels, sizeof(channels));
    memcpy(&frame[sizeof(channels)], tail, sizeof(tail));
    uart_write_buffer(VOFA_UART_INDEX, frame, sizeof(frame));
}

/**
 * @brief Stop all wheel targets and enter a terminal test state.
 */
static void mpu6500_yaw_turn_stop(
    mpu6500_yaw_turn_state_enum *state,
    mpu6500_yaw_turn_state_enum terminal_state)
{
    (void)control_scheduler_request_manual_target(0.0F, 0.0F);
    control_scheduler_request_disarm();
    *state = terminal_state;
}

/**
 * @brief Run one automatic speed-loop left turn and retain final yaw telemetry.
 */
void test_mpu6500_yaw_turn_run(void)
{
    control_scheduler_status_struct status;
    mpu6500_yaw_turn_state_enum state = MPU6500_YAW_TURN_WAIT_READY;
    float start_yaw_deg = 0.0F;
    float relative_yaw_deg = 0.0F;
    float solve_rate_hz = 0.0F;
    uint32 turn_start_tick = 0U;
    uint32 last_target_tick = 0U;
    uint32 last_rate_tick = 0U;
    uint32 last_rate_frame_count = 0U;
    uint32 last_vofa_tick = 0U;

    if((vofa_init_tx_only() == ZF_FALSE)
        || (control_scheduler_init() == ZF_FALSE)
        || (control_scheduler_start() == ZF_FALSE))
    {
        while(true)
        {
        }
    }

    while(true)
    {
        uint32 elapsed_rate_ticks;

        control_scheduler_process_foreground();
        control_scheduler_get_status(&status);

        if((status.tick_count - last_rate_tick)
            >= MPU6500_YAW_TURN_RATE_PERIOD_TICKS)
        {
            elapsed_rate_ticks = status.tick_count - last_rate_tick;
            solve_rate_hz = ((float)(status.imu_angle_frame_count
                - last_rate_frame_count) * 100.0F)
                / (float)elapsed_rate_ticks;
            last_rate_frame_count = status.imu_angle_frame_count;
            last_rate_tick = status.tick_count;
        }

        if((state == MPU6500_YAW_TURN_RUNNING)
            || (state == MPU6500_YAW_TURN_STOPPED)
            || (state == MPU6500_YAW_TURN_FAILED))
        {
            relative_yaw_deg = status.imu_yaw_continuous_deg
                - start_yaw_deg;
        }

        if((status.fault_flags != CONTROL_FAULT_NONE)
            || (status.mode == CONTROL_MODE_FAULT_LATCHED))
        {
            if((state != MPU6500_YAW_TURN_STOPPED)
                && (state != MPU6500_YAW_TURN_FAILED))
            {
                mpu6500_yaw_turn_stop(
                    &state,
                    MPU6500_YAW_TURN_FAILED);
            }
        }
        else if(state == MPU6500_YAW_TURN_WAIT_READY)
        {
            if((status.imu_ready != 0U) && (status.imu_fresh != 0U))
            {
                control_scheduler_request_arm();
                state = MPU6500_YAW_TURN_WAIT_ARM;
            }
        }
        else if(state == MPU6500_YAW_TURN_WAIT_ARM)
        {
            if(status.mode == CONTROL_MODE_MANUAL_ARMED)
            {
                (void)control_scheduler_request_chassis_motion_pid_profile(
                    MPU6500_YAW_TURN_SPEED_PROFILE);
                state = MPU6500_YAW_TURN_WAIT_PROFILE;
            }
            else if((status.mode == CONTROL_MODE_DISARMED)
                && (status.imu_ready != 0U)
                && (status.imu_fresh != 0U))
            {
                control_scheduler_request_arm();
            }
        }
        else if(state == MPU6500_YAW_TURN_WAIT_PROFILE)
        {
            if(status.mode != CONTROL_MODE_MANUAL_ARMED)
            {
                mpu6500_yaw_turn_stop(
                    &state,
                    MPU6500_YAW_TURN_FAILED);
            }
            else if(status.chassis_motion.active_profile_id
                == MPU6500_YAW_TURN_SPEED_PROFILE)
            {
                start_yaw_deg = status.imu_yaw_continuous_deg;
                relative_yaw_deg = 0.0F;
                turn_start_tick = status.tick_count;
                last_target_tick = status.tick_count;
                (void)control_scheduler_request_manual_target(
                    -MPU6500_YAW_TURN_WHEEL_SPEED_MM_S,
                    MPU6500_YAW_TURN_WHEEL_SPEED_MM_S);
                state = MPU6500_YAW_TURN_RUNNING;
            }
        }
        else if(state == MPU6500_YAW_TURN_RUNNING)
        {
            if((status.mode != CONTROL_MODE_MANUAL_ARMED)
                || (status.imu_fresh == 0U)
                || (status.imu_ready == 0U)
                || (relative_yaw_deg
                    <= MPU6500_YAW_TURN_WRONG_DIRECTION_DEG)
                || ((status.tick_count - turn_start_tick)
                    >= MPU6500_YAW_TURN_TIMEOUT_TICKS))
            {
                mpu6500_yaw_turn_stop(
                    &state,
                    MPU6500_YAW_TURN_FAILED);
            }
            else if(relative_yaw_deg >= MPU6500_YAW_TURN_TARGET_DEG)
            {
                mpu6500_yaw_turn_stop(
                    &state,
                    MPU6500_YAW_TURN_STOPPED);
            }
            else if((status.tick_count - last_target_tick)
                >= MPU6500_YAW_TURN_TARGET_REFRESH_TICKS)
            {
                (void)control_scheduler_request_manual_target(
                    -MPU6500_YAW_TURN_WHEEL_SPEED_MM_S,
                    MPU6500_YAW_TURN_WHEEL_SPEED_MM_S);
                last_target_tick = status.tick_count;
            }
        }

        if((status.imu_ready != 0U)
            && ((status.tick_count - last_vofa_tick)
                >= MPU6500_YAW_TURN_VOFA_PERIOD_TICKS))
        {
            mpu6500_yaw_turn_send_vofa(
                &status,
                relative_yaw_deg,
                solve_rate_hz);
            last_vofa_tick = status.tick_count;
        }
    }
}

#endif
