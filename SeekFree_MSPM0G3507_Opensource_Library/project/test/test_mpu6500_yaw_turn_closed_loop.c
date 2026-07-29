/**
 * @file    test_mpu6500_yaw_turn_closed_loop.c
 * @brief   MPU6500 chassis heading closed-loop turn to positive 90 degrees.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_MPU6500_YAW_TURN_CLOSED_LOOP)

#include "test_mpu6500_yaw_turn_closed_loop.h"

#include <string.h>

#include "control_scheduler.h"
#include "motor.h"
#include "vofa.h"

#define MPU6500_YAW_CLOSED_LOOP_TARGET_DEG          (73.9F)
#define MPU6500_YAW_CLOSED_LOOP_SPEED_DEG_S         (60.0F)
#define MPU6500_YAW_CLOSED_LOOP_WRONG_DIRECTION_DEG (-5.0F)
#define MPU6500_YAW_CLOSED_LOOP_READY_TIMEOUT_TICKS (3000U)
#define MPU6500_YAW_CLOSED_LOOP_REQUEST_TIMEOUT_TICKS (100U)
#define MPU6500_YAW_CLOSED_LOOP_TIMEOUT_TICKS       (1000U)
#define MPU6500_YAW_CLOSED_LOOP_RATE_PERIOD_TICKS   (100U)
#define MPU6500_YAW_CLOSED_LOOP_VOFA_PERIOD_TICKS   (2U)
#define MPU6500_YAW_CLOSED_LOOP_PROFILE             \
    (CHASSIS_MOTION_PID_PROFILE_TURN)

typedef enum
{
    MPU6500_YAW_CLOSED_LOOP_WAIT_READY = 0,
    MPU6500_YAW_CLOSED_LOOP_WAIT_ARM,
    MPU6500_YAW_CLOSED_LOOP_WAIT_PROFILE,
    MPU6500_YAW_CLOSED_LOOP_WAIT_ACTIVE,
    MPU6500_YAW_CLOSED_LOOP_RUNNING,
    MPU6500_YAW_CLOSED_LOOP_STOPPED,
    MPU6500_YAW_CLOSED_LOOP_FAILED,
} mpu6500_yaw_closed_loop_state_enum;

/**
 * @brief Send attitude, solve rate, and signed wheel duty through VOFA.
 */
static void mpu6500_yaw_closed_loop_send_vofa(
    const control_scheduler_status_struct *status,
    float relative_yaw_deg,
    float solve_rate_hz)
{
    static const uint8 tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float channels[6];
    uint8 frame[28];

    channels[0] = status->imu_roll_deg;
    channels[1] = status->imu_pitch_deg;
    channels[2] = relative_yaw_deg;
    channels[3] = solve_rate_hz;
    channels[4] = ((float)status->speed.left_duty * 100.0F)
        / (float)MOTOR_DUTY_MAX;
    channels[5] = ((float)status->speed.right_duty * 100.0F)
        / (float)MOTOR_DUTY_MAX;
    memcpy(frame, channels, sizeof(channels));
    memcpy(&frame[sizeof(channels)], tail, sizeof(tail));
    uart_write_buffer(VOFA_UART_INDEX, frame, sizeof(frame));
}

/**
 * @brief Stop chassis motion and retain a terminal test state.
 */
static void mpu6500_yaw_closed_loop_stop(
    mpu6500_yaw_closed_loop_state_enum *state,
    mpu6500_yaw_closed_loop_state_enum terminal_state)
{
    control_scheduler_request_chassis_motion_cancel();
    control_scheduler_request_disarm();
    *state = terminal_state;
}

/**
 * @brief Run one automatic +90 degree chassis heading closed-loop test.
 */
void test_mpu6500_yaw_turn_closed_loop_run(void)
{
    control_scheduler_status_struct status;
    mpu6500_yaw_closed_loop_state_enum state =
        MPU6500_YAW_CLOSED_LOOP_WAIT_READY;
    float start_yaw_deg = 0.0F;
    float relative_yaw_deg = 0.0F;
    float solve_rate_hz = 0.0F;
    uint32 action_start_tick = 0U;
    uint32 turn_start_tick = 0U;
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
            >= MPU6500_YAW_CLOSED_LOOP_RATE_PERIOD_TICKS)
        {
            elapsed_rate_ticks = status.tick_count - last_rate_tick;
            solve_rate_hz = ((float)(status.imu_angle_frame_count
                - last_rate_frame_count) * 100.0F)
                / (float)elapsed_rate_ticks;
            last_rate_frame_count = status.imu_angle_frame_count;
            last_rate_tick = status.tick_count;
        }

        if((state == MPU6500_YAW_CLOSED_LOOP_WAIT_ACTIVE)
            || (state == MPU6500_YAW_CLOSED_LOOP_RUNNING)
            || (state == MPU6500_YAW_CLOSED_LOOP_STOPPED)
            || (state == MPU6500_YAW_CLOSED_LOOP_FAILED))
        {
            relative_yaw_deg = status.imu_yaw_continuous_deg
                - start_yaw_deg;
        }

        if((status.fault_flags != CONTROL_FAULT_NONE)
            || (status.mode == CONTROL_MODE_FAULT_LATCHED))
        {
            if((state != MPU6500_YAW_CLOSED_LOOP_STOPPED)
                && (state != MPU6500_YAW_CLOSED_LOOP_FAILED))
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
        }
        else if(state == MPU6500_YAW_CLOSED_LOOP_WAIT_READY)
        {
            if((status.imu_ready != 0U) && (status.imu_fresh != 0U))
            {
                control_scheduler_request_arm();
                action_start_tick = status.tick_count;
                state = MPU6500_YAW_CLOSED_LOOP_WAIT_ARM;
            }
            else if(status.tick_count
                >= MPU6500_YAW_CLOSED_LOOP_READY_TIMEOUT_TICKS)
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
        }
        else if(state == MPU6500_YAW_CLOSED_LOOP_WAIT_ARM)
        {
            if(status.mode == CONTROL_MODE_MANUAL_ARMED)
            {
                (void)control_scheduler_request_chassis_motion_pid_profile(
                    MPU6500_YAW_CLOSED_LOOP_PROFILE);
                action_start_tick = status.tick_count;
                state = MPU6500_YAW_CLOSED_LOOP_WAIT_PROFILE;
            }
            else if((status.mode == CONTROL_MODE_DISARMED)
                && (status.imu_ready != 0U)
                && (status.imu_fresh != 0U))
            {
                control_scheduler_request_arm();
                action_start_tick = status.tick_count;
            }
            else if((status.tick_count - action_start_tick)
                >= MPU6500_YAW_CLOSED_LOOP_REQUEST_TIMEOUT_TICKS)
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
        }
        else if(state == MPU6500_YAW_CLOSED_LOOP_WAIT_PROFILE)
        {
            if(status.mode != CONTROL_MODE_MANUAL_ARMED)
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
            else if(status.chassis_motion.active_profile_id
                == MPU6500_YAW_CLOSED_LOOP_PROFILE)
            {
                start_yaw_deg = status.imu_yaw_continuous_deg;
                relative_yaw_deg = 0.0F;
                if(control_scheduler_request_chassis_motion_turn_relative(
                        MPU6500_YAW_CLOSED_LOOP_TARGET_DEG,
                        MPU6500_YAW_CLOSED_LOOP_SPEED_DEG_S) == ZF_FALSE)
                {
                    mpu6500_yaw_closed_loop_stop(
                        &state,
                        MPU6500_YAW_CLOSED_LOOP_FAILED);
                }
                else
                {
                    action_start_tick = status.tick_count;
                    state = MPU6500_YAW_CLOSED_LOOP_WAIT_ACTIVE;
                }
            }
            else if((status.tick_count - action_start_tick)
                >= MPU6500_YAW_CLOSED_LOOP_REQUEST_TIMEOUT_TICKS)
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
        }
        else if(state == MPU6500_YAW_CLOSED_LOOP_WAIT_ACTIVE)
        {
            if((status.mode == CONTROL_MODE_CHASSIS_MOTION)
                && (status.chassis_motion.active != 0U)
                && (status.chassis_motion.command
                    == CHASSIS_MOTION_COMMAND_TURN_RELATIVE))
            {
                turn_start_tick = status.tick_count;
                state = MPU6500_YAW_CLOSED_LOOP_RUNNING;
            }
            else if((status.tick_count - action_start_tick)
                >= MPU6500_YAW_CLOSED_LOOP_REQUEST_TIMEOUT_TICKS)
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
        }
        else if(state == MPU6500_YAW_CLOSED_LOOP_RUNNING)
        {
            if((status.imu_fresh == 0U)
                || (status.imu_ready == 0U)
                || (relative_yaw_deg
                    <= MPU6500_YAW_CLOSED_LOOP_WRONG_DIRECTION_DEG)
                || ((status.tick_count - turn_start_tick)
                    >= MPU6500_YAW_CLOSED_LOOP_TIMEOUT_TICKS))
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
            else if((status.chassis_motion.active == 0U)
                && (status.chassis_motion.result
                    == CHASSIS_MOTION_RESULT_COMPLETED))
            {
                if((relative_yaw_deg
                    >= (MPU6500_YAW_CLOSED_LOOP_TARGET_DEG
                        - CHASSIS_MOTION_TURN_TOLERANCE_DEG))
                    && (relative_yaw_deg
                        <= (MPU6500_YAW_CLOSED_LOOP_TARGET_DEG
                            + CHASSIS_MOTION_TURN_TOLERANCE_DEG)))
                {
                    mpu6500_yaw_closed_loop_stop(
                        &state,
                        MPU6500_YAW_CLOSED_LOOP_STOPPED);
                }
                else
                {
                    mpu6500_yaw_closed_loop_stop(
                        &state,
                        MPU6500_YAW_CLOSED_LOOP_FAILED);
                }
            }
            else if(status.mode != CONTROL_MODE_CHASSIS_MOTION)
            {
                mpu6500_yaw_closed_loop_stop(
                    &state,
                    MPU6500_YAW_CLOSED_LOOP_FAILED);
            }
        }

        if((status.imu_ready != 0U)
            && ((status.tick_count - last_vofa_tick)
                >= MPU6500_YAW_CLOSED_LOOP_VOFA_PERIOD_TICKS))
        {
            mpu6500_yaw_closed_loop_send_vofa(
                &status,
                relative_yaw_deg,
                solve_rate_hz);
            last_vofa_tick = status.tick_count;
        }
    }
}

#endif
