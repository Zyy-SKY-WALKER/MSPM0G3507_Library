/**
 * @file    control_imu_mpu6500.c
 * @brief   Foreground MPU6500 source for the vehicle control scheduler.
 */

#include "control_imu_mpu6500.h"

#include "control_scheduler.h"
#include "my_lib_mpu6500.h"
#include "my_lib_mpu6500_attitude.h"
#include "zf_common_interrupt.h"

static volatile control_imu_mpu6500_data_struct control_imu_data;
static uint32 control_imu_last_service_tick;
static uint32 control_imu_last_success_tick;
static uint32 control_imu_update_count;
static uint8 control_imu_initialized;
static uint8 control_imu_success_seen;
static uint8 control_imu_attitude_ready;
static uint8 control_imu_attitude_enabled = 1U;

/**
 * @brief Clear the shared source snapshot before hardware initialization.
 */
static void control_imu_mpu6500_clear(void)
{
    control_imu_data.accel_x_g = 0.0F;
    control_imu_data.roll_deg = 0.0F;
    control_imu_data.pitch_deg = 0.0F;
    control_imu_data.yaw_deg = 0.0F;
    control_imu_data.yaw_continuous_deg = 0.0F;
    control_imu_data.yaw_bias_deg_s = 0.0F;
    control_imu_data.temperature_c = 0.0F;
    control_imu_data.update_count = 0U;
    control_imu_data.calibration_sample_count = 0U;
    control_imu_data.calibration_progress = 0U;
    control_imu_data.valid = 0U;
    control_imu_data.ready = 0U;
}

/**
 * @brief Invalidate the shared sample after a missed foreground update.
 */
static void control_imu_mpu6500_invalidate(void)
{
    uint32 primask = interrupt_global_disable();

    control_imu_data.valid = 0U;
    control_imu_data.ready = 0U;
    interrupt_global_enable(primask);
}

/**
 * @brief Select raw-acceleration-only or attitude-estimation operation.
 */
void control_imu_mpu6500_set_attitude_enabled(uint8 enabled)
{
    if(control_imu_initialized == 0U)
    {
        control_imu_attitude_enabled = enabled != 0U ? 1U : 0U;
    }
}

/**
 * @brief Initialize the MPU6500 and its attitude estimator.
 */
uint8 control_imu_mpu6500_init(void)
{
    control_imu_initialized = 0U;
    control_imu_last_service_tick = 0U;
    control_imu_last_success_tick = 0U;
    control_imu_update_count = 0U;
    control_imu_success_seen = 0U;
    control_imu_attitude_ready = control_imu_attitude_enabled != 0U
        ? 0U : 1U;
    control_imu_mpu6500_clear();

    if(mpu6500_init() != 0U)
    {
        return ZF_FALSE;
    }

    if(control_imu_attitude_enabled != 0U)
    {
        mpu6500_attitude_init();
    }
    control_imu_initialized = 1U;
    return ZF_TRUE;
}

/**
 * @brief Read and solve one sample outside the real-time control interrupt.
 */
void control_imu_mpu6500_service(uint32 scheduler_tick)
{
    control_imu_mpu6500_data_struct next_data;
    mpu6500_attitude_data_struct attitude_data;
    mpu6500_data_struct sensor_data;
    uint32 elapsed_ticks;
    uint32 dt_ms;
    uint32 primask;

    if((control_imu_initialized == 0U)
        || (scheduler_tick == control_imu_last_service_tick))
    {
        return;
    }

    control_imu_last_service_tick = scheduler_tick;
    elapsed_ticks = 1U;
    if(control_imu_success_seen != 0U)
    {
        elapsed_ticks = scheduler_tick - control_imu_last_success_tick;
    }
    if(elapsed_ticks > 2U)
    {
        control_imu_mpu6500_invalidate();
        if((control_imu_attitude_enabled != 0U)
            && (control_imu_attitude_ready == 0U))
        {
            mpu6500_attitude_init();
        }
        control_imu_success_seen = 0U;
        return;
    }
    dt_ms = elapsed_ticks * CONTROL_SCHEDULER_PERIOD_MS;

    if(mpu6500_read(&sensor_data) != 0U)
    {
        control_imu_mpu6500_invalidate();
        if((control_imu_attitude_enabled != 0U)
            && (control_imu_attitude_ready == 0U))
        {
            mpu6500_attitude_init();
        }
        return;
    }

    next_data.accel_x_g = sensor_data.accel_g[0];
    next_data.roll_deg = 0.0F;
    next_data.pitch_deg = 0.0F;
    next_data.yaw_deg = 0.0F;
    next_data.yaw_continuous_deg = 0.0F;
    next_data.yaw_bias_deg_s = 0.0F;
    next_data.temperature_c = sensor_data.temperature_c;
    control_imu_update_count++;
    next_data.update_count = control_imu_update_count;
    next_data.calibration_sample_count = 0U;
    next_data.calibration_progress = 100U;
    next_data.valid = 1U;
    next_data.ready = 1U;
    if(control_imu_attitude_enabled != 0U)
    {
        if((mpu6500_attitude_update(&sensor_data, dt_ms) != 0U)
            || (mpu6500_attitude_get_data(&attitude_data) != 0U))
        {
            control_imu_mpu6500_invalidate();
            if(control_imu_attitude_ready == 0U)
            {
                mpu6500_attitude_init();
            }
            return;
        }

        next_data.roll_deg = attitude_data.roll_deg;
        next_data.pitch_deg = attitude_data.pitch_deg;
        next_data.yaw_deg = attitude_data.yaw_deg;
        next_data.yaw_continuous_deg = attitude_data.yaw_continuous_deg;
        next_data.yaw_bias_deg_s = attitude_data.yaw_bias_deg_s;
        next_data.temperature_c = attitude_data.temperature_c;
        next_data.calibration_sample_count =
            attitude_data.calibration_sample_count;
        next_data.calibration_progress = (uint8)(
            ((uint32)attitude_data.calibration_sample_count * 100U)
            / MPU6500_ATTITUDE_CALIBRATION_SAMPLES);
        if(next_data.calibration_progress > 100U)
        {
            next_data.calibration_progress = 100U;
        }
        next_data.ready = attitude_data.ready;
        control_imu_attitude_ready = attitude_data.ready;
    }
    control_imu_last_success_tick = scheduler_tick;
    control_imu_success_seen = 1U;

    primask = interrupt_global_disable();
    control_imu_data = next_data;
    interrupt_global_enable(primask);
}

/**
 * @brief Copy the latest coherent foreground-produced attitude snapshot.
 */
uint8 control_imu_mpu6500_get_data(control_imu_mpu6500_data_struct *data)
{
    uint32 primask;

    if(data == NULL)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    *data = control_imu_data;
    interrupt_global_enable(primask);

    return data->valid != 0U ? ZF_TRUE : ZF_FALSE;
}
