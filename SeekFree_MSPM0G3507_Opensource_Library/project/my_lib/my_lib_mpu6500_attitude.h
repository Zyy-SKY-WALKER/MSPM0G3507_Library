/**
 * @file    my_lib_mpu6500_attitude.h
 * @brief   MPU6500 Mahony roll/pitch and independent yaw integration.
 */

#ifndef MY_LIB_MPU6500_ATTITUDE_H
#define MY_LIB_MPU6500_ATTITUDE_H

#include "my_lib_mpu6500.h"

/** @brief Number of stationary samples used for yaw bias calibration. */
#define MPU6500_ATTITUDE_CALIBRATION_SAMPLES    (1000U)
/** @brief Largest gyro magnitude accepted during stationary calibration. */
#define MPU6500_ATTITUDE_CALIBRATION_GYRO_DPS   (8.0F)
/** @brief Smallest acceleration magnitude accepted during calibration. */
#define MPU6500_ATTITUDE_CALIBRATION_ACCEL_MIN_G (0.8F)
/** @brief Largest acceleration magnitude accepted during calibration. */
#define MPU6500_ATTITUDE_CALIBRATION_ACCEL_MAX_G (1.2F)
/** @brief Mahony proportional feedback gain for roll and pitch. */
#define MPU6500_ATTITUDE_MAHONY_KP              (2.0F)
/** @brief Mahony integral feedback gain for roll and pitch. */
#define MPU6500_ATTITUDE_MAHONY_KI              (0.005F)
/** @brief Smallest valid elapsed time between samples. */
#define MPU6500_ATTITUDE_DT_MIN_MS              (10U)
/** @brief Largest valid elapsed time between samples. */
#define MPU6500_ATTITUDE_DT_MAX_MS              (25U)
/** @brief Independent yaw-rate deadband. */
#define MPU6500_ATTITUDE_YAW_DEADBAND_DPS       (0.25F)
/** @brief Independent yaw-rate magnitude limit. */
#define MPU6500_ATTITUDE_YAW_RATE_LIMIT_DPS     (120.0F)
/** @brief Independent yaw increment magnitude limit. */
#define MPU6500_ATTITUDE_YAW_FRAME_LIMIT_DEG    (0.8F)

/** @brief Latest MPU6500 attitude, calibration, and filter diagnostics. */
typedef struct
{
    /** Roll about the vehicle forward X axis, in degrees. */
    float roll_deg;
    /** Pitch about the vehicle left Y axis, in degrees. */
    float pitch_deg;
    /** Wrapped independent yaw in the interval [-180, 180] degrees. */
    float yaw_deg;
    /** Independent yaw accumulated without wrapping, in degrees. */
    float yaw_continuous_deg;
    /** Startup stationary Z-axis gyro bias, in degrees per second. */
    float yaw_bias_deg_s;
    /** Current yaw temperature compensation, in degrees per second. */
    float yaw_temperature_compensation_deg_s;
    /** Current MPU6500 temperature, in degrees Celsius. */
    float temperature_c;
    /** Temperature averaged during startup calibration, in degrees Celsius. */
    float calibration_temperature_c;
    /** dt used by the most recent filter update, after limiting. */
    uint16 dt_ms;
    /** Number of valid startup calibration samples collected. */
    uint16 calibration_sample_count;
    /** Number of valid sensor samples accepted since initialization. */
    uint32 update_count;
    /** Nonzero after the 1000-sample startup calibration completes. */
    uint8 ready;
} mpu6500_attitude_data_struct;

/**
 * @brief Reset calibration, quaternion, and independent yaw integration state.
 */
void mpu6500_attitude_init(void);

/**
 * @brief Process one successful MPU6500 sample.
 * @param sensor_data Converted MPU6500 acceleration, angular rate, and temperature.
 * @param dt_ms Measured elapsed time since the previous successful sample.
 * @return 0 on success, otherwise 1 when sensor_data is NULL.
 * @note The board must remain stationary until ready becomes nonzero.
 */
uint8 mpu6500_attitude_update(
    const mpu6500_data_struct *sensor_data,
    uint32 dt_ms);

/**
 * @brief Copy the latest attitude and calibration state.
 * @param data Destination state structure.
 * @return 0 on success, otherwise 1 when data is NULL.
 */
uint8 mpu6500_attitude_get_data(mpu6500_attitude_data_struct *data);

/**
 * @brief Clear the independent yaw angle without recalibrating the gyro bias.
 */
void mpu6500_attitude_reset_yaw(void);

/**
 * @brief Configure linear yaw-bias temperature compensation.
 * @param slope_deg_s_per_c Bias slope relative to calibration temperature.
 * @note The initial slope is zero until thermal characterization is available.
 */
void mpu6500_attitude_set_yaw_temperature_slope(float slope_deg_s_per_c);

#endif
