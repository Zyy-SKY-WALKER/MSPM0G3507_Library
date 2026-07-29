/**
 * @file    my_lib_mpu6500_attitude.c
 * @brief   MPU6500 Mahony roll/pitch and independent yaw integration.
 */

#include "my_lib_mpu6500_attitude.h"

#include <math.h>

#define MPU6500_ATTITUDE_RAD_TO_DEG       (57.2957795131F)
#define MPU6500_ATTITUDE_DEG_TO_RAD       (0.0174532925F)

static mpu6500_attitude_data_struct mpu6500_attitude_data;
static float mpu6500_quaternion[4];
static float mpu6500_integral_feedback_x;
static float mpu6500_integral_feedback_y;
static float mpu6500_yaw_previous_rate_deg_s;
static float mpu6500_yaw_temperature_slope_deg_s_per_c;
static float mpu6500_calibration_accel_sum[3];
static float mpu6500_calibration_yaw_rate_sum_deg_s;
static float mpu6500_calibration_temperature_sum_c;

/**
 * @brief Clear the consecutive stationary calibration window.
 */
static void mpu6500_attitude_clear_calibration(void)
{
    uint8 index;

    mpu6500_attitude_data.calibration_sample_count = 0U;
    mpu6500_calibration_yaw_rate_sum_deg_s = 0.0F;
    mpu6500_calibration_temperature_sum_c = 0.0F;
    for(index = 0U; index < 3U; index++)
    {
        mpu6500_calibration_accel_sum[index] = 0.0F;
    }
}

/**
 * @brief Check whether one sample is suitable for stationary calibration.
 */
static uint8 mpu6500_attitude_calibration_sample_is_stationary(
    const mpu6500_data_struct *sensor_data)
{
    float accel_norm = sqrtf(
        (sensor_data->accel_g[0] * sensor_data->accel_g[0])
        + (sensor_data->accel_g[1] * sensor_data->accel_g[1])
        + (sensor_data->accel_g[2] * sensor_data->accel_g[2]));

    return (uint8)(
        (fabsf(sensor_data->gyro_deg_s[0])
            <= MPU6500_ATTITUDE_CALIBRATION_GYRO_DPS)
        && (fabsf(sensor_data->gyro_deg_s[1])
            <= MPU6500_ATTITUDE_CALIBRATION_GYRO_DPS)
        && (fabsf(sensor_data->gyro_deg_s[2])
            <= MPU6500_ATTITUDE_CALIBRATION_GYRO_DPS)
        && (accel_norm >= MPU6500_ATTITUDE_CALIBRATION_ACCEL_MIN_G)
        && (accel_norm <= MPU6500_ATTITUDE_CALIBRATION_ACCEL_MAX_G));
}

/**
 * @brief Limit a floating-point value to a closed interval.
 */
static float mpu6500_attitude_limit(float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }

    return value;
}

/**
 * @brief Wrap one angle to the interval [-180, 180] degrees.
 */
static float mpu6500_attitude_wrap_deg(float angle_deg)
{
    while(angle_deg > 180.0F)
    {
        angle_deg -= 360.0F;
    }
    while(angle_deg < -180.0F)
    {
        angle_deg += 360.0F;
    }

    return angle_deg;
}

/**
 * @brief Normalize the Mahony quaternion, restoring identity if invalid.
 */
static void mpu6500_attitude_normalize_quaternion(void)
{
    float norm = sqrtf(
        (mpu6500_quaternion[0] * mpu6500_quaternion[0])
        + (mpu6500_quaternion[1] * mpu6500_quaternion[1])
        + (mpu6500_quaternion[2] * mpu6500_quaternion[2])
        + (mpu6500_quaternion[3] * mpu6500_quaternion[3]));

    if(norm <= 0.000001F)
    {
        mpu6500_quaternion[0] = 1.0F;
        mpu6500_quaternion[1] = 0.0F;
        mpu6500_quaternion[2] = 0.0F;
        mpu6500_quaternion[3] = 0.0F;
        return;
    }

    mpu6500_quaternion[0] /= norm;
    mpu6500_quaternion[1] /= norm;
    mpu6500_quaternion[2] /= norm;
    mpu6500_quaternion[3] /= norm;
}

/**
 * @brief Initialize the quaternion from stationary acceleration.
 */
static void mpu6500_attitude_initialize_roll_pitch(
    float accel_x,
    float accel_y,
    float accel_z)
{
    float roll_rad;
    float pitch_rad;
    float half_roll;
    float half_pitch;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    roll_rad = atan2f(accel_y, accel_z);
    pitch_rad = atan2f(-accel_x, sqrtf(
        (accel_y * accel_y) + (accel_z * accel_z)));
    half_roll = roll_rad * 0.5F;
    half_pitch = pitch_rad * 0.5F;
    cos_roll = cosf(half_roll);
    sin_roll = sinf(half_roll);
    cos_pitch = cosf(half_pitch);
    sin_pitch = sinf(half_pitch);

    mpu6500_quaternion[0] = cos_roll * cos_pitch;
    mpu6500_quaternion[1] = sin_roll * cos_pitch;
    mpu6500_quaternion[2] = cos_roll * sin_pitch;
    mpu6500_quaternion[3] = -sin_roll * sin_pitch;
    mpu6500_attitude_normalize_quaternion();
}

/**
 * @brief Publish roll and pitch from the Mahony quaternion.
 */
static void mpu6500_attitude_update_roll_pitch(void)
{
    float q0 = mpu6500_quaternion[0];
    float q1 = mpu6500_quaternion[1];
    float q2 = mpu6500_quaternion[2];
    float q3 = mpu6500_quaternion[3];
    float pitch_sine = 2.0F * ((q0 * q2) - (q3 * q1));

    pitch_sine = mpu6500_attitude_limit(pitch_sine, -1.0F, 1.0F);
    mpu6500_attitude_data.roll_deg = atan2f(
        2.0F * ((q0 * q1) + (q2 * q3)),
        1.0F - (2.0F * ((q1 * q1) + (q2 * q2))))
        * MPU6500_ATTITUDE_RAD_TO_DEG;
    mpu6500_attitude_data.pitch_deg = asinf(pitch_sine)
        * MPU6500_ATTITUDE_RAD_TO_DEG;
}

/**
 * @brief Run Mahony propagation with acceleration feedback on roll and pitch.
 */
static void mpu6500_attitude_update_mahony(
    const mpu6500_data_struct *sensor_data,
    float dt_s)
{
    float accel_norm;
    float accel_x;
    float accel_y;
    float accel_z;
    float q0 = mpu6500_quaternion[0];
    float q1 = mpu6500_quaternion[1];
    float q2 = mpu6500_quaternion[2];
    float q3 = mpu6500_quaternion[3];
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float error_x;
    float error_y;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float half_dt = dt_s * 0.5F;

    accel_norm = sqrtf(
        (sensor_data->accel_g[0] * sensor_data->accel_g[0])
        + (sensor_data->accel_g[1] * sensor_data->accel_g[1])
        + (sensor_data->accel_g[2] * sensor_data->accel_g[2]));
    gyro_x = sensor_data->gyro_deg_s[0] * MPU6500_ATTITUDE_DEG_TO_RAD;
    gyro_y = sensor_data->gyro_deg_s[1] * MPU6500_ATTITUDE_DEG_TO_RAD;
    gyro_z = sensor_data->gyro_deg_s[2] * MPU6500_ATTITUDE_DEG_TO_RAD;

    if(accel_norm > 0.000001F)
    {
        accel_x = sensor_data->accel_g[0] / accel_norm;
        accel_y = sensor_data->accel_g[1] / accel_norm;
        accel_z = sensor_data->accel_g[2] / accel_norm;
        gravity_x = 2.0F * ((q1 * q3) - (q0 * q2));
        gravity_y = 2.0F * ((q0 * q1) + (q2 * q3));
        gravity_z = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);
        error_x = (accel_y * gravity_z) - (accel_z * gravity_y);
        error_y = (accel_z * gravity_x) - (accel_x * gravity_z);

        mpu6500_integral_feedback_x += MPU6500_ATTITUDE_MAHONY_KI
            * error_x * dt_s;
        mpu6500_integral_feedback_y += MPU6500_ATTITUDE_MAHONY_KI
            * error_y * dt_s;
        gyro_x += (MPU6500_ATTITUDE_MAHONY_KP * error_x)
            + mpu6500_integral_feedback_x;
        gyro_y += (MPU6500_ATTITUDE_MAHONY_KP * error_y)
            + mpu6500_integral_feedback_y;
    }

    mpu6500_quaternion[0] += (-q1 * gyro_x - q2 * gyro_y - q3 * gyro_z)
        * half_dt;
    mpu6500_quaternion[1] += (q0 * gyro_x + q2 * gyro_z - q3 * gyro_y)
        * half_dt;
    mpu6500_quaternion[2] += (q0 * gyro_y - q1 * gyro_z + q3 * gyro_x)
        * half_dt;
    mpu6500_quaternion[3] += (q0 * gyro_z + q1 * gyro_y - q2 * gyro_x)
        * half_dt;
    mpu6500_attitude_normalize_quaternion();
    mpu6500_attitude_update_roll_pitch();
}

/**
 * @brief Integrate yaw independently of Mahony acceleration feedback.
 */
static void mpu6500_attitude_update_yaw(
    const mpu6500_data_struct *sensor_data,
    float dt_s)
{
    float yaw_rate_deg_s;
    float yaw_delta_deg;

    mpu6500_attitude_data.yaw_temperature_compensation_deg_s =
        mpu6500_yaw_temperature_slope_deg_s_per_c
        * (sensor_data->temperature_c
            - mpu6500_attitude_data.calibration_temperature_c);
    yaw_rate_deg_s = sensor_data->gyro_deg_s[2]
        - mpu6500_attitude_data.yaw_bias_deg_s
        - mpu6500_attitude_data.yaw_temperature_compensation_deg_s;
    if(fabsf(yaw_rate_deg_s) < MPU6500_ATTITUDE_YAW_DEADBAND_DPS)
    {
        yaw_rate_deg_s = 0.0F;
    }
    yaw_rate_deg_s = mpu6500_attitude_limit(
        yaw_rate_deg_s,
        -MPU6500_ATTITUDE_YAW_RATE_LIMIT_DPS,
        MPU6500_ATTITUDE_YAW_RATE_LIMIT_DPS);
    yaw_delta_deg = 0.5F * (mpu6500_yaw_previous_rate_deg_s + yaw_rate_deg_s)
        * dt_s;
    yaw_delta_deg = mpu6500_attitude_limit(
        yaw_delta_deg,
        -MPU6500_ATTITUDE_YAW_FRAME_LIMIT_DEG,
        MPU6500_ATTITUDE_YAW_FRAME_LIMIT_DEG);
    mpu6500_yaw_previous_rate_deg_s = yaw_rate_deg_s;
    mpu6500_attitude_data.yaw_continuous_deg += yaw_delta_deg;
    mpu6500_attitude_data.yaw_deg = mpu6500_attitude_wrap_deg(
        mpu6500_attitude_data.yaw_continuous_deg);
}

/**
 * @brief Reset calibration, quaternion, and independent yaw integration state.
 */
void mpu6500_attitude_init(void)
{
    mpu6500_attitude_data.roll_deg = 0.0F;
    mpu6500_attitude_data.pitch_deg = 0.0F;
    mpu6500_attitude_data.yaw_deg = 0.0F;
    mpu6500_attitude_data.yaw_continuous_deg = 0.0F;
    mpu6500_attitude_data.yaw_bias_deg_s = 0.0F;
    mpu6500_attitude_data.yaw_temperature_compensation_deg_s = 0.0F;
    mpu6500_attitude_data.temperature_c = 0.0F;
    mpu6500_attitude_data.calibration_temperature_c = 0.0F;
    mpu6500_attitude_data.dt_ms = 0U;
    mpu6500_attitude_data.update_count = 0U;
    mpu6500_attitude_data.ready = 0U;
    mpu6500_quaternion[0] = 1.0F;
    mpu6500_quaternion[1] = 0.0F;
    mpu6500_quaternion[2] = 0.0F;
    mpu6500_quaternion[3] = 0.0F;
    mpu6500_integral_feedback_x = 0.0F;
    mpu6500_integral_feedback_y = 0.0F;
    mpu6500_yaw_previous_rate_deg_s = 0.0F;
    mpu6500_attitude_clear_calibration();
}

/**
 * @brief Process one successful MPU6500 sample.
 * @param sensor_data Converted MPU6500 acceleration, angular rate, and temperature.
 * @param dt_ms Measured elapsed time since the previous successful sample.
 * @return 0 on success, otherwise 1 when sensor_data is NULL.
 */
uint8 mpu6500_attitude_update(
    const mpu6500_data_struct *sensor_data,
    uint32 dt_ms)
{
    float dt_s;

    if(sensor_data == NULL)
    {
        return 1U;
    }

    dt_ms = (uint32)mpu6500_attitude_limit(
        (float)dt_ms,
        (float)MPU6500_ATTITUDE_DT_MIN_MS,
        (float)MPU6500_ATTITUDE_DT_MAX_MS);
    dt_s = (float)dt_ms / 1000.0F;
    mpu6500_attitude_data.dt_ms = (uint16)dt_ms;
    mpu6500_attitude_data.temperature_c = sensor_data->temperature_c;
    mpu6500_attitude_data.update_count++;

    if(mpu6500_attitude_data.ready == 0U)
    {
        uint16 sample_count;

        if(mpu6500_attitude_calibration_sample_is_stationary(sensor_data)
            == 0U)
        {
            mpu6500_attitude_clear_calibration();
            return 0U;
        }

        mpu6500_calibration_accel_sum[0] += sensor_data->accel_g[0];
        mpu6500_calibration_accel_sum[1] += sensor_data->accel_g[1];
        mpu6500_calibration_accel_sum[2] += sensor_data->accel_g[2];
        mpu6500_calibration_yaw_rate_sum_deg_s += sensor_data->gyro_deg_s[2];
        mpu6500_calibration_temperature_sum_c += sensor_data->temperature_c;
        mpu6500_attitude_data.calibration_sample_count++;
        sample_count = mpu6500_attitude_data.calibration_sample_count;

        if(sample_count >= MPU6500_ATTITUDE_CALIBRATION_SAMPLES)
        {
            float sample_count_float = (float)sample_count;

            mpu6500_attitude_data.yaw_bias_deg_s =
                mpu6500_calibration_yaw_rate_sum_deg_s / sample_count_float;
            mpu6500_attitude_data.calibration_temperature_c =
                mpu6500_calibration_temperature_sum_c / sample_count_float;
            mpu6500_attitude_initialize_roll_pitch(
                mpu6500_calibration_accel_sum[0] / sample_count_float,
                mpu6500_calibration_accel_sum[1] / sample_count_float,
                mpu6500_calibration_accel_sum[2] / sample_count_float);
            mpu6500_attitude_update_roll_pitch();
            mpu6500_attitude_data.ready = 1U;
        }
        return 0U;
    }

    mpu6500_attitude_update_mahony(sensor_data, dt_s);
    mpu6500_attitude_update_yaw(sensor_data, dt_s);
    return 0U;
}

/**
 * @brief Copy the latest attitude and calibration state.
 * @param data Destination state structure.
 * @return 0 on success, otherwise 1 when data is NULL.
 */
uint8 mpu6500_attitude_get_data(mpu6500_attitude_data_struct *data)
{
    if(data == NULL)
    {
        return 1U;
    }

    *data = mpu6500_attitude_data;
    return 0U;
}

/**
 * @brief Clear the independent yaw angle without recalibrating the gyro bias.
 */
void mpu6500_attitude_reset_yaw(void)
{
    mpu6500_attitude_data.yaw_deg = 0.0F;
    mpu6500_attitude_data.yaw_continuous_deg = 0.0F;
    mpu6500_yaw_previous_rate_deg_s = 0.0F;
}

/**
 * @brief Configure linear yaw-bias temperature compensation.
 * @param slope_deg_s_per_c Bias slope relative to calibration temperature.
 */
void mpu6500_attitude_set_yaw_temperature_slope(float slope_deg_s_per_c)
{
    mpu6500_yaw_temperature_slope_deg_s_per_c = slope_deg_s_per_c;
}
