/**
 * @file    gimbal_stepper.c
 * @brief   Reusable dual-axis STEP/DIR gimbal controller.
 */

#include "gimbal_stepper.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "drive_geometry.h"
#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"

#define GIMBAL_YAW_STEP_PIN                     (B4)
#define GIMBAL_PITCH_STEP_PIN                   (B5)
#define GIMBAL_YAW_DIR_PIN                      (B8)
#define GIMBAL_PITCH_DIR_PIN                    (B9)
#define GIMBAL_LASER_PIN                        (B13)

#define GIMBAL_SELECT_KEY_PIN                   (A30)
#define GIMBAL_STOP_KEY_PIN                     (A31)
#define GIMBAL_NEGATIVE_KEY_PIN                 (B0)
#define GIMBAL_POSITIVE_KEY_PIN                 (B1)

#define GIMBAL_PIT                              (PIT_TIM_G6)
#define GIMBAL_PIT_IRQ                          (TIMG6_INT_IRQn)
#define GIMBAL_PIT_IRQ_PRIORITY                 (1U)
#define GIMBAL_TICK_US                          \
    (GIMBAL_CONFIG_PULSE_TICK_US)
#define GIMBAL_TICK_HZ                          \
    (GIMBAL_CONFIG_PULSE_ENGINE_HZ)
#define GIMBAL_RATE_SCALE                       (1000U)
#define GIMBAL_MAX_RATE_MILLI_STEPS_S           \
    ((int32)((GIMBAL_TICK_HZ / 2U) * GIMBAL_RATE_SCALE))
#define GIMBAL_CONTROL_PERIOD_MS                \
    (GIMBAL_CONFIG_CONTROL_PERIOD_MS)
#define GIMBAL_MILLISECOND_DIVIDER_TICKS        \
    (1000U / GIMBAL_TICK_US)
#define GIMBAL_RATE_DIVIDER_TICKS               \
    ((GIMBAL_TICK_HZ * GIMBAL_CONTROL_PERIOD_MS) / 1000U)
#define GIMBAL_DEBOUNCE_MS                      (20U)
#define GIMBAL_ZERO_HOLD_MS                     (1000U)
#define GIMBAL_DIRECTION_SETTLE_TICKS           (1U)

#define GIMBAL_YAW_JOG_SPEED_DEG_S              \
    (GIMBAL_CONFIG_YAW_JOG_SPEED_DEG_S)
#define GIMBAL_PITCH_JOG_SPEED_DEG_S            \
    (GIMBAL_CONFIG_PITCH_JOG_SPEED_DEG_S)
#define GIMBAL_CALIBRATE_SPEED_DEG_S            \
    (GIMBAL_CONFIG_CALIBRATE_SPEED_DEG_S)
#define GIMBAL_POSITION_SPEED_DEG_S             \
    (GIMBAL_CONFIG_POSITION_SPEED_DEG_S)
#define GIMBAL_ACCEL_DEG_S2                     \
    (GIMBAL_CONFIG_ACCELERATION_DEG_S2)
#define GIMBAL_POSITION_GAIN_MILLI_RATE         \
    (GIMBAL_CONFIG_POSITION_GAIN_MILLI_RATE)

#define GIMBAL_YAW_JOG_RATE_MILLI_STEPS_S       \
    ((int32)(((uint64)GIMBAL_YAW_JOG_SPEED_DEG_S \
        * GIMBAL_STEPPER_STEPS_PER_REVOLUTION \
        * GIMBAL_RATE_SCALE) / 360U))
#define GIMBAL_PITCH_JOG_RATE_MILLI_STEPS_S     \
    ((int32)(((uint64)GIMBAL_PITCH_JOG_SPEED_DEG_S \
        * GIMBAL_STEPPER_STEPS_PER_REVOLUTION \
        * GIMBAL_RATE_SCALE) / 360U))
#define GIMBAL_CALIBRATE_RATE_MILLI_STEPS_S     \
    ((int32)(((uint64)GIMBAL_CALIBRATE_SPEED_DEG_S \
        * GIMBAL_STEPPER_STEPS_PER_REVOLUTION \
        * GIMBAL_RATE_SCALE) / 360U))
#define GIMBAL_POSITION_RATE_MILLI_STEPS_S      \
    ((int32)(((uint64)GIMBAL_POSITION_SPEED_DEG_S \
        * GIMBAL_STEPPER_STEPS_PER_REVOLUTION \
        * GIMBAL_RATE_SCALE) / 360U))
#define GIMBAL_RATE_DELTA_MILLI_STEPS_S         \
    ((int32)(((uint64)GIMBAL_ACCEL_DEG_S2 \
        * GIMBAL_STEPPER_STEPS_PER_REVOLUTION \
        * GIMBAL_RATE_SCALE \
        * GIMBAL_CONTROL_PERIOD_MS) \
        / (360U * 1000U)))

#define GIMBAL_YAW_CALIBRATE_TRAVEL_STEPS       \
    (GIMBAL_CONFIG_YAW_CALIBRATE_STEPS)
#define GIMBAL_PITCH_CALIBRATE_TRAVEL_STEPS     \
    (GIMBAL_CONFIG_PITCH_CALIBRATE_STEPS)
#define GIMBAL_YAW_POSITIVE_DIR_LEVEL           \
    (GIMBAL_CONFIG_YAW_POSITIVE_DIR_LEVEL)
#define GIMBAL_PITCH_POSITIVE_DIR_LEVEL         \
    (GIMBAL_CONFIG_PITCH_POSITIVE_DIR_LEVEL)
#define GIMBAL_LASER_ACTIVE_LEVEL               \
    (GIMBAL_CONFIG_LASER_ACTIVE_LEVEL)
#define GIMBAL_DEG_TO_RAD                       \
    (DRIVE_PI / 180.0F)
#define GIMBAL_RAD_TO_DEG                       \
    (180.0F / DRIVE_PI)
#define GIMBAL_TWO_PI                           \
    (2.0F * DRIVE_PI)
#define GIMBAL_FEEDFORWARD_MIN_NORM             (0.001F)
#define GIMBAL_FEEDFORWARD_MIN_DETERMINANT      (0.001F)

#if ((GIMBAL_CONFIG_POSITION_SPEED_DEG_S \
        * GIMBAL_CONFIG_STEPS_PER_REVOLUTION) \
    > (180U * GIMBAL_CONFIG_PULSE_ENGINE_HZ))
#error Gimbal position speed exceeds the STEP pulse engine capacity.
#endif
#if ((GIMBAL_CONFIG_YAW_JOG_SPEED_DEG_S \
        * GIMBAL_CONFIG_STEPS_PER_REVOLUTION) \
    > (180U * GIMBAL_CONFIG_PULSE_ENGINE_HZ))
#error Gimbal yaw jog speed exceeds the STEP pulse engine capacity.
#endif
#if ((GIMBAL_CONFIG_PITCH_JOG_SPEED_DEG_S \
        * GIMBAL_CONFIG_STEPS_PER_REVOLUTION) \
    > (180U * GIMBAL_CONFIG_PULSE_ENGINE_HZ))
#error Gimbal pitch jog speed exceeds the STEP pulse engine capacity.
#endif
#if ((GIMBAL_CONFIG_CALIBRATE_SPEED_DEG_S \
        * GIMBAL_CONFIG_STEPS_PER_REVOLUTION) \
    > (180U * GIMBAL_CONFIG_PULSE_ENGINE_HZ))
#error Gimbal calibrate speed exceeds the STEP pulse engine capacity.
#endif
#if ((1000U % GIMBAL_CONFIG_PULSE_TICK_US) != 0U)
#error Gimbal pulse tick must divide one millisecond exactly.
#endif
#if (((GIMBAL_CONFIG_PULSE_ENGINE_HZ \
        * GIMBAL_CONFIG_CONTROL_PERIOD_MS) % 1000U) != 0U)
#error Gimbal pulse rate must divide the control period exactly.
#endif
#if ((GIMBAL_CONFIG_PULSE_ENGINE_HZ \
        * GIMBAL_CONFIG_PULSE_TICK_US) != 1000000U)
#error Gimbal pulse rate and tick period are inconsistent.
#endif

typedef enum
{
    GIMBAL_CONTROL_IDLE = 0,
    GIMBAL_CONTROL_JOG,
    GIMBAL_CONTROL_STOPPING,
    GIMBAL_CONTROL_POSITION,
} gimbal_control_mode_enum;

typedef struct
{
    volatile uint16 mismatch_ms;
    volatile uint8 pressed;
} gimbal_key_struct;

typedef struct
{
    uint16 select_hold_ms;
    uint8 select_pressed;
    uint8 select_released;
    uint8 negative_pressed;
    uint8 positive_pressed;
} gimbal_key_snapshot_struct;

typedef struct
{
    gpio_pin_enum step_pin;
    gpio_pin_enum dir_pin;
    int32 min_position_steps;
    int32 max_position_steps;
    int32 calibrate_travel_steps;
    uint8 positive_dir_level;
    volatile int32 target_position_steps;
    volatile int32 target_rate_milli_steps_s;
    volatile int32 current_rate_milli_steps_s;
    volatile int32 command_rate_milli_steps_s;
    volatile int32 position_steps;
    volatile uint32 phase_accumulator;
    volatile uint32 phase_increment;
    volatile uint8 direction_positive;
    volatile uint8 direction_settle_ticks;
    volatile uint8 pulse_pending;
    volatile uint8 pulse_high;
    volatile uint8 zero_valid;
} gimbal_axis_struct;

typedef struct
{
    float x;
    float y;
    float z;
} gimbal_vector3_struct;

typedef struct
{
    float value[3][3];
} gimbal_matrix3_struct;

static gimbal_axis_struct gimbal_axes[GIMBAL_STEPPER_AXIS_COUNT];
static gimbal_key_struct gimbal_select_key;
static gimbal_key_struct gimbal_negative_key;
static gimbal_key_struct gimbal_positive_key;
static gimbal_stepper_axis_enum gimbal_selected_axis;
static volatile uint16 gimbal_select_hold_ms;
static volatile uint8 gimbal_select_long_handled;
static volatile uint8 gimbal_select_release_event;
static volatile uint8 gimbal_select_previous_pressed;
static uint8 gimbal_select_suppressed;
static volatile uint8 gimbal_stop_latched;
static uint8 gimbal_stop_reported;
static volatile gimbal_control_mode_enum gimbal_control_mode;
static volatile uint16 gimbal_pending_ms;
static volatile uint8 gimbal_millisecond_divider;
static volatile uint8 gimbal_rate_tick_divider;
static volatile uint16 gimbal_laser_settle_ms;
static volatile gimbal_target_mode_enum gimbal_target_mode;
static volatile float gimbal_target_phase_rad;
static volatile gimbal_feedforward_solution_struct
    gimbal_feedforward_solution;
static gimbal_stepper_log_callback gimbal_log_callback;

/**
 * @brief Send one log line through the global debug printf route.
 */
static void gimbal_default_log(const char *message)
{
    if(message != NULL)
    {
        printf("%s", message);
    }
}

/**
 * @brief Send one foreground log message to the configured sink.
 */
static void gimbal_log(const char *message)
{
    gimbal_stepper_log_callback callback = gimbal_log_callback;

    if((callback != NULL) && (message != NULL))
    {
        callback(message);
    }
}

/**
 * @brief Clamp one floating-point value to a closed interval.
 */
static float gimbal_clamp_float(
    float value,
    float minimum,
    float maximum)
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
 * @brief Check that one floating-point value is finite.
 */
static uint8 gimbal_float_is_valid(float value)
{
    return (uint8)((value == value)
        && (value >= -FLT_MAX)
        && (value <= FLT_MAX));
}

/**
 * @brief Wrap an angle in radians to the interval [-pi, pi].
 */
static float gimbal_wrap_angle_rad(float angle_rad)
{
    angle_rad = fmodf(angle_rad, GIMBAL_TWO_PI);
    if(angle_rad > DRIVE_PI)
    {
        angle_rad -= GIMBAL_TWO_PI;
    }
    else if(angle_rad < -DRIVE_PI)
    {
        angle_rad += GIMBAL_TWO_PI;
    }
    return angle_rad;
}

/**
 * @brief Wrap an angle in degrees to the interval [0, 360).
 */
static float gimbal_wrap_angle_positive_deg(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0F);
    if(angle_deg < 0.0F)
    {
        angle_deg += 360.0F;
    }
    return angle_deg;
}

/**
 * @brief Add two three-dimensional vectors.
 */
static gimbal_vector3_struct gimbal_vector_add(
    gimbal_vector3_struct left,
    gimbal_vector3_struct right)
{
    gimbal_vector3_struct result;

    result.x = left.x + right.x;
    result.y = left.y + right.y;
    result.z = left.z + right.z;
    return result;
}

/**
 * @brief Subtract two three-dimensional vectors.
 */
static gimbal_vector3_struct gimbal_vector_subtract(
    gimbal_vector3_struct left,
    gimbal_vector3_struct right)
{
    gimbal_vector3_struct result;

    result.x = left.x - right.x;
    result.y = left.y - right.y;
    result.z = left.z - right.z;
    return result;
}

/**
 * @brief Scale one three-dimensional vector.
 */
static gimbal_vector3_struct gimbal_vector_scale(
    gimbal_vector3_struct vector,
    float scale)
{
    gimbal_vector3_struct result;

    result.x = vector.x * scale;
    result.y = vector.y * scale;
    result.z = vector.z * scale;
    return result;
}

/**
 * @brief Return the dot product of two vectors.
 */
static float gimbal_vector_dot(
    gimbal_vector3_struct left,
    gimbal_vector3_struct right)
{
    return (left.x * right.x)
        + (left.y * right.y)
        + (left.z * right.z);
}

/**
 * @brief Return the squared norm of one vector.
 */
static float gimbal_vector_norm_squared(
    gimbal_vector3_struct vector)
{
    return gimbal_vector_dot(vector, vector);
}

/**
 * @brief Normalize one vector when its norm is usable.
 */
static uint8 gimbal_vector_normalize(
    gimbal_vector3_struct vector,
    gimbal_vector3_struct *normalized)
{
    float norm_squared;
    float inverse_norm;

    if(normalized == NULL)
    {
        return 0U;
    }

    norm_squared = gimbal_vector_norm_squared(vector);
    if(norm_squared <= GIMBAL_FEEDFORWARD_MIN_NORM)
    {
        return 0U;
    }

    inverse_norm = 1.0F / sqrtf(norm_squared);
    *normalized = gimbal_vector_scale(vector, inverse_norm);
    return 1U;
}

/**
 * @brief Return a 3x3 identity matrix.
 */
static gimbal_matrix3_struct gimbal_matrix_identity(void)
{
    gimbal_matrix3_struct matrix =
    {
        {
            {1.0F, 0.0F, 0.0F},
            {0.0F, 1.0F, 0.0F},
            {0.0F, 0.0F, 1.0F},
        }
    };

    return matrix;
}

/**
 * @brief Multiply two 3x3 matrices.
 */
static gimbal_matrix3_struct gimbal_matrix_multiply(
    gimbal_matrix3_struct left,
    gimbal_matrix3_struct right)
{
    gimbal_matrix3_struct result = gimbal_matrix_identity();
    uint8 row;
    uint8 column;
    uint8 index;

    for(row = 0U; row < 3U; row++)
    {
        for(column = 0U; column < 3U; column++)
        {
            result.value[row][column] = 0.0F;
            for(index = 0U; index < 3U; index++)
            {
                result.value[row][column] +=
                    left.value[row][index] * right.value[index][column];
            }
        }
    }

    return result;
}

/**
 * @brief Transform one vector by a 3x3 matrix.
 */
static gimbal_vector3_struct gimbal_matrix_transform(
    gimbal_matrix3_struct matrix,
    gimbal_vector3_struct vector)
{
    gimbal_vector3_struct result;

    result.x = (matrix.value[0][0] * vector.x)
        + (matrix.value[0][1] * vector.y)
        + (matrix.value[0][2] * vector.z);
    result.y = (matrix.value[1][0] * vector.x)
        + (matrix.value[1][1] * vector.y)
        + (matrix.value[1][2] * vector.z);
    result.z = (matrix.value[2][0] * vector.x)
        + (matrix.value[2][1] * vector.y)
        + (matrix.value[2][2] * vector.z);
    return result;
}

/**
 * @brief Return the transpose-transformed vector for a rotation matrix.
 */
static gimbal_vector3_struct gimbal_matrix_inverse_transform(
    gimbal_matrix3_struct matrix,
    gimbal_vector3_struct vector)
{
    gimbal_vector3_struct result;

    result.x = (matrix.value[0][0] * vector.x)
        + (matrix.value[1][0] * vector.y)
        + (matrix.value[2][0] * vector.z);
    result.y = (matrix.value[0][1] * vector.x)
        + (matrix.value[1][1] * vector.y)
        + (matrix.value[2][1] * vector.z);
    result.z = (matrix.value[0][2] * vector.x)
        + (matrix.value[1][2] * vector.y)
        + (matrix.value[2][2] * vector.z);
    return result;
}

/**
 * @brief Build a roll rotation matrix.
 */
static gimbal_matrix3_struct gimbal_matrix_roll(float angle_rad)
{
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    gimbal_matrix3_struct matrix = gimbal_matrix_identity();

    matrix.value[1][1] = cosine;
    matrix.value[1][2] = -sine;
    matrix.value[2][1] = sine;
    matrix.value[2][2] = cosine;
    return matrix;
}

/**
 * @brief Build a pitch rotation matrix.
 */
static gimbal_matrix3_struct gimbal_matrix_pitch(float angle_rad)
{
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    gimbal_matrix3_struct matrix = gimbal_matrix_identity();

    matrix.value[0][0] = cosine;
    matrix.value[0][2] = sine;
    matrix.value[2][0] = -sine;
    matrix.value[2][2] = cosine;
    return matrix;
}

/**
 * @brief Build a yaw rotation matrix.
 */
static gimbal_matrix3_struct gimbal_matrix_yaw(float angle_rad)
{
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    gimbal_matrix3_struct matrix = gimbal_matrix_identity();

    matrix.value[0][0] = cosine;
    matrix.value[0][1] = -sine;
    matrix.value[1][0] = sine;
    matrix.value[1][1] = cosine;
    return matrix;
}

/**
 * @brief Build a ZYX body-to-world rotation from Euler angles.
 */
static gimbal_matrix3_struct gimbal_body_rotation(
    const gimbal_feedforward_pose_struct *pose)
{
    gimbal_matrix3_struct roll;
    gimbal_matrix3_struct pitch;
    gimbal_matrix3_struct yaw;

    roll = gimbal_matrix_roll(pose->roll_deg * GIMBAL_DEG_TO_RAD);
    pitch = gimbal_matrix_pitch(pose->pitch_deg * GIMBAL_DEG_TO_RAD);
    yaw = gimbal_matrix_yaw(pose->heading_rad);
    return gimbal_matrix_multiply(
        yaw,
        gimbal_matrix_multiply(pitch, roll));
}

/**
 * @brief Build a fixed rotation from configured Euler installation offsets.
 */
static gimbal_matrix3_struct gimbal_configured_rotation(
    float roll_deg,
    float pitch_deg,
    float yaw_deg)
{
    gimbal_matrix3_struct roll;
    gimbal_matrix3_struct pitch;
    gimbal_matrix3_struct yaw;

    roll = gimbal_matrix_roll(roll_deg * GIMBAL_DEG_TO_RAD);
    pitch = gimbal_matrix_pitch(pitch_deg * GIMBAL_DEG_TO_RAD);
    yaw = gimbal_matrix_yaw(yaw_deg * GIMBAL_DEG_TO_RAD);
    return gimbal_matrix_multiply(
        yaw,
        gimbal_matrix_multiply(pitch, roll));
}

/**
 * @brief Return the configured target point in world coordinates.
 */
static gimbal_vector3_struct gimbal_get_target_point(void)
{
    gimbal_vector3_struct point;
    float phase_rad;
    float cosine;
    float sine;

    point.x = GIMBAL_CONFIG_TARGET_CENTER_X_MM;
    point.y = GIMBAL_CONFIG_TARGET_CENTER_Y_MM;
    point.z = GIMBAL_CONFIG_TARGET_CENTER_Z_MM;

    if(gimbal_target_mode != GIMBAL_TARGET_CIRCLE)
    {
        return point;
    }

    phase_rad = gimbal_wrap_angle_rad(gimbal_target_phase_rad);
    cosine = cosf(phase_rad);
    sine = sinf(phase_rad);
    point.x += GIMBAL_CONFIG_TARGET_CIRCLE_RADIUS_MM
        * ((GIMBAL_CONFIG_TARGET_AXIS_X_X * cosine)
            + (GIMBAL_CONFIG_TARGET_AXIS_Y_X * sine));
    point.y += GIMBAL_CONFIG_TARGET_CIRCLE_RADIUS_MM
        * ((GIMBAL_CONFIG_TARGET_AXIS_X_Y * cosine)
            + (GIMBAL_CONFIG_TARGET_AXIS_Y_Y * sine));
    point.z += GIMBAL_CONFIG_TARGET_CIRCLE_RADIUS_MM
        * ((GIMBAL_CONFIG_TARGET_AXIS_X_Z * cosine)
            + (GIMBAL_CONFIG_TARGET_AXIS_Y_Z * sine));
    return point;
}

/**
 * @brief Calculate the laser origin and direction for two joint angles.
 * @param pose Vehicle pose in the world frame.
 * @param yaw_deg Command yaw angle, zero is the mechanical minimum.
 * @param pitch_deg Pitch angle, zero is horizontal and positive is up.
 * @param origin_world Destination laser origin.
 * @param direction_world Destination unit laser direction.
 * @return 1 when the direction is valid.
 */
static uint8 gimbal_compute_laser_ray(
    const gimbal_feedforward_pose_struct *pose,
    float yaw_deg,
    float pitch_deg,
    gimbal_vector3_struct *origin_world,
    gimbal_vector3_struct *direction_world)
{
    gimbal_matrix3_struct body_rotation;
    gimbal_matrix3_struct pan_mount_rotation;
    gimbal_matrix3_struct pan_rotation;
    gimbal_matrix3_struct tilt_mount_rotation;
    gimbal_matrix3_struct pitch_rotation;
    gimbal_matrix3_struct laser_mount_rotation;
    gimbal_matrix3_struct pan_world_rotation;
    gimbal_matrix3_struct tilt_world_rotation;
    gimbal_matrix3_struct laser_world_rotation;
    gimbal_vector3_struct vehicle_origin;
    gimbal_vector3_struct pan_origin_body;
    gimbal_vector3_struct tilt_origin_pan;
    gimbal_vector3_struct laser_origin_tilt;
    gimbal_vector3_struct laser_direction_tilt;

    if((pose == NULL)
        || (origin_world == NULL)
        || (direction_world == NULL))
    {
        return 0U;
    }

    body_rotation = gimbal_body_rotation(pose);
    pan_mount_rotation = gimbal_configured_rotation(
        GIMBAL_CONFIG_PAN_MOUNT_ROLL_DEG,
        GIMBAL_CONFIG_PAN_MOUNT_PITCH_DEG,
        GIMBAL_CONFIG_PAN_MOUNT_YAW_DEG);
    pan_rotation = gimbal_matrix_yaw(
        GIMBAL_CONFIG_YAW_KINEMATIC_SIGN
            * yaw_deg * GIMBAL_DEG_TO_RAD);
    tilt_mount_rotation = gimbal_configured_rotation(
        GIMBAL_CONFIG_TILT_MOUNT_ROLL_DEG,
        GIMBAL_CONFIG_TILT_MOUNT_PITCH_DEG,
        GIMBAL_CONFIG_TILT_MOUNT_YAW_DEG);
    /* Positive pitch rotates the horizontal ray upward. */
    pitch_rotation = gimbal_matrix_pitch(
        -pitch_deg * GIMBAL_DEG_TO_RAD);
    laser_mount_rotation = gimbal_configured_rotation(
        GIMBAL_CONFIG_LASER_ROLL_OFFSET_DEG,
        GIMBAL_CONFIG_LASER_PITCH_OFFSET_DEG,
        GIMBAL_CONFIG_LASER_YAW_OFFSET_DEG);

    pan_world_rotation = gimbal_matrix_multiply(
        body_rotation,
        gimbal_matrix_multiply(pan_mount_rotation, pan_rotation));
    tilt_world_rotation = gimbal_matrix_multiply(
        pan_world_rotation,
        tilt_mount_rotation);
    laser_world_rotation = gimbal_matrix_multiply(
        tilt_world_rotation,
        gimbal_matrix_multiply(pitch_rotation, laser_mount_rotation));

    vehicle_origin.x = pose->x_mm;
    vehicle_origin.y = pose->y_mm;
    vehicle_origin.z = pose->z_mm;
    pan_origin_body.x = GIMBAL_CONFIG_PAN_ORIGIN_X_MM;
    pan_origin_body.y = GIMBAL_CONFIG_PAN_ORIGIN_Y_MM;
    pan_origin_body.z = GIMBAL_CONFIG_PAN_ORIGIN_Z_MM;
    tilt_origin_pan.x = GIMBAL_CONFIG_TILT_ORIGIN_X_MM;
    tilt_origin_pan.y = GIMBAL_CONFIG_TILT_ORIGIN_Y_MM;
    tilt_origin_pan.z = GIMBAL_CONFIG_TILT_ORIGIN_Z_MM;
    laser_origin_tilt.x = GIMBAL_CONFIG_LASER_ORIGIN_X_MM;
    laser_origin_tilt.y = GIMBAL_CONFIG_LASER_ORIGIN_Y_MM;
    laser_origin_tilt.z = GIMBAL_CONFIG_LASER_ORIGIN_Z_MM;
    laser_direction_tilt.x = 1.0F;
    laser_direction_tilt.y = 0.0F;
    laser_direction_tilt.z = 0.0F;
    *origin_world = gimbal_vector_add(
        vehicle_origin,
        gimbal_matrix_transform(body_rotation, pan_origin_body));
    *origin_world = gimbal_vector_add(
        *origin_world,
        gimbal_matrix_transform(pan_world_rotation, tilt_origin_pan));
    *origin_world = gimbal_vector_add(
        *origin_world,
        gimbal_matrix_transform(laser_world_rotation, laser_origin_tilt));
    *direction_world = gimbal_matrix_transform(
        laser_world_rotation,
        laser_direction_tilt);

    return gimbal_vector_normalize(*direction_world, direction_world);
}

/**
 * @brief Calculate azimuth/elevation error between laser ray and target point.
 */
static uint8 gimbal_compute_ray_error(
    const gimbal_feedforward_pose_struct *pose,
    gimbal_vector3_struct target,
    float yaw_deg,
    float pitch_deg,
    float *yaw_error_rad,
    float *pitch_error_rad)
{
    gimbal_vector3_struct origin_world;
    gimbal_vector3_struct laser_direction;
    gimbal_vector3_struct target_direction;
    float laser_azimuth;
    float target_azimuth;
    float laser_elevation;
    float target_elevation;
    float horizontal_norm;

    if((yaw_error_rad == NULL) || (pitch_error_rad == NULL))
    {
        return 0U;
    }
    if(gimbal_compute_laser_ray(
            pose,
            yaw_deg,
            pitch_deg,
            &origin_world,
            &laser_direction) == 0U)
    {
        return 0U;
    }

    if(gimbal_vector_normalize(
            gimbal_vector_subtract(target, origin_world),
            &target_direction) == 0U)
    {
        return 0U;
    }

    laser_azimuth = atan2f(laser_direction.y, laser_direction.x);
    target_azimuth = atan2f(target_direction.y, target_direction.x);
    horizontal_norm = sqrtf(
        (laser_direction.x * laser_direction.x)
        + (laser_direction.y * laser_direction.y));
    laser_elevation = atan2f(laser_direction.z, horizontal_norm);
    horizontal_norm = sqrtf(
        (target_direction.x * target_direction.x)
        + (target_direction.y * target_direction.y));
    target_elevation = atan2f(target_direction.z, horizontal_norm);

    *yaw_error_rad = gimbal_wrap_angle_rad(
        target_azimuth - laser_azimuth);
    *pitch_error_rad = target_elevation - laser_elevation;
    return 1U;
}

/**
 * @brief Return a bounded signed step count for one angle.
 */
static int32 gimbal_angle_to_steps(
    float angle_deg,
    int32 position_sign)
{
    float steps = angle_deg
        * ((float)GIMBAL_STEPPER_STEPS_PER_REVOLUTION / 360.0F)
        * (float)position_sign;

    if(steps >= 0.0F)
    {
        return (int32)(steps + 0.5F);
    }
    return (int32)(steps - 0.5F);
}

/**
 * @brief Select the continuous yaw representation nearest the prior target.
 */
static uint8 gimbal_select_yaw_target_steps(
    int32 nominal_steps,
    int32 previous_steps,
    int32 *selected_steps)
{
    int32 candidates[3];
    int64 best_distance = 0x7FFFFFFF;
    int32 best_steps = nominal_steps;
    uint8 found = 0U;
    uint8 index;

    if(selected_steps == NULL)
    {
        return 0U;
    }

    candidates[0] = nominal_steps;
    candidates[1] = nominal_steps
        + (int32)GIMBAL_STEPPER_STEPS_PER_REVOLUTION;
    candidates[2] = nominal_steps
        - (int32)GIMBAL_STEPPER_STEPS_PER_REVOLUTION;

    for(index = 0U; index < 3U; index++)
    {
        int64 distance;

        if((candidates[index] < GIMBAL_STEPPER_YAW_MIN_STEPS)
            || (candidates[index] > GIMBAL_STEPPER_YAW_MAX_STEPS))
        {
            continue;
        }

        distance = (int64)candidates[index] - previous_steps;
        if(distance < 0)
        {
            distance = -distance;
        }
        if((found == 0U) || (distance < best_distance))
        {
            found = 1U;
            best_distance = distance;
            best_steps = candidates[index];
        }
    }

    if(found == 0U)
    {
        return 0U;
    }

    *selected_steps = best_steps;
    return 1U;
}

/**
 * @brief Solve the configured two-axis gimbal inverse kinematics.
 */
static uint8 gimbal_solve_feedforward(
    const gimbal_feedforward_pose_struct *pose,
    gimbal_vector3_struct target,
    gimbal_feedforward_solution_struct *solution)
{
    gimbal_matrix3_struct body_rotation;
    gimbal_vector3_struct vehicle_origin;
    gimbal_vector3_struct pan_origin_body;
    gimbal_vector3_struct target_body;
    float yaw_deg;
    float pitch_deg;
    float error_yaw_rad = 0.0F;
    float error_pitch_rad = 0.0F;
    uint8 iteration;
    uint8 singular = 0U;

    if((pose == NULL) || (solution == NULL))
    {
        return 0U;
    }

    solution->target_x_mm = target.x;
    solution->target_y_mm = target.y;
    solution->target_z_mm = target.z;
    solution->yaw_deg = 0.0F;
    solution->pitch_deg = GIMBAL_CONFIG_PITCH_ZERO_DEG;
    solution->residual_deg = 0.0F;
    solution->valid = 0U;
    solution->singular = 0U;

    body_rotation = gimbal_body_rotation(pose);
    vehicle_origin.x = pose->x_mm;
    vehicle_origin.y = pose->y_mm;
    vehicle_origin.z = pose->z_mm;
    pan_origin_body.x = GIMBAL_CONFIG_PAN_ORIGIN_X_MM;
    pan_origin_body.y = GIMBAL_CONFIG_PAN_ORIGIN_Y_MM;
    pan_origin_body.z = GIMBAL_CONFIG_PAN_ORIGIN_Z_MM;
    target_body = gimbal_matrix_inverse_transform(
        body_rotation,
        gimbal_vector_subtract(
            target,
            gimbal_vector_add(
                vehicle_origin,
                gimbal_matrix_transform(
                    body_rotation,
                    pan_origin_body))));
    yaw_deg = gimbal_wrap_angle_positive_deg(
        ((atan2f(target_body.y, target_body.x)
                * GIMBAL_RAD_TO_DEG)
            - GIMBAL_CONFIG_PAN_MOUNT_YAW_DEG)
            / GIMBAL_CONFIG_YAW_KINEMATIC_SIGN);
    pitch_deg = atan2f(
        target_body.z,
        sqrtf((target_body.x * target_body.x)
            + (target_body.y * target_body.y))) * GIMBAL_RAD_TO_DEG;
    yaw_deg = gimbal_clamp_float(
        yaw_deg,
        GIMBAL_CONFIG_YAW_MIN_DEG,
        GIMBAL_CONFIG_YAW_MAX_DEG);
    pitch_deg = gimbal_clamp_float(
        pitch_deg,
        GIMBAL_CONFIG_PITCH_MIN_DEG,
        GIMBAL_CONFIG_PITCH_MAX_DEG);

    for(iteration = 0U;
        iteration < GIMBAL_CONFIG_FEEDFORWARD_ITERATIONS;
        iteration++)
    {
        float yaw_plus;
        float yaw_minus;
        float pitch_plus;
        float pitch_minus;
        float error_yaw_plus;
        float error_yaw_minus;
        float error_pitch_plus;
        float error_pitch_minus;
        float jacobian_00;
        float jacobian_01;
        float jacobian_10;
        float jacobian_11;
        float determinant;
        float delta_yaw;
        float delta_pitch;

        if(gimbal_compute_ray_error(
                pose,
                target,
                yaw_deg,
                pitch_deg,
                &error_yaw_rad,
                &error_pitch_rad) == 0U)
        {
            return 0U;
        }
        if(sqrtf(
                (error_yaw_rad * error_yaw_rad)
                + (error_pitch_rad * error_pitch_rad))
                * GIMBAL_RAD_TO_DEG
            <= GIMBAL_CONFIG_FEEDFORWARD_TOLERANCE_DEG)
        {
            break;
        }

        yaw_plus = gimbal_clamp_float(
            yaw_deg + GIMBAL_CONFIG_FEEDFORWARD_JACOBIAN_DEG,
            GIMBAL_CONFIG_YAW_MIN_DEG,
            GIMBAL_CONFIG_YAW_MAX_DEG);
        yaw_minus = gimbal_clamp_float(
            yaw_deg - GIMBAL_CONFIG_FEEDFORWARD_JACOBIAN_DEG,
            GIMBAL_CONFIG_YAW_MIN_DEG,
            GIMBAL_CONFIG_YAW_MAX_DEG);
        pitch_plus = gimbal_clamp_float(
            pitch_deg + GIMBAL_CONFIG_FEEDFORWARD_JACOBIAN_DEG,
            GIMBAL_CONFIG_PITCH_MIN_DEG,
            GIMBAL_CONFIG_PITCH_MAX_DEG);
        pitch_minus = gimbal_clamp_float(
            pitch_deg - GIMBAL_CONFIG_FEEDFORWARD_JACOBIAN_DEG,
            GIMBAL_CONFIG_PITCH_MIN_DEG,
            GIMBAL_CONFIG_PITCH_MAX_DEG);

        if((gimbal_compute_ray_error(
                pose,
                target,
                yaw_plus,
                pitch_deg,
                &error_yaw_plus,
                &error_pitch_plus) == 0U)
            || (gimbal_compute_ray_error(
                pose,
                target,
                yaw_minus,
                pitch_deg,
                &error_yaw_minus,
                &error_pitch_minus) == 0U))
        {
            solution->yaw_deg = yaw_deg;
            solution->pitch_deg = pitch_deg;
            return 0U;
        }
        jacobian_00 = gimbal_wrap_angle_rad(
            error_yaw_plus - error_yaw_minus)
            / ((yaw_plus - yaw_minus) * GIMBAL_DEG_TO_RAD);
        jacobian_10 = (error_pitch_plus - error_pitch_minus)
            / ((yaw_plus - yaw_minus) * GIMBAL_DEG_TO_RAD);

        if((gimbal_compute_ray_error(
                pose,
                target,
                yaw_deg,
                pitch_plus,
                &error_yaw_plus,
                &error_pitch_plus) == 0U)
            || (gimbal_compute_ray_error(
                pose,
                target,
                yaw_deg,
                pitch_minus,
                &error_yaw_minus,
                &error_pitch_minus) == 0U))
        {
            solution->yaw_deg = yaw_deg;
            solution->pitch_deg = pitch_deg;
            return 0U;
        }
        jacobian_01 = gimbal_wrap_angle_rad(
            error_yaw_plus - error_yaw_minus)
            / ((pitch_plus - pitch_minus) * GIMBAL_DEG_TO_RAD);
        jacobian_11 = (error_pitch_plus - error_pitch_minus)
            / ((pitch_plus - pitch_minus) * GIMBAL_DEG_TO_RAD);
        determinant = (jacobian_00 * jacobian_11)
            - (jacobian_01 * jacobian_10);
        if(fabsf(determinant) < GIMBAL_FEEDFORWARD_MIN_DETERMINANT)
        {
            singular = 1U;
            break;
        }

        delta_yaw = ((-error_yaw_rad * jacobian_11)
            + (jacobian_01 * error_pitch_rad)) / determinant;
        delta_pitch = ((jacobian_10 * error_yaw_rad)
            - (jacobian_00 * error_pitch_rad)) / determinant;
        delta_yaw = gimbal_clamp_float(
            delta_yaw * GIMBAL_RAD_TO_DEG,
            -GIMBAL_CONFIG_FEEDFORWARD_MAX_STEP_DEG,
            GIMBAL_CONFIG_FEEDFORWARD_MAX_STEP_DEG);
        delta_pitch = gimbal_clamp_float(
            delta_pitch * GIMBAL_RAD_TO_DEG,
            -GIMBAL_CONFIG_FEEDFORWARD_MAX_STEP_DEG,
            GIMBAL_CONFIG_FEEDFORWARD_MAX_STEP_DEG);
        yaw_deg = gimbal_clamp_float(
            yaw_deg + delta_yaw,
            GIMBAL_CONFIG_YAW_MIN_DEG,
            GIMBAL_CONFIG_YAW_MAX_DEG);
        pitch_deg = gimbal_clamp_float(
            pitch_deg + delta_pitch,
            GIMBAL_CONFIG_PITCH_MIN_DEG,
            GIMBAL_CONFIG_PITCH_MAX_DEG);
    }

    if(gimbal_compute_ray_error(
            pose,
            target,
            yaw_deg,
            pitch_deg,
            &error_yaw_rad,
            &error_pitch_rad) == 0U)
    {
        solution->yaw_deg = yaw_deg;
        solution->pitch_deg = pitch_deg;
        solution->singular = 1U;
        return 0U;
    }

    solution->yaw_deg = yaw_deg;
    solution->pitch_deg = pitch_deg;
    solution->residual_deg = sqrtf(
        (error_yaw_rad * error_yaw_rad)
        + (error_pitch_rad * error_pitch_rad)) * GIMBAL_RAD_TO_DEG;
    solution->singular = singular;
    solution->valid = (uint8)((singular == 0U)
        && (solution->residual_deg
            <= GIMBAL_CONFIG_FEEDFORWARD_TOLERANCE_DEG));
    return solution->valid;
}

/**
 * @brief Return the opposite digital output level.
 * @param level GPIO_LOW or GPIO_HIGH.
 * @return Opposite output level.
 */
static uint8 gimbal_invert_level(uint8 level)
{
    return level == GPIO_LOW ? GPIO_HIGH : GPIO_LOW;
}

/**
 * @brief Clamp a signed position to one axis software limits.
 */
static int32 gimbal_clamp_position(
    int64 position,
    int32 minimum,
    int32 maximum)
{
    if(position < (int64)minimum)
    {
        return minimum;
    }
    if(position > (int64)maximum)
    {
        return maximum;
    }
    return (int32)position;
}

/**
 * @brief Convert a signed fixed-point step rate to a DDS increment.
 */
static uint32 gimbal_rate_to_phase_increment(
    int32 rate_milli_steps_s)
{
    uint64 magnitude;
    uint64 denominator;

    if(rate_milli_steps_s < 0)
    {
        magnitude = (uint64)(-rate_milli_steps_s);
    }
    else
    {
        magnitude = (uint64)rate_milli_steps_s;
    }

    if(magnitude > (uint64)GIMBAL_MAX_RATE_MILLI_STEPS_S)
    {
        magnitude = (uint64)GIMBAL_MAX_RATE_MILLI_STEPS_S;
    }

    denominator = (uint64)GIMBAL_TICK_HZ * GIMBAL_RATE_SCALE;
    return (uint32)((magnitude << 32U) / denominator);
}

/**
 * @brief Move a signed rate toward a target without crossing zero.
 */
static int32 gimbal_ramp_rate(int32 current, int32 target)
{
    int32 delta = GIMBAL_RATE_DELTA_MILLI_STEPS_S;

    if(((current > 0) && (target < 0))
        || ((current < 0) && (target > 0)))
    {
        target = 0;
    }

    if(current < target)
    {
        if((target - current) <= delta)
        {
            return target;
        }
        return current + delta;
    }

    if(current > target)
    {
        if((current - target) <= delta)
        {
            return target;
        }
        return current - delta;
    }

    return current;
}

/**
 * @brief Clear one axis pulse command.
 */
static void gimbal_clear_axis_command(gimbal_axis_struct *axis)
{
    axis->target_rate_milli_steps_s = 0;
    axis->current_rate_milli_steps_s = 0;
    axis->command_rate_milli_steps_s = 0;
    axis->phase_accumulator = 0U;
    axis->phase_increment = 0U;
    axis->direction_settle_ticks = 0U;
    axis->pulse_pending = 0U;
}

/**
 * @brief Synchronize both position targets to current pulse positions.
 */
static void gimbal_sync_targets_to_positions(void)
{
    gimbal_axes[GIMBAL_STEPPER_AXIS_YAW].target_position_steps =
        gimbal_axes[GIMBAL_STEPPER_AXIS_YAW].position_steps;
    gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH].target_position_steps =
        gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH].position_steps;
}

/**
 * @brief Stop both axes atomically and discard pending position motion.
 */
static void gimbal_stop_all(void)
{
    uint32 primask = interrupt_global_disable();

    gimbal_laser_force_off();
    gimbal_laser_settle_ms = 0U;
    gimbal_clear_axis_command(
        &gimbal_axes[GIMBAL_STEPPER_AXIS_YAW]);
    gimbal_clear_axis_command(
        &gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH]);
    gimbal_sync_targets_to_positions();
    interrupt_global_enable(primask);
}

/**
 * @brief Check whether both foreground rates have reached zero.
 */
static uint8 gimbal_axes_stopped(void)
{
    return (uint8)(
        (gimbal_axes[GIMBAL_STEPPER_AXIS_YAW]
            .current_rate_milli_steps_s == 0)
        && (gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH]
            .current_rate_milli_steps_s == 0));
}

/**
 * @brief Check whether both axes have valid software zero positions.
 */
static uint8 gimbal_axes_zeroed(void)
{
    return (uint8)(
        (gimbal_axes[GIMBAL_STEPPER_AXIS_YAW].zero_valid != 0U)
        && (gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH].zero_valid != 0U));
}

/**
 * @brief Force the active-low laser output to its safe-off state.
 */
static void gimbal_laser_force_off(void)
{
    if(GIMBAL_CONFIG_LASER_POLARITY_VALID != 0U)
    {
        gpio_set_level(
            GIMBAL_LASER_PIN,
            gimbal_invert_level(GIMBAL_LASER_ACTIVE_LEVEL));
    }
}

/**
 * @brief Check all controller and raw-input laser interlocks.
 */
static uint8 gimbal_laser_can_enable(void)
{
    gimbal_axis_struct *yaw =
        &gimbal_axes[GIMBAL_STEPPER_AXIS_YAW];
    gimbal_axis_struct *pitch =
        &gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH];

    return (uint8)(
        (gimbal_stop_latched == 0U)
        && (gimbal_control_mode == GIMBAL_CONTROL_POSITION)
        && (gimbal_axes_zeroed() != 0U)
        && (gpio_get_level(GIMBAL_SELECT_KEY_PIN) == GPIO_HIGH)
        && (gpio_get_level(GIMBAL_NEGATIVE_KEY_PIN) == GPIO_HIGH)
        && (gpio_get_level(GIMBAL_POSITIVE_KEY_PIN) == GPIO_HIGH)
        && (yaw->position_steps == yaw->target_position_steps)
        && (pitch->position_steps == pitch->target_position_steps)
        && (yaw->current_rate_milli_steps_s == 0)
        && (pitch->current_rate_milli_steps_s == 0)
        && (yaw->command_rate_milli_steps_s == 0)
        && (pitch->command_rate_milli_steps_s == 0));
}

/**
 * @brief Stop both outputs immediately from the pulse callback.
 */
static void gimbal_emergency_stop_tick(void)
{
    gimbal_axis_struct *yaw =
        &gimbal_axes[GIMBAL_STEPPER_AXIS_YAW];
    gimbal_axis_struct *pitch =
        &gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH];

    gimbal_clear_axis_command(yaw);
    gimbal_clear_axis_command(pitch);
    yaw->pulse_high = 0U;
    pitch->pulse_high = 0U;
    gpio_low(yaw->step_pin);
    gpio_low(pitch->step_pin);
    gimbal_laser_force_off();
    gimbal_laser_settle_ms = 0U;
    gimbal_sync_targets_to_positions();
    gimbal_control_mode = GIMBAL_CONTROL_IDLE;
    gimbal_stop_latched = 1U;
}

/**
 * @brief Capture the selected axis software zero position.
 * @return 1 when captured, otherwise 0 when emergency stop won.
 */
static uint8 gimbal_zero_selected_axis(void)
{
    gimbal_axis_struct *axis = &gimbal_axes[gimbal_selected_axis];
    uint32 primask;

    primask = interrupt_global_disable();
    if(gimbal_stop_latched != 0U)
    {
        interrupt_global_enable(primask);
        return 0U;
    }
    gimbal_clear_axis_command(axis);
    axis->position_steps = 0;
    axis->target_position_steps = 0;
    axis->zero_valid = 1U;
    interrupt_global_enable(primask);

    gimbal_log(gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
        ? "Gimbal YAW zero captured.\r\n"
        : "Gimbal PITCH zero captured.\r\n");
    return 1U;
}

/**
 * @brief Update one active-low key debounce state.
 */
static void gimbal_update_key(
    gpio_pin_enum pin,
    gimbal_key_struct *key)
{
    uint8 raw_pressed = (uint8)(gpio_get_level(pin) == GPIO_LOW);

    if(raw_pressed == key->pressed)
    {
        key->mismatch_ms = 0U;
        return;
    }

    if(key->mismatch_ms < GIMBAL_DEBOUNCE_MS)
    {
        key->mismatch_ms++;
    }
    if(key->mismatch_ms >= GIMBAL_DEBOUNCE_MS)
    {
        key->pressed = raw_pressed;
        key->mismatch_ms = 0U;
    }
}

/**
 * @brief Debounce manual keys and track A30 hold time at 1 kHz.
 */
static void gimbal_update_keys_tick(void)
{
    gimbal_update_key(GIMBAL_SELECT_KEY_PIN, &gimbal_select_key);
    gimbal_update_key(GIMBAL_NEGATIVE_KEY_PIN, &gimbal_negative_key);
    gimbal_update_key(GIMBAL_POSITIVE_KEY_PIN, &gimbal_positive_key);

    if(gimbal_select_key.pressed != 0U)
    {
        if(gimbal_select_previous_pressed == 0U)
        {
            gimbal_select_hold_ms = 0U;
            gimbal_select_long_handled = 0U;
            gimbal_select_release_event = 0U;
        }
        if(gimbal_select_hold_ms < GIMBAL_ZERO_HOLD_MS)
        {
            gimbal_select_hold_ms++;
        }
    }
    else if(gimbal_select_previous_pressed != 0U)
    {
        gimbal_select_release_event = 1U;
    }

    gimbal_select_previous_pressed = gimbal_select_key.pressed;
}

/**
 * @brief Capture one coherent foreground snapshot of debounced keys.
 */
static void gimbal_get_key_snapshot(
    gimbal_key_snapshot_struct *snapshot)
{
    uint32 primask = interrupt_global_disable();

    snapshot->select_hold_ms = gimbal_select_hold_ms;
    snapshot->select_pressed = gimbal_select_key.pressed;
    snapshot->select_released = gimbal_select_release_event;
    snapshot->negative_pressed = gimbal_negative_key.pressed;
    snapshot->positive_pressed = gimbal_positive_key.pressed;
    gimbal_select_release_event = 0U;
    interrupt_global_enable(primask);
}

/**
 * @brief Handle short-select and long-zero actions on A30.
 */
static void gimbal_update_select_key(
    const gimbal_key_snapshot_struct *keys)
{
    uint8 jog_active = (uint8)(keys->negative_pressed
        || keys->positive_pressed);
    uint8 axes_stopped = gimbal_axes_stopped();

    if(gimbal_select_suppressed != 0U)
    {
        if(keys->select_pressed == 0U)
        {
            uint32 primask = interrupt_global_disable();

            gimbal_select_suppressed = 0U;
            gimbal_select_release_event = 0U;
            gimbal_select_hold_ms = 0U;
            gimbal_select_long_handled = 0U;
            interrupt_global_enable(primask);
        }
        return;
    }

    if((keys->select_pressed != 0U)
        && (keys->select_hold_ms >= GIMBAL_ZERO_HOLD_MS)
        && (gimbal_select_long_handled == 0U)
        && (jog_active == 0U)
        && (axes_stopped != 0U))
    {
        if(gimbal_zero_selected_axis() != 0U)
        {
            gimbal_select_long_handled = 1U;
        }
    }

    if(keys->select_released != 0U)
    {
        if((gimbal_select_long_handled == 0U)
            && (jog_active == 0U)
            && (axes_stopped != 0U))
        {
            uint8 changed = 0U;
            uint32 primask = interrupt_global_disable();

            if(gimbal_stop_latched == 0U)
            {
                gimbal_selected_axis =
                    gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
                        ? GIMBAL_STEPPER_AXIS_PITCH
                        : GIMBAL_STEPPER_AXIS_YAW;
                changed = 1U;
            }
            interrupt_global_enable(primask);

            if(changed != 0U)
            {
                gimbal_log(
                    gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
                        ? "Gimbal selected axis: YAW.\r\n"
                        : "Gimbal selected axis: PITCH.\r\n");
            }
        }
        gimbal_select_hold_ms = 0U;
        gimbal_select_long_handled = 0U;
    }
}

/**
 * @brief Apply B0 and B1 manual jog commands to the selected axis.
 */
static uint8 gimbal_update_jog_targets(
    const gimbal_key_snapshot_struct *keys)
{
    gimbal_axis_struct *selected =
        &gimbal_axes[gimbal_selected_axis];
    gimbal_axis_struct *unselected =
        &gimbal_axes[
            gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
                ? GIMBAL_STEPPER_AXIS_PITCH
                : GIMBAL_STEPPER_AXIS_YAW];
    int32 jog_rate = GIMBAL_CALIBRATE_RATE_MILLI_STEPS_S;
    uint8 negative_only = (uint8)(
        (keys->negative_pressed != 0U)
        && (keys->positive_pressed == 0U));
    uint8 positive_only = (uint8)(
        (keys->positive_pressed != 0U)
        && (keys->negative_pressed == 0U));
    uint8 both_pressed = (uint8)(
        (keys->negative_pressed != 0U)
        && (keys->positive_pressed != 0U));

    if((selected->zero_valid == 0U)
        && (((gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW)
                && (GIMBAL_CONFIG_ALLOW_PREZERO_YAW_JOG == 0U))
            || ((gimbal_selected_axis == GIMBAL_STEPPER_AXIS_PITCH)
                && (GIMBAL_CONFIG_ALLOW_PREZERO_PITCH_JOG == 0U))))
    {
        return 0U;
    }

    if(selected->zero_valid != 0U)
    {
        jog_rate = gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
            ? GIMBAL_YAW_JOG_RATE_MILLI_STEPS_S
            : GIMBAL_PITCH_JOG_RATE_MILLI_STEPS_S;
    }

    if(both_pressed != 0U)
    {
        if((gimbal_control_mode == GIMBAL_CONTROL_JOG)
            || (gimbal_control_mode == GIMBAL_CONTROL_POSITION))
        {
            gimbal_control_mode = GIMBAL_CONTROL_STOPPING;
        }
        return 0U;
    }

    if((negative_only == 0U) && (positive_only == 0U))
    {
        if(gimbal_control_mode == GIMBAL_CONTROL_JOG)
        {
            gimbal_control_mode = GIMBAL_CONTROL_STOPPING;
        }
        return 0U;
    }

    {
        uint32 primask = interrupt_global_disable();

        unselected->target_rate_milli_steps_s = 0;
        selected->target_rate_milli_steps_s = negative_only != 0U
            ? -jog_rate : jog_rate;
        gimbal_control_mode = GIMBAL_CONTROL_JOG;
        interrupt_global_enable(primask);
    }
    return 1U;
}

/**
 * @brief Convert one position error to a bounded signed step rate.
 */
static int32 gimbal_position_target_rate(
    const gimbal_axis_struct *axis)
{
    int32 error = axis->target_position_steps - axis->position_steps;
    int64 magnitude;

    if(error == 0)
    {
        return 0;
    }

    magnitude = error < 0 ? -(int64)error : (int64)error;
    magnitude *= GIMBAL_POSITION_GAIN_MILLI_RATE;
    if(magnitude > GIMBAL_POSITION_RATE_MILLI_STEPS_S)
    {
        magnitude = GIMBAL_POSITION_RATE_MILLI_STEPS_S;
    }

    return error < 0 ? -(int32)magnitude : (int32)magnitude;
}

/**
 * @brief Select target rates for stopping or position control.
 */
static void gimbal_update_position_targets(void)
{
    if(gimbal_control_mode == GIMBAL_CONTROL_STOPPING)
    {
        if(gimbal_axes_stopped() != 0U)
        {
            gimbal_sync_targets_to_positions();
            gimbal_control_mode = gimbal_axes_zeroed() != 0U
                ? GIMBAL_CONTROL_POSITION : GIMBAL_CONTROL_IDLE;
        }
        return;
    }

    if(gimbal_axes_zeroed() == 0U)
    {
        gimbal_control_mode = GIMBAL_CONTROL_IDLE;
        return;
    }

    if(gimbal_control_mode == GIMBAL_CONTROL_IDLE)
    {
        if(gimbal_axes_stopped() == 0U)
        {
            return;
        }
        gimbal_sync_targets_to_positions();
        gimbal_control_mode = GIMBAL_CONTROL_POSITION;
    }

}

/**
 * @brief Apply acceleration limits from the 100 Hz interrupt divider.
 */
static void gimbal_update_rates_tick(void)
{
    uint8 index;

    for(index = 0U; index < GIMBAL_STEPPER_AXIS_COUNT; index++)
    {
        gimbal_axis_struct *axis = &gimbal_axes[index];
        int32 desired_rate = 0;

        if(gimbal_control_mode == GIMBAL_CONTROL_POSITION)
        {
            desired_rate = gimbal_position_target_rate(axis);
        }
        else if(gimbal_control_mode == GIMBAL_CONTROL_JOG)
        {
            desired_rate = axis->target_rate_milli_steps_s;
        }

        axis->current_rate_milli_steps_s = gimbal_ramp_rate(
            axis->current_rate_milli_steps_s,
            desired_rate);
        axis->command_rate_milli_steps_s =
            axis->current_rate_milli_steps_s;
        axis->phase_increment = gimbal_rate_to_phase_increment(
            axis->current_rate_milli_steps_s);
    }
}

/**
 * @brief Execute one 5 kHz STEP/DIR pulse-engine tick for one axis.
 */
static void gimbal_axis_tick(gimbal_axis_struct *axis)
{
    int32 command_rate = axis->command_rate_milli_steps_s;
    uint8 positive;
    uint8 direction_level;
    uint32 previous_phase;
    uint8 phase_overflow;
    int32 minimum;
    int32 maximum;

    if((command_rate == 0) || (axis->phase_increment == 0U))
    {
        gpio_low(axis->step_pin);
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        return;
    }

    positive = (uint8)(command_rate > 0);
    if(positive != axis->direction_positive)
    {
        axis->direction_positive = positive;
        direction_level = positive != 0U
            ? axis->positive_dir_level
            : gimbal_invert_level(axis->positive_dir_level);
        gpio_set_level(axis->dir_pin, direction_level);
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        gpio_low(axis->step_pin);
        axis->direction_settle_ticks =
            GIMBAL_DIRECTION_SETTLE_TICKS;
        return;
    }

    if(axis->direction_settle_ticks > 0U)
    {
        axis->direction_settle_ticks--;
        return;
    }

    if(axis->zero_valid != 0U)
    {
        minimum = axis->min_position_steps;
        maximum = axis->max_position_steps;
    }
    else
    {
        minimum = -axis->calibrate_travel_steps;
        maximum = axis->calibrate_travel_steps;
    }

    if((axis->zero_valid != 0U)
        && (gimbal_control_mode == GIMBAL_CONTROL_POSITION)
        && (((positive != 0U)
                && (axis->position_steps
                    >= axis->target_position_steps))
            || ((positive == 0U)
                && (axis->position_steps
                    <= axis->target_position_steps))))
    {
        gpio_low(axis->step_pin);
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        return;
    }

    if(((positive != 0U) && (axis->position_steps >= maximum))
        || ((positive == 0U) && (axis->position_steps <= minimum)))
    {
        gpio_low(axis->step_pin);
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        return;
    }

    previous_phase = axis->phase_accumulator;
    axis->phase_accumulator += axis->phase_increment;
    phase_overflow = (uint8)(
        axis->phase_accumulator < previous_phase);

    if(axis->pulse_high != 0U)
    {
        gpio_low(axis->step_pin);
        axis->pulse_high = 0U;
        if(phase_overflow != 0U)
        {
            axis->pulse_pending = 1U;
        }
        return;
    }

    if((axis->pulse_pending != 0U) || (phase_overflow != 0U))
    {
        axis->pulse_pending = 0U;
        gpio_high(axis->step_pin);
        axis->pulse_high = 1U;
        axis->position_steps += positive != 0U ? 1 : -1;
    }
}

/**
 * @brief Generate independent yaw and pitch pulses from TIMG6.
 */
static void gimbal_pit_callback(uint32 event, void *context)
{
    uint8 laser_safe;

    (void)event;
    (void)context;

    laser_safe = gimbal_laser_can_enable();
    if(laser_safe == 0U)
    {
        gimbal_laser_settle_ms = 0U;
        gimbal_laser_force_off();
    }

    gimbal_millisecond_divider++;
    if(gimbal_millisecond_divider
        >= GIMBAL_MILLISECOND_DIVIDER_TICKS)
    {
        gimbal_millisecond_divider = 0U;
        gimbal_update_keys_tick();
        if((laser_safe != 0U)
            && (gimbal_laser_settle_ms
                < GIMBAL_CONFIG_LASER_SETTLE_MS))
        {
            gimbal_laser_settle_ms++;
        }
        if(gimbal_pending_ms < 1000U)
        {
            gimbal_pending_ms++;
        }
    }

    if((gpio_get_level(GIMBAL_STOP_KEY_PIN) == GPIO_LOW)
        || (gimbal_stop_latched != 0U))
    {
        gimbal_emergency_stop_tick();
        return;
    }

    gimbal_rate_tick_divider++;
    if(gimbal_rate_tick_divider >= GIMBAL_RATE_DIVIDER_TICKS)
    {
        gimbal_rate_tick_divider = 0U;
        gimbal_update_rates_tick();
    }

    gimbal_axis_tick(&gimbal_axes[GIMBAL_STEPPER_AXIS_YAW]);
    gimbal_axis_tick(&gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH]);
}

/**
 * @brief Initialize one axis runtime state and output pins.
 */
static void gimbal_initialize_axis(
    gimbal_axis_struct *axis,
    gpio_pin_enum step_pin,
    gpio_pin_enum dir_pin,
    int32 minimum,
    int32 maximum,
    int32 calibrate_travel_steps,
    uint8 positive_dir_level)
{
    axis->step_pin = step_pin;
    axis->dir_pin = dir_pin;
    axis->min_position_steps = minimum;
    axis->max_position_steps = maximum;
    axis->calibrate_travel_steps = calibrate_travel_steps;
    axis->positive_dir_level = positive_dir_level;
    axis->target_position_steps = 0;
    axis->target_rate_milli_steps_s = 0;
    axis->current_rate_milli_steps_s = 0;
    axis->command_rate_milli_steps_s = 0;
    axis->position_steps = 0;
    axis->phase_accumulator = 0U;
    axis->phase_increment = 0U;
    axis->direction_positive = 0U;
    axis->direction_settle_ticks = 0U;
    axis->pulse_pending = 0U;
    axis->pulse_high = 0U;
    axis->zero_valid = 0U;

    gpio_init(step_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(dir_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/**
 * @brief Initialize the complete dual-axis gimbal controller.
 */
void gimbal_stepper_init(void)
{
    gimbal_initialize_axis(
        &gimbal_axes[GIMBAL_STEPPER_AXIS_YAW],
        GIMBAL_YAW_STEP_PIN,
        GIMBAL_YAW_DIR_PIN,
        GIMBAL_STEPPER_YAW_MIN_STEPS,
        GIMBAL_STEPPER_YAW_MAX_STEPS,
        GIMBAL_YAW_CALIBRATE_TRAVEL_STEPS,
        GIMBAL_YAW_POSITIVE_DIR_LEVEL);
    gimbal_initialize_axis(
        &gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH],
        GIMBAL_PITCH_STEP_PIN,
        GIMBAL_PITCH_DIR_PIN,
        GIMBAL_STEPPER_PITCH_MIN_STEPS,
        GIMBAL_STEPPER_PITCH_MAX_STEPS,
        GIMBAL_PITCH_CALIBRATE_TRAVEL_STEPS,
        GIMBAL_PITCH_POSITIVE_DIR_LEVEL);

    gpio_init(
        GIMBAL_SELECT_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        GIMBAL_STOP_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        GIMBAL_NEGATIVE_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        GIMBAL_POSITIVE_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);

    gimbal_select_key.mismatch_ms = 0U;
    gimbal_select_key.pressed = 0U;
    gimbal_negative_key.mismatch_ms = 0U;
    gimbal_negative_key.pressed = 0U;
    gimbal_positive_key.mismatch_ms = 0U;
    gimbal_positive_key.pressed = 0U;
    gimbal_selected_axis = GIMBAL_STEPPER_AXIS_YAW;
    gimbal_select_hold_ms = 0U;
    gimbal_select_long_handled = 0U;
    gimbal_select_release_event = 0U;
    gimbal_select_previous_pressed = 0U;
    gimbal_select_suppressed = 0U;
    gimbal_stop_latched = 0U;
    gimbal_stop_reported = 0U;
    gimbal_control_mode = GIMBAL_CONTROL_IDLE;
    gimbal_pending_ms = 0U;
    gimbal_millisecond_divider = 0U;
    gimbal_rate_tick_divider = 0U;
    gimbal_laser_settle_ms = 0U;
    gimbal_log_callback = gimbal_default_log;
    gimbal_target_mode = GIMBAL_TARGET_CENTER;
    gimbal_target_phase_rad = 0.0F;
    gimbal_feedforward_solution.target_x_mm =
        GIMBAL_CONFIG_TARGET_CENTER_X_MM;
    gimbal_feedforward_solution.target_y_mm =
        GIMBAL_CONFIG_TARGET_CENTER_Y_MM;
    gimbal_feedforward_solution.target_z_mm =
        GIMBAL_CONFIG_TARGET_CENTER_Z_MM;
    gimbal_feedforward_solution.yaw_deg = 0.0F;
    gimbal_feedforward_solution.pitch_deg =
        GIMBAL_CONFIG_PITCH_ZERO_DEG;
    gimbal_feedforward_solution.residual_deg = 0.0F;
    gimbal_feedforward_solution.valid = 0U;
    gimbal_feedforward_solution.singular = 0U;
    gimbal_stepper_laser_init();

    interrupt_set_priority(GIMBAL_PIT_IRQ, GIMBAL_PIT_IRQ_PRIORITY);
    pit_us_init(
        GIMBAL_PIT,
        GIMBAL_TICK_US,
        gimbal_pit_callback,
        NULL);
}

/**
 * @brief Process foreground key events and control-mode transitions.
 */
static void gimbal_stepper_update_foreground(void)
{
    gimbal_key_snapshot_struct keys;

    if((gimbal_stop_latched != 0U)
        && (gimbal_stop_reported == 0U))
    {
        gimbal_stop_reported = 1U;
        gimbal_log("Gimbal emergency stop.\r\n");
    }

    if(gimbal_stop_latched != 0U)
    {
        gimbal_select_suppressed = 1U;
        gimbal_select_release_event = 0U;
    }

    if((gpio_get_level(GIMBAL_STOP_KEY_PIN) == GPIO_HIGH)
        && (gimbal_stop_latched != 0U)
        && (gpio_get_level(GIMBAL_NEGATIVE_KEY_PIN) == GPIO_HIGH)
        && (gpio_get_level(GIMBAL_POSITIVE_KEY_PIN) == GPIO_HIGH))
    {
        uint32 primask = interrupt_global_disable();

        gimbal_stop_latched = 0U;
        gimbal_sync_targets_to_positions();
        gimbal_control_mode = GIMBAL_CONTROL_IDLE;
        gimbal_negative_key.mismatch_ms = 0U;
        gimbal_negative_key.pressed = 0U;
        gimbal_positive_key.mismatch_ms = 0U;
        gimbal_positive_key.pressed = 0U;
        interrupt_global_enable(primask);
        gimbal_stop_reported = 0U;
        gimbal_log("Gimbal stop released.\r\n");
    }

    if(gimbal_stop_latched != 0U)
    {
        gimbal_stop_all();
    }
    else
    {
        gimbal_get_key_snapshot(&keys);
        gimbal_update_select_key(&keys);
        if(gimbal_update_jog_targets(&keys) == 0U)
        {
            gimbal_update_position_targets();
        }
    }
}

/**
 * @brief Consume real millisecond ticks generated by the 5 kHz callback.
 */
uint16 gimbal_stepper_service(void)
{
    uint16 elapsed_ms;
    uint32 primask = interrupt_global_disable();

    elapsed_ms = gimbal_pending_ms;
    gimbal_pending_ms = 0U;
    interrupt_global_enable(primask);

    gimbal_stepper_update_foreground();
    return elapsed_ms;
}

/**
 * @brief Accumulate one bounded relative position command.
 */
uint8 gimbal_stepper_move_relative_steps(
    int32 yaw_delta_steps,
    int32 pitch_delta_steps)
{
    gimbal_axis_struct *yaw =
        &gimbal_axes[GIMBAL_STEPPER_AXIS_YAW];
    gimbal_axis_struct *pitch =
        &gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH];
    uint32 primask;

    primask = interrupt_global_disable();
    if((gimbal_stop_latched != 0U)
        || (gimbal_control_mode != GIMBAL_CONTROL_POSITION)
        || (gimbal_axes_zeroed() == 0U)
        || (gimbal_select_key.pressed != 0U)
        || (gimbal_negative_key.pressed != 0U)
        || (gimbal_positive_key.pressed != 0U))
    {
        interrupt_global_enable(primask);
        return 0U;
    }

    yaw->target_position_steps = gimbal_clamp_position(
        (int64)yaw->target_position_steps + yaw_delta_steps,
        yaw->min_position_steps,
        yaw->max_position_steps);
    pitch->target_position_steps = gimbal_clamp_position(
        (int64)pitch->target_position_steps + pitch_delta_steps,
        pitch->min_position_steps,
        pitch->max_position_steps);
    gimbal_laser_force_off();
    gimbal_laser_settle_ms = 0U;
    interrupt_global_enable(primask);

    return 1U;
}

/**
 * @brief Check whether relative position commands may be accepted.
 */
uint8 gimbal_stepper_relative_ready(void)
{
    uint8 ready;
    uint32 primask = interrupt_global_disable();

    ready = (uint8)(
        (gimbal_stop_latched == 0U)
        && (gimbal_control_mode == GIMBAL_CONTROL_POSITION)
        && (gimbal_axes_zeroed() != 0U)
        && (gimbal_select_key.pressed == 0U)
        && (gimbal_negative_key.pressed == 0U)
        && (gimbal_positive_key.pressed == 0U));
    interrupt_global_enable(primask);
    return ready;
}

/**
 * @brief Copy an atomic controller status snapshot.
 */
void gimbal_stepper_get_status(
    gimbal_stepper_status_struct *status)
{
    uint8 index;
    uint32 primask;

    if(status == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    for(index = 0U; index < GIMBAL_STEPPER_AXIS_COUNT; index++)
    {
        status->axis[index].position_steps =
            gimbal_axes[index].position_steps;
        status->axis[index].target_position_steps =
            gimbal_axes[index].target_position_steps;
        status->axis[index].current_rate_steps_s =
            gimbal_axes[index].current_rate_milli_steps_s
                / (int32)GIMBAL_RATE_SCALE;
        status->axis[index].zero_valid =
            gimbal_axes[index].zero_valid;
    }
    status->selected_axis = gimbal_selected_axis;
    status->stop_latched = gimbal_stop_latched;
    status->relative_ready = (uint8)(
        (gimbal_control_mode == GIMBAL_CONTROL_POSITION)
        && (gimbal_axes_zeroed() != 0U)
        && (gimbal_stop_latched == 0U)
        && (gimbal_select_key.pressed == 0U)
        && (gimbal_negative_key.pressed == 0U)
        && (gimbal_positive_key.pressed == 0U));
    status->negative_key_pressed = gimbal_negative_key.pressed;
    status->positive_key_pressed = gimbal_positive_key.pressed;
    status->select_key_pressed = gimbal_select_key.pressed;
    interrupt_global_enable(primask);
}

/**
 * @brief Replace the foreground log sink or restore debug printf output.
 */
void gimbal_stepper_set_log_callback(
    gimbal_stepper_log_callback callback)
{
    uint32 primask = interrupt_global_disable();

    gimbal_log_callback = callback != NULL
        ? callback : gimbal_default_log;
    interrupt_global_enable(primask);
}

/**
 * @brief Select the target used by the geometric feedforward solver.
 */
void gimbal_stepper_set_target_mode(gimbal_target_mode_enum mode)
{
    uint32 primask;

    if(mode > GIMBAL_TARGET_CIRCLE)
    {
        return;
    }

    primask = interrupt_global_disable();
    gimbal_target_mode = mode;
    interrupt_global_enable(primask);
}

/**
 * @brief Set and wrap the target-circle phase.
 */
void gimbal_stepper_set_target_phase(float phase_rad)
{
    uint32 primask;

    if(gimbal_float_is_valid(phase_rad) == 0U)
    {
        return;
    }

    primask = interrupt_global_disable();
    gimbal_target_phase_rad = gimbal_wrap_angle_rad(phase_rad);
    interrupt_global_enable(primask);
}

/**
 * @brief Set bounded absolute position targets after manual zeroing.
 */
uint8 gimbal_stepper_set_absolute_target_steps(
    int32 yaw_steps,
    int32 pitch_steps)
{
    uint32 primask;

    primask = interrupt_global_disable();
    if((gimbal_stop_latched != 0U)
        || (gimbal_axes_zeroed() == 0U)
        || (gimbal_select_key.pressed != 0U)
        || (gimbal_negative_key.pressed != 0U)
        || (gimbal_positive_key.pressed != 0U))
    {
        interrupt_global_enable(primask);
        return 0U;
    }

    gimbal_axes[GIMBAL_STEPPER_AXIS_YAW].target_position_steps =
        gimbal_clamp_position(
            yaw_steps,
            GIMBAL_STEPPER_YAW_MIN_STEPS,
            GIMBAL_STEPPER_YAW_MAX_STEPS);
    gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH].target_position_steps =
        gimbal_clamp_position(
            pitch_steps,
            GIMBAL_STEPPER_PITCH_MIN_STEPS,
            GIMBAL_STEPPER_PITCH_MAX_STEPS);
    gimbal_laser_force_off();
    gimbal_laser_settle_ms = 0U;
    gimbal_control_mode = GIMBAL_CONTROL_POSITION;
    interrupt_global_enable(primask);
    return 1U;
}

/**
 * @brief Calculate feedforward without changing motor targets.
 */
uint8 gimbal_stepper_compute_feedforward(
    const gimbal_feedforward_pose_struct *pose,
    gimbal_feedforward_solution_struct *solution)
{
    gimbal_vector3_struct target;

    if(solution == NULL)
    {
        return 0U;
    }

    target = gimbal_get_target_point();
    solution->target_x_mm = target.x;
    solution->target_y_mm = target.y;
    solution->target_z_mm = target.z;
    solution->yaw_deg = 0.0F;
    solution->pitch_deg = GIMBAL_CONFIG_PITCH_ZERO_DEG;
    solution->residual_deg = 0.0F;
    solution->valid = 0U;
    solution->singular = 0U;

    if((pose == NULL)
        || (pose->valid == 0U)
        || (gimbal_float_is_valid(pose->x_mm) == 0U)
        || (gimbal_float_is_valid(pose->y_mm) == 0U)
        || (gimbal_float_is_valid(pose->z_mm) == 0U)
        || (gimbal_float_is_valid(pose->roll_deg) == 0U)
        || (gimbal_float_is_valid(pose->pitch_deg) == 0U)
        || (gimbal_float_is_valid(pose->heading_rad) == 0U))
    {
        return 0U;
    }

    return gimbal_solve_feedforward(pose, target, solution);
}

/**
 * @brief Run the configured world-to-gimbal feedforward inverse solver.
 */
uint8 gimbal_stepper_update_feedforward(
    const gimbal_feedforward_pose_struct *pose)
{
    gimbal_feedforward_solution_struct solution;
    uint32 primask;
    int32 previous_yaw_steps;
    int32 nominal_yaw_steps;
    int32 yaw_steps;
    int32 pitch_steps;

    if(gimbal_stepper_compute_feedforward(pose, &solution) == 0U)
    {
        primask = interrupt_global_disable();
        gimbal_feedforward_solution = solution;
        interrupt_global_enable(primask);
        return 0U;
    }

    nominal_yaw_steps = gimbal_angle_to_steps(
        solution.yaw_deg,
        GIMBAL_CONFIG_YAW_POSITION_SIGN);
    pitch_steps = gimbal_angle_to_steps(
        solution.pitch_deg - GIMBAL_CONFIG_PITCH_ZERO_DEG,
        GIMBAL_CONFIG_PITCH_POSITION_SIGN);

    primask = interrupt_global_disable();
    previous_yaw_steps =
        gimbal_axes[GIMBAL_STEPPER_AXIS_YAW].target_position_steps;
    interrupt_global_enable(primask);
    if(gimbal_select_yaw_target_steps(
            nominal_yaw_steps,
            previous_yaw_steps,
            &yaw_steps) == 0U)
    {
        solution.valid = 0U;
        primask = interrupt_global_disable();
        gimbal_feedforward_solution = solution;
        interrupt_global_enable(primask);
        return 0U;
    }

    primask = interrupt_global_disable();
    gimbal_feedforward_solution = solution;
    interrupt_global_enable(primask);
    return gimbal_stepper_set_absolute_target_steps(
        yaw_steps,
        pitch_steps);
}

/**
 * @brief Copy the latest geometric feedforward result.
 */
void gimbal_stepper_get_feedforward_solution(
    gimbal_feedforward_solution_struct *solution)
{
    uint32 primask;

    if(solution == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *solution = gimbal_feedforward_solution;
    interrupt_global_enable(primask);
}

/**
 * @brief Initialize B13 to the configured laser-safe state.
 */
void gimbal_stepper_laser_init(void)
{
    uint8 off_level = GIMBAL_LASER_ACTIVE_LEVEL == GPIO_HIGH
        ? GPIO_LOW : GPIO_HIGH;

    if(GIMBAL_CONFIG_LASER_POLARITY_VALID == 0U)
    {
        gpio_init(
            GIMBAL_LASER_PIN,
            GPI,
            GPIO_LOW,
            GPI_FLOATING_IN);
    }
    else
    {
        gpio_init(
            GIMBAL_LASER_PIN,
            GPO,
            off_level,
            GPO_PUSH_PULL);
    }
}

/**
 * @brief Set B13 according to the configured laser active level.
 */
uint8 gimbal_stepper_set_laser(uint8 enabled)
{
    uint8 accepted = 1U;
    uint32 primask;

    if(GIMBAL_CONFIG_LASER_POLARITY_VALID == 0U)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    if(enabled == 0U)
    {
        gimbal_laser_force_off();
    }
    else if((gimbal_laser_settle_ms
            < GIMBAL_CONFIG_LASER_SETTLE_MS)
        || (gimbal_laser_can_enable() == 0U))
    {
        gimbal_laser_force_off();
        accepted = 0U;
    }
    else
    {
        gpio_set_level(GIMBAL_LASER_PIN, GIMBAL_LASER_ACTIVE_LEVEL);
    }
    interrupt_global_enable(primask);
    return accepted;
}
