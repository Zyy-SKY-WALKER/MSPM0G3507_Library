/**
 * @file    speed_pid.c
 * @brief   Dual-wheel position-form speed PID controller implementation.
 */

#include "speed_pid.h"

#include "drive_geometry.h"
#include "motor.h"
#include "zf_common_interrupt.h"

#define SPEED_PID_SAMPLE_PERIOD_S          \
    ((float)SPEED_PID_SAMPLE_PERIOD_MS / 1000.0F)
#define SPEED_PID_GAIN_LIMIT                (1000.0F)

/** @brief Dynamic state and calibration for one position-form wheel PID. */
typedef struct
{
    /** Requested wheel speed in millimeters per second. */
    float target_mm_s;
    /** Current and previous errors in millimeters per second. */
    float error;
    float previous_error;
    /** Independently bounded integral duty contribution. */
    float integral_output;
    /** Latest total position PID output. */
    float output;
    float kp;
    float ki;
    float kd;
    /** Velocity feedforward gain in duty per millimeter per second. */
    float kff;
    /** Conversion factor from counts per sample to mm/s. */
    float mm_s_per_count;
    /** Four-sample rolling encoder-count average state. */
    int16 speed_history[SPEED_PID_SPEED_FILTER_SAMPLES];
    int32 speed_history_sum;
    uint8 speed_history_index;
    uint8 speed_history_count;
    /** Nonzero when the candidate output was clamped to the duty limit. */
    uint8 saturated;
    /** Nonzero while output is held at zero for a direction reversal. */
    uint8 reversing;
} speed_pid_controller_struct;

static volatile speed_pid_controller_struct speed_pid_left;
static volatile speed_pid_controller_struct speed_pid_right;
static volatile speed_pid_status_struct speed_pid_status;

/**
 * @brief Return the sign of a floating-point value.
 * @param value Input value.
 * @return 1 for positive, -1 for negative and 0 for zero.
 */
static int8 speed_pid_get_sign(float value)
{
    int8 sign = 0;

    if (value > 0.0F)
    {
        sign = 1;
    }
    else if (value < 0.0F)
    {
        sign = -1;
    }

    return sign;
}

/**
 * @brief Clamp a target speed to the configured physical limit.
 * @param target_mm_s Requested target speed.
 * @return Clamped target speed.
 */
static float speed_pid_clamp_target(float target_mm_s)
{
    if (target_mm_s != target_mm_s)
    {
        target_mm_s = 0.0F;
    }
    else if (target_mm_s > SPEED_PID_TARGET_LIMIT_MM_S)
    {
        target_mm_s = SPEED_PID_TARGET_LIMIT_MM_S;
    }
    else if (target_mm_s < -SPEED_PID_TARGET_LIMIT_MM_S)
    {
        target_mm_s = -SPEED_PID_TARGET_LIMIT_MM_S;
    }

    return target_mm_s;
}

/**
 * @brief Check that one configurable gain is finite and bounded.
 * @param gain Gain to validate.
 * @return Nonzero when the gain is valid.
 */
static uint8 speed_pid_gain_is_valid(float gain)
{
    return (uint8)((gain == gain)
        && (gain >= -SPEED_PID_GAIN_LIMIT)
        && (gain <= SPEED_PID_GAIN_LIMIT));
}

/**
 * @brief Clamp one floating point value to a closed range.
 */
static float speed_pid_clamp(float value, float minimum, float maximum)
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
 * @brief Clear one controller's dynamic state while preserving its gains.
 * @param controller Controller to reset.
 */
static void speed_pid_reset_runtime(
    volatile speed_pid_controller_struct *controller)
{
    controller->error = 0.0F;
    controller->previous_error = 0.0F;
    controller->integral_output = 0.0F;
    controller->output = 0.0F;
    controller->saturated = 0U;
}

/**
 * @brief Clear one controller's rolling speed measurement history.
 */
static void speed_pid_reset_speed_filter(
    volatile speed_pid_controller_struct *controller)
{
    uint8 index;

    for(index = 0U; index < SPEED_PID_SPEED_FILTER_SAMPLES; index++)
    {
        controller->speed_history[index] = 0;
    }
    controller->speed_history_sum = 0;
    controller->speed_history_index = 0U;
    controller->speed_history_count = 0U;
}

/**
 * @brief Add one raw count sample and return filtered speed in mm/s.
 */
static float speed_pid_update_speed_filter(
    volatile speed_pid_controller_struct *controller,
    int16 measured_count)
{
    controller->speed_history_sum -=
        controller->speed_history[controller->speed_history_index];
    controller->speed_history[controller->speed_history_index] = measured_count;
    controller->speed_history_sum += measured_count;
    controller->speed_history_index++;
    if(controller->speed_history_index >= SPEED_PID_SPEED_FILTER_SAMPLES)
    {
        controller->speed_history_index = 0U;
    }
    if(controller->speed_history_count < SPEED_PID_SPEED_FILTER_SAMPLES)
    {
        controller->speed_history_count++;
    }

    return ((float)controller->speed_history_sum
        / (float)controller->speed_history_count)
        * controller->mm_s_per_count;
}

/**
 * @brief Initialize one private controller instance.
 * @param controller Controller to initialize.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
static void speed_pid_controller_init(
    volatile speed_pid_controller_struct *controller,
    float kp,
    float ki,
    float kd,
    float kff,
    float mm_per_count)
{
    controller->target_mm_s = 0.0F;
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->kff = kff;
    controller->mm_s_per_count =
        mm_per_count / SPEED_PID_SAMPLE_PERIOD_S;
    controller->reversing = 0U;
    speed_pid_reset_runtime(controller);
    speed_pid_reset_speed_filter(controller);
}

/**
 * @brief Set one controller target and detect direct sign reversals.
 * @param controller Controller to update.
 * @param target_mm_s Requested target speed.
 */
static void speed_pid_controller_set_target(
    volatile speed_pid_controller_struct *controller,
    float target_mm_s)
{
    int8 previous_sign = speed_pid_get_sign(controller->target_mm_s);
    int8 new_sign;

    target_mm_s = speed_pid_clamp_target(target_mm_s);
    new_sign = speed_pid_get_sign(target_mm_s);

    if ((previous_sign != 0) && (new_sign != 0)
        && (previous_sign != new_sign))
    {
        /* Drop accumulated duty before waiting for the wheel to stop. */
        speed_pid_reset_runtime(controller);
        controller->reversing = 1U;
    }
    else if (new_sign == 0)
    {
        speed_pid_reset_runtime(controller);
        speed_pid_reset_speed_filter(controller);
        controller->reversing = 0U;
    }

    controller->target_mm_s = target_mm_s;
}

/**
 * @brief Calculate one position PID output from filtered wheel speed.
 * @param controller Controller state.
 * @param measured_count Raw encoder count for direction-reversal safety.
 * @param measured_speed_mm_s Four-sample filtered speed feedback.
 * @return Signed motor duty command.
 */
static int16 speed_pid_controller_update(
    volatile speed_pid_controller_struct *controller,
    int16 measured_count,
    float measured_speed_mm_s)
{
    float proportional_output;
    float derivative_output;
    float feedforward_output;
    float integral_delta;
    float candidate_integral;
    float candidate_output;

    if (controller->target_mm_s == 0.0F)
    {
        speed_pid_reset_runtime(controller);
        return 0;
    }

    if (controller->reversing != 0U)
    {
        /* Keep PWM disabled until measured motion reaches the stop band. */
        speed_pid_reset_runtime(controller);

        if ((measured_count >= -SPEED_PID_REVERSE_STOP_COUNT)
            && (measured_count <= SPEED_PID_REVERSE_STOP_COUNT))
        {
            controller->reversing = 0U;
            speed_pid_reset_speed_filter(controller);
        }

        return 0;
    }

    controller->error = controller->target_mm_s - measured_speed_mm_s;
    proportional_output = controller->kp * controller->error;
    derivative_output = controller->kd
        * (controller->error - controller->previous_error);
    feedforward_output = speed_pid_clamp(
        controller->kff * controller->target_mm_s,
        -SPEED_PID_FEEDFORWARD_LIMIT,
        SPEED_PID_FEEDFORWARD_LIMIT);
    integral_delta = controller->ki * controller->error;
    candidate_integral = speed_pid_clamp(
        controller->integral_output + integral_delta,
        -SPEED_PID_INTEGRAL_LIMIT,
        SPEED_PID_INTEGRAL_LIMIT);
    candidate_output = proportional_output + candidate_integral
        + derivative_output + feedforward_output;

    /* Do not wind the integral farther into a saturated total output. */
    if(((candidate_output > SPEED_PID_OUTPUT_LIMIT) && (integral_delta > 0.0F))
        || ((candidate_output < -SPEED_PID_OUTPUT_LIMIT)
            && (integral_delta < 0.0F)))
    {
        candidate_integral = controller->integral_output;
        candidate_output = proportional_output + candidate_integral
            + derivative_output + feedforward_output;
    }

    controller->saturated = 0U;
    if (candidate_output > (float)SPEED_PID_OUTPUT_LIMIT)
    {
        candidate_output = (float)SPEED_PID_OUTPUT_LIMIT;
        controller->saturated = 1U;
    }
    else if (candidate_output < -(float)SPEED_PID_OUTPUT_LIMIT)
    {
        candidate_output = -(float)SPEED_PID_OUTPUT_LIMIT;
        controller->saturated = 1U;
    }

    controller->integral_output = candidate_integral;
    controller->output = candidate_output;
    controller->previous_error = controller->error;

    if (candidate_output >= 0.0F)
    {
        return (int16)(candidate_output + 0.5F);
    }

    return (int16)(candidate_output - 0.5F);
}

/**
 * @brief Initialize motor outputs and both speed controllers.
 */
void speed_pid_init(void)
{
    motor_init();
    speed_pid_controller_init(
        &speed_pid_left,
        SPEED_PID_LEFT_KP,
        SPEED_PID_LEFT_KI,
        SPEED_PID_LEFT_KD,
        SPEED_PID_SPEED_KFF,
        DRIVE_LEFT_MM_PER_COUNT);
    speed_pid_controller_init(
        &speed_pid_right,
        SPEED_PID_RIGHT_KP,
        SPEED_PID_RIGHT_KI,
        SPEED_PID_RIGHT_KD,
        SPEED_PID_SPEED_KFF,
        DRIVE_RIGHT_MM_PER_COUNT);
    motor_stop();
    speed_pid_reset();
}

/**
 * @brief Set signed left and right wheel target speeds.
 * @param left_mm_s Left wheel target in millimeters per second.
 * @param right_mm_s Right wheel target in millimeters per second.
 */
void speed_pid_set_target(float left_mm_s, float right_mm_s)
{
    uint32 primask = interrupt_global_disable();

    speed_pid_controller_set_target(&speed_pid_left, left_mm_s);
    speed_pid_controller_set_target(&speed_pid_right, right_mm_s);

    interrupt_global_enable(primask);
}

/**
 * @brief Apply one shared PID gain set to both wheel controllers.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @return ZF_TRUE when the gains were accepted.
 * @note Runtime state and current output are preserved during the update.
 */
uint8 speed_pid_set_shared_gains(float kp, float ki, float kd)
{
    uint32 primask;

    if ((speed_pid_gain_is_valid(kp) == 0U)
        || (speed_pid_gain_is_valid(ki) == 0U)
        || (speed_pid_gain_is_valid(kd) == 0U))
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    speed_pid_left.kp = kp;
    speed_pid_left.ki = ki;
    speed_pid_left.kd = kd;
    speed_pid_right.kp = kp;
    speed_pid_right.ki = ki;
    speed_pid_right.kd = kd;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Set one shared velocity feedforward gain for both wheel controllers.
 */
uint8 speed_pid_set_shared_feedforward(float kff)
{
    uint32 primask;

    if(speed_pid_gain_is_valid(kff) == 0U)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    speed_pid_left.kff = kff;
    speed_pid_right.kff = kff;
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Execute one 10 millisecond dual-wheel control update.
 */
void speed_pid_update_10ms(int16 left_count, int16 right_count)
{
    int16 left_duty;
    int16 right_duty;
    float left_speed_mm_s;
    float right_speed_mm_s;

    if(speed_pid_left.target_mm_s == 0.0F)
    {
        speed_pid_reset_speed_filter(&speed_pid_left);
        left_speed_mm_s = 0.0F;
    }
    else
    {
        left_speed_mm_s = speed_pid_update_speed_filter(
            &speed_pid_left,
            left_count);
    }
    if(speed_pid_right.target_mm_s == 0.0F)
    {
        speed_pid_reset_speed_filter(&speed_pid_right);
        right_speed_mm_s = 0.0F;
    }
    else
    {
        right_speed_mm_s = speed_pid_update_speed_filter(
            &speed_pid_right,
            right_count);
    }

    left_duty = speed_pid_controller_update(
        &speed_pid_left,
        left_count,
        left_speed_mm_s);
    right_duty = speed_pid_controller_update(
        &speed_pid_right,
        right_count,
        right_speed_mm_s);

    motor_left_set_duty(left_duty);
    motor_right_set_duty(right_duty);

    speed_pid_status.left_target_mm_s = speed_pid_left.target_mm_s;
    speed_pid_status.right_target_mm_s = speed_pid_right.target_mm_s;
    speed_pid_status.left_speed_mm_s = left_speed_mm_s;
    speed_pid_status.right_speed_mm_s = right_speed_mm_s;
    speed_pid_status.left_count = left_count;
    speed_pid_status.right_count = right_count;
    speed_pid_status.left_duty = left_duty;
    speed_pid_status.right_duty = right_duty;
    speed_pid_status.left_saturated = speed_pid_left.saturated;
    speed_pid_status.right_saturated = speed_pid_right.saturated;
    speed_pid_status.left_reversing = speed_pid_left.reversing;
    speed_pid_status.right_reversing = speed_pid_right.reversing;

}

/**
 * @brief Stop both wheels and clear all controller state.
 */
void speed_pid_stop(void)
{
    speed_pid_reset();
}

/**
 * @brief Clear controller state and stop both motor outputs.
 */
void speed_pid_reset(void)
{
    uint32 primask = interrupt_global_disable();

    speed_pid_controller_set_target(&speed_pid_left, 0.0F);
    speed_pid_controller_set_target(&speed_pid_right, 0.0F);
    speed_pid_reset_speed_filter(&speed_pid_left);
    speed_pid_reset_speed_filter(&speed_pid_right);
    motor_stop();

    speed_pid_status.left_target_mm_s = 0.0F;
    speed_pid_status.right_target_mm_s = 0.0F;
    speed_pid_status.left_speed_mm_s = 0.0F;
    speed_pid_status.right_speed_mm_s = 0.0F;
    speed_pid_status.left_count = 0;
    speed_pid_status.right_count = 0;
    speed_pid_status.left_duty = 0;
    speed_pid_status.right_duty = 0;
    speed_pid_status.left_saturated = 0U;
    speed_pid_status.right_saturated = 0U;
    speed_pid_status.left_reversing = 0U;
    speed_pid_status.right_reversing = 0U;

    interrupt_global_enable(primask);
}

/**
 * @brief Set left-wheel position PID gains.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void speed_pid_set_left_gains(float kp, float ki, float kd)
{
    uint32 primask;

    if ((speed_pid_gain_is_valid(kp) == 0U)
        || (speed_pid_gain_is_valid(ki) == 0U)
        || (speed_pid_gain_is_valid(kd) == 0U))
    {
        return;
    }

    primask = interrupt_global_disable();

    speed_pid_left.kp = kp;
    speed_pid_left.ki = ki;
    speed_pid_left.kd = kd;
    speed_pid_reset_runtime(&speed_pid_left);

    interrupt_global_enable(primask);
}

/**
 * @brief Set right-wheel position PID gains.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void speed_pid_set_right_gains(float kp, float ki, float kd)
{
    uint32 primask;

    if ((speed_pid_gain_is_valid(kp) == 0U)
        || (speed_pid_gain_is_valid(ki) == 0U)
        || (speed_pid_gain_is_valid(kd) == 0U))
    {
        return;
    }

    primask = interrupt_global_disable();

    speed_pid_right.kp = kp;
    speed_pid_right.ki = ki;
    speed_pid_right.kd = kd;
    speed_pid_reset_runtime(&speed_pid_right);

    interrupt_global_enable(primask);
}

/**
 * @brief Copy the most recent dual-wheel control status.
 * @param status Destination status structure.
 */
void speed_pid_get_status(speed_pid_status_struct *status)
{
    uint32 primask;

    if (status == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *status = speed_pid_status;
    interrupt_global_enable(primask);
}
