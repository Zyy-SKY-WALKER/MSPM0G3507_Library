/**
 * @file    line_tracker.c
 * @brief   Pure eight-channel line tracking target generator.
 */

#include "line_tracker.h"

#include <float.h>

#include "zf_common_interrupt.h"

#define LINE_TRACKER_NORMALIZE_SCALE    (5.0F / 3.5F)
#define LINE_TRACKER_DEVIATION_LIMIT    (3.5F)
#define LINE_TRACKER_DIRECTION_EPSILON  (0.05F)
#define LINE_TRACKER_FLOAT_EPSILON      (0.001F)
#define LINE_TRACKER_PID_LIMIT_EPSILON  (0.001F)
#define LINE_TRACKER_COUNTER_MAX        (0xFFFFU)
#define LINE_TRACKER_UPDATE_PERIOD_S    \
    ((float)LINE_TRACKER_UPDATE_PERIOD_MS * 0.001F)

static const line_tracker_config_struct line_tracker_default_config =
{
    .base_speed_mm_s = {550.0F, 500.0F, 480.0F, 440.0F, 400.0F},
    .pid_kp = {30.0F, 38.0F, 43.0F, 55.0F, 68.0F},
    .pid_ki = 0.0F,
    .pid_kd = 0.0F,
    .pid_integral_limit_mm_s = 50.0F,
    .pid_derivative_filter_alpha = 0.2F,
    .max_target_mm_s = 800.0F,
    .max_correction_mm_s = 400.0F,
    .arc_outer_speed_mm_s = 630.0F,
    .arc_inner_speed_mm_s = 120.0F,
    .pivot_speed_mm_s = 300.0F,
    .lost_debounce_samples = 3U,
    .reacquire_samples = 3U,
    .arc_duration_samples = 50U,
    .search_timeout_samples = 500U,
    .default_search_direction = LINE_TRACKER_DIRECTION_RIGHT,
};

static volatile line_tracker_config_struct line_tracker_config;
static volatile line_tracker_status_struct line_tracker_status;
static volatile uint8 line_tracker_initialized;

/** @brief Private history required by the lateral PID controller. */
typedef struct
{
    /** Bounded integral contribution, already scaled to mm/s. */
    float integral_mm_s;
    /** Previous normalized error used by the discrete derivative. */
    float previous_error;
    /** Low-pass-filtered normalized-error derivative per second. */
    float filtered_derivative;
    /** Nonzero after the first valid tracking sample. */
    uint8 initialized;
} line_tracker_pid_state_struct;

static volatile line_tracker_pid_state_struct line_tracker_pid_state;

/**
 * @brief Check that a float is finite and nonnegative.
 * @param value Value to validate.
 * @return Nonzero when valid.
 */
static uint8 line_tracker_float_is_nonnegative(float value)
{
    return (uint8)((value == value)
        && (value >= 0.0F)
        && (value <= FLT_MAX));
}

/**
 * @brief Validate one complete runtime configuration.
 * @param config Configuration to validate.
 * @return ZF_TRUE when every field is valid.
 */
static uint8 line_tracker_config_is_valid(
    const line_tracker_config_struct *config)
{
    uint8 index;

    if (config == NULL)
    {
        return ZF_FALSE;
    }

    if ((line_tracker_float_is_nonnegative(config->max_target_mm_s) == 0U)
        || (config->max_target_mm_s <= 0.0F)
        || (line_tracker_float_is_nonnegative(
                config->max_correction_mm_s) == 0U)
        || (line_tracker_float_is_nonnegative(config->pid_ki) == 0U)
        || (line_tracker_float_is_nonnegative(config->pid_kd) == 0U)
        || (line_tracker_float_is_nonnegative(
                config->pid_integral_limit_mm_s) == 0U)
        || (line_tracker_float_is_nonnegative(
                config->pid_derivative_filter_alpha) == 0U)
        || (config->pid_derivative_filter_alpha > 1.0F)
        || (line_tracker_float_is_nonnegative(
                config->arc_outer_speed_mm_s) == 0U)
        || (line_tracker_float_is_nonnegative(
                config->arc_inner_speed_mm_s) == 0U)
        || (line_tracker_float_is_nonnegative(
                config->pivot_speed_mm_s) == 0U))
    {
        return ZF_FALSE;
    }

    if ((config->arc_outer_speed_mm_s > config->max_target_mm_s)
        || (config->arc_inner_speed_mm_s > config->max_target_mm_s)
        || (config->pivot_speed_mm_s > config->max_target_mm_s)
        || (config->arc_outer_speed_mm_s
            <= config->arc_inner_speed_mm_s)
        || (config->lost_debounce_samples == 0U)
        || (config->reacquire_samples == 0U)
        || (config->arc_duration_samples == 0U)
        || (config->search_timeout_samples
            <= config->arc_duration_samples)
        || ((config->default_search_direction
                != LINE_TRACKER_DIRECTION_LEFT)
            && (config->default_search_direction
                != LINE_TRACKER_DIRECTION_RIGHT)))
    {
        return ZF_FALSE;
    }

    for (index = 0U; index < LINE_TRACKER_SPEED_BAND_COUNT; index++)
    {
        if ((line_tracker_float_is_nonnegative(
                config->base_speed_mm_s[index]) == 0U)
            || (line_tracker_float_is_nonnegative(
                config->pid_kp[index]) == 0U)
            || (config->base_speed_mm_s[index]
                > config->max_target_mm_s))
        {
            return ZF_FALSE;
        }
    }

    return ZF_TRUE;
}

/**
 * @brief Clear lateral PID history without changing tracker state.
 */
static void line_tracker_pid_reset(void)
{
    line_tracker_pid_state.integral_mm_s = 0.0F;
    line_tracker_pid_state.previous_error = 0.0F;
    line_tracker_pid_state.filtered_derivative = 0.0F;
    line_tracker_pid_state.initialized = 0U;
    line_tracker_status.correction_mm_s = 0.0F;
    line_tracker_status.pid_integral_mm_s = 0.0F;
    line_tracker_status.pid_filtered_derivative = 0.0F;
}

/**
 * @brief Calculate the usable correction before either wheel clips.
 * @param base_speed Base wheel speed for the selected band.
 * @return Symmetric effective correction limit in mm/s.
 */
static float line_tracker_pid_get_effective_limit(float base_speed)
{
    float target_margin =
        line_tracker_config.max_target_mm_s - base_speed;
    float effective_limit = line_tracker_config.max_correction_mm_s;

    if (target_margin < effective_limit)
    {
        effective_limit = target_margin;
    }
    if (base_speed < effective_limit)
    {
        effective_limit = base_speed;
    }

    return effective_limit;
}

/**
 * @brief Calculate one filtered PID correction with anti-windup.
 * @param error Normalized lateral error.
 * @param band Active base-speed and proportional-gain band.
 * @return Signed differential speed correction in millimeters per second.
 */
static float line_tracker_pid_update(float error, uint8 band)
{
    float proportional;
    float derivative;
    float raw_derivative = 0.0F;
    float integral_candidate;
    float correction_candidate;
    float correction;
    float effective_limit;
    uint8 drives_further_into_saturation;

    /* Seed the derivative history to prevent a kick on reacquisition. */
    if (line_tracker_pid_state.initialized == 0U)
    {
        line_tracker_pid_state.previous_error = error;
        line_tracker_pid_state.filtered_derivative = 0.0F;
        line_tracker_pid_state.initialized = 1U;
    }
    else
    {
        /* Filter the discrete derivative before applying derivative gain. */
        raw_derivative =
            (error - line_tracker_pid_state.previous_error)
            / LINE_TRACKER_UPDATE_PERIOD_S;
        line_tracker_pid_state.filtered_derivative +=
            line_tracker_config.pid_derivative_filter_alpha
            * (raw_derivative
                - line_tracker_pid_state.filtered_derivative);
    }

    proportional = line_tracker_config.pid_kp[band] * error;
    derivative = line_tracker_config.pid_kd
        * line_tracker_pid_state.filtered_derivative;

    /* Build and clamp a candidate integral contribution first. */
    integral_candidate = line_tracker_pid_state.integral_mm_s
        + (line_tracker_config.pid_ki
            * error
            * LINE_TRACKER_UPDATE_PERIOD_S);

    if (integral_candidate
        > line_tracker_config.pid_integral_limit_mm_s)
    {
        integral_candidate =
            line_tracker_config.pid_integral_limit_mm_s;
    }
    else if (integral_candidate
        < -line_tracker_config.pid_integral_limit_mm_s)
    {
        integral_candidate =
            -line_tracker_config.pid_integral_limit_mm_s;
    }

    correction_candidate = proportional
        + integral_candidate
        + derivative;

    /* Wheel limits can be tighter than the configured correction limit. */
    effective_limit = line_tracker_pid_get_effective_limit(
        line_tracker_config.base_speed_mm_s[band]);
    drives_further_into_saturation = (uint8)(
        ((correction_candidate
                > (effective_limit + LINE_TRACKER_PID_LIMIT_EPSILON))
            && (error > 0.0F))
        || ((correction_candidate
                < (-effective_limit - LINE_TRACKER_PID_LIMIT_EPSILON))
            && (error < 0.0F)));

    /* Freeze only integration that would push farther into saturation. */
    if (drives_further_into_saturation != 0U)
    {
        line_tracker_status.output_limited = 1U;
    }
    else
    {
        line_tracker_pid_state.integral_mm_s = integral_candidate;
    }

    correction = proportional
        + line_tracker_pid_state.integral_mm_s
        + derivative;
    line_tracker_pid_state.previous_error = error;
    line_tracker_status.pid_integral_mm_s =
        line_tracker_pid_state.integral_mm_s;
    line_tracker_status.pid_filtered_derivative =
        line_tracker_pid_state.filtered_derivative;

    return correction;
}

/**
 * @brief Set output targets and mirror them into status.
 * @param output Destination output.
 * @param left_mm_s Left target speed.
 * @param right_mm_s Right target speed.
 */
static void line_tracker_set_output(
    line_tracker_output_struct *output,
    float left_mm_s,
    float right_mm_s)
{
    output->left_target_mm_s = left_mm_s;
    output->right_target_mm_s = right_mm_s;
    line_tracker_status.left_target_mm_s = left_mm_s;
    line_tracker_status.right_target_mm_s = right_mm_s;
}

/**
 * @brief Latch a fault and force both outputs to zero.
 * @param output Destination output when available.
 */
static void line_tracker_enter_fault(line_tracker_output_struct *output)
{
    line_tracker_pid_reset();
    line_tracker_status.state = LINE_TRACKER_STATE_FAULT;
    line_tracker_status.output_limited = 0U;

    if (output != NULL)
    {
        line_tracker_set_output(output, 0.0F, 0.0F);
    }
    else
    {
        line_tracker_status.left_target_mm_s = 0.0F;
        line_tracker_status.right_target_mm_s = 0.0F;
    }
}

/**
 * @brief Clamp one target speed to configured bounds.
 * @param target Target to clamp.
 * @param allow_reverse Nonzero permits negative output.
 * @return Clamped target.
 */
static float line_tracker_clamp_target(float target, uint8 allow_reverse)
{
    float minimum = allow_reverse != 0U
        ? -line_tracker_config.max_target_mm_s
        : 0.0F;

    if (target > line_tracker_config.max_target_mm_s)
    {
        target = line_tracker_config.max_target_mm_s;
        line_tracker_status.output_limited = 1U;
    }
    else if (target < minimum)
    {
        target = minimum;
        line_tracker_status.output_limited = 1U;
    }

    return target;
}

/**
 * @brief Validate consistency of one grayscale result.
 * @param sensor Sensor result to validate.
 * @return ZF_TRUE when internally consistent.
 */
static uint8 line_tracker_sensor_is_valid(
    const gray_sensor_result_struct *sensor)
{
    gray_sensor_result_struct expected;
    float position_error;
    float deviation_error;

    if (sensor == NULL)
    {
        return ZF_FALSE;
    }

    gray_sensor_calculate(sensor->active_mask, &expected);
    position_error = sensor->position - expected.position;
    deviation_error = sensor->deviation - expected.deviation;
    if (position_error < 0.0F)
    {
        position_error = -position_error;
    }
    if (deviation_error < 0.0F)
    {
        deviation_error = -deviation_error;
    }

    if ((sensor->status != expected.status)
        || (sensor->active_count != expected.active_count)
        || (sensor->weighted_sum != expected.weighted_sum)
        || (position_error > LINE_TRACKER_FLOAT_EPSILON)
        || (deviation_error > LINE_TRACKER_FLOAT_EPSILON))
    {
        return ZF_FALSE;
    }

    switch (sensor->status)
    {
        case GRAY_SENSOR_STATUS_LOST:
        {
            return ZF_TRUE;
        }

        case GRAY_SENSOR_STATUS_ALL_ACTIVE:
        {
            return ZF_TRUE;
        }

        case GRAY_SENSOR_STATUS_VALID:
        {
            return (uint8)((sensor->deviation == sensor->deviation)
                && (sensor->deviation >= -LINE_TRACKER_DEVIATION_LIMIT)
                && (sensor->deviation <= LINE_TRACKER_DEVIATION_LIMIT));
        }

        default:
        {
            return ZF_FALSE;
        }
    }
}

/**
 * @brief Select the low-speed tracking band.
 * @param absolute_deviation Absolute normalized deviation.
 * @return Band index from zero through four.
 */
static uint8 line_tracker_get_speed_band(float absolute_deviation)
{
    uint8 band;

    if (absolute_deviation <= 1.0F)
    {
        band = 0U;
    }
    else if (absolute_deviation <= 2.0F)
    {
        band = 1U;
    }
    else if (absolute_deviation <= 3.0F)
    {
        band = 2U;
    }
    else if (absolute_deviation <= 4.0F)
    {
        band = 3U;
    }
    else
    {
        band = 4U;
    }

    return band;
}

/**
 * @brief Generate normal forward tracking targets.
 * @param sensor Valid grayscale result.
 * @param output Destination output.
 */
static void line_tracker_update_tracking(
    const gray_sensor_result_struct *sensor,
    line_tracker_output_struct *output)
{
    float normalized = sensor->deviation * LINE_TRACKER_NORMALIZE_SCALE;
    float absolute = normalized >= 0.0F ? normalized : -normalized;
    float correction;
    float left_target;
    float right_target;
    uint8 band = line_tracker_get_speed_band(absolute);

    line_tracker_status.output_limited = 0U;
    correction = line_tracker_pid_update(normalized, band);
    if (correction > line_tracker_config.max_correction_mm_s)
    {
        correction = line_tracker_config.max_correction_mm_s;
        line_tracker_status.output_limited = 1U;
    }
    else if (correction < -line_tracker_config.max_correction_mm_s)
    {
        correction = -line_tracker_config.max_correction_mm_s;
        line_tracker_status.output_limited = 1U;
    }

    left_target = line_tracker_config.base_speed_mm_s[band] + correction;
    right_target = line_tracker_config.base_speed_mm_s[band] - correction;
    left_target = line_tracker_clamp_target(left_target, 0U);
    right_target = line_tracker_clamp_target(right_target, 0U);

    line_tracker_status.speed_band = band;
    line_tracker_status.normalized_deviation = normalized;
    line_tracker_status.correction_mm_s = correction;
    line_tracker_set_output(output, left_target, right_target);
}

/**
 * @brief Select search direction from the latest useful deviation.
 */
static void line_tracker_select_search_direction(void)
{
    if (line_tracker_status.last_valid_deviation
        > LINE_TRACKER_DIRECTION_EPSILON)
    {
        line_tracker_status.search_direction =
            LINE_TRACKER_DIRECTION_RIGHT;
    }
    else if (line_tracker_status.last_valid_deviation
        < -LINE_TRACKER_DIRECTION_EPSILON)
    {
        line_tracker_status.search_direction =
            LINE_TRACKER_DIRECTION_LEFT;
    }
    else
    {
        line_tracker_status.search_direction =
            line_tracker_config.default_search_direction;
    }
}

/**
 * @brief Generate one bounded lost-line search output.
 * @param output Destination output.
 */
static void line_tracker_update_search(line_tracker_output_struct *output)
{
    float left_target;
    float right_target;

    line_tracker_status.output_limited = 0U;
    /* Search timeout transitions to the terminal fault state. */
    if (line_tracker_status.search_samples
        >= (uint16)(line_tracker_config.search_timeout_samples - 1U))
    {
        line_tracker_enter_fault(output);
        return;
    }

    /* Lost-line search transitions from a forward arc to a pivot. */
    if (line_tracker_status.search_samples
        < line_tracker_config.arc_duration_samples)
    {
        line_tracker_status.state = LINE_TRACKER_STATE_LOST_ARC;
        if (line_tracker_status.search_direction
            == LINE_TRACKER_DIRECTION_RIGHT)
        {
            left_target = line_tracker_config.arc_outer_speed_mm_s;
            right_target = line_tracker_config.arc_inner_speed_mm_s;
        }
        else
        {
            left_target = line_tracker_config.arc_inner_speed_mm_s;
            right_target = line_tracker_config.arc_outer_speed_mm_s;
        }
    }
    else
    {
        line_tracker_status.state = LINE_TRACKER_STATE_LOST_PIVOT;
        if (line_tracker_status.search_direction
            == LINE_TRACKER_DIRECTION_RIGHT)
        {
            left_target = line_tracker_config.pivot_speed_mm_s;
            right_target = -line_tracker_config.pivot_speed_mm_s;
        }
        else
        {
            left_target = -line_tracker_config.pivot_speed_mm_s;
            right_target = line_tracker_config.pivot_speed_mm_s;
        }
    }

    left_target = line_tracker_clamp_target(left_target, 1U);
    right_target = line_tracker_clamp_target(right_target, 1U);
    line_tracker_set_output(output, left_target, right_target);
    if (line_tracker_status.search_samples < LINE_TRACKER_COUNTER_MAX)
    {
        line_tracker_status.search_samples++;
    }
}

/**
 * @brief Initialize with caller configuration or safe defaults.
 * @param config Optional validated runtime configuration.
 */
void line_tracker_init(const line_tracker_config_struct *config)
{
    line_tracker_initialized = 0U;
    line_tracker_config = line_tracker_default_config;
    line_tracker_reset();

    if ((config != NULL) && (line_tracker_set_config(config) == ZF_FALSE))
    {
        line_tracker_enter_fault(NULL);
        return;
    }

    line_tracker_initialized = 1U;
    line_tracker_reset();
}

/**
 * @brief Reset tracking history and clear all outputs.
 */
void line_tracker_reset(void)
{
    uint32 primask = interrupt_global_disable();

    line_tracker_status.state = LINE_TRACKER_STATE_TRACKING;
    line_tracker_status.sensor_status = GRAY_SENSOR_STATUS_LOST;
    line_tracker_status.search_direction =
        line_tracker_config.default_search_direction;
    line_tracker_status.deviation = 0.0F;
    line_tracker_status.normalized_deviation = 0.0F;
    line_tracker_status.last_valid_deviation = 0.0F;
    line_tracker_pid_reset();
    line_tracker_status.left_target_mm_s = 0.0F;
    line_tracker_status.right_target_mm_s = 0.0F;
    line_tracker_status.lost_samples = 0U;
    line_tracker_status.valid_samples = 0U;
    line_tracker_status.all_active_samples = 0U;
    line_tracker_status.search_samples = 0U;
    line_tracker_status.speed_band = 0U;
    line_tracker_status.output_limited = 0U;

    interrupt_global_enable(primask);
}

/**
 * @brief Replace runtime configuration atomically.
 * @param config New configuration.
 * @return ZF_TRUE when accepted.
 */
uint8 line_tracker_set_config(
    const line_tracker_config_struct *config)
{
    uint32 primask;

    if (line_tracker_config_is_valid(config) == ZF_FALSE)
    {
        return ZF_FALSE;
    }

    primask = interrupt_global_disable();
    line_tracker_config = *config;
    line_tracker_pid_reset();
    interrupt_global_enable(primask);

    return ZF_TRUE;
}

/**
 * @brief Convert one grayscale result into bounded wheel targets.
 * @param sensor Coherent grayscale result.
 * @param output Destination wheel targets.
 * @return ZF_TRUE unless the tracker is in a fault state.
 */
uint8 line_tracker_update(
    const gray_sensor_result_struct *sensor,
    line_tracker_output_struct *output)
{
    if ((output == NULL) || (line_tracker_initialized == 0U)
        || (line_tracker_sensor_is_valid(sensor) == ZF_FALSE))
    {
        line_tracker_enter_fault(output);
        return ZF_FALSE;
    }

    if (line_tracker_status.state == LINE_TRACKER_STATE_FAULT)
    {
        line_tracker_set_output(output, 0.0F, 0.0F);
        return ZF_FALSE;
    }

    line_tracker_status.sensor_status = sensor->status;
    line_tracker_status.deviation = sensor->deviation;

    /* Nontracking sensor states discard PID history before state handling. */
    if (sensor->status != GRAY_SENSOR_STATUS_VALID)
    {
        line_tracker_pid_reset();
    }

    if (sensor->status == GRAY_SENSOR_STATUS_ALL_ACTIVE)
    {
        /* All-active is a stopped state until valid samples reacquire line. */
        line_tracker_status.state = LINE_TRACKER_STATE_ALL_ACTIVE;
        if (line_tracker_status.all_active_samples
            < LINE_TRACKER_COUNTER_MAX)
        {
            line_tracker_status.all_active_samples++;
        }
        line_tracker_status.valid_samples = 0U;
        line_tracker_status.lost_samples = 0U;
        line_tracker_status.search_samples = 0U;
        line_tracker_status.normalized_deviation = 0.0F;
        line_tracker_status.output_limited = 0U;
        line_tracker_set_output(output, 0.0F, 0.0F);
        return ZF_TRUE;
    }

    if (sensor->status == GRAY_SENSOR_STATUS_LOST)
    {
        /* Debounce line loss before entering the directed search states. */
        line_tracker_status.valid_samples = 0U;
        line_tracker_status.all_active_samples = 0U;
        if (line_tracker_status.lost_samples
            < line_tracker_config.lost_debounce_samples)
        {
            line_tracker_status.lost_samples++;
        }
        line_tracker_status.normalized_deviation = 0.0F;

        if (line_tracker_status.lost_samples
            < line_tracker_config.lost_debounce_samples)
        {
            output->left_target_mm_s =
                line_tracker_status.left_target_mm_s;
            output->right_target_mm_s =
                line_tracker_status.right_target_mm_s;
            return ZF_TRUE;
        }

        if (line_tracker_status.search_samples == 0U)
        {
            line_tracker_select_search_direction();
        }
        line_tracker_update_search(output);
        return line_tracker_status.state != LINE_TRACKER_STATE_FAULT
            ? ZF_TRUE
            : ZF_FALSE;
    }

    line_tracker_status.lost_samples = 0U;
    line_tracker_status.all_active_samples = 0U;
    if (line_tracker_status.valid_samples
        < line_tracker_config.reacquire_samples)
    {
        line_tracker_status.valid_samples++;
    }
    line_tracker_status.last_valid_deviation = sensor->deviation;

    /* Leaving a nontracking state requires consecutive valid samples. */
    if ((line_tracker_status.state != LINE_TRACKER_STATE_TRACKING)
        && (line_tracker_status.valid_samples
            < line_tracker_config.reacquire_samples))
    {
        line_tracker_pid_reset();
        line_tracker_status.output_limited = 0U;
        line_tracker_set_output(output, 0.0F, 0.0F);
        return ZF_TRUE;
    }

    /* Confirmed reacquisition returns ownership to normal tracking. */
    line_tracker_status.state = LINE_TRACKER_STATE_TRACKING;
    line_tracker_status.search_samples = 0U;
    line_tracker_update_tracking(sensor, output);

    return ZF_TRUE;
}

/**
 * @brief Copy one coherent tracker status snapshot.
 * @param status Destination status structure.
 */
void line_tracker_get_status(line_tracker_status_struct *status)
{
    uint32 primask;

    if (status == NULL)
    {
        return;
    }

    primask = interrupt_global_disable();
    *status = line_tracker_status;
    interrupt_global_enable(primask);
}
