/**
 * @file    gimbal_stepper.c
 * @brief   Reusable dual-axis STEP/DIR gimbal controller.
 */

#include "gimbal_stepper.h"

#include <stdio.h>

#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"

#define GIMBAL_YAW_STEP_PIN                     (B4)
#define GIMBAL_PITCH_STEP_PIN                   (B5)
#define GIMBAL_YAW_DIR_PIN                      (B8)
#define GIMBAL_PITCH_DIR_PIN                    (B9)

#define GIMBAL_SELECT_KEY_PIN                   (A30)
#define GIMBAL_STOP_KEY_PIN                     (A31)
#define GIMBAL_NEGATIVE_KEY_PIN                 (B0)
#define GIMBAL_POSITIVE_KEY_PIN                 (B1)

#define GIMBAL_PIT                              (PIT_TIM_G6)
#define GIMBAL_PIT_IRQ                          (TIMG6_INT_IRQn)
#define GIMBAL_PIT_IRQ_PRIORITY                 (1U)
#define GIMBAL_TICK_US                          (200U)
#define GIMBAL_TICK_HZ                          (5000U)
#define GIMBAL_RATE_SCALE                       (1000U)
#define GIMBAL_CONTROL_PERIOD_MS                (10U)
#define GIMBAL_DEBOUNCE_MS                      (20U)
#define GIMBAL_ZERO_HOLD_MS                     (1000U)
#define GIMBAL_DIRECTION_SETTLE_TICKS           (1U)

#define GIMBAL_JOG_SPEED_DEG_S                  (30U)
#define GIMBAL_CALIBRATE_SPEED_DEG_S            (5U)
#define GIMBAL_POSITION_SPEED_DEG_S             (60U)
#define GIMBAL_ACCEL_DEG_S2                     (720U)
#define GIMBAL_POSITION_GAIN_MILLI_RATE         (10000)

#define GIMBAL_JOG_RATE_MILLI_STEPS_S           \
    ((int32)(((uint64)GIMBAL_JOG_SPEED_DEG_S \
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

#define GIMBAL_CALIBRATE_TRAVEL_STEPS           (3200)
#define GIMBAL_YAW_POSITIVE_DIR_LEVEL           (GPIO_HIGH)
#define GIMBAL_PITCH_POSITIVE_DIR_LEVEL         (GPIO_HIGH)

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
 * @brief Stop both outputs immediately from the 5 kHz callback.
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

    printf(
        "Gimbal %s zero captured.\r\n",
        gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
            ? "YAW" : "PITCH");
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
                printf(
                    "Gimbal selected axis: %s.\r\n",
                    gimbal_selected_axis == GIMBAL_STEPPER_AXIS_YAW
                        ? "YAW" : "PITCH");
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
    int32 jog_rate = selected->zero_valid != 0U
        ? GIMBAL_JOG_RATE_MILLI_STEPS_S
        : GIMBAL_CALIBRATE_RATE_MILLI_STEPS_S;
    uint8 negative_only = (uint8)(
        (keys->negative_pressed != 0U)
        && (keys->positive_pressed == 0U));
    uint8 positive_only = (uint8)(
        (keys->positive_pressed != 0U)
        && (keys->negative_pressed == 0U));
    uint8 both_pressed = (uint8)(
        (keys->negative_pressed != 0U)
        && (keys->positive_pressed != 0U));

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
        minimum = -GIMBAL_CALIBRATE_TRAVEL_STEPS;
        maximum = GIMBAL_CALIBRATE_TRAVEL_STEPS;
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
    (void)event;
    (void)context;

    gimbal_millisecond_divider++;
    if(gimbal_millisecond_divider >= 5U)
    {
        gimbal_millisecond_divider = 0U;
        gimbal_update_keys_tick();
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
    if(gimbal_rate_tick_divider >= 50U)
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
    uint8 positive_dir_level)
{
    axis->step_pin = step_pin;
    axis->dir_pin = dir_pin;
    axis->min_position_steps = minimum;
    axis->max_position_steps = maximum;
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
        GIMBAL_YAW_POSITIVE_DIR_LEVEL);
    gimbal_initialize_axis(
        &gimbal_axes[GIMBAL_STEPPER_AXIS_PITCH],
        GIMBAL_PITCH_STEP_PIN,
        GIMBAL_PITCH_DIR_PIN,
        GIMBAL_STEPPER_PITCH_MIN_STEPS,
        GIMBAL_STEPPER_PITCH_MAX_STEPS,
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
        printf("Gimbal emergency stop.\r\n");
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
        printf("Gimbal stop released.\r\n");
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
