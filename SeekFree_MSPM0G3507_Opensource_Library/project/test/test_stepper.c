/**
 * @file    test_stepper.c
 * @author  Project team
 * @version V1.0
 * @date    2026-07-19
 * @brief   Dual-axis STEP/DIR pulse and manual jog test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_STEPPER)

#include "test_stepper.h"

#include <stdio.h>

#include "zf_common_interrupt.h"
#include "zf_common_typedef.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"

#include "ml_stepper_debug.h"

#define STEPPER_TEST_PAN_STEP_PIN               (B4)
#define STEPPER_TEST_TILT_STEP_PIN              (B5)
#define STEPPER_TEST_PAN_DIR_PIN                (B8)
#define STEPPER_TEST_TILT_DIR_PIN               (B9)

#define STEPPER_TEST_SELECT_KEY_PIN             (A30)
#define STEPPER_TEST_STOP_KEY_PIN               (A31)
#define STEPPER_TEST_NEGATIVE_KEY_PIN           (B0)
#define STEPPER_TEST_POSITIVE_KEY_PIN           (B1)

#define STEPPER_TEST_PIT                        (PIT_TIM_G6)
#define STEPPER_TEST_PIT_IRQ                    (TIMG6_INT_IRQn)
#define STEPPER_TEST_PIT_IRQ_PRIORITY           (1U)
#define STEPPER_TEST_TICK_US                    (200U)
#define STEPPER_TEST_TICK_HZ                    (5000U)
#define STEPPER_TEST_RATE_SCALE                 (1000U)
#define STEPPER_TEST_CONTROL_PERIOD_MS          (10U)
#define STEPPER_TEST_LOOP_PERIOD_MS             (1U)
#define STEPPER_TEST_DEBOUNCE_MS                (20U)
#define STEPPER_TEST_ZERO_HOLD_MS               (1000U)
#define STEPPER_TEST_STATUS_PERIOD_MS           (500U)
#define STEPPER_TEST_DIRECTION_SETTLE_TICKS     (1U)

#define STEPPER_TEST_STEPS_PER_REVOLUTION       (12800U)
#define STEPPER_TEST_JOG_SPEED_DEG_S            (30U)
#define STEPPER_TEST_CALIBRATE_SPEED_DEG_S       (5U)
#define STEPPER_TEST_ACCEL_DEG_S2                (720U)
#define STEPPER_TEST_JOG_RATE_MILLI_STEPS_S     \
    ((int32)(((uint64)STEPPER_TEST_JOG_SPEED_DEG_S \
        * STEPPER_TEST_STEPS_PER_REVOLUTION \
        * STEPPER_TEST_RATE_SCALE) / 360U))
#define STEPPER_TEST_CALIBRATE_RATE_MILLI_STEPS_S \
    ((int32)(((uint64)STEPPER_TEST_CALIBRATE_SPEED_DEG_S \
        * STEPPER_TEST_STEPS_PER_REVOLUTION \
        * STEPPER_TEST_RATE_SCALE) / 360U))
#define STEPPER_TEST_RATE_DELTA_MILLI_STEPS_S   \
    ((int32)(((uint64)STEPPER_TEST_ACCEL_DEG_S2 \
        * STEPPER_TEST_STEPS_PER_REVOLUTION \
        * STEPPER_TEST_RATE_SCALE \
        * STEPPER_TEST_CONTROL_PERIOD_MS) \
        / (360U * 1000U)))

#define STEPPER_TEST_PAN_MIN_STEPS              (-6400)
#define STEPPER_TEST_PAN_MAX_STEPS              (6400)
#define STEPPER_TEST_TILT_MIN_STEPS             (-3700)
#define STEPPER_TEST_TILT_MAX_STEPS             (3400)
#define STEPPER_TEST_CALIBRATE_TRAVEL_STEPS     (3200)

#define STEPPER_TEST_PAN_POSITIVE_DIR_LEVEL      (GPIO_HIGH)
#define STEPPER_TEST_TILT_POSITIVE_DIR_LEVEL     (GPIO_HIGH)

typedef enum
{
    STEPPER_TEST_AXIS_PAN = 0,
    STEPPER_TEST_AXIS_TILT,
    STEPPER_TEST_AXIS_COUNT,
} stepper_test_axis_enum;

/** @brief Debounced state for one active-low test key. */
typedef struct
{
    uint16 mismatch_ms;
    uint8 pressed;
} stepper_test_key_struct;

/** @brief Runtime state for one STEP/DIR axis. */
typedef struct
{
    gpio_pin_enum step_pin;
    gpio_pin_enum dir_pin;
    int32 min_position_steps;
    int32 max_position_steps;
    uint8 positive_dir_level;
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
} stepper_test_axis_struct;

static stepper_test_axis_struct stepper_test_axes[
    STEPPER_TEST_AXIS_COUNT];
static stepper_test_key_struct stepper_test_select_key;
static stepper_test_key_struct stepper_test_negative_key;
static stepper_test_key_struct stepper_test_positive_key;
static stepper_test_axis_enum stepper_test_selected_axis;
static uint16 stepper_test_select_hold_ms;
static uint8 stepper_test_select_was_pressed;
static uint8 stepper_test_select_long_handled;
static volatile uint8 stepper_test_stop_latched;
static uint8 stepper_test_stop_reported;

/**
 * @brief Return the opposite digital output level.
 * @param level GPIO_LOW or GPIO_HIGH.
 * @return Opposite output level.
 */
static uint8 stepper_test_invert_level(uint8 level)
{
    return level == GPIO_LOW ? GPIO_HIGH : GPIO_LOW;
}

/**
 * @brief Convert a signed fixed-point step rate into a DDS increment.
 * @param rate_milli_steps_s Signed rate in 0.001 step per second.
 * @return Unsigned 32-bit phase increment.
 */
static uint32 stepper_test_rate_to_phase_increment(
    int32 rate_milli_steps_s)
{
    uint64 magnitude;
    uint64 denominator;

    if (rate_milli_steps_s < 0)
    {
        magnitude = (uint64)(-rate_milli_steps_s);
    }
    else
    {
        magnitude = (uint64)rate_milli_steps_s;
    }

    denominator = (uint64)STEPPER_TEST_TICK_HZ
        * STEPPER_TEST_RATE_SCALE;
    return (uint32)((magnitude << 32U) / denominator);
}

/**
 * @brief Move a signed rate toward a target without crossing zero.
 * @param current Current rate.
 * @param target Requested rate.
 * @return Rate for the next 10 ms control interval.
 */
static int32 stepper_test_ramp_rate(int32 current, int32 target)
{
    int32 delta = STEPPER_TEST_RATE_DELTA_MILLI_STEPS_S;

    if (((current > 0) && (target < 0))
        || ((current < 0) && (target > 0)))
    {
        target = 0;
    }

    if (current < target)
    {
        if ((target - current) <= delta)
        {
            return target;
        }
        return current + delta;
    }

    if (current > target)
    {
        if ((current - target) <= delta)
        {
            return target;
        }
        return current - delta;
    }

    return current;
}

/**
 * @brief Publish one foreground rate to its interrupt-owned pulse state.
 * @param axis Axis state.
 */
static void stepper_test_publish_rate(stepper_test_axis_struct *axis)
{
    uint32 primask;
    uint32 phase_increment;

    phase_increment = stepper_test_rate_to_phase_increment(
        axis->current_rate_milli_steps_s);

    primask = interrupt_global_disable();
    axis->command_rate_milli_steps_s =
        axis->current_rate_milli_steps_s;
    axis->phase_increment = phase_increment;
    interrupt_global_enable(primask);
}

/**
 * @brief Clear one axis command while interrupts are already excluded.
 * @param axis Axis state.
 */
static void stepper_test_clear_axis_command(
    stepper_test_axis_struct *axis)
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
 * @brief Atomically request both axes to stop at the next pulse tick.
 */
static void stepper_test_stop_all(void)
{
    uint32 primask = interrupt_global_disable();

    stepper_test_clear_axis_command(
        &stepper_test_axes[STEPPER_TEST_AXIS_PAN]);
    stepper_test_clear_axis_command(
        &stepper_test_axes[STEPPER_TEST_AXIS_TILT]);

    interrupt_global_enable(primask);
}

/**
 * @brief Stop both outputs from the 5 kHz callback and latch the stop.
 */
static void stepper_test_emergency_stop_tick(void)
{
    stepper_test_axis_struct *pan =
        &stepper_test_axes[STEPPER_TEST_AXIS_PAN];
    stepper_test_axis_struct *tilt =
        &stepper_test_axes[STEPPER_TEST_AXIS_TILT];

    stepper_test_clear_axis_command(pan);
    stepper_test_clear_axis_command(tilt);
    pan->pulse_high = 0U;
    tilt->pulse_high = 0U;
    gpio_low(pan->step_pin);
    gpio_low(tilt->step_pin);
    stepper_test_stop_latched = 1U;
}

/**
 * @brief Reset the selected axis software position after manual alignment.
 */
static void stepper_test_zero_selected_axis(void)
{
    stepper_test_axis_struct *axis =
        &stepper_test_axes[stepper_test_selected_axis];
    uint32 primask;

    primask = interrupt_global_disable();
    stepper_test_clear_axis_command(axis);
    axis->position_steps = 0;
    axis->zero_valid = 1U;
    interrupt_global_enable(primask);

    printf(
        "Stepper %s zero captured.\r\n",
        stepper_test_selected_axis == STEPPER_TEST_AXIS_PAN
            ? "PAN" : "TILT");
}

/**
 * @brief Update one active-low key debounce state.
 * @param pin Key GPIO pin.
 * @param key Key state to update.
 */
static void stepper_test_update_key(
    gpio_pin_enum pin,
    stepper_test_key_struct *key)
{
    uint8 raw_pressed = (uint8)(gpio_get_level(pin) == GPIO_LOW);

    if (raw_pressed == key->pressed)
    {
        key->mismatch_ms = 0U;
        return;
    }

    if (key->mismatch_ms < STEPPER_TEST_DEBOUNCE_MS)
    {
        key->mismatch_ms++;
    }
    if (key->mismatch_ms >= STEPPER_TEST_DEBOUNCE_MS)
    {
        key->pressed = raw_pressed;
        key->mismatch_ms = 0U;
    }
}

/**
 * @brief Handle short-select and long-zero actions on A30.
 */
static void stepper_test_update_select_key(void)
{
    uint8 jog_active = (uint8)(stepper_test_negative_key.pressed
        || stepper_test_positive_key.pressed);
    uint8 axes_stopped = (uint8)(
        (stepper_test_axes[STEPPER_TEST_AXIS_PAN]
            .current_rate_milli_steps_s == 0)
        && (stepper_test_axes[STEPPER_TEST_AXIS_TILT]
            .current_rate_milli_steps_s == 0));

    if (stepper_test_select_key.pressed != 0U)
    {
        if (stepper_test_select_hold_ms < STEPPER_TEST_ZERO_HOLD_MS)
        {
            stepper_test_select_hold_ms++;
        }
        if ((stepper_test_select_hold_ms
                >= STEPPER_TEST_ZERO_HOLD_MS)
            && (stepper_test_select_long_handled == 0U)
            && (jog_active == 0U)
            && (axes_stopped != 0U))
        {
            stepper_test_zero_selected_axis();
            stepper_test_select_long_handled = 1U;
        }
    }
    else if (stepper_test_select_was_pressed != 0U)
    {
        if ((stepper_test_select_long_handled == 0U)
            && (jog_active == 0U)
            && (axes_stopped != 0U))
        {
            stepper_test_selected_axis =
                stepper_test_selected_axis == STEPPER_TEST_AXIS_PAN
                    ? STEPPER_TEST_AXIS_TILT
                    : STEPPER_TEST_AXIS_PAN;
            printf(
                "Stepper selected axis: %s.\r\n",
                stepper_test_selected_axis == STEPPER_TEST_AXIS_PAN
                    ? "PAN" : "TILT");
        }
        stepper_test_select_hold_ms = 0U;
        stepper_test_select_long_handled = 0U;
    }

    stepper_test_select_was_pressed =
        stepper_test_select_key.pressed;
}

/**
 * @brief Select signed jog targets from the current key state.
 */
static void stepper_test_update_jog_targets(void)
{
    stepper_test_axis_struct *selected =
        &stepper_test_axes[stepper_test_selected_axis];
    stepper_test_axis_struct *unselected =
        &stepper_test_axes[
            stepper_test_selected_axis == STEPPER_TEST_AXIS_PAN
                ? STEPPER_TEST_AXIS_TILT
                : STEPPER_TEST_AXIS_PAN];
    int32 jog_rate = selected->zero_valid != 0U
        ? STEPPER_TEST_JOG_RATE_MILLI_STEPS_S
        : STEPPER_TEST_CALIBRATE_RATE_MILLI_STEPS_S;

    unselected->target_rate_milli_steps_s = 0;
    if ((stepper_test_negative_key.pressed != 0U)
        && (stepper_test_positive_key.pressed == 0U))
    {
        selected->target_rate_milli_steps_s =
            -jog_rate;
    }
    else if ((stepper_test_positive_key.pressed != 0U)
        && (stepper_test_negative_key.pressed == 0U))
    {
        selected->target_rate_milli_steps_s =
            jog_rate;
    }
    else
    {
        selected->target_rate_milli_steps_s = 0;
    }
}

/**
 * @brief Apply acceleration limits and publish both step rates.
 */
static void stepper_test_update_rates(void)
{
    uint8 index;

    for (index = 0U; index < STEPPER_TEST_AXIS_COUNT; index++)
    {
        stepper_test_axis_struct *axis = &stepper_test_axes[index];

        axis->current_rate_milli_steps_s = stepper_test_ramp_rate(
            axis->current_rate_milli_steps_s,
            axis->target_rate_milli_steps_s);
        stepper_test_publish_rate(axis);
    }
}

/**
 * @brief Execute one 5 kHz pulse-engine tick for one axis.
 * @param axis Axis state.
 */
static void stepper_test_axis_tick(stepper_test_axis_struct *axis)
{
    int32 command_rate = axis->command_rate_milli_steps_s;
    uint8 positive;
    uint8 direction_level;
    uint32 previous_phase;
    uint8 phase_overflow;
    int32 min_position;
    int32 max_position;

    if ((command_rate == 0) || (axis->phase_increment == 0U))
    {
        gpio_low(axis->step_pin);
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        return;
    }

    positive = (uint8)(command_rate > 0);
    if (positive != axis->direction_positive)
    {
        axis->direction_positive = positive;
        direction_level = positive != 0U
            ? axis->positive_dir_level
            : stepper_test_invert_level(axis->positive_dir_level);
        gpio_set_level(axis->dir_pin, direction_level);
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        gpio_low(axis->step_pin);
        axis->direction_settle_ticks =
            STEPPER_TEST_DIRECTION_SETTLE_TICKS;
        return;
    }

    if (axis->direction_settle_ticks > 0U)
    {
        axis->direction_settle_ticks--;
        return;
    }

    if (axis->zero_valid != 0U)
    {
        min_position = axis->min_position_steps;
        max_position = axis->max_position_steps;
    }
    else
    {
        min_position = -STEPPER_TEST_CALIBRATE_TRAVEL_STEPS;
        max_position = STEPPER_TEST_CALIBRATE_TRAVEL_STEPS;
    }

    if (((positive != 0U)
            && (axis->position_steps >= max_position))
        || ((positive == 0U)
            && (axis->position_steps <= min_position)))
    {
        axis->phase_accumulator = 0U;
        axis->pulse_pending = 0U;
        axis->pulse_high = 0U;
        gpio_low(axis->step_pin);
        return;
    }

    previous_phase = axis->phase_accumulator;
    axis->phase_accumulator += axis->phase_increment;
    phase_overflow = (uint8)(
        axis->phase_accumulator < previous_phase);

    if (axis->pulse_high != 0U)
    {
        gpio_low(axis->step_pin);
        axis->pulse_high = 0U;
        if (phase_overflow != 0U)
        {
            axis->pulse_pending = 1U;
        }
        return;
    }

    if ((axis->pulse_pending != 0U) || (phase_overflow != 0U))
    {
        axis->pulse_pending = 0U;
        gpio_high(axis->step_pin);
        axis->pulse_high = 1U;
        axis->position_steps += positive != 0U ? 1 : -1;
    }
}

/**
 * @brief Generate independent pan and tilt STEP pulses from TIMG6.
 * @param event Unused PIT event value.
 * @param context Unused callback context.
 */
static void stepper_test_pit_callback(uint32 event, void *context)
{
    (void)event;
    (void)context;

    if ((gpio_get_level(STEPPER_TEST_STOP_KEY_PIN) == GPIO_LOW)
        || (stepper_test_stop_latched != 0U))
    {
        stepper_test_emergency_stop_tick();
        return;
    }

    stepper_test_axis_tick(
        &stepper_test_axes[STEPPER_TEST_AXIS_PAN]);
    stepper_test_axis_tick(
        &stepper_test_axes[STEPPER_TEST_AXIS_TILT]);
}

/**
 * @brief Initialize GPIO, runtime state and the unique pulse timer.
 */
static void stepper_test_init(void)
{
    stepper_test_axis_struct *pan =
        &stepper_test_axes[STEPPER_TEST_AXIS_PAN];
    stepper_test_axis_struct *tilt =
        &stepper_test_axes[STEPPER_TEST_AXIS_TILT];

    pan->step_pin = STEPPER_TEST_PAN_STEP_PIN;
    pan->dir_pin = STEPPER_TEST_PAN_DIR_PIN;
    pan->min_position_steps = STEPPER_TEST_PAN_MIN_STEPS;
    pan->max_position_steps = STEPPER_TEST_PAN_MAX_STEPS;
    pan->positive_dir_level = STEPPER_TEST_PAN_POSITIVE_DIR_LEVEL;

    tilt->step_pin = STEPPER_TEST_TILT_STEP_PIN;
    tilt->dir_pin = STEPPER_TEST_TILT_DIR_PIN;
    tilt->min_position_steps = STEPPER_TEST_TILT_MIN_STEPS;
    tilt->max_position_steps = STEPPER_TEST_TILT_MAX_STEPS;
    tilt->positive_dir_level = STEPPER_TEST_TILT_POSITIVE_DIR_LEVEL;

    gpio_init(pan->step_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(pan->dir_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(tilt->step_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(tilt->dir_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);

    gpio_init(
        STEPPER_TEST_SELECT_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        STEPPER_TEST_STOP_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        STEPPER_TEST_NEGATIVE_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);
    gpio_init(
        STEPPER_TEST_POSITIVE_KEY_PIN,
        GPI,
        GPIO_HIGH,
        GPI_PULL_UP);

    stepper_test_selected_axis = STEPPER_TEST_AXIS_PAN;
    interrupt_set_priority(
        STEPPER_TEST_PIT_IRQ,
        STEPPER_TEST_PIT_IRQ_PRIORITY);
    pit_us_init(
        STEPPER_TEST_PIT,
        STEPPER_TEST_TICK_US,
        stepper_test_pit_callback,
        NULL);

    ml_stepper_debug_init();
}

/**
 * @brief Print one low-rate diagnostic snapshot over debug UART0.
 */
static void stepper_test_print_status(void)
{
    const stepper_test_axis_struct *pan =
        &stepper_test_axes[STEPPER_TEST_AXIS_PAN];
    const stepper_test_axis_struct *tilt =
        &stepper_test_axes[STEPPER_TEST_AXIS_TILT];

    printf(
        "Axis=%s Pan=%ld/%ldpps/Z%u "
        "Tilt=%ld/%ldpps/Z%u Stop=%u\r\n",
        stepper_test_selected_axis == STEPPER_TEST_AXIS_PAN
            ? "PAN" : "TILT",
        (long)pan->position_steps,
        (long)(pan->current_rate_milli_steps_s
            / (int32)STEPPER_TEST_RATE_SCALE),
        (unsigned int)pan->zero_valid,
        (long)tilt->position_steps,
        (long)(tilt->current_rate_milli_steps_s
            / (int32)STEPPER_TEST_RATE_SCALE),
        (unsigned int)tilt->zero_valid,
        (unsigned int)stepper_test_stop_latched);
}

/**
 * @brief Run the dual-axis manual STEP/DIR bench test.
 */
void test_stepper_run(void)
{
    uint16 control_elapsed_ms = 0U;
    uint16 status_elapsed_ms = 0U;

    stepper_test_init();
    printf("Dual-axis STEP/DIR test ready.\r\n");
    printf("A30 short: select; A30 long: zero selected axis.\r\n");
    printf("B0/B1: negative/positive jog; A31: immediate stop.\r\n");
    printf("Before zero: 5 deg/s and +/-320-step travel envelope.\r\n");

    while (true)
    {
        stepper_test_update_key(
            STEPPER_TEST_SELECT_KEY_PIN,
            &stepper_test_select_key);
        stepper_test_update_key(
            STEPPER_TEST_NEGATIVE_KEY_PIN,
            &stepper_test_negative_key);
        stepper_test_update_key(
            STEPPER_TEST_POSITIVE_KEY_PIN,
            &stepper_test_positive_key);

        if ((stepper_test_stop_latched != 0U)
            && (stepper_test_stop_reported == 0U))
        {
            stepper_test_stop_reported = 1U;
            printf("Stepper emergency stop.\r\n");
        }
        if ((gpio_get_level(STEPPER_TEST_STOP_KEY_PIN) == GPIO_HIGH)
            && (stepper_test_stop_latched != 0U)
            && (stepper_test_negative_key.pressed == 0U)
            && (stepper_test_positive_key.pressed == 0U))
        {
            uint32 primask = interrupt_global_disable();

            stepper_test_stop_latched = 0U;
            interrupt_global_enable(primask);
            stepper_test_stop_reported = 0U;
            printf("Stepper stop released.\r\n");
        }

        stepper_test_update_select_key();
        if (stepper_test_stop_latched == 0U)
        {
            stepper_test_update_jog_targets();
        }
        else
        {
            stepper_test_stop_all();
        }

        control_elapsed_ms += STEPPER_TEST_LOOP_PERIOD_MS;
        if (control_elapsed_ms >= STEPPER_TEST_CONTROL_PERIOD_MS)
        {
            stepper_test_update_rates();
            control_elapsed_ms = 0U;
        }

        status_elapsed_ms += STEPPER_TEST_LOOP_PERIOD_MS;
        if (status_elapsed_ms >= STEPPER_TEST_STATUS_PERIOD_MS)
        {
            stepper_test_print_status();

            ml_stepper_debug_update(
                stepper_test_selected_axis,
                stepper_test_axes[0].position_steps,
                stepper_test_axes[0].current_rate_milli_steps_s
                    / (int32)STEPPER_TEST_RATE_SCALE,
                stepper_test_axes[0].zero_valid,
                stepper_test_axes[1].position_steps,
                stepper_test_axes[1].current_rate_milli_steps_s
                    / (int32)STEPPER_TEST_RATE_SCALE,
                stepper_test_axes[1].zero_valid,
                stepper_test_stop_latched,
                stepper_test_negative_key.pressed,
                stepper_test_positive_key.pressed,
                stepper_test_select_key.pressed);

            status_elapsed_ms = 0U;
        }

        system_delay_ms(STEPPER_TEST_LOOP_PERIOD_MS);
    }
}

#endif
