/**
 * @file    gray_sensor.h
 * @brief   Eight-channel digital grayscale sensor acquisition.
 */

#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define GRAY_SENSOR_CHANNEL_COUNT       (8U)
#define GRAY_SENSOR_ALL_ACTIVE_MASK     (0xFFU)

#define GRAY_SENSOR_PINS_CONFIGURED     (1U)

#define GRAY_SENSOR_D1_PIN              (B26)
#define GRAY_SENSOR_D2_PIN              (B23)
#define GRAY_SENSOR_D3_PIN              (A29)
#define GRAY_SENSOR_D4_PIN              (A28)
#define GRAY_SENSOR_D5_PIN              (B27)
#define GRAY_SENSOR_D6_PIN              (A16)
#define GRAY_SENSOR_D7_PIN              (A17)
#define GRAY_SENSOR_D8_PIN              (B20)

#define GRAY_SENSOR_ACTIVE_LEVEL        (GPIO_HIGH)
#define GRAY_SENSOR_INPUT_MODE          (GPI_PULL_UP)

/** @brief Classification of one normalized sensor sample. */
typedef enum
{
    /** No channel is active and the line is not detected. */
    GRAY_SENSOR_STATUS_LOST = 0,
    /** One through seven channels are active. */
    GRAY_SENSOR_STATUS_VALID,
    /** All eight channels are active. */
    GRAY_SENSOR_STATUS_ALL_ACTIVE,
} gray_sensor_status_enum;

/** @brief Raw and calculated values for one eight-channel sample. */
typedef struct
{
    /** Electrical-high mask with D1 in bit 0 and D8 in bit 7. */
    uint8 raw_mask;
    /** Polarity-normalized active mask with D1 in bit 0. */
    uint8 active_mask;
    /** Number of set bits in active_mask, from 0 through 8. */
    uint8 active_count;
    /** Sum of zero-based active channel indexes, from 0 through 28. */
    uint8 weighted_sum;
    /** Mean active channel index from 0.0 through 7.0. */
    float position;
    /** Position minus the 3.5 center index, from -3.5 through 3.5. */
    float deviation;
    /** Classification derived from active_count. */
    gray_sensor_status_enum status;
} gray_sensor_result_struct;

/**
 * @brief Configure all eight grayscale sensor GPIO inputs.
 * @return ZF_TRUE when the configured pins and polarity are valid.
 * @note This function clears the module's initialized state on failure.
 */
uint8 gray_sensor_init(void);

/**
 * @brief Sample the GPIO inputs and calculate the current line position.
 * @param result Destination for the raw and normalized sample data.
 * @return ZF_TRUE when initialized and result is non-NULL.
 * @note Call gray_sensor_init() successfully before sampling.
 */
uint8 gray_sensor_sample(gray_sensor_result_struct *result);

/**
 * @brief Calculate line metrics from a polarity-normalized active mask.
 * @param active_mask D1 in bit 0 and D8 in bit 7; set means active.
 * @param result Destination result; NULL causes no operation.
 * @note raw_mask is set equal to active_mask in the calculated result.
 */
void gray_sensor_calculate(
    uint8 active_mask,
    gray_sensor_result_struct *result);

#endif
