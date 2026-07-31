/**
 * @file    gray_sensor.h
 * @brief   Eight-channel analog grayscale sensor acquisition.
 */

#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "zf_common_typedef.h"
#include "zf_driver_adc.h"

#define GRAY_SENSOR_CHANNEL_COUNT       (8U)
#define GRAY_SENSOR_ALL_ACTIVE_MASK     (0xFFU)

#define GRAY_SENSOR_ADC_PINS_CONFIGURED (1U)

#define GRAY_SENSOR_ADC_REFERENCE_MV    (3300U)
#define GRAY_SENSOR_ADC_COMPARE_MV      (2000U)
#define GRAY_SENSOR_ADC_MIN_VALUE       (0U)
#define GRAY_SENSOR_ADC_MAX_VALUE       (4095U)
#define GRAY_SENSOR_ADC_MIN_DIFF        (100U)
#define GRAY_SENSOR_ADC_COMPARE_RAW     \
    ((GRAY_SENSOR_ADC_COMPARE_MV * GRAY_SENSOR_ADC_MAX_VALUE \
        + (GRAY_SENSOR_ADC_REFERENCE_MV / 2U)) \
        / GRAY_SENSOR_ADC_REFERENCE_MV)

#define GRAY_SENSOR_D1_PIN              (A25)
#define GRAY_SENSOR_D2_PIN              (A24)
#define GRAY_SENSOR_D3_PIN              (A15)
#define GRAY_SENSOR_D4_PIN              (B19)
#define GRAY_SENSOR_D5_PIN              (A26)
#define GRAY_SENSOR_D6_PIN              (A16)
#define GRAY_SENSOR_D7_PIN              (A17)
#define GRAY_SENSOR_D8_PIN              (B20)

#define GRAY_SENSOR_D1_ADC_PIN          (ADC0_CH2_A25)
#define GRAY_SENSOR_D2_ADC_PIN          (ADC0_CH3_A24)
#define GRAY_SENSOR_D3_ADC_PIN          (ADC1_CH0_A15)
#define GRAY_SENSOR_D4_ADC_PIN          (ADC1_CH6_B19)
#define GRAY_SENSOR_D5_ADC_PIN          (ADC0_CH1_A26)
#define GRAY_SENSOR_D6_ADC_PIN          (ADC1_CH1_A16)
#define GRAY_SENSOR_D7_ADC_PIN          (ADC1_CH2_A17)
#define GRAY_SENSOR_D8_ADC_PIN          (ADC0_CH6_B20)

/** @brief Origin of a grayscale result passed to the line tracker. */
typedef enum
{
    /** Result was calculated from a binary active mask. */
    GRAY_SENSOR_RESULT_MODE_DIGITAL = 0,
    /** Result was calculated from continuous ADC weights. */
    GRAY_SENSOR_RESULT_MODE_ANALOG,
} gray_sensor_result_mode_enum;

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
    /** Comparator mask with D1 in bit 0 and D8 in bit 7. */
    uint8 raw_mask;
    /** Software-comparator active mask with D1 in bit 0. */
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
    /** Whether the result came from binary or analog processing. */
    gray_sensor_result_mode_enum calculation_mode;
    /** Latest ADC sample for each channel, valid for analog results. */
    uint16 analog_raw[GRAY_SENSOR_CHANNEL_COUNT];
    /** Sum of all continuous analog channel weights. */
    uint32 analog_weight_sum;
    /** Sum of analog weight multiplied by channel index. */
    uint32 analog_weighted_sum;
    /** Largest continuous analog channel weight. */
    uint16 analog_max_weight;
} gray_sensor_result_struct;

/**
 * @brief Configure all eight grayscale sensor ADC inputs.
 * @return ZF_TRUE when the configured ADC pins and analog parameters are valid.
 * @note This function clears the module's initialized state on failure.
 */
uint8 gray_sensor_init(void);

/**
 * @brief Sample ADC inputs and calculate the current analog line position.
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

/**
 * @brief Calculate line metrics from eight ADC samples.
 * @param adc_raw Eight 12-bit ADC samples in D1-D8 order.
 * @param result Destination result; NULL causes no operation.
 */
void gray_sensor_calculate_analog(
    const uint16 adc_raw[GRAY_SENSOR_CHANNEL_COUNT],
    gray_sensor_result_struct *result);

#endif
