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

#define GRAY_SENSOR_PINS_CONFIGURED     (0U)

#define GRAY_SENSOR_D1_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D2_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D3_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D4_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D5_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D6_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D7_PIN              (GPIO_MAX)
#define GRAY_SENSOR_D8_PIN              (GPIO_MAX)

#define GRAY_SENSOR_ACTIVE_LEVEL        (GPIO_HIGH)
#define GRAY_SENSOR_INPUT_MODE          (GPI_PULL_UP)

typedef enum
{
    GRAY_SENSOR_STATUS_LOST = 0,
    GRAY_SENSOR_STATUS_VALID,
    GRAY_SENSOR_STATUS_ALL_ACTIVE,
} gray_sensor_status_enum;

typedef struct
{
    uint8 raw_mask;
    uint8 active_mask;
    uint8 active_count;
    uint8 weighted_sum;
    float position;
    float deviation;
    gray_sensor_status_enum status;
} gray_sensor_result_struct;

uint8 gray_sensor_init(void);
uint8 gray_sensor_sample(gray_sensor_result_struct *result);
void gray_sensor_calculate(
    uint8 active_mask,
    gray_sensor_result_struct *result);

#endif
