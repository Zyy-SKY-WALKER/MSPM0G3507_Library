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

#define GRAY_SENSOR_D1_PIN              (B17)
#define GRAY_SENSOR_D2_PIN              (B19)
#define GRAY_SENSOR_D3_PIN              (A29)
#define GRAY_SENSOR_D4_PIN              (A28)
#define GRAY_SENSOR_D5_PIN              (B27)
#define GRAY_SENSOR_D6_PIN              (A16)
#define GRAY_SENSOR_D7_PIN              (A17)
#define GRAY_SENSOR_D8_PIN              (B20)

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
