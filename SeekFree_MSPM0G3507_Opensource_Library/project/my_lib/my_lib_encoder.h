/**
 * @file    my_lib_encoder.h
 * @brief   Dual single-edge quadrature encoder driver for MSPM0G3507.
 */

#ifndef MY_LIB_ENCODER_H
#define MY_LIB_ENCODER_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define MY_ENCODER_LEFT_PHASE_A_PIN        (B22)
#define MY_ENCODER_LEFT_PHASE_B_PIN        (B23)
#define MY_ENCODER_LEFT_POSITIVE_B_LEVEL   (GPIO_HIGH)

#define MY_ENCODER_RIGHT_PHASE_A_PIN       (B26)
#define MY_ENCODER_RIGHT_PHASE_B_PIN       (B21)
#define MY_ENCODER_RIGHT_POSITIVE_B_LEVEL  (GPIO_HIGH)

void my_encoder_init(void);

void my_encoder_get_delta(int16 *left_count, int16 *right_count);
uint8 my_encoder_get_left_phase_a(void);
uint8 my_encoder_get_left_phase_b(void);
uint8 my_encoder_get_right_phase_a(void);
uint8 my_encoder_get_right_phase_b(void);

void my_encoder_clear_left_count(void);
void my_encoder_clear_right_count(void);
void my_encoder_clear_count(void);

#endif
