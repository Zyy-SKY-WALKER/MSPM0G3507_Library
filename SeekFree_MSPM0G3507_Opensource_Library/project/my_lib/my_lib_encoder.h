/**
 * @file    my_lib_encoder.h
 * @brief   Hybrid hardware/software x2 quadrature encoder driver.
 */

#ifndef MY_LIB_ENCODER_H
#define MY_LIB_ENCODER_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define MY_ENCODER_LEFT_PHASE_A_PIN        (B21)
#define MY_ENCODER_LEFT_PHASE_B_PIN        (B22)
#define MY_ENCODER_LEFT_COUNT_SIGN         (1)

#define MY_ENCODER_RIGHT_PHASE_A_PIN       (B17)
#define MY_ENCODER_RIGHT_PHASE_B_PIN       (B18)
#define MY_ENCODER_RIGHT_POSITIVE_AB_EQUAL (1U)

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
