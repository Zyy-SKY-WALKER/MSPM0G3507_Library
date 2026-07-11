/**
 * @file    my_lib_encoder.h
 * @brief   Dual single-edge motor encoder driver for MSPM0G3507.
 */

#ifndef MY_LIB_ENCODER_H
#define MY_LIB_ENCODER_H

#include "zf_common_typedef.h"

#define MY_ENCODER_LEFT_PULSE_PIN          (B22)
#define MY_ENCODER_LEFT_DIRECTION_PIN      (B23)

#define MY_ENCODER_RIGHT_PULSE_PIN         (B26)
#define MY_ENCODER_RIGHT_DIRECTION_PIN     (B21)

#define MY_ENCODER_COUNTS_PER_REVOLUTION   (10250U)

void my_encoder_init(void);

int16 my_encoder_get_left_count(void);
int16 my_encoder_get_right_count(void);
uint8 my_encoder_get_left_direction(void);
uint8 my_encoder_get_right_direction(void);

void my_encoder_clear_left_count(void);
void my_encoder_clear_right_count(void);
void my_encoder_clear_count(void);

#endif
