/**
 * @file    ml_stepper_debug.h
 * @brief   TFT debug display for stepper motor dual-axis jog test.
 */

#ifndef ML_STEPPER_DEBUG_H
#define ML_STEPPER_DEBUG_H

#include "zf_common_typedef.h"

#define ML_STEPPER_DEBUG_AXIS_PAN              (0U)
#define ML_STEPPER_DEBUG_AXIS_TILT             (1U)

/**
 * @brief Initialize ILI9341 and clear the display.
 * @note Call once before entering the test loop.
 */
void ml_stepper_debug_init(void);

/**
 * @brief Refresh all dynamic data on the TFT.
 * @param selected_axis ML_STEPPER_DEBUG_AXIS_PAN or ML_STEPPER_DEBUG_AXIS_TILT.
 * @param pan_position  PAN position in steps.
 * @param pan_speed     PAN speed in full steps per second.
 * @param pan_zero      PAN zero-valid flag.
 * @param tilt_position TILT position in steps.
 * @param tilt_speed    TILT speed in full steps per second.
 * @param tilt_zero     TILT zero-valid flag.
 * @param stop_latched  Non-zero when the emergency stop is latched.
 * @param key_negative  Non-zero when the negative jog key is pressed.
 * @param key_positive  Non-zero when the positive jog key is pressed.
 * @param key_select    Non-zero when the select key is pressed.
 */
void ml_stepper_debug_update(
    uint8  selected_axis,
    int32  pan_position,  int32  pan_speed,  uint8 pan_zero,
    int32  tilt_position, int32  tilt_speed, uint8 tilt_zero,
    uint8  stop_latched,
    uint8  key_negative,  uint8  key_positive, uint8 key_select);

#endif
