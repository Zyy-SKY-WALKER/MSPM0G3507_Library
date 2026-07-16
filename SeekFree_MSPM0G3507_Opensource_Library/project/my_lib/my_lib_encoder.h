/**
 * @file    my_lib_encoder.h
 * @brief   Hybrid hardware/software x2 quadrature encoder driver.
 */

#ifndef MY_LIB_ENCODER_H
#define MY_LIB_ENCODER_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/** @brief Left encoder phase-A QEI input pin. */
#define MY_ENCODER_LEFT_PHASE_A_PIN        (B21)
/** @brief Left encoder phase-B QEI input pin. */
#define MY_ENCODER_LEFT_PHASE_B_PIN        (B22)
/** @brief Left count polarity; use 1 or -1 to select positive rotation. */
#define MY_ENCODER_LEFT_COUNT_SIGN         (1)

/** @brief Right encoder phase-A interrupt input pin. */
#define MY_ENCODER_RIGHT_PHASE_A_PIN       (B17)
/** @brief Right encoder phase-B sampled input pin. */
#define MY_ENCODER_RIGHT_PHASE_B_PIN       (B18)
/**
 * @brief Select equal or unequal phases as the positive direction.
 * @note Use 1U for equal phases or 0U for unequal phases.
 */
#define MY_ENCODER_RIGHT_POSITIVE_AB_EQUAL (1U)

/**
 * @brief Initialize both encoders for x2 quadrature counting.
 * @note Call once before using any other encoder API. Pending counts are reset.
 */
void my_encoder_init(void);

/**
 * @brief Read and consume both encoder counts accumulated since the last read.
 * @param left_count Destination for the signed left x2 interval count.
 * @param right_count Destination for the signed right x2 interval count.
 * @note One x2 count represents one of two counted edges per quadrature cycle.
 *       Positive direction is selected by the polarity macros above. Results
 *       are clamped to -32768 through 32767.
 * @note This is a destructive read: the left sampling baseline advances and
 *       the right accumulator is cleared. NULL destinations leave both intact.
 * @pre my_encoder_init() has completed. Sample often enough that the left x4
 *      hardware delta remains in the signed 16-bit range between reads.
 */
void my_encoder_get_delta(int16 *left_count, int16 *right_count);

/**
 * @brief Read the left encoder phase-A logic level.
 * @return GPIO_HIGH or GPIO_LOW.
 * @pre my_encoder_init() has completed.
 */
uint8 my_encoder_get_left_phase_a(void);
/**
 * @brief Read the left encoder phase-B logic level.
 * @return GPIO_HIGH or GPIO_LOW.
 * @pre my_encoder_init() has completed.
 */
uint8 my_encoder_get_left_phase_b(void);
/**
 * @brief Read the right encoder phase-A logic level.
 * @return GPIO_HIGH or GPIO_LOW.
 * @pre my_encoder_init() has completed.
 */
uint8 my_encoder_get_right_phase_a(void);
/**
 * @brief Read the right encoder phase-B logic level.
 * @return GPIO_HIGH or GPIO_LOW.
 * @pre my_encoder_init() has completed.
 */
uint8 my_encoder_get_right_phase_b(void);

/**
 * @brief Discard pending left counts and the x4-to-x2 scaling remainder.
 * @pre my_encoder_init() has completed.
 */
void my_encoder_clear_left_count(void);
/**
 * @brief Discard all pending right x2 counts.
 * @pre my_encoder_init() has completed.
 */
void my_encoder_clear_right_count(void);
/**
 * @brief Discard pending counts for both encoders near-synchronously.
 * @pre my_encoder_init() has completed.
 */
void my_encoder_clear_count(void);

#endif
