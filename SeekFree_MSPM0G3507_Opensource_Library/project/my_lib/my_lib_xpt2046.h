/**
 * @file    my_lib_xpt2046.h
 * @brief   Independent XPT2046 resistive touch controller driver.
 */

#ifndef MY_LIB_XPT2046_H
#define MY_LIB_XPT2046_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

/** @brief SPI peripheral dedicated to the touch controller. */
#define XPT2046_SPI                       (SPI_1)
/** @brief XPT2046 serial clock frequency in hertz. */
#define XPT2046_SPI_SPEED                 (2U * 1000U * 1000U)
/** @brief XPT2046 SPI clock polarity and phase mode. */
#define XPT2046_SPI_MODE                  (SPI_MODE0)

#define XPT2046_SCK_PIN                   (SPI1_SCK_A17)
#define XPT2046_MOSI_PIN                  (SPI1_MOSI_B22)
#define XPT2046_MISO_PIN                  (SPI1_MISO_A16)
#define XPT2046_CS_PIN                    (B20)
#define XPT2046_IRQ_PIN                   (B23)

/** @brief Portrait touch coordinate width. */
#define XPT2046_SCREEN_WIDTH              (240U)
/** @brief Portrait touch coordinate height. */
#define XPT2046_SCREEN_HEIGHT             (320U)

/** @brief Reference raw value measured near the left calibration point. */
#define XPT2046_CAL_X1_RAW                (3875)
/** @brief Pixel coordinate paired with XPT2046_CAL_X1_RAW. */
#define XPT2046_CAL_X1_PIXEL              (235)
/** @brief Reference raw value measured near the right calibration point. */
#define XPT2046_CAL_X2_RAW                (215)
/** @brief Pixel coordinate paired with XPT2046_CAL_X2_RAW. */
#define XPT2046_CAL_X2_PIXEL              (5)
/** @brief Reference raw value measured near the top calibration point. */
#define XPT2046_CAL_Y1_RAW                (3865)
/** @brief Pixel coordinate paired with XPT2046_CAL_Y1_RAW. */
#define XPT2046_CAL_Y1_PIXEL              (5)
/** @brief Reference raw value measured near the bottom calibration point. */
#define XPT2046_CAL_Y2_RAW                (400)
/** @brief Pixel coordinate paired with XPT2046_CAL_Y2_RAW. */
#define XPT2046_CAL_Y2_PIXEL              (315)

/**
 * @brief Initialize the dedicated SPI1 bus and touch control GPIOs.
 * @note This function is independent from ili9341_init().
 */
void xpt2046_init(void);

/**
 * @brief Check the active-low PENIRQ input.
 * @return 1 when the panel is pressed, otherwise 0.
 */
uint8 xpt2046_is_pressed(void);

/**
 * @brief Read filtered 12-bit touch ADC values.
 * @param raw_x Destination for the filtered X ADC value.
 * @param raw_y Destination for the filtered Y ADC value.
 * @return 1 when a valid pressed sample is returned, otherwise 0.
 */
uint8 xpt2046_read_raw(uint16 *raw_x, uint16 *raw_y);

/**
 * @brief Convert raw ADC values to portrait screen coordinates.
 * @param raw_x Filtered X ADC value.
 * @param raw_y Filtered Y ADC value.
 * @param x Destination for the horizontal pixel coordinate.
 * @param y Destination for the vertical pixel coordinate.
 */
void xpt2046_convert_point(
    uint16 raw_x,
    uint16 raw_y,
    uint16 *x,
    uint16 *y);

/**
 * @brief Read a filtered and calibrated portrait touch point.
 * @param x Destination for the horizontal pixel coordinate.
 * @param y Destination for the vertical pixel coordinate.
 * @return 1 when a valid pressed point is returned, otherwise 0.
 */
uint8 xpt2046_read_point(uint16 *x, uint16 *y);

#endif
