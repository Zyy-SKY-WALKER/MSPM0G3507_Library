/**
 * @file        ml_oled.h
 * @author      xxs13
 * @version     V1.0
 * @date        2026-07-11
 * @brief       SSD1306 OLED software I2C driver interface
 * @attention   This driver is intended for the MSPM0G3507 platform.
 */

#ifndef ML_OLED_H
#define ML_OLED_H

#include <stdbool.h>
#include <stdint.h>

#include "zf_driver_gpio.h"

#define ML_OLED_I2C_ADDRESS          (0x3CU)
#define ML_OLED_I2C_DELAY            (100U)
#define ML_OLED_SCL_PIN              (B8)
#define ML_OLED_SDA_PIN              (B9)

#define ML_OLED_WIDTH                (128U)
#define ML_OLED_HEIGHT               (64U)
#define ML_OLED_PAGE_COUNT           (8U)
#define ML_OLED_TEXT_LINE_COUNT      (4U)
#define ML_OLED_TEXT_COLUMN_COUNT    (16U)

bool ml_oled_init(void);
bool ml_oled_clear(void);
bool ml_oled_set_cursor(uint8_t page, uint8_t x);
bool ml_oled_show_char(uint8_t line, uint8_t column, char character);
bool ml_oled_show_string(uint8_t line, uint8_t column,
                         const char *string);
bool ml_oled_show_uint(uint8_t line, uint8_t column, uint32_t number,
                       uint8_t length);
bool ml_oled_show_int(uint8_t line, uint8_t column, int32_t number,
                      uint8_t length);
bool ml_oled_show_hex(uint8_t line, uint8_t column, uint32_t number,
                      uint8_t length);
bool ml_oled_show_binary(uint8_t line, uint8_t column, uint32_t number,
                         uint8_t length);
bool ml_oled_show_float(uint8_t line, uint8_t column, float number,
                        uint8_t integer_length,
                        uint8_t fractional_length);
bool ml_oled_draw_bitmap(uint8_t x_start, uint8_t page_start,
                         uint8_t x_end, uint8_t page_end,
                         const uint8_t *bitmap);

#endif
