
/**
 * @file    ml_stepper_debug.c
 * @brief   TFT debug display for stepper motor dual-axis jog test.
 * @note    The layout uses the 8x16 font in 240x320 portrait orientation.
 *          Dynamic fields are redrawn from scratch every 500 ms cycle.
 */

#include "ml_stepper_debug.h"

#include "gimbal_stepper.h"
#include "my_lib_ili9341.h"

/* Static label X positions (8x16 font, 8 px per glyph) -----*/
#define ML_SD_LABEL_X        (8U)
#define ML_SD_VALUE_X        (72U)
#define ML_SD_UNIT_X         (184U)

/* Row Y positions ------------------------------------------*/
#define ML_SD_ROW_HEADER     (0U)

#define ML_SD_ROW_PAN_TITLE  (32U)
#define ML_SD_ROW_PAN_POS    (48U)
#define ML_SD_ROW_PAN_SPD    (64U)
#define ML_SD_ROW_PAN_ZERO   (80U)

#define ML_SD_ROW_TILT_TITLE (128U)
#define ML_SD_ROW_TILT_POS   (144U)
#define ML_SD_ROW_TILT_SPD   (160U)
#define ML_SD_ROW_TILT_ZERO  (176U)

#define ML_SD_ROW_STOP       (216U)
#define ML_SD_ROW_KEYS       (232U)

/* Progress-bar geometry ------------------------------------*/
#define ML_SD_BAR_X          (48U)
#define ML_SD_BAR_W          (160U)
#define ML_SD_BAR_H          (8U)

#define ML_SD_BAR_PAN_Y      (96U)
#define ML_SD_BAR_TILT_Y     (192U)

/* Colour used for the empty portion of the bar ------------*/
#define ML_SD_BAR_EMPTY      (0x2104U)

/**
 * @brief Draw a horizontal progress bar at a given fill percentage.
 * @param x       Left edge in pixels.
 * @param y       Top edge in pixels.
 * @param w       Total bar width in pixels.
 * @param fill_pct Fill percentage from 0 through 100.
 */
static void ml_sd_draw_bar(uint16 x, uint16 y, uint16 w, uint8 fill_pct)
{
    uint16 fill_w;
    uint16 colour;

    fill_w = (w * (uint16)fill_pct) / 100U;
    colour = (fill_pct > 80U) ? ILI9341_COLOR_RED
           : (fill_pct > 60U) ? ILI9341_COLOR_YELLOW
                              : ILI9341_COLOR_GREEN;

    ili9341_fill_rect(x, y, (uint16)(x + w - 1U), (uint16)(y + ML_SD_BAR_H - 1U),
                      ML_SD_BAR_EMPTY);
    if (fill_w > 0U)
    {
        ili9341_fill_rect(x, y, (uint16)(x + fill_w - 1U),
                          (uint16)(y + ML_SD_BAR_H - 1U), colour);
    }
}

/**
 * @brief Convert one bounded axis position to a bar percentage.
 */
static uint8 ml_sd_position_percent(
    int32 position,
    int32 minimum,
    int32 maximum)
{
    uint32 range;
    uint32 offset;

    if(position <= minimum)
    {
        return 0U;
    }
    if(position >= maximum)
    {
        return 100U;
    }

    range = (uint32)(maximum - minimum);
    offset = (uint32)(position - minimum);
    return (uint8)((offset * 100U) / range);
}

/**
 * @brief Initialise the ILI9341 display and set the default colour scheme.
 */
void ml_stepper_debug_init(void)
{
    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_set_font(ILI9341_FONT_8X16);
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_clear();
}

/**
 * @brief Redraw the complete debug dashboard with current values.
 */
void ml_stepper_debug_update(
    uint8  selected_axis,
    int32  pan_position,  int32  pan_speed,  uint8 pan_zero,
    int32  tilt_position, int32  tilt_speed, uint8 tilt_zero,
    uint8  stop_latched,
    uint8  key_negative,  uint8  key_positive, uint8 key_select)
{
    uint16 colour;
    uint8  pct;

    ili9341_clear();

    /* Header -------------------------------------------------*/
    ili9341_set_color(ILI9341_COLOR_CYAN, ILI9341_COLOR_BLACK);
    ili9341_show_string(40U, ML_SD_ROW_HEADER, "== STEPPER ==");

    /* PAN section --------------------------------------------*/
    colour = (selected_axis == ML_STEPPER_DEBUG_AXIS_PAN)
                 ? ILI9341_COLOR_GREEN
                 : ILI9341_COLOR_WHITE;
    ili9341_set_color(colour, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_PAN_TITLE, ">> PAN <<");

    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_PAN_POS, "Pos:");
    ili9341_show_int(ML_SD_VALUE_X, ML_SD_ROW_PAN_POS, pan_position, 6);
    ili9341_show_string(ML_SD_UNIT_X, ML_SD_ROW_PAN_POS, "st");

    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_PAN_SPD, "Spd:");
    ili9341_show_int(ML_SD_VALUE_X, ML_SD_ROW_PAN_SPD, pan_speed, 6);
    ili9341_show_string(ML_SD_UNIT_X, ML_SD_ROW_PAN_SPD, "pps");

    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_PAN_ZERO, "Zero:");
    colour = (pan_zero != 0U) ? ILI9341_COLOR_GREEN : ILI9341_COLOR_RED;
    ili9341_set_color(colour, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_VALUE_X, ML_SD_ROW_PAN_ZERO,
                        (pan_zero != 0U) ? "YES" : "NO");

    pct = ml_sd_position_percent(
        pan_position,
        GIMBAL_STEPPER_YAW_MIN_STEPS,
        GIMBAL_STEPPER_YAW_MAX_STEPS);
    ml_sd_draw_bar(ML_SD_BAR_X, ML_SD_BAR_PAN_Y, ML_SD_BAR_W, pct);

    /* TILT section -------------------------------------------*/
    colour = (selected_axis == ML_STEPPER_DEBUG_AXIS_TILT)
                 ? ILI9341_COLOR_GREEN
                 : ILI9341_COLOR_WHITE;
    ili9341_set_color(colour, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_TILT_TITLE, ">>TILT<<");

    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_TILT_POS, "Pos:");
    ili9341_show_int(ML_SD_VALUE_X, ML_SD_ROW_TILT_POS, tilt_position, 6);
    ili9341_show_string(ML_SD_UNIT_X, ML_SD_ROW_TILT_POS, "st");

    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_TILT_SPD, "Spd:");
    ili9341_show_int(ML_SD_VALUE_X, ML_SD_ROW_TILT_SPD, tilt_speed, 6);
    ili9341_show_string(ML_SD_UNIT_X, ML_SD_ROW_TILT_SPD, "pps");

    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_TILT_ZERO, "Zero:");
    colour = (tilt_zero != 0U) ? ILI9341_COLOR_GREEN : ILI9341_COLOR_RED;
    ili9341_set_color(colour, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_VALUE_X, ML_SD_ROW_TILT_ZERO,
                        (tilt_zero != 0U) ? "YES" : "NO");

    if(tilt_zero == 0U)
    {
        pct = 0U;
    }
    else
    {
        pct = ml_sd_position_percent(
            tilt_position,
            GIMBAL_STEPPER_PITCH_MIN_STEPS,
            GIMBAL_STEPPER_PITCH_MAX_STEPS);
        if(GIMBAL_CONFIG_PITCH_POSITION_SIGN < 0)
        {
            pct = 100U - pct;
        }
    }
    ml_sd_draw_bar(ML_SD_BAR_X, ML_SD_BAR_TILT_Y, ML_SD_BAR_W, pct);

    /* Status section -----------------------------------------*/
    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_STOP, "Stop:");
    colour = (stop_latched != 0U) ? ILI9341_COLOR_RED : ILI9341_COLOR_GREEN;
    ili9341_set_color(colour, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_VALUE_X, ML_SD_ROW_STOP,
                        (stop_latched != 0U) ? "LATCHED" : "OK");

    ili9341_set_color(ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ili9341_show_string(ML_SD_LABEL_X, ML_SD_ROW_KEYS, "Keys:");
    ili9341_show_string(ML_SD_VALUE_X, ML_SD_ROW_KEYS,
                        (key_negative != 0U) ? "[-] " : " .  ");
    ili9341_show_string((uint16)(ML_SD_VALUE_X + 24U), ML_SD_ROW_KEYS,
                        (key_positive != 0U) ? "[+] " : " .  ");
    ili9341_show_string((uint16)(ML_SD_VALUE_X + 48U), ML_SD_ROW_KEYS,
                        (key_select != 0U) ? "[SEL]" : " .  ");
}
