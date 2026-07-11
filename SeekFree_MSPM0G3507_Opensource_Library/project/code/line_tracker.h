/**
 * @file    line_tracker.h
 * @brief   Pure eight-channel line tracking target generator.
 */

#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include "gray_sensor.h"
#include "zf_common_typedef.h"

#define LINE_TRACKER_SPEED_BAND_COUNT    (5U)
#define LINE_TRACKER_UPDATE_PERIOD_MS    (10U)

typedef enum
{
    LINE_TRACKER_STATE_TRACKING = 0,
    LINE_TRACKER_STATE_LOST_ARC,
    LINE_TRACKER_STATE_LOST_PIVOT,
    LINE_TRACKER_STATE_ALL_ACTIVE,
    LINE_TRACKER_STATE_FAULT,
} line_tracker_state_enum;

typedef enum
{
    LINE_TRACKER_DIRECTION_LEFT = -1,
    LINE_TRACKER_DIRECTION_NONE = 0,
    LINE_TRACKER_DIRECTION_RIGHT = 1,
} line_tracker_direction_enum;

typedef struct
{
    float base_speed_mm_s[LINE_TRACKER_SPEED_BAND_COUNT];
    float turn_gain[LINE_TRACKER_SPEED_BAND_COUNT];
    float max_target_mm_s;
    float max_correction_mm_s;
    float arc_outer_speed_mm_s;
    float arc_inner_speed_mm_s;
    float pivot_speed_mm_s;
    uint16 lost_debounce_samples;
    uint16 reacquire_samples;
    uint16 arc_duration_samples;
    uint16 search_timeout_samples;
    line_tracker_direction_enum default_search_direction;
} line_tracker_config_struct;

typedef struct
{
    float left_target_mm_s;
    float right_target_mm_s;
} line_tracker_output_struct;

typedef struct
{
    line_tracker_state_enum state;
    gray_sensor_status_enum sensor_status;
    line_tracker_direction_enum search_direction;
    float deviation;
    float normalized_deviation;
    float last_valid_deviation;
    float left_target_mm_s;
    float right_target_mm_s;
    uint16 lost_samples;
    uint16 valid_samples;
    uint16 all_active_samples;
    uint16 search_samples;
    uint8 speed_band;
    uint8 output_limited;
} line_tracker_status_struct;

void line_tracker_init(const line_tracker_config_struct *config);
void line_tracker_reset(void);
uint8 line_tracker_set_config(
    const line_tracker_config_struct *config);

/**
 * @note Call from exactly one periodic context.
 */
uint8 line_tracker_update(
    const gray_sensor_result_struct *sensor,
    line_tracker_output_struct *output);
void line_tracker_get_status(line_tracker_status_struct *status);

#endif
