/**
 * @file    snow_image_select.h
 * @brief   Select one embedded image for the physical snow test.
 */

#ifndef SNOW_IMAGE_SELECT_H
#define SNOW_IMAGE_SELECT_H

#define SNOW_IMAGE_1                  (1U)
#define SNOW_IMAGE_2                  (2U)
#define SNOW_IMAGE_3                  (3U)

#define SNOW_IMAGE_SELECT             SNOW_IMAGE_3

#if (SNOW_IMAGE_SELECT == SNOW_IMAGE_1)

#include "splash_image_tiles_exact.h"

#define SNOW_IMAGE_TILE_SIZE          SPLASH_EXACT_TILE_SIZE
#define SNOW_IMAGE_TILE_COLUMNS       SPLASH_EXACT_TILE_COLUMNS
#define SNOW_IMAGE_TILE_ROWS          SPLASH_EXACT_TILE_ROWS
#define SNOW_IMAGE_TILE_COUNT         SPLASH_EXACT_TILE_COUNT
#define SNOW_IMAGE_ROW_OFFSETS        splash_exact_tile_row_offsets
#define SNOW_IMAGE_TILE_DATA          splash_exact_tile_data
#define SNOW_IMAGE_TILE_DATA_SIZE     SPLASH_EXACT_TILE_DATA_SIZE

#elif (SNOW_IMAGE_SELECT == SNOW_IMAGE_2)

#include "splash_image2_tiles_quant5.h"

#define SNOW_IMAGE_TILE_SIZE          SPLASH_IMAGE2_TILE_SIZE
#define SNOW_IMAGE_TILE_COLUMNS       SPLASH_IMAGE2_TILE_COLUMNS
#define SNOW_IMAGE_TILE_ROWS          SPLASH_IMAGE2_TILE_ROWS
#define SNOW_IMAGE_TILE_COUNT         SPLASH_IMAGE2_TILE_COUNT
#define SNOW_IMAGE_ROW_OFFSETS        splash_image2_tile_row_offsets
#define SNOW_IMAGE_TILE_DATA          splash_image2_tile_data
#define SNOW_IMAGE_TILE_DATA_SIZE     SPLASH_IMAGE2_TILE_DATA_SIZE

#elif (SNOW_IMAGE_SELECT == SNOW_IMAGE_3)

#include "splash_image3_tiles_quant8.h"

#define SNOW_IMAGE_TILE_SIZE          SPLASH_IMAGE3_Q8_TILE_SIZE
#define SNOW_IMAGE_TILE_COLUMNS       SPLASH_IMAGE3_Q8_TILE_COLUMNS
#define SNOW_IMAGE_TILE_ROWS          SPLASH_IMAGE3_Q8_TILE_ROWS
#define SNOW_IMAGE_TILE_COUNT         SPLASH_IMAGE3_Q8_TILE_COUNT
#define SNOW_IMAGE_ROW_OFFSETS        splash_image3_q8_tile_row_offsets
#define SNOW_IMAGE_TILE_DATA          splash_image3_q8_tile_data
#define SNOW_IMAGE_TILE_DATA_SIZE     SPLASH_IMAGE3_Q8_TILE_DATA_SIZE

#else

#error "Unsupported SNOW_IMAGE_SELECT value"

#endif

#endif
