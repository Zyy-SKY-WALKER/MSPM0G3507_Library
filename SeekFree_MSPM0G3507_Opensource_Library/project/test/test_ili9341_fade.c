/**
 * @file    test_ili9341_fade.c
 * @brief   Multi-pass RGB565 fade-in animation from white.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341_FADE)

#include "test_ili9341_fade.h"

#include <stdio.h>

#include "my_lib_ili9341.h"
#include "splash_image_jpg.h"
#include "splash_jpeg_player.h"
#include "zf_driver_delay.h"

#define FADE_STAGE_COUNT               (10U)
#define FADE_STAGE_DELAY_MS            (450U)
#define FADE_BLOCK_MAX_PIXELS          (64U)

typedef struct
{
    uint8 alpha;
} fade_context_struct;

static uint16 fade_block[FADE_BLOCK_MAX_PIXELS];

/**
 * @brief Blend one RGB565 color from white toward its final value.
 */
static uint16 fade_blend_white(uint16 color, uint8 alpha)
{
    uint32 red = (color >> 11U) & 0x1FU;
    uint32 green = (color >> 5U) & 0x3FU;
    uint32 blue = color & 0x1FU;

    red = 31U - (((31U - red) * alpha + 127U) / 255U);
    green = 63U - (((63U - green) * alpha + 127U) / 255U);
    blue = 31U - (((31U - blue) * alpha + 127U) / 255U);

    return (uint16)((red << 11U) | (green << 5U) | blue);
}

static int fade_draw_block(
    const uint16 pixels[],
    uint16 left,
    uint16 top,
    uint16 width,
    uint16 height,
    uint16 image_width,
    uint16 image_height,
    void *context)
{
    fade_context_struct *fade_context = (fade_context_struct *)context;
    uint32 pixel_count = (uint32)width * height;
    uint32 index;

    if ((pixels == NULL) || (fade_context == NULL)
        || (image_width != SPLASH_IMAGE_WIDTH)
        || (image_height != SPLASH_IMAGE_HEIGHT)
        || (pixel_count > FADE_BLOCK_MAX_PIXELS))
    {
        return 0;
    }

    for (index = 0U; index < pixel_count; index++)
    {
        fade_block[index] = fade_blend_white(
            pixels[index],
            fade_context->alpha);
    }
    ili9341_show_rgb565_image(
        left,
        top,
        fade_block,
        width,
        height);

    return 1;
}

void test_ili9341_fade_run(void)
{
    fade_context_struct context;
    uint8 stage;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(ILI9341_COLOR_WHITE);

    for (stage = 0U; stage < FADE_STAGE_COUNT; stage++)
    {
        JRESULT result;

        context.alpha = (uint8)(
            ((uint16)stage + 1U) * 255U / FADE_STAGE_COUNT);
        result = splash_jpeg_decode(
            splash_image_jpg,
            SPLASH_IMAGE_JPG_SIZE,
            0U,
            fade_draw_block,
            &context);
        if (result != JDR_OK)
        {
            printf(
                "Fade stage %u failed: %u.\r\n",
                (unsigned int)stage,
                (unsigned int)result);
            return;
        }
        if ((stage + 1U) < FADE_STAGE_COUNT)
        {
            system_delay_ms(FADE_STAGE_DELAY_MS);
        }
    }
}

#endif
