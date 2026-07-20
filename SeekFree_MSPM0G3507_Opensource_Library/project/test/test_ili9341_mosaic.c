/**
 * @file    test_ili9341_mosaic.c
 * @brief   Progressive 1/8 to full-resolution mosaic animation.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341_MOSAIC)

#include "test_ili9341_mosaic.h"

#include <stdio.h>

#include "my_lib_ili9341.h"
#include "splash_image_jpg.h"
#include "splash_jpeg_player.h"
#include "zf_driver_delay.h"

#define MOSAIC_STAGE_COUNT             (4U)
#define MOSAIC_STAGE_DELAY_MS          (1000U)
#define MOSAIC_BLOCK_MAX_PIXELS        (64U)

typedef struct
{
    uint8 scale;
} mosaic_context_struct;

static uint16 mosaic_expanded_block[MOSAIC_BLOCK_MAX_PIXELS];

static int mosaic_draw_block(
    const uint16 pixels[],
    uint16 left,
    uint16 top,
    uint16 width,
    uint16 height,
    uint16 image_width,
    uint16 image_height,
    void *context)
{
    mosaic_context_struct *mosaic_context =
        (mosaic_context_struct *)context;
    uint16 factor;
    uint16 output_width;
    uint16 output_height;
    uint16 source_y;
    uint16 source_x;

    (void)image_width;
    (void)image_height;
    if ((pixels == NULL) || (mosaic_context == NULL))
    {
        return 0;
    }

    factor = (uint16)(1U << mosaic_context->scale);
    output_width = (uint16)(width * factor);
    output_height = (uint16)(height * factor);
    if ((uint32)output_width * output_height
        > MOSAIC_BLOCK_MAX_PIXELS)
    {
        return 0;
    }

    for (source_y = 0U; source_y < height; source_y++)
    {
        for (source_x = 0U; source_x < width; source_x++)
        {
            uint16 repeat_y;
            uint16 repeat_x;
            uint16 color = pixels[(uint32)source_y * width + source_x];

            for (repeat_y = 0U; repeat_y < factor; repeat_y++)
            {
                for (repeat_x = 0U; repeat_x < factor; repeat_x++)
                {
                    uint16 output_x = (uint16)(
                        source_x * factor + repeat_x);
                    uint16 output_y = (uint16)(
                        source_y * factor + repeat_y);

                    mosaic_expanded_block[
                        (uint32)output_y * output_width + output_x] = color;
                }
            }
        }
    }

    ili9341_show_rgb565_image(
        (uint16)(left * factor),
        (uint16)(top * factor),
        mosaic_expanded_block,
        output_width,
        output_height);

    return 1;
}

void test_ili9341_mosaic_run(void)
{
    static const uint8 scales[MOSAIC_STAGE_COUNT] = {3U, 2U, 1U, 0U};
    mosaic_context_struct context;
    uint8 stage;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(ILI9341_COLOR_WHITE);

    for (stage = 0U; stage < MOSAIC_STAGE_COUNT; stage++)
    {
        JRESULT result;

        context.scale = scales[stage];
        result = splash_jpeg_decode(
            splash_image_jpg,
            SPLASH_IMAGE_JPG_SIZE,
            context.scale,
            mosaic_draw_block,
            &context);
        if (result != JDR_OK)
        {
            printf(
                "Mosaic stage %u failed: %u.\r\n",
                (unsigned int)stage,
                (unsigned int)result);
            return;
        }
        if ((stage + 1U) < MOSAIC_STAGE_COUNT)
        {
            system_delay_ms(MOSAIC_STAGE_DELAY_MS);
        }
    }
}

#endif
