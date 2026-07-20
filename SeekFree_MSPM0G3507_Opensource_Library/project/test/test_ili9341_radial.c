/**
 * @file    test_ili9341_radial.c
 * @brief   Image blocks expanding from the character's face.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341_RADIAL)

#include "test_ili9341_radial.h"

#include <stdio.h>

#include "my_lib_ili9341.h"
#include "splash_image_jpg.h"
#include "splash_jpeg_player.h"
#include "zf_driver_delay.h"

#define RADIAL_BACKGROUND_COLOR        (0x0966U)
#define RADIAL_CENTER_X                (120)
#define RADIAL_CENTER_Y                (95)
#define RADIAL_PASS_COUNT              (12U)
#define RADIAL_MAX_DISTANCE            (285U)
#define RADIAL_PASS_DELAY_MS           (300U)

typedef struct
{
    uint8 pass;
} radial_context_struct;

/**
 * @brief Estimate block radius without square root operations.
 */
static uint16 radial_distance(uint16 x, uint16 y)
{
    int32 delta_x = (int32)x - RADIAL_CENTER_X;
    int32 delta_y = (int32)y - RADIAL_CENTER_Y;
    uint32 absolute_x;
    uint32 absolute_y;
    uint32 maximum;
    uint32 minimum;

    absolute_x = (uint32)(delta_x < 0 ? -delta_x : delta_x);
    absolute_y = (uint32)(delta_y < 0 ? -delta_y : delta_y);
    maximum = absolute_x > absolute_y ? absolute_x : absolute_y;
    minimum = absolute_x > absolute_y ? absolute_y : absolute_x;

    return (uint16)(maximum + minimum / 2U);
}

static int radial_draw_block(
    const uint16 pixels[],
    uint16 left,
    uint16 top,
    uint16 width,
    uint16 height,
    uint16 image_width,
    uint16 image_height,
    void *context)
{
    radial_context_struct *radial_context =
        (radial_context_struct *)context;
    uint16 center_x;
    uint16 center_y;
    uint16 distance;
    uint8 block_pass;

    if ((pixels == NULL) || (radial_context == NULL)
        || (image_width != SPLASH_IMAGE_WIDTH)
        || (image_height != SPLASH_IMAGE_HEIGHT))
    {
        return 0;
    }

    center_x = (uint16)(left + width / 2U);
    center_y = (uint16)(top + height / 2U);
    distance = radial_distance(center_x, center_y);
    block_pass = (uint8)(
        (uint32)distance * RADIAL_PASS_COUNT / RADIAL_MAX_DISTANCE);
    if (block_pass >= RADIAL_PASS_COUNT)
    {
        block_pass = RADIAL_PASS_COUNT - 1U;
    }

    if (block_pass == radial_context->pass)
    {
        ili9341_show_rgb565_image(
            left,
            top,
            pixels,
            width,
            height);
    }

    return 1;
}

void test_ili9341_radial_run(void)
{
    radial_context_struct context;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(RADIAL_BACKGROUND_COLOR);

    for (context.pass = 0U;
        context.pass < RADIAL_PASS_COUNT;
        context.pass++)
    {
        JRESULT result = splash_jpeg_decode(
            splash_image_jpg,
            SPLASH_IMAGE_JPG_SIZE,
            0U,
            radial_draw_block,
            &context);

        if (result != JDR_OK)
        {
            printf(
                "Radial pass %u failed: %u.\r\n",
                (unsigned int)context.pass,
                (unsigned int)result);
            return;
        }
        if ((context.pass + 1U) < RADIAL_PASS_COUNT)
        {
            system_delay_ms(RADIAL_PASS_DELAY_MS);
        }
    }
}

#endif
