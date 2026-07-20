/**
 * @file    test_ili9341_dissolve.c
 * @brief   Random 8x8 block dissolve startup animation.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341_DISSOLVE)

#include "test_ili9341_dissolve.h"

#include <stdio.h>

#include "my_lib_ili9341.h"
#include "splash_image_jpg.h"
#include "splash_jpeg_player.h"
#include "zf_driver_delay.h"

#define DISSOLVE_BACKGROUND_COLOR      (0x0966U)
#define DISSOLVE_PASS_COUNT            (8U)
#define DISSOLVE_PASS_DELAY_MS         (400U)

typedef struct
{
    uint8 pass;
} dissolve_context_struct;

/**
 * @brief Assign one image block to a deterministic random pass.
 */
static uint8 dissolve_block_pass(uint16 left, uint16 top)
{
    uint32 value = (uint32)(left / 8U) * 0x45D9F3BU;

    value ^= (uint32)(top / 8U) * 0x119DE1F3U;
    value ^= value >> 16U;
    value *= 0x45D9F3BU;
    value ^= value >> 16U;

    return (uint8)(value % DISSOLVE_PASS_COUNT);
}

static int dissolve_draw_block(
    const uint16 pixels[],
    uint16 left,
    uint16 top,
    uint16 width,
    uint16 height,
    uint16 image_width,
    uint16 image_height,
    void *context)
{
    dissolve_context_struct *dissolve_context =
        (dissolve_context_struct *)context;

    if ((pixels == NULL) || (dissolve_context == NULL)
        || (image_width != SPLASH_IMAGE_WIDTH)
        || (image_height != SPLASH_IMAGE_HEIGHT))
    {
        return 0;
    }

    if (dissolve_block_pass(left, top) == dissolve_context->pass)
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

void test_ili9341_dissolve_run(void)
{
    dissolve_context_struct context;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(DISSOLVE_BACKGROUND_COLOR);

    for (context.pass = 0U;
        context.pass < DISSOLVE_PASS_COUNT;
        context.pass++)
    {
        JRESULT result = splash_jpeg_decode(
            splash_image_jpg,
            SPLASH_IMAGE_JPG_SIZE,
            0U,
            dissolve_draw_block,
            &context);

        if (result != JDR_OK)
        {
            printf(
                "Dissolve pass %u failed: %u.\r\n",
                (unsigned int)context.pass,
                (unsigned int)result);
            return;
        }
        if ((context.pass + 1U) < DISSOLVE_PASS_COUNT)
        {
            system_delay_ms(DISSOLVE_PASS_DELAY_MS);
        }
    }
}

#endif
