/**
 * @file    test_ili9341_neon.c
 * @brief   Neon scanline and transient glitch startup animation.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341_NEON)

#include "test_ili9341_neon.h"

#include <stdio.h>
#include <string.h>

#include "my_lib_ili9341.h"
#include "splash_image_jpg.h"
#include "splash_jpeg_player.h"
#include "zf_driver_delay.h"

#define NEON_BACKGROUND_COLOR          (0x0966U)
#define NEON_BAND_HEIGHT               (8U)
#define NEON_SCAN_DELAY_MS             (100U)
#define NEON_GLITCH_COUNT              (6U)

static uint16 neon_band_pixels[
    SPLASH_IMAGE_WIDTH * NEON_BAND_HEIGHT];
static uint32 neon_random_state = 0x4F1BBCDCU;

static uint32 neon_random(void)
{
    uint32 value = neon_random_state;

    value ^= value << 13U;
    value ^= value >> 17U;
    value ^= value << 5U;
    neon_random_state = value;

    return value;
}

/**
 * @brief Draw temporary horizontal glitch fragments around the scanline.
 */
static void neon_draw_glitches(uint16 top, uint16 height)
{
    static const uint16 colors[] =
    {
        ILI9341_COLOR_CYAN,
        ILI9341_COLOR_MAGENTA,
        0x07F0U,
    };
    uint8 index;

    for (index = 0U; index < NEON_GLITCH_COUNT; index++)
    {
        uint16 x = (uint16)(neon_random() % SPLASH_IMAGE_WIDTH);
        uint16 width = (uint16)(8U + (neon_random() % 40U));
        uint16 y = (uint16)(top + (neon_random() % height));
        uint16 x_end = (uint16)(x + width - 1U);

        if (x_end >= SPLASH_IMAGE_WIDTH)
        {
            x_end = SPLASH_IMAGE_WIDTH - 1U;
        }
        ili9341_fill_rect(
            x,
            y,
            x_end,
            y,
            colors[neon_random()
                % (sizeof(colors) / sizeof(colors[0]))]);
    }
}

static int neon_collect_block(
    const uint16 pixels[],
    uint16 left,
    uint16 top,
    uint16 width,
    uint16 height,
    uint16 image_width,
    uint16 image_height,
    void *context)
{
    uint16 row;

    (void)context;
    if ((pixels == NULL) || (image_width != SPLASH_IMAGE_WIDTH)
        || (image_height != SPLASH_IMAGE_HEIGHT)
        || (height > NEON_BAND_HEIGHT)
        || ((uint32)left + width > SPLASH_IMAGE_WIDTH))
    {
        return 0;
    }

    for (row = 0U; row < height; row++)
    {
        memcpy(
            &neon_band_pixels[(uint32)row * SPLASH_IMAGE_WIDTH + left],
            &pixels[(uint32)row * width],
            (size_t)width * sizeof(uint16));
    }

    if ((uint32)left + width >= image_width)
    {
        uint16 line_bottom = (uint16)(top + height - 1U);

        ili9341_fill_rect(
            0U,
            top,
            SPLASH_IMAGE_WIDTH - 1U,
            line_bottom,
            0x03EFU);
        neon_draw_glitches(top, height);
        system_delay_ms(20U);
        ili9341_show_rgb565_image(
            0U,
            top,
            neon_band_pixels,
            SPLASH_IMAGE_WIDTH,
            height);

        if ((uint32)line_bottom + 1U < SPLASH_IMAGE_HEIGHT)
        {
            uint16 scan_y = (uint16)(line_bottom + 1U);

            ili9341_fill_rect(
                0U,
                scan_y,
                SPLASH_IMAGE_WIDTH - 1U,
                scan_y,
                ILI9341_COLOR_CYAN);
        }
        system_delay_ms(NEON_SCAN_DELAY_MS);
    }

    return 1;
}

void test_ili9341_neon_run(void)
{
    JRESULT result;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(NEON_BACKGROUND_COLOR);

    result = splash_jpeg_decode(
        splash_image_jpg,
        SPLASH_IMAGE_JPG_SIZE,
        0U,
        neon_collect_block,
        NULL);
    if (result != JDR_OK)
    {
        printf("Neon scan failed: %u.\r\n", (unsigned int)result);
    }
}

#endif
