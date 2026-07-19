/**
 * @file    test_ili9341.c
 * @brief   Five-second JPEG startup-image display test.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341)

#include "test_ili9341.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "my_lib_ili9341.h"
#include "splash_image_jpg.h"
#include "tjpgd.h"
#include "zf_driver_delay.h"

#define SPLASH_ANIMATION_TIME_MS       (5000U)
#define SPLASH_JPEG_WORK_SIZE          (4096U)

typedef struct
{
    const uint8 *data;
    uint32 size;
    uint32 offset;
    uint32 delay_remainder;
} splash_jpeg_context_struct;

static JDEC splash_jpeg_decoder;
static uint32 splash_jpeg_work[
    SPLASH_JPEG_WORK_SIZE / sizeof(uint32)];

/**
 * @brief Read or skip bytes from the embedded JPEG array.
 * @param decoder Active JPEG decoder.
 * @param buffer Destination buffer, or NULL to skip bytes.
 * @param count Requested byte count.
 * @return Number of bytes supplied or skipped.
 */
static size_t splash_jpeg_input(
    JDEC *decoder,
    uint8_t *buffer,
    size_t count)
{
    splash_jpeg_context_struct *context =
        (splash_jpeg_context_struct *)decoder->device;
    uint32 remaining;
    size_t available;

    if ((context == NULL) || (context->offset > context->size))
    {
        return 0U;
    }

    remaining = context->size - context->offset;
    available = count;
    if (available > remaining)
    {
        available = remaining;
    }

    if ((buffer != NULL) && (available > 0U))
    {
        memcpy(
            buffer,
            &context->data[context->offset],
            available);
    }
    context->offset += (uint32)available;

    return available;
}

/**
 * @brief Draw one decoded RGB565 block and pace the row reveal.
 * @param decoder Active JPEG decoder.
 * @param bitmap Contiguous RGB565 output block.
 * @param rect Inclusive output rectangle.
 * @return One to continue decoding, zero to abort.
 */
static int splash_jpeg_output(
    JDEC *decoder,
    void *bitmap,
    JRECT *rect)
{
    splash_jpeg_context_struct *context =
        (splash_jpeg_context_struct *)decoder->device;
    uint16 width;
    uint16 height;
    uint32 delay_ms;

    if ((context == NULL) || (bitmap == NULL) || (rect == NULL)
        || (rect->right < rect->left)
        || (rect->bottom < rect->top))
    {
        return 0;
    }

    width = (uint16)(rect->right - rect->left + 1U);
    height = (uint16)(rect->bottom - rect->top + 1U);
    ili9341_show_rgb565_image(
        rect->left,
        rect->top,
        (const uint16 *)bitmap,
        width,
        height);

    if ((uint32)rect->right + 1U >= decoder->width)
    {
        context->delay_remainder +=
            SPLASH_ANIMATION_TIME_MS * height;
        delay_ms = context->delay_remainder / decoder->height;
        context->delay_remainder %= decoder->height;
        if (delay_ms > 0U)
        {
            system_delay_ms(delay_ms);
        }
    }

    return 1;
}

/**
 * @brief Decode and display the embedded full-screen startup image.
 * @return TJpgDec result code.
 */
static JRESULT splash_image_show(void)
{
    splash_jpeg_context_struct context;
    JRESULT result;

    context.data = splash_image_jpg;
    context.size = SPLASH_IMAGE_JPG_SIZE;
    context.offset = 0U;
    context.delay_remainder = 0U;

    result = jd_prepare(
        &splash_jpeg_decoder,
        splash_jpeg_input,
        splash_jpeg_work,
        sizeof(splash_jpeg_work),
        &context);
    if (result != JDR_OK)
    {
        return result;
    }
    if ((splash_jpeg_decoder.width != SPLASH_IMAGE_WIDTH)
        || (splash_jpeg_decoder.height != SPLASH_IMAGE_HEIGHT))
    {
        return JDR_PAR;
    }

    return jd_decomp(
        &splash_jpeg_decoder,
        splash_jpeg_output,
        0U);
}

/**
 * @brief Initialize the TFT and retain the decoded startup image.
 */
void test_ili9341_run(void)
{
    JRESULT result;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(ILI9341_COLOR_WHITE);

    result = splash_image_show();
    if (result != JDR_OK)
    {
        printf("Splash JPEG failed: %u.\r\n", (unsigned int)result);
    }
}

#endif
