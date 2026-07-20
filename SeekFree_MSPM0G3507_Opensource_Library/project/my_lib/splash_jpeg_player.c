/**
 * @file    splash_jpeg_player.c
 * @brief   Shared embedded-JPEG block decoder for TFT splash effects.
 */

#include "splash_jpeg_player.h"

#include <stddef.h>
#include <string.h>

#define SPLASH_JPEG_WORK_SIZE          (4096U)

typedef struct
{
    const uint8 *data;
    uint32 size;
    uint32 offset;
    splash_jpeg_block_callback callback;
    void *callback_context;
} splash_jpeg_session_struct;

static JDEC splash_jpeg_decoder;
static uint32 splash_jpeg_work[
    SPLASH_JPEG_WORK_SIZE / sizeof(uint32)];

/**
 * @brief Supply or skip JPEG bytes from a Flash array.
 */
static size_t splash_jpeg_input(
    JDEC *decoder,
    uint8_t *buffer,
    size_t count)
{
    splash_jpeg_session_struct *session =
        (splash_jpeg_session_struct *)decoder->device;
    uint32 remaining;
    size_t available;

    if ((session == NULL) || (session->offset > session->size))
    {
        return 0U;
    }

    remaining = session->size - session->offset;
    available = count;
    if (available > remaining)
    {
        available = remaining;
    }
    if ((buffer != NULL) && (available > 0U))
    {
        memcpy(
            buffer,
            &session->data[session->offset],
            available);
    }
    session->offset += (uint32)available;

    return available;
}

/**
 * @brief Forward one TJpgDec block to the selected effect.
 */
static int splash_jpeg_output(
    JDEC *decoder,
    void *bitmap,
    JRECT *rect)
{
    splash_jpeg_session_struct *session =
        (splash_jpeg_session_struct *)decoder->device;
    uint16 width;
    uint16 height;
    uint16 image_width;
    uint16 image_height;

    if ((session == NULL) || (session->callback == NULL)
        || (bitmap == NULL) || (rect == NULL)
        || (rect->right < rect->left)
        || (rect->bottom < rect->top))
    {
        return 0;
    }

    width = (uint16)(rect->right - rect->left + 1U);
    height = (uint16)(rect->bottom - rect->top + 1U);
    image_width = (uint16)(decoder->width >> decoder->scale);
    image_height = (uint16)(decoder->height >> decoder->scale);

    return session->callback(
        (const uint16 *)bitmap,
        rect->left,
        rect->top,
        width,
        height,
        image_width,
        image_height,
        session->callback_context);
}

JRESULT splash_jpeg_decode(
    const uint8 data[],
    uint32 size,
    uint8 scale,
    splash_jpeg_block_callback callback,
    void *context)
{
    splash_jpeg_session_struct session;
    JRESULT result;

    if ((data == NULL) || (size == 0U) || (callback == NULL)
        || (scale > 3U))
    {
        return JDR_PAR;
    }

    session.data = data;
    session.size = size;
    session.offset = 0U;
    session.callback = callback;
    session.callback_context = context;

    result = jd_prepare(
        &splash_jpeg_decoder,
        splash_jpeg_input,
        splash_jpeg_work,
        sizeof(splash_jpeg_work),
        &session);
    if (result != JDR_OK)
    {
        return result;
    }

    return jd_decomp(
        &splash_jpeg_decoder,
        splash_jpeg_output,
        scale);
}
