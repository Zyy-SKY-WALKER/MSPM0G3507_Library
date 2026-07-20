/**
 * @file    splash_jpeg_player.h
 * @brief   Shared embedded-JPEG block decoder for TFT splash effects.
 */

#ifndef SPLASH_JPEG_PLAYER_H
#define SPLASH_JPEG_PLAYER_H

#include "tjpgd.h"
#include "zf_common_typedef.h"

typedef int (*splash_jpeg_block_callback)(
    const uint16 pixels[],
    uint16 left,
    uint16 top,
    uint16 width,
    uint16 height,
    uint16 image_width,
    uint16 image_height,
    void *context);

/**
 * @brief Decode one JPEG array and dispatch contiguous RGB565 blocks.
 * @param data JPEG byte array in Flash.
 * @param size JPEG byte count.
 * @param scale TJpgDec scale from 0 through 3.
 * @param callback RGB565 block consumer.
 * @param context User context passed to the block consumer.
 * @return TJpgDec result code.
 * @note This module is synchronous and not reentrant.
 */
JRESULT splash_jpeg_decode(
    const uint8 data[],
    uint32 size,
    uint8 scale,
    splash_jpeg_block_callback callback,
    void *context);

#endif
