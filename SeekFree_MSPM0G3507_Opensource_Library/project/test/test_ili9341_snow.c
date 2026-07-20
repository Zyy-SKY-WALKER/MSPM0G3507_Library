/**
 * @file    test_ili9341_snow.c
 * @brief   Continuous snow physics that accumulates into the startup image.
 */

#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341_SNOW)

#include "test_ili9341_snow.h"

#include <stdio.h>

#include "my_lib_ili9341.h"
#include "splash_image_tiles_exact.h"
#include "zf_driver_delay.h"

#define SNOW_BACKGROUND_COLOR          (0x0966U)
#define SNOW_PARTICLE_COUNT            (128U)
#define SNOW_FRAME_DELAY_MS            (25U)
#define SNOW_FIXED_SHIFT               (8U)
#define SNOW_FIXED_ONE                 (1L << SNOW_FIXED_SHIFT)
#define SNOW_GRAVITY                   (24L)
#define SNOW_MAX_HORIZONTAL_SPEED      (2L * SNOW_FIXED_ONE)
#define SNOW_TERMINAL_MIN              (5L * SNOW_FIXED_ONE)
#define SNOW_TERMINAL_RANGE            (4L * SNOW_FIXED_ONE)
#define SNOW_REPOSE_DIFFERENCE         (2U)
#define SNOW_COLLISION_PASSES          (8U)

typedef enum
{
    SNOW_SHAPE_DOT = 0,
    SNOW_SHAPE_CROSS,
    SNOW_SHAPE_STAR,
    SNOW_SHAPE_COUNT,
} snow_shape_enum;

typedef struct
{
    int32 x_fixed;
    int32 y_fixed;
    int32 velocity_x;
    int32 velocity_y;
    int32 terminal_velocity;
    int16 drawn_x;
    int16 drawn_y;
    uint16 color;
    uint8 shape;
    uint8 drawn_size;
    uint8 drawn;
    uint8 active;
} snow_particle_struct;

static snow_particle_struct snow_particles[SNOW_PARTICLE_COUNT];
static uint8 snow_column_height[SPLASH_TILE_COLUMNS];
static uint16 snow_sprite_pixels[25U];
static uint16 snow_exact_tile_pixels[
    SPLASH_EXACT_TILE_SIZE * SPLASH_EXACT_TILE_SIZE];
static uint32 snow_random_state = 0x7A3C9E21U;
static int32 snow_global_wind;
static uint32 snow_settled_tiles;
static uint8 snow_data_error;

static const uint16 snow_particle_colors[] =
{
    ILI9341_COLOR_WHITE,
    0xDFFFU,
    0xBFF7U,
    0xA7FFU,
};

/**
 * @brief Generate one deterministic pseudo-random value.
 */
static uint32 snow_random(void)
{
    uint32 value = snow_random_state;

    value ^= value << 13U;
    value ^= value >> 17U;
    value ^= value << 5U;
    snow_random_state = value;

    return value;
}

/**
 * @brief Convert signed Q8.8 to a floor-rounded integer pixel.
 */
static int32 snow_fixed_to_pixel(int32 value)
{
    if (value >= 0)
    {
        return value >> SNOW_FIXED_SHIFT;
    }

    return -(((-value) + SNOW_FIXED_ONE - 1L) >> SNOW_FIXED_SHIFT);
}

/**
 * @brief Return the bounding size of one snowflake sprite.
 */
static uint8 snow_shape_size(uint8 shape)
{
    static const uint8 sizes[SNOW_SHAPE_COUNT] = {2U, 3U, 5U};

    if (shape >= SNOW_SHAPE_COUNT)
    {
        return 2U;
    }

    return sizes[shape];
}

/**
 * @brief Find a random column that still has room for accumulated snow.
 */
static int16 snow_find_open_column(void)
{
    uint16 attempt;
    uint16 column;

    for (attempt = 0U; attempt < 128U; attempt++)
    {
        column = (uint16)(snow_random() % SPLASH_TILE_COLUMNS);
        if (snow_column_height[column] < SPLASH_TILE_ROWS)
        {
            return (int16)column;
        }
    }

    for (column = 0U; column < SPLASH_TILE_COLUMNS; column++)
    {
        if (snow_column_height[column] < SPLASH_TILE_ROWS)
        {
            return (int16)column;
        }
    }

    return -1;
}

/**
 * @brief Spawn one new flake above a column that is not full.
 */
static void snow_spawn_particle(snow_particle_struct *particle)
{
    int16 column = snow_find_open_column();
    uint8 shape;
    uint8 size;
    int32 center_x;
    int32 x;

    particle->drawn = 0U;
    if (column < 0)
    {
        particle->active = 0U;
        return;
    }

    shape = (uint8)(snow_random() % SNOW_SHAPE_COUNT);
    size = snow_shape_size(shape);
    center_x = (int32)column * SPLASH_TILE_SIZE
        + SPLASH_TILE_SIZE / 2U;
    x = center_x - size / 2U;
    if (x < 0)
    {
        x = 0;
    }
    if ((x + size) > ILI9341_PORTRAIT_WIDTH)
    {
        x = ILI9341_PORTRAIT_WIDTH - size;
    }

    particle->x_fixed = x * SNOW_FIXED_ONE;
    particle->y_fixed = -(int32)(size + (snow_random() % 160U))
        * SNOW_FIXED_ONE;
    particle->velocity_x = (int32)(snow_random() % 257U) - 128L;
    particle->velocity_y = SNOW_FIXED_ONE / 2L
        + (int32)(snow_random() % SNOW_FIXED_ONE);
    particle->terminal_velocity = SNOW_TERMINAL_MIN
        + (int32)(snow_random() % SNOW_TERMINAL_RANGE);
    particle->color = snow_particle_colors[
        snow_random()
            % (sizeof(snow_particle_colors)
                / sizeof(snow_particle_colors[0]))];
    particle->shape = shape;
    particle->drawn_size = size;
    particle->active = 1U;
}

/**
 * @brief Fill the reusable sprite buffer for one shape and color.
 */
static void snow_build_sprite(
    uint8 shape,
    uint8 size,
    uint16 color)
{
    uint8 index;
    uint8 center = (uint8)(size / 2U);

    for (index = 0U; index < (uint8)(size * size); index++)
    {
        snow_sprite_pixels[index] = SNOW_BACKGROUND_COLOR;
    }

    if (shape == SNOW_SHAPE_DOT)
    {
        for (index = 0U; index < (uint8)(size * size); index++)
        {
            snow_sprite_pixels[index] = color;
        }
    }
    else
    {
        for (index = 0U; index < size; index++)
        {
            snow_sprite_pixels[(uint32)center * size + index] = color;
            snow_sprite_pixels[(uint32)index * size + center] = color;
        }

        if (shape == SNOW_SHAPE_STAR)
        {
            snow_sprite_pixels[1U * size + 1U] = color;
            snow_sprite_pixels[1U * size + 3U] = color;
            snow_sprite_pixels[3U * size + 1U] = color;
            snow_sprite_pixels[3U * size + 3U] = color;
        }
    }
}

/**
 * @brief Erase every flake drawn during the previous frame.
 */
static void snow_erase_particles(void)
{
    uint16 index;

    for (index = 0U; index < SNOW_PARTICLE_COUNT; index++)
    {
        snow_particle_struct *particle = &snow_particles[index];

        if (particle->drawn != 0U)
        {
            ili9341_fill_rect(
                (uint16)particle->drawn_x,
                (uint16)particle->drawn_y,
                (uint16)(particle->drawn_x
                    + particle->drawn_size - 1U),
                (uint16)(particle->drawn_y
                    + particle->drawn_size - 1U),
                SNOW_BACKGROUND_COLOR);
            particle->drawn = 0U;
        }
    }
}

/**
 * @brief Return the packed palette index width for one exact tile record.
 */
static uint8 snow_exact_index_bits(uint8 palette_count)
{
    uint8 bits = 0U;
    uint8 value = (uint8)(palette_count - 1U);

    while (value > 0U)
    {
        bits++;
        value >>= 1U;
    }

    return bits;
}

/**
 * @brief Decode one lossless 4x4 RGB565 tile from Flash.
 * @param tile_row Tile row from the top of the image.
 * @param tile_column Tile column from the left of the image.
 * @return One on success, zero on malformed tile data.
 */
static uint8 snow_decode_exact_tile(
    uint16 tile_row,
    uint16 tile_column)
{
    uint32 row_start;
    uint32 row_end;
    uint32 record_offset;
    uint8 column;
    uint8 record_length;
    uint8 palette_count;
    uint8 index_bits;
    uint32 palette_offset;
    uint32 packed_offset;
    uint32 packed_size;
    uint32 bit_buffer = 0U;
    uint8 bits_available = 0U;
    uint32 bit_offset = 0U;
    uint16 pixel_index;

    if ((tile_row >= SPLASH_EXACT_TILE_ROWS)
        || (tile_column >= SPLASH_EXACT_TILE_COLUMNS))
    {
        return 0U;
    }

    row_start = splash_exact_tile_row_offsets[tile_row];
    row_end = splash_exact_tile_row_offsets[tile_row + 1U];
    if ((row_start > row_end)
        || (row_end > SPLASH_EXACT_TILE_DATA_SIZE))
    {
        return 0U;
    }

    record_offset = row_start;
    for (column = 0U; column < tile_column; column++)
    {
        if (record_offset >= row_end)
        {
            return 0U;
        }
        record_length = splash_exact_tile_data[record_offset];
        if ((record_length < 4U)
            || (record_offset + record_length > row_end))
        {
            return 0U;
        }
        record_offset += record_length;
    }

    if (record_offset >= row_end)
    {
        return 0U;
    }
    record_length = splash_exact_tile_data[record_offset];
    palette_count = splash_exact_tile_data[record_offset + 1U];
    if ((record_length < 4U) || (palette_count == 0U)
        || (palette_count > 16U)
        || (record_offset + record_length > row_end))
    {
        return 0U;
    }

    index_bits = snow_exact_index_bits(palette_count);
    palette_offset = record_offset + 2U;
    packed_offset = palette_offset + (uint32)palette_count * 2U;
    packed_size = ((uint32)16U * index_bits + 7U) / 8U;
    if ((packed_offset + packed_size)
        > (record_offset + record_length))
    {
        return 0U;
    }

    for (pixel_index = 0U; pixel_index < 16U; pixel_index++)
    {
        uint8 palette_index;
        uint32 palette_color_offset;

        while (bits_available < index_bits)
        {
            bit_buffer |= (uint32)splash_exact_tile_data[
                packed_offset + bit_offset++] << bits_available;
            bits_available += 8U;
        }
        if (index_bits == 0U)
        {
            palette_index = 0U;
        }
        else
        {
            palette_index = (uint8)(
                bit_buffer & ((1UL << index_bits) - 1UL));
            bit_buffer >>= index_bits;
            bits_available -= index_bits;
        }
        if (palette_index >= palette_count)
        {
            return 0U;
        }

        palette_color_offset = palette_offset
            + (uint32)palette_index * 2U;
        snow_exact_tile_pixels[pixel_index] =
            ((uint16)splash_exact_tile_data[palette_color_offset] << 8U)
            | splash_exact_tile_data[palette_color_offset + 1U];
    }

    return 1U;
}

/**
 * @brief Update gravity, drag, wind and screen-edge reflection.
 */
static void snow_integrate_particles(uint32 frame_index)
{
    uint16 index;

    if ((frame_index % 64U) == 0U)
    {
        snow_global_wind = (int32)(snow_random() % 257U) - 128L;
    }

    for (index = 0U; index < SNOW_PARTICLE_COUNT; index++)
    {
        snow_particle_struct *particle = &snow_particles[index];
        uint8 size;
        int32 maximum_x;
        int32 jitter;

        if (particle->active == 0U)
        {
            continue;
        }

        jitter = (int32)(snow_random() % 17U) - 8L;
        particle->velocity_x +=
            (snow_global_wind - particle->velocity_x) / 32L + jitter;
        particle->velocity_x = particle->velocity_x * 250L / 256L;
        if (particle->velocity_x > SNOW_MAX_HORIZONTAL_SPEED)
        {
            particle->velocity_x = SNOW_MAX_HORIZONTAL_SPEED;
        }
        else if (particle->velocity_x < -SNOW_MAX_HORIZONTAL_SPEED)
        {
            particle->velocity_x = -SNOW_MAX_HORIZONTAL_SPEED;
        }

        particle->velocity_y += SNOW_GRAVITY;
        if (particle->velocity_y > particle->terminal_velocity)
        {
            particle->velocity_y = particle->terminal_velocity;
        }
        particle->x_fixed += particle->velocity_x;
        particle->y_fixed += particle->velocity_y;

        size = snow_shape_size(particle->shape);
        maximum_x = (ILI9341_PORTRAIT_WIDTH - size) * SNOW_FIXED_ONE;
        if (particle->x_fixed < 0)
        {
            particle->x_fixed = 0;
            particle->velocity_x = -particle->velocity_x / 2L;
        }
        else if (particle->x_fixed > maximum_x)
        {
            particle->x_fixed = maximum_x;
            particle->velocity_x = -particle->velocity_x / 2L;
        }
    }
}

/**
 * @brief Detect the highest accumulated column under one particle.
 */
static uint8 snow_particle_collides(
    const snow_particle_struct *particle,
    uint8 *contact_column)
{
    int32 x = snow_fixed_to_pixel(particle->x_fixed);
    int32 y = snow_fixed_to_pixel(particle->y_fixed);
    uint8 size = snow_shape_size(particle->shape);
    uint16 left_column;
    uint16 right_column;
    uint16 column;
    uint8 maximum_height = 0U;
    uint16 surface_y;

    if (x < 0)
    {
        x = 0;
    }
    left_column = (uint16)x / SPLASH_TILE_SIZE;
    right_column = (uint16)(x + size - 1) / SPLASH_TILE_SIZE;
    if (right_column >= SPLASH_TILE_COLUMNS)
    {
        right_column = SPLASH_TILE_COLUMNS - 1U;
    }

    *contact_column = (uint8)left_column;
    for (column = left_column; column <= right_column; column++)
    {
        if (snow_column_height[column] > maximum_height)
        {
            maximum_height = snow_column_height[column];
            *contact_column = (uint8)column;
        }
    }

    surface_y = (uint16)(ILI9341_PORTRAIT_HEIGHT
        - (uint16)maximum_height * SPLASH_TILE_SIZE);
    return (uint8)((y + size) >= surface_y);
}

/**
 * @brief Move a small flake toward a lower adjacent snow column.
 */
static uint8 snow_try_slide(
    snow_particle_struct *particle,
    uint8 contact_column)
{
    uint8 current_height = snow_column_height[contact_column];
    int16 target_column = -1;
    uint8 target_height = current_height;
    uint8 size = snow_shape_size(particle->shape);
    int32 center_x;

    if (size > 3U)
    {
        return 0U;
    }

    if ((contact_column > 0U)
        && ((uint16)snow_column_height[contact_column - 1U]
            + SNOW_REPOSE_DIFFERENCE <= current_height))
    {
        target_column = (int16)contact_column - 1;
        target_height = snow_column_height[contact_column - 1U];
    }
    if (((uint16)contact_column + 1U < SPLASH_TILE_COLUMNS)
        && ((uint16)snow_column_height[contact_column + 1U]
            + SNOW_REPOSE_DIFFERENCE <= current_height)
        && ((target_column < 0)
            || (snow_column_height[contact_column + 1U] < target_height)))
    {
        target_column = (int16)contact_column + 1;
    }

    if (target_column < 0)
    {
        return 0U;
    }

    center_x = (int32)target_column * SPLASH_TILE_SIZE
        + SPLASH_TILE_SIZE / 2U;
    particle->x_fixed = (center_x - size / 2U) * SNOW_FIXED_ONE;
    particle->velocity_x = target_column < contact_column
        ? -SNOW_FIXED_ONE / 2L
        : SNOW_FIXED_ONE / 2L;
    particle->velocity_y = SNOW_FIXED_ONE / 2L;

    return 1U;
}

/**
 * @brief Convert one stable collision into a permanent image tile.
 */
static void snow_settle_particle(
    snow_particle_struct *particle,
    uint8 column)
{
    uint8 height = snow_column_height[column];
    uint16 tile_row;

    if (height >= SPLASH_TILE_ROWS)
    {
        snow_spawn_particle(particle);
        return;
    }

    tile_row = (uint16)(SPLASH_TILE_ROWS - 1U - height);
    if (snow_decode_exact_tile(tile_row, column) == 0U)
    {
        snow_data_error = 1U;
        particle->active = 0U;
        return;
    }
    ili9341_show_rgb565_image(
        (uint16)column * SPLASH_EXACT_TILE_SIZE,
        tile_row * SPLASH_EXACT_TILE_SIZE,
        snow_exact_tile_pixels,
        SPLASH_EXACT_TILE_SIZE,
        SPLASH_EXACT_TILE_SIZE);

    snow_column_height[column]++;
    snow_settled_tiles++;
    snow_spawn_particle(particle);
}

/**
 * @brief Resolve all collisions until no active flake overlaps the pile.
 */
static void snow_resolve_collisions(void)
{
    uint8 pass;

    for (pass = 0U; pass < SNOW_COLLISION_PASSES; pass++)
    {
        uint8 changed = 0U;
        uint16 index;

        for (index = 0U; index < SNOW_PARTICLE_COUNT; index++)
        {
            snow_particle_struct *particle = &snow_particles[index];
            uint8 contact_column;

            if ((particle->active == 0U)
                || (snow_settled_tiles >= SPLASH_TILE_COUNT))
            {
                continue;
            }
            if (snow_particle_collides(particle, &contact_column) == 0U)
            {
                continue;
            }

            if (snow_try_slide(particle, contact_column) == 0U)
            {
                snow_settle_particle(particle, contact_column);
            }
            changed = 1U;
        }

        if (changed == 0U)
        {
            break;
        }
    }

    if (snow_settled_tiles < SPLASH_TILE_COUNT)
    {
        uint16 index;

        for (index = 0U; index < SNOW_PARTICLE_COUNT; index++)
        {
            uint8 contact_column;

            if ((snow_particles[index].active != 0U)
                && (snow_particle_collides(
                        &snow_particles[index],
                        &contact_column) != 0U))
            {
                snow_settle_particle(
                    &snow_particles[index],
                    contact_column);
            }
        }
    }
}

/**
 * @brief Draw all active flakes that remain safely above the pile.
 */
static void snow_draw_particles(void)
{
    uint16 index;

    for (index = 0U; index < SNOW_PARTICLE_COUNT; index++)
    {
        snow_particle_struct *particle = &snow_particles[index];
        uint8 contact_column;
        uint8 size;
        int32 x;
        int32 y;

        if (particle->active == 0U)
        {
            continue;
        }
        if (snow_particle_collides(particle, &contact_column) != 0U)
        {
            snow_spawn_particle(particle);
            continue;
        }

        size = snow_shape_size(particle->shape);
        x = snow_fixed_to_pixel(particle->x_fixed);
        y = snow_fixed_to_pixel(particle->y_fixed);
        if ((y < 0) || ((y + size) > ILI9341_PORTRAIT_HEIGHT))
        {
            continue;
        }

        snow_build_sprite(particle->shape, size, particle->color);
        ili9341_show_rgb565_image(
            (uint16)x,
            (uint16)y,
            snow_sprite_pixels,
            size,
            size);
        particle->drawn_x = (int16)x;
        particle->drawn_y = (int16)y;
        particle->drawn_size = size;
        particle->drawn = 1U;
    }
}

void test_ili9341_snow_run(void)
{
    uint16 index;
    uint16 column;
    uint32 frame_index = 0U;

    ili9341_init();
    ili9341_set_dir(ILI9341_DIR_PORTRAIT);
    ili9341_full(SNOW_BACKGROUND_COLOR);
    snow_random_state = 0x7A3C9E21U;
    snow_global_wind = 0L;
    snow_settled_tiles = 0U;
    snow_data_error = 0U;
    for (column = 0U; column < SPLASH_TILE_COLUMNS; column++)
    {
        snow_column_height[column] = 0U;
    }

    for (index = 0U; index < SNOW_PARTICLE_COUNT; index++)
    {
        snow_spawn_particle(&snow_particles[index]);
    }

    while ((snow_settled_tiles < SPLASH_EXACT_TILE_COUNT)
        && (snow_data_error == 0U))
    {
        snow_erase_particles();
        snow_integrate_particles(frame_index);
        snow_resolve_collisions();
        if (snow_settled_tiles >= SPLASH_EXACT_TILE_COUNT)
        {
            break;
        }
        snow_draw_particles();

        frame_index++;
        if ((frame_index % 40U) == 0U)
        {
            printf(
                "Snow tiles: %lu/%u.\r\n",
                (unsigned long)snow_settled_tiles,
                (unsigned int)SPLASH_EXACT_TILE_COUNT);
        }
        system_delay_ms(SNOW_FRAME_DELAY_MS);
    }

    if (snow_data_error != 0U)
    {
        printf("Snow tile data decode failed.\r\n");
    }
    else
    {
        printf(
            "Snow accumulation complete in %lu frames.\r\n",
            (unsigned long)frame_index);
    }
}

#endif
