/**
 * ESP32 Blinkenlights.
 * Copyright (C) 2019-2022  Tido Klaassen <tido@4gh.eu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "esp_log.h"
#include <math.h>
#include <sys/param.h>

#include "klist.h"
#include "ws2812.h"
#include "blinken.h"
#include "openhaystack_main.h"


#define REFRESH             50
#define FBUFFER_LEN         16
#define BRIGHTNESS_STEPS    8
#define MAX_DIST            75
#define GLYPH_DIST          55
#define CLAMP(val, min, max) (MIN(MAX(val, (min)), (max)))

static const char *TAG = "BADGE";

struct arg_badge
{
    unsigned int fbuffer_len;
    unsigned int offset;
};

struct ctx_root {
    unsigned int brightness;
};

struct ctx_base {
    unsigned int fbuffer_len;
    unsigned int offset;
    unsigned int ticks;
    uint64_t wait;
};

enum air_qual {
    air_undef = 0,
    air_init,
    air_good,
    air_normal,
    air_bad
};

struct ctx_air {
    struct ctx_base base;
    uint64_t last_trigger;
    uint64_t wait;
    enum air_qual quality;
};

enum nfc_event {
    nfc_undef = 0,
    nfc_read,
    nfc_write
};

struct ctx_nfc {
    struct ctx_base base;
    uint64_t last_trigger;
    uint64_t wait;
    enum nfc_event event;
};

enum ir_event {
    ir_undef = 0,
    ir_rcvd,
    ir_sent
};

struct ctx_ir {
    struct ctx_base base;
    uint64_t last_trigger;
    uint64_t wait;
    enum ir_event event;
};

struct ctx_badge
{
    struct ctx_base base;
    unsigned int list_idx;
    unsigned int seq_idx;
    unsigned int loop_cnt;
};

typedef int (*badge_scene_fn)(struct ctx_badge *ctx, void *arg);

struct badge_scene {
    badge_scene_fn scene;
    unsigned int loops;
    void *arg;
};

struct badge_sequence {
    size_t seq_len;
    struct badge_scene scenes[];
};

struct badge_playlist {
    size_t list_len;
    struct badge_sequence *sequences[];
};

static int badge_scene_fade(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_hold(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_sparkle(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_ping(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_reflect(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_radar(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_pulse(struct ctx_badge *ctx, void *arg __maybe_unused);
static int badge_scene_rainbow(struct ctx_badge *ctx, void *arg __maybe_unused);

struct vector {
    float x;
    float y;
};

struct badge_reflect_arg {
    struct vector origin;
    hsv_value_t hsv;
};

struct badge_reflect_arg ping_ref_arg = {
    .origin = { .x = 0, .y = 400},
    .hsv = {.hue = HSV_RED, .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX},
};

struct badge_reflect_arg fan_ref_arg = {
    .origin = { .x = 0, .y = -100},
    .hsv = {.hue = HSV_GREEN + HSV_YELLOW / 4, .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX},
};

struct badge_sequence seq_sparkle = {
    .scenes = {
        { badge_scene_fade,     1, NULL },
        { badge_scene_sparkle,   0, NULL },
    },
    .seq_len = 2,
};

struct badge_sequence seq_mixed = {
    .scenes = {
        { badge_scene_fade,     1, NULL },
        { badge_scene_pulse,   15, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_sparkle,  1, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_rainbow,  5, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_sparkle,  1, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_ping,     3, NULL },
        { badge_scene_reflect,  1, &ping_ref_arg },
        { badge_scene_hold,     4, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_sparkle,  1, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_radar,    3, NULL },
        { badge_scene_fade,     1, NULL },
        { badge_scene_sparkle,  1, NULL },
    },
    .seq_len = 18,
};

struct badge_sequence seq_ping = {
    .scenes = {
        { badge_scene_fade,     1, NULL },
        { badge_scene_ping,     3, NULL },
        { badge_scene_reflect,  1, &ping_ref_arg },
        { badge_scene_hold,     4, NULL },
    },
    .seq_len = 4,
};

struct badge_sequence seq_radar = {
    .scenes = {
        { badge_scene_fade,     1, NULL },
        { badge_scene_radar,    0, NULL },
    },
    .seq_len = 2,
};

struct badge_sequence seq_rainbow = {
    .scenes = {
        { badge_scene_fade,     1, NULL },
        { badge_scene_rainbow,  0, NULL },
    },
    .seq_len = 2,
};

struct badge_sequence seq_pulse = {
    .scenes = {
        { badge_scene_fade,     1, NULL },
        { badge_scene_pulse,    0, NULL },
    },
    .seq_len = 2,
};

struct badge_playlist playlist = {
    .sequences = {
        &seq_mixed,
        &seq_pulse,
        &seq_rainbow,
        &seq_ping,
        &seq_radar,
    },
    .list_len = 5,
};

struct g_shape {
    size_t num_pixels;
    struct vector pixels[];
};

struct glyph {
    struct vector position;
    struct g_shape *shape;
    hsv_value_t **pixels;
};


struct g_shape g_top = {
    1, { { 0.0, 26.5 } } };
struct g_shape g_tr = {
    3, { { 18.5, 26.5 }, { 37.0, 26.5 }, { 37.0, 13.25 } } };
struct g_shape g_right = {
    1, { { 37.0, 0.0 } } };
struct g_shape g_br = {
    3, { { 37.0, -13.25 }, { 37.0, -26.5 }, { 18.5, -26.5 } } };
struct g_shape g_bottom = {
    1, { { 0.0, -26.5 } } };
struct g_shape g_bl = {
    3, { { -18.5, -26.5 }, { -37.0, -26.5 }, { -37.0, -13.25 } } };
struct g_shape g_left = {
    1, { { -37.0, 0.0 } } };
struct g_shape g_tl = {
    3, { { -37.0, 13.25 }, { -37.0, 26.5 }, { -18.5, 26.5 } } };

struct glyph badge_glyphs[] = {
    {.shape = &g_top},      // 0
    {.shape = &g_tr},       // 1
    {.shape = &g_right},    // 2
    {.shape = &g_br},       // 3
    {.shape = &g_bottom},   // 4
    {.shape = &g_bl},       // 5
    {.shape = &g_left},     // 6
    {.shape = &g_tl},       // 7
};

struct glyph *all_glyphs[] = {
    &(badge_glyphs[0]),
    &(badge_glyphs[1]),
    &(badge_glyphs[2]),
    &(badge_glyphs[3]),
    &(badge_glyphs[4]),
    &(badge_glyphs[5]),
    &(badge_glyphs[6]),
    &(badge_glyphs[7]),
};

struct glyph *corner_glyphs[] = {
    &(badge_glyphs[1]),
    &(badge_glyphs[3]),
    &(badge_glyphs[5]),
    &(badge_glyphs[7]),
};

struct glyph *side_glyphs[] = {
    &(badge_glyphs[0]),
    &(badge_glyphs[2]),
    &(badge_glyphs[4]),
    &(badge_glyphs[6]),
};

/* Easy look-up table for all primary and secondary colours. */
hsv_value_t colours[] = {
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED     },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_YELLOW  },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_GREEN   },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_CYAN    },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_BLUE    },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_MAGENTA },
};

static struct led_filter f_root;
static struct led_filter f_air;
static struct led_filter f_ir;
static struct led_filter f_nfc;
static struct led_filter f_badge;
static hsv_value_t fbuffer[FBUFFER_LEN];

static void scale_fbuffer(hsv_value_t *hsv, size_t len, float factor)
{
    unsigned int idx;

    for(idx = 0; idx < len; ++idx){
        hsv->value *= factor;
        ++hsv;
    }
}

static void badge_init(void)
{
    unsigned int g_idx, s_idx, cnt;
    hsv_value_t hsv;
    init_ble();

    hsv.saturation = HSV_SAT_MAX;
    hsv.value = HSV_VAL_MIN;
    hsv.hue = HSV_GREEN;

    for(s_idx = 0; s_idx < FBUFFER_LEN; ++s_idx){
        fbuffer[s_idx] = hsv;
    }

    /* Map all pixels in the glyphs to pixels in the fbuffer */
    cnt = 0;
    for(g_idx = 0; g_idx < ARRAY_SIZE(badge_glyphs); ++g_idx){
        badge_glyphs[g_idx].pixels = malloc(badge_glyphs[g_idx].shape->num_pixels * sizeof(hsv_value_t *));
        configASSERT(badge_glyphs[g_idx].pixels != NULL);
        configASSERT((cnt + badge_glyphs[g_idx].shape->num_pixels) <= ARRAY_SIZE(fbuffer));

        for(s_idx = 0; s_idx < badge_glyphs[g_idx].shape->num_pixels; ++s_idx){
            badge_glyphs[g_idx].pixels[s_idx] = &(fbuffer[cnt]);
            ++cnt;
        }
    }
}

static void badge_fade(struct glyph *glyphs[], size_t len, float factor)
{
    unsigned int g_idx, s_idx;

    for(g_idx = 0; g_idx < len; ++g_idx){
        for(s_idx = 0; s_idx < glyphs[g_idx]->shape->num_pixels; ++s_idx){
            glyphs[g_idx]->pixels[s_idx]->value *= factor;
        }
    }
}

static void badge_paint(struct glyph *glyphs[], size_t len, hsv_value_t *hsv)
{
    unsigned int g_idx, s_idx;

    for(g_idx = 0; g_idx < len; ++g_idx){
        for(s_idx = 0; s_idx < glyphs[g_idx]->shape->num_pixels; ++s_idx){
            *glyphs[g_idx]->pixels[s_idx] = *hsv;
        }
    }
}

static void badge_blend(struct glyph *glyphs[], size_t len, hsv_value_t hsv, float factor)
{
    unsigned int g_idx, s_idx;
    hsv_value_t tmp;

    for(g_idx = 0; g_idx < len; ++g_idx){
        for(s_idx = 0; s_idx < glyphs[g_idx]->shape->num_pixels; ++s_idx){
            tmp = *glyphs[g_idx]->pixels[s_idx];
            tmp.hue = (int)(tmp.hue * (1.0 - factor) + hsv.hue * factor) % HSV_HUE_MAX;
            tmp.saturation = CLAMP(tmp.saturation * (1.0 - factor) + hsv.saturation * factor, HSV_SAT_MIN, HSV_SAT_MAX);
            tmp.value = CLAMP(tmp.value * (1.0 - factor) + hsv.value * factor, HSV_VAL_MIN, HSV_VAL_MAX);
            *glyphs[g_idx]->pixels[s_idx] = tmp;
        }
    }
}

static void badge_circle(struct glyph *glyphs[], size_t len, struct vector origin, float radius, float width, hsv_value_t hsv)
{
    unsigned int g_idx, s_idx;
    float dist, x_dist, y_dist;
    struct vector *vec;

    width = fabs(width);
    for(g_idx = 0; g_idx < len; ++g_idx){
        for(s_idx = 0; s_idx < glyphs[g_idx]->shape->num_pixels; ++s_idx){
            vec = &(glyphs[g_idx]->shape->pixels[s_idx]);
            x_dist = vec->x - origin.x;
            y_dist = vec->y - origin.y;
            dist = sqrtf(x_dist * x_dist + y_dist * y_dist);
            if(fabs(dist - radius) <= width ){
                *(glyphs[g_idx]->pixels[s_idx]) = hsv;
            }
        }
    }
}

static void badge_line(struct glyph *glyphs[], size_t len, struct vector origin, float angle, float width, hsv_value_t hsv)
{
    unsigned int g_idx, s_idx;
    float dist;
    struct vector *vec;

    width = fabs(width);
    for(g_idx = 0; g_idx < len; ++g_idx){
        for(s_idx = 0; s_idx < glyphs[g_idx]->shape->num_pixels; ++s_idx){
            vec = &(glyphs[g_idx]->shape->pixels[s_idx]);
            dist = fabs(cosf(angle) * (origin.y - vec->y) - sinf(angle) * (origin.x - vec->x));
            if(dist <= width){
                *(glyphs[g_idx]->pixels[s_idx]) = hsv;
            }
        }
    }
}

static int badge_scene_sparkle(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    unsigned int idx;
    hsv_value_t hsv;

    hsv.saturation = HSV_SAT_MAX;
    hsv.hue = HSV_GREEN + HSV_YELLOW / 4;
    if(ctx->base.ticks < 50){
        hsv.value = (ctx->base.ticks * 0.7f * HSV_VAL_MAX) / 50;
        badge_paint(all_glyphs, ARRAY_SIZE(all_glyphs), &hsv);
    } else {
        hsv.value = 0.7f * HSV_VAL_MAX;
        if(rand() % 50 == 0){
            idx = rand() % ARRAY_SIZE(fbuffer);
            fbuffer[idx] = hsv;
            fbuffer[idx].hue = HSV_BLUE;
            fbuffer[idx].saturation = HSV_SAT_MIN;
        }
        badge_blend(all_glyphs, ARRAY_SIZE(all_glyphs), hsv, 0.05f);
    }

    return ctx->base.ticks > 2000;
}

static int badge_scene_ping(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    struct vector origin;
    float radius;
    hsv_value_t hsv;
    int result;

    result = 0;
    origin.x = 0;
    origin.y = 0;
    radius = 4 * ctx->base.ticks;

    hsv.saturation = HSV_SAT_MAX;
    hsv.value = HSV_VAL_MAX;
    hsv.hue = HSV_BLUE;

    scale_fbuffer(fbuffer, ARRAY_SIZE(fbuffer), 0.90f);
    badge_circle(all_glyphs, ARRAY_SIZE(all_glyphs), origin, radius, 10.0, hsv);

    if(ctx->base.ticks >= 150){
        result = 1;
    }

    return result;
}

static int badge_scene_reflect(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    struct vector origin;
    struct badge_reflect_arg *my_arg;
    float radius;
    hsv_value_t hsv;
    int result;

    result = 0;

    my_arg = (struct badge_reflect_arg *) arg;

    origin = my_arg->origin;
    hsv = my_arg->hsv;

    radius = 4 * ctx->base.ticks;

    badge_circle(all_glyphs, ARRAY_SIZE(all_glyphs), origin, radius, 10.0, hsv);

    if(ctx->base.ticks >= 300){
        result = 1;
    }

    return result;
}

static int badge_scene_radar(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    struct vector origin;
    hsv_value_t hsv;
    float angle;
    int result;

    origin.x = 0;
    origin.y = 0;
    result = 0;

    if((ctx->base.ticks / 250) % 2 != 0){
        angle = M_PI - ((4 * ctx->base.ticks) % 250) * M_PI / 250;
    }else {
        angle = ((4 * ctx->base.ticks) % 250) * M_PI / 250;
    }

    hsv.saturation = HSV_SAT_MAX;
    hsv.value = HSV_VAL_MAX;
    hsv.hue = HSV_GREEN;

    badge_fade(all_glyphs, ARRAY_SIZE(all_glyphs), 0.99f);
    badge_line(all_glyphs, ARRAY_SIZE(all_glyphs), origin, angle, 2.0, hsv);

    if(ctx->base.ticks >= 500){
        result = 1;
    }

    return result;
}

static int badge_scene_pulse(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    struct vector origin;
    float radius;
    hsv_value_t hsv;
    int result;

    result = 0;

    origin.x = 0;
    origin.y = 0;

    radius = 2 * ctx->base.ticks;

    hsv.saturation = HSV_SAT_MAX;
    hsv.value = HSV_VAL_MAX;
    hsv.hue = (int)((HSV_HUE_MAX / MAX_DIST) * radius + (ctx->loop_cnt * HSV_HUE_MAX / 8)) % HSV_HUE_MAX;

    scale_fbuffer(fbuffer, ARRAY_SIZE(fbuffer), 0.9925f);
    badge_circle(all_glyphs, ARRAY_SIZE(all_glyphs), origin, radius, 10.0, hsv);

    if(1.25 * ctx->base.ticks >= MAX_DIST){
        result = 1;
    }

    return result;
}

static int badge_scene_rainbow(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    unsigned int idx, ticks, offset;
    size_t len;
    hsv_value_t hsv;
    int result;

    result = 0;
    len = ARRAY_SIZE(fbuffer);
    hsv.saturation = HSV_SAT_MAX;
    hsv.value = HSV_VAL_MAX;

    offset = (ctx->base.ticks / 4) % len;

    scale_fbuffer(fbuffer, len, 0.9925f);

    hsv.hue = (HSV_HUE_MAX / len) * ((ctx->base.ticks / 4) % len);
    fbuffer[offset] = hsv;

    /* signal scene completion after three rounds. */
    if(ctx->base.ticks >= 4 * 3 * ARRAY_SIZE(fbuffer)){
        result = 1;
    }

    return result;
}

static int badge_scene_fade(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    int result = 0;

    scale_fbuffer(fbuffer, ARRAY_SIZE(fbuffer), 0.95f);

    if(ctx->base.ticks >= 50){
        result = 1;
    }

    return result;
}

static int badge_scene_hold(struct ctx_badge *ctx, void *arg __maybe_unused)
{
    int result = 0;


    if(ctx->base.ticks >= 50){
        result = 1;
    }

    return result;
}

/* Root filter. Basically just copy framebuffer and adjust brightness*/
static void filter_root(struct led_filter *this,
                        void *scene_ptr,
                        hsv_value_t hsv_vals[],
                        unsigned int strip_len,
                        unsigned int offset,
                        uint64_t now)
{
    struct ctx_root *ctx;
    unsigned int idx, tmp;

    ctx = (typeof(ctx)) this->priv;

    run_child_filters(this, scene_ptr, hsv_vals, strip_len, offset, now);

    memmove(&hsv_vals[0], fbuffer, sizeof(fbuffer));

    for(idx = 0; idx < ARRAY_SIZE(fbuffer); ++idx){
        tmp = (hsv_vals[idx].value * (HSV_VAL_MAX / (BRIGHTNESS_STEPS - 1) * ctx->brightness)) / HSV_VAL_MAX;
        hsv_vals[idx].value = tmp;
    }
}

static int event_root(struct led_filter *this, void *scene_ptr, struct ctrl_event *evt)
{
    struct ctx_root *ctx;
    int result;

    ctx = (typeof(ctx)) this->priv;

    result = 0;
    switch(evt->event){
    case EVNT_BTN0_S:
    case EVNT_VOLUP:
        ctx->brightness++;
        ctx->brightness %= BRIGHTNESS_STEPS;
        result = 1;
        break;
    default:
        result = forward_event(this, scene_ptr, evt);
    }

    return result;
}

static int init_root(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    struct ctx_root *ctx;
    struct arg_badge *my_arg = arg;
    int result;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        if(my_arg == NULL){
            ESP_LOGE(TAG, "[%s] No argument given.", __func__);
            result = ESP_ERR_INVALID_ARG;
            goto err_out;
        }

        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;

        this->parent = NULL;
        this->name = "root";
        this->filter = filter_root;
        this->event = event_root;
        this->init = init_root;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));

        ctx->brightness = 2;
    }

err_out:
    return result;
}

/* Air filter. React to air quality events. Top priority. */
static void filter_air(struct led_filter *this,
                        void *scene_ptr,
                        hsv_value_t hsv_vals[],
                        unsigned int strip_len,
                        unsigned int offset,
                        uint64_t now)
{
    struct ctx_air *ctx;
    unsigned int idx, len;
    hsv_value_t hsv;

    ctx = (typeof(ctx)) this->priv;

    if(ctx->quality == air_bad || ctx->quality == air_init){
        hsv.saturation = HSV_SAT_MAX;
        hsv.value = HSV_VAL_MAX;

        switch(ctx->quality){
        case air_init:
            hsv.hue = HSV_GREEN;
            break;
        case air_bad:
            hsv.hue = HSV_RED;
            break;
        default:
            hsv.hue = HSV_YELLOW;
            break;
        }

        if(ctx->base.wait <= now){
            len = ARRAY_SIZE(fbuffer);

            if(ctx->base.ticks % 100 < 10){
                for(idx = 0; idx < FBUFFER_LEN; ++idx){
                    fbuffer[idx] = hsv;
                }
            } else {
                scale_fbuffer(fbuffer, len, 0.9f);
            }

            fbuffer[(0 * len / 4 + ctx->base.ticks / 5) % len] = hsv;
            fbuffer[(1 * len / 4 + ctx->base.ticks / 5) % len] = hsv;
            fbuffer[(2 * len / 4 + ctx->base.ticks / 5) % len] = hsv;
            fbuffer[(3 * len / 4 + ctx->base.ticks / 5) % len] = hsv;
            ctx->base.wait += ms_to_us(10);
            ctx->base.ticks++;
        }
    } else {
        run_child_filters(this, scene_ptr, hsv_vals, strip_len, offset, now);
    }
}

static int event_air(struct led_filter *this, void *scene_ptr, struct ctrl_event *evt)
{
    struct ctx_air *ctx;
    int result;

    ctx = (typeof(ctx)) this->priv;

    result = 0;
    switch(evt->event){
    case EVNT_AIR_INIT:
        ESP_LOGE(TAG, "Event Air Init");
        ctx->quality = air_init;
        ctx->last_trigger = esp_timer_get_time();
        result = 1;
        break;
    case EVNT_AIR_GOOD:
        ESP_LOGE(TAG, "Event Air Good");
        ctx->quality = air_good;
        ctx->last_trigger = esp_timer_get_time();
        result = 1;
        break;
    case EVNT_AIR_NORMAL:
        ESP_LOGE(TAG, "Event Air normal");
        ctx->quality = air_normal;
        ctx->last_trigger = esp_timer_get_time();
        result = 1;
        break;
    case EVNT_AIR_BAD:
        ESP_LOGE(TAG, "Event Air Bad");
        ctx->last_trigger = esp_timer_get_time();
        if(ctx->quality != air_bad){
            ctx->base.wait = ctx->last_trigger;
            ctx->base.ticks = 0;
        }
        ctx->quality = air_bad;
        result = 1;
        break;
    default:
        result = forward_event(this, scene_ptr, evt);
    }

    return result;
}

static int init_air(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    struct ctx_air *ctx;
    struct arg_badge *my_arg = arg;
    int result;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        if(my_arg == NULL){
            ESP_LOGE(TAG, "[%s] No argument given.", __func__);
            result = ESP_ERR_INVALID_ARG;
            goto err_out;
        }

        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;

        this->parent = NULL;
        this->name = "badge";
        this->filter = filter_air;
        this->event = event_air;
        this->init = init_air;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));

        ctx->base.fbuffer_len = my_arg->fbuffer_len;
        ctx->base.offset = my_arg->offset;
        ctx->base.wait = 0;
    }

err_out:
    return result;
}

/* Infrared filter. React to infrared events. */
static void filter_ir(struct led_filter *this,
                        void *scene_ptr,
                        hsv_value_t hsv_vals[],
                        unsigned int strip_len,
                        unsigned int offset,
                        uint64_t now)
{
    struct ctx_ir *ctx;
    unsigned int idx, len;
    hsv_value_t hsv;

    ctx = (typeof(ctx)) this->priv;
    len = ARRAY_SIZE(fbuffer);

    if(ctx->base.ticks < 3 * len + 20){
        hsv.saturation = HSV_SAT_MAX;
        hsv.value = HSV_VAL_MAX;
        hsv.hue = HSV_YELLOW;

        if(ctx->base.wait <= now){
            len = ARRAY_SIZE(fbuffer);

            if(ctx->base.ticks < 10 || ctx->base.ticks >= (3 * len + 10)){
                for(idx = 0; idx < len; ++idx){
                    fbuffer[idx] = hsv;
                }
            } else {
                scale_fbuffer(fbuffer, len, 0.9f);
                fbuffer[(ctx->base.ticks - 10) % len] = hsv;
            }

            ctx->base.wait += ms_to_us(10);
            ctx->base.ticks++;
        }
    } else {
        run_child_filters(this, scene_ptr, hsv_vals, strip_len, offset, now);
    }
}

static int event_ir(struct led_filter *this, void *scene_ptr, struct ctrl_event *evt)
{
    struct ctx_ir *ctx;
    int result;

    ctx = (typeof(ctx)) this->priv;

    result = 0;
    switch(evt->event){
    case EVNT_OK:
        ESP_LOGE(TAG, "Event OK");
        ctx->last_trigger = esp_timer_get_time();
        ctx->base.wait = ctx->last_trigger;
        ctx->base.ticks = 0;
        result = 1;
        break;
    default:
        result = forward_event(this, scene_ptr, evt);
    }

    return result;
}

static int init_ir(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    struct ctx_ir *ctx;
    struct arg_badge *my_arg = arg;
    int result;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        if(my_arg == NULL){
            ESP_LOGE(TAG, "[%s] No argument given.", __func__);
            result = ESP_ERR_INVALID_ARG;
            goto err_out;
        }

        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        ctx->base.ticks = 42000; // prevent filter from triggering
        this->priv = ctx;

        this->parent = NULL;
        this->name = "ir";
        this->filter = filter_ir;
        this->event = event_ir;
        this->init = init_ir;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));
    }

err_out:
    return result;
}

/* NFC filter. React to NFC events. */
static void filter_nfc(struct led_filter *this,
                        void *scene_ptr,
                        hsv_value_t hsv_vals[],
                        unsigned int strip_len,
                        unsigned int offset,
                        uint64_t now)
{
    struct ctx_nfc *ctx;

    ctx = (typeof(ctx)) this->priv;

    run_child_filters(this, scene_ptr, hsv_vals, strip_len, offset, now);

    if(ctx->base.wait <= now){
        ctx->base.wait += ms_to_us(10);

    }
}

static int event_nfc(struct led_filter *this, void *scene_ptr, struct ctrl_event *evt)
{
    struct ctx_nfc *ctx;
    int result;

    ctx = (typeof(ctx)) this->priv;

    result = 0;
    switch(evt->event){
    default:
        result = forward_event(this, scene_ptr, evt);
    }

    return result;
}

static int init_nfc(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    struct ctx_nfc *ctx;
    struct arg_badge *my_arg = arg;
    int result;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        if(my_arg == NULL){
            ESP_LOGE(TAG, "[%s] No argument given.", __func__);
            result = ESP_ERR_INVALID_ARG;
            goto err_out;
        }

        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;

        this->parent = NULL;
        this->name = "nfc";
        this->filter = filter_nfc;
        this->event = event_nfc;
        this->init = init_nfc;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));
    }

err_out:
    return result;
}

/* Base badge filter. All the standard blinky stuff. */
static void filter_badge(struct led_filter *this,
                        void *scene_ptr,
                        hsv_value_t hsv_vals[],
                        unsigned int strip_len,
                        unsigned int offset,
                        uint64_t now)
{
    struct ctx_badge *ctx;
    struct badge_scene *scene;
    int result;

    ctx = (typeof(ctx)) this->priv;

    run_child_filters(this, scene_ptr, hsv_vals, strip_len, offset, now);

    if(ctx->base.wait <= now){
        ctx->base.wait += ms_to_us(10);

        scene = &playlist.sequences[ctx->list_idx]->scenes[ctx->seq_idx];
        result = scene->scene(ctx, scene->arg);

        ++ctx->base.ticks;

        if(result != 0){
            ctx->base.ticks = 0;
            ctx->loop_cnt++;

            if(scene->loops > 0 && ctx->loop_cnt >= scene->loops){
                ctx->seq_idx++;
                ctx->seq_idx %= playlist.sequences[ctx->list_idx]->seq_len;
                ctx->loop_cnt = 0;
            }
        }
    }
}

static int event_badge(struct led_filter *this, void *scene_ptr, struct ctrl_event *evt)
{
    struct ctx_badge *ctx;
    int result;

    ctx = (typeof(ctx)) this->priv;

    result = 0;
    switch(evt->event){
    case EVNT_BTN0_L:
    case EVNT_OK:
        ctx->list_idx++;
        ctx->list_idx %= playlist.list_len;
        ctx->seq_idx = 0;
        ctx->loop_cnt = 0;
        ctx->base.ticks = 0;
        result = 1;
        break;
    default:
        result = forward_event(this, scene_ptr, evt);
    }

    return result;
}


static int init_badge(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    struct ctx_badge *ctx;
    struct arg_badge *my_arg = arg;
    int result;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        if(my_arg == NULL){
            ESP_LOGE(TAG, "[%s] No argument given.", __func__);
            result = ESP_ERR_INVALID_ARG;
            goto err_out;
        }

        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;

        this->parent = NULL;
        this->name = "badge";
        this->filter = filter_badge;
        this->event = event_badge;
        this->init = init_badge;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));

        ctx->base.fbuffer_len = my_arg->fbuffer_len;
        ctx->base.offset = my_arg->offset;
        ctx->base.wait = 0;
    }

err_out:
    return result;
}

int config_override(struct blinken_cfg *cfg)
{
    int updated;

    updated = 0;
    if(cfg->refresh != REFRESH){
        cfg->refresh = REFRESH;
        updated = 1;
    }

    if(cfg->strip_len != FBUFFER_LEN){
        cfg->strip_len = FBUFFER_LEN;
        updated = 1;
    }

    if(cfg->brightness != HSV_VAL_MAX){
        cfg->brightness = HSV_VAL_MAX;
        updated = 1;
    }

    return updated;
}

int create_filters(struct blinken_cfg *strip_cfg,
                         struct led_filter **root,
                         void **state)
{
    struct arg_badge badge_arg;
    int result;

    badge_arg.fbuffer_len = FBUFFER_LEN;
    badge_arg.offset = 0;

    result = init_root(&f_root, strip_cfg, false, &badge_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_root() failed\n", __func__);
        goto err_out;
    }

    result = init_air(&f_air, strip_cfg, false, &badge_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_air() failed\n", __func__);
        goto err_out;
    }

    result = filter_set_parent(&f_air, &f_root);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] filter_set_parent() failed for air\n", __func__);
        goto err_out;
    }

    result = init_ir(&f_ir, strip_cfg, false, &badge_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_ir() failed\n", __func__);
        goto err_out;
    }

    result = filter_set_parent(&f_ir, &f_air);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] filter_set_parent() failed for ir\n", __func__);
        goto err_out;
    }

    result = init_nfc(&f_nfc, strip_cfg, false, &badge_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_nfc() failed\n", __func__);
        goto err_out;
    }

    result = filter_set_parent(&f_nfc, &f_ir);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] filter_set_parent() failed for nfc\n", __func__);
        goto err_out;
    }

    result = init_badge(&f_badge, strip_cfg, false, &badge_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_badge() failed\n", __func__);
        goto err_out;
    }

    result = filter_set_parent(&f_badge, &f_nfc);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] filter_set_parent() failed for nfc\n", __func__);
        goto err_out;
    }

    badge_init();

    *root = &f_root;
    *state = NULL;

err_out:
    return result;
}

