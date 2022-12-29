/**
 * ESP32 Blinkenlights.
 * Copyright (C) 2019-2022  Tido Klaassen <tido_blinken@4gh.eu>
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_err.h>
#include <esp_event.h>
#include <math.h>

#include "klist.h"
#include "ws2812.h"
#include "blinken.h"

static const char *TAG = "EYES";

#define REFRESH 50

#if 1 && !defined(ESP_LOG_DEBUG)
#define ESP_LOG_DEBUG   1
#define ESP_LOG_INFO    1
#define ESP_LOG_ERROR   1
#define ESP_LOG_WARN    1
#define ESP_LOG_VERBOSE 1
#endif

struct led_filter eyes;

/*
 * "Eye" patterns for a 7 element round module. LED0 is at position 0, 1-6 are
 * placed clockwise around it: 
 *
 *       1 2
 *     6  0  3
 *       5 4
 */

/* "Normal" eye. Cyan pupil, read iris. */
hsv_value_t eye_normal[] = {
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_CYAN },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
};

/* "Squinting" eye. Just a red line. */
hsv_value_t eye_squint[] = {
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = 0, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = 0, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = 0, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = 0, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
};

/* Eye with a gradient around the iris. Useful for a rotating animation. */
hsv_value_t eye_gradient[] = {
        { .value = HSV_VAL_MAX / 4,  .saturation = HSV_SAT_MAX, .hue = HSV_CYAN },
        { .value = HSV_VAL_MAX,      .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX / 2,  .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX / 4,  .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX / 8,  .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX / 16, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
        { .value = HSV_VAL_MAX / 32, .saturation = HSV_SAT_MAX, .hue = HSV_RED },
};

struct arg_eyes
{
    unsigned int left_off;
    unsigned int right_off;
};

enum air_qual {
    air_normal,
    air_good,
    air_bad,
};

struct ctx_eyes
{
    unsigned int left_off;
    unsigned int right_off;
    unsigned int ticks;
    enum air_qual air;
    uint64_t wait;
};

void rotate_eye(hsv_value_t hsv_vals[],
                hsv_value_t eye_vals[],
                unsigned int shift,
                bool mirror,
                enum air_qual air)
{
    unsigned int idx;
    uint16_t color;

    hsv_vals[0] = eye_vals[0];

    switch(air){
    case air_good:
        color = HSV_GREEN;
        break;
    case air_normal:
        color = HSV_YELLOW;
        break;
    case air_bad:
        color = HSV_RED;
        break;
    default:
        color = HSV_BLUE;
    }

    if(!mirror){
        for(idx = 0; idx < 6; ++idx){
            hsv_vals[idx + 1] = eye_vals[((idx + shift) % 6) + 1];
            hsv_vals[idx + 1].hue = color;
        }
    } else {
        for(idx = 0; idx < 6; ++idx){
            hsv_vals[(7 - idx) % 6 + 1] = eye_vals[((idx + shift) % 6) + 1];
            hsv_vals[(7 - idx) % 6 + 1].hue = color;
        }
    }
}

void filter_eyes(struct led_filter *this,
                 void *state_ptr,
                 hsv_value_t hsv_vals[],
                 unsigned int strip_len,
                 unsigned int offset,
                 uint64_t now)
{
    struct ctx_eyes *ctx;
    unsigned int shift;

    ctx = (struct ctx_eyes *) this->priv;

    run_child_filters(this, state_ptr, hsv_vals, strip_len, offset, now);

    shift = ctx->ticks % 6;

    if(offset <= ctx->left_off
       && offset + strip_len >= ctx->left_off + ARRAY_SIZE(eye_gradient))
    {
        rotate_eye(&(hsv_vals[ctx->left_off - offset]), eye_gradient, shift, false, ctx->air);
    }

    if(offset <= ctx->right_off
       && offset + strip_len >= ctx->right_off + ARRAY_SIZE(eye_gradient))
    {
        rotate_eye(&(hsv_vals[ctx->right_off - offset]), eye_gradient, shift, true, ctx->air);
    }

    if(ctx->wait <= now){
        ++ctx->ticks;
        ctx->wait = now + ms_to_us(50);
    }
}

int event_eyes(struct led_filter *this, void *state_ptr, struct ctrl_event *evt)
{
    unsigned int idx;
    enum strip_state *state;
    struct ctx_eyes *ctx;
    int handled;

    state = (enum strip_state *) state_ptr;
    ctx = (struct ctx_eyes *) this->priv;

    handled = 0;
    switch(evt->event){
    case EVNT_AIR_GOOD:
        ctx->air = air_good;
        handled = 1;
        break;
    case EVNT_AIR_NORMAL:
        ctx->air = air_normal;
        handled = 1;
        break;
    case EVNT_AIR_BAD:
        ctx->air = air_bad;
        handled = 1;
        break;
    default:
        break;
    }

    if(handled == 0){
        handled = forward_event(this, state_ptr, evt);
    }

    return handled;
}

esp_err_t init_eyes(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    struct ctx_eyes *ctx;
    struct arg_eyes *my_arg = arg;
    esp_err_t result;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;

        this->parent = NULL;
        this->name = "eyes";
        this->filter = filter_eyes;
        this->init = init_eyes;
        this->deinit = filter_deinit;
        this->event = event_eyes;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));

        if(my_arg){
            ctx->left_off = my_arg->left_off;
            ctx->right_off = my_arg->right_off;
        } else {
            ctx->left_off = 0;
            ctx->right_off = ARRAY_SIZE(eye_normal);
        }
    }

    ctx->wait = 0;
    ctx->ticks = 0;
    ctx->air = air_normal;

err_out:
    return result;
}

int config_override(struct blinken_cfg *cfg)
{
    int updated;

    updated = 0;
    if(cfg->refresh != REFRESH){
        ESP_LOGE(TAG, "Overriding refresh rate");
        cfg->refresh = REFRESH;
        updated = 1;
    }

    if(cfg->strip_len != 2 * ARRAY_SIZE(eye_normal)){
        ESP_LOGE(TAG, "Overriding strip_len");
        cfg->strip_len = 2 * ARRAY_SIZE(eye_normal);
        updated = 1;
    }

    return updated;
}


esp_err_t create_filters(struct blinken_cfg *strip_cfg,
                         struct led_filter **root,
                         void **state)
{
    struct arg_eyes arg;
    int result;

    arg.left_off = 0;
    arg.right_off = ARRAY_SIZE(eye_normal);

    result = init_eyes(&eyes, strip_cfg, false, &arg);
    if(result != 0){
        printf("[%s] init_eye() failed\n", __func__);
        goto err_out;
    }

    *root = &eyes;
    *state = NULL;

err_out:
    return result;
}

