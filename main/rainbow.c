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
#include <esp_err.h>
#include <esp_event.h>
#include <math.h>

#include "klist.h"
#include "ws2812.h"
#include "blinken.h"

#define REFRESH     50
#define STRIP_LEN   16

static const char *TAG = "RAINBOW";

enum strip_state
{
    state_rainbow,
    state_flicker,
    state_lurker,
    state_max
};

static enum strip_state rainbow_state;
static struct led_filter rainbow;
static struct led_filter fade;
static struct led_filter flicker;
static struct led_filter lurker;

struct ctx_rainbow
{
    int32_t hue_min;
    int32_t hue_max;
    int32_t hue_step;
    int32_t cycle_step;
    int32_t curr_hue;
};

void filter_rainbow(struct led_filter *this,
                    void *state_ptr,
                    hsv_value_t hsv_vals[],
                    unsigned int strip_len,
                    unsigned int offset,
                    uint64_t now)
{
    int i;
    hsv_value_t tmp_hsv;
    uint32_t tmp_hue;
    struct ctx_rainbow *ctx;
    enum strip_state *state;

    state = (enum strip_state *) state_ptr;
    ctx = (struct ctx_rainbow *) this->priv;

    run_child_filters(this, state, hsv_vals, strip_len, offset, now);

    tmp_hue = ctx->curr_hue;
    tmp_hsv.saturation = HSV_SAT_MAX;
    tmp_hsv.value = HSV_VAL_MAX;

    for(i = 0; i < strip_len; ++i){
        tmp_hsv.hue = tmp_hue;
        hsv_vals[i] = tmp_hsv;

        tmp_hue += ctx->hue_step;
        if(ctx->hue_min == HSV_HUE_MIN && ctx->hue_max == HSV_HUE_MAX){
            tmp_hue %= HSV_HUE_STEPS;
        } else {
            if(tmp_hue > ctx->hue_max){
                tmp_hue = ctx->hue_max;
                ctx->hue_step = -ctx->hue_step;
            }else if(tmp_hue < ctx->hue_min){
                tmp_hue = ctx->hue_min;
                ctx->hue_step = -ctx->hue_step;
            }
        }
    }

    ctx->curr_hue += ctx->cycle_step;
    if(ctx->hue_min == HSV_HUE_MIN && ctx->hue_max == HSV_HUE_MAX){
        ctx->curr_hue %= HSV_HUE_STEPS;
    } else {
        if(ctx->curr_hue > ctx->hue_max){
            ctx->curr_hue = ctx->hue_max;
            ctx->cycle_step = -ctx->cycle_step;
        }else if(ctx->curr_hue < ctx->hue_min){
            ctx->curr_hue = ctx->hue_min;
            ctx->cycle_step = -ctx->cycle_step;
        }
    }
}

esp_err_t init_rainbow(struct led_filter *this, struct blinken_cfg *cfg,
                        bool update, void *arg)
{
    esp_err_t result;
    struct ctx_rainbow *ctx;

    result = ESP_OK;

    if(update){
        ctx = (struct ctx_rainbow *) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        this->parent = NULL;
        this->name = "rainbow";
        this->filter = filter_rainbow;
        this->init = init_rainbow;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));
        
        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;
    }

    ctx->hue_min = HSV_HUE_MIN;
    ctx->hue_max = HSV_HUE_MAX;
    ctx->hue_step = cfg->strip_len;
    ctx->cycle_step = cfg->strip_len;

    ctx->curr_hue = min(ctx->curr_hue, ctx->hue_max);
    ctx->curr_hue = max(ctx->curr_hue, ctx->hue_min);

err_out:
    return result;
}

struct ctx_fade
{
    int32_t min;
    int32_t max;
    int32_t curr_val;
    int32_t curr_step;
};


void filter_fade(struct led_filter *this,
                 void *state_ptr,
                 hsv_value_t hsv_vals[],
                 unsigned int strip_len,
                 unsigned int offset,
                 uint64_t now)
{
    int i;
    struct ctx_fade *ctx;
    enum strip_state *state;

    state = (enum strip_state *) state_ptr;
    ctx = (struct ctx_fade *) this->priv;

    run_child_filters(this, state, hsv_vals, strip_len, offset, now);

    for(i = 0; i < strip_len; ++i){
        hsv_vals[i].value = ctx->curr_val;
    }

    if(ctx->curr_step == 0){
        return;
    }

    ctx->curr_val += ctx->curr_step;

    if(ctx->curr_val <= ctx->min){
        ctx->curr_step = -ctx->curr_step;
        ctx->curr_val = ctx->min;
    } else if(ctx->curr_val >= ctx->max){
        ctx->curr_step =  -ctx->curr_step;
        ctx->curr_val = ctx->max;
    }
}

esp_err_t init_fade(struct led_filter *this, struct blinken_cfg *cfg,
                        bool update, void *arg)
{
    esp_err_t result;
    struct ctx_fade *ctx;

    result = 0;

    if(update){
        ctx = (struct ctx_fade *) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        this->parent = NULL;
        this->name = "fade";
        this->filter = filter_fade;
        this->init = init_fade;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));
        
        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;
    }

    ctx->min = HSV_VAL_MAX / 2;
    ctx->max = HSV_VAL_MAX;
    ctx->curr_step = HSV_VAL_MAX / cfg->refresh;
    
    ctx->curr_val = max(ctx->curr_val, ctx->min);
    ctx->curr_val = min(ctx->curr_val, ctx->max);

err_out:
    return result;
}

struct arg_flicker
{
    enum strip_state next;
};

struct ctx_flicker
{
    uint32_t rate;

    unsigned int on_time;
    unsigned int off_time;
    uint64_t wait;
    enum strip_state next_state;
    bool blackout;
};

void filter_flicker(struct led_filter *this,
                    void *state_ptr,
                    hsv_value_t hsv_vals[],
                    unsigned int strip_len,
                    unsigned int offset,
                    uint64_t now)
{
    int i;
    struct ctx_flicker *ctx;
    enum strip_state *state;

    state = (enum strip_state *) state_ptr;
    ctx = (struct ctx_flicker *) this->priv;

    if(*state != state_flicker){
        run_child_filters(this, state, hsv_vals, strip_len, offset, now);
        return;
    }

    /*
     * Create an "intermittent failure" effect. Black out the strip for ~100ms,
     * then go back to seemingly normal operation for random period of time.
     * Maximum length of this period will be halved after each blackout until
     * it is <= 1. This will have the effect of   
     */
    if(ctx->wait < now){
        if(ctx->blackout){
            ctx->on_time /= 2;
            if(ctx->on_time <= 1){
                ctx->on_time = 2000;
                *state = ctx->next_state;
            }else{
                ctx->wait = now + ms_to_us(100 + (esp_random() % ctx->on_time));
            }
        } else {
            ctx->wait = now + ms_to_us(ctx->off_time);
        }
        ctx->blackout = !ctx->blackout;
    }

    if(!ctx->blackout){
        run_child_filters(this, state, hsv_vals, strip_len, offset, now);
    } else {
        for(i = 0; i < strip_len; ++i){
            hsv_vals[i].value = HSV_VAL_MIN;
        }
    }
}

int init_flicker(struct led_filter *this, struct blinken_cfg *cfg,
                    bool update, void *arg)
{
    int result;
    struct ctx_flicker *ctx;
    struct arg_flicker *my_arg = arg;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        this->parent = NULL;
        this->name = "flicker";
        this->filter = filter_flicker;
        this->init = init_flicker;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));
        
        ctx = malloc(sizeof(*ctx));
        if(ctx == NULL){
            ESP_LOGE(TAG, "[%s] malloc() failed\n", __func__);
            result = ESP_ERR_NO_MEM;
            goto err_out;
        }

        memset(ctx, 0x0, sizeof(*ctx));
        this->priv = ctx;

        if(my_arg){
            ctx->next_state = my_arg->next;
        } else {
            ctx->next_state = state_flicker;
        } 
    }


    ctx->rate = 100;
    ctx->on_time = 2000; // max time in ms between blackouts
    ctx->off_time = 50; // duration of blackout
    ctx->blackout = false;
    ctx->wait = 0;
    
err_out:
    return result;
}

enum lurker_state
{
    lurker_init,
    lurker_waking,
    lurker_searching,
    lurker_found,
    lurker_hiding,
    lurker_sleeping
};

struct arg_lurker
{
    enum strip_state prev; // the state to change into before we run
    enum strip_state next; // the state to switch to after we finished running
};

struct ctx_lurker
{
    enum lurker_state state;
    unsigned int rate;
    unsigned int brightness;
    int inc;
    int tmp_peak;
    int tmp_floor;
    int level;
    unsigned int curr_pos;
    unsigned int target_pos;
    int curr_speed;
    unsigned int jump_len;
    unsigned int jumps;
    uint64_t wait;
    uint64_t last_active;
    enum strip_state state_prev;
    enum strip_state state_next;
};

void filter_lurker(struct led_filter *this,
                void *state_ptr,
                hsv_value_t hsv_vals[],
                unsigned int strip_len,
                unsigned int offset,
                uint64_t now)
{
    int i, jump;
    struct ctx_lurker *ctx;
    enum strip_state *state;

    state = (enum strip_state *) state_ptr;
    ctx = (struct ctx_lurker *) this->priv;

    /*
     * Make sure we do not get stuck in lurker state if config is changed
     * while active.
     */
    if(ctx->rate == 0 && *state == state_lurker){
        *state = ctx->state_next;
    }

    if(*state != state_lurker){
        /* If lurker is enabled and sleeping, roll the dice once a minute. */
        if(*state != ctx->state_prev
            && ctx->rate > 0
            && ctx->last_active + ms_to_us(60000) < now)
        {
            if(esp_random() % ctx->rate == 0){
                /*
                 * Critical hit! Start waking lurker by moving to
                 * preparing state.
                 */ 
                *state = ctx->state_prev;
                ctx->state = lurker_init;
                ctx->wait = now;
            }
        }

        run_child_filters(this, state, hsv_vals, strip_len, offset, now);
        return;
    }

    /* keep rendering current state until next update is due. */
    if(ctx->wait > now){
        goto on_exit;
    }

    switch (ctx->state) {
    case lurker_init:
        ctx->brightness = HSV_VAL_MAX;
        ctx->tmp_floor = HSV_VAL_MIN;
        ctx->tmp_peak = HSV_VAL_MAX / 16;
        ctx->level = HSV_VAL_MIN;
        ctx->inc = max(1, (ctx->tmp_peak - ctx->tmp_floor) / 16);
        ctx->target_pos = ctx->curr_pos;
        ctx->state = lurker_waking;
        ctx->jumps = 5 + esp_random() % 10;
        ctx->jump_len = strip_len / 2;
        ctx->curr_speed = 1;
        ctx->wait = now + ms_to_us(600);
        /* fall through */
    case lurker_waking:
        if(ctx->level >= ctx->brightness){
            ctx->state = lurker_searching;
            ctx->wait = now + ms_to_us(2000);
        }else{
            /*
             * Make lurker "breathe" to life. Fade up to tmp_peak and back down
             * to tmp_floor. Then set tmp_floor to tmp_peak, double tmp_peak
             * and start fading up again. Repeat until tmp_peak reaches set
             * brightness.
             */ 
            ctx->level += ctx->inc;
            if(ctx->level >= ctx->tmp_peak){
                ctx->inc = -ctx->inc;
            }else if(ctx->inc < 0 && ctx->level <= ctx->tmp_floor){
                ctx->level = ctx->tmp_floor;
                ctx->tmp_floor = ctx->tmp_peak;
                ctx->tmp_peak *= 2;
                ctx->tmp_peak = min(ctx->tmp_peak, ctx->brightness);
                ctx->inc = max(1, (ctx->tmp_peak - ctx->tmp_floor) / 16);
            }
            ctx->wait = now + ms_to_us(50);
        }
        ctx->level = min(ctx->level, ctx->brightness);
        break;
    case lurker_searching:
        ESP_LOGD(TAG, "[%s] curr: %d target: %d speed: %d jumps: %d", __func__,
                    ctx->curr_pos, ctx->target_pos, ctx->curr_speed, ctx->jumps);

        /*
         * Let the lurker search for a victim to glare at along the strip.
         * We will randomly select a new target point to jump to and then move
         * the lurker along the strip with increasing speed. When the target
         * point is reached, we halve the possible jump range and select a new
         * target point. Repeat for 5-15 times, then make lurker hide again.
         */
        if(ctx->curr_pos != ctx->target_pos){
            ctx->curr_pos += ctx->curr_speed;
            ctx->curr_speed += ctx->target_pos > ctx->curr_pos ? 1 : -1;
            if(abs(ctx->target_pos - ctx->curr_pos) < abs(ctx->curr_speed)){
                ctx->curr_speed = ctx->target_pos - ctx->curr_pos;
            }
            ctx->wait = now + ms_to_us(50);
        } else {
            if(ctx->jumps == 0){
                ctx->state = lurker_found;
                ctx->wait = now + ms_to_us(2000);
            } else {
                --ctx->jumps;

                /* determine length and direction of next jump. */
                jump = 2 + esp_random() % (1 + ctx->jump_len);
                ctx->jump_len /= 2;

                if(esp_random() % 2){
                    jump = -jump;
                }
                
                /* avoid getting stuck on the ends of the strip. */
                if(ctx->curr_pos + jump < 2
                   || ctx->curr_pos + jump + 2 >= strip_len)
                {
                    jump = -jump;
                }

                /* clamp next target position valid range. */
                ctx->target_pos = max(2, ctx->curr_pos + jump);
                ctx->target_pos = min(strip_len - 2, ctx->target_pos);
                ctx->curr_speed = 0;
                
                ctx->wait = now + ms_to_us(500 + esp_random() % 500);
            }
        }
        break;
    case lurker_found:
            ctx->state = lurker_hiding;
            ctx->wait = now + ms_to_us(1000);
            ctx->inc = min(-1, -(ctx->brightness / 100));
        break;
    case lurker_hiding:
        if(ctx->level <= HSV_VAL_MIN){
            ctx->state = lurker_sleeping;
            ctx->wait = now + ms_to_us(1000);
        }else{
            ctx->level = max(HSV_VAL_MIN, (ctx->level + ctx->inc));
            ctx->wait = now + ms_to_us(10);
        }
        break;
    case lurker_sleeping:
        ctx->state = lurker_init;
        ctx->last_active = now;
        *state = ctx->state_next;
    }

on_exit:
    if(ctx->state != lurker_sleeping){
        uint32_t pos;

        for(i = 0;i < strip_len;++i){
            hsv_vals[i].value = 0;
        }

        pos = max(ctx->curr_pos, 2);
        pos = min(pos, strip_len - 2);

        hsv_vals[pos].hue = HSV_HUE_MIN;
        hsv_vals[pos].saturation = HSV_SAT_MAX;
        hsv_vals[pos].value = ctx->level;
        hsv_vals[pos - 1] = hsv_vals[pos];
        hsv_vals[pos - 1].value = hsv_vals[pos].value / 2;
        hsv_vals[pos + 1] = hsv_vals[pos];
        hsv_vals[pos + 1].value = hsv_vals[pos].value / 2;
    }

}

int init_lurker(struct led_filter *this, struct blinken_cfg *cfg, bool update, void *arg)
{
    int result;
    struct ctx_lurker *ctx;
    struct arg_lurker *my_arg = arg;

    result = 0;

    if(update){
        ctx = (typeof(ctx)) this->priv;
        update_child_filters(this, cfg, update);
    } else {
        this->parent = NULL;
        this->name = "lurker";
        this->filter = filter_lurker;
        this->init = init_lurker;
        this->deinit = filter_deinit;
        INIT_KLIST_HEAD(&(this->children));
        INIT_KLIST_HEAD(&(this->siblings));
       
        if(my_arg == NULL){
            ESP_LOGE(TAG, "[%s] Filter configuration arg missing", __func__);
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
    
        ctx->state = lurker_init;
        ctx->last_active = 0;
        ctx->wait = 0;
        ctx->state_prev = my_arg->prev;
        ctx->state_next = my_arg->next;
        rainbow_state = state_rainbow;
    }

    ctx->curr_pos = max(cfg->strip_len / 2, 2);
    ctx->curr_pos = min(ctx->curr_pos, cfg->strip_len - 2);
    ctx->rate = 10;

err_out:
    return result;
}

esp_err_t create_filters(struct blinken_cfg *strip_cfg,
                         struct led_filter **root,
                         void **state)
{
    struct arg_flicker flicker_arg;
    struct arg_lurker lurker_arg;
    int result;

    result = init_rainbow(&rainbow, strip_cfg, false, NULL);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_rainbow() failed\n", __func__);
        goto err_out;
    }

    result = init_fade(&fade, strip_cfg, false, NULL);
    if(result != 0){
        ESP_LOGE(TAG,"[%s] init_fade() failed\n", __func__);
        goto err_out;
    }

    flicker_arg.next = state_lurker;
    result = init_flicker(&flicker, strip_cfg, false, &flicker_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_flicker() failed\n", __func__);
        goto err_out;
    }

    lurker_arg.next = state_rainbow;
    lurker_arg.prev = state_flicker;
    result = init_lurker(&lurker, strip_cfg, false, &lurker_arg);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_lurker() failed\n", __func__);
        goto err_out;
    }


    filter_set_parent(&flicker, &lurker);
    filter_set_parent(&fade, &flicker);
    filter_set_parent(&rainbow, &fade);
    rainbow_state = state_rainbow;

    *root = &lurker;
    *state = &rainbow_state;

err_out:
    return result;
}

int config_override(struct blinken_cfg *cfg)
{
    int updated;

    updated = 0;
    if(cfg->refresh != REFRESH){
        ESP_LOGI(TAG, "Overriding refresh rate");
        cfg->refresh = REFRESH;
        updated = 1;
    }

    if(cfg->strip_len != STRIP_LEN){
        ESP_LOGI(TAG, "Overriding strip_len");
        cfg->strip_len = STRIP_LEN;
        updated = 1;
    }

    return updated;
}