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

#ifndef __BLINKEN_H__
#define __BLINKEN_H__

#include <esp_err.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "klist.h"
#include "ws2812.h"
#include "control.h"

#if !defined(min)
#define min(a,b)            ((a) < (b) ? (a) : (b))
#endif

#if !defined(max)
#define max(a,b)            ((a) > (b) ? (a) : (b))
#endif

#define MIN_STRIP_LEN       0
#define MAX_STRIP_LEN       CONFIG_WS2812_MAX_LEDS
#define DEF_STRIP_LEN       MAX_STRIP_LEN
#define MIN_STRIP_REFRESH   1
#define MAX_STRIP_REFRESH   100
#define DEF_STRIP_REFRESH   25

#define ms_to_us(ms) ((ms) * 1000)

struct blinken_cfg {
    uint32_t strip_len;
    enum pixel_type type;
    uint32_t refresh; 
    uint32_t brightness;
};

esp_err_t blinken_get_config(struct blinken_cfg *cfg);
esp_err_t blinken_set_config(struct blinken_cfg *cfg);

struct led_filter;

typedef void (*filter_fn)(struct led_filter *this, void *state,
                            hsv_value_t hsv_vals[],  unsigned int num_vals,
                            unsigned int offset, uint64_t time);

typedef int (*event_fn)(struct led_filter *this, void *state,
                            struct ctrl_event *event);

typedef int (*init_fn)(struct led_filter *this, struct blinken_cfg *cfg,
                       bool update, void *arg);

typedef void (*deinit_fn)(struct led_filter *this);

struct led_filter
{
    char *name;
    struct led_filter *parent;
    struct klist_head siblings;
    struct klist_head children;
    filter_fn filter;
    event_fn event;
    init_fn init;
    deinit_fn deinit;
    void *priv;
};

esp_err_t filter_set_parent(struct led_filter *this,
                            struct led_filter *parent);

esp_err_t filter_unset_parent(struct led_filter *this);

void run_child_filters(struct led_filter *this,
                        void *state,
                        hsv_value_t hsv_vals[],
                        unsigned int num_vals,
                        unsigned int offset,
                        uint64_t time);

int forward_event(struct led_filter *this, void *state, struct ctrl_event *evt);

esp_err_t create_filters(struct blinken_cfg *cfg,
                         struct led_filter **root,
                         void **state);

void update_child_filters(struct led_filter *this,
                          struct blinken_cfg *cfg,
                          bool update);

void filter_deinit(struct led_filter *this);
int config_override(struct blinken_cfg *cfg);

typedef int (*event_cb_fn)(struct ctrl_event *event, void *priv);
esp_err_t register_event_cb(event_cb_fn func, void *priv);


#endif
