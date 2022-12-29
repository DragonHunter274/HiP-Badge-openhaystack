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

#if defined(CONFIG_BLINKEN_GAS)
#include "gassens.h"
#endif // defined(CONFIG_BLINKEN_GAS)

#include "klist.h"
#include "ws2812.h"
#include "blinken.h"
#include "control.h"

#include "gamma_23.h"
#define gamma_tbl   gamma_23

static const char *TAG = "BLINK";

struct blinken_cfg *strip_cfg;
static ws2812_t *ws2812_cfg = NULL;
static TimerHandle_t refresh_timer = NULL;
static SemaphoreHandle_t refresh_sema;

static SemaphoreHandle_t cb_sema;
KLIST_HEAD(cb_list);
struct event_cb {
    struct klist_head list;
    event_cb_fn func;
    void *priv;
};

hsv_value_t hsv_buffer[CONFIG_WS2812_MAX_LEDS];

esp_err_t filter_set_parent(struct led_filter *child,
                            struct led_filter *parent)
{
    esp_err_t result;

    result = ESP_OK;

    if(child->parent != NULL){
        result = ESP_ERR_INVALID_STATE;
        goto on_exit;
    }

    child->parent = parent;
    klist_add_tail(&child->siblings, &parent->children);

on_exit:
    return result;
}

esp_err_t filter_unset_parent(struct led_filter *child)
{
    esp_err_t result;

    result = ESP_OK;
    if(child->parent == NULL || klist_empty(&child->siblings)){
        result = ESP_ERR_INVALID_STATE;
        goto on_exit;
    }

    klist_del_init(&child->siblings);
    child->parent = NULL;

on_exit:
    return result;
}

void update_child_filters(struct led_filter *this,
                          struct blinken_cfg *cfg,
                          bool update)
{
    struct led_filter *child;

    klist_for_each_entry(child, &(this->children), siblings){
        child->init(child, cfg, update, NULL);
    }
}

void run_child_filters(struct led_filter *this,
                        void *state_ptr,
                        hsv_value_t leds[],
                        unsigned int num_leds,
                        unsigned int offset,
                        uint64_t now)
{
    struct led_filter *child;

    klist_for_each_entry(child, &(this->children), siblings){
        child->filter(child, state_ptr, leds, num_leds, offset, now);
    }
}

struct strip_handler
{
    void *state_ptr;
    struct led_filter *filter_root;
    hsv_value_t *hsv_vals;
    volatile size_t strip_len;
    volatile uint32_t brightness;
};

struct strip_handler handler;

SemaphoreHandle_t cfg_sema = NULL;

void filter_deinit(struct led_filter *this)
{
    struct led_filter *child, *tmp;
    void *priv;

    klist_for_each_entry_safe(child, tmp, &(this->children), siblings){
        klist_del_init(&child->siblings);
        child->deinit(child);
    }

    this->parent = NULL;

    if(this->priv != NULL){
        priv = this->priv;
        this->priv = NULL;

        free(priv);
    }
    this->filter = NULL;
    this->name = NULL;
    this->init = NULL;
}

int forward_event(struct led_filter *this, void *state, struct ctrl_event *evt)
{
    struct led_filter *child;
    int handled;

    handled = 0;
    klist_for_each_entry(child, &(this->children), siblings){
        if(child->event != NULL){
            handled = child->event(child, state, evt);
        }

        if(handled != 0){
            break;
        }
    }

    return handled;
}

/* let other modules register call-back functions for control events. */
esp_err_t register_event_cb(event_cb_fn func, void *priv)
{
    esp_err_t result;
    BaseType_t status;
    struct event_cb *cb;

    result = ESP_OK;
    cb = NULL;

    if(func == NULL){
        ESP_LOGE(TAG, "Refusing to register NULL call-back fn");
        result = ESP_ERR_INVALID_ARG;
        goto err_out;
    }

    cb = malloc(sizeof(*cb));
    if(cb == NULL){
        ESP_LOGE(TAG, "Out of memory for call-back registration");
        result = ESP_ERR_NO_MEM;
        goto err_out;
    }

    /* make sure we have exclusive access to the call-back list */
    status = xSemaphoreTake(cb_sema, portMAX_DELAY);
    if(status != pdTRUE){
        ESP_LOGE(TAG, "[%s] Timeout waiting for call-back sema.\n", __func__);
        result = ESP_ERR_TIMEOUT;
        goto err_out;
    }

    /* initialise call-back entry struct and append it to the list */
    INIT_KLIST_HEAD(&cb->list);
    cb->func = func;
    cb->priv = priv;
    klist_add_tail(&cb->list, &cb_list);

    /* release lock on call-back list. */
    (void) xSemaphoreGive(cb_sema);

err_out:
    if(result != ESP_OK && cb != NULL){
        free(cb);
    }

    return result;
}

/* run registered call-backs for unhandled events. */
static int run_event_cb(struct ctrl_event *evt)
{
    int result;
    BaseType_t status;
    struct event_cb *cb;

    result = 0;
    cb = NULL;

    /* get exclusive access to list of call-backs. */
    status = xSemaphoreTake(cb_sema, portMAX_DELAY);
    if(status != pdTRUE){
        ESP_LOGE(TAG, "[%s] Timeout waiting for call-back sema.\n", __func__);
        result = ESP_ERR_TIMEOUT;
        goto err_out;
    }

    /* execute call-backs in list until one was able to handle the event. */
    klist_for_each_entry(cb, &cb_list, list){
        result = cb->func(evt, cb->priv);
        if(result != 0){
            /* call-back was able to handle event. Exit loop.*/
            break;
        }
    }

    /* release lock on call-back list.*/
    (void) xSemaphoreGive(cb_sema);

err_out:
    return result;

}
int __attribute__((weak)) config_override(struct blinken_cfg *cfg)
{
    ESP_LOGE(TAG, "[%s] Called.", __func__);

    return 0;
}

static esp_err_t init_handler(struct strip_handler *this,
                              struct blinken_cfg *cfg,
                              ws2812_t *ws2812,
                              bool update)
{
    BaseType_t status;
    struct led_filter *filter;
    esp_err_t result;

    result = ESP_OK;

    (void) config_override(cfg);

    if(cfg->strip_len > MAX_STRIP_LEN){
        ESP_LOGE(TAG, "[%s] Invalid strip_len found: %d.",
                    __func__, cfg->strip_len);
        cfg->strip_len = MAX_STRIP_LEN;
    }

    if(cfg->refresh < MIN_STRIP_REFRESH){
        ESP_LOGE(TAG, "[%s] Refresh rate too low: %d.",
                    __func__, cfg->refresh);
        cfg->refresh = MIN_STRIP_REFRESH;
    }

    if(cfg->refresh > MAX_STRIP_REFRESH){
        ESP_LOGE(TAG, "[%s] Refresh rate too high: %d.",
                    __func__, cfg->refresh);
        cfg->refresh = MAX_STRIP_REFRESH;
    }

    if(cfg->brightness > HSV_VAL_MAX){
        ESP_LOGE(TAG, "[%s] Brightness too high: %d.",
                    __func__, cfg->brightness);
        cfg->brightness = HSV_VAL_MAX;
    }

    this->strip_len = cfg->strip_len;
    this->brightness = cfg->brightness;
    this->hsv_vals = hsv_buffer;

    result = ws2812_set_len(ws2812, this->strip_len);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] ws2812_set_len() failed.", __func__);
        goto err_out;
    }

    status = xTimerChangePeriod(refresh_timer,
                                pdMS_TO_TICKS(1000) / cfg->refresh,
                                portMAX_DELAY);
    if(status == pdFAIL){
        ESP_LOGE(TAG, "[%s] Setting refresh rate failed.", __func__);
        result = ESP_FAIL;
        goto err_out;
    }

    if(update){
        result = this->filter_root->init(this->filter_root, cfg, true, NULL);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] updating filter %s failed.\n", __func__,
                    filter->name != NULL ? filter->name : "unknown");
            goto err_out;
        }
    }

err_out:
    return result;
}

esp_err_t blinken_get_config(struct blinken_cfg *cfg)
{
    BaseType_t status;

    if(cfg == NULL){
        return ESP_ERR_INVALID_ARG;
    }

    if(strip_cfg == NULL){
        return ESP_ERR_INVALID_STATE;
    }

    status = xSemaphoreTake(cfg_sema, portMAX_DELAY);
    if(status != pdTRUE){
        ESP_LOGE(TAG, "[%s] Timeout waiting for config sema.\n", __func__);
        return ESP_ERR_TIMEOUT;
    }

    memmove(cfg, strip_cfg, sizeof(*cfg));

    xSemaphoreGive(cfg_sema);

    return ESP_OK;
}

esp_err_t blinken_set_config(struct blinken_cfg *cfg)
{
    esp_err_t result;
    BaseType_t status;

    result = 0;
    if(cfg == NULL){
        result = ESP_ERR_INVALID_ARG;
        goto err_out;
    }

    status = xSemaphoreTake(cfg_sema, portMAX_DELAY);
    if(status != pdTRUE){
        ESP_LOGE(TAG, "[%s] Timeout waiting for config sema.\n", __func__);
        result = ESP_ERR_TIMEOUT;
        goto err_out;
    }

    result = init_handler(&handler, cfg, ws2812_cfg, true);
    if(result == ESP_OK){
        memmove(strip_cfg, cfg, sizeof(*strip_cfg));
    }

    xSemaphoreGive(cfg_sema);

err_out:
    return result;
}

static void timer_cb(TimerHandle_t timer __attribute__((unused)))
{
    (void) xSemaphoreGive(refresh_sema);
}

void run_strip(void)
{
    QueueHandle_t evt_queue;
    struct ctrl_event evt;
    struct led_filter *root;
    tx_buffer_t *buffer;
    unsigned int idx, brightness;
    uint64_t now;
    int evt_handled;
    int result;
    BaseType_t status;

    strip_cfg = calloc(1, sizeof(*strip_cfg));
    if(strip_cfg == NULL){
        ESP_LOGE(TAG, "[%s] Out of memory.", __func__);
        abort();
    }

    strip_cfg->strip_len = DEF_STRIP_LEN;
    strip_cfg->refresh = DEF_STRIP_REFRESH;
    strip_cfg->brightness = HSV_VAL_MAX / 4;
    strip_cfg->type =
#if defined(CONFIG_BLINKEN_TYPE_RGBW)
        pixel_rgbw;
#elif defined(CONFIG_BLINKEN_TYPE_RGB)
        pixel_rgb;
#elif defined(CONFIG_BLINKEN_TYPE_GRB)
        pixel_grb;
#endif

    ws2812_cfg = ws2812_init(CONFIG_WS2812_MAX_LEDS, strip_cfg->type);
    if(ws2812_cfg == NULL){
        ESP_LOGE(TAG, "[%s] ws2812_init() failed\n", __func__);
        goto err_out;
    }

    result = init_handler(&handler, strip_cfg, ws2812_cfg, false);
    if(result != 0){
        ESP_LOGE(TAG, "[%s] init_handler() failed\n", __func__);
        goto err_out;
    }

    result = create_filters(strip_cfg, &root, &handler.state_ptr);
    if(result != 0 || root == NULL){
        ESP_LOGE(TAG, "[%s] create_filters() failed\n", __func__);
        goto err_out;
    }

    evt_queue = blinken_ctrl_get_queue();
    if(evt_queue == NULL){
        ESP_LOGE(TAG, "[%s] blinken_ctrl_get_queue() failed\n", __func__);
        goto err_out;
    }

    handler.filter_root = root;
    while(1){
        status = xSemaphoreTake(cfg_sema, portMAX_DELAY);
        if(status != pdTRUE){
            ESP_LOGE(TAG, "[%s] timeout waiting for config sema\n", __func__);
            goto err_out;
        }

        /* handle events in event queue. */
        while(xQueueReceive(evt_queue, &evt, 0) == pdTRUE){
            evt_handled = 0;

            /* hand event to filter stack first. */
            if(root->event != NULL){
                evt_handled = root->event(root, handler.state_ptr, &evt);
            }

            /* if filter stack could not handle it, check if it is a
             * light level event. If so, adjust the brightness. */
            if(evt_handled == 0){
                switch(evt.event){
                case EVNT_VOLUP:
                    strip_cfg->brightness += HSV_VAL_MAX / 20;
                    if(strip_cfg->brightness > HSV_VAL_MAX){
                        strip_cfg->brightness = HSV_VAL_MAX;
                    }
                    evt_handled = 1;
                    break;
                case EVNT_VOLDOWN:
                    if(strip_cfg->brightness >= HSV_VAL_MAX / 20){
                        strip_cfg->brightness -= HSV_VAL_MAX / 20;
                    } else {
                        strip_cfg->brightness = 0;
                    }
                    evt_handled = 1;
                    break;
                default:
                    break;
                }
            }

            /* if the event is still unhandled, hand it to the registered
             * call-backs. */
            if(evt_handled == 0){
                evt_handled = run_event_cb(&evt);
            }

            if(evt_handled == 0){
                ESP_LOGW(TAG, "[%s] Unhandled control event 0x%x.",
                    __func__, evt.event);
            }
        }

        /* Get current timestamp. */
        /* Fixme: Maybe use the time of the next refresh instead. */
        now = esp_timer_get_time();

        /* Call filter chain to generate next "frame". */
        root->filter(root, handler.state_ptr, handler.hsv_vals,
                        handler.strip_len, 0, now);

        /*
         * Copy current brightness value so we can release the config sema
         * before doing the probably lengthy brightness correction.
         */
        brightness = strip_cfg->brightness;

        xSemaphoreGive(cfg_sema);

        /* Adjust brightness. */
        if(brightness != HSV_VAL_MAX){
            for(idx = 0; idx < handler.strip_len; ++idx){
                handler.hsv_vals[idx].value = handler.hsv_vals[idx].value * brightness / HSV_VAL_MAX;
            }
        }

#if 1
        /* Do gamma correction. */
        for(idx = 0; idx < handler.strip_len; ++idx){
            /*
             * LED driver will only use the upper 8 bit, so round to nearest
             * downscaled value and apply gamma table.
             */
            handler.hsv_vals[idx].value = SCALE_UP(gamma_tbl[SCALE_DOWN_ROUND(handler.hsv_vals[idx].value)]);
#endif
        }

        /* Prepare bitstream from HSV data. */
        result = ws2812_prepare(ws2812_cfg, handler.hsv_vals,
                                handler.strip_len, &buffer);
        if(result != ESP_OK){
            ESP_LOGW(TAG, "[%s] ws2812_prepare() failed.", __func__);
            continue;
        }

        /*
         * Wait for the refresh timer to trigger before sending the data
         * to the LED strip.
         */
        status = xSemaphoreTake(refresh_sema, portMAX_DELAY);
        if(status != pdTRUE){
            ESP_LOGE(TAG, "[%s] Timeout waiting for refresh sema.\n", __func__);
            continue;
        }

        ws2812_send(buffer);
    }

err_out:
    while(1){
        vTaskDelay(1000);
    }
}

void app_main(void)
{
    int result;

    ESP_LOGD(TAG, "[%s] Called\n", __func__);

    cfg_sema = xSemaphoreCreateMutex();
    if(cfg_sema == NULL){
        ESP_LOGE(TAG, "[%s] Creating cfg_sema failed.", __func__);
        abort();
    }

    cb_sema = xSemaphoreCreateBinary();
    if(cb_sema == NULL){
        ESP_LOGE(TAG, "[%s] Mutex creation failed\n", __func__);
        abort();
    }
    (void) xSemaphoreGive(cb_sema);

    refresh_sema = xSemaphoreCreateBinary();
    if(refresh_sema == NULL){
        ESP_LOGE(TAG, "[%s] Mutex creation failed\n", __func__);
        abort();
    }
    (void) xSemaphoreGive(refresh_sema);

    refresh_timer = xTimerCreate("Blinken_Timer",
                                 pdMS_TO_TICKS(1000) / DEF_STRIP_REFRESH,
                                 pdTRUE, NULL, timer_cb);

    if(refresh_timer == NULL){
        ESP_LOGE(TAG, "[%s] Refresh timer creation failed.", __func__);
        abort();
    }

    result = esp_event_loop_create_default();
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_event_create_default() failed.", __func__);
        abort();
    }

    memset(&handler, 0x0, sizeof(handler));
    blinken_ctrl_start();

#if defined(CONFIG_BLINKEN_GAS)
    start_gas_sensor();
#endif

    run_strip();
}
