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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "kutils.h"
#include "ws2812.h"
#include "blinken.h"

#if 0 && !defined(ESP_LOG_DEBUG)
#define ESP_LOG_DEBUG   1
#define ESP_LOG_INFO    1
#define ESP_LOG_ERROR   1
#define ESP_LOG_WARN    1
#define ESP_LOG_VERBOSE 1
#endif

static const char *TAG = "WS2812";

#define SCLK_FREQ       2500000 // four "bits per bit" -> 800kHz

/* Events to signal completion of DMA transfer */
#define BIT_START       (1 << 0)
#define BIT_DONE        (1 << 1)

/* wake up waiting tasks when SPI transfer is complete */
static void IRAM_ATTR trans_done_cb(spi_transaction_t *trans)
{
    tx_buffer_t *buffer;

    buffer = container_of(trans, tx_buffer_t, trans);

    (void) xQueueSendFromISR(buffer->cfg->free_queue, &buffer, NULL);
}

/* stolen from http://www.vagrearg.org/content/hsvrgb */
static void hsv2rgb(hsv_value_t *hsv, rgb_value_t *rgb, enum pixel_type type)
{
    uint8_t sec, hue, sat, val, base;
    uint32_t slope;
    uint16_t tmp;        // Intermediate result

    sec = hsv->hue >> HSV_SEXTANT_SHIFT;
    hue = hsv->hue & 0xff;
    sat = SCALE_DOWN(hsv->saturation);
    val = SCALE_DOWN(hsv->value);

    /* Take shortcut for pure grey. */
    if(sat == 0){
        if(type == pixel_rgbw){
            rgb->white = val;
            val = 0;
        }

        rgb->red = rgb->green = rgb->blue = val;
        return;
    }

    if(sec > 5){
        sec %= 6;
    }

    /*
     * base white level: value * (1.0 - saturation)
     * --> (val * (255 - sat) + error_corr + 1) / 256
     */
    tmp = val * (255 - sat);
    tmp += 1;               // Error correction
    tmp += tmp >> 8;        // Error correction
    base = tmp >> 8;

    if(sec % 2 == 0){
        /* upward slope */
        slope = val * (uint32_t)((255 << 8) - (uint16_t)(sat * (256 - hue)));
    } else {
        /* downward slope */
        slope = val * (uint32_t)((255 << 8) - (uint16_t)(sat * hue));
    }

    slope += slope >> 8;    // Error correction
    slope += val;           // Error correction
    slope >>= 16;

    /* move white component to separate led if pixel supports it. */
    if(type == pixel_rgbw){
        rgb->white = base;
        val -= base;
        slope -= base;
    }

    switch(sec){
    case 0:
        rgb->red = val;
        rgb->green = slope;
        rgb->blue = base;
        break;
    case 1:
        rgb->red = slope;
        rgb->green = val;
        rgb->blue = base;
        break;
    case 2:
        rgb->red = base;
        rgb->green = val;
        rgb->blue = slope;
        break;
    case 3:
        rgb->red = base;
        rgb->green = slope;
        rgb->blue = val;
        break;
    case 4:
        rgb->red = slope;
        rgb->green = base;
        rgb->blue = val;
        break;
    case 5:
        rgb->red = val;
        rgb->green = base;
        rgb->blue = slope;
        break;
    }
}

void rgb2hsv(rgb_value_t *rgb, hsv_value_t *hsv)
{
    uint16_t r, g, b, hue, sat, val, chr, min, max;
    int32_t tmp;        // Intermediate result

    r = SCALE_UP(rgb->red)   / 255;
    g = SCALE_UP(rgb->green) / 255;
    b = SCALE_UP(rgb->blue)  / 255;
    min = min(r, g);
    min = min(min, b);
    max = max(r, g);
    max = max(max, b);

    val = max;
    chr = max - min;

    if(chr == 0){
        hue = 0;
    } else if(max == r){
        tmp = SCALE_UP(g - b) / chr;
        if(tmp < 0){
            hue = HSV_HUE_STEPS;
        } else {
            hue = HSV_HUE_MIN;
        }
        hue += SCALE_DOWN(tmp * HSV_HUE_SEXTANT);
    } else if(max == g){
        tmp = SCALE_UP(b - r) / chr;
        hue = HSV_GREEN + SCALE_DOWN(tmp * HSV_HUE_SEXTANT);
    } else {
        tmp = SCALE_UP(r - g) / chr;
        hue = HSV_BLUE + SCALE_DOWN(tmp * HSV_HUE_SEXTANT);
    }

    if(val == 0){
        sat = 0;
    } else {
        sat = SCALE_UP(chr) / val;
    }

    hsv->hue = hue;
    hsv->value = SCALE_DOWN(HSV_VAL_MAX * val);
    hsv->saturation = SCALE_DOWN(HSV_SAT_MAX * sat);
}

/* convert an RGB byte into the data stream with 2 bits per byte */
static uint8_t *rgb2pwm(uint8_t *dst, const uint8_t colour)
{
    unsigned int cnt;
    uint32_t data = colour;

    for(cnt = 0; cnt < 4; ++cnt){
        switch (data & 0xC0) {
        case 0x00:
            *dst = WS_BITS_00;
            break;
        case 0x40:
            *dst = WS_BITS_01;
            break;
        case 0x80:
            *dst = WS_BITS_10;
            break;
        case 0xC0:
            *dst = WS_BITS_11;
            break;
        }

        dst++;
        data <<= 2;
    }

    return dst;
}

esp_err_t ws2812_send(tx_buffer_t *buffer)
{
    ws2812_t *cfg;
    spi_transaction_t *trans;
    esp_err_t result;

    result = 0;
    if(buffer == NULL){
        ESP_LOGE(TAG, "[%s] DMA buffer invalid\n", __func__);
        result = ESP_FAIL;
        goto err_out;
    }
    trans = &(buffer->trans);
    memset(trans, 0x0, sizeof(*trans));
    
    cfg = buffer->cfg;
    trans->tx_buffer = buffer->buff;
    trans->length = ws2812_dmabuf_len(cfg->type, cfg->strip_len) * 8;
    trans->user = buffer;
    result = spi_device_queue_trans(cfg->spi_master, trans, portMAX_DELAY);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] spi_device_queue_trans() failed: %s.",
                __func__, esp_err_to_name(result));
        goto err_out;
    }

err_out:
    return result;
}

esp_err_t ws2812_prepare(ws2812_t *cfg, hsv_value_t hsv_values[],
                         size_t strip_len, tx_buffer_t **buffer)
{
    tx_buffer_t *tx_buff;
    uint8_t *bufp;
    size_t len;
    unsigned int i;
    rgb_value_t rgb;
    BaseType_t status;
    esp_err_t result;

    result = ESP_OK;

    status = xQueueReceive(cfg->free_queue, &tx_buff, portMAX_DELAY);
    if(status != pdTRUE){
        ESP_LOGE(TAG, "[%s] Error fetching buffer.", __func__);
        result = ESP_ERR_NO_MEM;
        goto err_out;
    }

    /* make sure that we do not exceed the buffer */
    len = min(strip_len, cfg->strip_len);

    bufp = tx_buff->buff;

    /* copy pixel data into DMA buffer */
    for(i = 0; i < len; ++i){
        hsv2rgb(&hsv_values[i], &rgb, cfg->type);

        switch(cfg->type){
        case pixel_grb:
            bufp = rgb2pwm(bufp, rgb.green);
            bufp = rgb2pwm(bufp, rgb.red);
            bufp = rgb2pwm(bufp, rgb.blue);
            break;
        case pixel_rgb:
            bufp = rgb2pwm(bufp, rgb.red);
            bufp = rgb2pwm(bufp, rgb.green);
            bufp = rgb2pwm(bufp, rgb.blue);
            break;
        case pixel_rgbw:
            bufp = rgb2pwm(bufp, rgb.red);
            bufp = rgb2pwm(bufp, rgb.green);
            bufp = rgb2pwm(bufp, rgb.blue);
            bufp = rgb2pwm(bufp, rgb.white);
            break;
        default:
            ESP_LOGE(TAG, "Undefined Pixel Type.");
            (void) xQueueSend(cfg->free_queue, &(tx_buff), portMAX_DELAY);
            result = ESP_ERR_INVALID_ARG;
            goto err_out;
        }
    }

    /* turn unused pixels at end of strip off */
    if(cfg->strip_len > len){
        memset(bufp, WS_BITS_00, ws2812_data_len(cfg->type, cfg->strip_len - len));
        bufp += ws2812_data_len(cfg->type, cfg->strip_len - len);
    }

    /* add reset pulse */
    memset(bufp, WS_BITS_RESET, WS2812_RESET_LEN);

    tx_buff->cfg = cfg;
    *buffer = tx_buff;

err_out:
    return result;
}

ws2812_t *ws2812_init(uint16_t strip_len, enum pixel_type type)
{
    unsigned int i;
    esp_err_t result;
    ws2812_t *cfg = NULL;
    gpio_config_t gpio_cfg;
    spi_bus_config_t buscfg = {
         .miso_io_num = -1,
         .mosi_io_num = CONFIG_WS2812_DATA_PIN,
         .sclk_io_num = -1,
         .quadwp_io_num = -1,
         .quadhd_io_num = -1,
         .max_transfer_sz = ws2812_dmabuf_len(type, CONFIG_WS2812_MAX_LEDS),
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SCLK_FREQ,
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 2,
        .post_cb = trans_done_cb,
    };

    result = 0;
    memset(&gpio_cfg, 0x0, sizeof(gpio_cfg));
    gpio_cfg.pin_bit_mask = (1LL << CONFIG_WS2812_DATA_PIN);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;

    result = gpio_config(&gpio_cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_config() failed for MOSI", __func__);
        goto err_out;
    }

    cfg = calloc(1, sizeof(*cfg));
    if(cfg == NULL){
        ESP_LOGE(TAG, "[%s] malloc for cfg failed\n", __func__);
        result = ESP_ERR_NO_MEM;
        goto err_out;
    }

    cfg->type = type;

    cfg->lock = xSemaphoreCreateMutex();
    if(cfg->lock == NULL){
        ESP_LOGE(TAG, "[%s] Creating config lock failed.", __func__);
        result = ESP_FAIL;
        goto err_out;
    }

    cfg->free_queue = xQueueCreate(NUM_DMA_BUFFS, sizeof(tx_buffer_t *));
    if(cfg->free_queue == NULL){
        ESP_LOGE(TAG, "[%s] Error creating free_queue.", __func__);
        result = ESP_FAIL;
        goto err_out;
    }

    result = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] spi_bus_initialize() failed: %s.",
                __func__, esp_err_to_name(result));
        goto err_out;
    }

    result = spi_bus_add_device(SPI2_HOST, &devcfg, &(cfg->spi_master));
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] spi_bus_add_device() failed: %s.",
                __func__, esp_err_to_name(result));
        goto err_out;
    }

#if defined(CONFIG_WS2812_INVERT_SPI)
    GPIO.func_out_sel_cfg[CONFIG_WS2812_DATA_PIN].inv_sel = 1;
#endif

    result = ws2812_set_len(cfg, strip_len);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] ws2812_set_len() failed: %s.",
                __func__, esp_err_to_name(result));
        goto err_out;
    }

err_out:
    if(result != 0 && cfg != NULL){
        if(cfg->lock != NULL){
            vSemaphoreDelete(cfg->lock);
        }

        if(cfg->events != NULL){
            vEventGroupDelete(cfg->events);
        }

        if(cfg->free_queue != NULL){
            vQueueDelete(cfg->free_queue);
        }

        for(i = 0; i < NUM_DMA_BUFFS; ++i){
            if(cfg->tx_buffers[i].buff != NULL){
                free(cfg->tx_buffers[i].buff);
            }
        }

        if(cfg->spi_master != NULL){
            (void) spi_bus_remove_device(cfg->spi_master);
        }

        if(cfg != NULL){
            free(cfg);
        }
        cfg = NULL;
    }
    
    return cfg;
}

esp_err_t ws2812_set_len(ws2812_t *cfg, uint16_t strip_len)
{
    unsigned int i;
    esp_err_t result;
    BaseType_t status;
    tx_buffer_t *tx_buff;
    uint32_t reset_off;

    result = ESP_OK;

    if(cfg == NULL){
        ESP_LOGE(TAG, "[%s] no config given\n", __func__);
        result = ESP_FAIL;
        goto err_out;
    }

    /* lock the config mutex */
    status = xSemaphoreTake(cfg->lock, configTICK_RATE_HZ);
    if(status != pdTRUE){
        ESP_LOGE(TAG, "[%s] Timeout waiting for config mutex.\n", __func__);
        result = ESP_FAIL;
        goto err_out;
    }

    (void) xQueueReset(cfg->free_queue);

    if(strip_len <= CONFIG_WS2812_MAX_LEDS){
        cfg->strip_len = strip_len;
        reset_off = ws2812_dmabuf_len(cfg->type, strip_len) - WS2812_RESET_LEN;

        for(i = 0; i < NUM_DMA_BUFFS; ++i){
            tx_buff = &(cfg->tx_buffers[i]);

            if(tx_buff->buff != NULL){
                free(tx_buff->buff);
            }

            tx_buff->buff = malloc(ws2812_dmabuf_len(cfg->type, strip_len));
            if(tx_buff->buff == NULL){
                result = ESP_ERR_NO_MEM;
                goto err_out;
            }

            /* initialise LEDs to off and add reset pulse at end of strip */
            memset(tx_buff->buff, WS_BITS_00, reset_off);
            memset(&(tx_buff->buff[reset_off]), WS_BITS_RESET, WS2812_RESET_LEN);

            status = xQueueSend(cfg->free_queue, &(tx_buff), portMAX_DELAY);
            if(status != pdTRUE){
                result = ESP_FAIL;
                goto err_out;
            }
        }
    } else {
        ESP_LOGE(TAG, "[%s] Strip too long for DMA buffer\n", __func__);
        result = ESP_FAIL;
    }

    xSemaphoreGive(cfg->lock);

err_out:
    if(result != ESP_OK){
        (void) xQueueReset(cfg->free_queue);

        for(i = 0; i < NUM_DMA_BUFFS; i++){
            if(cfg->tx_buffers[i].buff != NULL){
                free(cfg->tx_buffers[i].buff);
                cfg->tx_buffers[i].buff = NULL;
            }
        }
    }
    return result;
}

size_t ws2812_data_len(enum pixel_type type, uint16_t len)
{
    unsigned int colours;

    switch(type){
    case pixel_rgbw:
        colours = 4;
        break;
    case pixel_rgb:
    default:
        colours = 3;
        break;
    }

    /*
     * We have three or four RGB(W) bytes per LED and need to send out four
     * bits on SPI to transmit one bit to the WS2812.
     */
    return len * colours * 4;
}

size_t ws2812_dmabuf_len(enum pixel_type type, uint16_t len)
{
     /* Add length of reset signal to data length. */
    return ws2812_data_len(type, len) + WS2812_RESET_LEN;
}

