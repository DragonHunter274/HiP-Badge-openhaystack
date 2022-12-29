/**
 * ESP32 Blinkenlights.
 * Copyright (C) 2019-2022 Tido Klaassen <tido_blinken@4gh.eu>
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

#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <driver/spi_master.h>

/*
 * WS2812 bit period is 1.25us. We need to send out four bits to form the
 * high-low signal, so one SPI bit takes 0.25us. According to the data sheets,
 * the reset signal is 50us low, but various sources claim that latching
 * happens at around ~10us. To be on the safe side, we will use 20us, so we
 * need to send out 80 bits. 
 */
#define WS2812_RESET_LEN        (160 / 8)

/* We send two WS2812-bits per byte, one bit per nibble. */
#if defined(CONFIG_WS2812_INVERT_SPI)
#define WS_BITS_00              0x77 /* -___-___ */
#define WS_BITS_01              0x71 /* -___---_ */
#define WS_BITS_10              0x17 /* ---_-___ */
#define WS_BITS_11              0x11 /* ---_---_ */
#define WS_BITS_RESET           0xff /* ________ */
#else
#define WS_BITS_00              0x88 /* -___-___ */
#define WS_BITS_01              0x8e /* -___---_ */
#define WS_BITS_10              0xe8 /* ---_-___ */
#define WS_BITS_11              0xee /* ---_---_ */
#define WS_BITS_RESET           0x00 /* ________ */
#endif // defined(CONFIG_WS2812_INVERT_SPI)

typedef struct {
    uint8_t     red;
    uint8_t     green;
    uint8_t     blue;
    uint8_t     white;
} rgb_value_t;

typedef struct {
    uint16_t    hue;
    uint16_t    saturation;
    uint16_t    value;
} hsv_value_t;

typedef struct _ws2812 ws2812_t;

typedef struct {
    spi_transaction_t trans;
    ws2812_t *cfg;
    uint8_t *buff;
} tx_buffer_t;

#define NUM_DMA_BUFFS   3
enum pixel_type {
    pixel_rgb,
    pixel_grb,
    pixel_rgbw
};

typedef struct _ws2812 {
    spi_device_handle_t spi_master;
    EventGroupHandle_t *events;
    SemaphoreHandle_t   lock;
    QueueHandle_t       free_queue;
    tx_buffer_t         tx_buffers[NUM_DMA_BUFFS];
    uint16_t            strip_len;
    enum pixel_type     type;
} ws2812_t;


#define SCALE_SHIFT         8
#define SCALE_UP(x)         ((x) << SCALE_SHIFT)
#define SCALE_DOWN(x)       ((x) >> SCALE_SHIFT)
#define SCALE_ROUND(x)      ((x) + (1 << (SCALE_SHIFT - 1)))
#define SCALE_DOWN_ROUND(x) SCALE_DOWN(SCALE_ROUND(x))

#define HSV_SEXTANT_SHIFT   8
#define HSV_HUE_SEXTANT     (1 << HSV_SEXTANT_SHIFT)
#define HSV_HUE_STEPS       (6 << HSV_SEXTANT_SHIFT)

#define HSV_HUE_MIN         0
#define HSV_HUE_MAX         (HSV_HUE_STEPS - 1)
#define HSV_SAT_MIN         0
#define HSV_SAT_MAX         SCALE_UP(0xFFu)
#define HSV_VAL_MIN         0
#define HSV_VAL_MAX         SCALE_UP(0xFFu)

/* Define hue values for primary and secondary colours. */
#define HSV_RED             (0 << HSV_SEXTANT_SHIFT)
#define HSV_YELLOW          (1 << HSV_SEXTANT_SHIFT)
#define HSV_GREEN           (2 << HSV_SEXTANT_SHIFT)
#define HSV_CYAN            (3 << HSV_SEXTANT_SHIFT)
#define HSV_BLUE            (4 << HSV_SEXTANT_SHIFT)
#define HSV_MAGENTA         (5 << HSV_SEXTANT_SHIFT)

ws2812_t *ws2812_init(uint16_t strip_len, enum pixel_type type);
esp_err_t ws2812_set_len(ws2812_t *cfg, uint16_t strip_len);
esp_err_t ws2812_deinit(ws2812_t *cfg);
esp_err_t ws2812_prepare(ws2812_t *cfg, hsv_value_t hsv_values[],
                         size_t strip_len, tx_buffer_t **buffer);
esp_err_t ws2812_send(tx_buffer_t *buffer);
size_t ws2812_data_len(enum pixel_type type, uint16_t len);
size_t ws2812_dmabuf_len(enum pixel_type type, uint16_t len);

void rgb2hsv(rgb_value_t *rgb, hsv_value_t *hsv);
#endif
