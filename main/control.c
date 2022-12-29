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
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "kutils.h"

#include "blinken.h"
#include "control.h"
#if defined(CONFIG_BLINKEN_RMT)
#include "driver/rmt.h"
#include "ir_tools.h"
#endif

#if defined(CONFIG_BLINKEN_BUTTONS)
#include "freertos/timers.h"
#endif
#if defined(CONFIG_BLINKEN_ROTENC)
#include <rotary_encoder.h>
#endif

static const char *TAG = "RMT";

#define EVNT_QUEUE_LEN   10

static QueueHandle_t ctrl_queue;
static QueueSetHandle_t ctrl_queue_set;
static SemaphoreHandle_t tx_sema;

#if defined(CONFIG_BLINKEN_RMT)
static rmt_channel_t rx_channel = RMT_CHANNEL_2;
static rmt_channel_t tx_channel = RMT_CHANNEL_0;
static RingbufHandle_t rxrb_handle;
static ir_parser_t *ir_parser;

struct rmt_code {
    enum ctrl_event_type event;
    uint32_t addr;
    uint32_t code;
};

#define RMT_ADDR    0xd880
struct rmt_code rmt_table[] = {
    {EVNT_0,        RMT_ADDR, 0xff00},
    {EVNT_1,        RMT_ADDR, 0xfe01},
    {EVNT_2,        RMT_ADDR, 0xfd02},
    {EVNT_3,        RMT_ADDR, 0xfc03},
    {EVNT_4,        RMT_ADDR, 0xfb04},
    {EVNT_5,        RMT_ADDR, 0xfa05},
    {EVNT_6,        RMT_ADDR, 0xf906},
    {EVNT_7,        RMT_ADDR, 0xf807},
    {EVNT_8,        RMT_ADDR, 0xf708},
    {EVNT_9,        RMT_ADDR, 0xf609},
    {EVNT_PWR,      RMT_ADDR, 0xd02f},
    {EVNT_INFO,     RMT_ADDR, 0x916e},
    {EVNT_BACK,     RMT_ADDR, 0xdc23},
    {EVNT_UP,       RMT_ADDR, 0xe11e},
    {EVNT_DOWN,     RMT_ADDR, 0xe01f},
    {EVNT_LEFT,     RMT_ADDR, 0xdf20},
    {EVNT_RIGHT,    RMT_ADDR, 0xde21},
    {EVNT_OK,       RMT_ADDR, 0xdd22},
    {EVNT_VOLUP,    RMT_ADDR, 0xef10},
    {EVNT_VOLDOWN,  RMT_ADDR, 0xee11},
    {EVNT_MUTE,     RMT_ADDR, 0xf10e},
    {EVNT_PRGUP,    RMT_ADDR, 0xed12},
    {EVNT_PRGDOWN,  RMT_ADDR, 0xec13},
    {EVNT_MENU,     RMT_ADDR, 0x906f},
    {EVNT_GUIDE,    RMT_ADDR, 0xd926},
    {EVNT_RED,      RMT_ADDR, 0x9a65},
    {EVNT_GREEN,    RMT_ADDR, 0x9966},
    {EVNT_YELLOW,   RMT_ADDR, 0x9867},
    {EVNT_BLUE,     RMT_ADDR, 0x9768},
    {EVNT_REWIND,   RMT_ADDR, 0xea15},
    {EVNT_FASTFWD,  RMT_ADDR, 0xeb14},
    {EVNT_SKIPBCK,  RMT_ADDR, 0xe41b},
    {EVNT_SKIPFWD,  RMT_ADDR, 0xe51a},
    {EVNT_PLAY,     RMT_ADDR, 0xe916},
    {EVNT_PAUSE,    RMT_ADDR, 0xe718},
    {EVNT_STOP,     RMT_ADDR, 0xe619},
};

/* Call-back function for the TX button event. */
static int rmt_tx_event_cb(struct ctrl_event *event, void *priv)
{
    int result;

    ESP_LOGD(TAG, "[%s] Called for event %d", __func__, event->event);

    result = 0;

    /* wake up the TX function if this is the right event. */
    if(event->event == EVNT_BTN1_S){
        (void) xSemaphoreGive(tx_sema);
        /* signal that event was handled */
        result = 1;
    }

    return result;
}


static esp_err_t setup_rmt(void)
{
    BaseType_t result;
    esp_err_t ret;
    rmt_config_t rmt_rx_config =
        RMT_DEFAULT_CONFIG_RX(CONFIG_BLINKEN_RMT_RX_GPIO, rx_channel);
    ir_parser_config_t ir_parser_config =
        IR_PARSER_DEFAULT_CONFIG((ir_dev_t)rx_channel);

    ret = rmt_config(&rmt_rx_config);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error setting RMT config");
        goto err_out;
    }

    ret = rmt_driver_install(rx_channel, 1000, 0);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error installing RMT driver");
        goto err_out;
    }

    // Using extended IR protocols (both NEC and RC5 have extended version)
    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT;
#if CONFIG_BLINKEN_RMT_PROTO_NEC
    ir_parser = ir_parser_rmt_new_nec(&ir_parser_config);
#elif CONFIG_BLINKEN_RMT_PROTO_RC5
    ir_parser = ir_parser_rmt_new_rc5(&ir_parser_config);
#else
#error No IR protocol specified
#endif

    ret = rmt_get_ringbuf_handle(rx_channel, &rxrb_handle);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error getting RMT ringbuffer");
        goto err_out;
    }

    result = xRingbufferAddToQueueSetRead(rxrb_handle, ctrl_queue_set);
    if (result != pdPASS){
        ESP_LOGE(TAG, "Error adding RMT ringbuffer to CMD queue set");
        ret = ESP_FAIL;
        goto err_out;
    }

    ret = rmt_rx_start(rx_channel, true);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error starting RMT RX");
        goto err_out;
    }

    /* create a semaphore for the TX task to sleep on until a button
     * event arrives. */
    tx_sema = xSemaphoreCreateBinary();
    if(tx_sema == NULL){
        ESP_LOGE(TAG, "[%s] TX mutex creation failed\n", __func__);
        ret = ESP_ERR_NO_MEM;
        goto err_out;
    }

    /* register the call-back function for the button event. */
    ret = register_event_cb(rmt_tx_event_cb, NULL);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "[%s] Registering ir_tx_event_cb failed.", __func__);
        goto err_out;
    }

err_out:
    return ret;
}

static void handle_rmt_rx(void)
{
    rmt_item32_t *items;
    struct ctrl_event event;
    uint32_t addr, cmd;
    size_t len;
    unsigned int idx;
    bool rep;

    BaseType_t result;

    items = (rmt_item32_t *) xRingbufferReceive(rxrb_handle, &len, 0);
    if (items) {
        len /= 4; // one RMT = 4 Bytes
        if (ir_parser->input(ir_parser, items, len) == ESP_OK) {
            if (ir_parser->get_scan_code(ir_parser, &addr, &cmd, &rep) == ESP_OK) {
                ESP_LOGI(TAG, "Scan Code %s --- addr: 0x%04x cmd: 0x%04x",
                            rep ? "(repeat)" : "", addr, cmd);

                for (idx = 0; idx < ARRAY_SIZE(rmt_table); ++idx){
                    if (rmt_table[idx].addr == addr && rmt_table[idx].code == cmd){
                        event.event = rmt_table[idx].event;
                        event.repeat = rep;
                        result = xQueueSendToBack(ctrl_queue,
                                                  (void *) &event,
                                                  (TickType_t) 0);
                        if (result != pdPASS){
                            ESP_LOGW(TAG, "IR RMT command dropped");
                        }
                        break;
                    }
                }
            }
        }
        //after parsing the data, return spaces to ringbuffer.
        vRingbufferReturnItem(rxrb_handle, (void *) items);
    }
}

/* Send out an IR signal if triggered by a button event */
static void rmt_tx_task(void *arg)
{
    uint32_t addr, cmd, length;
    esp_err_t result;
    BaseType_t status;
    rmt_item32_t *items = NULL;
    ir_builder_t *ir_builder = NULL;
    rmt_config_t rmt_tx_config =
        RMT_DEFAULT_CONFIG_TX(CONFIG_BLINKEN_RMT_TX_GPIO, tx_channel);
    ir_builder_config_t ir_builder_config =
        IR_BUILDER_DEFAULT_CONFIG((ir_dev_t)tx_channel);

    /* send the code for EVNT_OK */
    addr = RMT_ADDR;
    cmd = 0xdd22;
    length = 0;

    /* configure and install the IR driver */
    rmt_tx_config.tx_config.carrier_en = true;
    result = rmt_config(&rmt_tx_config);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] rmt_config() failed.", __func__);
        goto err_out;
    }

    result = rmt_driver_install(tx_channel, 0, 0);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] rmt_driver_install() failed.", __func__);
        goto err_out;
    }

    /* set up the IR code builder */
    ir_builder_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT;
    ir_builder = ir_builder_rmt_new_nec(&ir_builder_config);
    if(ir_builder == NULL){
        ESP_LOGE(TAG, "[%s] ir_builder_rmt_new_nec() failed.", __func__);
        goto err_out;
    }

    while (1) {
        /* sleep until semaphore is released by the event call-back function */
        status = xSemaphoreTake(tx_sema, portMAX_DELAY);
        if(status != pdTRUE){
            printf("[%s] timeout waiting for TX sema\n", __func__);
            continue;
        }

        ESP_LOGD(TAG, "Send command 0x%x to address 0x%x", cmd, addr);

        /* build IR frame */
        result = ir_builder->build_frame(ir_builder, addr, cmd);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] build_frame() failed.", __func__);
            continue;
        }

        /* get compiled IR frame data */
        result = ir_builder->get_result(ir_builder, &items, &length);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] get_result() failed.", __func__);
            continue;
        }

        /* stop RX channel to prevent self-triggering */
        rmt_rx_stop(rx_channel);

        /* send IR frame and wait until completion */
        rmt_write_items(tx_channel, items, length, true);

        /* re-enable RX channel */
        rmt_rx_start(rx_channel, true);
    }

err_out:
    ir_builder->del(ir_builder);
    rmt_driver_uninstall(tx_channel);
    vTaskDelete(NULL);
}

#endif //defined(CONFIG_BLINKEN_RMT)

#if defined(CONFIG_BLINKEN_BUTTONS)

struct debounce {
    TimerHandle_t timer;
    enum ctrl_event_type event_short;
    enum ctrl_event_type event_long;
    unsigned int gpio;
    unsigned int count;
};

#if CONFIG_BLINKEN_BUTTON_0 == -1 && CONFIG_BLINKEN_BUTTON_1 == -1 && CONFIG_BLINKEN_BUTTON_2 == -1
#error No valid button GPIO set
#endif

#if defined(CONFIG_BLINKEN_BUTTON_0) && CONFIG_BLINKEN_BUTTON_0 != -1
static struct debounce debounce0 = {NULL, EVNT_NONE, EVNT_NONE, CONFIG_BLINKEN_BUTTON_0, 0};
#endif

#if defined(CONFIG_BLINKEN_BUTTON_1) && CONFIG_BLINKEN_BUTTON_1 != -1
static struct debounce debounce1 = {NULL, EVNT_NONE, EVNT_NONE, CONFIG_BLINKEN_BUTTON_1, 0};
#endif

#if defined(CONFIG_BLINKEN_BUTTON_2) && CONFIG_BLINKEN_BUTTON_2 != -1
static struct debounce debounce2 = {NULL, EVNT_NONE, EVNT_NONE, CONFIG_BLINKEN_BUTTON_2, 0};
#endif

static void debounce_cb(TimerHandle_t timer)
{
    struct debounce *deb;
    struct ctrl_event event;
    BaseType_t result;

    deb = (struct debounce *) pvTimerGetTimerID(timer);

    ++deb->count;
    event.event = EVNT_NONE;
    if(gpio_get_level(deb->gpio) == 1){
        /* Button was released. If count is below long press threshold,
         * send short press event. */
        if(deb->count < 50){
            event.event = deb->event_short;
        }

        /* reset long press counter */
        deb->count = 0;
    } else {
        /* keep retarting timer until long press threshold is reached,
         * then send long press event. */
        if(deb->count < 50){
            xTimerReset(timer, 10);
        } else {
            event.event = deb->event_long;
        }
    }

    if(event.event != EVNT_NONE){
        event.repeat = false;
        result = xQueueSendToBack(ctrl_queue, (void *) &event, 0);
        if(result != pdPASS){
            ESP_LOGW(TAG, "Button command dropped");
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t result, task_woken;
    struct debounce *deb;

    result = pdFAIL;
    task_woken = pdFALSE;
    deb = (struct debounce *) arg;

    /* Reset debounce timer. If the timer is already running, it will be
     * re-scheduled, so the call-back function will not be called until the
     * line has been stable for the debounce time. */
    result = xTimerResetFromISR(deb->timer, &task_woken);

    if(result != pdFAIL && task_woken != pdFALSE){
        portYIELD_FROM_ISR();
    }
}

static esp_err_t setup_gpio(struct debounce *deb, unsigned int gpio,
                            enum ctrl_event_type short_press,
                            enum ctrl_event_type long_press)
{
    gpio_config_t cfg;
    esp_err_t result;

    deb->timer = xTimerCreate("Debounce_Timer",
                              pdMS_TO_TICKS(10),
                              pdFALSE, deb, debounce_cb);
    if(deb->timer == NULL){
        ESP_LOGE(TAG, "[%s] Debounce timer creation failed.", __func__);
        result = ESP_ERR_NO_MEM;
        goto on_exit;
    }

    result = gpio_isr_handler_add(gpio, gpio_isr_handler, deb);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_isr_handler_add() failed", __func__);
        goto on_exit;
    }

    memset(&cfg, 0x0, sizeof(cfg));
    cfg.pin_bit_mask |= (1LL << gpio);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_ANYEDGE;

    result = gpio_config(&cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_config() failed for reset button", __func__);
        goto on_exit;
    }

    deb->event_short = short_press;
    deb->event_long = long_press;

on_exit:
    // FIXME: remove ISRs on failure
    return result;
}

static esp_err_t setup_buttons(void)
{
    esp_err_t result;

    result = gpio_install_isr_service(0);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Installing ISR service failed.", __func__);
        goto on_exit;
    }

#if defined(CONFIG_BLINKEN_BUTTON_0) && CONFIG_BLINKEN_BUTTON_0 != -1
    result = setup_gpio(&debounce0, CONFIG_BLINKEN_BUTTON_0, EVNT_BTN0_S, EVNT_BTN0_L);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_isr_handler_add() failed", __func__);
        goto on_exit;
    }
#endif // BLINKEN_BUTTON_0

#if defined(CONFIG_BLINKEN_BUTTON_1) && CONFIG_BLINKEN_BUTTON_1 != -1
    result = setup_gpio(&debounce1, CONFIG_BLINKEN_BUTTON_1, EVNT_BTN1_S, EVNT_BTN1_L);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_isr_handler_add() failed", __func__);
        goto on_exit;
    }
#endif // BLINKEN_BUTTON_1

#if defined(CONFIG_BLINKEN_BUTTON_2) && CONFIG_BLINKEN_BUTTON_2 != -1
    result = setup_gpio(&debounce2, CONFIG_BLINKEN_BUTTON_2, EVNT_BTN2_S, EVNT_BTN2_L);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_isr_handler_add() failed", __func__);
        goto on_exit;
    }
#endif // BLINKEN_BUTTON_2

on_exit:
    // FIXME: remove ISRs on failure
    return result;
}
#endif // defined(CONFIG_BLINKEN_BUTTONS)

#if defined(CONFIG_BLINKEN_ROTENC)

static rotary_encoder_info_t rot_info = { 0 };
static QueueHandle_t rot_queue = NULL;
static esp_err_t setup_rotenc(void)
{
    gpio_config_t cfg;
    BaseType_t ret;
    esp_err_t result;

    result = gpio_install_isr_service(0);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Installing ISR service failed.", __func__);
        goto on_exit;
    }

    result = rotary_encoder_init(&rot_info,
                                    CONFIG_BLINKEN_ROTENC_A,
                                    CONFIG_BLINKEN_ROTENC_B);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] rotary_encoder_init()  failed.", __func__);
        goto on_exit;
    }

    result = rotary_encoder_enable_half_steps(&rot_info, false);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] ..._enable_half_steps() failed.", __func__);
        goto on_exit;
    }

    rot_queue = rotary_encoder_create_queue();
    if(rot_queue == NULL){
        ESP_LOGE(TAG, "[%s] rotary_encoder_create_queue() failed.", __func__);
        result = ESP_ERR_NO_MEM;
        goto on_exit;
    }

    ret = xQueueAddToSet((QueueSetMemberHandle_t) rot_queue, ctrl_queue_set);
    if(ret != pdPASS){
        ESP_LOGE(TAG, "[%s] xQueueAddToSet() failed.", __func__);
        result = ESP_FAIL;
        goto on_exit;
    }

    result = rotary_encoder_set_queue(&rot_info, rot_queue);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] rotary_encoder_set_queue() failed.", __func__);
        goto on_exit;
    }

on_exit:
    return result;
}

static void handle_rotenc_rx(void)
{
    rotary_encoder_event_t rot_evt;
    struct ctrl_event event;
    BaseType_t result;

    if (xQueueReceive(rot_queue, &rot_evt, 0) == pdTRUE){
        event.event = EVNT_NONE;
        event.repeat = false;

        switch(rot_evt.state.direction){
        case ROTARY_ENCODER_DIRECTION_CLOCKWISE:
            event.event = EVNT_VOLUP;
            break;
        case ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE:
            event.event = EVNT_VOLDOWN;
            break;
        default:
            ESP_LOGD(TAG, "[%s] Received unknown rotary encoder event.",
                        __func__);
            break;
        }

        if (event.event != EVNT_NONE){
            result = xQueueSendToBack(ctrl_queue, (void *) &event,
                                        (TickType_t) 0);

            if (result != pdPASS){
                ESP_LOGW(TAG, "Knob RMT command dropped");
            }
        }
    }
}
#endif // defined(CONFIG_BLINKEN_ROTENC)

static void rmt_event_task(void *arg)
{
    xQueueSetMemberHandle member;

    while (1) {
        ESP_LOGD(TAG, "Remote Control Thread running");
        member = xQueueSelectFromSet(ctrl_queue_set, pdMS_TO_TICKS(1000));
        if (member != NULL){
#if defined(CONFIG_BLINKEN_RMT)
            if(xRingbufferCanRead(rxrb_handle, member) == pdTRUE) {
                handle_rmt_rx();
            } else
#endif
#if defined(CONFIG_BLINKEN_ROTENC)
            if(member == rot_queue) {
                handle_rotenc_rx();
            } else
#endif
            {
                ESP_LOGW(TAG, "Unknown queue set member.");
            }
        }
    }
    ESP_LOGE(TAG, "Stopping Remote Control Thread");
    vTaskDelete(NULL);
}

QueueHandle_t blinken_ctrl_get_queue(void)
{
    return ctrl_queue;
}

esp_err_t blinken_ctrl_start(void)
{
    esp_err_t result;
    ESP_LOGI(TAG, "Starting Remote Control Thread");

    result = ESP_OK;
    ctrl_queue = xQueueCreate(EVNT_QUEUE_LEN, sizeof(struct ctrl_event));
    if(ctrl_queue == NULL){
        ESP_LOGE(TAG, "xQueueCreate() faied for ctrl_queue");
        goto err_out;
    }
    ctrl_queue_set = xQueueCreateSet(10);
    if(ctrl_queue_set == NULL){
        ESP_LOGE(TAG, "xQueueCreateSet() faied for ctrl_queue_set");
        goto err_out;
    }

#if defined(CONFIG_BLINKEN_BUTTONS)
    setup_buttons();
#endif // defined(CONFIG_BLINKEN_ROTENC)

#if defined(CONFIG_BLINKEN_ROTENC)
    setup_rotenc();
#endif // defined(CONFIG_BLINKEN_ROTENC)

#if defined(CONFIG_BLINKEN_RMT)
    setup_rmt();
    xTaskCreate(rmt_tx_task, "rmt_tx_task", 2048, NULL, 10, NULL);
#endif

#if defined(CONFIG_BLINKEN_RMT) || defined(CONFIG_BLINKEN_ROTENC) || defined(CONFIG_BLINKEN_BUTTONS)
    xTaskCreate(rmt_event_task, "rmt_event_task", 2048, NULL, 10, NULL);
#endif

err_out:
    return result;
}
