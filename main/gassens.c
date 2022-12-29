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

/**
 * Gas sensor driver.
 *
 * Adapted from Renato Freitas SGP30 Air Quality Sensor Library Example
 *
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "SGP30.h"
#include "gassens.h"
#include "control.h"
#include "klist.h"

static const char *TAG = "GAS";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_SCL_IO CONFIG_BLINKEN_GAS_I2C_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_BLINKEN_GAS_I2C_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_BLINKEN_GAS_I2C_PORT) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_BLINKEN_GAS_I2C_FREQ        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

i2c_port_t i2c_num = I2C_MASTER_NUM;

#define AVG_LENGTH          10      /* length of the moving average queue */
#define AVG_PRIME_VAL       400     /* default value to prime the moving average with */
#define MEAS_INTERVAL       5000    /* measurement interval in milliseconds*/
#define AIR_GOOD_THRESH     500     /* good air quality threshold */
#define AIR_NORMAL_THRESH   1500    /* normal air quality threshold */

esp_err_t i2c_master_driver_initialize(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


/**
 * @brief generic function for reading I2C data
 *
 * @param reg_addr register adress to read from
 * @param reg_data pointer to save the data read
 * @param len length of data to be read
 * @param intf_ptr
 *
 * >init: dev->intf_ptr = &dev_addr;
 *
 * @return ESP_OK if reading was successful
 */
int8_t main_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    if (reg_addr != 0xff) {
        i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
    }

    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    return ret;
}


/**
 * @brief generic function for writing data via I2C
 *
 * @param reg_addr register adress to write to
 * @param reg_data register data to be written
 * @param len length of data to be written
 * @param intf_ptr
 *
 * @return ESP_OK if writing was successful
 */
int8_t main_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t chip_addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);

    if (reg_addr != 0xFF) {
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    return ret;
}

static void gas_task(void *arg)
{
    sgp30_dev_t main_sgp30_sensor;
    uint16_t eco2_baseline, tvoc_baseline;
    unsigned int mov_avg;
    QueueHandle_t ctrl_queue;
    struct ctrl_event event;
    BaseType_t result;

    /* fetch pointer to blinkenlight's control event queue */
    ctrl_queue = blinken_ctrl_get_queue();
    if(ctrl_queue == NULL){
        ESP_LOGE(TAG, "Getting control queue failed");
        goto err_out;
    }

    i2c_master_driver_initialize();
    sgp30_init(&main_sgp30_sensor, (sgp30_read_fptr_t)main_i2c_read, (sgp30_write_fptr_t)main_i2c_write);

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 14; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        sgp30_IAQ_measure(&main_sgp30_sensor);
        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);

        /* During calibration TVOC is always 0 and eCO2 is 400. If we see
         * anything else, then something is wrong. */
        if(main_sgp30_sensor.TVOC != 0 && main_sgp30_sensor.eCO2 != 400){
            ESP_LOGE(TAG, "SGP30 callibration failed.");
            goto err_out;
        }
    }

    // Read initial baselines
    sgp30_get_IAQ_baseline(&main_sgp30_sensor, &eco2_baseline, &tvoc_baseline);
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);

    ESP_LOGI(TAG, "SGP30 main task is running...");

    /* prime the moving average */
    mov_avg = AVG_LENGTH * AVG_PRIME_VAL;

    /* initialise the the event struct with the first event to send */
    event.event = EVNT_AIR_INIT;
    event.repeat = false;

    while(1) {
        /* if there is valid event data, send it to the control queue */
        if (event.event != EVNT_NONE){
            result = xQueueSendToBack(ctrl_queue, (void *) &event,
                                        (TickType_t) 0);

            if (result != pdPASS){
                ESP_LOGW(TAG, "Gas sensor event was dropped");
            }
        }

        /* sleep until next measurement is due */
        vTaskDelay(MEAS_INTERVAL / portTICK_RATE_MS);

        /* fetch current reading and update the moving average */
        sgp30_IAQ_measure(&main_sgp30_sensor);
        mov_avg -= mov_avg / AVG_LENGTH;
        mov_avg += main_sgp30_sensor.eCO2;

        ESP_LOGD(TAG, "TVOC: %d,  eCO2: %d, mov_avg: %d",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2, mov_avg);

        /* update the event data with according to the current moving average */
        if(mov_avg <= AIR_GOOD_THRESH * AVG_LENGTH){
            event.event = EVNT_AIR_GOOD;
        } else if(mov_avg <= AIR_NORMAL_THRESH * AVG_LENGTH){
            event.event = EVNT_AIR_NORMAL;
        } else {
            event.event = EVNT_AIR_BAD;
        }
    }

err_out:
    ESP_LOGE(TAG, "Stopping Gas Sensor Thread");
    vTaskDelete(NULL);
}

void start_gas_sensor(void)
{
    ESP_LOGI(TAG, "Starting Gas Sensor Thread");

    xTaskCreate(gas_task, "gas_task", 2048, NULL, 10, NULL);
}
