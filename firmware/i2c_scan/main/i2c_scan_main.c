/*
 * Copyright 2026 Plawan Kumar Rath
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C bus scan for the Waveshare ESP32-S3 AI Smart Speaker board.
 * Expected: 0x18 (ES8311), 0x40 (ES7210), 0x51 (PCF85063), 0x20 (TCA9555).
 */

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 10
#define I2C_MASTER_SDA_IO 11
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

static const char *TAG = "i2c_scan";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d @ %d Hz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    int found = 0;
    for (int addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                             pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            const char *name = "";
            switch (addr) {
                case 0x18: name = "ES8311 speaker codec"; break;
                case 0x20: name = "TCA9555 IO expander (amp enable EXIO8)"; break;
                case 0x40: name = "ES7210 mic ADC"; break;
                case 0x51: name = "PCF85063 RTC"; break;
            }
            ESP_LOGI(TAG, "found device at 0x%02X %s", addr, name);
            found++;
        }
    }
    ESP_LOGI(TAG, "scan complete: %d device(s) found", found);
}
