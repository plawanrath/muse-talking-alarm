/*
 * Copyright 2026 Plawan Kumar Rath
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microphone capture test for the Waveshare ESP32-S3-AUDIO-Board.
 *
 * Bring-up step: verify the microphone path end to end.
 *   1. Initialize the ES7210 4-channel mic ADC (16-bit, 16 kHz, I2S slave,
 *      MCLK = 256 * 16 kHz = 4.096 MHz from the ESP32).
 *   2. Capture stereo I2S RX with the ESP32 as I2S master
 *      (left = MIC1/ADC1, right = MIC2/ADC2).
 *   3. Print an ASCII VU meter every ~200 ms so a clap or speech near the
 *      board shows up as moving bars.
 *
 * ES7210 register sequence mirrors es7210_config_codec() in Espressif's
 * official `espressif/es7210` component driver (Apache-2.0, from esp-bsp),
 * with:
 *   - es7210_set_i2s_sample_rate(16000, mclk_ratio=256), which selects the
 *     {mclk=4096000, lrck=16000} coefficient row
 *     (adc_div=1, dll=1, doubler=1, osr=0x20, lrck_h=0x01, lrck_l=0x00)
 *   - es7210_set_i2s_format(I2S, 16-bit, TDM off) -> REG11=0x60, REG12=0x00
 *   - 30 dB mic gain, 2.87 V mic bias
 * REG08 (master/slave select) is left at its reset default (I2S slave),
 * exactly as the driver does.
 *
 * Pin map (Waveshare ESP32-S3-AUDIO-Board):
 *   I2C:  SDA = GPIO11, SCL = GPIO10, 100 kHz
 *   I2S:  MCLK = GPIO12, BCLK = GPIO13, LRCK = GPIO14,
 *         DIN (mic data into ESP32) = GPIO15
 *   ES7210 @ 0x40
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mic_test";

/* --------------------------------- I2C ---------------------------------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    11
#define I2C_SCL_GPIO    10
#define I2C_FREQ_HZ     100000

#define ES7210_ADDR     0x40

/* --------------------------------- I2S ---------------------------------- */
#define I2S_MCLK_GPIO   12
#define I2S_BCLK_GPIO   13
#define I2S_LRCK_GPIO   14
#define I2S_DIN_GPIO    15
#define SAMPLE_RATE_HZ  16000
#define MCLK_MULT       256   /* MCLK = 16 kHz * 256 = 4.096 MHz */

/* ------------------------------ VU meter -------------------------------- */
#define CHUNK_FRAMES    512   /* frames per i2s_channel_read */
#define WINDOW_MS       200   /* one meter line per window */
#define LOUD_THRESHOLD  20000 /* peak above this prints LOUD! (clap) */
#define BAR_WIDTH       40

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t es7210;

static esp_err_t i2c_reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 1000);
}

/*
 * ES7210 init: 16 kHz / 16-bit / I2S slave, MCLK = 4.096 MHz on the MCLK pin,
 * MIC1..MIC4 powered with 30 dB gain and 2.87 V bias.
 * Values copied exactly from the official driver sequence described above.
 */
static const uint8_t es7210_init_seq[][2] = {
    {0x00, 0xFF}, /* software reset */
    {0x00, 0x32},
    {0x09, 0x30}, /* chip initial-state period */
    {0x0A, 0x30}, /* power-up state period */
    {0x23, 0x2A}, /* ADC12 HPF1 */
    {0x22, 0x0A}, /* ADC12 HPF2 */
    {0x21, 0x2A}, /* ADC34 HPF1 */
    {0x20, 0x0A}, /* ADC34 HPF2 */
    {0x11, 0x60}, /* 16-bit, standard I2S format */
    {0x12, 0x00}, /* TDM off: ADC1 -> left, ADC2 -> right */
    {0x40, 0xC3}, /* analog power + VMID */
    {0x41, 0x70}, /* MIC12 bias 2.87 V */
    {0x42, 0x70}, /* MIC34 bias 2.87 V */
    {0x43, 0x1A}, /* MIC1 gain 30 dB */
    {0x44, 0x1A}, /* MIC2 gain 30 dB */
    {0x45, 0x1A}, /* MIC3 gain 30 dB */
    {0x46, 0x1A}, /* MIC4 gain 30 dB */
    {0x47, 0x08}, /* MIC1 power on */
    {0x48, 0x08}, /* MIC2 power on */
    {0x49, 0x08}, /* MIC3 power on */
    {0x4A, 0x08}, /* MIC4 power on */
    {0x07, 0x20}, /* OSR for 4.096 MHz / 16 kHz */
    {0x02, 0xC1}, /* adc_div=1, doubler=1, dll=1 */
    {0x04, 0x01}, /* LRCK divider high */
    {0x05, 0x00}, /* LRCK divider low */
    {0x06, 0x04}, /* power down DLL */
    {0x4B, 0x0F}, /* MIC12 bias + ADC12 + PGA12 power */
    {0x4C, 0x0F}, /* MIC34 bias + ADC34 + PGA34 power */
    {0x00, 0x71}, /* enable device */
    {0x00, 0x41},
};

static esp_err_t es7210_init(void)
{
    for (unsigned i = 0; i < sizeof(es7210_init_seq) / sizeof(es7210_init_seq[0]); i++) {
        esp_err_t r = i2c_reg_write(es7210, es7210_init_seq[i][0], es7210_init_seq[i][1]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "ES7210 reg 0x%02x write failed", es7210_init_seq[i][0]);
            return r;
        }
    }
    ESP_LOGI(TAG, "ES7210 initialized: 16-bit, 16 kHz, slave, MCLK 4.096 MHz, mic gain 30 dB");
    return ESP_OK;
}

static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &i2c_bus), TAG, "i2c bus create failed");

    i2c_device_config_t es_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES7210_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &es_cfg, &es7210), TAG, "es7210 add failed");

    return ESP_OK;
}

static esp_err_t i2s_rx_init(i2s_chan_handle_t *rx_handle)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, rx_handle), TAG, "i2s channel create failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_LRCK_GPIO,
            .dout = GPIO_NUM_NC,
            .din = I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256; /* 16 kHz * 256 = 4.096 MHz */

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(*rx_handle, &std_cfg), TAG, "i2s std mode init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(*rx_handle), TAG, "i2s channel enable failed");

    ESP_LOGI(TAG, "I2S RX ready: 16 kHz 16-bit stereo, MCLK=GPIO%d BCLK=GPIO%d LRCK=GPIO%d DIN=GPIO%d",
             I2S_MCLK_GPIO, I2S_BCLK_GPIO, I2S_LRCK_GPIO, I2S_DIN_GPIO);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "mic_test starting");

    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "I2C ready: SDA=GPIO%d SCL=GPIO%d @ %d Hz", I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);

    ESP_ERROR_CHECK(es7210_init());

    i2s_chan_handle_t rx_handle = NULL;
    ESP_ERROR_CHECK(i2s_rx_init(&rx_handle));

    /* Let clocks and the ADC settle before measuring. */
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "MIC TEST RUNNING - clap or talk near the board");

    static int16_t buf[CHUNK_FRAMES * 2];
    int peak_l = 0, peak_r = 0;
    int64_t window_end = esp_timer_get_time() + (int64_t)WINDOW_MS * 1000;

    while (1) {
        size_t bytes_read = 0;
        ESP_ERROR_CHECK(i2s_channel_read(rx_handle, buf, sizeof(buf), &bytes_read, portMAX_DELAY));

        int frames = (int)(bytes_read / 4); /* stereo 16-bit: 4 bytes per frame */
        for (int i = 0; i < frames; i++) {
            int s0 = abs((int)buf[2 * i]);
            int s1 = abs((int)buf[2 * i + 1]);
            if (s0 > peak_l) {
                peak_l = s0;
            }
            if (s1 > peak_r) {
                peak_r = s1;
            }
        }

        if (esp_timer_get_time() >= window_end) {
            int peak = peak_l > peak_r ? peak_l : peak_r;

            char bar[BAR_WIDTH + 1];
            int n = (int)((int64_t)peak * BAR_WIDTH / 32768);
            if (n > BAR_WIDTH) {
                n = BAR_WIDTH;
            }
            for (int i = 0; i < BAR_WIDTH; i++) {
                bar[i] = i < n ? '#' : '-';
            }
            bar[BAR_WIDTH] = '\0';

            double db = peak > 0 ? 20.0 * log10((double)peak / 32768.0) : -99.0;
            if (peak >= LOUD_THRESHOLD) {
                ESP_LOGI(TAG, "[%s] %6.1f dB (L=%5d R=%5d)  <-- LOUD!", bar, db, peak_l, peak_r);
            } else {
                ESP_LOGI(TAG, "[%s] %6.1f dB (L=%5d R=%5d)", bar, db, peak_l, peak_r);
            }

            peak_l = 0;
            peak_r = 0;
            window_end = esp_timer_get_time() + (int64_t)WINDOW_MS * 1000;
        }
    }
}
