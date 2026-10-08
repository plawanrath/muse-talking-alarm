/*
 * Copyright 2026 Plawan Kumar Rath
 * SPDX-License-Identifier: Apache-2.0
 *
 * Full-duplex audio test for the Waveshare ESP32-S3-AUDIO-Board.
 *
 * Bring-up step: verify the speaker and the microphones can stream
 * simultaneously on the shared I2S bus (the last hardware bring-up step
 * before application logic).
 *   1. Enable the speaker amplifier via the TCA9555 I/O expander
 *      (EXIO8, active high).
 *   2. Initialize the ES8311 DAC (16-bit, 16 kHz, I2S slave,
 *      MCLK = 256 * 16 kHz = 4.096 MHz).
 *   3. Initialize the ES7210 4-channel mic ADC (16-bit, 16 kHz, I2S slave,
 *      same clocks).
 *   4. Create ONE I2S channel pair with the ESP32 as I2S master
 *      (both codecs are slaves): TX -> ES8311 speaker, RX <- ES7210 mics.
 *   5. Play a continuous quiet 660 Hz tone on the speaker while
 *      simultaneously printing the mic VU meter. Plawan claps or talks
 *      over the tone and confirms the mic still hears him (bars jump).
 *
 * The ES8311 sequence, the TCA9555 amp enable, and the ES7210 sequence are
 * copied verbatim from firmware/audio_test and firmware/mic_test, which
 * were verified on the actual board (beeps heard, mic VU responds to
 * yelling). The tx+rx-on-one-channel topology matches the vendor demo
 * (ESP32-S3-AUDIO-Board-Demo, ESP-IDF mp3_play_03, bsp_board.c).
 *
 * Pin map (Waveshare ESP32-S3-AUDIO-Board):
 *   I2C:  SDA = GPIO11, SCL = GPIO10, 100 kHz
 *   I2S:  MCLK = GPIO12, BCLK = GPIO13, LRCK = GPIO14,
 *         DIN (mic data into ESP32) = GPIO15,
 *         DOUT (DAC data out of ESP32) = GPIO16
 *   TCA9555 @ 0x20, ES8311 @ 0x18, ES7210 @ 0x40
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

static const char *TAG = "duplex_test";

/* --------------------------------- I2C ---------------------------------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    11
#define I2C_SCL_GPIO    10
#define I2C_FREQ_HZ     100000

#define TCA9555_ADDR    0x20
#define ES8311_ADDR     0x18
#define ES7210_ADDR     0x40

/* TCA9555 register addresses (16-bit device: port 0 regs, then port 1 regs) */
#define TCA9555_OUT_PORT1   0x03  /* output values, port 1 (EXIO8..EXIO15) */
#define TCA9555_CFG_PORT1   0x07  /* direction, port 1: 1 = input, 0 = output */

/* EXIO8 = bit 0 of port 1. Active HIGH enables the speaker amplifier. */
#define AMP_ENABLE_BIT      0

/* ------------------------------ ES8311 regs ----------------------------- */
#define ES8311_RESET_REG00      0x00
#define ES8311_CLK_MGR_REG01    0x01
#define ES8311_CLK_MGR_REG02    0x02
#define ES8311_CLK_MGR_REG03    0x03
#define ES8311_CLK_MGR_REG04    0x04
#define ES8311_CLK_MGR_REG05    0x05
#define ES8311_CLK_MGR_REG06    0x06
#define ES8311_CLK_MGR_REG07    0x07
#define ES8311_CLK_MGR_REG08    0x08
#define ES8311_SDPIN_REG09      0x09
#define ES8311_SDPOUT_REG0A     0x0A
#define ES8311_SYSTEM_REG0D     0x0D
#define ES8311_SYSTEM_REG0E     0x0E
#define ES8311_SYSTEM_REG12     0x12
#define ES8311_SYSTEM_REG13     0x13
#define ES8311_ADC_REG1C        0x1C
#define ES8311_DAC_REG32        0x32
#define ES8311_DAC_REG37        0x37

/* --------------------------------- I2S ---------------------------------- */
#define I2S_MCLK_GPIO   12
#define I2S_BCLK_GPIO   13
#define I2S_LRCK_GPIO   14
#define I2S_DIN_GPIO    15
#define I2S_DOUT_GPIO   16
#define SAMPLE_RATE_HZ  16000
#define MCLK_MULT       256   /* MCLK = 16 kHz * 256 = 4.096 MHz */

/* ------------------------------ test signal ----------------------------- */
#define TONE_FREQ_HZ        660
#define TONE_AMPLITUDE      2500  /* quiet: ~8% of full scale */
#define TX_CHUNK_FRAMES     512

/* ------------------------------- VU meter ------------------------------- */
#define RX_CHUNK_FRAMES     512   /* frames per i2s_channel_read */
#define WINDOW_MS           200   /* one meter line per window */
#define LOUD_THRESHOLD      20000 /* peak above this prints LOUD! (clap) */
#define BAR_WIDTH           40

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t tca9555;
static i2c_master_dev_handle_t es8311;
static i2c_master_dev_handle_t es7210;

static esp_err_t i2c_reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 1000);
}

static esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 1000);
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

    i2c_device_config_t tca_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9555_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &tca_cfg, &tca9555), TAG, "tca9555 add failed");

    i2c_device_config_t es_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES8311_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &es_cfg, &es8311), TAG, "es8311 add failed");

    i2c_device_config_t es7210_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES7210_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &es7210_cfg, &es7210), TAG, "es7210 add failed");

    return ESP_OK;
}

/* Drive TCA9555 EXIO8 high. Matches the vendor demo (Set_EXIO(EXIO8, true)). */
static esp_err_t amp_enable(void)
{
    uint8_t reg;

    /* EXIO8 as output: clear bit 0 of config port 1 */
    ESP_RETURN_ON_ERROR(i2c_reg_read(tca9555, TCA9555_CFG_PORT1, &reg), TAG, "tca9555 cfg read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(tca9555, TCA9555_CFG_PORT1, reg & ~(1 << AMP_ENABLE_BIT)),
                        TAG, "tca9555 cfg write failed");

    /* EXIO8 high: set bit 0 of output port 1 */
    ESP_RETURN_ON_ERROR(i2c_reg_read(tca9555, TCA9555_OUT_PORT1, &reg), TAG, "tca9555 out read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(tca9555, TCA9555_OUT_PORT1, reg | (1 << AMP_ENABLE_BIT)),
                        TAG, "tca9555 out write failed");

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "amplifier enabled (TCA9555 EXIO8 = high)");
    return ESP_OK;
}

/*
 * ES8311 init for 16 kHz / 16-bit / I2S slave, MCLK = 4.096 MHz on the MCLK pin.
 * Verbatim from firmware/audio_test (verified working on the board).
 */
static esp_err_t es8311_init(void)
{
    uint8_t reg;

    /* Reset to defaults, then power-on command */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, 0x1F), TAG, "reset1 failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, 0x00), TAG, "reset2 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, 0x80), TAG, "power-on failed");

    /* Clock source: 0x3F = enable all clocks, MCLK from MCLK pin, not inverted */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG01, 0x3F), TAG, "reg01 failed");

    /* SCLK not inverted (clear bit 5 of reg06) */
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG06, &reg), TAG, "reg06 read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG06, reg & ~0x20), TAG, "reg06 failed");

    /* Clock dividers for 4.096 MHz MCLK -> 16 kHz */
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG02, &reg), TAG, "reg02 read failed");
    reg &= 0x07;                       /* (pre_div-1)<<5 | pre_multi<<3 = 0 */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG02, reg), TAG, "reg02 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG03, 0x10), TAG, "reg03 failed"); /* fs_mode=0, adc_osr=0x10 */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG04, 0x10), TAG, "reg04 failed"); /* dac_osr=0x10 */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG05, 0x00), TAG, "reg05 failed"); /* adc_div=1, dac_div=1 */
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG06, &reg), TAG, "reg06 read failed");
    reg &= 0xE0;
    reg |= (4 - 1);                    /* bclk_div=4 */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG06, reg), TAG, "reg06 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG07, &reg), TAG, "reg07 read failed");
    reg &= 0xC0;                       /* lrck_h=0 */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG07, reg), TAG, "reg07 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG08, 0xFF), TAG, "reg08 failed"); /* lrck_l */

    /* Slave serial port (clear bit 6 of reg00), 16-bit I2S on SDP in/out */
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_RESET_REG00, &reg), TAG, "reg00 read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, reg & 0xBF), TAG, "reg00 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SDPIN_REG09, 0x0C), TAG, "reg09 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SDPOUT_REG0A, 0x0C), TAG, "reg0a failed");

    /* Power up analog path and DAC output */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG0D, 0x01), TAG, "reg0d failed"); /* power up analog */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG0E, 0x02), TAG, "reg0e failed"); /* PGA + ADC modulator */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG12, 0x00), TAG, "reg12 failed"); /* power-up DAC */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG13, 0x10), TAG, "reg13 failed"); /* enable HP drive out */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_ADC_REG1C, 0x6A), TAG, "reg1c failed");   /* ADC EQ bypass */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_DAC_REG37, 0x08), TAG, "reg37 failed");   /* DAC EQ bypass */

    /* Volume 50/100 -> reg32 = 50*256/100 - 1 = 127 (same mapping as the driver) */
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_DAC_REG32, 0x7F), TAG, "volume failed");

    ESP_LOGI(TAG, "ES8311 initialized: 16-bit, 16 kHz, slave, MCLK 4.096 MHz, volume 50");
    return ESP_OK;
}

/*
 * ES7210 init: 16 kHz / 16-bit / I2S slave, MCLK = 4.096 MHz on the MCLK pin,
 * MIC1..MIC4 powered with 30 dB gain and 2.87 V bias.
 * Verbatim from firmware/mic_test (verified working on the board).
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

/*
 * One I2S channel pair (I2S_NUM_0, ESP32 = master): TX feeds the ES8311
 * (DOUT=GPIO16), RX reads the ES7210 (DIN=GPIO15). Both 16 kHz / 16-bit /
 * stereo, MCLK = 16 kHz * 256 = 4.096 MHz on GPIO12.
 */
static esp_err_t i2s_duplex_init(i2s_chan_handle_t *tx_handle, i2s_chan_handle_t *rx_handle)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, tx_handle, rx_handle), TAG, "i2s channel create failed");

    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_LRCK_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din = GPIO_NUM_NC,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    tx_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    i2s_std_config_t rx_cfg = {
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
    rx_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(*tx_handle, &tx_cfg), TAG, "i2s tx std mode init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(*rx_handle, &rx_cfg), TAG, "i2s rx std mode init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(*tx_handle), TAG, "i2s tx channel enable failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(*rx_handle), TAG, "i2s rx channel enable failed");

    ESP_LOGI(TAG, "I2S duplex ready: 16 kHz 16-bit stereo, MCLK=GPIO%d BCLK=GPIO%d LRCK=GPIO%d DIN=GPIO%d DOUT=GPIO%d",
             I2S_MCLK_GPIO, I2S_BCLK_GPIO, I2S_LRCK_GPIO, I2S_DIN_GPIO, I2S_DOUT_GPIO);
    return ESP_OK;
}

/* Continuously stream a quiet sine tone to the speaker (runs in its own task). */
static void tone_task(void *arg)
{
    i2s_chan_handle_t tx = (i2s_chan_handle_t)arg;
    static int16_t buf[TX_CHUNK_FRAMES * 2];
    const double phase_inc = 2.0 * M_PI * TONE_FREQ_HZ / SAMPLE_RATE_HZ;
    double phase = 0.0;

    ESP_LOGI(TAG, "tone task started: %d Hz sine at amplitude %d", TONE_FREQ_HZ, TONE_AMPLITUDE);
    while (1) {
        for (int i = 0; i < TX_CHUNK_FRAMES; i++) {
            int16_t s = (int16_t)(TONE_AMPLITUDE * sin(phase));
            phase += phase_inc;
            if (phase >= 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
            buf[2 * i] = s;
            buf[2 * i + 1] = s;
        }
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx, buf, sizeof(buf), &bytes_written, portMAX_DELAY));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "duplex_test starting");

    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "I2C ready: SDA=GPIO%d SCL=GPIO%d @ %d Hz", I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);

    ESP_ERROR_CHECK(amp_enable());
    ESP_ERROR_CHECK(es8311_init());
    ESP_ERROR_CHECK(es7210_init());

    i2s_chan_handle_t tx_handle = NULL;
    i2s_chan_handle_t rx_handle = NULL;
    ESP_ERROR_CHECK(i2s_duplex_init(&tx_handle, &rx_handle));

    /* Let clocks and both codecs settle before measuring. */
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "DUPLEX TEST RUNNING - tone playing, clap over it");
    xTaskCreate(tone_task, "tone", 4096, tx_handle, 5, NULL);

    /* Mic VU meter (same as firmware/mic_test). */
    static int16_t buf[RX_CHUNK_FRAMES * 2];
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
