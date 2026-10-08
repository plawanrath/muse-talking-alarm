/*
 * Copyright 2026 Plawan Kumar Rath
 * SPDX-License-Identifier: Apache-2.0
 *
 * Speaker test-tone firmware for the Waveshare ESP32-S3-AUDIO-Board.
 *
 * Bring-up step: verify the speaker path end to end.
 *   1. Enable the speaker amplifier via the TCA9555 I/O expander (EXIO8, active high).
 *   2. Initialize the ES8311 DAC (16-bit, 16 kHz, MCLK = 256 * 16 kHz = 4.096 MHz).
 *   3. Stream a test pattern over I2S TX: three 440 Hz beeps, then one 880 Hz beep.
 *
 * Register sequence is taken from the vendor demo for this board
 * (ESP32-S3-AUDIO-Board-Demo, Arduino LVGL_Arduino example: 16 kHz sample rate,
 * MCLK multiple 256) and from Espressif's espressif/es8311 component driver
 * (Apache-2.0, esp-bsp), which is what the vendor demo itself builds on.
 *
 * Pin map (Waveshare ESP32-S3-AUDIO-Board):
 *   I2C:  SDA = GPIO11, SCL = GPIO10, 100 kHz
 *   I2S:  MCLK = GPIO12, BCLK = GPIO13, LRCK = GPIO14,
 *         DOUT (DAC data in) = GPIO16
 *   TCA9555 @ 0x20, ES8311 @ 0x18
 */

#include <math.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_test";

/* ------------------------------- I2C bus -------------------------------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    11
#define I2C_SCL_GPIO    10
#define I2C_FREQ_HZ     100000

#define TCA9555_ADDR    0x20
#define ES8311_ADDR     0x18

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
#define I2S_DOUT_GPIO   16
#define SAMPLE_RATE_HZ  16000
#define MCLK_MULT       256   /* MCLK = 16 kHz * 256 = 4.096 MHz */

/* ------------------------------ test pattern ---------------------------- */
#define BEEP_FREQ_HZ        440
#define BEEP_FINAL_FREQ_HZ  880
#define BEEP_MS             250
#define BEEP_GAP_MS         150
#define FINAL_BEEP_MS       400
#define TONE_AMPLITUDE      7000  /* modest: ~21% of full scale */

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t tca9555;
static i2c_master_dev_handle_t es8311;

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
 * Mirrors espressif/es8311 es8311_init() with the coefficient row
 * {mclk=4096000, rate=16000, pre_div=1, pre_multi=0, adc_div=1, dac_div=1,
 *  fs_mode=0, lrck_h=0, lrck_l=0xFF, bclk_div=4, adc_osr=0x10, dac_osr=0x10}.
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

static esp_err_t i2s_tx_init(i2s_chan_handle_t *tx_handle)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, tx_handle, NULL), TAG, "i2s channel create failed");

    i2s_std_config_t std_cfg = {
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
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256; /* 16 kHz * 256 = 4.096 MHz */

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(*tx_handle, &std_cfg), TAG, "i2s std mode init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(*tx_handle), TAG, "i2s channel enable failed");

    ESP_LOGI(TAG, "I2S TX ready: 16 kHz 16-bit stereo, MCLK=GPIO%d BCLK=GPIO%d LRCK=GPIO%d DOUT=GPIO%d",
             I2S_MCLK_GPIO, I2S_BCLK_GPIO, I2S_LRCK_GPIO, I2S_DOUT_GPIO);
    return ESP_OK;
}

/* Stream `ms` milliseconds of a sine at `freq_hz` (mono duplicated to both channels). */
static void play_tone(i2s_chan_handle_t tx, int freq_hz, int ms)
{
    const int total_frames = SAMPLE_RATE_HZ * ms / 1000;
    const int chunk_frames = 512;
    static int16_t buf[512 * 2];
    const double phase_inc = 2.0 * M_PI * freq_hz / SAMPLE_RATE_HZ;
    double phase = 0.0;
    int done = 0;

    while (done < total_frames) {
        int n = total_frames - done > chunk_frames ? chunk_frames : total_frames - done;
        for (int i = 0; i < n; i++) {
            int16_t s = (int16_t)(TONE_AMPLITUDE * sin(phase));
            phase += phase_inc;
            buf[2 * i] = s;
            buf[2 * i + 1] = s;
        }
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx, buf, n * 2 * sizeof(int16_t), &bytes_written, portMAX_DELAY));
        done += n;
    }
}

/* Stream `ms` milliseconds of silence (lets the beeps separate cleanly). */
static void play_silence(i2s_chan_handle_t tx, int ms)
{
    const int total_frames = SAMPLE_RATE_HZ * ms / 1000;
    const int chunk_frames = 512;
    static int16_t buf[512 * 2] = { 0 };
    int done = 0;

    while (done < total_frames) {
        int n = total_frames - done > chunk_frames ? chunk_frames : total_frames - done;
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx, buf, n * 2 * sizeof(int16_t), &bytes_written, portMAX_DELAY));
        done += n;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "audio_test starting");

    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "I2C ready: SDA=GPIO%d SCL=GPIO%d @ %d Hz", I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);

    ESP_ERROR_CHECK(amp_enable());
    ESP_ERROR_CHECK(es8311_init());

    i2s_chan_handle_t tx_handle = NULL;
    ESP_ERROR_CHECK(i2s_tx_init(&tx_handle));

    ESP_LOGI(TAG, "playing test pattern: 3x %d Hz beeps, then 1x %d Hz beep", BEEP_FREQ_HZ, BEEP_FINAL_FREQ_HZ);
    for (int i = 0; i < 3; i++) {
        play_tone(tx_handle, BEEP_FREQ_HZ, BEEP_MS);
        play_silence(tx_handle, BEEP_GAP_MS);
    }
    play_tone(tx_handle, BEEP_FINAL_FREQ_HZ, FINAL_BEEP_MS);

    ESP_LOGI(TAG, "TEST DONE - idling");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
