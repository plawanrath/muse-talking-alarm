/*
 * Copyright 2026 Plawan Kumar Rath
 * SPDX-License-Identifier: Apache-2.0
 *
 * Talking alarm clock for the Waveshare ESP32-S3-AUDIO-Board.
 *
 * Phase 1 application: RTC-based daily alarms with an escalating chime,
 * a serial command line, and NVS persistence.
 *
 * Hardware (all verified during bring-up):
 *   I2C:  SDA = GPIO11, SCL = GPIO10, 100 kHz
 *   I2S:  MCLK = GPIO12, BCLK = GPIO13, LRCK = GPIO14, DOUT = GPIO16
 *   TCA9555 @ 0x20 (EXIO8 = amp enable, active high)
 *   ES8311  @ 0x18 (DAC, 16-bit, 16 kHz, slave, MCLK 4.096 MHz)
 *   PCF85063 @ 0x51 (RTC; polled, alarm interrupt pin not used)
 *
 * ES8311 init mirrors espressif/es8311 es8311_init() (Apache-2.0, esp-bsp)
 * exactly as proven by firmware/audio_test. PCF85063 register map is from
 * the NXP PCF85063A datasheet (Control_1 0x00, seconds 0x04 .. year 0x0A,
 * all BCD; bit 7 of seconds is the oscillator-stop flag).
 */

#include <ctype.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "alarm_clock";

/* ------------------------------- I2C bus -------------------------------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    11
#define I2C_SCL_GPIO    10
#define I2C_FREQ_HZ     100000

#define TCA9555_ADDR    0x20
#define ES8311_ADDR     0x18
#define PCF85063_ADDR   0x51

#define TCA9555_OUT_PORT1   0x03
#define TCA9555_CFG_PORT1   0x07
#define AMP_ENABLE_BIT      0   /* EXIO8 = bit 0 of port 1, active high */

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

/* ------------------------------ PCF85063 -------------------------------- */
#define RTC_REG_CTRL1   0x00
#define RTC_REG_CTRL2   0x01
#define RTC_REG_SEC     0x04
#define RTC_OS_FLAG     0x80  /* bit 7 of seconds: oscillator stopped */

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t tca9555;
static i2c_master_dev_handle_t es8311;
static i2c_master_dev_handle_t pcf85063;
static i2s_chan_handle_t i2s_tx;

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

    i2c_device_config_t rtc_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF85063_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &rtc_cfg, &pcf85063), TAG, "pcf85063 add failed");

    return ESP_OK;
}

/* Drive TCA9555 EXIO8 high. Matches the vendor demo (Set_EXIO(EXIO8, true)). */
static esp_err_t amp_enable(void)
{
    uint8_t reg;

    ESP_RETURN_ON_ERROR(i2c_reg_read(tca9555, TCA9555_CFG_PORT1, &reg), TAG, "tca9555 cfg read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(tca9555, TCA9555_CFG_PORT1, reg & ~(1 << AMP_ENABLE_BIT)),
                        TAG, "tca9555 cfg write failed");

    ESP_RETURN_ON_ERROR(i2c_reg_read(tca9555, TCA9555_OUT_PORT1, &reg), TAG, "tca9555 out read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(tca9555, TCA9555_OUT_PORT1, reg | (1 << AMP_ENABLE_BIT)),
                        TAG, "tca9555 out write failed");

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "amplifier enabled (TCA9555 EXIO8 = high)");
    return ESP_OK;
}

/*
 * ES8311 init for 16 kHz / 16-bit / I2S slave, MCLK = 4.096 MHz.
 * Verbatim from firmware/audio_test (verified on the board).
 */
static esp_err_t es8311_init(void)
{
    uint8_t reg;

    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, 0x1F), TAG, "reset1 failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, 0x00), TAG, "reset2 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, 0x80), TAG, "power-on failed");

    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG01, 0x3F), TAG, "reg01 failed");

    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG06, &reg), TAG, "reg06 read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG06, reg & ~0x20), TAG, "reg06 failed");

    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG02, &reg), TAG, "reg02 read failed");
    reg &= 0x07;
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG02, reg), TAG, "reg02 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG03, 0x10), TAG, "reg03 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG04, 0x10), TAG, "reg04 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG05, 0x00), TAG, "reg05 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG06, &reg), TAG, "reg06 read failed");
    reg &= 0xE0;
    reg |= (4 - 1);
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG06, reg), TAG, "reg06 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_CLK_MGR_REG07, &reg), TAG, "reg07 read failed");
    reg &= 0xC0;
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG07, reg), TAG, "reg07 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_CLK_MGR_REG08, 0xFF), TAG, "reg08 failed");

    ESP_RETURN_ON_ERROR(i2c_reg_read(es8311, ES8311_RESET_REG00, &reg), TAG, "reg00 read failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_RESET_REG00, reg & 0xBF), TAG, "reg00 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SDPIN_REG09, 0x0C), TAG, "reg09 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SDPOUT_REG0A, 0x0C), TAG, "reg0a failed");

    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG0D, 0x01), TAG, "reg0d failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG0E, 0x02), TAG, "reg0e failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG12, 0x00), TAG, "reg12 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_SYSTEM_REG13, 0x10), TAG, "reg13 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_ADC_REG1C, 0x6A), TAG, "reg1c failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_DAC_REG37, 0x08), TAG, "reg37 failed");

    ESP_RETURN_ON_ERROR(i2c_reg_write(es8311, ES8311_DAC_REG32, 0x7F), TAG, "volume failed");

    ESP_LOGI(TAG, "ES8311 initialized: 16-bit, 16 kHz, slave, MCLK 4.096 MHz, volume 50");
    return ESP_OK;
}

static esp_err_t i2s_tx_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &i2s_tx, NULL), TAG, "i2s channel create failed");

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
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(i2s_tx, &std_cfg), TAG, "i2s std mode init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(i2s_tx), TAG, "i2s channel enable failed");

    ESP_LOGI(TAG, "I2S TX ready");
    return ESP_OK;
}

/* ------------------------------ RTC helpers ----------------------------- */
typedef struct {
    int year, month, day, hour, min, sec;
    bool os_flag;   /* oscillator stopped at some point -> time untrustworthy */
} rtc_time_t;

static uint8_t to_bcd(int v)   { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static int from_bcd(uint8_t b) { return ((b >> 4) * 10) + (b & 0x0F); }
static bool is_leap(int y)     { return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0); }

/* Weekday with 0 = Sunday. 2000-01-01 was a Saturday. */
static int weekday_0sun(int y, int m, int d)
{
    static const int mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    int days = 0;
    for (int yy = 2000; yy < y; yy++) {
        days += 365 + (is_leap(yy) ? 1 : 0);
    }
    for (int mm = 1; mm < m; mm++) {
        days += mdays[mm - 1];
        if (mm == 2 && is_leap(y)) {
            days++;
        }
    }
    days += d - 1;
    return (6 + days) % 7;
}

static esp_err_t rtc_read(rtc_time_t *t)
{
    uint8_t reg = RTC_REG_SEC, buf[7];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pcf85063, &reg, 1, buf, sizeof(buf), 1000),
                        TAG, "rtc read failed");
    t->os_flag = (buf[0] & RTC_OS_FLAG) != 0;
    t->sec   = from_bcd(buf[0] & 0x7F);
    t->min   = from_bcd(buf[1] & 0x7F);
    t->hour  = from_bcd(buf[2] & 0x3F);
    t->day   = from_bcd(buf[3] & 0x3F);
    /* buf[4] is weekday, not needed */
    t->month = from_bcd(buf[5] & 0x1F);
    t->year  = 2000 + from_bcd(buf[6]);
    return ESP_OK;
}

static esp_err_t rtc_write(const rtc_time_t *t)
{
    /* Make sure the clock is running (STOP bit = 0) and alarms are off. */
    ESP_RETURN_ON_ERROR(i2c_reg_write(pcf85063, RTC_REG_CTRL1, 0x00), TAG, "rtc ctrl1 failed");
    ESP_RETURN_ON_ERROR(i2c_reg_write(pcf85063, RTC_REG_CTRL2, 0x00), TAG, "rtc ctrl2 failed");

    uint8_t buf[8];
    buf[0] = RTC_REG_SEC;
    buf[1] = to_bcd(t->sec) & 0x7F;   /* bit 7 = 0 clears the OS flag */
    buf[2] = to_bcd(t->min);
    buf[3] = to_bcd(t->hour);
    buf[4] = to_bcd(t->day);
    buf[5] = (uint8_t)weekday_0sun(t->year, t->month, t->day);
    buf[6] = to_bcd(t->month);
    buf[7] = to_bcd(t->year - 2000);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pcf85063, buf, sizeof(buf), 1000), TAG, "rtc write failed");
    return ESP_OK;
}

static bool rtc_valid(const rtc_time_t *t)
{
    if (t->os_flag) {
        return false;
    }
    return t->year >= 2026 && t->year <= 2099 && t->month >= 1 && t->month <= 12 &&
           t->day >= 1 && t->day <= 31 && t->hour <= 23 && t->min <= 59 && t->sec <= 59;
}

/* Monotonic-ish minute key for "already fired" / snooze bookkeeping. */
static uint32_t minute_key(const rtc_time_t *t)
{
    return (uint32_t)(((t->year * 372 + t->month * 31 + t->day) * 1440) + t->hour * 60 + t->min);
}

/* ------------------------------- Alarms --------------------------------- */
#define MAX_ALARMS 8

typedef struct {
    uint8_t hour, min;
    char label[32];
    uint32_t fired_key;   /* minute_key of the last firing, 0 = never */
} alarm_t;

static alarm_t alarms[MAX_ALARMS];
static int alarm_count = 0;
static SemaphoreHandle_t state_mutex;

static bool ringing = false;
static int ringing_idx = -1;
static char ringing_label[40];
static uint32_t ring_start_key = 0;
static uint32_t ring_elapsed_ds = 0;  /* deciseconds, for the volume ramp */
static bool snooze_armed = false;
static bool snooze_use_rtc = true;   /* false: deadline is boot-relative */
static uint32_t snooze_until_key = 0;
static uint64_t snooze_until_us = 0;
static char snooze_label[40];
static int snooze_idx = -1;

static nvs_handle_t nvs_h;

static void alarms_save(void)
{
    nvs_set_u8(nvs_h, "count", (uint8_t)alarm_count);
    for (int i = 0; i < MAX_ALARMS; i++) {
        char key[8], val[40];
        snprintf(key, sizeof(key), "a%d", i);
        if (i < alarm_count) {
            snprintf(val, sizeof(val), "%02d:%02d;%s", alarms[i].hour, alarms[i].min, alarms[i].label);
            nvs_set_str(nvs_h, key, val);
        } else {
            nvs_erase_key(nvs_h, key);
        }
    }
    nvs_commit(nvs_h);
}

static void alarms_load(void)
{
    uint8_t count = 0;
    alarm_count = 0;
    if (nvs_get_u8(nvs_h, "count", &count) != ESP_OK || count > MAX_ALARMS) {
        return;
    }
    for (int i = 0; i < count; i++) {
        char key[8], val[40];
        size_t len = sizeof(val);
        int h, m;
        snprintf(key, sizeof(key), "a%d", i);
        if (nvs_get_str(nvs_h, key, val, &len) != ESP_OK) {
            continue;
        }
        if (sscanf(val, "%2d:%2d", &h, &m) != 2 || h < 0 || h > 23 || m < 0 || m > 59) {
            continue;
        }
        alarms[alarm_count].hour = (uint8_t)h;
        alarms[alarm_count].min = (uint8_t)m;
        alarms[alarm_count].fired_key = 0;
        const char *semi = strchr(val, ';');
        if (semi && *(semi + 1)) {
            snprintf(alarms[alarm_count].label, sizeof(alarms[alarm_count].label), "%s", semi + 1);
        } else {
            alarms[alarm_count].label[0] = '\0';
        }
        alarm_count++;
    }
}

/* --------------------------- Countdown timers --------------------------- */
/*
 * Timers count down from now using esp_timer (boot-relative, monotonic;
 * no RTC needed). Persisted in NVS as remaining seconds + wall-clock
 * target epoch (when the RTC is valid); on boot, timers whose target
 * already passed are dropped.
 */
#define MAX_TIMERS 8

typedef struct {
    uint32_t duration_s;   /* original duration, for display */
    uint64_t target_us;    /* esp_timer_get_time() deadline, boot-relative */
    char label[32];
    bool active;
} alarm_timer_t;

static alarm_timer_t timers[MAX_TIMERS];
static int timer_count = 0;  /* compacted: timers[0..timer_count) are live */

static uint64_t rtc_epoch_s(const rtc_time_t *t)
{
    static const int mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    uint64_t days = 0;
    for (int y = 2000; y < t->year; y++) {
        days += 365 + (is_leap(y) ? 1 : 0);
    }
    for (int m = 1; m < t->month; m++) {
        days += mdays[m - 1];
        if (m == 2 && is_leap(t->year)) {
            days++;
        }
    }
    days += (uint64_t)(t->day - 1);
    return days * 86400ULL + (uint64_t)t->hour * 3600ULL + (uint64_t)t->min * 60ULL + (uint64_t)t->sec;
}

static void timers_save(void)
{
    nvs_set_u8(nvs_h, "tcount", (uint8_t)timer_count);
    rtc_time_t now;
    bool rtc_ok = (rtc_read(&now) == ESP_OK && rtc_valid(&now));
    uint64_t now_us = esp_timer_get_time();
    for (int i = 0; i < MAX_TIMERS; i++) {
        char key[8];
        snprintf(key, sizeof(key), "t%d", i);
        if (i < timer_count && timers[i].active) {
            uint64_t rem_us = timers[i].target_us > now_us ? timers[i].target_us - now_us : 0;
            uint32_t rem_s = (uint32_t)(rem_us / 1000000ULL);
            uint64_t target_epoch = rtc_ok ? rtc_epoch_s(&now) + rem_s : 0;
            char val[72];
            snprintf(val, sizeof(val), "%lu;%llu;%s",
                     rem_s, (unsigned long long)target_epoch, timers[i].label);
            nvs_set_str(nvs_h, key, val);
        } else {
            nvs_erase_key(nvs_h, key);
        }
    }
    nvs_commit(nvs_h);
}

static void timers_load(void)
{
    timer_count = 0;
    uint8_t count = 0;
    if (nvs_get_u8(nvs_h, "tcount", &count) != ESP_OK || count > MAX_TIMERS) {
        return;
    }
    rtc_time_t now;
    bool rtc_ok = (rtc_read(&now) == ESP_OK && rtc_valid(&now));
    uint64_t now_epoch = rtc_ok ? rtc_epoch_s(&now) : 0;
    uint64_t now_us = esp_timer_get_time();
    for (int i = 0; i < count; i++) {
        char key[8], val[72];
        size_t len = sizeof(val);
        snprintf(key, sizeof(key), "t%d", i);
        if (nvs_get_str(nvs_h, key, val, &len) != ESP_OK) {
            continue;
        }
        char *p1 = strchr(val, ';');
        if (!p1) {
            continue;
        }
        char *p2 = strchr(p1 + 1, ';');
        unsigned rem = (unsigned)atoi(val);
        unsigned long long target_epoch = strtoull(p1 + 1, NULL, 10);
        uint32_t use_rem = 0;
        if (rtc_ok && target_epoch > 0) {
            if (target_epoch <= now_epoch) {
                continue;  /* already passed while powered off: drop */
            }
            uint64_t r = target_epoch - now_epoch;
            if (r > 2 * 86400ULL) {
                r = 2 * 86400ULL;
            }
            use_rem = (uint32_t)r;
        } else if (!rtc_ok) {
            continue;  /* no trustworthy clock: drop rather than misfire */
        } else {
            use_rem = rem;  /* no epoch was stored; best effort */
        }
        if (use_rem == 0) {
            continue;
        }
        timers[timer_count].duration_s = rem;
        timers[timer_count].target_us = now_us + (uint64_t)use_rem * 1000000ULL;
        timers[timer_count].active = true;
        if (p2 && *(p2 + 1)) {
            snprintf(timers[timer_count].label, sizeof(timers[timer_count].label), "%s", p2 + 1);
        } else {
            timers[timer_count].label[0] = '\0';
        }
        timer_count++;
    }
}

/* ------------------------------ Chime engine ---------------------------- */
/*
 * Gentle escalating two-tone chime. Continuous phase across notes plus a
 * short fade at note boundaries keeps it click-free. Amplitude ramps from
 * soft to firm over ~90 s so the first notes never startle.
 */
static const struct { int freq_hz; int ms; } chime_motif[] = {
    { 784, 300 }, { 659, 300 }, { 0, 200 },
    { 784, 300 }, { 880, 450 }, { 0, 300 },
};
#define CHIME_MOTIF_LEN (sizeof(chime_motif) / sizeof(chime_motif[0]))

#define CHIME_CHUNK_MS   100
#define CHIME_FRAMES     (SAMPLE_RATE_HZ * CHIME_CHUNK_MS / 1000)
static int16_t chime_buf[CHIME_FRAMES * 2];

static double chime_phase = 0.0;
static int chime_note = 0;
static int chime_note_frame = 0;   /* frames elapsed in current note */

static void chime_reset(void)
{
    chime_phase = 0.0;
    chime_note = 0;
    chime_note_frame = 0;
}

/* Fill one 100 ms chunk. amp is 0..32767 linear amplitude. */
static void chime_fill(double amp)
{
    const int fade_frames = SAMPLE_RATE_HZ * 6 / 1000;  /* 6 ms fades */
    for (int i = 0; i < CHIME_FRAMES; i++) {
        int note_frames = chime_motif[chime_note].ms * SAMPLE_RATE_HZ / 1000;
        if (note_frames <= 0) {
            note_frames = 1;
        }
        if (chime_note_frame >= note_frames) {
            chime_note = (chime_note + 1) % CHIME_MOTIF_LEN;
            chime_note_frame = 0;
            note_frames = chime_motif[chime_note].ms * SAMPLE_RATE_HZ / 1000;
            if (note_frames <= 0) {
                note_frames = 1;
            }
        }
        int f = chime_motif[chime_note].freq_hz;
        int head = chime_note_frame;
        int tail = note_frames - chime_note_frame;
        int edge = head < tail ? head : tail;
        double env = 1.0;
        if (edge < fade_frames) {
            env = (double)edge / (double)fade_frames;
        }
        int16_t s = 0;
        if (f > 0) {
            s = (int16_t)(amp * env * sin(chime_phase));
            chime_phase += 2.0 * M_PI * f / SAMPLE_RATE_HZ;
            if (chime_phase > 2.0 * M_PI) {
                chime_phase -= 2.0 * M_PI;
            }
        }
        chime_buf[2 * i] = s;
        chime_buf[2 * i + 1] = s;
        chime_note_frame++;
    }
}

static void chime_write_chunk(void)
{
    size_t written = 0;
    ESP_ERROR_CHECK(i2s_channel_write(i2s_tx, chime_buf, sizeof(chime_buf), &written, portMAX_DELAY));
}

/* Short fade-out so `stop` never ends on a click. */
static void chime_fade_out(double amp)
{
    const int frames = SAMPLE_RATE_HZ * 50 / 1000;
    static int16_t fade_buf[800 * 2];  /* 50 ms @ 16 kHz stereo */
    for (int i = 0; i < frames; i++) {
        double env = 1.0 - (double)i / (double)frames;
        int16_t s = (int16_t)(amp * env * sin(chime_phase));
        chime_phase += 2.0 * M_PI * 784.0 / SAMPLE_RATE_HZ;
        if (chime_phase > 2.0 * M_PI) {
            chime_phase -= 2.0 * M_PI;
        }
        fade_buf[2 * i] = s;
        fade_buf[2 * i + 1] = s;
    }
    size_t written = 0;
    ESP_ERROR_CHECK(i2s_channel_write(i2s_tx, fade_buf, frames * 2 * sizeof(int16_t), &written, portMAX_DELAY));
}

static double chime_amp_for_elapsed(uint32_t elapsed_ds)
{
    /* 1200 (~4%) -> 8800 (~27%) over 90 s. */
    double k = (double)elapsed_ds / 900.0;
    if (k > 1.0) {
        k = 1.0;
    }
    return 1200.0 + k * 7600.0;
}

/* ------------------------------ Alarm task ------------------------------ */
#define RING_MAX_MIN 5  /* auto-stop a ringing alarm after 5 minutes */

static void start_ringing_locked(int idx, const char *label, uint32_t key)
{
    ringing = true;
    ringing_idx = idx;
    ring_start_key = key;
    ring_elapsed_ds = 0;
    snprintf(ringing_label, sizeof(ringing_label), "%s", label && *label ? label : "alarm");
    chime_reset();
    printf("ALARM RINGING: %s (commands: stop | snooze [minutes])\n", ringing_label);
    fflush(stdout);
}

/* Fade-out bookkeeping: only the alarm task ever writes to I2S, so a
 * stop/snooze request just arms the fade; the alarm task performs it. */
static bool fadeout_pending = false;
static double fadeout_amp = 0.0;

/* Call with state_mutex held. Marks ringing stopped; the alarm task
 * performs the actual fade-out on its next pass. */
static void request_stop_locked(double last_amp)
{
    if (!ringing) {
        return;
    }
    ringing = false;
    ringing_idx = -1;
    snooze_armed = false;
    if (last_amp > 10.0) {
        fadeout_pending = true;
        fadeout_amp = last_amp;
    }
}

static void alarm_task(void *arg)
{
    (void)arg;
    int tick = 0;

    while (1) {
        bool do_poll = (tick % 10 == 0);
        tick++;

        bool is_ringing;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        is_ringing = ringing;
        xSemaphoreGive(state_mutex);

        if (is_ringing) {
            double amp;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            amp = chime_amp_for_elapsed(ring_elapsed_ds);
            ring_elapsed_ds++;
            bool too_long = (ring_elapsed_ds > RING_MAX_MIN * 600);
            xSemaphoreGive(state_mutex);
            chime_fill(amp);
            chime_write_chunk();
            if (too_long) {
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                printf("alarm auto-stopped after %d minutes\n", RING_MAX_MIN);
                request_stop_locked(amp);
                xSemaphoreGive(state_mutex);
            }
        } else {
            /* Perform any pending fade-out here: single I2S writer. */
            bool do_fade = false;
            double fa = 0.0;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            do_fade = fadeout_pending;
            fa = fadeout_amp;
            fadeout_pending = false;
            xSemaphoreGive(state_mutex);
            if (do_fade) {
                chime_fade_out(fa);
                /* Push silence through the I2S DMA pipeline so it can't
                 * keep replaying the last audio buffer (heard as beeping). */
                memset(chime_buf, 0, sizeof(chime_buf));
                chime_write_chunk();
                chime_write_chunk();
            }
        }

        if (do_poll && !is_ringing) {
            rtc_time_t t;
            bool have_time = (rtc_read(&t) == ESP_OK && rtc_valid(&t));
            uint32_t key = have_time ? minute_key(&t) : 0;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            /* Timers need no RTC: check expiries first. */
            uint64_t now_us = esp_timer_get_time();
            for (int i = 0; i < timer_count; i++) {
                if (timers[i].active && now_us >= timers[i].target_us) {
                    char lbl[40];
                    if (timers[i].label[0]) {
                        snprintf(lbl, sizeof(lbl), "timer: %s", timers[i].label);
                    } else {
                        snprintf(lbl, sizeof(lbl), "timer %lus done", timers[i].duration_s);
                    }
                    for (int j = i; j < timer_count - 1; j++) {
                        timers[j] = timers[j + 1];
                    }
                    timer_count--;
                    timers_save();
                    start_ringing_locked(-1, lbl, key);
                    break;
                }
            }
            if (!ringing && snooze_armed) {
                bool due;
                if (snooze_use_rtc) {
                    due = have_time && key >= snooze_until_key;
                } else {
                    due = (int64_t)(esp_timer_get_time() - snooze_until_us) >= 0;
                }
                if (due) {
                    snooze_armed = false;
                    start_ringing_locked(snooze_idx, snooze_label, key);
                }
            }
            if (have_time && !ringing && !snooze_armed) {
                for (int i = 0; i < alarm_count; i++) {
                    if (alarms[i].hour == t.hour && alarms[i].min == t.min &&
                        alarms[i].fired_key != key) {
                        alarms[i].fired_key = key;
                        char lbl[40];
                        if (alarms[i].label[0]) {
                            snprintf(lbl, sizeof(lbl), "%s", alarms[i].label);
                        } else {
                            snprintf(lbl, sizeof(lbl), "%02d:%02d", alarms[i].hour, alarms[i].min);
                        }
                        start_ringing_locked(i, lbl, key);
                        break;
                    }
                }
            }
            xSemaphoreGive(state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(CHIME_CHUNK_MS));
    }
}

/* ------------------------------- Console -------------------------------- */
static void print_prompt(void)
{
    printf("alarm> ");
    fflush(stdout);
}

/* ------------------------------ WiFi + NTP ------------------------------ */
/* WiFi credentials live in NVS on the device only; they never go into the
 * repo. Set once via the console:  wifi set <ssid> <password>
 * On boot the wifi task connects, syncs time via NTP, and writes it to the
 * RTC so alarms keep working across power loss. Re-syncs hourly. */

#define WIFI_NVS_NS "wifi"
#define WIFI_GOT_IP_BIT BIT0

static EventGroupHandle_t wifi_events;
static TaskHandle_t wifi_task_handle;
static bool wifi_connected;
static time_t wifi_last_sync;

static bool wifi_creds_load(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz)
{
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t n = ssid_sz;
    bool ok = (nvs_get_str(h, "ssid", ssid, &n) == ESP_OK && ssid[0] != '\0');
    n = pass_sz;
    ok = ok && (nvs_get_str(h, "pass", pass, &n) == ESP_OK);
    nvs_close(h);
    return ok;
}

static esp_err_t wifi_creds_save(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(WIFI_NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        return r;
    }
    r = nvs_set_str(h, "ssid", ssid);
    if (r == ESP_OK) {
        r = nvs_set_str(h, "pass", pass);
    }
    if (r == ESP_OK) {
        r = nvs_commit(h);
    }
    nvs_close(h);
    return r;
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        wifi_connected = false;
        xSemaphoreGive(state_mutex);
        ESP_LOGI(TAG, "wifi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        wifi_connected = true;
        xSemaphoreGive(state_mutex);
        xEventGroupSetBits(wifi_events, WIFI_GOT_IP_BIT);
        ESP_LOGI(TAG, "wifi got IP");
    }
}

/* Write the current system time (Pacific wall clock) into the RTC. */
static void rtc_sync_from_system(void)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    rtc_time_t t = {
        .year = tm.tm_year + 1900,
        .month = tm.tm_mon + 1,
        .day = tm.tm_mday,
        .hour = tm.tm_hour,
        .min = tm.tm_min,
        .sec = tm.tm_sec,
        .os_flag = false,
    };
    if (rtc_write(&t) == ESP_OK) {
        wifi_last_sync = now;
        ESP_LOGI(TAG, "RTC synced from NTP: %04d-%02d-%02d %02d:%02d:%02d",
                 t.year, t.month, t.day, t.hour, t.min, t.sec);
    } else {
        ESP_LOGE(TAG, "RTC write failed during NTP sync");
    }
}

static bool sntp_sync_wait(uint32_t timeout_ms)
{
    uint32_t waited = 0;
    while (waited < timeout_ms) {
        if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    return false;
}

static void wifi_task(void *arg)
{
    (void)arg;
    char ssid[33], pass[65];

    if (!wifi_creds_load(ssid, sizeof(ssid), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "no wifi credentials; use: wifi set <ssid> <password>");
        wifi_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, pass, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    bool sntp_started = false;
    for (;;) {
        xEventGroupWaitBits(wifi_events, WIFI_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        if (!sntp_started) {
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            esp_sntp_init();
            sntp_started = true;
        }
        ESP_LOGI(TAG, "waiting for NTP sync...");
        if (sntp_sync_wait(30000)) {
            rtc_sync_from_system();
            printf("time synced from network\n");
            fflush(stdout);
        } else {
            ESP_LOGW(TAG, "NTP sync timed out");
        }
        vTaskDelay(pdMS_TO_TICKS(3600 * 1000));  /* re-sync hourly */
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool up = wifi_connected;
        xSemaphoreGive(state_mutex);
        if (!up) {
            xEventGroupClearBits(wifi_events, WIFI_GOT_IP_BIT);
        }
    }
}

static void cmd_wifi(const char *args)
{
    while (*args && isspace((unsigned char)*args)) {
        args++;
    }
    if (strncmp(args, "set ", 4) == 0) {
        const char *p = args + 4;
        const char *sp = strchr(p, ' ');
        if (!sp || sp == p || *(sp + 1) == '\0') {
            printf("usage: wifi set <ssid> <password>\n");
            return;
        }
        char ssid[33], pass[65];
        size_t sl = (size_t)(sp - p);
        if (sl >= sizeof(ssid)) {
            sl = sizeof(ssid) - 1;
        }
        memcpy(ssid, p, sl);
        ssid[sl] = '\0';
        const char *pw = sp + 1;
        size_t pl = strlen(pw);
        while (pl > 0 && isspace((unsigned char)pw[pl - 1])) {
            pl--;
        }
        if (pl >= sizeof(pass)) {
            pl = sizeof(pass) - 1;
        }
        memcpy(pass, pw, pl);
        pass[pl] = '\0';
        if (wifi_creds_save(ssid, pass) != ESP_OK) {
            printf("failed to save wifi credentials\n");
            return;
        }
        printf("wifi credentials saved for \"%s\"\n", ssid);
        if (wifi_task_handle == NULL) {
            xTaskCreate(wifi_task, "wifi", 4096, NULL, 3, &wifi_task_handle);
            printf("connecting...\n");
        } else {
            printf("press RESET to reconnect with the new credentials\n");
        }
    } else if (strcmp(args, "status") == 0) {
        char ssid[33], pass[65];
        bool has = wifi_creds_load(ssid, sizeof(ssid), pass, sizeof(pass));
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool up = wifi_connected;
        time_t last = wifi_last_sync;
        xSemaphoreGive(state_mutex);
        if (!has) {
            printf("wifi: not configured (use: wifi set <ssid> <password>)\n");
            return;
        }
        printf("wifi: configured for \"%s\", %s\n", ssid, up ? "connected" : "not connected");
        if (last != 0) {
            struct tm tm;
            localtime_r(&last, &tm);
            printf("last NTP sync: %04d-%02d-%02d %02d:%02d:%02d\n",
                   tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                   tm.tm_hour, tm.tm_min, tm.tm_sec);
        } else {
            printf("last NTP sync: never\n");
        }
    } else if (strcmp(args, "forget") == 0) {
        nvs_handle_t h;
        if (nvs_open(WIFI_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_all(h);
            nvs_commit(h);
            nvs_close(h);
        }
        printf("wifi credentials erased (press RESET to disconnect)\n");
    } else {
        printf("usage:\n"
               "  wifi set <ssid> <password>\n"
               "  wifi status\n"
               "  wifi forget\n");
    }
}

/* ------------------------------ Console ------------------------------ */

static void cmd_help(void)
{
    printf("commands:\n"
           "  time                        show RTC time\n"
           "  settime YYYY-MM-DD HH:MM:SS set the RTC\n"
           "  wifi set <ssid> <pass>      save WiFi credentials and connect\n"
           "  wifi status                 show WiFi and NTP sync status\n"
           "  wifi forget                 erase saved WiFi credentials\n"
           "  alarm add HH:MM [label]     add a daily alarm\n"
           "  alarm list                  list alarms\n"
           "  alarm del N                 delete alarm N\n"
           "  timer <Nm|Nh|Ns> [label]    countdown timer (e.g. timer 10m pasta)\n"
           "  timer list                  list timers with remaining time\n"
           "  timer cancel N              cancel timer N\n"
           "  stop                        silence a ringing alarm\n"
           "  snooze [minutes]            silence and re-ring (default 9 min)\n"
           "  help                        this list\n");
}

static void cmd_time(void)
{
    rtc_time_t t;
    if (rtc_read(&t) != ESP_OK) {
        printf("RTC read failed\n");
        return;
    }
    printf("RTC: %04d-%02d-%02d %02d:%02d:%02d%s\n",
           t.year, t.month, t.day, t.hour, t.min, t.sec,
           rtc_valid(&t) ? "" : "  (NOT SET)");
}

static void cmd_settime(const char *args)
{
    rtc_time_t t = { 0 };
    int n = sscanf(args, "%d-%d-%d %d:%d:%d",
                   &t.year, &t.month, &t.day, &t.hour, &t.min, &t.sec);
    if (n != 6 || t.year < 2026 || t.year > 2099 || t.month < 1 || t.month > 12 ||
        t.day < 1 || t.day > 31 || t.hour > 23 || t.min > 59 || t.sec > 59) {
        printf("usage: settime YYYY-MM-DD HH:MM:SS  (year 2026-2099)\n");
        return;
    }
    if (rtc_write(&t) != ESP_OK) {
        printf("RTC write failed\n");
        return;
    }
    printf("RTC set to %04d-%02d-%02d %02d:%02d:%02d\n",
           t.year, t.month, t.day, t.hour, t.min, t.sec);
}

static void cmd_alarm_add(const char *args)
{
    int h = -1, m = -1;
    char label[32] = "";
    /* Try with label first, then bare HH:MM. */
    int n = sscanf(args, "%2d:%2d %31[^\n]", &h, &m, label);
    if (n < 2) {
        printf("usage: alarm add HH:MM [label]\n");
        return;
    }
    if (h < 0 || h > 23 || m < 0 || m > 59) {
        printf("invalid time (need HH 00-23, MM 00-59)\n");
        return;
    }
    /* Trim leading spaces from the label, if any. */
    char *lp = label;
    while (*lp && isspace((unsigned char)*lp)) {
        lp++;
    }
    if (lp != label) {
        memmove(label, lp, strlen(lp) + 1);
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (alarm_count >= MAX_ALARMS) {
        xSemaphoreGive(state_mutex);
        printf("alarm list full (%d max)\n", MAX_ALARMS);
        return;
    }
    alarms[alarm_count].hour = (uint8_t)h;
    alarms[alarm_count].min = (uint8_t)m;
    alarms[alarm_count].fired_key = 0;
    snprintf(alarms[alarm_count].label, sizeof(alarms[alarm_count].label), "%s", label);
    int idx = alarm_count;
    alarm_count++;
    alarms_save();
    xSemaphoreGive(state_mutex);
    printf("alarm %d added: %02d:%02d%s%s\n", idx, h, m,
           label[0] ? " - " : "", label);
}

static void cmd_alarm_list(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (alarm_count == 0) {
        printf("no alarms\n");
    } else {
        for (int i = 0; i < alarm_count; i++) {
            printf("%d: %02d:%02d%s%s\n", i, alarms[i].hour, alarms[i].min,
                   alarms[i].label[0] ? " - " : "", alarms[i].label);
        }
    }
    xSemaphoreGive(state_mutex);
}

static void cmd_alarm_del(const char *args)
{
    int idx = -1;
    if (sscanf(args, "%d", &idx) != 1) {
        printf("usage: alarm del N\n");
        return;
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (idx < 0 || idx >= alarm_count) {
        xSemaphoreGive(state_mutex);
        printf("no alarm %d\n", idx);
        return;
    }
    for (int i = idx; i < alarm_count - 1; i++) {
        alarms[i] = alarms[i + 1];
    }
    alarm_count--;
    alarms_save();
    xSemaphoreGive(state_mutex);
    printf("alarm %d deleted\n", idx);
}

static void cmd_stop(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!ringing) {
        printf("no alarm ringing\n");
    } else {
        double amp = chime_amp_for_elapsed(ring_elapsed_ds);
        request_stop_locked(amp);
        printf("alarm stopped\n");
    }
    xSemaphoreGive(state_mutex);
}

static void cmd_snooze(const char *args)
{
    int minutes = 9;
    if (*args) {
        if (sscanf(args, "%d", &minutes) != 1 || minutes < 1 || minutes > 120) {
            printf("usage: snooze [minutes 1-120]\n");
            return;
        }
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!ringing) {
        printf("no alarm ringing - nothing to snooze\n");
        xSemaphoreGive(state_mutex);
        return;
    }
    double amp = chime_amp_for_elapsed(ring_elapsed_ds);
    int idx = ringing_idx;
    char rlabel[40];
    snprintf(rlabel, sizeof(rlabel), "%s", ringing_label);
    request_stop_locked(amp);
    rtc_time_t t;
    if (rtc_read(&t) == ESP_OK && rtc_valid(&t)) {
        uint32_t key = minute_key(&t);
        /* Keep this minute marked as fired so the base alarm doesn't retrigger. */
        if (idx >= 0 && idx < alarm_count) {
            alarms[idx].fired_key = key;
        }
        snooze_use_rtc = true;
        snooze_until_key = key + (uint32_t)minutes;
    } else {
        /* No RTC (e.g. a timer rang with the clock unset): snooze
         * boot-relative instead of wall-clock-relative. */
        snooze_use_rtc = false;
        snooze_until_us = esp_timer_get_time() + (uint64_t)minutes * 60ULL * 1000000ULL;
    }
    snprintf(snooze_label, sizeof(snooze_label), "snooze (%.30s)", rlabel);
    snooze_idx = idx;
    snooze_armed = true;
    xSemaphoreGive(state_mutex);
    printf("snoozed for %d minute%s\n", minutes, minutes == 1 ? "" : "s");
}

/* Parse durations like "10m", "90m", "1h30m", "45s". Bare number = minutes. */
static bool parse_duration(const char *s, uint32_t *secs_out)
{
    uint32_t total = 0;
    bool any = false;
    while (*s) {
        if (!isdigit((unsigned char)*s)) {
            return false;
        }
        char *end = NULL;
        unsigned long v = strtoul(s, &end, 10);
        s = end;
        uint32_t mult = 60;  /* bare number means minutes */
        if (*s == 'h' || *s == 'H') {
            mult = 3600;
            s++;
        } else if (*s == 'm' || *s == 'M') {
            mult = 60;
            s++;
        } else if (*s == 's' || *s == 'S') {
            mult = 1;
            s++;
        }
        total += (uint32_t)v * mult;
        any = true;
        if (total > 7 * 86400UL) {
            return false;  /* cap at 7 days */
        }
    }
    *secs_out = total;
    return any && total > 0;
}

static void cmd_timer(const char *args)
{
    while (*args && isspace((unsigned char)*args)) {
        args++;
    }
    if (strncmp(args, "list", 4) == 0 && (args[4] == '\0' || isspace((unsigned char)args[4]))) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (timer_count == 0) {
            printf("no timers\n");
        } else {
            uint64_t now_us = esp_timer_get_time();
            for (int i = 0; i < timer_count; i++) {
                uint64_t rem_us = timers[i].target_us > now_us ? timers[i].target_us - now_us : 0;
                uint32_t rem = (uint32_t)(rem_us / 1000000ULL);
                printf("%d: %02lu:%02lu remaining%s%s\n", i, rem / 60, rem % 60,
                       timers[i].label[0] ? " - " : "", timers[i].label);
            }
        }
        xSemaphoreGive(state_mutex);
        return;
    }
    if (strncmp(args, "cancel", 6) == 0 && (args[6] == ' ' || args[6] == '\0')) {
        int idx = -1;
        const char *rest = args[6] ? args + 7 : "";
        if (sscanf(rest, "%d", &idx) != 1) {
            printf("usage: timer cancel N\n");
            return;
        }
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (idx < 0 || idx >= timer_count) {
            xSemaphoreGive(state_mutex);
            printf("no timer %d\n", idx);
            return;
        }
        for (int i = idx; i < timer_count - 1; i++) {
            timers[i] = timers[i + 1];
        }
        timer_count--;
        timers_save();
        xSemaphoreGive(state_mutex);
        printf("timer %d cancelled\n", idx);
        return;
    }
    /* Otherwise: timer <duration> [label] */
    char durtok[32];
    int di = 0;
    while (*args && !isspace((unsigned char)*args) && di < (int)sizeof(durtok) - 1) {
        durtok[di++] = *args++;
    }
    durtok[di] = '\0';
    uint32_t secs = 0;
    if (di == 0 || !parse_duration(durtok, &secs)) {
        printf("usage: timer <Nm|Nh|Ns> [label]  (e.g. timer 10m pasta, timer 1h30m)\n"
               "       timer list | timer cancel N\n");
        return;
    }
    while (*args && isspace((unsigned char)*args)) {
        args++;
    }
    char label[32];
    snprintf(label, sizeof(label), "%s", args);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (timer_count >= MAX_TIMERS) {
        xSemaphoreGive(state_mutex);
        printf("timer list full (%d max)\n", MAX_TIMERS);
        return;
    }
    timers[timer_count].duration_s = secs;
    timers[timer_count].target_us = esp_timer_get_time() + (uint64_t)secs * 1000000ULL;
    timers[timer_count].active = true;
    snprintf(timers[timer_count].label, sizeof(timers[timer_count].label), "%s", label);
    int idx = timer_count;
    timer_count++;
    timers_save();
    xSemaphoreGive(state_mutex);
    printf("timer %d set: %s%s%s\n", idx, durtok, label[0] ? " - " : "", label);
}

static void handle_line(char *line)
{
    /* Strip trailing newline / carriage return. */
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[--len] = '\0';
    }
    /* Skip leading whitespace. */
    while (*line && isspace((unsigned char)*line)) {
        line++;
    }
    if (*line == '\0') {
        return;
    }

    if (strcmp(line, "time") == 0) {
        cmd_time();
    } else if (strncmp(line, "settime", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        cmd_settime(line[7] ? line + 8 : "");
    } else if (strncmp(line, "alarm add", 9) == 0 && (line[9] == ' ' || line[9] == '\0')) {
        cmd_alarm_add(line[9] ? line + 10 : "");
    } else if (strcmp(line, "alarm list") == 0) {
        cmd_alarm_list();
    } else if (strncmp(line, "alarm del", 9) == 0 && (line[9] == ' ' || line[9] == '\0')) {
        cmd_alarm_del(line[9] ? line + 10 : "");
    } else if (strcmp(line, "stop") == 0) {
        cmd_stop();
    } else if (strncmp(line, "snooze", 6) == 0 && (line[6] == ' ' || line[6] == '\0')) {
        cmd_snooze(line[6] ? line + 7 : "");
    } else if (strncmp(line, "timer", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        cmd_timer(line[5] ? line + 6 : "");
    } else if (strncmp(line, "wifi", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        cmd_wifi(line[4] ? line + 5 : "");
    } else if (strcmp(line, "help") == 0) {
        cmd_help();
    } else {
        printf("unknown command (try: help)\n");
    }
}

static void console_task(void *arg)
{
    (void)arg;
    char line[128];
    size_t pos = 0;
    print_prompt();
    fflush(stdout);
    for (;;) {
        int c = getchar();
        if (c == EOF) {
            break;
        }
        if (c == '\r' || c == '\n') {
            putchar('\n');
            line[pos] = '\0';
            pos = 0;
            handle_line(line);
            print_prompt();
        } else if (c == 0x7f || c == '\b') {
            if (pos > 0) {
                pos--;
                fputs("\b \b", stdout);
            }
        } else if (pos < sizeof(line) - 1 && isprint((unsigned char)c)) {
            line[pos++] = (char)c;
            putchar(c);
        }
        fflush(stdout);
    }
    /* stdin closed; park the task. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* --------------------------------- main --------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "alarm_clock starting");

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* ESP-IDF's default VFS binding for the USB-Serial-JTAG console is
     * output-only: stdin reads return EOF forever. Install the
     * interrupt-driven driver with an RX buffer and rebind VFS so that
     * typed input actually arrives. */
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t jtag_config = {
            .tx_buffer_size = 256,
            .rx_buffer_size = 256,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&jtag_config));
    }
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    usb_serial_jtag_vfs_use_driver();
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_LOGI(TAG, "USB-Serial-JTAG console input enabled");
#endif

    /* Pacific time with DST rules; used when converting NTP (UTC) to RTC. */
    setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1);
    tzset();

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(nvs_open("alarms", NVS_READWRITE, &nvs_h));
    alarms_load();
    ESP_LOGI(TAG, "restored %d alarm(s) from NVS", alarm_count);

    state_mutex = xSemaphoreCreateMutex();
    configASSERT(state_mutex);

    ESP_ERROR_CHECK(i2c_init());
    timers_load();
    ESP_LOGI(TAG, "restored %d timer(s) from NVS", timer_count);
    ESP_ERROR_CHECK(amp_enable());
    ESP_ERROR_CHECK(es8311_init());
    ESP_ERROR_CHECK(i2s_tx_init());

    rtc_time_t t;
    if (rtc_read(&t) == ESP_OK) {
        if (rtc_valid(&t)) {
            ESP_LOGI(TAG, "RTC time: %04d-%02d-%02d %02d:%02d:%02d",
                     t.year, t.month, t.day, t.hour, t.min, t.sec);
        } else {
            printf("RTC NOT SET - use: settime YYYY-MM-DD HH:MM:SS\n");
        }
    } else {
        printf("RTC read failed at boot\n");
    }

    xTaskCreate(alarm_task, "alarm_task", 4096, NULL, 5, NULL);
    xTaskCreate(console_task, "console", 4096, NULL, 4, NULL);
    {
        /* Start WiFi only if credentials were saved before. */
        char ssid[33], pass[65];
        if (wifi_creds_load(ssid, sizeof(ssid), pass, sizeof(pass))) {
            xTaskCreate(wifi_task, "wifi", 4096, NULL, 3, &wifi_task_handle);
        } else {
            ESP_LOGI(TAG, "no wifi credentials; use: wifi set <ssid> <password>");
        }
    }

    printf("\nTalking alarm clock ready. Type 'help' for commands.\n");
    fflush(stdout);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
