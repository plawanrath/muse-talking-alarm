# audio_test

Minimal speaker bring-up firmware for the Waveshare ESP32-S3-AUDIO-Board
(ESP32-S3R8, 16 MB flash). It enables the speaker amplifier, initializes the
ES8311 DAC, and plays an audible test pattern through the built-in speaker
(factory-wired to the SPK header, no hardware changes needed).

Test pattern: three 440 Hz beeps (250 ms each, 150 ms gaps), then one
880 Hz beep (400 ms), at modest volume (ES8311 volume 50/100, ~21% sine
amplitude). The monitor prints `TEST DONE` when the pattern finishes.

The ES8311 register sequence mirrors Espressif's `espressif/es8311`
component driver (Apache-2.0, from esp-bsp) for 16 kHz / 16-bit / I2S slave
with MCLK = 16 kHz x 256 = 4.096 MHz, which is the same configuration the
vendor demo (`ESP32-S3-AUDIO-Board-Demo`, Arduino `LVGL_Arduino` example)
uses on this board. The amplifier enable matches the vendor demo exactly:
TCA9555 EXIO8 driven high.

## Build and flash (on the Mac)

In every new terminal, export the toolchain first:

```sh
. $HOME/esp/esp-idf/export.sh
```

Then:

```sh
cd firmware/audio_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

(If the port name changed, check `ls /dev/cu.usbmodem*` first.)

## What success looks like

```
I (xxx) audio_test: I2C ready: SDA=GPIO11 SCL=GPIO10 @ 100000 Hz
I (xxx) audio_test: amplifier enabled (TCA9555 EXIO8 = high)
I (xxx) audio_test: ES8311 initialized: 16-bit, 16 kHz, slave, MCLK 4.096 MHz, volume 50
I (xxx) audio_test: I2S TX ready: 16 kHz 16-bit stereo, MCLK=GPIO12 BCLK=GPIO13 LRCK=GPIO14 DOUT=GPIO16
I (xxx) audio_test: playing test pattern: 3x 440 Hz beeps, then 1x 880 Hz beep
I (xxx) audio_test: TEST DONE - idling
```

You should hear: beep beep beep, BEEP (higher). Exit the monitor with `Ctrl-]`.

No Wi-Fi, no Muse SDK, no secrets in this project.
