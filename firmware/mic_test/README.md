# mic_test

Minimal microphone bring-up firmware for the Waveshare ESP32-S3-AUDIO-Board
(ESP32-S3R8, 16 MB flash). It initializes the ES7210 mic ADC and captures
stereo audio (left = MIC1, right = MIC2) over I2S RX, printing an ASCII VU
meter so you can clap or talk near the board and watch the bars move.

The ES7210 register sequence mirrors `es7210_config_codec()` in Espressif's
official `espressif/es7210` component driver (Apache-2.0, from esp-bsp):
16 kHz / 16-bit / I2S slave, MCLK = 16 kHz x 256 = 4.096 MHz on GPIO12,
30 dB mic gain, 2.87 V mic bias. The ESP32 is the I2S master (it drives
BCLK/LRCK/MCLK); the ES7210 is the slave.

## Build and flash (on the Mac)

In every new terminal, export the toolchain first:

```sh
. $HOME/esp/esp-idf/export.sh
```

Then:

```sh
cd firmware/mic_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

(If the port name changed, check `ls /dev/cu.usbmodem*` first.)

## What success looks like

After flashing you should see one meter line every ~200 ms:

```
I (xxx) mic_test: MIC TEST RUNNING - clap or talk near the board
I (xxx) mic_test: [###-------------------------------------]  -23.4 dB (L= 2187 R= 2011)
I (xxx) mic_test: [##################################------]   -3.1 dB (L=22910 R=22480)  <-- LOUD!
```

Clap once near the board: the bar should jump and a line tagged `LOUD!`
should appear. In a quiet room the bar sits near the left at around
-40 to -60 dB. If both L and R stay at 0 while you clap, the mic path is
not capturing.

Exit the monitor with `Ctrl-]`.

No Wi-Fi, no Muse SDK, no secrets in this project.
