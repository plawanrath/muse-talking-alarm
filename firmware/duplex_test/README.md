# duplex_test

Full-duplex bring-up firmware for the Waveshare ESP32-S3-AUDIO-Board
(ESP32-S3R8, 16 MB flash). The last hardware bring-up step before
application logic: it verifies the speaker and the microphones can stream
simultaneously on the shared I2S bus.

What it does:

1. Enables the speaker amplifier (TCA9555 EXIO8, active high).
2. Initializes the ES8311 DAC (16 kHz / 16-bit / I2S slave, MCLK 4.096 MHz,
   volume 50/100) — same sequence as `firmware/audio_test`, verified
   working on the board.
3. Initializes the ES7210 mic ADC (16 kHz / 16-bit / I2S slave, mic gain
   30 dB) — same sequence as `firmware/mic_test`, verified working on
   the board.
4. Creates ONE I2S channel pair with the ESP32 as I2S master (both codecs
   are slaves): TX feeds the speaker (DOUT=GPIO16), RX reads the mics
   (DIN=GPIO15). This matches the vendor demo's topology
   (`ESP32-S3-AUDIO-Board-Demo`, ESP-IDF `mp3_play_03`).
5. Plays a continuous quiet 660 Hz sine tone on the speaker (amplitude
   ~8% of full scale) while printing the mic VU meter every ~200 ms.

Test: with the tone playing, clap or talk near the board. The VU bars
should jump and a `LOUD!` flag should appear on sharp peaks — proving the
mic captures while the speaker plays.

## Build and flash (on the Mac)

In every new terminal, export the toolchain first:

```sh
. $HOME/esp/esp-idf/export.sh
```

Then:

```sh
cd firmware/duplex_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

(If the port name changed, check `ls /dev/cu.usbmodem*` first.)

## What success looks like

```
I (xxx) duplex_test: I2C ready: SDA=GPIO11 SCL=GPIO10 @ 100000 Hz
I (xxx) duplex_test: amplifier enabled (TCA9555 EXIO8 = high)
I (xxx) duplex_test: ES8311 initialized: 16-bit, 16 kHz, slave, MCLK 4.096 MHz, volume 50
I (xxx) duplex_test: ES7210 initialized: 16-bit, 16 kHz, slave, MCLK 4.096 MHz, mic gain 30 dB
I (xxx) duplex_test: I2S duplex ready: 16 kHz 16-bit stereo, MCLK=GPIO12 BCLK=GPIO13 LRCK=GPIO14 DIN=GPIO15 DOUT=GPIO16
I (xxx) duplex_test: DUPLEX TEST RUNNING - tone playing, clap over it
I (xxx) duplex_test: [###-------------------------------------] -27.4 dB (L= 1402 R= 1395)
```

You should hear a soft continuous tone from the speaker, and the VU meter
should react when you clap over it. Exit the monitor with `Ctrl-]`.

No Wi-Fi, no Muse SDK, no secrets in this project.
