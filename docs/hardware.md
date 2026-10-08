# Verified hardware notes — Waveshare ESP32-S3 AI Smart Speaker

Verified 2026-10-05/06 from two independent community ESPHome builds,
cross-checked against Waveshare schematic v1.1, plus three independent
working firmware projects.

## Bus map

- Shared I2S bus: MCLK=12, BCLK=13, LRCK=14, DIN=15 (mic), DOUT=16 (speaker)
- I2C: SDA=11, SCL=10, confirmed working at 100 kHz
- ES8311 (speaker DAC) @ 0x18
- ES7210 (mic ADC) @ 0x40
- PCF85063 (RTC) @ 0x51
- TCA9555 I/O expander @ 0x20
- 7x WS2812B ring on GPIO38 (RMT; RGB order — verify on board)
- GPIO33-37 unusable (octal PSRAM)

## Speaker amp enable — read this before debugging silence

The NS4150B Class-D speaker amp is enabled via **EXIO8 on the TCA9555
expander @ 0x20, driven over I2C**. It is NOT a direct GPIO. The board
profile must toggle it over I2C or there will be no sound even with the
ES8311 codec configured correctly.

## Audio path (Path A board profile)

- The mic path MUST go through the ES7210 ADC. A plain I2S mic routed
  through the ES8311 alone streams data but yields no usable speech
  recognition.
- ES8311 (DAC) and ES7210 (ADC) share one I2S bus; only one side can
  drive the clocks. Configure the mic bus as I2S **master** (captures
  continuously for wake word) and the speaker bus as **slave** on the
  same pins.
- Pin the mic frame to **16-bit**. A 32-bit frame against the 16-bit
  DAC comes out as noise.
- ES7210 is a 4-channel ADC with 2 physical mics (CH1/CH2). CH3 is a
  hardware playback-reference loopback usable for AEC in the
  voice-control phase.
- ESP32-S3 has no Bluetooth Classic, so no A2DP audio streaming.

## Speakers

MECCANIXITY 2-pack 8Ω 2W 40mm speakers. The 1.25mm plugs fit the board's
GH1.25 2-pin speaker header (H3) directly — no adapter needed.
