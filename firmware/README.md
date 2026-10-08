# firmware

ESP-IDF v6.0.1 application goes here.

Planned contents:

- Custom Muse Gadget SDK board profile for the Waveshare ESP32-S3 AI
  Smart Speaker (Path A): ES8311 output, ES7210 microphones, buttons,
  WS2812B RGB ring, PCF85063 RTC, TCA9555 amp enable (EXIO8 over I2C).
- Bring-up test apps (I2C scan, mic capture, playback).
- Alarm logic, voice control, Muse responses, RGB feedback.

Toolchain: ESP-IDF v6.0.1 exactly. Run the setup script in `scripts/`
on the Mac first.
