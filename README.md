# Talking Alarm Clock / Voice Assistant Speaker

Custom firmware for the Waveshare ESP32-S3 AI Smart Speaker board
(ESP32-S3, ES8311 speaker codec, ES7210 dual-mic ADC, PCF85063 RTC,
7x WS2812B LED ring, TCA9555 I/O expander), built as a Muse Gadget SDK
board profile (Path A).

## Layout

- `firmware/` — ESP-IDF v6.0.1 application: custom Muse Gadget SDK board
  profile, bring-up tests, alarm logic, voice pipeline.
- `scripts/` — host-side helper scripts (setup, flashing helpers).
- `docs/` — hardware notes and the bring-up checklist.

## Tomorrow's bring-up plan (Oct 7)

1. Create the GitHub repo yourself and push this project to it.
2. Run the macOS ESP-IDF v6.0.1 setup script (20-40 min, safe to re-run):
   copy it from your assistant's files into `scripts/` first.
3. Plug in the board; confirm USB enumeration and boot logs on serial.
4. I2C scan at 100 kHz — expect ES8311 @ 0x18, ES7210 @ 0x40,
   PCF85063 @ 0x51, TCA9555 @ 0x20.
5. Start the board-profile skeleton: buttons, WS2812B ring on GPIO38,
   RTC reads. No audio yet.
6. When the speakers arrive: enable the speaker amp via EXIO8 on the
   TCA9555 expander @ 0x20 (I2C — NOT a GPIO), then verify playback,
   mic capture via ES7210, and the CH3 AEC loopback.

## Secrets

Wi-Fi credentials, tokens, and API keys go in local files only
(`secrets.h`, `.env`) — these are gitignored and must never be committed.

## License

Copyright 2026 Plawan Kumar Rath.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

### License headers

New source files should carry this header (adapted to the file's
comment syntax):

    Copyright 2026 Plawan Kumar Rath
    SPDX-License-Identifier: Apache-2.0
