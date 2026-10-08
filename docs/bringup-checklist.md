# Bring-up checklist — Oct 7

Work from a git clone of this repo on the Mac. Commit as you go.

## No speakers needed

- [ ] Run the macOS ESP-IDF v6.0.1 setup script from `scripts/`
      (20-40 min, safe to re-run).
- [ ] Plug in the board; confirm USB enumeration and boot logs.
- [ ] I2C scan at 100 kHz: expect 0x18 (ES8311), 0x40 (ES7210),
      0x51 (PCF85063), 0x20 (TCA9555).
- [ ] Board-profile skeleton: buttons, WS2812B ring (GPIO38), RTC reads.
- [ ] Mic capture test through the ES7210 (no speakers required).

## After the speakers arrive

- [ ] Connect speakers to the GH1.25 header (H3). Plugs fit directly.
- [ ] Enable the amp: drive EXIO8 on TCA9555 @ 0x20 over I2C.
- [ ] Playback test through ES8311 (speaker bus = I2S slave, 16-bit).
- [ ] Full duplex test; verify CH3 AEC loopback reference.

## Commit hygiene

- Commit working states with short messages.
- Never commit `secrets.h`, `.env`, or anything in `.gitignore`.
- New source files get the Apache 2.0 header (see README).
