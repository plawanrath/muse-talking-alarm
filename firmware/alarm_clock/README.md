# alarm_clock — talking alarm clock (phase 1)

RTC-based daily alarms plus countdown timers for the Waveshare
ESP32-S3-AUDIO-Board. Alarms ring with a gentle escalating chime through
the onboard speaker; everything is driven from a serial command line and
persisted in NVS.

Hardware init (I2C, TCA9555 amp enable, ES8311, I2S TX 16 kHz / 16-bit)
is reused verbatim from the proven `firmware/audio_test` project.
The PCF85063 RTC is polled once per second; its hardware alarm pin is
not used.

## Build and flash (on Plawan's Mac)

```
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

Exit the monitor with `Ctrl-]`.

## Commands (prompt: `alarm> `)

```
time                        show RTC time
settime YYYY-MM-DD HH:MM:SS set the RTC (year 2026-2099)
alarm add HH:MM [label]     add a daily alarm (e.g. alarm add 07:30 Wake up)
alarm list                  list alarms with indices
alarm del N                 delete alarm N
timer <Nm|Nh|Ns> [label]    countdown timer (e.g. timer 10m pasta, timer 1h30m)
timer list                  list timers with remaining time
timer cancel N              cancel timer N
stop                        silence a ringing alarm/timer
snooze [minutes]            silence and re-ring (default 9 min)
help                        this list
```

Alarms are daily (HH:MM, 24 h). Timers count down from now via
`esp_timer` and need no RTC. Both ring with the same escalating chime
(soft at first, louder over ~90 s) and both honor `stop` / `snooze`.
Alarms and timers persist across reboots in NVS; timers whose target
already passed while powered off are dropped. A ringing alarm
auto-stops after 5 minutes.

## Quick start

1. Flash and open the monitor. If you see
   `RTC NOT SET - use: settime YYYY-MM-DD HH:MM:SS`, set the clock:
   ```
   alarm> settime 2026-10-07 21:35:00
   ```
2. Add an alarm one minute in the future:
   ```
   alarm> alarm add 21:36 test
   ```
3. Wait for `ALARM RINGING: test`, then silence it:
   ```
   alarm> stop
   ```
4. Try a timer:
   ```
   alarm> timer 1m tea
   alarm> timer list
   ```
   After 60 s it rings; `stop` silences it.
5. Try snooze: while it rings, `alarm> snooze 2` re-rings in 2 minutes.
