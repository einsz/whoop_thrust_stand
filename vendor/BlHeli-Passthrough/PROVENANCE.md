# Provenance

Upstream: https://github.com/BrushlessPower/BlHeli-Passthrough

Only the `rp2040/` subtree is vendored, plus the upstream `LICENSE` and
`README.md`. The ESP32 and 328P ports are not kept: they need external
libraries, and the board here is an RP2040.

Licence is GPL-3.0, which matches this project.

## Why it is here

The ESC's own settings and firmware version cannot be read over bidirectional
DShot. That protocol carries eRPM and, with EDT enabled, temperature, voltage,
current and stress. There is no version field. Reading one needs 4-way over
MSP, which is what this tree implements.

It works on ARM-bootloader ESCs (AM32, BLHeli_32) and not on BLHeli_S or
Bluejay, which run on SiLabs EFM8 with a different bootloader protocol. An
earlier copy of this tree was removed in `6f8c0e6` for exactly that reason,
when the fitted ESC ran Bluejay.

## How it is used

As a standalone sketch, not linked into `firmware/`. Flash it, talk to the ESC
with `tools/am32.py` or the AM32 configurator, then flash the stand firmware
back:

```bash
/usr/bin/arduino-cli compile --fqbn rp2040:rp2040:rpipico vendor/BlHeli-Passthrough/rp2040
/usr/bin/arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 vendor/BlHeli-Passthrough/rp2040
# ... talk to the ESC, then:
/usr/bin/arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 firmware
```

Keeping it standalone avoids every problem integration would raise. Core 1
owns the DShot pin through a PIO state machine and would have to be stopped
and restarted around the handover; `processPassthrough()` blocks until the
configurator disconnects, which core 0 may not do; and MSP wants the USB CDC
line that the host protocol already owns. A separate sketch shares none of
those.

`rp2040.ino` uses pin 15 as shipped, which is already `ESC_PIN` in
`firmware/config.h`. Check that before flashing if the wiring changes.

## Before you flash

4-way can write to the ESC, not only read it, and the ESC reboots when the
tool disconnects. Take the prop off first.
