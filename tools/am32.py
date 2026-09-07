#!/usr/bin/env python3
# Thrust stand -- ESC: read, edit and flash an AM32 ESC over 4-Way
# Copyright (C) 2026 Julian Jakobs
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details. You should have received a copy of the GNU General Public
# License along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Talk to an AM32 ESC through the vendored 4-Way passthrough sketch.

The point of this tool is that `esc_firmware` in `stand.json` is a *declared*
value. Nothing in bidirectional DShot carries a firmware version, so a stale
declaration is recorded confidently and looks right forever. On an ARM ESC the
settings page can be read back instead, which is the only way to check it.

Flash `vendor/BlHeli-Passthrough/rp2040/` to the board first, power the ESC,
then run this. Flash `firmware/` back afterwards. The passthrough is a separate
sketch on purpose: integrating it would hand over core 1's PIO pin, block core 0
and share the host's serial line.

    python tools/am32.py read                     # decode the settings page
    python tools/am32.py set variable_pwm=0 motor_kv=40
    python tools/am32.py dump -o shipped.bin      # back up the application
    python tools/am32.py flash AM32_x_2.21.hex    # write and verify it back

**Back the application up with `dump` before you `flash`.** The bootloader is
never erased, so a failed write is recoverable, but the image a vendor shipped
is not downloadable if it predates the public releases. This one did.

Two traps, both learned the hard way:

**The settings page is not self-describing and the layout has versions.** Byte 1
is the layout version and this tool decodes version 4, from AM32 2.21's
`Inc/eeprom.h`. Read `--raw` and check against the firmware you actually
flashed rather than trusting the field names below.

**Check every field after a firmware update.** AM32 2.21 migrates an older page
in `loadEEpromSettings` and that migration is incomplete: it clears the bytes
where layout 1 kept its 12-byte firmware name string, but not the last two, so
two ASCII characters survive into what layout 4 reads as `input_type` and
`auto_advance`. An ESC can come up with a feature enabled by a leftover letter.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("am32.py needs pyserial: pip install pyserial")

DEFAULT_PORT = "/dev/ttyACM0"

# 4-Way framing. Host frames open with cmd_Local_Escape, replies with
# cmd_Remote_Escape; see vendor/BlHeli-Passthrough/rp2040/4Way.h.
LOCAL, REMOTE = 0x2F, 0x2E
CMD_RESET, CMD_INIT_FLASH = 0x35, 0x37
CMD_PAGE_ERASE, CMD_READ, CMD_WRITE = 0x39, 0x3A, 0x3B
ACK_OK = 0x00

# The bootloader occupies pages 0-3 and is never touched. The settings page is
# the last one and is not erased by a firmware flash, which is why an update
# migrates it in place rather than starting clean.
FLASH_BASE = 0x08000000
APP_START, APP_END, PAGE_BYTES = 0x1000, 0x7C00, 0x400
EEPROM_ADDR, EEPROM_LEN = 0x7C00, 192
EEPROM_PAGE = EEPROM_ADDR // PAGE_BYTES

# EEPROM layout version 4, from AM32 v2.21 Inc/eeprom.h. Bytes 14-16 are
# reserved, 48-191 are the tune and CAN blocks and are unused on non-CAN
# targets.
FIELDS = {
    1: "eeprom_version", 2: "bootloader_version",
    3: "version_major", 4: "version_minor",
    5: "max_ramp", 6: "minimum_duty_cycle", 7: "disable_stick_calibration",
    8: "absolute_voltage_cutoff", 9: "current_P", 10: "current_I",
    11: "current_D", 12: "active_brake_power", 13: "brake_on_zero_throttle",
    17: "dir_reversed", 18: "bi_direction", 19: "use_sine_start",
    20: "comp_pwm", 21: "variable_pwm", 22: "stuck_rotor_protection",
    23: "advance_level", 24: "pwm_frequency", 25: "startup_power",
    26: "motor_kv", 27: "motor_poles", 28: "brake_on_stop",
    29: "stall_protection", 30: "beep_volume", 31: "telemetry_on_interval",
    32: "servo_low", 33: "servo_high", 34: "servo_neutral",
    35: "servo_dead_band", 36: "low_voltage_cut_off", 37: "low_cell_volt_cutoff",
    38: "rc_car_reverse", 39: "use_hall_sensors", 40: "sine_mode_changeover",
    41: "drag_brake_strength", 42: "driving_brake_strength",
    43: "limit_temperature", 44: "limit_current", 45: "sine_mode_power",
    46: "input_type", 47: "auto_advance",
}
BY_NAME = {v: k for k, v in FIELDS.items()}

# Notes printed beside the value, for the fields where the number alone misleads.
NOTES = {
    24: "kHz. Choose by measurement across the WHOLE throttle range",
    26: "x40+20 KV. NOT a motor property: it scales the low-RPM duty ceiling",
    28: "must be 0, or COASTDOWN measures the brake",
    13: "must be 0, or COASTDOWN measures the brake",
    21: "1 or 2 move PWM frequency with SPEED, not throttle. 2 pins the "
        "PWM-to-commutation ratio near 9.2 and is AM32's fix for the "
        "entrainment locks; it also confounds a sweep",
    22: "on, this can lock out during the sweep's 1-15% idle hunt",
    23: "x0.9375 degrees, less 10 in the post-1.90 format",
}


def crc16(data):
    """XMODEM CRC, matching _crc_xmodem_update in the passthrough sketch."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_hex(path):
    """Intel HEX to {absolute address: byte}."""
    mem, base = {}, 0
    for line in open(path):
        line = line.strip()
        if not line.startswith(":"):
            continue
        rec = bytes.fromhex(line[1:])
        count, addr_hi, addr_lo, rec_type = rec[0], rec[1], rec[2], rec[3]
        if rec_type == 4:
            base = int.from_bytes(rec[4:6], "big") << 16
        elif rec_type == 0:
            addr = base + (addr_hi << 8) + addr_lo
            for i, value in enumerate(rec[4:4 + count]):
                mem[addr + i] = value
    return mem


class FourWay:
    """One 4-Way session. Entering it costs the MSP link, so this owns the port.

    cmd_InterfaceExit is never sent. It ends the sketch's passthrough loop, and
    the Pico then needs a reboot before anything can reach the ESC again, which
    turns a second command into a confusing timeout.
    """

    def __init__(self, port, timeout=5.0):
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.ser.write(b"$M<" + bytes([0, 245, 245]))  # MSP_SET_4WAY_IF
        self.ser.flush()
        time.sleep(0.3)
        self.ser.read(64)

    def _read_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                raise IOError("timed out after %d of %d bytes; is the "
                              "passthrough sketch flashed and the ESC powered?"
                              % (len(buf), n))
            buf += chunk
        return buf

    def cmd(self, cmd, addr=0, params=b"\x00"):
        body = bytes([LOCAL, cmd, (addr >> 8) & 0xFF, addr & 0xFF,
                      len(params) & 0xFF]) + params
        crc = crc16(body)
        self.ser.write(body + bytes([crc >> 8, crc & 0xFF]))
        self.ser.flush()
        head = self._read_exact(5)
        if head[0] != REMOTE:
            raise IOError("bad frame start 0x%02X" % head[0])
        length = head[4] or 256  # a length byte of 0 means 256
        rest = self._read_exact(length + 3)
        return rest[:length], rest[length]

    def init_flash(self):
        params, ack = self.cmd(CMD_INIT_FLASH, 0, b"\x00")
        if ack != ACK_OK:
            sys.exit("the ESC did not answer its bootloader (ack=0x%02X). "
                     "Check the rail is on and the signal wire is on the "
                     "passthrough pin." % ack)
        return params

    def read_block(self, addr, length, chunk=64):
        out = bytearray()
        while len(out) < length:
            n = min(chunk, length - len(out))
            data, ack = self.cmd(CMD_READ, addr + len(out), bytes([n]))
            if ack != ACK_OK:
                sys.exit("read failed at 0x%04X (ack=0x%02X)" % (addr + len(out), ack))
            out += data
        return bytes(out)


def show(buf, raw=False):
    print("eeprom layout v%d, firmware %d.%02d, bootloader %d"
          % (buf[1], buf[3], buf[4], buf[2]))
    if buf[1] != 4:
        print("  WARNING: this tool decodes layout 4; the names below are wrong")
    for index in sorted(FIELDS):
        if index in (1, 2, 3, 4):
            continue
        note = NOTES.get(index, "")
        print("  %-26s [%3d] = %3d %s" % (FIELDS[index], index, buf[index],
                                          ("  <- " + note) if note else ""))
    if raw:
        print("\nraw:")
        for off in range(0, len(buf), 16):
            print("  %3d: %s" % (off, buf[off:off + 16].hex(" ")))


def cmd_read(args):
    fw = FourWay(args.port)
    fw.init_flash()
    show(fw.read_block(EEPROM_ADDR, EEPROM_LEN), args.raw)
    return 0


def cmd_set(args):
    fw = FourWay(args.port)
    fw.init_flash()
    current = fw.read_block(EEPROM_ADDR, EEPROM_LEN)
    new = bytearray(current)
    for item in args.assignments:
        name, sep, value = item.partition("=")
        if not sep or name not in BY_NAME:
            sys.exit("expected NAME=VALUE with NAME one of: %s"
                     % ", ".join(sorted(BY_NAME)))
        index = BY_NAME[name]
        new[index] = int(value) & 0xFF
        print("  %-26s [%3d] %3d -> %3d" % (name, index, current[index], new[index]))
    if new == current:
        print("nothing to do")
        return 0

    _, ack = fw.cmd(CMD_PAGE_ERASE, 0, bytes([EEPROM_PAGE]))
    if ack != ACK_OK:
        sys.exit("erase of the settings page failed (ack=0x%02X)" % ack)
    _, ack = fw.cmd(CMD_WRITE, EEPROM_ADDR, bytes(new))
    if ack != ACK_OK:
        sys.exit("write failed (ack=0x%02X); the settings page is now erased, "
                 "so re-run this before power cycling" % ack)
    if fw.read_block(EEPROM_ADDR, EEPROM_LEN) != bytes(new):
        sys.exit("verify FAILED; the settings page does not match what was sent")
    print("verify: OK")
    fw.cmd(CMD_RESET, 0, b"\x00")
    return 0


def cmd_dump(args):
    fw = FourWay(args.port)
    print("bootloader signature: %s" % fw.init_flash().hex(" "), file=sys.stderr)
    image = fw.read_block(APP_START, APP_END - APP_START, chunk=128)
    open(args.output, "wb").write(image)
    print("wrote %d bytes of 0x%04X-0x%04X to %s"
          % (len(image), APP_START, APP_END - 1, args.output))
    return 0


def cmd_flash(args):
    mem = parse_hex(args.hexfile)
    low, high = min(mem), max(mem)
    if low < FLASH_BASE + APP_START or high >= FLASH_BASE + APP_END:
        sys.exit("image spans 0x%X-0x%X, outside the application region; this "
                 "tool refuses to touch the bootloader or the settings page"
                 % (low, high))
    image = bytes(mem.get(FLASH_BASE + a, 0xFF) for a in range(APP_START, APP_END))
    print("image: 0x%04X-0x%04X, %d bytes" % (APP_START, APP_END - 1, len(image)))

    fw = FourWay(args.port)
    print("bootloader signature: %s" % fw.init_flash().hex(" "))

    if not args.verify_only:
        for page in range(APP_START // PAGE_BYTES, APP_END // PAGE_BYTES):
            _, ack = fw.cmd(CMD_PAGE_ERASE, 0, bytes([page]))
            if ack != ACK_OK:
                sys.exit("erase of page %d failed (ack=0x%02X)" % (page, ack))
        print("erased pages %d-%d"
              % (APP_START // PAGE_BYTES, APP_END // PAGE_BYTES - 1))
        for off in range(0, len(image), 128):
            _, ack = fw.cmd(CMD_WRITE, APP_START + off, image[off:off + 128])
            if ack != ACK_OK:
                sys.exit("write at 0x%04X failed (ack=0x%02X)"
                         % (APP_START + off, ack))
        print("wrote %d bytes" % len(image))

    read_back = fw.read_block(APP_START, len(image), chunk=128)
    bad = [i for i in range(len(image)) if read_back[i] != image[i]]
    for i in bad[:8]:
        print("  mismatch 0x%04X: read 0x%02X want 0x%02X"
              % (APP_START + i, read_back[i], image[i]))
    print("verify: %s (%d mismatched bytes)"
          % ("OK" if not bad else "FAILED", len(bad)))
    if bad:
        return 1
    fw.cmd(CMD_RESET, 0, b"\x00")
    print("ESC reset. Flash firmware/ back to the board, then check the "
          "settings page: an update migrates it and the migration is not "
          "complete.")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Flash vendor/BlHeli-Passthrough/rp2040/ to the board first.")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("read", help="decode the settings page")
    p.add_argument("--raw", action="store_true", help="also print all 192 bytes")
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("set", help="change named settings fields")
    p.add_argument("assignments", nargs="+", metavar="NAME=VALUE")
    p.set_defaults(func=cmd_set)

    p = sub.add_parser("dump", help="back up the application image")
    p.add_argument("-o", "--output", default="am32_backup.bin")
    p.set_defaults(func=cmd_dump)

    p = sub.add_parser("flash", help="write an Intel HEX application image")
    p.add_argument("hexfile")
    p.add_argument("--verify-only", action="store_true",
                   help="compare without writing")
    p.set_defaults(func=cmd_flash)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
