#!/usr/bin/env python3
# Thrust stand -- host: swap the Pico between stand firmware and ESC passthrough
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

"""Swap the board between the stand firmware and the ESC passthrough sketch.

Reading or changing an ESC setting means running `vendor/BlHeli-Passthrough/`
instead of `firmware/`, then putting the stand firmware back. That is two
flashes per setting change, which is enough friction to discourage checking a
setting at all. This makes each direction one command.

    python tools/flash_board.py esc      # then: python tools/am32.py read
    python tools/flash_board.py stand    # then: python measure.py --check

It exists because `arduino-cli upload` cannot finish on a host with no
auto-mounter. It resets the board into its bootloader correctly and then fails
with "No drive to deploy", because the RPI-RP2 volume is never mounted and
`udisksctl` has no polkit agent over ssh. So the reset and the load are done
separately here: a 1200 baud open-and-close puts the RP2040 in BOOTSEL, and
`picotool load` writes the image over USB without mounting anything.

**The wait for re-enumeration is the point, not padding.** `picotool` returns as
soon as it has rebooted the board, well before the USB CDC device is back. A
command that opens the port immediately after gets "No such file or directory",
which reads like a dead board rather than an impatient script.
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

try:
    import serial
except ImportError:
    sys.exit("flash_board.py needs pyserial: pip install pyserial")

ARDUINO_CLI = "/usr/bin/arduino-cli"  # /usr/bin is not always on PATH here
PICOTOOL = "picotool"
FQBN = "rp2040:rp2040:rpipico"
DEFAULT_PORT = "/dev/ttyACM0"

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCHES = {
    "stand": os.path.join(ROOT, "firmware"),
    "esc": os.path.join(ROOT, "vendor", "BlHeli-Passthrough", "rp2040"),
}


def run(cmd, **kw):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, **kw)
    except FileNotFoundError:
        sys.exit("%s is not on PATH.\n"
                 "  arduino-cli: invoked by absolute path here, because /usr/bin\n"
                 "  is not always on PATH in this environment.\n"
                 "  picotool: install it, or flash by copying the .uf2 to a\n"
                 "  mounted RPI-RP2 volume by hand." % cmd[0])


def compile_sketch(sketch, build_dir):
    print("compiling %s" % os.path.relpath(sketch, ROOT))
    result = run([ARDUINO_CLI, "compile", "--fqbn", FQBN,
                  "--output-dir", build_dir, sketch])
    if result.returncode != 0:
        sys.exit("compile failed:\n" + result.stdout + result.stderr)
    for name in os.listdir(build_dir):
        if name.endswith(".uf2"):
            return os.path.join(build_dir, name)
    sys.exit("compile produced no .uf2 in %s" % build_dir)


# Raspberry Pi vendor id, and the product ids the RP2040 and RP2350 present
# while in their USB mass-storage bootloader.
RP_VENDOR_ID = "2e8a"
RP_BOOT_PRODUCT_IDS = {"0003", "000f"}
USB_DEVICES = "/sys/bus/usb/devices"


def in_bootloader():
    """True when an RP2040/RP2350 is enumerated in its bootloader.

    Deliberately not `picotool info`: some picotool builds abort with a
    shared_ptr assertion when asked about an RP2040, which would read as "no
    board" rather than "wrong tool for the question". Reading the USB ids has
    no such failure mode and needs nothing installed.
    """
    try:
        names = os.listdir(USB_DEVICES)
    except OSError:
        return False
    for name in names:
        try:
            with open(os.path.join(USB_DEVICES, name, "idVendor")) as fh:
                if fh.read().strip() != RP_VENDOR_ID:
                    continue
            with open(os.path.join(USB_DEVICES, name, "idProduct")) as fh:
                if fh.read().strip() in RP_BOOT_PRODUCT_IDS:
                    return True
        except OSError:
            continue
    return False


def enter_bootloader(port, timeout=15.0):
    if in_bootloader():
        print("board is already in the bootloader")
        return
    if not os.path.exists(port):
        sys.exit("no %s and no board in the bootloader; is it plugged in?" % port)
    print("resetting %s into the bootloader" % port)
    try:
        # The arduino-pico core watches for a 1200 baud open, same as arduino-cli
        # does. The board disappears mid-close, so the exception is expected.
        serial.Serial(port, 1200).close()
    except serial.SerialException:
        pass
    deadline = time.time() + timeout
    while time.time() < deadline:
        if in_bootloader():
            return
        time.sleep(0.3)
    sys.exit("board did not enter the bootloader within %.0fs" % timeout)


def wait_for_port(port, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(port):
            time.sleep(2.0)  # enumerated is not the same as ready to open
            return True
        time.sleep(0.3)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("target", choices=sorted(SKETCHES))
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("--build-dir", default=None,
                    help="where to put the compiled image "
                         "(default: a directory under the system temp dir)")
    args = ap.parse_args()

    for tool in (ARDUINO_CLI, PICOTOOL):
        run([tool, "version"])  # only to fail early if the binary is missing

    sketch = SKETCHES[args.target]
    build_dir = args.build_dir or os.path.join(
        tempfile.gettempdir(), "thrust_stand_build", args.target)
    os.makedirs(build_dir, exist_ok=True)

    image = compile_sketch(sketch, build_dir)
    enter_bootloader(args.port)

    print("loading %s" % os.path.basename(image))
    result = run([PICOTOOL, "load", "-x", image])
    if result.returncode != 0:
        sys.exit("picotool load failed:\n" + result.stdout + result.stderr)

    if not wait_for_port(args.port):
        sys.exit("%s did not come back; the image may still be fine, but check "
                 "before assuming a failure" % args.port)
    print("%s is running the %s image on %s"
          % (os.path.basename(image), args.target, args.port))
    return 0


if __name__ == "__main__":
    sys.exit(main())
