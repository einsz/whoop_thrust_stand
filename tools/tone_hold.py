#!/usr/bin/env python3
# Thrust stand -- host: one continuous hold, recorded acoustically
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

"""One uninterrupted hold, logged per sample, for comparison against a recording.

`soak_test.py` answers "does the reported speed decay across a run" but its
holds are separated by re-tares, so the audio has gaps and the telemetry is one
number per hold. The decay is fully developed within about four seconds of a
cold start, which means a single continuous hold captures the whole thing --
thousands of telemetry rows and an unbroken tone over the same window.

That is what makes the comparison airtight. Two slopes measured over one
physical event need no clock alignment between the recorder and the stand: if
the rotor really slows, the blade tone falls with the reported speed; if only
the ESC's arithmetic drifts, the tone stays put while the telemetry walks down.

The firmware caps a hold's logged window at 10 s (`runHoldSequence`), so that is
the ceiling here too.

Usage:
    python tools/tone_hold.py -o data/tone-hold.csv        # start recording at the countdown
    python tools/blade_tone.py hold.wav --compare data/tone-hold.csv
"""

import argparse
import importlib.util
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "measure", os.path.join(_HERE, os.pardir, "measure.py"))
measure = importlib.util.module_from_spec(_spec)
sys.modules["measure"] = measure
_spec.loader.exec_module(measure)

FIRMWARE_LOG_MS_MAX = 10000


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dshot", type=int, default=2000,
                    help="Raw DShot setpoint (default 2000). Raw rather than "
                         "percent for the reason DHOLD exists: integer percent "
                         "reaches only 100 of 2000 steps")
    ap.add_argument("--log-ms", type=int, default=FIRMWARE_LOG_MS_MAX,
                    help="Logged window in ms (default and firmware maximum %d)"
                         % FIRMWARE_LOG_MS_MAX)
    ap.add_argument("--settle-ms", type=int, default=300,
                    help="Discarded spin-up before logging starts (default 300). "
                         "Kept short deliberately: the decay under test happens "
                         "in the first seconds, and settling longer throws it away")
    ap.add_argument("--countdown", type=float, default=8.0,
                    help="Seconds to wait before spinning, to start the recorder "
                         "(default 8)")
    ap.add_argument("--tare", action="store_true",
                    help="Re-zero the cell first. Off by default: the point of "
                         "this run is RPM, and taring costs a second of settling")
    ap.add_argument("-o", "--output", default=None, metavar="CSV")
    ap.add_argument("-n", "--notes", default=None)
    ap.add_argument("-p", "--port", default=measure.SERIAL_PORT)
    args = ap.parse_args()

    if args.log_ms > FIRMWARE_LOG_MS_MAX:
        # Silently clamping would produce a shorter run than asked for, and the
        # recording would be trimmed against a window that never existed.
        sys.exit("--log-ms %d exceeds the firmware cap of %d ms; the board would "
                 "clamp it and the CSV would not say so."
                 % (args.log_ms, FIRMWARE_LOG_MS_MAX))

    link = None
    rows = []
    try:
        print("Connecting to %s..." % args.port)
        link = measure.Link(args.port, measure.BAUD_RATE)
        link.handshake()
        cfg = measure.load_stand_config()
        link.scale = cfg.get("hx711_scale")
        link.start_keepalive()
        print("--> %s   scale %s"
              % (link.meta.get("fw", ["?"])[0],
                 "%.4f" % link.scale if link.scale else "none"))
        print("--> bus %.3f V" % measure.require_bus_power(link))

        if args.tare:
            link.write_line("TARE")
            time.sleep(1.0)

        print("\nOne hold: dshot %d, %.1f s logged after %d ms of settling."
              % (args.dshot, args.log_ms / 1000.0, args.settle_ms))
        print("The motor must be at ambient for this to mean anything -- check it")
        print("with the camera before you start the recorder.\n")
        for remaining in range(int(args.countdown), 0, -1):
            print("  starting in %d... " % remaining, end="", flush=True)
            time.sleep(1.0)
        print("\n")

        rows = measure.collect_samples(
            link, "DHOLD,%d,%d,%d" % (args.dshot, args.log_ms, args.settle_ms),
            "START_HOLD", "END_HOLD", "tonehold")

    except Exception as exc:
        print("\n[ERROR] %s" % exc, file=sys.stderr)
    finally:
        print("\nSafety: commanding motor stop.")
        if link is not None:
            try:
                link.write_line("0")
                link.write_line("STOP")
                time.sleep(0.2)
            finally:
                link.close()

    if not rows:
        return

    rpms = [r["rpm"] for r in rows if r.get("rpm")]
    if rpms:
        span = rows[-1]["time_s"] - rows[0]["time_s"]
        head = sum(rpms[:250]) / len(rpms[:250])
        tail = sum(rpms[-250:]) / len(rpms[-250:])
        print("\nReported RPM across the window: %.0f -> %.0f over %.2f s  (%+.2f%%)"
              % (head, tail, span, 100.0 * (tail / head - 1)))
        print("That is the number the recording has to either confirm or refute.")

    if args.output:
        measure.write_csv(args.output, rows, measure.TRANSIENT_FIELDS, link,
                          "TONEHOLD", args.notes)


if __name__ == "__main__":
    main()
