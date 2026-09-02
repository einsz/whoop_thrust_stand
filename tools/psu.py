#!/usr/bin/env python3
# Thrust stand -- host: programmable bench supply over SCPI
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

"""Drive the bench supply from the host, so a Kv run stops depending on a knob.

The Kv protocol holds one throttle at several supply voltages and fits the
slope. Done by hand it needs an operator to dial each voltage and press Enter,
which makes settling whatever the operator's patience was and puts a
transcription step between the supply and the record.

Verified against a Kiprim DC310S (`KIPRIM,DC310S,22471006,FV:V4.0.0`), widely a
rebadged OWON. Three things about it drove the design here:

**Every write is verified by reading it back.** There is no usable error queue --
`SYST:ERR?` returns `0x0000` unconditionally, including immediately after a
command the supply ignored. A rejected `VOLT` leaves the previous setpoint in
place and says nothing. In a Kv run that is a step that silently did not happen,
which yields a plausible wrong slope rather than an error, so nothing here sets
a value without confirming it landed.

**`VOLT:LIM` enforces a 1.0 V margin above the setpoint, both ways.** With
`VOLT=4.0`, asking for a limit of 4.2, 4.5 or 5.0 all silently become 5.000; and
with `LIM=6.0`, `VOLT 5.100` is refused while 5.000 is accepted. So the supply's
own over-voltage protection **cannot** guard a 1S lithium cell: at a 4.0 V
setpoint the tightest limit available is 5.0 V, well past the 4.2 V ceiling.
That guard has to live here instead, which is what `max_volts` is.

**It answers only to CRLF.** A bare newline gets total silence, which presents
exactly like a dead device.

Usage:
    python tools/psu.py --probe
    python tools/psu.py --set-voltage 3.4
    python tools/psu.py --output on
"""

import argparse
import json
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("psu.py needs pyserial: pip install pyserial")

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200
TERMINATOR = "\r\n"

# A 1S lithium cell may be sitting in parallel with the supply as a regen sink,
# and the supply's own OVP cannot be set tight enough to protect it (see above).
# Exceeding this is a cell-damage event, not a bad measurement, so it is refused
# rather than warned about.
LI_ION_CEILING_V = 4.2

# The supply refuses a setpoint within 1.0 V of its over-voltage limit. Knowing
# the rule lets a rejected write be explained instead of merely reported.
VOLT_LIM_MARGIN_V = 1.0


class PSUError(RuntimeError):
    pass


class PSU:
    def __init__(self, port=DEFAULT_PORT, baud=BAUD, timeout=2.0,
                 max_volts=LI_ION_CEILING_V):
        self.max_volts = max_volts
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
        except serial.SerialException as exc:
            raise PSUError("cannot open %s: %s" % (port, exc))
        # The CH340 needs a moment after open before it will pass traffic.
        time.sleep(0.15)
        self.ser.reset_input_buffer()

    def ask(self, cmd):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + TERMINATOR).encode())
        reply = self.ser.readline().decode(errors="replace").strip()
        if not reply:
            raise PSUError("no reply to %r -- wrong baud, or the supply wants "
                           "CRLF and got something else" % cmd)
        return reply

    def send(self, cmd, settle=0.3):
        self.ser.write((cmd + TERMINATOR).encode())
        # The supply needs time to apply a setpoint before it will report it
        # back correctly; reading too early returns the old value and looks
        # exactly like a rejected write.
        time.sleep(settle)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # --- reads ---------------------------------------------------------

    def identify(self):
        return self.ask("*IDN?")

    def measure(self):
        """(volts, amps) actually at the supply's terminals, in one round trip.

        Not the same node as the stand's INA226: the harness measured 15-16
        mOhm, so at 3.6 A these differ by ~90 mV by construction. This is the
        second point for a source-versus-harness split, not a better version of
        the stand's own reading.
        """
        raw = self.ask("MEAS:ALL?")
        try:
            v, a = raw.split(",")
            return float(v), float(a)
        except ValueError:
            raise PSUError("could not parse MEAS:ALL? reply %r" % raw)

    def voltage_setpoint(self):
        return float(self.ask("VOLT?"))

    def current_limit(self):
        return float(self.ask("CURR?"))

    def ovp_limit(self):
        return float(self.ask("VOLT:LIM?"))

    def output_on(self):
        return self.ask("OUTP?").upper().startswith("ON")

    # --- writes, all verified ------------------------------------------

    def set_voltage(self, volts, tolerance=0.005, allow_above_ceiling=False):
        if volts > self.max_volts and not allow_above_ceiling:
            raise PSUError(
                "refusing %.3f V: above the %.2f V ceiling. A 1S lithium cell "
                "may be in parallel as a regen sink, and the supply's own OVP "
                "cannot be set tight enough to protect it. Pass "
                "allow_above_ceiling=True (or --max-volts) only with the cell "
                "disconnected." % (volts, self.max_volts))
        self.send("VOLT %.3f" % volts)
        got = self.voltage_setpoint()
        if abs(got - volts) > tolerance:
            hint = ""
            lim = self.ovp_limit()
            if volts > lim - VOLT_LIM_MARGIN_V:
                hint = ("; VOLT:LIM is %.3f V and the supply refuses a setpoint "
                        "within %.1f V of it, so the ceiling right now is "
                        "%.3f V" % (lim, VOLT_LIM_MARGIN_V,
                                    lim - VOLT_LIM_MARGIN_V))
            raise PSUError("asked for %.3f V, supply reports %.3f V%s"
                           % (volts, got, hint))
        return got

    def set_current_limit(self, amps, tolerance=0.01):
        self.send("CURR %.3f" % amps)
        got = self.current_limit()
        if abs(got - amps) > tolerance:
            raise PSUError("asked for %.3f A limit, supply reports %.3f A"
                           % (amps, got))
        return got

    def set_output(self, on):
        self.send("OUTP %s" % ("ON" if on else "OFF"), settle=0.5)
        got = self.output_on()
        if got != bool(on):
            raise PSUError("asked for output %s, supply reports %s"
                           % ("ON" if on else "OFF", "ON" if got else "OFF"))
        return got

    def settle(self, target=None, stability=0.01, timeout=8.0, quiet_for=0.5,
               sanity=0.5):
        """Wait until the measured output stops moving, and return it.

        Settling is judged on the reading having gone quiet, **not** on it
        reaching the setpoint, because under load it legitimately will not.
        The supply sits below its setpoint by the source drop -- ~43 mV at
        3.6 A through 12 mOhm on this bench -- plus a fixed ~20 mV by which
        this unit's setpoint reads optimistically against its own meter. An
        earlier version compared against the target within 50 mV and would
        have timed out on a perfectly healthy loaded rail.

        `target` is used only as a loose sanity bound: being a long way off it
        means something is wrong (a current limit engaged, or a cell holding
        the rail up), not that settling needs longer.
        """
        deadline = time.time() + timeout
        stable_since = None
        last = None
        while time.time() < deadline:
            v, _ = self.measure()
            still = last is not None and abs(v - last) <= stability
            last = v
            if still:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= quiet_for:
                    if target is not None and abs(v - target) > sanity:
                        raise PSUError(
                            "output settled at %.3f V but %.3f V was asked for. "
                            "Something is holding the rail: a current limit, or "
                            "a cell still connected in parallel." % (v, target))
                    return v
            else:
                stable_since = None
        raise PSUError("output still moving after %.1f s (last read %.3f V)"
                       % (timeout, last if last is not None else float("nan")))



def load_declared_voltage():
    """The psu_voltage_v that stand.json claims, or None.

    Worth checking because stand.json's declared setup exists precisely for
    values that cannot be read back from the hardware -- and this is the one
    that now can. A stale declaration is otherwise recorded into every CSV
    confidently and looks right forever.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        os.pardir, "stand.json")
    try:
        with open(path) as fh:
            return json.load(fh).get("psu_voltage_v")
    except (OSError, ValueError):
        return None


def probe(psu):
    print("identity      %s" % psu.identify())
    print("output        %s" % ("ON" if psu.output_on() else "OFF"))
    setpoint = psu.voltage_setpoint()
    print("V setpoint    %.3f V" % setpoint)
    print("I limit       %.3f A" % psu.current_limit())
    print("OVP limit     %.3f V   (setpoint ceiling %.3f V)"
          % (psu.ovp_limit(), psu.ovp_limit() - VOLT_LIM_MARGIN_V))
    v, a = psu.measure()
    print("measured      %.3f V, %.3f A" % (v, a))

    declared = load_declared_voltage()
    if declared is None:
        print("\nstand.json declares no psu_voltage_v.")
    elif abs(declared - setpoint) > 0.005:
        print("\n[WARN] stand.json declares psu_voltage_v=%.3f but the supply is "
              "set to %.3f." % (declared, setpoint))
        print("       Every CSV records the declared value, so fix whichever is "
              "wrong before running.")
    else:
        print("\nstand.json psu_voltage_v=%.3f agrees with the supply." % declared)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("--probe", action="store_true",
                    help="Print identity, setpoints and measured output, and "
                         "check them against stand.json")
    ap.add_argument("--set-voltage", type=float, default=None, metavar="V")
    ap.add_argument("--set-current", type=float, default=None, metavar="A",
                    help="Current limit. Note a cell in parallel bypasses it "
                         "entirely, so it only bounds a fault with the cell out")
    ap.add_argument("--output", choices=["on", "off"], default=None)
    ap.add_argument("--max-volts", type=float, default=LI_ION_CEILING_V,
                    help="Refuse setpoints above this (default %.1f, the 1S "
                         "lithium ceiling). Raise it only with any parallel "
                         "cell disconnected" % LI_ION_CEILING_V)
    args = ap.parse_args()

    if not any((args.probe, args.set_voltage is not None,
                args.set_current is not None, args.output)):
        args.probe = True

    psu = None
    try:
        psu = PSU(args.port, max_volts=args.max_volts)
        if args.set_current is not None:
            print("current limit -> %.3f A" % psu.set_current_limit(args.set_current))
        if args.set_voltage is not None:
            print("setpoint      -> %.3f V" % psu.set_voltage(args.set_voltage))
        # Output last, so a setpoint is never applied to a live rail, and the
        # settle check after it -- with the output still off there is nothing
        # to settle and the wait would just time out.
        if args.output:
            print("output        -> %s"
                  % ("ON" if psu.set_output(args.output == "on") else "OFF"))
        if args.set_voltage is not None and psu.output_on():
            print("settled at     %.3f V" % psu.settle(args.set_voltage))
        if args.probe:
            probe(psu)
    except PSUError as exc:
        sys.exit("[ERROR] %s" % exc)
    finally:
        if psu is not None:
            psu.close()


if __name__ == "__main__":
    main()
