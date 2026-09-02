#!/usr/bin/env python3
"""A pty that answers like the Kiprim DC310S, to test the Kv PSU path dry.

fake_stand.py deliberately emulates the firmware and not the supply, so the
--psu-port branch of run_kv has never had a way to run without hardware. This
fills that gap far enough to check *when* the host talks to the supply, which
is the property the mid-hold reading depends on.

Every command is logged with a timestamp. The signature to look for:

    VOLT 3.400        step commanded
    MEAS:ALL? x N     settle(), polled back to back until the rail goes quiet
    <gap>             the firmware's settle delay, rows not yet printing
    MEAS:ALL?         the one mid-window reading -- must land one settle delay
                      after settling, not one settle delay + one log window,
                      which is where it lands if it is taken after END_HOLD

Models the output cap as a first-order lag so settle() has something real to
converge on, and the setpoint's ~20 mV optimism against the supply's own meter.
--cell holds the rail up like a parallel 18650, to exercise the abort path.
"""
import argparse
import os
import pty
import sys
import time

TERM = "\r\n"
SETPOINT_OPTIMISM_V = 0.020   # setpoint reads high against its own meter
LOAD_DROP_V = 0.043           # ~3.6 A through the source impedance
TAU_S = 0.15                  # output cap decay


class FakeSupply:
    def __init__(self, cell_v=None, output_on=True, log=None):
        self.setpoint = 4.0
        self.measured = 4.0 - SETPOINT_OPTIMISM_V - LOAD_DROP_V
        self.target_at = time.time()
        self.current_limit = 5.0
        self.ovp = 5.0
        self.on = output_on
        self.cell_v = cell_v
        self.log = log
        self.t0 = time.time()

    def note(self, cmd):
        line = "%8.3f  %s" % (time.time() - self.t0, cmd)
        print(line, file=sys.stderr)
        if self.log:
            self.log.write(line + "\n")
            self.log.flush()

    def rail_target(self):
        if not self.on:
            return 0.0
        ideal = self.setpoint - SETPOINT_OPTIMISM_V - LOAD_DROP_V
        # A cell in parallel cannot be pulled below its own voltage, which is
        # what makes the low end of a Kv sweep unreachable with one fitted.
        if self.cell_v is not None:
            return max(ideal, self.cell_v)
        return ideal

    def read_volts(self):
        # First-order approach to the target, so an immediate read after a
        # downward step catches the decay rather than the destination.
        target = self.rail_target()
        dt = time.time() - self.target_at
        alpha = 1.0 - pow(2.718281828, -dt / TAU_S)
        self.measured += (target - self.measured) * alpha
        self.target_at = time.time()
        return self.measured

    def read_amps(self):
        return 0.0 if not self.on else 3.60

    def handle(self, cmd):
        self.note(cmd)
        upper = cmd.upper()
        if upper == "*IDN?":
            return "KIPRIM,DC310S,00000000,FV:V4.0.0"
        if upper == "MEAS:ALL?":
            return "%.3f,%.4f" % (self.read_volts(), self.read_amps())
        if upper == "MEAS:VOLT?":
            return "%.3f" % self.read_volts()
        if upper == "MEAS:CURR?":
            return "%.4f" % self.read_amps()
        if upper == "VOLT?":
            return "%.3f" % self.setpoint
        if upper == "CURR?":
            return "%.3f" % self.current_limit
        if upper == "VOLT:LIM?":
            return "%.3f" % self.ovp
        if upper == "OUTP?":
            return "ON" if self.on else "OFF"
        if upper.startswith("VOLT "):
            value = float(cmd.split()[1])
            # The real unit refuses a setpoint within 1.0 V of its OVP limit,
            # and says nothing about it.
            if value > self.ovp - 1.0:
                return None
            self.setpoint = value
            return None
        if upper.startswith("CURR "):
            self.current_limit = float(cmd.split()[1])
            return None
        if upper.startswith("OUTP "):
            self.on = upper.split()[1] == "ON"
            return None
        return "ERR"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", type=float, default=None, metavar="V",
                    help="Model a parallel cell holding the rail at this voltage")
    ap.add_argument("--output-off", action="store_true",
                    help="Start with the output off, to test the up-front check")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    log = open(args.log, "w") if args.log else None
    supply = FakeSupply(cell_v=args.cell, output_on=not args.output_off, log=log)

    master, slave = pty.openpty()
    print(os.ttyname(slave), flush=True)
    print("fake supply on %s" % os.ttyname(slave), file=sys.stderr)

    buf = b""
    try:
        while True:
            try:
                chunk = os.read(master, 4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                cmd = raw.decode(errors="replace").strip()
                if not cmd:
                    continue
                reply = supply.handle(cmd)
                if reply is not None:
                    os.write(master, (reply + TERM).encode())
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()


if __name__ == "__main__":
    main()
