#!/usr/bin/env python3
"""Does the direction of approach decide the reported eRPM? (Bluejay dead-zone test)

Bluejay's `calc_next_comm_period_fast` updates its commutation-period average as

    P4 += floor(N/4) - floor(P4/16)

with both divisions truncating, so P4 stops moving as soon as
floor(P4/16) == floor(N/4) -- every P4 in [16k, 16k+15] is a fixed point. The
telemetry reports ~0.75*P4, so one 16-tick dead zone is ~12 us of reported
period, and *where inside it P4 stops depends on which side it came from*.

That is a sharp prediction testable without touching the ESC: hold one setpoint,
but arrive at it from a higher speed in one case and a lower speed in the other.
If the model is right the two give different reported periods -- the fast
approach landing near the low (correct) end, the slow approach near the high end
-- while voltage and current stay put, because the motor is doing the same thing
either way.

Approach is set up with DSHOT (a direct setpoint that does not spin down), then
DHOLD logs the target without passing through zero.

Usage:
    python tools/approach_test.py --targets 1930,1940,1950
"""

import argparse
import collections
import importlib.util
import os
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "measure", os.path.join(_HERE, os.pardir, "measure.py"))
measure = importlib.util.module_from_spec(_spec)
sys.modules["measure"] = measure
_spec.loader.exec_module(measure)


def hold(link, target, settle_ms=1200, log_ms=1500):
    """One DHOLD at `target`, returning (period histogram, volts, amps, rpm)."""
    link.write_line("DHOLD,%d,%d,%d" % (target, log_ms, settle_ms))
    acc = measure.Accumulator()
    recording = False
    while True:
        line = link.readline()
        if not line:
            continue
        if line.startswith("#"):
            link.absorb_meta(line)
            continue
        if line.startswith("START_HOLD"):
            recording = True
            continue
        if line == "END_HOLD":
            break
        if not recording:
            continue
        row = link.parse_row(line)
        if row is not None:
            acc.add(row)
    return acc.summarize(link, "approach", None, target)


def periods(summary):
    counts = collections.Counter()
    for item in (summary["erpm_raw_hist"] or "").split("|"):
        code, _, n = item.partition(":")
        if n:
            counts[measure.period_us_from_code(code)] += int(n)
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--targets", default="1930,1940,1950",
                    help="DShot setpoints to test, comma separated")
    ap.add_argument("--from-above", type=int, default=2000,
                    help="Setpoint to approach from for the 'fast' case (default 2000)")
    ap.add_argument("--from-below", type=int, default=1700,
                    help="Setpoint to approach from for the 'slow' case (default 1700)")
    ap.add_argument("--approach-s", type=float, default=2.5,
                    help="Seconds to dwell at the approach setpoint (default 2.5)")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("-p", "--port", default=measure.SERIAL_PORT)
    args = ap.parse_args()

    targets = [int(t) for t in args.targets.split(",") if t.strip()]
    link = None
    try:
        print("Connecting to %s..." % args.port)
        link = measure.Link(args.port, measure.BAUD_RATE)
        link.handshake()
        # Same two safety measures every other host path keeps: the firmware
        # watchdog armed and fed, and a finally that zeroes the throttle.
        link.start_keepalive()
        print("--> %s" % link.meta.get("fw", ["?"])[0])
        print("--> bus %.3f V\n" % measure.require_bus_power(link))

        print("Approaching each setpoint from %d (above) and %d (below)."
              % (args.from_above, args.from_below))
        print("Prediction: the two approaches settle on different reported periods")
        print("at the same setpoint, while volts and amps stay put.\n")
        print("  %-7s %-9s %-8s %-8s %-8s %-8s %s"
              % ("dshot", "approach", "period", "rpm", "volts", "amps", "codes"))

        results = collections.defaultdict(dict)
        for rep in range(args.reps):
            for target in targets:
                for label, start in (("above", args.from_above),
                                     ("below", args.from_below)):
                    link.write_line("DSHOT,%d" % start)
                    time.sleep(args.approach_s)
                    s = hold(link, target)
                    counts = periods(s)
                    total = sum(counts.values()) or 1
                    mean_p = sum(p * n for p, n in counts.items()) / total
                    results[target].setdefault(label, []).append(
                        (mean_p, s["rpm_mean"], s["volts_mean"], s["amps_mean"]))
                    print("  %-7d %-9s %-8.2f %-8d %-8.4f %-8.5f %s"
                          % (target, label, mean_p, s["rpm_mean"],
                             s["volts_mean"], s["amps_mean"],
                             " ".join("%d:%d" % (p, n)
                                      for p, n in sorted(counts.items()))))
            link.write_line("0")
            time.sleep(1.0)

        print("\n%-7s %-14s %-14s %-10s %s"
              % ("dshot", "from above", "from below", "delta us", "power delta"))
        for target in targets:
            a = results[target].get("above", [])
            b = results[target].get("below", [])
            if not a or not b:
                continue
            pa = statistics.mean(x[0] for x in a)
            pb = statistics.mean(x[0] for x in b)
            wa = statistics.mean(x[2] * x[3] for x in a)
            wb = statistics.mean(x[2] * x[3] for x in b)
            print("%-7d %-14.2f %-14.2f %-10.2f %.2f%%"
                  % (target, pa, pb, pb - pa, 100.0 * (wb - wa) / wa))
        print("\nA positive delta with power unchanged is the dead zone: same")
        print("operating point, different reported period, selected by history.")

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


if __name__ == "__main__":
    main()
