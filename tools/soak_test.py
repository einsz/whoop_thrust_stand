#!/usr/bin/env python3
"""Does the reported eRPM decay with accumulated running time, and recover on rest?

The one pattern that survives every other test on this stand: a high-throttle hold
reads faster the earlier it sits in a run, by up to ~5%, always in the same
direction, while bus voltage and current stay constant. Neither the Bluejay
dead zone (its approach-direction prediction fails, see tools/approach_test.py)
nor supply sag (regulated PSU) nor load cell drift accounts for it.

This repeats one identical setpoint many times, then rests and repeats again, to
turn that impression into a curve: how fast it decays, how far, and how quickly
rest undoes it.

Thrust is the arbiter, not power. Thrust goes as n^2, so it measures true speed
directly; power at constant voltage is nearly the same whether the rotor slowed or
the telemetry drifted, which is why the earlier runs could not close the argument.
The run is kept short and the cell is re-tared before every hold, because HOLD
does not re-tare and the zero walks ~0.2 g over a few minutes -- one order below
the ~0.8 g of thrust change a real 2% slowdown would produce.

Usage:
    python tools/soak_test.py --dshot 2000 --holds 8 --rests 60,240
"""

import argparse
import collections
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


def tare(link, timeout=6.0):
    """Re-zero the cell with the motor stopped, so thrust stays comparable."""
    link.write_line("0")
    time.sleep(0.6)
    link.write_line("TARE")
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = link.readline()
        if line.startswith("#TARE") or line.startswith("#ERROR"):
            return line
    return "#TARE,timeout"


def hold(link, dshot, settle_ms=1200, log_ms=1500):
    link.write_line("DHOLD,%d,%d,%d" % (dshot, log_ms, settle_ms))
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
    s = acc.summarize(link, "soak", None, dshot)
    counts = collections.Counter()
    for item in (s["erpm_raw_hist"] or "").split("|"):
        code, _, n = item.partition(":")
        if n:
            counts[measure.period_us_from_code(code)] += int(n)
    total = sum(counts.values()) or 1
    s["mean_period"] = sum(p * n for p, n in counts.items()) / total
    return s


def show(tag, t0, s, ref):
    w = s["volts_mean"] * s["amps_mean"]
    # A real slowdown must show in thrust as n^2. Report both so they can be
    # compared directly rather than argued about afterwards.
    # Guard the ratios: a hold that produced no rotation would otherwise divide
    # by zero and lose the run, which is exactly when the readings matter least
    # and the message matters most.
    usable = ref and ref["rpm_mean"] and ref["thrust_g_mean"]
    d_rpm = 100.0 * (s["rpm_mean"] / ref["rpm_mean"] - 1) if usable else 0.0
    need = ((1 + d_rpm / 100.0) ** 2 - 1) * 100.0
    got = 100.0 * (s["thrust_g_mean"] / ref["thrust_g_mean"] - 1) if usable else 0.0
    print("  %-11s %-7.1f %-8.2f %-8d %-8.2f %-7.4f %-7.3f %-8s %s"
          % (tag, time.time() - t0, s["mean_period"], s["rpm_mean"],
             s["thrust_g_mean"], s["volts_mean"], w, s["n_codes"],
             "" if not usable else "rpm %+.2f%%  thrust %+.2f%% (n^2 needs %+.2f%%)"
             % (d_rpm, got, need)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dshot", type=int, default=2000)
    ap.add_argument("--holds", type=int, default=8,
                    help="Back-to-back holds in the soak phase (default 8)")
    ap.add_argument("--rests", default="60,240",
                    help="Rest intervals in seconds, each followed by one hold")
    ap.add_argument("--settle-first", type=float, default=0.0,
                    help="Seconds to idle before starting, to begin from a rested ESC")
    ap.add_argument("--abort-drop-pct", type=float, default=5.0,
                    help="End the soak phase once a hold reads this far below the "
                         "first one (default 5%%). The decay is the measurement; "
                         "driving it further only risks the motor")
    ap.add_argument("-p", "--port", default=measure.SERIAL_PORT)
    args = ap.parse_args()

    rests = [int(r) for r in args.rests.split(",") if r.strip()]
    link = None
    try:
        print("Connecting to %s..." % args.port)
        link = measure.Link(args.port, measure.BAUD_RATE)
        link.handshake()
        cfg = measure.load_stand_config()
        link.scale = cfg.get("hx711_scale")
        link.start_keepalive()
        print("--> %s   scale %.4f" % (link.meta.get("fw", ["?"])[0], link.scale))
        print("--> bus %.3f V" % measure.require_bus_power(link))

        if args.settle_first:
            print("\nIdling %.0f s so the ESC starts rested..." % args.settle_first)
            time.sleep(args.settle_first)

        t0 = time.time()
        print("\nSoak: %d identical holds at dshot %d, re-taring before each.\n"
              % (args.holds, args.dshot))
        print("  %-11s %-7s %-8s %-8s %-8s %-7s %-7s %-8s %s"
              % ("phase", "t_s", "period", "rpm", "thrust", "volts", "watts",
                 "n_codes", "vs first"))
        ref = None
        for i in range(args.holds):
            tare(link)
            s = hold(link, args.dshot)
            if ref is None:
                ref = s
            show("soak %d" % (i + 1), t0, s, None if s is ref else ref)

            # A stalled motor at full commanded throttle is the one state that
            # can destroy the part: no airflow, and the drive still pushing.
            # A 2026-08-14 run desynced on hold 39 and this loop cheerfully ran
            # two more. Stop on both the stall and the slow decay -- once the
            # decay is established the curve is the result, and further holds
            # add heat, not information.
            drop = 100.0 * (1 - s["rpm_mean"] / ref["rpm_mean"]) if ref["rpm_mean"] else 0.0
            if s["rpm_mean"] < 0.5 * ref["rpm_mean"]:
                print("\n  *** STALL: %d rpm against %d on the first hold. Stopping"
                      % (s["rpm_mean"], ref["rpm_mean"]))
                print("      the soak. Check the motor and prop before running again.")
                break
            if drop >= args.abort_drop_pct:
                print("\n  *** decay reached -%.2f%% (limit %.2f%%). Soak phase ends"
                      % (drop, args.abort_drop_pct))
                print("      here; the rest phase still runs, to measure recovery.")
                break

        for rest in rests:
            print("\n  resting %d s (motor stopped)..." % rest)
            link.write_line("0")
            time.sleep(rest)
            tare(link)
            s = hold(link, args.dshot)
            show("rest %ds" % rest, t0, s, ref)

        print("\nIf the period climbed through the soak while thrust held up, the")
        print("rotor did not slow and the telemetry drifted. If thrust fell by the")
        print("n^2 amount, the motor really was slowing and this is not a telemetry")
        print("fault at all.")

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
