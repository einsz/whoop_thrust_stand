#!/usr/bin/env python3
"""Provoke and detect the half-speed branch, the stand's open drive defect.

The motor intermittently mis-commutates into a slower branch: 100-400 ms
excursions from ~18,000 to ~12,000 RPM at *higher* current, several per 10 s
hold. See "The branch is sub-second DIPPING" in BENCH_NOTES.md. Current rising
through the dip is the signature that separates it from lost telemetry.

**Approach is the only handle ever found on it.** Jumping to the setpoint from a
standstill provoked dips in 4 of 8 runs, while a 1% staircase up from idle or a
step down from th 50 gave none across 36 s. This tool therefore jumps from rest,
every time, and that is not an implementation detail to tidy away.

**Run it FIRST, before anything else spins the motor.** Provocability fades
within a session, fast: 4 of 8, then 1 of 10, then 0 of 12 across roughly one
hour on 2026-08-20 with nothing changed. A batch run at the end of a session is
weak evidence, which is how 0 of 22 was collected on 2026-09-02.

**A clean run of attempts is not a fix.** That error has been made twice on this
bench, once on four sweeps and once on eleven. When the fault IS live, drop
everything and take the two measurements that need it: a scope trace via
tools/phase_probe.py triggered below 13,500 RPM, and the old ~19 kKV motor swap.
Lead permutation does not discriminate and must not be attempted -- it re-pairs
channels with windings while leaving every one of them in the circuit.

    python tools/branch_probe.py                        # th 20, 10 attempts
    python tools/branch_probe.py --throttles 14,16,18,22 --attempts 3
"""

import argparse
import statistics
import sys
import time

BAUD = 500000
DEFAULT_PORT = "/dev/ttyACM0"

# A dip is a drop to roughly 65% of the upper branch. The cut sits well above
# that and well below ordinary hold scatter, which measured 6% at worst across
# 22 clean attempts on 2026-09-02.
DIP_FRACTION = 0.85
# Shorter excursions are single-sample noise rather than the 100-400 ms branch.
MIN_DIP_MS = 50
PHASE_SPINDOWN = "8"


def get_cols(ser):
    """Capture #COLS once, before any buffer reset can destroy it.

    The firmware prints it only in the banner. Reading it inside the harvest
    loop loses it to the reset_input_buffer() there -- the same defect
    bus_check.py shipped with, hit twice in one day.
    """
    ser.reset_input_buffer()
    ser.write(b"ID\n")
    deadline = time.time() + 5
    while time.time() < deadline:
        line = ser.readline().decode("ascii", "replace").strip()
        if line.startswith("#COLS"):
            return line.split(",")[1:]
    return None


def hold(ser, cols, pct, log_ms, settle_ms, budget):
    """One jump-from-rest hold, feeding the watchdog throughout.

    PING and STOP are the only commands honoured mid-sequence, and without PING
    the firmware aborts the hold in well under a second. measure.py runs a
    daemon thread for this; anything driving a sequence directly must do it
    itself.
    """
    rows = []
    ser.reset_input_buffer()
    ser.write(b"HOLD,%d,%d,%d\n" % (pct, log_ms, settle_ms))
    last_ping = time.time()
    deadline = time.time() + budget
    while time.time() < deadline:
        if time.time() - last_ping > 0.2:
            ser.write(b"PING\n")
            last_ping = time.time()
        line = ser.readline().decode("ascii", "replace").strip()
        if not line:
            continue
        if line.startswith("END_"):
            break
        if line.startswith("D,"):
            f = line.split(",")[1:]
            if len(f) == len(cols):
                rows.append(dict(zip(cols, f)))
    return rows


def find_dips(rows):
    """Excursions below DIP_FRACTION of the run's own upper branch.

    Referenced to the run's own median rather than to an absolute RPM, because
    the upper branch moves with throttle, supply and thermal state. Spin-down
    rows are excluded: the ramp at the end of every hold would otherwise read
    as one enormous dip.
    """
    live = [r for r in rows
            if r.get("phase") != PHASE_SPINDOWN and float(r["rpm"]) > 500]
    if len(live) < 100:
        return None, [], None
    t = [float(r["t_us"]) * 1e-6 for r in live]
    rpm = [float(r["rpm"]) for r in live]
    ua = [float(r["bus_ua"]) for r in live]
    upper = statistics.median(rpm)
    cut = DIP_FRACTION * upper
    dips, start = [], None
    for i, v in enumerate(rpm):
        if v < cut:
            if start is None:
                start = i
        elif start is not None:
            ms = (t[i] - t[start]) * 1000
            if ms > MIN_DIP_MS:
                dips.append((ms, min(rpm[start:i]),
                             statistics.mean(ua[start:i]) / 1e6))
            start = None
    return upper, dips, min(rpm)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("--throttles", default="20",
                    help="Throttle percentages to test, comma separated. The "
                         "collapse was recorded worst at th 14-22 (default 20)")
    ap.add_argument("--attempts", type=int, default=10,
                    help="Attempts per throttle (default 10)")
    ap.add_argument("--log-ms", type=int, default=8000,
                    help="Logged window per attempt; the firmware caps a hold "
                         "at 10 s (default 8000)")
    ap.add_argument("--rest-s", type=float, default=4.0,
                    help="Seconds stopped between attempts (default 4)")
    args = ap.parse_args()

    import serial
    throttles = [int(v) for v in args.throttles.split(",") if v.strip()]

    ser = serial.Serial(args.port, BAUD, timeout=0.2)
    try:
        time.sleep(2.0)
        cols = get_cols(ser)
        if not cols:
            return "no #COLS from firmware -- is it running and schema 1?"
        ser.write(b"STOP\n")
        time.sleep(1.0)

        seen = 0
        trials = [(t, n) for t in throttles for n in range(1, args.attempts + 1)]
        for thr, n in trials:
            rows = hold(ser, cols, thr, args.log_ms, 500, args.log_ms / 1000.0 + 22)
            upper, dips, low = find_dips(rows)
            if upper is None:
                print("th %2d #%d: only %d rows, skipped" % (thr, n, len(rows)))
                continue
            base = statistics.median(float(r["bus_ua"]) for r in rows) / 1e6
            if dips:
                seen += 1
                print("th %2d #%d: upper %6.0f RPM, base %.3f A   ** %d DIP(S) **"
                      % (thr, n, upper, base, len(dips)))
                for ms, lo, amps in dips:
                    print("            %5.0f ms  min %6.0f RPM (%.0f%%)  %.3f A (%+.1f%%)"
                          % (ms, lo, 100.0 * lo / upper, amps,
                             100.0 * (amps - base) / base if base else 0.0))
            else:
                print("th %2d #%d: upper %6.0f RPM, base %.3f A, min %6.0f RPM "
                      "(%.0f%%), clean"
                      % (thr, n, upper, base, low, 100.0 * low / upper))
            ser.write(b"STOP\n")
            time.sleep(args.rest_s)

        print("\n%d of %d attempts showed a dip" % (seen, len(trials)))
        if not seen:
            print("A clean run of attempts is NOT a fix. Provocability fades within")
            print("a session, so a negative taken late in one says very little --")
            print("run this first, before anything else spins the motor.")
        else:
            print("THE FAULT IS LIVE. Drop everything and take the measurements that")
            print("need it: a scope trace (tools/phase_probe.py, trigger below")
            print("13,500 RPM) and the old ~19 kKV motor swap.")
    finally:
        ser.write(b"STOP\n")
        time.sleep(0.5)
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
