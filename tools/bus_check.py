#!/usr/bin/env python3
"""Detect I2C transfers the bus mangled, live or in a saved run.

The stand's I2C bus is corrupted whenever the bench supply is switched on --
roughly one transfer in a thousand, at zero load, with the ESC disconnected and
no current flowing. See "The I2C bus is corrupted whenever the bench supply is
on" in BENCH_NOTES.md. This tool is how that was measured and how any fix gets
scored, so a session does not have to reinvent the method.

Two things it counts, and they are independent evidence:

**The probe.** Register 0x1F of the NAU7802 holds a constant whose low nibble is
0xF. The firmware reads it once per conversion, over the same bus, from the same
address, in the same shape of transaction as the ADC data, and reports
`i2c_probe_reads` / `i2c_probe_bad` in `#STATS`. A wrong constant can only be a
wrong TRANSFER, which is what separates a corrupted read from a corrupted
conversion -- a distinction three attempted fixes were made without.

**The outliers.** Corrupted thrust samples themselves, found against a rolling
median. Their raw values are printed in hex because the pattern is the clue: on
this bench the top byte is 0xAA every time while the lower bytes vary, and 0xAA
is 0x55 shifted left one bit, 0x55 being the device's read address byte.

Read the two together. A clean probe is WEAKER evidence than a dirty one,
because the ADC read is three bytes to the probe's one and so is more exposed
per transaction.

**Use --repeat for any comparison.** Not because the rate is overdispersed: that
claim was withdrawn on 2026-09-02. The four counts behind it (37, 37, 24, 11)
were separate runs pooled across the capacitor being tested, and two windows
taken back to back under one configuration gave 29 and 25, agreeing to 0.5σ. The
reason is plainer arithmetic. One 300 s window holds 15 to 20 events, so its own
Poisson bar is around 25%, and no single window can settle a difference smaller
than that.

    python tools/bus_check.py --seconds 300              # one window
    python tools/bus_check.py --seconds 300 --repeat 4   # what a comparison needs
    python tools/bus_check.py --csv data/sweep.csv       # score a saved run

The live modes need the motor stopped and the rail ON, because the fault does
not occur with the supply off -- that is the control, not the measurement.
"""

import argparse
import csv
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

BAUD = 500000
DEFAULT_PORT = "/dev/ttyACM0"

# A corrupted sample lands hundreds of grams out, while ordinary noise is
# hundredths. Anything between those is not a corruption event, so the threshold
# is deliberately far above the noise rather than tuned near it.
OUTLIER_COUNTS = 100000


def _stats(ser, timeout=2.0, banner=None):
    """Ask for #STATS and return it as a dict, or None.

    Pass `banner` to keep the #COLS line. This has to read past whatever else is
    in the buffer to find #STATS, and the boot banner sits there when ID was
    sent just before. Dropping #COLS here left the caller unable to name a
    single column, so the outlier half of the tool silently counted nothing.
    """
    ser.write(b"STATS\n")
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode("ascii", "replace").strip()
        if line.startswith("#COLS") and banner is not None:
            banner["cols"] = line.split(",")[1:]
        if not line.startswith("#STATS"):
            continue
        out = {}
        for item in line.split(","):
            key, sep, value = item.partition("=")
            if sep:
                try:
                    out[key] = float(value)
                except ValueError:
                    out[key] = value
        return out
    return None


def live(port, seconds, tare=True):
    """Capture one window and report both counters."""
    import serial

    ser = serial.Serial(port, BAUD, timeout=1.0)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"ID\n")
    time.sleep(1.0)
    if tare:
        # Without this the NAU7802's boot tare leaves the zero tens of grams
        # out, which does not affect the counts but makes the log unreadable.
        ser.write(b"TARE\n")
        time.sleep(1.5)

    banner = {}
    before = _stats(ser, banner=banner)
    cols = banner.get("cols")
    vals = []
    outliers = []
    end = time.time() + seconds
    while time.time() < end:
        line = ser.readline().decode("ascii", "replace").strip()
        if line.startswith("#COLS"):
            cols = line.split(",")[1:]
        elif line.startswith("D,") and cols:
            fields = line.split(",")[1:]
            if len(fields) != len(cols):
                continue
            row = dict(zip(cols, fields))
            try:
                raw = int(float(row["thrust_raw"]))
            except (KeyError, ValueError):
                continue
            vals.append(raw)
            # Compare against a trailing median, not the whole run: the zero
            # legitimately drifts, and a fixed reference would flag the drift.
            if len(vals) > 40:
                med = statistics.median(vals[-40:])
                if abs(raw - med) > OUTLIER_COUNTS:
                    outliers.append((raw, int(med)))
    after = _stats(ser, banner=banner)
    ser.close()

    reads = bad = 0
    if before and after and "i2c_probe_reads" in after:
        reads = int(after["i2c_probe_reads"]) - int(before.get("i2c_probe_reads", 0))
        bad = int(after["i2c_probe_bad"]) - int(before.get("i2c_probe_bad", 0))
        if reads < 0 or bad < 0:
            # Counters only ever climb, so a negative delta means the board
            # rebooted mid-window and the window is not a measurement.
            print("  [WARN] counters went backwards -- the board rebooted, discard this window")
            reads = bad = 0
    elif after is not None:
        print("  [WARN] no i2c_probe fields in #STATS -- firmware predates the probe, reflash")

    return {"reads": reads, "bad": bad, "samples": len(vals), "outliers": outliers}


def report_live(runs):
    print("\n  %-6s %-10s %-8s %-9s %-8s %s"
          % ("run", "reads", "mangled", "rate", "samples", "outliers"))
    t_reads = t_bad = t_out = 0
    for i, r in enumerate(runs, 1):
        rate = 100.0 * r["bad"] / r["reads"] if r["reads"] else 0.0
        print("  %-6d %-10d %-8d %-9s %-8d %d"
              % (i, r["reads"], r["bad"], "%.3f%%" % rate, r["samples"], len(r["outliers"])))
        t_reads += r["reads"]
        t_bad += r["bad"]
        t_out += len(r["outliers"])

    if not t_reads:
        print("\n  No probe reads. Is the NAU7802 fitted and the firmware current?")
        return
    rate = 100.0 * t_bad / t_reads
    # Poisson: the count is the variance, so this is the honest error bar and
    # the reason a single window cannot settle anything.
    err = 100.0 * (t_bad ** 0.5) / t_reads
    print("\n  total   %d mangled of %d transfers = %.3f%% +- %.3f%% (Poisson)"
          % (t_bad, t_reads, rate, err))
    print("  thrust  %d corrupted samples of %d printed"
          % (t_out, sum(r["samples"] for r in runs)))

    seen = [o for r in runs for o in r["outliers"]]
    if seen:
        print("\n  Corrupted raw values, hex is the clue:")
        for raw, med in seen[:10]:
            print("    raw=%-12d 0x%06X   median=%-10d" % (raw, raw & 0xFFFFFF, med))
        tops = {}
        for raw, _ in seen:
            tops[(raw & 0xFF0000) >> 16] = tops.get((raw & 0xFF0000) >> 16, 0) + 1
        print("    top bytes: %s"
              % ", ".join("0x%02X x%d" % (k, v)
                          for k, v in sorted(tops.items(), key=lambda x: -x[1])))

    if t_bad:
        print("\n  The bus is mangling transfers. A register that cannot change came")
        print("  back wrong, so a corrupt thrust sample is a corrupt READ. Already")
        print("  excluded by measurement: bus speed, the DRDY pin, the DShot signal")
        print("  wire, pull-ups. See BENCH_NOTES.md before repeating any of those.")
    else:
        print("\n  No mangled transfers. Weaker evidence than a hit -- the ADC read is")
        print("  3 bytes to the probe's 1. Confirm the rail was ON: with it off this")
        print("  reads clean by construction and measures nothing.")


def scan_csv(path):
    """Score a saved run. Corrupted samples inflate a step's own stdev."""
    with open(path, newline="") as fh:
        rows = [l for l in fh if not l.startswith("#")]
    rd = list(csv.DictReader(rows))
    if not rd:
        print("%s: no data rows" % path)
        return 1
    cols = rd[0].keys()
    # By NAME, never by index: RESPONSE files use a different column order and
    # an index-based scan silently compares the wrong two columns.
    sd = "thrust_g_std" if "thrust_g_std" in cols else None
    mean = "thrust_g_mean" if "thrust_g_mean" in cols else None
    if not sd:
        print("%s: no thrust_g_std column -- this scan needs an aggregated run "
              "(SWEEP), not a raw one" % path)
        return 1

    sds = []
    for r in rd:
        try:
            sds.append(float(r[sd] or 0))
        except ValueError:
            pass
    if not sds:
        return 1
    typical = statistics.median(sds)
    # A corrupted sample raises its step's stdev by orders of magnitude, so the
    # cut is against the median step rather than an absolute gram figure.
    limit = max(1.0, 50.0 * typical)
    hits = [r for r in rd if float(r[sd] or 0) > limit]

    print("  %s" % path)
    print("    steps            %d" % len(rd))
    print("    median step sd   %.3f g" % typical)
    print("    suspect steps    %d (sd > %.2f g)" % (len(hits), limit))
    if mean:
        peak = max(float(r[mean] or 0) for r in rd)
        print("    peak thrust      %.2f g" % peak)
    for r in hits[:12]:
        print("      %-5s th=%-4s thrust=%8.3f g  sd=%8.3f g"
              % (r.get("direction", "?"), r.get("throttle_pct", "?"),
                 float(r.get(mean) or 0) if mean else 0.0, float(r[sd] or 0)))
    if hits:
        print("\n    Those steps are not measurements. Do not read peak thrust,")
        print("    efficiency or hysteresis from this file.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("--seconds", type=int, default=300,
                    help="Window length. 300 gives ~95k probe reads at 320 SPS, "
                         "which is the size every recorded run used (default 300)")
    ap.add_argument("--repeat", type=int, default=1,
                    help="Windows to run. Use 3 or more for any comparison: the "
                         "rate fluctuates beyond Poisson (default 1)")
    ap.add_argument("--no-tare", action="store_true")
    ap.add_argument("--csv", metavar="FILE",
                    help="Score a saved SWEEP instead of measuring live")
    args = ap.parse_args()

    if args.csv:
        return scan_csv(args.csv)

    runs = []
    for i in range(args.repeat):
        print("  window %d/%d, %ds..." % (i + 1, args.repeat, args.seconds))
        runs.append(live(args.port, args.seconds, tare=not args.no_tare))
    report_live(runs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
