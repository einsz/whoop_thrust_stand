#!/usr/bin/env python3
"""Read the bus during a ramped stop, so the ramp rate can be tuned on data.

-m KV and friends write per-step summaries, so the phase-8 rows the firmware now
emits during the stop never reach a CSV. This drives one DHOLD directly and
keeps every row, then reports what the rail did while the motor was giving its
energy back.

    spindown_probe.py --volts 3.5 --dshot 2000
"""
import argparse
import importlib.util
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MEASURE = os.path.join(HERE, "measure.py")
PSU_PY = os.path.join(HERE, "tools", "psu.py")
PHASE_SPINDOWN = 8


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--volts", type=float, default=3.5)
    ap.add_argument("--dshot", type=int, default=2000)
    ap.add_argument("--log-ms", type=int, default=500)
    ap.add_argument("--settle-ms", type=int, default=1500)
    ap.add_argument("--no-psu", action="store_true")
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("--psu-port", default="/dev/ttyUSB0", metavar="DEV")
    args = ap.parse_args()

    measure = load(MEASURE, "measure")
    link = measure.Link(args.port, measure.BAUD_RATE)
    link.write_line("ID")
    deadline = time.time() + 3.0
    while time.time() < deadline and link.cols is None:
        line = link.readline()
        if line.startswith("#"):
            link.absorb_meta(line)
    if link.cols is None:
        sys.exit("no banner")

    psu = None
    if not args.no_psu:
        psu_mod = load(PSU_PY, "psu")
        psu = psu_mod.PSU(args.psu_port)
        psu.set_voltage(args.volts)
        print("supply settled at %.3f V" % psu.settle(args.volts))

    link.start_keepalive()
    rows, recording = [], False
    quiet = measure.SilenceTimer("END_HOLD")
    try:
        link.write_line("DHOLD,%d,%d,%d" % (args.dshot, args.log_ms, args.settle_ms))
        while True:
            line = link.readline()
            if not line:
                quiet.check()
                continue
            quiet.feed()
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
                rows.append(row)
    finally:
        link.write_line("STOP")
        link.close()
        if psu is not None:
            psu.close()

    hold = [r for r in rows if r["phase"] != PHASE_SPINDOWN]
    spin = [r for r in rows if r["phase"] == PHASE_SPINDOWN]
    if not spin:
        sys.exit("no spindown rows -- firmware may predate the instrumented ramp")

    def volts(r):
        return r["bus_mv"] / 1000.0

    def amps(r):
        return r["bus_ua"] / 1e6

    v_hold = sum(volts(r) for r in hold) / len(hold) if hold else float("nan")
    rpm_hold = max(r["rpm"] for r in hold) if hold else 0
    t0 = spin[0]["t_us"]
    peak = max(spin, key=volts)
    trough = min(spin, key=amps)

    print("\nhold: %d rows, %.3f V mean, %d RPM" % (len(hold), v_hold, rpm_hold))
    print("spindown: %d rows over %.0f ms" % (len(spin), (spin[-1]["t_us"] - t0) / 1000.0))
    print("  peak bus     %.3f V  at t+%.0f ms  (%+.3f V above the hold)"
          % (volts(peak), (peak["t_us"] - t0) / 1000.0, volts(peak) - v_hold))
    print("  most negative current %.3f A at t+%.0f ms"
          % (amps(trough), (trough["t_us"] - t0) / 1000.0))
    print("  headroom to an 8.000 V cutout: %.3f V" % (8.0 - volts(peak)))

    print("\n  t (ms)   bus V    bus A     RPM")
    step = max(1, len(spin) // 25)
    for r in spin[::step]:
        print("  %-8.1f %-8.3f %-9.3f %d"
              % ((r["t_us"] - t0) / 1000.0, volts(r), amps(r), r["rpm"]))


if __name__ == "__main__":
    main()
