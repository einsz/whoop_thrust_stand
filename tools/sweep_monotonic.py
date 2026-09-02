#!/usr/bin/env python3
# Thrust stand -- check a sweep for non-monotonic RPM
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

"""Flag sweeps where RPM falls while commanded throttle rises.

A healthy sweep is monotonic within each leg: every 1% step up should raise the
speed, every step down should lower it. A step that goes the wrong way is not
noise -- a motor that answers a *higher* setpoint with a *lower* speed is either
mis-commutating or losing drive on a phase, and both invalidate the numbers
around them.

    sweep_monotonic.py data/*.csv

The check is per-leg and needs no cross-leg comparison, which is what makes it
trustworthy. Comparing the up and down legs at the same throttle looks like the
obvious test and is not: a motor spins up more easily on the way down, so
10-25% gaps just above idle are ordinary, and a sweep that is clean by every
other measure will show several. Judged that way, healthy runs fail. Within one
leg there is no such effect to subtract.

Two gates keep it honest, both tunable:

- `--min-rpm` excludes the bottom of the sweep. Below the motor's reliable idle
  a percentage is taken on near-zero numbers, and the idle hunt, a marginal
  start and ordinary scatter all produce tens of percent there.
- `--dip-pct` is the size of drop worth reporting. Set it below the run-to-run
  scatter of the rig and every sweep flags.

`scatter` is the worst per-step `rpm_std/rpm` over the top half of the speed
range. A sweep can be perfectly monotonic and still be dithering between two
commutation states within each step, which shows up here before it shows up as
a dip.

`asym` counts steps where the legs disagree by more than `--asym-pct` *and* the
slower leg draws more power. That inversion -- more energy in, less speed out --
is what separates a commutation fault from settling lag or thermal drift.

**Both measures are needed, and neither is sufficient.** A leg that enters a
collapsed state and stays there is still internally monotonic: throttle falls,
speed falls, just far too little of it. `dips` sees nothing, because no step
goes the wrong way. Only the comparison against the other leg shows it. So a
large, power-inverted asymmetry flags on its own -- `--asym-flag-pct`, well
above the 10-25% that ordinary hysteresis produces near idle.

Exits non-zero if any file is flagged, so it can gate a batch.
"""
import argparse
import csv
import sys

REQUIRED = ("direction", "throttle_pct", "rpm_mean", "rpm_std", "watts_mean")


def load(path):
    with open(path) as fh:
        rows = list(csv.DictReader(l for l in fh if not l.startswith("#")))
    if not rows:
        raise ValueError("no data rows")
    missing = [c for c in REQUIRED if c not in rows[0]]
    if missing:
        raise ValueError("not a sweep CSV -- missing %s" % ", ".join(missing))
    return rows


def analyse(path, min_rpm, dip_pct, asym_pct, pwr_pct, flag_asym):
    rows = load(path)
    legs = {}
    for r in rows:
        legs.setdefault(r["direction"], {})[int(r["throttle_pct"])] = r

    dips, worst_dip, where = 0, 0.0, None
    for leg, steps in legs.items():
        ths = sorted(steps)
        for a, b in zip(ths, ths[1:]):
            ra = float(steps[a]["rpm_mean"])
            rb = float(steps[b]["rpm_mean"])
            if ra < min_rpm or rb < min_rpm:
                continue
            drop = (rb - ra) / ra * 100.0
            if drop < -dip_pct:
                dips += 1
                if drop < worst_dip:
                    worst_dip, where = drop, "%s %d->%d" % (leg, a, b)

    up, down = legs.get("up", {}), legs.get("down", {})
    asym, worst_asym = 0, 0.0
    for th in sorted(set(up) & set(down)):
        ru, rv = float(up[th]["rpm_mean"]), float(down[th]["rpm_mean"])
        wu, wv = float(up[th]["watts_mean"]), float(down[th]["watts_mean"])
        if ru < min_rpm or rv < min_rpm:
            continue
        rel = (rv - ru) / ru * 100.0
        slower_draws_more = (wv > wu) if rv < ru else (wu > wv)
        margin = abs(wv - wu) / max(wu, wv, 1e-9) * 100.0
        if abs(rel) > asym_pct and slower_draws_more and margin > pwr_pct:
            asym += 1
            if abs(rel) > abs(worst_asym):
                worst_asym = rel

    # Scatter is judged over the top half of the speed range, not over
    # everything above --min-rpm. Relative stdev grows towards idle whatever the
    # motor is doing, so including the low end buries the figure of interest
    # under noise that is always there.
    peak = max(float(r["rpm_mean"]) for r in rows)
    scatter = max(
        (float(r["rpm_std"]) / float(r["rpm_mean"]) * 100.0
         for r in rows if float(r["rpm_mean"]) >= 0.5 * peak),
        default=0.0)
    return {
        "run": path.split("/")[-1],
        "steps": len(rows),
        "dips": dips,
        "worst": "%.1f%%" % worst_dip if dips else "-",
        "at": where or "-",
        "asym": asym,
        "worst_asym": "%+.1f%%" % worst_asym if asym else "-",
        "scatter": "%.2f%%" % scatter,
        # "check" is a single dip: worth looking at, not worth condemning. An
        # asym count below the flag threshold is left as "ok" -- it is in the
        # table either way, and ordinary hysteresis produces a few every run, so
        # promoting those would make the verdict column mean nothing.
        "verdict": ("FLAG" if (dips >= 2 or (asym >= 3 and abs(worst_asym) >= flag_asym))
                    else ("check" if dips else "ok")),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", nargs="+", help="sweep CSVs")
    ap.add_argument("--min-rpm", type=float, default=5000.0,
                    help="ignore steps below this speed (default 5000)")
    ap.add_argument("--dip-pct", type=float, default=3.0,
                    help="report drops larger than this %% (default 3)")
    ap.add_argument("--asym-pct", type=float, default=10.0,
                    help="up/down gap to report, %% (default 10)")
    ap.add_argument("--power-pct", type=float, default=2.0,
                    help="power inversion needed alongside it, %% (default 2)")
    ap.add_argument("--asym-flag-pct", type=float, default=30.0,
                    help="up/down gap that flags on its own, %% (default 30)")
    args = ap.parse_args()

    results, failed = [], False
    for path in args.csv:
        try:
            results.append(analyse(path, args.min_rpm, args.dip_pct,
                                   args.asym_pct, args.power_pct,
                                   args.asym_flag_pct))
        except (OSError, ValueError) as exc:
            print("%s: %s" % (path, exc), file=sys.stderr)
            failed = True

    if not results:
        return 2
    cols = ["run", "steps", "dips", "worst", "at", "asym", "worst_asym",
            "scatter", "verdict"]
    width = {c: max(len(c), max(len(str(r[c])) for r in results)) for c in cols}
    print("  ".join(c.ljust(width[c]) for c in cols))
    for r in results:
        print("  ".join(str(r[c]).ljust(width[c]) for c in cols))

    flagged = [r for r in results if r["verdict"] == "FLAG"]
    if flagged:
        print("\n%d of %d sweeps answered the throttle backwards, or ran a whole leg"
              % (len(flagged), len(results)))
        print("slow at higher power. Those runs are not usable unexplained.")
    return 1 if (flagged or failed) else 0


if __name__ == "__main__":
    sys.exit(main())
