#!/usr/bin/env python3
# Thrust stand -- host: plots one or more runs
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

"""Plot thrust stand runs (telemetry schema 1).

Reads columns by name rather than position, so adding a column no longer
silently shifts every plot. Flagged samples (saturated or stale sensors) are
excluded from the curves and reported, because plotting them draws a smooth,
plausible line through invalid data.

Steady-state runs default to RPM on the x-axis: throttle % is not comparable
across packs, ESC firmware, or sessions, whereas RPM is a physical quantity.

For the same reason `--thrust iso` plots the density-normalised thrust column,
which is what makes two runs taken on different days comparable at all. It is
not the default because it silently changes what the curve means; the hint
below fires when the runs on screen were taken in air that actually differed.
"""

import argparse
import csv
import os
import sys

import matplotlib
import matplotlib.pyplot as plt


# Set by --save. A run plots exactly one figure -- run_kind() picks one of the
# three plot functions and the others never fire -- so one path is enough and
# there is no numbering scheme to get wrong.
_SAVE_PATH = None


def show_or_save():
    """Open the window, or write the file when --save was given.

    Saving switches the backend to Agg, so this works over ssh with no display.
    That is the point of the flag: the stand lives on a laptop reached by ssh,
    and until now getting a figure off it meant reimplementing the plot.
    """
    if _SAVE_PATH is None:
        plt.show()
        return
    plt.savefig(_SAVE_PATH, dpi=120, bbox_inches="tight")
    plt.close("all")
    print("wrote %s" % _SAVE_PATH)


def parse_csv(path):
    """Returns (metadata dict, list of row dicts) for a schema-1 CSV."""
    if not os.path.exists(path):
        print("[ERROR] file not found: %s" % path, file=sys.stderr)
        return None, None

    meta = {}
    data_lines = []
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                key, _, rest = line[1:].strip().partition(",")
                meta.setdefault(key.strip(), []).append(rest)
            else:
                data_lines.append(line)

    # Columns are read by name, so a future schema that only adds columns will
    # still plot. A bump here means something was renamed or removed.
    if meta.get("schema", ["0"])[0].strip() != "1":
        print("[ERROR] %s is not schema 1 -- it was written by a different "
              "version of measure.py" % path, file=sys.stderr)
        return None, None

    rows = []
    for row in csv.DictReader(data_lines):
        parsed = {}
        for key, value in row.items():
            if key is None:
                continue
            try:
                parsed[key] = float(value) if value not in ("", None) else None
            except ValueError:
                parsed[key] = value
        rows.append(parsed)
    return meta, rows


def run_rho(meta):
    """Air density recorded for the run, or None if it went unrecorded."""
    for entry in meta.get("conditions", []):
        for item in entry.split(","):
            key, _, value = item.partition("=")
            if key.strip() == "rho":
                try:
                    return float(value)
                except ValueError:
                    return None
    return None


# Only flags that invalidate a plotted quantity justify dropping a row.
# esc_alert and edt_stale describe the ESC's own telemetry link: worth surfacing,
# but they say nothing about thrust, RPM or power. Treating every flag as fatal
# can discard an entire run because one ESC status frame set one bit.
# rpm_stale / thrust_stale are likewise not fatal: a summary carries them only
# as the "at least one row had a gap" OR (see n_rpm_stale / n_thrust_stale in
# measure.py), and the means are built from deduplicated fresh samples, so
# dropping the step for one stale row deletes a valid measurement.
FATAL_FLAGS = frozenset((
    "ina_shunt_sat", "ina_i_sat", "watchdog"))


def row_flag_names(row):
    return {n for n in (row.get("flag_names") or "").split("|") if n}


def is_partial(meta):
    """True if this run stopped early and holds only the rows it got to.

    measure.py saves what it collected when a run fails, which is the right
    thing for the samples and the wrong thing to plot silently: a sweep that
    died at 40% throttle draws a curve that simply ends, and nothing about the
    line says why. Runs written before the marker existed report False, which
    is all that can be said about them.
    """
    for entry in meta.get("result", []):
        if entry.strip().startswith("status=aborted"):
            return True
    return False


def drop_flagged(rows, label):
    clean = [r for r in rows if not (row_flag_names(r) & FATAL_FLAGS)]
    dropped = len(rows) - len(clean)
    if dropped:
        names = sorted({n for r in rows for n in row_flag_names(r) & FATAL_FLAGS})
        print("[WARN] %s: excluding %d/%d rows with invalidating flags (%s)"
              % (label, dropped, len(rows), ", ".join(names)))
    noted = sorted({n for r in rows for n in row_flag_names(r) - FATAL_FLAGS})
    if noted:
        print("[note] %s: %s present but kept -- ESC telemetry only, "
              "measurements unaffected" % (label, ", ".join(noted)))
    return clean


# Runs fall into three families with disjoint column sets. Dispatching on the
# mode cell rather than testing for one known value means a new mode fails
# loudly here instead of KeyError-ing deep inside a plot function.
TIME_SERIES_MODES = ("transient", "response", "coastdown")
STEADY_MODES = ("sweep", "static")


def run_kind(rows):
    mode = rows[0].get("mode") if rows else None
    if mode in TIME_SERIES_MODES:
        return "timeseries"
    if mode in STEADY_MODES:
        return "steady"
    if mode == "kv":
        return "kv"
    raise SystemExit("[ERROR] unrecognised run mode %r -- plot_comparison.py needs "
                     "updating alongside measure.py" % mode)


def plot_timeseries(datasets):
    fig, axs = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True)
    fig.suptitle("Step response", fontsize=14, fontweight="bold")

    for label, rows in datasets.items():
        axs[0].plot([r["time_s"] for r in rows], [r["rpm"] for r in rows], label=label, linewidth=1.6)
        # Only rows carrying a fresh HX711 conversion are real thrust measurements;
        # the rest are re-prints of the last one at 250 Hz.
        fresh = [r for r in rows if r.get("new_thrust")]
        axs[1].plot([r["time_s"] for r in fresh], [r["thrust_g"] for r in fresh],
                    linewidth=1.6, marker=".", markersize=3)
        axs[2].plot([r["time_s"] for r in rows], [r["volts"] for r in rows], linewidth=1.6)
        axs[3].plot([r["time_s"] for r in rows], [r["amps"] for r in rows], linewidth=1.6)

    for ax, ylabel in zip(axs, ["Speed\n(RPM)", "Thrust\n(g)", "Voltage\n(V)", "Current\n(A)"]):
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle="--", alpha=0.6)
    axs[3].set_xlabel("Time (s)")
    axs[0].legend(loc="lower right", frameon=True)
    plt.tight_layout()
    show_or_save()


def plot_kv(datasets, metas):
    """Measured points plus the fitted no-load line.

    The gap between them is the I*R drop, i.e. exactly what the naive
    'RPM divided by volts' reading gets wrong.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("Kv: supply sweep at full throttle", fontsize=14, fontweight="bold")

    for label, rows in datasets.items():
        volts = [r["volts_mean"] for r in rows]
        ax1.plot(volts, [r["rpm_mean"] for r in rows], marker="o", linestyle="--",
                 linewidth=1.5, label="%s (measured, loaded)" % label)
        ax2.plot(volts, [r["amps_mean"] for r in rows], marker="s", linestyle="--",
                 linewidth=1.5, label=label)

        fit = {}
        for entry in metas.get(label, {}).get("kv_fit", []):
            for item in entry.split(","):
                k, _, v = item.partition("=")
                try:
                    fit[k] = float(v)
                except ValueError:
                    pass
        if "kv_rpm_per_v" in fit and volts:
            kv = fit["kv_rpm_per_v"]
            span = [min(volts), max(volts)]
            ax1.plot(span, [kv * v for v in span], linewidth=2, alpha=0.8,
                     label="%s fit: Kv=%.0f, R=%.0f mOhm"
                           % (label, kv, fit.get("r_ohm", 0.0) * 1000))

    ax1.set_title("Speed vs supply voltage")
    ax1.set_xlabel("Bus voltage (V)")
    ax1.set_ylabel("RPM")
    ax2.set_title("Current draw")
    ax2.set_xlabel("Bus voltage (V)")
    ax2.set_ylabel("Current (A)")
    for ax in (ax1, ax2):
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(fontsize=8)
    plt.tight_layout()
    show_or_save()


def split_legs(rows):
    """Rows grouped into [(direction, rows)], in the order they were recorded.

    A sweep carries an up and a down leg; measure.py reports the gap between
    them because it is a real effect (settling, motor heating), so the plot has
    to keep them apart rather than joining them into one line.
    """
    legs = []
    for row in rows:
        leg = row.get("direction") or "up"
        if not legs or legs[-1][0] != leg:
            legs.append((leg, []))
        legs[-1][1].append(row)
    return legs


def leg_label(label, leg, legs):
    return label if len(legs) < 2 else "%s (%s)" % (label, leg)


def plot_steady(datasets, xaxis, thrust="measured"):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.suptitle("Steady-state performance", fontsize=14, fontweight="bold")

    xkey, xlabel = {
        "rpm": ("rpm_mean", "Rotational speed (RPM)"),
        "throttle": ("throttle_pct", "Command throttle (%)"),
    }[xaxis]
    tkey, tlabel = {
        "measured": ("thrust_g_mean", "Thrust (g)"),
        "iso": ("thrust_g_iso", "Thrust (g, normalised to ISA density)"),
    }[thrust]

    for label, rows in datasets.items():
        if any(r.get(tkey) is None for r in rows):
            sys.exit("[ERROR] %s has no %s values -- the run was taken without ambient "
                     "data, so it cannot be density-normalised after the fact" % (label, tkey))
        style = dict(linewidth=2, alpha=0.9) if rows[0].get("mode") == "sweep" else \
                dict(marker="o", markersize=5, linestyle="--", linewidth=1.5)
        # One line per leg. A sweep's rows are written up leg then down leg, both
        # ascending in throttle, so drawing them as a single line jumps from the
        # top of the up leg back to the bottom of the down leg -- a straight
        # chord across the whole chart that is pure plotting artifact.
        legs = split_legs(rows)
        color = None
        for leg, leg_rows in legs:
            leg_style = dict(style)
            if len(legs) > 1:
                leg_style["linestyle"] = "-" if leg == "up" else "--"
                leg_style["alpha"] = 0.9 if leg == "up" else 0.65
            line, = ax1.plot([r[xkey] for r in leg_rows], [r[tkey] for r in leg_rows],
                             color=color, label=leg_label(label, leg, legs), **leg_style)
            color = line.get_color()   # keep both legs of a run one colour
            ax2.plot([r[tkey] for r in leg_rows], [r["eff_g_per_w"] for r in leg_rows],
                     color=color, label=leg_label(label, leg, legs), **leg_style)

    ax1.set_title("Thrust")
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(tlabel)

    # Efficiency against thrust, not throttle: the question a pilot asks is
    # "at my hover thrust, which combo draws least power".
    ax2.set_title("Efficiency at a given thrust")
    ax2.set_xlabel(tlabel)
    # Only the thrust axis is normalised. measure.py deliberately leaves power
    # alone -- aerodynamic power scales with density but motor I^2R losses do
    # not -- so these stay g/W as measured even on the ISA axis.
    ax2.set_ylabel("Efficiency (g/W, as measured)")

    for ax in (ax1, ax2):
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend()
    plt.tight_layout()
    show_or_save()


def warn_density_spread(datasets, metas, threshold=0.005):
    """Say so when the runs on screen were not taken in the same air.

    Thrust is proportional to density, so a few percent between sessions is a
    real vertical offset between the curves that has nothing to do with the
    hardware being compared -- and it looks exactly like a better prop.
    """
    rhos = {label: run_rho(metas.get(label, {})) for label in datasets}
    known = {label: rho for label, rho in rhos.items() if rho}
    missing = [label for label in rhos if label not in known]
    if missing:
        print("[WARN] no ambient recorded for: %s -- these curves cannot be "
              "density-corrected" % ", ".join(sorted(missing)))
    if len(known) < 2:
        return
    lo, hi = min(known.values()), max(known.values())
    if (hi - lo) / lo <= threshold:
        return
    print("[WARN] runs span %.1f%% in air density (rho %.4f to %.4f)."
          % (100 * (hi - lo) / lo, lo, hi))
    print("       Thrust scales with density, so part of the gap between these")
    print("       curves is the weather. Re-run with --thrust iso to compare.")


def main():
    parser = argparse.ArgumentParser(description="Plot thrust stand CSVs (schema 1)")
    parser.add_argument("files", nargs="+")
    parser.add_argument("-x", "--xaxis", choices=["rpm", "throttle"], default="rpm",
                        help="x-axis for steady-state plots (default: rpm)")
    parser.add_argument("-t", "--thrust", choices=["measured", "iso"], default="measured",
                        help="thrust column for steady-state plots: as measured, or "
                             "normalised to ISA density for cross-day comparison")
    parser.add_argument("--keep-flagged", action="store_true",
                        help="Plot rows with sensor flags instead of excluding them")
    parser.add_argument("--save", metavar="PATH",
                        help="Write the figure to PATH instead of opening a "
                             "window. Forces the Agg backend, so it works over "
                             "ssh with no display. The extension picks the format")
    args = parser.parse_args()

    global _SAVE_PATH
    if args.save:
        # Before any figure exists, and force=True because pyplot is already
        # imported at module scope.
        matplotlib.use("Agg", force=True)
        _SAVE_PATH = args.save

    datasets = {}
    metas = {}
    kinds = set()
    for path in args.files:
        label = os.path.basename(path).replace(".csv", "")
        meta, rows = parse_csv(path)
        if not rows:
            continue
        if is_partial(meta):
            # Into the label, not just the console: the figure gets shared and
            # the warning does not travel with it.
            reason = meta["result"][0].partition("reason=")[2]
            print("[WARN] %s is a PARTIAL run (%s) -- plotted as (partial)"
                  % (label, reason or "no reason recorded"), file=sys.stderr)
            label += " (partial)"
        metas[label] = meta
        if not args.keep_flagged:
            rows = drop_flagged(rows, label)
        if not rows:
            print("[WARN] %s: nothing left after excluding flagged rows" % label)
            continue
        datasets[label] = rows
        kinds.add(run_kind(rows))

    if not datasets:
        sys.exit("no plottable data")
    if len(kinds) > 1:
        sys.exit("[ERROR] cannot mix %s runs in one figure" % " and ".join(sorted(kinds)))

    kind = kinds.pop()
    if kind == "timeseries":
        plot_timeseries(datasets)
    elif kind == "kv":
        plot_kv(datasets, metas)
    else:
        if args.thrust == "measured":
            warn_density_spread(datasets, metas)
        plot_steady(datasets, args.xaxis, args.thrust)


if __name__ == "__main__":
    main()
