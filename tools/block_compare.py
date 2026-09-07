#!/usr/bin/env python3
# Thrust stand -- compare two blocks of repeated sweeps
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

"""Plot one block of sweeps against another, with the ratio underneath.

    block_compare.py "A=data/branchbase_[0-9]_*.csv" "B=data/motorb_[0-9]_*.csv"

Two or more blocks. Ratios are taken against the first, so name the reference
block first: usually the one you trust most, or the earliest.

`plot_comparison.py` answers a different question. It overlays runs so a reader
can see whether a curve has the shape it should, and with sixteen files it draws
sixteen legend entries over two sets of curves that differ by two percent. That
is unreadable by construction: at this bench's repeatability the difference
between two motors is thinner than the line width.

So this plots the **mean of each block** with its full spread as a band, and
puts the **ratio** in its own panel. A 2% difference is invisible on a curve and
obvious on a ratio, which is why the ratio panel is the point of the tool rather
than a decoration.

Two axes are compared, and they answer different questions:

- **Current against RPM.** The load cell is not in this path, so a difference
  here cannot come from calibration. This is the honest comparison when the two
  blocks were taken at different scale factors, which any block either side of a
  mount swap will be.
- **Power against thrust.** What the setup actually delivers, calibration and
  all. Read it knowing the thrust axis carries each block's own factor.

**Supply-limited steps are dropped, not plotted.** When the bench supply is
current limiting, the top of a sweep stops being a property of the motor: the
rail sags, and the lossier motor sags further and reads as if it were weaker.
`--amp-limit` sets the cut and the excluded fraction is printed, so a comparison
never silently includes clipped steps. Pass `--amp-limit 0` to keep everything.

Only up legs are used. The down leg reaches a given RPM from a different
direction and at a different temperature, and mixing the two doubles the spread
for nothing.
"""

import argparse
import csv
import glob
import statistics as st
import sys

# Categorical hues in fixed order, validated for colour-vision deficiency
# separation against a light surface. Do not cycle them. A fourth block needs a
# hue that passes the same check, not the next one that looks different: the
# obvious green fails deuteranopia separation against this orange.
COLORS = ["#3b6bc4", "#c2591a", "#7a4fa3"]
BAND_ALPHA = 0.18
# What counts as a lock rather than ordinary step-to-step scatter.
THRESH = 35.0
GRID = "#d8d8d4"
INK = "#2a2a28"
MUTED = "#6b6b66"


def read_block(pattern, amp_limit):
    """Up-leg rows from every file matching one glob, supply-limited steps cut."""
    runs, dropped, total = [], 0, 0
    for path in sorted(glob.glob(pattern)):
        with open(path) as fh:
            rows = [r for r in csv.DictReader(
                line for line in fh if not line.startswith("#"))]
        if not rows or "direction" not in rows[0]:
            print("  skipped %s: not a sweep CSV" % path, file=sys.stderr)
            continue
        up = [r for r in rows if r.get("direction") == "up"]
        total += len(up)
        if amp_limit:
            kept = [r for r in up if float(r["amps_mean"]) < amp_limit]
            dropped += len(up) - len(kept)
            up = kept
        if len(up) > 2:
            runs.append(up)
    return runs, dropped, total


def curve(run, xcol, ycol):
    """(x, y) sorted on x, from one run."""
    pts = sorted((float(r[xcol]), float(r[ycol])) for r in run
                 if r.get(xcol) not in (None, "") and r.get(ycol) not in (None, ""))
    return [p[0] for p in pts], [p[1] for p in pts]


def interp(xs, ys, x):
    for i in range(1, len(xs)):
        if xs[i - 1] <= x <= xs[i]:
            if xs[i] == xs[i - 1]:
                return ys[i]
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return None


def aggregate(runs, xcol, ycol, n_grid=60):
    """Mean, min and max of y over a common x grid shared by every run.

    The grid is the intersection of the runs' x ranges, so no point is an
    extrapolation and the band is a real spread rather than an edge artefact.
    """
    curves = [curve(r, xcol, ycol) for r in runs]
    curves = [c for c in curves if len(c[0]) > 2]
    if not curves:
        return [], [], [], []
    lo = max(min(xs) for xs, _ in curves)
    hi = min(max(xs) for xs, _ in curves)
    if not hi > lo:
        return [], [], [], []
    grid = [lo + (hi - lo) * i / (n_grid - 1) for i in range(n_grid)]
    mean, low, high = [], [], []
    for x in grid:
        vals = [v for v in (interp(xs, ys, x) for xs, ys in curves) if v is not None]
        if len(vals) < len(curves):
            continue
        mean.append(st.mean(vals))
        low.append(min(vals))
        high.append(max(vals))
    return grid[:len(mean)], mean, low, high


def label_at_max_spread(ax, curves):
    """Direct-label each curve where the curves are furthest apart.

    Labelling at the right-hand end is the obvious choice and the wrong one
    here: these curves converge at full throttle, so three labels land on top
    of each other exactly where the data says least.
    """
    if len(curves) < 2:
        return
    grids = [g for g, _, _ in curves]
    lo = max(g[0] for g in grids)
    hi = min(g[-1] for g in grids)
    best, best_spread = None, -1.0
    for i in range(40):
        x = lo + (hi - lo) * i / 39.0
        vals = [interp(g, m, x) for g, m, _ in curves]
        if any(v is None for v in vals):
            continue
        spread = (max(vals) - min(vals)) / (abs(st.mean(vals)) or 1.0)
        if spread > best_spread:
            best, best_spread = x, spread
    if best is None:
        return
    placed = []
    for grid, mean, (label, color) in curves:
        y = interp(grid, mean, best)
        if y is not None:
            placed.append((y, label, color))
    if len(placed) < 2:
        return
    span = max(y for y, _, _ in placed) - min(y for y, _, _ in placed)
    lo, hi = ax.get_ylim()
    # Curves that sit on top of each other cannot be direct-labelled without
    # the labels sitting on top of each other too. The legend already names
    # them, so say nothing rather than something unreadable.
    if hi > lo and span < 0.05 * (hi - lo):
        return
    # Stagger by rank so two close curves still get two readable labels.
    placed.sort(reverse=True)
    for rank, (y, label, color) in enumerate(placed):
        ax.annotate(label, (best, y), textcoords="offset points",
                    xytext=(6, 8 - 13 * rank), color=color, fontsize=9)


def panel(ax, blocks, xcol, ycol, xlabel, ylabel, title):
    handles, curves = [], []
    for (label, runs), color in zip(blocks, COLORS):
        grid, mean, low, high = aggregate(runs, xcol, ycol)
        if not grid:
            continue
        ax.fill_between(grid, low, high, color=color, alpha=BAND_ALPHA, linewidth=0)
        line, = ax.plot(grid, mean, color=color, linewidth=2, label=label)
        handles.append(line)
        curves.append((grid, mean, (label, color)))
    label_at_max_spread(ax, curves)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=9)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=9)
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    return handles


def ratio_panel(ax, blocks, xcol, ycol, xlabel, title):
    """Every block against the first, in percent. Zero is the reference."""
    la, ra = blocks[0]
    ga, ma, _, _ = aggregate(ra, xcol, ycol)
    if not ga:
        return None
    ax.axhline(0, color=MUTED, linewidth=1)
    for (lb, rb), color in zip(blocks[1:], COLORS[1:]):
        gb, mb, _, _ = aggregate(rb, xcol, ycol)
        if not gb:
            continue
        lo, hi = max(ga[0], gb[0]), min(ga[-1], gb[-1])
        grid = [lo + (hi - lo) * i / 59 for i in range(60)]
        pct, xs = [], []
        for x in grid:
            a, b = interp(ga, ma, x), interp(gb, mb, x)
            if a and b:
                xs.append(x)
                pct.append((b / a - 1) * 100)
        if not xs:
            continue
        ax.plot(xs, pct, color=color, linewidth=2)
        ax.annotate(lb, (xs[-1], pct[-1]), textcoords="offset points",
                    xytext=(6, 0), color=INK, fontsize=9, va="center")
    ax.set_xlabel(xlabel, color=MUTED, fontsize=9)
    ax.set_ylabel("vs %s (%%)" % la, color=MUTED, fontsize=9)
    ax.set_title(title, color=INK, fontsize=11, loc="left")


def step_panel(ax, blocks):
    """Each 1% throttle step against the median of its eight neighbours.

    This is the panel that finds locks. A held step reads deeply negative and
    the release that follows reads high; on a plotted curve both are invisible.
    """
    ax.axhline(0, color=MUTED, linewidth=1)
    for (label, runs), color in zip(blocks, COLORS):
        per = {}
        for run in runs:
            rows = sorted(run, key=lambda r: float(r["throttle_pct"]))
            for a, b in zip(rows, rows[1:]):
                if float(b["throttle_pct"]) - float(a["throttle_pct"]) != 1:
                    continue
                per.setdefault(int(float(b["throttle_pct"])), []).append(
                    (float(b["rpm_mean"]) - float(a["rpm_mean"]),
                     float(a["rpm_mean"])))
        pts = [(t, st.mean([d for d, _ in v]), st.mean([r for _, r in v]))
               for t, v in sorted(per.items())]
        pts = [p for p in pts if 30 <= p[0] <= 95]
        if len(pts) < 10:
            continue
        ds = [p[1] for p in pts]
        xs, ys = [], []
        for i, (t, d, rpm0) in enumerate(pts):
            j, k = max(0, i - 4), min(len(ds), i + 5)
            base = st.median([ds[m] for m in range(j, k) if m != i])
            if base:
                xs.append(rpm0)
                ys.append((d / base - 1) * 100)
        # The trace is context and the crossings are the finding, so the trace
        # recedes. Three of these at full weight is an unreadable hairball.
        ax.plot(xs, ys, color=color, linewidth=1.2, alpha=0.45)
        hits = [(x, y) for x, y in zip(xs, ys) if abs(y) >= THRESH]
        if hits:
            ax.plot([h[0] for h in hits], [h[1] for h in hits], "o",
                    color=color, markersize=8, markeredgecolor="white",
                    markeredgewidth=2, linestyle="none")
            worst = max(hits, key=lambda h: abs(h[1]))
            ax.annotate("%s  %+.0f%% at %.0f" % (label, worst[1], worst[0]),
                        worst, textcoords="offset points", xytext=(8, 6),
                        color=color, fontsize=9)
    for edge in (-THRESH, THRESH):
        ax.axhline(edge, color=MUTED, linewidth=1, linestyle=":")
    ax.set_xlabel("RPM", color=MUTED, fontsize=9)
    ax.set_ylabel("step vs local trend (%)", color=MUTED, fontsize=9)
    ax.set_title("Per-step deviation: dots are locks, dotted lines the %d%% cut"
                 % THRESH, color=INK, fontsize=11, loc="left")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("blocks", nargs="+", metavar="LABEL=GLOB",
                    help="two blocks of sweep CSVs, e.g. \"A=data/run_*.csv\"")
    ap.add_argument("--amp-limit", type=float, default=4.0,
                    help="drop steps at or above this current, as supply-limited "
                         "(default 4.0 A; 0 keeps every step)")
    ap.add_argument("--save", metavar="PATH", help="write the figure here")
    args = ap.parse_args()

    if not 2 <= len(args.blocks) <= len(COLORS):
        ap.error("give between 2 and %d blocks" % len(COLORS))

    blocks = []
    for spec in args.blocks:
        if "=" not in spec:
            ap.error("expected LABEL=GLOB, got %r" % spec)
        label, pattern = spec.split("=", 1)
        runs, dropped, total = read_block(pattern, args.amp_limit)
        if not runs:
            ap.error("no usable sweeps matched %r" % pattern)
        print("%-10s %d runs, %d up-leg steps, %d dropped as supply-limited (%.1f%%)"
              % (label, len(runs), total, dropped, 100.0 * dropped / total if total else 0))
        blocks.append((label, runs))

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(12, 12))
    for ax in axes.flat:
        ax.grid(True, color=GRID, linewidth=0.6)
        ax.set_axisbelow(True)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=9)

    handles = panel(axes[0][0], blocks, "rpm_mean", "amps_mean",
                    "RPM", "current (A)", "Current at a given speed")
    ratio_panel(axes[1][0], blocks, "rpm_mean", "amps_mean", "RPM",
                "Current difference")
    panel(axes[0][1], blocks, "thrust_g_iso", "watts_mean",
          "thrust (g, ISA normalised)", "power (W)", "Power for a given thrust")
    ratio_panel(axes[1][1], blocks, "thrust_g_iso", "watts_mean",
                "thrust (g, ISA normalised)", "Power difference")
    step_panel(axes[2][0], blocks)
    panel(axes[2][1], blocks, "rpm_mean", "thrust_g_iso",
          "RPM", "thrust (g, ISA normalised)", "Thrust at a given speed")

    if handles:
        fig.legend(handles=handles, loc="upper right", frameon=False, fontsize=9)
    fig.suptitle("Block comparison: %s, against %s"
                 % (", ".join(b[0] for b in blocks[1:]), blocks[0][0]),
                 color=INK, fontsize=13, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    if args.save:
        fig.savefig(args.save, dpi=130)
        print("wrote %s" % args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
