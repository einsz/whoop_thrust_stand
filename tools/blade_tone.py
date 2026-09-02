#!/usr/bin/env python3
# Thrust stand -- host: rotor speed from the blade-pass tone
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

"""Rotor speed measured from sound, with no part of the ESC involved.

Every RPM figure this bench produces comes from the ESC's own telemetry, so it
cannot arbitrate a question about that telemetry. The reported speed drifts down
by up to 5% with running time at a fixed setpoint and recovers after a rest, and
nothing on the stand can say whether the rotor is really slowing or only the
ESC's arithmetic says so. Power cannot settle it either: at constant voltage it
looks nearly the same both ways.

A prop turning at n rev/s with B blades radiates a tone at n*B. That number is a
property of the air, so it does not care what the ESC believes.

    RPM = 60 * f_peak / blades

Why acoustic rather than an optical tachometer: a tacho needs reflective tape on
a blade, and on a 31 mm ultralight that strip is a real fraction of the prop's
mass applied to one blade. You would unbalance the thing you are measuring. A
microphone touches nothing, and it gives speed as a continuous function of time,
which is the shape the drift question actually needs.

Usage:
    python tools/blade_tone.py hold.wav --expect-rpm 47000
    python tools/blade_tone.py hold.wav --compare data/2026-08-14-hold.csv
    python tools/blade_tone.py hold.wav --expect-rpm 47000 --plot out.png

Phone recorders usually write m4a. Convert first:
    ffmpeg -i clip.m4a -ac 1 -ar 48000 clip.wav

Two things will give you a confident wrong answer:

**The blade tone is not the loudest thing in the spectrum.** On the reference
build a 2-per-rev component sits below it at nearly twice the amplitude, so a
wide search band lets the peak-picker wander and rail at a band edge. At the
default --tolerance 15 that produced 5.9% scatter and a slope of the wrong sign;
narrowing to a few percent around the expected tone gave 0.32%. Narrow the band
until the implied blade count printed below lands on the real one, and treat any
result where it does not as invalid.

**Silence tracks noise.** A recording longer than the hold puts the tracker on
room tone for the extra frames, and those frames get a peak like any other. Trim
to the burst before analysing:
    ffmpeg -i run.wav -ss 7.0 -t 9.6 -c copy hold.wav

A fixed-frequency source in the room — a supply fan, say — is worth keeping
rather than removing. Recorded on the same clock as the blade tone, it shows
whether an apparent drift is the rotor or the recorder. Check it does not fall
inside the search band at the speed you are measuring.

Needs numpy. The optional --plot needs matplotlib.
"""

import argparse
import csv
import os
import sys
import wave

try:
    import numpy as np
except ImportError:
    sys.exit("blade_tone needs numpy: pip install numpy")


DEFAULT_BLADES = 3           # HQprop Ultralight 3-blade, the reference prop here


# =========================================================================
# Audio
# =========================================================================

def read_wav(path):
    """Mono float samples and the sample rate.

    Deliberately stdlib: this reads the PCM wav that ffmpeg or Audacity writes
    and nothing else, which keeps the dependency list at numpy.
    """
    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        width = wf.getsampwidth()
        fs = wf.getframerate()
        raw = wf.readframes(wf.getnframes())

    if width == 1:                       # unsigned 8-bit
        x = (np.frombuffer(raw, dtype=np.uint8).astype(np.float64) - 128.0) / 128.0
    elif width == 2:
        x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif width == 4:
        x = np.frombuffer(raw, dtype="<i4").astype(np.float64) / 2147483648.0
    elif width == 3:                     # 24-bit packed, sign-extended by hand
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        v = np.where(v & 0x800000, v - 0x1000000, v)
        x = v.astype(np.float64) / 8388608.0
    else:
        raise ValueError("unsupported sample width: %d bytes" % width)

    if channels > 1:
        x = x.reshape(-1, channels).mean(axis=1)
    return fs, x


def refine_peak(mag, k):
    """Sub-bin peak position by parabolic interpolation on the log magnitude.

    The FFT bin alone is far too coarse here. A 0.25 s window at 48 kHz gives
    4 Hz bins, which at 2,350 Hz is 0.17% and the effect being chased is 5%.
    Interpolating across the peak and its neighbours recovers roughly an order
    of magnitude, taking the estimate well below 0.05%.
    """
    if k <= 0 or k >= len(mag) - 1:
        return float(k)
    a, b, c = (np.log(mag[k - 1] + 1e-20),
               np.log(mag[k] + 1e-20),
               np.log(mag[k + 1] + 1e-20))
    denom = a - 2 * b + c
    if denom == 0:
        return float(k)
    return k + 0.5 * (a - c) / denom


def track_tone(x, fs, f_lo, f_hi, win_s, hop_s):
    """Follow the strongest peak inside [f_lo, f_hi] over time.

    Returns (t, f, snr). `snr` is the peak height over the median of the search
    band, which is what says whether a frame found the tone or found noise.
    """
    win = int(round(win_s * fs))
    hop = max(1, int(round(hop_s * fs)))
    if win < 64 or len(x) < win:
        raise ValueError("recording too short for a %.2f s window" % win_s)
    window = np.hanning(win)
    freqs = np.fft.rfftfreq(win, 1.0 / fs)
    lo = int(np.searchsorted(freqs, f_lo))
    hi = int(np.searchsorted(freqs, f_hi))
    if hi - lo < 4:
        raise ValueError("search band %.0f-%.0f Hz is narrower than the FFT "
                         "resolution; use a longer --window" % (f_lo, f_hi))

    ts, fpk, snrs = [], [], []
    for start in range(0, len(x) - win + 1, hop):
        seg = x[start:start + win] * window
        mag = np.abs(np.fft.rfft(seg))
        band = mag[lo:hi]
        k = int(np.argmax(band)) + lo
        kk = refine_peak(mag, k)
        ts.append((start + win / 2.0) / fs)
        fpk.append(kk * fs / win)
        med = float(np.median(band)) + 1e-20
        snrs.append(float(mag[k]) / med)
    return np.array(ts), np.array(fpk), np.array(snrs)


# =========================================================================
# Stand CSV, for comparison
# =========================================================================

def read_stand_csv(path):
    """(mean reported RPM, per-row [(time_s, rpm)] if the file has a clock).

    Handles both steady-state files, which carry one row per throttle step, and
    per-sample files. Rows with no usable RPM are skipped rather than counted as
    zero, which would drag a mean toward the floor.
    """
    with open(path) as fh:
        rows = list(csv.DictReader([l for l in fh if not l.startswith("#")]))
    if not rows:
        return None, []
    cols = rows[0].keys()
    rpm_key = next((k for k in ("rpm_mean", "rpm") if k in cols), None)
    if rpm_key is None:
        return None, []
    t_key = "time_s" if "time_s" in cols else None

    vals, series = [], []
    for r in rows:
        try:
            v = float(r[rpm_key])
        except (TypeError, ValueError):
            continue
        if v <= 0:
            continue
        vals.append(v)
        if t_key:
            try:
                series.append((float(r[t_key]), v))
            except (TypeError, ValueError):
                pass
    return (sum(vals) / len(vals) if vals else None), series


def slope_per_s(t, y):
    """Least-squares slope in units per second, or None if degenerate."""
    if len(t) < 3:
        return None
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    tm, ym = t.mean(), y.mean()
    denom = float(((t - tm) ** 2).sum())
    if denom == 0:
        return None
    return float(((t - tm) * (y - ym)).sum() / denom)


# =========================================================================

def main():
    p = argparse.ArgumentParser(
        description="Rotor RPM from the blade-pass tone, independent of the ESC")
    p.add_argument("wav", help="mono or stereo PCM wav of the motor running")
    p.add_argument("--blades", type=int, default=DEFAULT_BLADES,
                   help="blade count (default %d, the HQprop Ultralight 3-blade "
                        "fitted here). Wrong by a factor here is wrong by the "
                        "same factor in every RPM out" % DEFAULT_BLADES)
    p.add_argument("--expect-rpm", type=float, default=None,
                   help="roughly what the stand reported, used to centre the "
                        "search band. Taken from --compare when not given")
    p.add_argument("--compare", default=None, metavar="CSV",
                   help="a stand CSV from the same hold, to put the telemetry "
                        "next to the acoustic answer")
    p.add_argument("--tolerance", type=float, default=15.0,
                   help="search band as %% either side of the expected tone "
                        "(default 15). Comfortably wider than the ~5%% drift "
                        "being chased, so the band cannot follow it")
    p.add_argument("--window", type=float, default=0.25,
                   help="FFT window in seconds (default 0.25)")
    p.add_argument("--hop", type=float, default=0.05,
                   help="hop between windows in seconds (default 0.05)")
    p.add_argument("--min-snr", type=float, default=4.0,
                   help="drop frames whose peak is below this multiple of the "
                        "band median (default 4)")
    p.add_argument("--skip", type=float, default=0.0,
                   help="seconds to discard from the start, for spool-up")
    p.add_argument("-o", "--output", default=None, metavar="CSV",
                   help="write the per-frame series here")
    p.add_argument("--plot", default=None, metavar="PNG",
                   help="save a plot of RPM against time (needs matplotlib)")
    args = p.parse_args()

    if args.blades < 1:
        sys.exit("--blades must be at least 1")

    fs, x = read_wav(args.wav)
    if args.skip > 0:
        x = x[int(args.skip * fs):]
    dur = len(x) / fs
    print("%s: %.1f s, %d Hz, %d samples" % (os.path.basename(args.wav), dur, fs, len(x)))

    csv_mean, csv_series = (None, [])
    if args.compare:
        csv_mean, csv_series = read_stand_csv(args.compare)
        if csv_mean is None:
            print("[WARN] no usable RPM column in %s" % args.compare)

    expect = args.expect_rpm if args.expect_rpm is not None else csv_mean
    if expect is None:
        sys.exit("need --expect-rpm, or --compare with a CSV carrying an RPM column.\n"
                 "Without one the strongest peak in a wide band is as likely to be a\n"
                 "harmonic, or the motor's own whine, as the blade tone.")

    f_expect = expect / 60.0 * args.blades
    tol = args.tolerance / 100.0
    f_lo, f_hi = f_expect * (1 - tol), f_expect * (1 + tol)
    if f_hi > fs / 2:
        sys.exit("blade tone at ~%.0f Hz is above this recording's Nyquist (%.0f Hz)"
                 % (f_expect, fs / 2))
    print("expecting ~%.0f RPM -> %.0f Hz on %d blades; searching %.0f-%.0f Hz"
          % (expect, f_expect, args.blades, f_lo, f_hi))

    t, f, snr = track_tone(x, fs, f_lo, f_hi, args.window, args.hop)
    good = snr >= args.min_snr
    if good.sum() < 3:
        sys.exit("only %d of %d frames cleared --min-snr %.1f. Either the tone is "
                 "buried, or the search band is in the wrong place: check --blades "
                 "and --expect-rpm." % (int(good.sum()), len(t), args.min_snr))
    t, f, snr = t[good], f[good], snr[good]
    rpm = 60.0 * f / args.blades

    bin_hz = fs / int(round(args.window * fs))
    print("kept %d of %d frames (snr >= %.1f); bin %.1f Hz = %.3f%% before "
          "interpolation" % (len(t), len(good), args.min_snr, bin_hz,
                             100 * bin_hz / f_expect))

    print("\nAcoustic RPM")
    print("  mean      %8.0f" % rpm.mean())
    print("  min-max   %8.0f  %.0f" % (rpm.min(), rpm.max()))
    print("  sd        %8.0f  (%.2f%%)" % (rpm.std(), 100 * rpm.std() / rpm.mean()))
    ac_slope = slope_per_s(t, rpm)
    if ac_slope is not None:
        span = t[-1] - t[0]
        print("  drift     %+8.1f RPM/s  (%+.0f RPM over %.1f s, %+.2f%%)"
              % (ac_slope, ac_slope * span, span, 100 * ac_slope * span / rpm.mean()))

    # The implied blade count is the cheapest possible check that the search band
    # is centred on the fundamental and not a harmonic.
    if expect:
        implied = f.mean() / (expect / 60.0)
        print("\n  implied blade count against the expected %.0f RPM: %.2f" % (expect, implied))
        if abs(implied - args.blades) > 0.25:
            print("  [WARN] that is not close to --blades %d. Either the blade count is"
                  % args.blades)
            print("         wrong or the peak found is a harmonic.")

    if csv_mean is not None:
        print("\nAgainst the stand (%s)" % os.path.basename(args.compare))
        print("  telemetry mean %8.0f" % csv_mean)
        print("  acoustic mean  %8.0f  (%+.2f%%)"
              % (rpm.mean(), 100 * (rpm.mean() / csv_mean - 1)))
        tel_slope = slope_per_s([r[0] for r in csv_series], [r[1] for r in csv_series]) \
            if csv_series else None
        if tel_slope is not None and ac_slope is not None:
            print("  telemetry drift %+.1f RPM/s" % tel_slope)
            print("  acoustic drift  %+.1f RPM/s" % ac_slope)
            print("\n  Comparing the two slopes needs no clock alignment between the")
            print("  recording and the CSV, only that both cover the same hold.")
            print("  Both sloping together says the rotor is really slowing.")
            print("  Only the telemetry sloping says the defect is in the ESC.")
        elif ac_slope is not None:
            print("  the CSV has no time column, so only the means compare here;")
            print("  use a per-sample file to put the two drift rates side by side")

    if args.output:
        with open(args.output, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["time_s", "freq_hz", "rpm", "snr"])
            for i in range(len(t)):
                w.writerow(["%.4f" % t[i], "%.2f" % f[i], "%.1f" % rpm[i], "%.1f" % snr[i]])
        print("\nWrote %d frames to %s" % (len(t), args.output))

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[WARN] --plot needs matplotlib; skipping")
        else:
            fig, ax = plt.subplots(figsize=(9, 4.5))
            ax.plot(t, rpm, lw=1.2, label="acoustic")
            if csv_series:
                ax.plot([r[0] for r in csv_series], [r[1] for r in csv_series],
                        lw=1.2, label="ESC telemetry")
            elif csv_mean:
                ax.axhline(csv_mean, ls="--", lw=1.0, label="telemetry mean")
            ax.set_xlabel("time (s)")
            ax.set_ylabel("RPM")
            ax.set_title("Rotor speed from the blade tone, %d blades" % args.blades)
            ax.legend()
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(args.plot, dpi=130)
            print("Wrote %s" % args.plot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
