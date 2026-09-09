#!/usr/bin/env python3
# Thrust stand -- host link: commands sequences, records CSV
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

"""Host link for the thrust stand (telemetry schema 1).

Commands test sequences over serial, harvests the samples, and writes CSV.

Two properties matter more than anything else here:

  * Statistics are computed over *unique* sensor samples. The firmware prints at
    250 Hz but the HX711 runs at ~80 Hz and the INA at ~100 Hz, so a naive mean
    over printed rows counts each physical reading 2-3 times and reports a
    standard deviation roughly sqrt(3) too small. Every row carries per-channel
    sequence counters, and this deduplicates on those.

  * The firmware's boot banner (chip identity, shunt value, pole count, firmware
    version) is captured and written into every CSV. A dataset that does not
    record what produced it cannot be merged with anyone else's.

  * Load cell calibration lives HERE, not in firmware. The board reports tared
    raw counts; this script converts them to grams. Recalibrating is therefore
    never a reflash, which matters because the factor is a property of the
    mounting and moves between sessions. The factor in force is stored in
    stand.json and recorded in every CSV, and because raw counts are kept in
    the output, any run can be re-derived later with --rescale.

Run `--check` first on a new build; it verifies the whole chain without spinning
the motor. See docs/HARDWARE.md and docs/CALIBRATION.md.
"""

import argparse
import collections
import csv
import datetime
import json
import os
import random
import statistics
import sys
import threading
import time
import tempfile

import serial

SCHEMA_VERSION = 1
# Firmware's in-sequence print period on the HX711 build; must match
# PRINT_FAST_US in firmware.ino. That constant is per fitted part -- 1500 us on
# the NAU7802 build, 4000 us on the HX711 -- so the host cannot hold one value:
# use rows_per_second(), which reads the part off the #SCALE banner, wherever a
# row rate matters. Rows per second is the real ceiling on samples per step,
# which is what makes a telemetry loss percentage meaningful or irrelevant.
PRINT_FAST_US = 4000


def rows_per_second(link):
    """Printed rows/s for the fitted part, mirroring firmware's PRINT_FAST_US.

    The firmware compiles a different in-sequence print period per load-cell
    part (#if LOADCELL_NAU7802 in firmware.ino: 1500 us ~= 667 Hz nominal,
    4000 us = 250 Hz), and its boot banner announces the part in #SCALE. A host
    check that compares against the wrong row rate -- e.g. link-test headroom --
    is off by the 667/250 ratio on the NAU7802 build.
    """
    scale = (link.meta.get("scale") or [""])[-1]
    part = scale.split(",", 1)[0]
    if part == "nau7802":
        return 1e6 / 1500.0
    return 1e6 / PRINT_FAST_US   # hx711, or a pre-#SCALE session

# Per-sample flag bits, matching FLAG_* in firmware.ino. The firmware computes
# them per printed row; the host decides what they mean at aggregate level.
FLAG_INA_SHUNT_SAT = 1 << 0
FLAG_INA_I_SAT = 1 << 1
FLAG_THRUST_STALE = 1 << 2
FLAG_RPM_STALE = 1 << 3
FLAG_WATCHDOG = 1 << 4
FLAG_ESC_ALERT = 1 << 5
FLAG_EDT_STALE = 1 << 6

# How long to keep reading after the firmware reports an abort. endSequence()
# needs only the time to read one ambient snapshot; the bound is what ends a
# run whose error came with no sequence teardown behind it.
ABORT_DRAIN_S = 5.0

# Banner keys describing the configuration in force: exactly one value is true at
# a time, so a reprint replaces rather than accumulates. "ambient" and "stats"
# are deliberately excluded -- multiple readings are the point of those.
SINGLETON_META = frozenset(("schema", "fw", "board", "esc", "scale", "ina",
                            "flags", "phase", "cols"))
# Override with -p. Typical values: /dev/ttyACM0 on Linux, COM3 on Windows,
# /dev/cu.usbmodem* on macOS.
SERIAL_PORT = '/dev/ttyACM0'
# The RP2040's USB CDC ignores this, but pyserial requires a value.
BAUD_RATE = 115200

STAND_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stand.json")


def load_stand_config():
    """Per-stand settings: calibration factor and pole count.

    These describe the physical bench rather than the code, so they live in a
    gitignored file instead of a constant. MASSCHECK rewrites the factor here,
    which is what makes recalibration a one-step operation.
    """
    try:
        with open(STAND_CONFIG) as fh:
            return json.load(fh)
    except OSError:
        # No file yet: a normal first run. The first MASSCHECK creates it.
        return {}
    except ValueError as exc:
        # The file exists but is not valid JSON. Returning {} silently would
        # drop calibration and pole count and let a session run uncalibrated --
        # thrust reads 0, every ratio is wrong, and the run is wasted.
        print("[WARN] %s is not valid JSON (%s); ignoring it. Fix or remove the"
              % (STAND_CONFIG, exc), file=sys.stderr)
        print("       file before calibrating -- a MASSCHECK will overwrite it.",
              file=sys.stderr)
        return {}


def save_stand_config(updates):
    cfg = load_stand_config()
    cfg.update(updates)
    with open(STAND_CONFIG, "w") as fh:
        json.dump(cfg, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return cfg


# Setup facts the stand cannot discover for itself, in header order. Each entry
# is (stand.json key, CSV header field, argparse dest, banner label). A None
# stand.json key means "CLI-only, no persisted fallback" -- see specimen below.
#
# Everything else in the header is measured or reported by the hardware. The
# first five are declarations, and they exist because the alternative is
# worse: on 2026-08-13 the ESC firmware, the PWM frequency and the bench
# supply all changed within a few hours, none of them appeared in any file,
# and which run had what survives only in notes and filenames.
#
# prop_hand is the odd one out among those five: a bench-level fact, not a
# per-run one. It answers "which ESC spin direction is this prop batch's own
# design-forward direction" -- one fact for the whole prop set in service, not
# something that changes between runs, so it belongs here in stand.json rather
# than on a per-run flag. --reverse (a per-run flag; see its help text) is
# combined with this to compute the actual SPIN command. Sticky state that is
# not echoed every run is exactly how a reversed ESC direction survived a
# reboot unnoticed, so this is echoed and warned-about
# exactly like the other four -- and unlike the other four, an unset value
# refuses to start a real run (see main()): a wrong one mis-declares an entire
# session, and both the banner warning and the site's sign check only catch it
# after the runs already exist.
#
# specimen is the second odd one out, in the opposite direction. It answers
# "which physical prop was mounted" -- a model is not a specimen; two samples
# of HQProp Ultralight 1.2x1x3 need different ids to be told apart, and this
# is the only field that gives them one. It has no stand.json key (cfg_key is
# None) on purpose: the other five are stable across a session, so a sticky
# value is a feature; a specimen changes at every prop swap, so a persisted
# value goes stale after the first one and silently attaches the wrong id to
# every run after it -- a wrong id is indistinguishable from a right one,
# which is worse than an absent one that can still be filled in later (the
# consuming site's fallback: `specimen:` in a run record). It is also never
# mandatory, in any mode, unlike prop_hand: omitting it is a normal run, not
# an error, because the whole point is being able to bench first and work out
# which sample it was afterwards.
SETUP_FIELDS = (
    ("esc_firmware",    "esc_fw",      "esc_firmware", "ESC fw"),
    ("esc_pwm_khz",     "esc_pwm_khz", "esc_pwm_khz",   "ESC PWM"),
    ("psu",             "psu",         "psu",           "supply"),
    ("psu_voltage_v",   "psu_v_set",   "psu_voltage",   "supply set"),
    ("esc_dir_forward", "prop_hand",   "prop_hand",     "prop hand"),
    (None,              "specimen",    "specimen",      "specimen"),
)


def setup_provenance(cfg, args):
    """Resolve the declared setup, CLI flag winning over stand.json.

    Returns [(header_field, value, source, label)] with unset fields included as
    a None value, so the caller can report what is *not* recorded rather than
    printing a tidy banner that quietly omits it. A field whose stand.json key
    is None (specimen) has no persisted fallback at all: the flag or nothing.
    """
    out = []
    for cfg_key, field, dest, label in SETUP_FIELDS:
        flag = getattr(args, dest, None)
        if flag is not None and str(flag) != "":
            out.append((field, flag, "--" + dest.replace("_", "-"), label))
        elif cfg_key is not None and cfg.get(cfg_key) not in (None, ""):
            out.append((field, cfg[cfg_key], "stand.json", label))
        else:
            out.append((field, None, None, label))
    return out


def format_setup_meta(entries):
    """The `# setup,` header line, or None when nothing is declared.

    The header parser splits on "," and then on "=", so a comma inside a value
    would silently split one field into two. Values are sanitised rather than
    rejected: losing a comma out of "Bluejay 0.21.0, 96 kHz" is cosmetic, and
    refusing to record a run over it is not.
    """
    parts = []
    for field, value, _source, _label in entries:
        if value is None:
            continue
        parts.append("%s=%s" % (field, str(value).replace(",", ";").strip()))
    return ",".join(parts) if parts else None


def print_setup_banner(entries):
    """Echo the declared setup before the motor spins.

    This is the whole defence against a stale declaration. None of these six
    can be read back from the hardware -- a patched Bluejay reports the same
    version string as a stock one, and nothing on the stand can see which
    physical prop is bolted on either -- so a wrong value here is recorded
    confidently and looks right forever afterwards, which is worse than a
    blank field. Putting it on screen every run is what makes that catchable.
    """
    for field, value, source, label in entries:
        if value is None:
            print("--> %-10s (not recorded)" % (label + ":"))
        else:
            print("--> %-10s %s  (%s)" % (label + ":", value, source))
    if any(f == "esc_fw" and v is not None for f, v, _s, _l in entries):
        print("    a patched build still reports version 0.21.0; check this")
        print("    matches the hex actually on the ESC")
    missing = [l for f, v, _s, l in entries if v is None]
    if missing:
        print("\n[WARN] Not recorded in this run: %s." % ", ".join(missing))
        print("       Set them in stand.json or pass the matching flag. Nothing")
        print("       downstream can recover them from the file afterwards.")


R_DRY_AIR = 287.058          # J/(kg K)
ISA_RHO = 1.225              # kg/m3 at 15 C, 1013.25 hPa


def air_density(temp_c, press_pa):
    """Dry-air density. Humidity is not measured yet: across any realistic
    indoor range it moves density by ~0.35%, against ~2% for a few degrees of
    temperature and ~3% for ordinary weather swings, and it is smaller than the
    BMP280's own temperature error contributes. A BME280 swap is a possible but
    minor upgrade (docs/HARDWARE.md)."""
    return press_pa / (R_DRY_AIR * (temp_c + 273.15))


def ideal_efficiency_g_per_w(thrust_g, diameter_mm, rho):
    """Actuator-disk ceiling: the best any propeller of this diameter could do.

    Measuring above this is physically impossible, so it catches wrong shunt
    values, wrong scale factors and clipped current -- failures that otherwise
    produce smooth, plausible-looking curves."""
    if thrust_g <= 0 or diameter_mm <= 0:
        return float("inf")
    thrust_n = thrust_g * 9.80665e-3
    area = 3.141592653589793 * (diameter_mm / 2000.0) ** 2
    p_ideal = thrust_n ** 1.5 / (2.0 * rho * area) ** 0.5
    return thrust_g / p_ideal if p_ideal > 0 else float("inf")

# Legacy static mode (host-timed steps). Kept for comparison with older runs;
# SWEEP supersedes it.
THROTTLE_STEPS = range(10, 101, 10)
SETTLE_TIME_SEC = 0.8
SAMPLES_PER_STEP = 5


class Link:
    """Serial link that keeps the firmware watchdog fed."""

    # Grams per raw count, set from stand.json (or --scale) before any run.
    # None means "not calibrated yet"; MASSCHECK still works, since it derives
    # the factor from raw counts and known masses.
    scale = None

    def grams(self, row):
        """Tared raw counts -> grams. The firmware no longer does this."""
        if not self.scale:
            return 0.0
        return row["thrust_raw"] / self.scale

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=2)
        self.lock = threading.Lock()
        self.meta = {}
        self.cols = None
        self.flag_names = {}
        self.ambient = {}
        self._stop = threading.Event()
        self.journal = None
        self.journal_path = None
        self.acquiring = False
        self.partial_rows = []
        self.partial_snapshot = None
        self.pending_abort = None
        self.abort_deadline = 0.0

    def begin_capture(self, output):
        """Keep received wire data even if acquisition or summarizing fails."""
        parent = os.path.dirname(os.path.abspath(output))
        os.makedirs(parent, exist_ok=True)
        self.journal = tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=parent,
            prefix=os.path.basename(output) + ".raw-", suffix=".log",
            delete=False, buffering=1)
        self.journal_path = self.journal.name
        for key, values in self.meta.items():
            for value in values:
                self.journal.write("#%s,%s\n" % (key.upper(), value))
        self.journal.write("#CALIBRATION,hx711_scale=%s\n" % self.scale)
        self.acquiring = True
        print("Raw capture: %s" % self.journal_path)

    def recovered_rows(self):
        if self.partial_snapshot is not None:
            return self.partial_snapshot()
        return self.partial_rows

    # -- io ---------------------------------------------------------------
    def write_line(self, text):
        with self.lock:
            self.ser.write((text + "\n").encode("utf-8"))

    def readline(self):
        line = self.ser.readline().decode("utf-8", "replace").strip()
        if self.journal is not None and line:
            self.journal.write(line + "\n")
        if self.acquiring and line == "SYSTEM_READY":
            raise RuntimeError("firmware restarted during acquisition")
        if self.pending_abort is not None and (line.startswith("END_")
                                               or time.time() > self.abort_deadline):
            # The teardown has been read (or is not coming). Raise now, with the
            # stats and closing ambient already absorbed into the metadata.
            reason, self.pending_abort = self.pending_abort, None
            raise RuntimeError("firmware: " + reason)
        return line

    def start_keepalive(self, interval=0.25):
        """Arms the firmware watchdog and keeps it fed.

        Once armed the motor stops if the firmware hears nothing for 1 s, so if
        this process dies the prop does not keep spinning.
        """
        def run():
            while not self._stop.wait(interval):
                try:
                    self.write_line("PING")
                except Exception:
                    return
        threading.Thread(target=run, daemon=True).start()

    def close(self):
        self._stop.set()
        try:
            if self.ser.is_open:
                self.ser.close()
        finally:
            if self.journal is not None:
                self.journal.close()

    # -- banner -----------------------------------------------------------
    def absorb_meta(self, line):
        body = line[1:]
        if not body:
            return
        key, _, rest = body.partition(",")
        key = key.lower()
        if key == "cols":
            self.cols = rest.split(",")
        elif key == "ambient":
            parts = rest.split(",")
            entry = {"tag": parts[0]}
            for item in parts[1:]:
                k, _, v = item.partition("=")
                try:
                    entry[k] = float(v)
                except ValueError:
                    entry[k] = v
            self.ambient[entry["tag"]] = entry
        elif key == "flags":
            for item in rest.split(","):
                bit, _, name = item.partition("=")
                try:
                    self.flag_names[int(bit)] = name
                except ValueError:
                    pass
        if self.acquiring and (key == "error" or
                               (key == "warn" and rest.startswith("sequence_aborted,"))):
            self.meta.setdefault(key, []).append(rest)
            # Latch, do not raise. endSequence() prints the abort reason first
            # and the stats, closing ambient and END_ sentinel after it, so
            # raising here would drop the telemetry health record on exactly
            # the runs that need explaining. readline() raises once it has read
            # the sentinel, or once ABORT_DRAIN_S proves none is coming --
            # `#ERROR,busy` and the like arrive with no teardown behind them.
            #
            # Note what raising costs: main() unwinds to an immediate STOP, not
            # the measured spin-down ramp. That is right for the aborts we get
            # today, which all reach here with the throttle already at zero.
            # A firmware `#ERROR` printed while the motor is spinning would not
            # be, so anything that adds one has to send the ramp itself first.
            if self.pending_abort is None:
                self.pending_abort = body
                self.abort_deadline = time.time() + ABORT_DRAIN_S
            return
        if key in SINGLETON_META:
            # `ID` reprints the banner, and POLES makes that reprint differ from
            # boot. Appending would leave two "# esc" lines in the CSV disagreeing
            # about the pole count, with the stale one first.
            self.meta[key] = [rest]
        else:
            self.meta.setdefault(key, []).append(rest)

    def handshake(self, timeout=5.0):
        """Request the banner and collect it. Works whether or not we caught boot."""
        self.write_line("ID")
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.readline()
            if not line:
                continue
            if line.startswith("#"):
                self.absorb_meta(line)
                if self.cols:
                    # Banner ends with COLS; drain to SYSTEM_READY if present.
                    break
            elif line == "SYSTEM_READY":
                self.write_line("ID")
        if not self.cols:
            raise RuntimeError(
                "no #COLS banner from firmware -- is it running schema %d?" % SCHEMA_VERSION)
        self.ser.reset_input_buffer()

    def conditions(self, manual_temp=None, manual_press_hpa=None):
        """Ambient for this run: sensor mean of the start/end snapshots, else
        manual entry. Returns (temp_c, press_pa, source) or None."""
        if manual_temp is not None and manual_press_hpa is not None:
            return manual_temp, manual_press_hpa * 100.0, "manual"
        usable = [e for e in self.ambient.values()
                  if e.get("present") == 1.0 and "temp_c" in e and "press_pa" in e]
        # Prefer the snapshots bracketing this run. The banner reading is taken
        # at connect time, possibly minutes earlier with the sensor still cold,
        # so it is only a fallback.
        readings = [e for e in usable if e["tag"] in ("start", "end")] or usable
        if not readings:
            return None
        temp = sum(e["temp_c"] for e in readings) / len(readings)
        press = sum(e["press_pa"] for e in readings) / len(readings)
        return temp, press, "bmp280"

    def describe_flags(self, mask):
        if not mask:
            return ""
        return "|".join(name for bit, name in sorted(self.flag_names.items()) if mask & bit)

    # -- data -------------------------------------------------------------
    def parse_row(self, line):
        parts = line.split(",")
        if parts[0] != "D" or len(parts) != len(self.cols) + 1:
            return None
        try:
            return {name: int(value) for name, value in zip(self.cols, parts[1:])}
        except ValueError:
            return None


# A step logs for 100 ms; allow a little over one full step period before an EDT
# reading stops being attributable to the step it is printed in.
EDT_FRESH_US = 250000

# Firmware phase code for the ramped stop at the end of a sequence. Kept with
# the accumulator rather than the sweep's local PHASE_UP/PHASE_DOWN because
# every run type has to exclude it, not just the sweep.
PHASE_SPINDOWN = 8


def edt_requested(link):
    """Did this session ask the ESC for extended telemetry?

    Empty esc_* columns mean two opposite things and only the banner separates
    them: unasked is a configuration choice and says nothing about the ESC,
    while asked-and-empty is a finding about the ESC or the signal line.

    Firmware predating the switch always asked and reports no `edt=` field, so
    absence reads as True and those runs are described the way they were.
    """
    banner = (link.meta.get("esc") or [""])[-1]
    for part in banner.split(","):
        if part.startswith("edt="):
            return part.split("=", 1)[1].strip() == "1"
    return True


def require_bus_power(link, min_volts=1.0, samples=6, timeout=4.0):
    """Abort unless the motor supply is actually live.

    The Pico is powered over USB, so every sensor answers and the banner looks
    perfect with the ESC rail dead. A spin test then runs to completion and
    reports zeros, which is indistinguishable at a glance from a run where the
    motor simply did not respond. Checking first turns a confusing dataset into
    one clear message.
    """
    seen = []
    deadline = time.time() + timeout
    while len(seen) < samples and time.time() < deadline:
        line = link.readline()
        if not line or line.startswith("#"):
            continue
        row = link.parse_row(line)
        if row is not None:
            seen.append(row["bus_mv"] / 1000.0)
    if not seen:
        raise RuntimeError("no data rows from the firmware -- is it running?")
    volts = max(seen)
    if volts < min_volts:
        raise RuntimeError(
            "motor supply reads %.3f V (need >%.1f V). The board is powered over "
            "USB but the ESC rail is not -- switch the bench supply on, or check "
            "it has not tripped, then re-run." % (volts, min_volts))
    return volts


# The eRPM telemetry word is a period, not a speed: a 9-bit mantissa shifted by
# a 3-bit exponent, in microseconds per electrical revolution. Every RPM figure
# the stand reports is 60e6/(period x poles/2), so resolution is won and lost in
# the period, and it degrades as RPM^2 -- one microsecond is ~4 RPM at 6,000 and
# ~220 at 47,000. Reading the word back is the only way to tell "the ESC sent a
# different number" from "the ESC sent the same number again", which a rounded
# RPM hides completely.
ERPM_STOPPED_CODE = 0xFFF


def period_us_from_code(code):
    """Microseconds per electrical revolution encoded in a raw eRPM word."""
    code = int(code)
    if code == ERPM_STOPPED_CODE:
        return 0
    return (code & 0x1FF) << (code >> 9)


# A transfer the I2C bus mangled lands the raw thrust count hundreds of grams
# out: every corrupt value measured on this bench read +312 to +314 g. Real
# thrust cannot go near that -- peak thrust on this stand is 29 g and the cell
# is rated 100 g -- so a deviation of 50 g from a step's own median is
# physically impossible and unambiguously a bad transfer.
#
# **This was 100000 counts (5.6 g) and that was too tight.** It was set against
# a median step sd of 0.5 g and described as "far above the noise". It is not:
# the mount resonance drives step sd to 3.0 g at the worst steps, putting the
# cut at 1.85 sigma, and on 2026-09-02 it clipped 8 of 32 real samples from one
# sweep step and moved that step's mean 2 g below both its neighbours. The
# probe counted zero bad transfers over the same run, so the bus was clean and
# the rejections were false.
#
# 900000 counts is ~50 g at this cell's factor: ~16 sigma above the worst
# observed noise and 6x below the smallest corruption ever seen. Deliberately
# NOT the same as tools/bus_check.py's OUTLIER_COUNTS, which stays at 100000
# because it runs with the motor stopped, where the noise floor is 0.1 g and a
# tighter cut is the right detector.
MANGLED_COUNTS = 900000


# The same fault on the same bus reaches the INA226, and the cuts are set the
# same way: far above the measured clean spread, far below the smallest observed
# corruption. Across 376 sweep steps on two ADCs the largest clean within-step
# volts_std was 0.0015 V; the three corrupted steps measured on 2026-09-02 ran
# 0.31 to 4.01 V, implying single-sample excursions of 1.0 V and up. 100 mV sits
# 67x above the clean spread and 10x below the smallest corruption seen.
MANGLED_MV = 100
# No corrupted current read has been observed yet, so this one is precautionary
# rather than measured. Clean amps_std peaked at 0.022 A over the same 376
# steps, so 0.5 A cannot reach a real reading; the register spans +-8 A, so a
# corrupt one usually will.
MANGLED_UA = 500000


def reject_mangled_ina(pairs):
    """Drop INA readings the bus corrupted in transfer.

    Takes (bus_mv, bus_ua) pairs, returns (kept, n_rejected).

    A pair is rejected when either channel is grossly off its own median, and
    the whole pair goes rather than the offending channel. Both values come
    from one conversion over one bus, so a transfer bad enough to corrupt the
    voltage leaves no reason to trust the current that came with it.

    This matters more than it looks. A corrupt thrust sample lands hundreds of
    grams out and is unmissable in a plot; a corrupt bus voltage lands a volt or
    two out on a 4 V rail, which reads as a plausible number and silently
    inflates every watt and g/W figure derived from that step. Measured on
    2026-09-02: three steps of 189 carried one bad read each, and they moved
    watts by up to +34%.
    """
    if len(pairs) < 3:
        return list(pairs), 0
    med_mv = statistics.median(mv for mv, _ in pairs)
    med_ua = statistics.median(ua for _, ua in pairs)
    kept = [(mv, ua) for mv, ua in pairs
            if abs(mv - med_mv) <= MANGLED_MV and abs(ua - med_ua) <= MANGLED_UA]
    return kept, len(pairs) - len(kept)


def reject_mangled(raw):
    """Drop raw thrust counts the bus corrupted in transfer.

    Returns (kept, n_rejected).

    This is not smoothing, and the distinction is the whole reason it is
    allowed to exist. The bus corrupts a whole transfer, so a mangled sample is
    one that never happened, in the same sense as a frame that fails its
    checksum. Nothing here touches a sample the ADC actually converted, and
    nothing is interpolated to fill the hole.

    It has to run BEFORE the mean is formed, because it cannot be undone after.
    A sweep CSV keeps per-step aggregates only, so one corrupt sample among 24
    is baked into thrust_g_mean with no way back -- measured on this bench,
    where it moved a step from 26.3 g to 38.5 g.

    The count is returned rather than swallowed so that n_thrust_mangled can
    carry it into every file. A rejection nobody can see is exactly the silent
    repair this project exists to prevent.
    """
    if len(raw) < 3:
        # Median is meaningless on one or two points, and a step this short
        # cannot distinguish the outlier from the signal anyway.
        return list(raw), 0
    med = statistics.median(raw)
    kept = [v for v in raw if abs(v - med) <= MANGLED_COUNTS]
    return kept, len(raw) - len(kept)


class Accumulator:
    """Collects rows for one throttle step, keeping only unique sensor samples."""

    def __init__(self):
        self.thrust = {}   # n_thrust -> milligrams
        self.ina = {}      # n_ina    -> (mV, uA)
        self.rpm = {}      # n_rpm    -> rpm
        self.rpm_raw = {}  # n_rpm    -> raw 12-bit eRPM word
        self.edt = {}          # n_edt    -> (temp_c, stress)
        # Age of the newest EDT frame on the LAST row of the step. Freshness is
        # judged from the step's end, not from its start: one frame that lands
        # early in a long hold must not certify esc_* as fresh for the rest of
        # the step after the ESC has gone silent.
        self.edt_age_last = None
        self.flags = 0
        self.rows = 0
        # A stale row is a re-print of a sample already counted fresh, so it
        # contributes nothing to the means above. It is still worth knowing how
        # many there were -- the dedup is what makes the flag harmless -- but
        # as a count, not as a flag on the whole step.
        self.n_rpm_stale = 0
        self.n_thrust_stale = 0

    def add(self, row):
        # Rows from the ramped stop at the end of a sequence. They are real
        # measurements and stay in the CSV -- the stop is where the bus sees its
        # worst excursion, and it used to be logged nowhere -- but they belong
        # to the stop, not to the setpoint that preceded it. A 400 ms ramp
        # against a 100 ms logged window would otherwise dominate every mean.
        if row.get("phase") == PHASE_SPINDOWN:
            return
        self.rows += 1
        # Only flags that invalidate an *aggregate* survive to the summary.
        # rpm_stale / thrust_stale describe a single printed row (its value was
        # a repeat of a fresh sample), and the dedup above already removed that
        # row from the means -- ORing it over 25 rows would condemn the whole
        # step for one gap. Count those instead; keep the rest as flags.
        self.flags |= row["flags"] & AGGREGATE_FATAL_MASK
        if row["flags"] & FLAG_RPM_STALE:
            self.n_rpm_stale += 1
        if row["flags"] & FLAG_THRUST_STALE:
            self.n_thrust_stale += 1
        self.thrust[row["n_thrust"]] = row["thrust_raw"]
        self.ina[row["n_ina"]] = (row["bus_mv"], row["bus_ua"])
        self.rpm[row["n_rpm"]] = row["rpm"]
        # The raw word travels with the rpm for the same reason thrust_raw
        # travels with the grams. Absent on firmware predating the column.
        if "erpm_raw" in row:
            self.rpm_raw[row["n_rpm"]] = row["erpm_raw"]
        # EDT gets the same treatment as every other channel: frames arrive a
        # few times a second against 250 printed rows/s, so a max taken over
        # rows reports one sample once per step for the length of the run.
        if "n_edt" in row:
            self.edt[row["n_edt"]] = (row.get("esc_temp_c", 0), row.get("esc_stress", 0))
            # Rows arrive in print order and edt_age_us grows between frames
            # (it is the age of the newest frame at print time), so the last
            # row's age is the newest frame's distance from the step's end.
            age = row.get("edt_age_us")
            if age is not None:
                self.edt_age_last = age

    @staticmethod
    def _stats(values):
        if not values:
            return 0.0, 0.0, 0
        if len(values) == 1:
            return float(values[0]), 0.0, 1
        return statistics.mean(values), statistics.stdev(values), len(values)

    @staticmethod
    def _missed(keys):
        """Conversions that happened but never reached a printed row.

        thrust and INA are latest-value registers: emitSample() prints whichever
        sample is current, and the host deduplicates on the sequence counter. That
        assumes rows outpace conversions, which holds at 250 rows/s against the
        HX711's 80 SPS and the INA's 100 SPS. It stops holding the moment a faster
        ADC is fitted -- a NAU7802 at 320 SPS outruns the row rate, and the samples
        it laps are simply never seen. The dict keys go 4,5,7,8,10 and nothing
        anywhere says a 6 and a 9 existed.

        A gap inside [min,max] is exactly that loss. Silence is the danger here,
        not the loss itself: a subsample that is systematic rather than random can
        alias periodic content -- prop-order vibration above all -- into a mean
        that still looks perfectly well behaved.

        Not applied to rpm. Core 1 decodes telemetry thousands of times a second
        by design, so gaps there are the intended behaviour rather than a defect.
        """
        if not keys:
            return 0
        return (max(keys) - min(keys) + 1) - len(keys)

    def summarize(self, link, mode, throttle, dshot, direction="up"):
        # Rejected before any mean is formed. self.thrust keeps its keys, so
        # _missed() below still sees the true conversion sequence -- a mangled
        # sample was printed, it just cannot be believed, which is a different
        # thing from one that never reached a row.
        raw, n_mangled = reject_mangled(list(self.thrust.values()))
        thrust_g = [v / link.scale for v in raw] if link.scale else [0.0] * len(raw)
        ina, n_ina_mangled = reject_mangled_ina(list(self.ina.values()))
        volts = [mv / 1000.0 for mv, _ in ina]
        amps = [ua / 1e6 for _, ua in ina]
        rpms = list(self.rpm.values())

        rpm_m, rpm_s, n_rpm = self._stats(rpms)
        thr_m, thr_s, n_thr = self._stats(thrust_g)
        v_m, v_s, n_ina = self._stats(volts)
        a_m, a_s, _ = self._stats(amps)

        # Ratio of means, not mean of ratios: efficiency is a derived quantity
        # and averaging per-sample ratios biases it when power is small.
        watts = v_m * a_m
        eff = (thr_m / watts) if watts > 0 else 0.0

        # Report esc_* only when a frame actually landed near this step. Held-over
        # values look identical to fresh ones in the row, and once an ESC stops
        # sending EDT they stay frozen for the rest of the run -- every step then
        # claims the same stress reading as if it had been measured.
        # Freshness uses the newest frame's age at the step's end (edt_age_last),
        # so a channel that dies part-way through a hold is not certified fresh
        # for the rows that follow the death.
        # What the ESC actually sent this step, before it became an RPM. A mean
        # over 25 samples can sit anywhere between two codes, so a step whose
        # samples all landed on one code has a mean that could not have moved --
        # which is the difference between a plateau that is a measurement and a
        # plateau that is the telemetry running out of resolution.
        codes = collections.Counter(self.rpm_raw.values())
        mode_code = codes.most_common(1)[0][0] if codes else None
        period = period_us_from_code(mode_code) if mode_code is not None else 0
        # RPM per microsecond of period: the finest change the telemetry can
        # express here. Grows as RPM^2, so it must be read alongside any step
        # near the top of a sweep before the step means anything.
        rpm_per_us = round(rpm_m / period, 1) if period else 0.0

        fresh = self.edt_age_last is not None and self.edt_age_last < EDT_FRESH_US
        temp_max = max((t for t, _ in self.edt.values()), default=None) if fresh else None
        stress_max = max((s for _, s in self.edt.values()), default=None) if fresh else None

        return {
            "mode": mode,
            "direction": direction,
            "throttle_pct": throttle,
            "dshot": dshot,
            "rpm_mean": round(rpm_m), "rpm_std": round(rpm_s, 1), "n_rpm": n_rpm,
            "erpm_period_us": period, "rpm_per_us": rpm_per_us, "n_codes": len(codes),
            "erpm_raw_hist": "|".join("%d:%d" % (c, n) for c, n in sorted(codes.items())),
            "thrust_g_mean": round(thr_m, 3), "thrust_g_std": round(thr_s, 3), "n_thrust": n_thr,
            # Raw counts travel with the grams so a run stays re-derivable if the
            # factor turns out to be wrong. Without this a sweep taken under a
            # bad calibration is unrecoverable, which is the failure this whole
            # project is built to avoid.
            "thrust_raw_mean": round(statistics.mean(raw), 1) if raw else 0.0,
            "volts_mean": round(v_m, 4), "volts_std": round(v_s, 4),
            "amps_mean": round(a_m, 5), "amps_std": round(a_s, 5), "n_ina": n_ina,
            "watts_mean": round(watts, 4),
            "eff_g_per_w": round(eff, 3),
            "esc_temp_max_c": temp_max,
            "esc_stress_max": stress_max,
            "n_edt": len(self.edt),
            "rows_printed": self.rows,
            "n_rpm_stale": self.n_rpm_stale,
            "n_thrust_stale": self.n_thrust_stale,
            # The complement of the stale counts: stale means a sample was printed
            # twice, missed means one was never printed at all. Both are zero on a
            # healthy 250 rows/s against 80 SPS; only the second can appear once
            # the ADC outruns the row rate.
            "n_thrust_missed": self._missed(self.thrust.keys()),
            # Samples thrown out as corrupt transfers. Distinct from both the
            # stale and missed counts: stale was printed twice, missed was
            # never printed, mangled was printed and was not a reading.
            "n_thrust_mangled": n_mangled,
            # Same fault, quieter symptom. A rejected INA reading would
            # otherwise have moved watts and g/W by tens of percent while
            # looking like an ordinary voltage.
            "n_ina_mangled": n_ina_mangled,
            "n_ina_missed": self._missed(self.ina.keys()),
            "flags": self.flags,
            "flag_names": link.describe_flags(self.flags),
        }


STEADY_FIELDS = [
    "mode", "direction", "throttle_pct", "dshot",
    "rpm_mean", "rpm_std", "n_rpm",
    "erpm_period_us", "rpm_per_us", "n_codes",
    "thrust_g_mean", "thrust_g_std", "n_thrust", "thrust_raw_mean",
    "volts_mean", "volts_std", "amps_mean", "amps_std", "n_ina",
    "watts_mean", "eff_g_per_w", "thrust_g_iso", "fom",
    "esc_temp_max_c", "esc_stress_max", "n_edt",
    "rows_printed", "n_rpm_stale", "n_thrust_stale", "n_thrust_missed", "n_thrust_mangled", "n_ina_missed", "n_ina_mangled",
    "erpm_raw_hist", "flags", "flag_names",
]

LINK_FIELDS = [
    "mode", "throttle_pct", "rpm_mean", "rpm_std", "n_rpm",
    # A LINKTEST hold is thousands of samples at one throttle, which is the run
    # that says whether a suspect step mean is genuinely wrong or merely
    # under-sampled -- a sweep step keeps only ~25. Worth nothing without the
    # code distribution behind it, so it carries the same period columns as a
    # steady-state step.
    "erpm_period_us", "rpm_per_us", "n_codes",
    # Voltage belongs with a hold, not just current: the pack sags under a
    # sustained high-throttle hold, and without it a lower RPM at the same
    # throttle later in a run is indistinguishable from a telemetry fault.
    "volts_mean", "amps_mean", "thrust_g_mean",
    "telem_ok", "telem_corrupt", "telem_silent", "silent_pct", "corrupt_pct",
    "edt_frames", "n_rpm_stale", "n_thrust_stale", "n_thrust_missed", "n_thrust_mangled", "n_ina_missed", "n_ina_mangled",
    "erpm_raw_hist", "flags", "flag_names",
]

WALK_FIELDS = [
    # dshot_set is the commanded value and dshot what the firmware echoed back;
    # they should be equal, and a disagreement means the setpoint did not take.
    # visit and rep record *when* each hold happened, which is what makes the
    # time drift separable from the throttle response after the fact.
    "mode", "dshot_set", "dshot", "throttle_pct", "visit", "rep",
    "rpm_mean", "rpm_std", "n_rpm",
    "erpm_period_us", "rpm_per_us", "n_codes",
    "volts_mean", "amps_mean", "n_ina", "thrust_g_mean", "thrust_raw_mean",
    "n_rpm_stale", "n_thrust_stale", "n_thrust_missed", "n_thrust_mangled", "n_ina_missed", "n_ina_mangled",
    "erpm_raw_hist", "flags", "flag_names",
]

KV_FIELDS = [
    # dshot is the second axis. With one setpoint the fit is ill-conditioned --
    # at a fixed prop and full throttle, current is a function of voltage, so
    # V and I sweep together and Kv cannot be separated from R. Recording which
    # setpoint each point was taken at is what lets the fit use both axes.
    "mode", "v_set", "dshot", "duty_fitted", "rpm_mean", "rpm_std", "n_rpm",
    # The Kv fit is solved on RPM, so it inherits whatever the telemetry
    # resolution was at each hold point. Carry that with the fit's inputs.
    "erpm_period_us", "rpm_per_us", "n_codes",
    "volts_mean", "amps_mean", "n_ina", "thrust_g_mean", "thrust_raw_mean",
    # Blank unless --psu-port drove the sweep. The supply's own meter, at its
    # terminals rather than at the stand, so v_psu - volts_mean is the harness
    # drop and a_psu is an independent check on INA_SHUNT_UOHM.
    "v_psu", "a_psu",
    # The accumulator has always computed these; the field list dropped them,
    # so a Kv run recorded no temperature at all. That cost a session: nine
    # start attempts decayed from three-of-three to zero-of-three and the
    # thermal reading could not be checked. n_edt comes with the temperature
    # because a held value and a live one look identical without it.
    "esc_temp_max_c", "esc_stress_max", "n_edt",
    "n_rpm_stale", "n_thrust_stale", "n_thrust_missed", "n_thrust_mangled", "n_ina_missed", "n_ina_mangled",
    "erpm_raw_hist", "flags", "flag_names",
]

TRANSIENT_FIELDS = [
    # phase is the firmware's own label for what the sequence was doing when
    # the row was printed, decoded by the #PHASE dictionary in the banner. The
    # rows always carried it and this list dropped it, which left every
    # consumer inferring the step sequence from throttle values alone. That
    # inference is wrong in a specific way: a RESPONSE file does not end at the
    # last step, it walks the throttle down a staircase to zero, and the final
    # tail segment is long enough to pass a sample-count filter and be averaged
    # in as a fall edge. Writing the label out lets a reader select the phase
    # instead of guessing at it. Added column, so still schema 1.
    "mode", "time_s", "phase", "throttle_pct", "dshot",
    "rpm", "erpm", "erpm_raw", "new_rpm",
    "thrust_g", "thrust_raw", "new_thrust", "thrust_age_us", "thrust_t_s",
    "volts", "amps", "shunt_uv", "new_ina", "ina_age_us",
    "esc_temp_c", "esc_stress", "esc_dbg1", "esc_dbg2",
    "new_edt", "edt_age_us", "flags", "flag_names",
]


# =========================================================================
# Sequences
# =========================================================================

# How long a harvest loop will wait with the board saying nothing at all before
# it gives up. Not a budget for the sequence -- sequences differ by an order of
# magnitude in length and would each need their own number -- but a silence
# timeout, which is what actually distinguishes a board that is working from one
# that has stopped.
#
# The firmware prints at 250 Hz while logging and 10 Hz when idle, so silence is
# never normal for long. The two legitimately quiet stretches are both windows a
# sequence has chosen to discard: a sweep's idle hunt, up to ~7.3 s if it fails
# at every step, and a HOLD's settle, capped at 10 s by runHoldSequence. 20 s
# clears both with room to spare.
#
# This exists because the loops below used to be `while True` over a serial read
# that returns empty on timeout. When core 0 stalled mid-sweep with the throttle
# latched, the host sat in one of them forever -- so Ctrl-C was the only way out,
# and even that only reached a `finally` whose STOP the firmware was no longer
# reading.
HARVEST_SILENCE_S = 20.0


class HarvestTimeout(RuntimeError):
    """The board stopped talking mid-sequence."""


class SilenceTimer:
    """Bounds a harvest loop by time since the last line, not total elapsed.

    Call feed() for every line that arrives and check() every time one does
    not. Nothing here can stop a motor -- the caller's `finally` and the
    firmware's own watchdogs do that -- but it turns an indefinite hang into an
    error the operator can see, with the rows collected so far still in hand.
    """

    def __init__(self, what, limit=HARVEST_SILENCE_S):
        self.what = what
        self.limit = limit
        self.last = time.time()

    def feed(self):
        self.last = time.time()

    def check(self):
        quiet = time.time() - self.last
        if quiet > self.limit:
            raise HarvestTimeout(
                "no output from the board for %.1f s while waiting for %s -- "
                "the firmware may have stalled with the throttle latched; "
                "check the motor and power-cycle the board" % (quiet, self.what))


def run_sweep(link):
    print("Requesting firmware sweep...")
    link.write_line("SWEEP")

    groups = {}
    dshot_of = {}
    recording = False
    PHASE_UP, PHASE_DOWN = 2, 4
    def snapshot():
        return [groups[(p, t)].summarize(
            link, "sweep", t, dshot_of[(p, t)], "up" if p == PHASE_UP else "down")
            for p, t in sorted(groups) if p in (PHASE_UP, PHASE_DOWN)]
    link.partial_snapshot = snapshot
    quiet = SilenceTimer("END_SWEEP")

    while True:
        line = link.readline()
        if not line:
            quiet.check()
            continue
        quiet.feed()
        if line.startswith("#"):
            link.absorb_meta(line)
            print("   " + line)
            continue
        if line == "START_SWEEP":
            print("--> sweep running, harvesting samples...")
            recording = True
            continue
        if line.startswith("MIN_RELIABLE_IDLE"):
            print("--> minimum reliable idle: %s%% throttle" % line.split(",")[1])
            continue
        if line == "END_SWEEP":
            print("--> sweep complete.")
            break
        if not recording:
            continue
        row = link.parse_row(line)
        if row is None:
            continue
        key = (row["phase"], row["throttle_pct"])
        groups.setdefault(key, Accumulator()).add(row)
        dshot_of[key] = row["dshot"]

    rows = []
    for phase in (PHASE_UP, PHASE_DOWN):
        for (p, t) in sorted((k for k in groups if k[0] == phase), key=lambda k: k[1]):
            rows.append(groups[(p, t)].summarize(
                link, "sweep", t, dshot_of[(p, t)],
                "up" if phase == PHASE_UP else "down"))
    return rows


def _period_spread(pooled):
    """Distinct periods seen and the gaps between adjacent ones.

    The gaps are the diagnostic. If a run only ever reports periods spaced two
    or three microseconds apart where one is representable, the ESC is
    quantising its own period measurement before sending it, and the true
    resolution is coarser than the encoding suggests -- which changes what a
    plateau in an RPM curve is allowed to mean.
    """
    pooled = sorted(p for p in pooled if p)
    gaps = [b - a for a, b in zip(pooled, pooled[1:])]
    return pooled, gaps


def report_period_codes(steps, detail=False):
    """What the ESC sent, before it became an RPM.

    `steps` is a list of (label, {period_us: count}, rpm_mean). Separates the
    three things a flat patch in an RPM curve can be: the telemetry too coarse
    to express the change, the ESC repeating one value, or the speed genuinely
    not moving. The per-step detail is opt-in because a sweep has ~190 steps,
    but nothing is lost by leaving it off -- erpm_raw_hist in the CSV carries
    every count, and this is only a view of it.
    """
    steps = [s for s in steps if s[1]]
    if not steps:
        return

    print("\nTelemetry resolution, from the raw eRPM words")
    if detail:
        print("%-14s %-11s %-9s %s"
              % ("step", "period_us", "rpm/us", "periods seen (us x n)"))
        print("-" * 78)
        for label, counts, rpm_mean in steps:
            top = max(counts.items(), key=lambda kv: kv[1])[0]
            # Period 0 is the protocol's "motor stopped", not a measured period.
            seen = " ".join("%sx%d" % (p if p else "stop", n)
                            for p, n in sorted(counts.items()))
            print("%-14s %-11d %-9.0f %s"
                  % (label, top, (rpm_mean / top) if top else 0.0,
                     seen[:40] + ("..." if len(seen) > 40 else "")))

    pooled, gaps = _period_spread(set().union(*[set(c) for _, c, _ in steps]))
    if len(pooled) > 1:
        common = collections.Counter(gaps).most_common()
        print("  %d distinct periods over %d-%d us; smallest gap %d us"
              % (len(pooled), pooled[0], pooled[-1], min(gaps)))
        print("  gaps between adjacent periods: %s%s"
              % (", ".join("%d us x%d" % (g, n) for g, n in common[:5]),
                 " (+%d more)" % (len(common) - 5) if len(common) > 5 else ""))
    single = sum(1 for _, counts, _ in steps if len(counts) == 1)
    if single:
        print("  %d of %d steps landed on a single period -- their means could not "
              "have moved," % (single, len(steps)))
        print("  whatever the motor did. Read any plateau there against that first.")
    if not detail:
        print("  Per-step detail: rerun with --codes (always saved in erpm_raw_hist).")


def report_missed_samples(rows):
    """Warn when conversions never reached a row, and say what actually fixes it.

    Prints nothing on a healthy run, so it costs a reader nothing until it
    matters. Silence here is a real statement: every conversion reached a row.
    """
    # The advice is per channel because the two channels fail for different
    # reasons. The load cell can genuinely outrun the row rate; the INA226,
    # converting at ~100 Hz against hundreds of rows a second, cannot.
    thrust_fix = (
        "  Rows must outpace conversions with enough MARGIN to absorb loop\n"
        "  jitter, and the nominal rate is not the achieved one: the loop also\n"
        "  polls serial, services sensors and feeds the watchdog, which measured\n"
        "  ~357 us of overhead on 2026-09-02.\n"
        "  PRINT_FAST_US is 1500 us for the NAU7802, ~540 rows/s achieved, a\n"
        "  margin of ~1.7x over 320 SPS. Seeing this warning at 320 SPS means\n"
        "  something has eaten that margin -- check rows_dropped in #STATS and\n"
        "  whether anything new runs in the loop.\n"
        "  Otherwise: shorten PRINT_FAST_US again, or lower the rate with --sps.")
    ina_fix = (
        "  The INA226 converts at ~100 Hz, far below any normal row rate, so\n"
        "  this is not the part outrunning the rows. A handful of samples means\n"
        "  the row rate dipped instead -- a blocked loop, or rows dropped on a\n"
        "  full USB buffer, which #STATS reports separately as rows_dropped.")

    for label, key, seen_key, part, fix in (
            ("thrust", "n_thrust_missed", "n_thrust", "HX711/NAU7802", thrust_fix),
            ("INA", "n_ina_missed", "n_ina", "INA226", ina_fix)):
        missed = sum(r.get(key) or 0 for r in rows)
        if not missed:
            continue
        seen = sum(r.get(seen_key) or 0 for r in rows)
        total = seen + missed
        print("\n  WARNING: %d of %d %s conversions never reached a row (%.1f%%)."
              % (missed, total, label, 100.0 * missed / total if total else 0.0))
        # Deliberately does not name a cause here. The two channels miss
        # conversions for opposite reasons, and the per-channel text below is
        # where that gets said; asserting it twice once contradicted itself.
        print("  The %s CSV therefore holds a subsample. The loss is systematic"
              % part)
        print("  rather than random, so it can alias periodic content -- prop-order")
        print("  vibration above all -- into means that still look well behaved.")
        print("  Those samples are not recoverable.")
        print(fix)


def report_mangled_samples(rows):
    """Say how many samples were thrown out as corrupt transfers, and where.

    Prints nothing on a healthy run. When it does print, the run is still
    usable -- the corrupt samples never entered a mean -- but the bus is
    faulty, and that is a hardware finding rather than a data-quality note.
    Rejection keeps a file readable; it does not make the stand correct.
    """
    for label, key, seen_key, note in (
            ("thrust", "n_thrust_mangled", "n_thrust",
             "A corrupt thrust sample lands hundreds of grams out, so it is\n"
             "  unmistakable once you look. These never reached a mean."),
            ("INA", "n_ina_mangled", "n_ina",
             "A corrupt bus voltage lands a volt or two out on a 4 V rail, which\n"
             "  reads as a plausible number. Measured on this bench: three steps\n"
             "  of 189 carried one bad read each and moved watts by up to +34%.")):
        mangled = sum(r.get(key) or 0 for r in rows)
        if not mangled:
            continue
        kept = sum(r.get(seen_key) or 0 for r in rows)
        total = kept + mangled
        affected = sum(1 for r in rows if r.get(key))
        print("\n  %d of %d %s samples were rejected as corrupt transfers (%.2f%%),"
              % (mangled, total, label, 100.0 * mangled / total if total else 0.0))
        print("  spread over %d of %d steps. They were dropped before any mean was"
              % (affected, len(rows)))
        print("  formed, so the columns below are clean. The bus is not.")
        print("  %s" % note)

    if any(r.get("n_thrust_mangled") or r.get("n_ina_mangled") for r in rows):
        print("\n  Both channels are rejected against the step's own median, which")
        print("  needs several samples per step. TRANSIENT, RESPONSE and COASTDOWN")
        print("  print raw rows with no aggregation, so nothing filters them and a")
        print("  corrupt sample there is still in the file.")
        print("  Run tools/bus_check.py and fix the bus before trusting a run.")


def report_hysteresis(rows):
    """Up/down gap at matched throttle: settling lag plus thermal drift."""
    up = {r["throttle_pct"]: r for r in rows if r["direction"] == "up"}
    down = {r["throttle_pct"]: r for r in rows if r["direction"] == "down"}
    common = sorted(set(up) & set(down))
    if not common:
        return
    # Percentages are meaningless where thrust is near zero, so the summary is
    # restricted to steps carrying at least 10% of peak thrust. A 15% "gap" on
    # 0.02 g is noise, and reporting it buries the real gaps higher up the curve.
    peak = max(abs(r["thrust_g_mean"]) for r in rows)
    # An uncalibrated stand reports every thrust as 0 -- link.scale stays None
    # until a MASSCHECK has been run, and stand.json is gitignored, so this is
    # the state of a fresh clone running the documented first sweep. Without
    # this the percentages below are 0/0 and the run ends in a traceback after
    # the data has already been collected.
    if peak <= 0:
        print("\nUp/down hysteresis: every step recorded zero thrust, so the gap")
        print("  cannot be expressed as a percentage. If the load cell is not")
        print("  calibrated yet, run -m MASSCHECK; thrust reads 0 until it is.")
        return
    floor = 0.10 * peak
    gaps = []
    for t in common:
        a, b = up[t]["thrust_g_mean"], down[t]["thrust_g_mean"]
        scale = max(abs(a), abs(b))
        if scale < floor or scale == 0:
            continue
        gaps.append((t, b - a, 100.0 * (b - a) / scale))
    if not gaps:
        return
    worst = max(gaps, key=lambda g: abs(g[2]))
    mean_pct = sum(abs(g[2]) for g in gaps) / len(gaps)
    print("\nUp/down hysteresis over %d steps above %.2f g (of %d matched):"
          % (len(gaps), floor, len(common)))
    print("  mean |gap| %.1f%%   worst %+.1f%% at %d%% throttle (%+.2f g)"
          % (mean_pct, worst[2], worst[0], worst[1]))
    print("  A large gap means 100 ms of settling is not enough, or the motor")
    print("  heated appreciably during the run. Both are worth knowing before")
    print("  the numbers go in a database.")


def run_static(link):
    print("Starting host-timed static step test...")
    results = []
    link.partial_rows = results
    for throttle in THROTTLE_STEPS:
        print("--> %d%%..." % throttle, end="", flush=True)
        link.write_line(str(throttle))
        time.sleep(SETTLE_TIME_SEC)
        link.ser.reset_input_buffer()

        acc = Accumulator()
        dshot = 0
        link.partial_snapshot = lambda: results + (
            [acc.summarize(link, "static", throttle, dshot)] if acc.rows else [])
        deadline = time.time() + 5.0
        while len(acc.thrust) < SAMPLES_PER_STEP and time.time() < deadline:
            line = link.readline()
            if line.startswith("#"):
                link.absorb_meta(line)
                continue
            if not line:
                continue
            row = link.parse_row(line)
            if row is None:
                continue
            acc.add(row)
            dshot = row["dshot"]
        results.append(acc.summarize(link, "static", throttle, dshot))
        link.partial_snapshot = None
        print(" done (%d unique thrust samples)" % len(acc.thrust))

    link.write_line("0")
    return results


def collect_samples(link, command, start_prefix, end_token, mode):
    """Runs a firmware time-series sequence and returns one dict per printed row."""
    print("Requesting firmware %s..." % mode)
    link.write_line(command)

    logs = []
    link.partial_rows = logs
    recording = False
    t0 = None
    last_seq = {"n_rpm": None, "n_thrust": None, "n_ina": None, "n_edt": None}
    quiet = SilenceTimer(end_token)

    while True:
        line = link.readline()
        if not line:
            quiet.check()
            continue
        quiet.feed()
        if line.startswith("#"):
            link.absorb_meta(line)
            print("   " + line)
            continue
        if line.startswith(start_prefix):
            print("--> capturing (%s)" % line)
            recording = True
            continue
        if line == end_token:
            print("--> capture complete.")
            break
        if not recording:
            continue
        row = link.parse_row(line)
        if row is None:
            continue
        if t0 is None:
            t0 = row["t_us"]

        fresh = {}
        for key in last_seq:
            # n_edt is absent on firmware predating the EDT counter; treat it as
            # never-fresh rather than crashing on an older board.
            fresh[key] = key in row and row[key] != last_seq[key]
            last_seq[key] = row.get(key)

        logs.append({
            "mode": mode,
            "time_s": round((row["t_us"] - t0) / 1e6, 6),
            # Absent on firmware predating the column, like erpm_raw below.
            "phase": row.get("phase", ""),
            "throttle_pct": row["throttle_pct"],
            "dshot": row["dshot"],
            "rpm": row["rpm"],
            "erpm": row["erpm"],
            "erpm_raw": row.get("erpm_raw", ""),
            "new_rpm": int(fresh["n_rpm"]),
            "thrust_g": round(link.grams(row), 3),
            "thrust_raw": row["thrust_raw"],
            "new_thrust": int(fresh["n_thrust"]),
            "thrust_age_us": row["thrust_age_us"],
            # When the HX711 conversion actually happened, not when we printed it.
            # Still excludes the sensor's own ~33 ms group delay, which must be
            # characterised per load cell and removed separately.
            "thrust_t_s": round((row["t_us"] - row["thrust_age_us"] - t0) / 1e6, 6),
            "volts": round(row["bus_mv"] / 1000.0, 4),
            "amps": round(row["bus_ua"] / 1e6, 5),
            "shunt_uv": row["shunt_uv"],
            "new_ina": int(fresh["n_ina"]),
            "ina_age_us": row["ina_age_us"],
            "esc_temp_c": row.get("esc_temp_c", 0),
            "esc_stress": row.get("esc_stress", 0),
            # Constants on stock Bluejay; the ESC's internal Comm_Period4x low and
            # high bytes on a firmware patched by tools/patch_bluejay_debug.py.
            # Carried per row because they are the point of that experiment.
            "esc_dbg1": row.get("esc_dbg1", ""),
            "esc_dbg2": row.get("esc_dbg2", ""),
            "new_edt": int(fresh["n_edt"]),
            "edt_age_us": row.get("edt_age_us", ""),
            "flags": row["flags"],
            "flag_names": link.describe_flags(row["flags"]),
        })

    return logs


def step_metrics(rows, levels=None):
    """Rise/fall metrics per commanded step edge, measured on RPM.

    RPM is used rather than thrust deliberately: the HX711 imposes ~30 ms of
    group delay and only resolves ~8 samples across a rise, whereas the DShot
    telemetry updates at kHz. Thrust response follows from RPM plus a
    separately-characterised sensor delay.

    `levels` is the pair of commanded throttles a step sequence alternates
    between, and only edges with one of them on each side are timed. Without
    it every throttle change counts, including the ramp the firmware walks the
    setpoint down at the end of a run to keep regen off the rail: that ramp's
    last leg is long enough to clear the filters below, is not a step, and
    starts from a much lower speed, so averaging it in drags the fall time
    toward whatever the stop did rather than what the motor does.

    RESPONSE knows its two levels and passes them. COASTDOWN passes None on
    purpose: it has no step pair, and the single change to idle is the
    measurement rather than something to filter out.
    """
    changes = [i for i in range(1, len(rows))
               if rows[i]["throttle_pct"] != rows[i - 1]["throttle_pct"]]
    if levels is None:
        edges = changes
    else:
        wanted = set(levels)
        edges = [i for i in changes
                 if rows[i]["throttle_pct"] in wanted
                 and rows[i - 1]["throttle_pct"] in wanted]

    results = []
    for i in edges:
        # Ends at the next throttle change of ANY kind, not the next edge that
        # survived the filter: a rejected change can sit between two accepted
        # ones, and running through it would average a third setpoint into the
        # settled value.
        end = next((j for j in changes if j > i), len(rows))
        seg = rows[i:end]
        if len(seg) < 8:
            continue
        start_rpm = rows[i - 1]["rpm"]
        # Settled value: mean over the last 25% of the segment.
        tail = seg[int(len(seg) * 0.75):]
        end_rpm = sum(r["rpm"] for r in tail) / len(tail)
        delta = end_rpm - start_rpm
        if abs(delta) < 500:
            continue
        t_edge = seg[0]["time_s"]

        def crossing(fraction):
            target = start_rpm + delta * fraction
            for r in seg:
                if (delta > 0 and r["rpm"] >= target) or (delta < 0 and r["rpm"] <= target):
                    return r["time_s"] - t_edge
            return None

        t10, t90, t63 = crossing(0.10), crossing(0.90), crossing(0.632)
        if None in (t10, t90, t63):
            continue
        results.append({
            "direction": "up" if delta > 0 else "down",
            "from_rpm": round(start_rpm), "to_rpm": round(end_rpm),
            "rise_10_90_ms": round((t90 - t10) * 1000, 1),
            "tau_ms": round(t63 * 1000, 1),
        })
    return results


def report_step_metrics(metrics):
    if not metrics:
        print("\n(no clean step edges found)")
        return
    print("\nStep response (measured on RPM):")
    for direction in ("up", "down"):
        subset = [m for m in metrics if m["direction"] == direction]
        if not subset:
            continue
        rise = [m["rise_10_90_ms"] for m in subset]
        tau = [m["tau_ms"] for m in subset]
        spread = (statistics.stdev(rise) if len(rise) > 1 else 0.0,
                  statistics.stdev(tau) if len(tau) > 1 else 0.0)
        print("  %-5s n=%d   10-90%%: %6.1f +- %4.1f ms    tau: %6.1f +- %4.1f ms"
              % (direction, len(subset), statistics.mean(rise), spread[0],
                 statistics.mean(tau), spread[1]))
        print("        %d -> %d RPM" % (subset[0]["from_rpm"], subset[0]["to_rpm"]))


def open_psu(port):
    """A PSU driver for the Kv sweep, or None to fall back to prompting.

    Imported lazily and by path, the same way tools/ reaches measure.py, so a
    missing pyserial-visible supply or a missing tools/ directory cannot stop
    an ordinary run from starting.
    """
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools", "psu.py")
    spec = importlib.util.spec_from_file_location("psu", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    psu = module.PSU(port)
    print("--> supply:   %s on %s" % (psu.identify(), port))
    return psu, module


def run_kv(link, voltages, hold_ms, psu_port=None, dshots=(2000,), settle_ms=1500):
    """Kv by crossing supply voltage with throttle setpoint, prop fitted.

    Voltage alone is one axis, and one axis cannot separate Kv from R: at a
    fixed prop the current is set by the voltage, so the two regressors are
    collinear to r > 0.99 and the fit slides along a ridge. Holding two or more
    DShot setpoints at each voltage gives different currents at the same
    voltage, which is what identifies the two parameters apart. See fit_kv.

    Full throttle stays in the set as the reference level, because at duty ~= 1
    the motor sees the bus voltage directly; the lower levels carry a fitted
    duty rather than an assumed one.

    Setpoints are raw DShot, not percent: `setThrottle` maps integer percent
    onto 100 of the 2000 available steps, and the difference matters most near
    full throttle where the reference level sits.

    The prop stays ON deliberately. An unloaded whoop motor at ~23 kKV would chase
    ~94 kRPM on 4 V, well past its bearing rating. Keeping the load holds RPM in
    the normal band while voltage still provides the spread needed to separate
    Kv from resistance.
    """
    psu, psu_mod, first_step = None, None, True
    if psu_port:
        psu, psu_mod = open_psu(psu_port)
        # Checked before anything spins. With the output off every step would
        # read ~0 V and settle() would blame a cell holding the rail, which is
        # the opposite of what is wrong.
        if not psu.output_on():
            print("[ERROR] the supply's output is off; enable it before a Kv sweep "
                  "(python tools/psu.py --output on)", file=sys.stderr)
            raise RuntimeError("supply output is off")

    points = []
    link.partial_rows = points
    for v_set in voltages:
        if psu is None:
            print("\n>>> Set the bench supply to %.2f V (18650 disconnected)." % v_set)
            input("    Press Enter when settled, or Ctrl-C to stop: ")
        else:
            print("\n>>> Supply to %.2f V..." % v_set, end="", flush=True)
            try:
                psu.set_voltage(v_set)
                # settle() is what detects a parallel cell that should have been
                # disconnected, and it has to come first: the rail takes a moment
                # to fall after a downward step, so an immediate reading catches
                # it mid-decay and blames a cell that is not there. A cell holds
                # the rail *stable* at the wrong value; a discharging output cap
                # keeps moving until it arrives. settle() waits for quiet, then
                # bounds the result against the setpoint.
                measured = psu.settle(v_set)
                print(" settled %.3f V" % measured)
            except psu_mod.PSUError as exc:
                print("\n[ERROR] %s" % exc, file=sys.stderr)
                if first_step:
                    print("        On the first step this is usually the 18650 "
                          "still connected.", file=sys.stderr)
                raise
            first_step = False

        # All throttle levels at this voltage before moving the supply: a
        # voltage step needs settling and a hold does not, so this is both
        # faster and less time spent off the setpoint.
        for dshot in dshots:
            point = run_kv_hold(link, psu, psu_mod, v_set, dshot, hold_ms, settle_ms)
            points.append(point)
    return points


def run_kv_hold(link, psu, psu_mod, v_set, dshot, hold_ms, settle_ms=1500):
    """One logged hold at a given voltage and DShot setpoint.

    settle_ms is time spent at the setpoint before logging starts, and it is
    time the motor spends spinning. Loaded, it has to cover the prop's
    mechanical settling. Unloaded there is nothing to settle -- the rotor
    reaches speed in tens of milliseconds -- and the default would be over a
    second of needless overspeed on a motor with no load to hold it back.
    """
    link.write_line("DHOLD,%d,%d,%d" % (dshot, hold_ms, settle_ms))
    acc = Accumulator()
    recording = False
    supply, supply_tried = None, False
    def snapshot():
        if not acc.rows:
            return link.partial_rows
        s = acc.summarize(link, "kv", round(dshot / 20.0), dshot)
        s.update(v_set=v_set, dshot=dshot)
        if supply:
            s["v_psu"], s["a_psu"] = supply
        return link.partial_rows + [s]
    link.partial_snapshot = snapshot
    quiet = SilenceTimer("END_HOLD")
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
        if row is None:
            continue
        acc.add(row)
            # The supply's own reading, at its terminals rather than at the
            # stand. Two nodes ~16 mOhm of harness apart, so these are not
            # duplicates: the pair is an independent check on INA_SHUNT_UOHM,
            # which otherwise scales every current and watt with nothing able
            # to catch it.
            #
            # It has to be taken from inside the logging window. The firmware
            # zeroes the throttle before it prints END_HOLD, so a reading taken
            # once the harvest loop ends describes a coasting motor on an
            # unloaded rail -- no harness drop to measure, a current that is
            # idle or frankly negative, and every risk of landing in the
            # spin-down overvoltage. START_HOLD is printed before the settle
            # delay, so the first parsed row is the earliest proof the window
            # is really running.
            #
        # One query, not a poll: ~30 ms of round trip during which rows
        # queue in the OS buffer and are read late rather than lost.
        if psu is not None and not supply_tried:
            supply_tried = True
            try:
                v_psu, a_psu = psu.measure()
                supply = (round(v_psu, 4), round(a_psu, 4))
            except psu_mod.PSUError as exc:
                print("\n[WARN] supply did not answer mid-hold: %s" % exc,
                      file=sys.stderr)

    summary = acc.summarize(link, "kv", round(dshot / 20.0), dshot)
    summary["v_set"] = v_set
    summary["dshot"] = dshot
    if supply:
        summary["v_psu"], summary["a_psu"] = supply
    print("    %.2f V @ dshot %-4d -> %5d RPM, %.3f A, %6.2f g%s%s"
          % (v_set, dshot, summary["rpm_mean"], summary["amps_mean"],
             summary["thrust_g_mean"],
             "  [supply %.3f V, %.3f A]" % supply if supply else "",
             "  [" + summary["flag_names"] + "]" if summary["flags"] else ""))
    link.partial_snapshot = None
    return summary


def run_link_test(link, throttles, hold_ms=2000):
    """Telemetry link quality against throttle, motor loaded.

    Separates the two reasons a bidirectional DShot frame goes missing, which
    look identical in a single "errors" number but have opposite fixes:

      corrupt -- the ESC answered and the checksum failed. Electrical: noise
                 pickup, ground offset, reflections. Fix the wiring.
      silent  -- the ESC did not answer at all. On a BLHeli_S/Bluejay ESC the
                 DShot response is generated in software, so as commutation
                 load rises the ESC runs out of time to reply. Nothing about
                 the wiring changes that.

    Silence that climbs with RPM is the ESC's CPU budget, not your cabling.
    """
    print("\nLink test: holding each throttle and counting telemetry outcomes.")
    print("Each hold re-runs from a clean set of counters.\n")
    print("  %-6s %-8s %-9s %-9s %-9s %s"
          % ("thr%", "rpm", "decoded", "corrupt", "silent", "verdict"))
    rows = []
    link.partial_rows = rows
    for thr in throttles:
        link.write_line("HOLD,%d,%d,1200" % (thr, hold_ms))
        acc = Accumulator()
        stats = None
        def snapshot():
            if not acc.rows:
                return rows
            s = acc.summarize(link, "linktest", thr, 0)
            # End-of-hold telemetry counters may never arrive. Leave them
            # absent rather than inventing a zero error rate for an aborted hold.
            return rows + [s]
        link.partial_snapshot = snapshot
        recording = False
        quiet = SilenceTimer("END_HOLD")
        while True:
            line = link.readline()
            if not line:
                quiet.check()
                continue
            quiet.feed()
            if line.startswith("#"):
                link.absorb_meta(line)
                if line.startswith("#STATS"):
                    stats = {}
                    for item in line.split(","):
                        key, sep, value = item.partition("=")
                        if sep:
                            try:
                                stats[key] = float(value)
                            except ValueError:
                                pass
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

        summary = acc.summarize(link, "linktest", thr, 0)
        ok = (stats or {}).get("telem_ok", 0)
        corrupt = (stats or {}).get("telem_corrupt", 0)
        silent = (stats or {}).get("telem_silent", 0)
        total = ok + corrupt + silent
        pct = (lambda v: 100.0 * v / total if total else 0.0)
        verdict = "-"
        if total:
            if corrupt > 0.01 * total:
                verdict = "noisy line"
            elif silent > 0.02 * total:
                verdict = "ESC not replying"
            else:
                verdict = "clean"
        print("  %-6d %-8d %-9s %-9s %-9s %s"
              % (thr, summary["rpm_mean"],
                 "%.1f%%" % pct(ok), "%.2f%%" % pct(corrupt),
                 "%.1f%%" % pct(silent), verdict))
        rows.append({"mode": "linktest", "throttle_pct": thr,
                     "rpm_mean": summary["rpm_mean"],
                     # Spread and sample count carried through with the mean: a
                     # hold is only evidence about a suspect step if it says how
                     # many samples stand behind it and whether they dithered.
                     "rpm_std": summary["rpm_std"], "n_rpm": summary["n_rpm"],
                     "erpm_period_us": summary["erpm_period_us"],
                     "rpm_per_us": summary["rpm_per_us"],
                     "n_codes": summary["n_codes"],
                     "erpm_raw_hist": summary["erpm_raw_hist"],
                     "volts_mean": summary["volts_mean"],
                     "amps_mean": summary["amps_mean"],
                     "thrust_g_mean": summary["thrust_g_mean"],
                     "telem_ok": int(ok), "telem_corrupt": int(corrupt),
                     "telem_silent": int(silent),
                     "silent_pct": round(pct(silent), 2),
                     "corrupt_pct": round(pct(corrupt), 3),
                     "edt_frames": int((stats or {}).get("edt_frames", 0)),
                     "flags": summary["flags"], "flag_names": summary["flag_names"]})
        link.partial_snapshot = None
    return rows


def run_fine_walk(link, values, reps, hold_ms, settle_ms=1200):
    """Hold a series of raw DShot setpoints, in randomised order.

    Integer percent reaches 100 of DShot's 2000 steps, so near full throttle one
    percent is several hundred RPM -- coarser than the telemetry's own lattice,
    and far too coarse to say where a plateau starts. `DHOLD` addresses the value
    directly, which is what makes a fine walk possible at all.

    The order is randomised and every setpoint visited `reps` times, because the
    reported RPM drifts upward in period over running time at a *fixed* operating
    point. Walked in ascending order, that drift is indistinguishable from a
    throttle response -- it would read as a smooth curve that flattens at the top,
    which is exactly the artefact being investigated. Randomised, the drift lands
    on all setpoints equally and can be estimated separately from the residuals.
    """
    order = [(v, rep) for rep in range(reps) for v in values]
    # Fixed seed: the run stays reproducible, which matters when comparing two
    # sessions, while still decorrelating setpoint from time.
    random.Random(20260813).shuffle(order)

    print("\nFine DShot walk: %d setpoints x %d reps, %d ms each, randomised order."
          % (len(values), reps, hold_ms))
    print("Randomised because the reported period drifts with running time; "
          "ascending order\nwould record that drift as a throttle response.\n")
    print("  %-6s %-6s %-6s %-9s %-9s %-8s %s"
          % ("visit", "dshot", "pct", "rpm", "period", "n_codes", "volts"))
    rows = []
    link.partial_rows = rows
    for visit, (value, rep) in enumerate(order):
        link.write_line("DHOLD,%d,%d,%d" % (value, hold_ms, settle_ms))
        acc = Accumulator()
        recording = False
        dshot = value
        pct = 0
        def snapshot():
            if not acc.rows:
                return rows
            s = acc.summarize(link, "finewalk", pct, dshot)
            s.update(dshot_set=value, visit=visit, rep=rep)
            return rows + [s]
        link.partial_snapshot = snapshot
        quiet = SilenceTimer("END_HOLD")
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
                acc.add(row)
                dshot = row["dshot"]
                pct = row["throttle_pct"]

        s = acc.summarize(link, "finewalk", pct, dshot)
        s["dshot_set"] = value
        s["visit"] = visit
        s["rep"] = rep
        rows.append(s)
        link.partial_snapshot = None
        print("  %-6d %-6d %-6s %-9d %-9d %-8d %.4f"
              % (visit, value, s["throttle_pct"], s["rpm_mean"],
                 s["erpm_period_us"], s["n_codes"], s["volts_mean"]))
    return rows


def report_fine_walk(rows):
    """Separate the throttle response from the time drift.

    Two numbers matter. The slope of RPM against setpoint says what the motor
    actually does per DShot step -- the thing a 1% sweep cannot resolve. The slope
    of the residual against visit order says how much of any apparent plateau is
    just the telemetry drifting while the run proceeded.
    """
    if len(rows) < 4:
        return
    xs = [float(r["dshot_set"]) for r in rows]
    ys = [float(r["rpm_mean"]) for r in rows]

    def lstsq(a, b):
        n = len(a)
        ma, mb = sum(a) / n, sum(b) / n
        den = sum((v - ma) ** 2 for v in a)
        if den == 0:
            return 0.0, mb
        slope = sum((a[i] - ma) * (b[i] - mb) for i in range(n)) / den
        return slope, mb - slope * ma

    slope, intercept = lstsq(xs, ys)
    resid = [ys[i] - (slope * xs[i] + intercept) for i in range(len(xs))]
    drift, _ = lstsq([float(r["visit"]) for r in rows], resid)

    print("\nFine walk: throttle response vs time drift")
    print("  RPM per DShot step   %+.1f   (a 1%% throttle step is ~20 of these, "
          "so ~%.0f RPM)" % (slope, abs(slope) * 20.2))
    print("  RPM per visit        %+.1f   drift while the run proceeded, "
          "estimated from residuals" % drift)
    span = (max(xs) - min(xs)) * abs(slope)
    print("  Over the %d-step span walked, response is ~%.0f RPM against "
          "~%.0f RPM of drift" % (int(max(xs) - min(xs)), span,
                                  abs(drift) * len(rows)))
    if abs(drift) * len(rows) > 0.25 * span:
        print("  The drift is a substantial fraction of the response. Any "
              "ascending-order run\n  over this band is measuring both at once.")

    # Per-setpoint means across reps. Randomised order means the drift is spread
    # evenly over setpoints, so these means are the closest thing to a
    # drift-free throttle response the stand can produce.
    by_set = {}
    for r in rows:
        by_set.setdefault(r["dshot_set"], []).append(r)
    print("\n  %-7s %-6s %-10s %-9s %-9s %s"
          % ("dshot", "pct", "rpm_mean", "spread", "period", "reps"))
    prev = None
    for value in sorted(by_set):
        group = by_set[value]
        rpms = [float(g["rpm_mean"]) for g in group]
        mean = sum(rpms) / len(rpms)
        step = "" if prev is None else "  %+.0f vs previous" % (mean - prev)
        print("  %-7d %-6s %-10.0f %-9.0f %-9.1f %d%s"
              % (value, group[0]["throttle_pct"], mean,
                 max(rpms) - min(rpms),
                 sum(float(g["erpm_period_us"]) for g in group) / len(group),
                 len(group), step))
        prev = mean
    print("  spread is max-min across reps of the same setpoint: it is the "
          "repeatability\n  of a single hold, and any real step has to exceed it.")


def report_link_test(link, rows, hold_ms=1000):
    usable = [r for r in rows if r["rpm_mean"] > 0]
    if len(usable) < 2:
        return
    corrupt = sum(r["telem_corrupt"] for r in rows)
    silent = sum(r["telem_silent"] for r in rows)
    print("\n  totals: %d corrupt, %d silent" % (corrupt, silent))

    # A loss percentage is meaningless on its own: frames are polled at a few
    # kHz while rows print at ~250-670 Hz depending on the fitted part, so what
    # matters is whether enough answers survive to fill a step, not what
    # fraction was lost.
    worst = min(usable, key=lambda r: r["telem_ok"])
    answered_hz = worst["telem_ok"] / (hold_ms / 1000.0)
    rows_s = rows_per_second(link)
    headroom = answered_hz / rows_s
    print("  worst case %.0f answered frames/s at %d rpm, against %.0f rows/s"
          % (answered_hz, worst["rpm_mean"], rows_s))
    if headroom >= 5:
        print("  -> %.0fx more answers than rows printed. The loss costs no"
              % headroom)
        print("     samples and needs no action; RPM per step is set by the")
        print("     print rate, not by the link.")
        return
    print("  -> only %.1fx headroom over the print rate; losses now cost real"
          % headroom)
    print("     RPM samples per step. Worth pursuing.")
    if corrupt > silent:
        print("\n  Corruption dominates. The ESC is answering and the answer is")
        print("  arriving damaged, so this is electrical: check the signal ground")
        print("  return, twist, routing away from the phase wires, and length.")
        return
    lo = min(usable, key=lambda r: r["rpm_mean"])
    hi = max(usable, key=lambda r: r["rpm_mean"])
    print("\n  Silence dominates -- the ESC is not replying rather than replying")
    print("  badly, which no amount of cabling work changes.")
    print("    %5d rpm -> %.1f%% silent" % (lo["rpm_mean"], lo["silent_pct"]))
    print("    %5d rpm -> %.1f%% silent" % (hi["rpm_mean"], hi["silent_pct"]))
    if hi["silent_pct"] > lo["silent_pct"] + 2.0:
        print("  Silence rises with RPM: that is the ESC running out of time to")
        print("  generate the response between commutations, which is a known")
        print("  limit of software DShot on BLHeli_S. Expected, not a fault --")
        print("  budget for fewer RPM samples per step at high throttle.")
    else:
        print("  Silence does not track RPM, so ESC CPU load is not the whole")
        print("  story. Worth checking whether it tracks current instead, which")
        print("  would point back at the ground return.")


# Flags that invalidate an *aggregate*. Deliberately not the same set that
# invalidates a row (see FATAL_FLAGS in plot_comparison.py), because a summary
# is not a row:
#
# Accumulator deduplicates on the sequence counters, so a row flagged
# rpm_stale or thrust_stale contributed nothing to the means -- its value was a
# repeat of a sample already counted when it was fresh. In a summary those
# flags say only "at least one of N printed rows was stale", and the longer the
# window the likelier that is. A 1 s hold prints 250 rows against a sweep
# step's 25, so ORing them over rows condemns long windows for being long. What
# actually decides whether an aggregate is supported is how many *fresh*
# samples landed, which is what n_rpm / n_ina count.
#
# Saturation and watchdog stay fatal: those corrupt the samples that do enter
# the means, so no amount of deduplication saves them.
AGGREGATE_FATAL_FLAGS = frozenset(("ina_shunt_sat", "ina_i_sat", "watchdog"))
AGGREGATE_FATAL_MASK = (FLAG_INA_SHUNT_SAT | FLAG_INA_I_SAT | FLAG_WATCHDOG)

# Floors, not expectations. Rows print at 250 Hz and the INA converts at 100 Hz,
# so a healthy 1 s hold lands 250 and 100; these only catch a point whose mean
# rests on almost nothing.
MIN_RPM_SAMPLES = 20
MIN_INA_SAMPLES = 10

# A failed start, not a slow motor. Below the ESC's minimum operating voltage
# the link stays up and answers telemetry while the startup ramp never
# completes, which reads as a few hundred RPM at a few milliamps against a
# commanded full throttle. Left in, one such point drags a Kv fit to a negative
# slope and the whole run reports as unfittable rather than as one bad step.
MIN_RUNNING_RPM = 1000


def usable_point(point):
    """Whether a summary point may enter a fit, and why not if it may not."""
    names = {n for n in (point.get("flag_names") or "").split("|") if n}
    fatal = names & AGGREGATE_FATAL_FLAGS
    if fatal:
        return False, "invalidating flags (%s)" % ", ".join(sorted(fatal))
    if not point.get("rpm_mean"):
        return False, "no RPM recorded"
    if point["rpm_mean"] < MIN_RUNNING_RPM:
        return False, ("motor did not start (%d RPM at %.3f A -- below the ESC's "
                       "minimum operating voltage?)"
                       % (point["rpm_mean"], point.get("amps_mean", 0)))
    if point.get("n_rpm", 0) < MIN_RPM_SAMPLES:
        return False, "only %d fresh RPM samples" % point.get("n_rpm", 0)
    if point.get("n_ina", 0) < MIN_INA_SAMPLES:
        return False, "only %d fresh INA samples" % point.get("n_ina", 0)
    return True, ""


def collinearity(pairs):
    """Pearson r between the two regressors of the Kv fit.

    Worth reporting with every fit. At |r| near 1 the two columns are nearly
    the same variable, Kv and R trade off along a ridge, and the solution
    slides freely: sub-percent changes in RPM then move Kv by several percent
    while the residuals stay put. That is a property of how the points were
    collected, not of the motor, and it is invisible in the fitted numbers.
    """
    n = len(pairs)
    if n < 2:
        return None
    mx = sum(x for x, _ in pairs) / n
    my = sum(y for _, y in pairs) / n
    num = sum((x - mx) * (y - my) for x, y in pairs)
    den = (sum((x - mx) ** 2 for x, _ in pairs)
           * sum((y - my) ** 2 for _, y in pairs)) ** 0.5
    return num / den if den else None


def _solve_kv_linear(samples):
    """Least squares of rpm = a*v_eff - b*i_eff over (v_eff, i_eff, rpm).

    Returns (a, b, rms) where a is Kv and b is Kv*R.
    """
    sxx = sum(v * v for v, _, _ in samples)
    sxz = sum(v * i for v, i, _ in samples)
    szz = sum(i * i for _, i, _ in samples)
    sxy = sum(v * r for v, _, r in samples)
    szy = sum(i * r for _, i, r in samples)
    det = sxx * szz - sxz * sxz
    if abs(det) < 1e-12:
        return None
    a = (sxy * szz - szy * sxz) / det
    b = -(szy * sxx - sxy * sxz) / det
    resid = [r - (a * v - b * i) for v, i, r in samples]
    rms = (sum(e * e for e in resid) / len(resid)) ** 0.5
    return a, b, rms


def fit_kv(points):
    """Kv and lumped resistance from RPM against supply voltage and current.

    One throttle setpoint gives an ill-conditioned fit and cannot be rescued
    afterwards. With a fixed prop at full throttle, load torque goes as n^2 and
    n goes as V, so current is a function of voltage: the two regressors are
    collinear to r > 0.99, the model has two free parameters and the data has
    one axis. Kv and R then trade off along a ridge.

    Extra throttle setpoints are supported and are worth having, but be clear
    about what they do NOT do. Below duty 1 the motor stops seeing the bus:

        V_motor = duty * V_bus        I_motor = I_bus / duty

    which means a throttle level is just another way of changing the voltage
    the motor sees. A fixed prop holds the motor on a one-dimensional operating
    curve, so those points land on the *same* curve as the voltage sweep, and
    the motor-side regressors stay collinear (checked against the emulator,
    where corr rises to 0.9998 across two levels). They add averaging, not an
    axis. Separating Kv from R properly needs the load changed -- a different
    prop gives a different current at the same V_motor.

    Duty is not readable from the ESC and the DShot-to-duty mapping is
    quantised (8-bit of high byte at 96 kHz on Bluejay), so it is fitted rather
    than assumed: one unknown per throttle level, with the highest level pinned
    at duty 1. For fixed duties the model is linear in (Kv, Kv*R), so this is a
    small search over the duties with an exact solve inside it, not a general
    optimiser. On noiseless emulator data it recovers the true duty to three
    decimals, so the machinery is sound even where the conditioning is not.
    """
    usable = [p for p in points if usable_point(p)[0]]
    if len(usable) < 2:
        return None

    levels = sorted({int(p.get("dshot") or 2000) for p in usable}, reverse=True)
    pairs = [(p["volts_mean"], p["amps_mean"]) for p in usable]

    if len(levels) < 2:
        solved = _solve_kv_linear(
            [(p["volts_mean"], p["amps_mean"], p["rpm_mean"]) for p in usable])
        if not solved or solved[0] <= 0:
            return None
        kv, b, rms = solved
        return {"kv_rpm_per_v": kv, "r_ohm": b / kv, "rms_rpm": rms,
                "n_points": len(usable), "n_levels": 1,
                "duty": {levels[0]: 1.0}, "corr": collinearity(pairs)}

    # Coordinate descent over one duty per level below the top one. The surface
    # is smooth and one-dimensional per level, so a coarse-to-fine bracket is
    # enough and keeps this dependency-free.
    duty = {levels[0]: 1.0}
    for level in levels[1:]:
        duty[level] = float(level) / float(levels[0])   # starting guess only

    def residual_for(duties):
        samples = []
        for p in usable:
            d = duties[int(p.get("dshot") or 2000)]
            samples.append((d * p["volts_mean"], p["amps_mean"] / d, p["rpm_mean"]))
        return _solve_kv_linear(samples)

    for _ in range(4):
        for level in levels[1:]:
            best, best_rms = duty[level], None
            lo, hi = max(0.05, duty[level] - 0.35), min(1.0, duty[level] + 0.35)
            steps = 70
            for k in range(steps + 1):
                trial = dict(duty)
                trial[level] = lo + (hi - lo) * k / steps
                solved = residual_for(trial)
                if not solved or solved[0] <= 0:
                    continue
                if best_rms is None or solved[2] < best_rms:
                    best, best_rms = trial[level], solved[2]
            duty[level] = best

    solved = residual_for(duty)
    if not solved or solved[0] <= 0:
        return None
    kv, b, rms = solved
    # Conditioning is judged on the regressors actually fitted, which is the
    # whole point of the second axis.
    eff = [(duty[int(p.get("dshot") or 2000)] * p["volts_mean"],
            p["amps_mean"] / duty[int(p.get("dshot") or 2000)]) for p in usable]
    return {"kv_rpm_per_v": kv, "r_ohm": b / kv, "rms_rpm": rms,
            "n_points": len(usable), "n_levels": len(levels),
            "duty": duty, "corr": collinearity(eff),
            "corr_raw": collinearity(pairs)}


def report_kv(points, fit):
    print("\n%-8s %-7s %-9s %-9s %-9s %-7s %-4s %s"
          % ("V set", "dshot", "V bus", "Amps", "RPM", "n_rpm", "use", "flags"))
    print("-" * 76)
    excluded = []
    for p in points:
        ok, why = usable_point(p)
        if not ok:
            excluded.append((p["v_set"], why))
        print("%-8.2f %-7d %-9.3f %-9.4f %-9d %-7d %-4s %s"
              % (p["v_set"], int(p.get("dshot") or 2000), p["volts_mean"],
                 p["amps_mean"], p["rpm_mean"],
                 p.get("n_rpm", 0), "yes" if ok else "NO", p["flag_names"]))
    for v_set, why in excluded:
        print("  %.2f V excluded: %s" % (v_set, why))
    # Staleness flags are reported and kept: the means are built from fresh
    # samples only, so what these mark is that some printed row repeated a
    # value, not that the average rests on one.
    kept = sorted({n for p in points for n in (p["flag_names"] or "").split("|")
                   if n and n not in AGGREGATE_FATAL_FLAGS})
    if kept:
        print("  [note] %s present but kept -- deduplicated out of the means "
              "(n_rpm/n_ina show the support)" % ", ".join(kept))
    if not fit:
        n_usable = sum(1 for p in points if usable_point(p)[0])
        if n_usable < 2:
            print("\n[WARN] only %d usable point(s); a Kv fit needs at least 2"
                  % n_usable)
        else:
            # Distinguished because the two have completely different fixes:
            # collect more points, versus look at the points you have.
            print("\n[WARN] %d usable points but the fit is degenerate (it returned"
                  " a non-positive Kv)." % n_usable)
            print("       The voltage span may be too small to give a slope, or a")
            print("       point that looks usable is not -- check the table above.")
        return
    print("\nFit over %d points, %d throttle level(s):  RPM = Kv * (V - I*R)"
          % (fit["n_points"], fit.get("n_levels", 1)))
    print("  Kv = %.0f RPM/V" % fit["kv_rpm_per_v"])
    # Not "motor + ESC + wiring" as this once claimed, and not a waveform factor
    # either -- that explanation was tested with a scope and failed: the ESC on
    # this bench runs at duty 1 at full throttle, so there is no PWM to inflate
    # I_rms/I_avg. Measured directly, the winding and the ESC-plus-wiring path
    # together account for about three quarters of the fitted value. The rest is
    # not attributed to anything, which is why this says only what it is.
    print("  R  = %.0f mOhm   (effective: exceeds the measured DC path, "
          "mechanism not established)" % (fit["r_ohm"] * 1000))
    print("  residual rms = %.0f RPM" % fit["rms_rpm"])
    if fit.get("n_levels", 1) > 1:
        print("  fitted duty:  %s"
              % ", ".join("dshot %d = %.3f" % (d, v)
                          for d, v in sorted(fit["duty"].items(), reverse=True)))
    naive = max(p["rpm_mean"] / p["volts_mean"] for p in points if p["volts_mean"] > 0)
    print("  for comparison, naive RPM/V = %.0f (understates Kv by %.0f%%)"
          % (naive, 100 * (1 - naive / fit["kv_rpm_per_v"])))

    # The number that says whether the two fitted parameters were separable at
    # all. Printed always, because an ill-conditioned fit looks completely
    # normal from its coefficients -- it was only caught here by running the
    # same sweep twice and watching Kv move 5% while the RPMs moved 0.5%.
    corr = fit.get("corr")
    if corr is not None:
        # Motor-side regressors (duty*V and I/duty), which are the physical
        # ones. The bus-side pair can look better conditioned than it is,
        # purely because bus current is duty-scaled -- do not read that as
        # separation.
        print("\n  conditioning: corr(V_motor, I_motor) = %+.4f" % corr)
        if fit.get("corr_raw") is not None and fit.get("n_levels", 1) > 1:
            print("                (bus-side corr(V, I) = %+.4f, which is the same"
                  % fit["corr_raw"])
            print("                 points seen through the duty scaling, not a")
            print("                 second axis)")
        if abs(corr) > 0.99:
            print("  [WARN] the regressors are collinear, so Kv and R are only")
            print("         weakly identified: they trade off along a ridge, and")
            print("         sub-percent changes in RPM move Kv by several percent")
            print("         while the residuals barely change.")
            print("         Throttle levels do NOT fix this. A fixed prop holds the")
            print("         motor on a one-dimensional operating curve, and lowering")
            print("         duty moves along that curve exactly as lowering the bus")
            print("         voltage does. Separating Kv from R needs the LOAD")
            print("         changed -- a different prop at the same voltages gives a")
            print("         different current at the same V_motor, which is what")
            print("         identifies R. Failing that, quote Kv with its spread")
            print("         across repeats rather than as a single figure.")

    print("\n  R is lumped and effective, not a resistance an ohmmeter would")
    print("  give: it carries lead and connector drop and an unattributed")
    print("  remainder -- on this bench the directly measured winding and ESC")
    print("  path account for ~76% of it. Sense voltage at the ESC input, not")
    print("  the supply display, or that drop lands in R and depresses Kv.")


def apply_density(rows, rho):
    """Normalise thrust to ISA density. Valid at matched RPM, where thrust is
    directly proportional to rho. Electrical power is left alone: aerodynamic
    power scales with density but motor I^2R losses do not."""
    scale = ISA_RHO / rho
    for r in rows:
        if "thrust_g_mean" in r:
            r["thrust_g_iso"] = round(r["thrust_g_mean"] * scale, 3)
        elif "thrust_g" in r:
            r["thrust_g_iso"] = round(r["thrust_g"] * scale, 3)


def qc_actuator_disk(rows, diameter_mm, rho):
    """Flags any row claiming better efficiency than an ideal actuator disk."""
    if not diameter_mm:
        print("\n[WARN] no --prop-diameter-mm given, skipping the actuator-disk QC gate.")
        return
    peak = max((abs(r.get("thrust_g_mean", 0.0)) for r in rows), default=0.0)
    floor = 0.10 * peak
    worst = None
    violations = 0
    for r in rows:
        thrust = r.get("thrust_g_mean", 0.0)
        eff = r.get("eff_g_per_w", 0.0)
        if thrust < floor or eff <= 0:
            continue
        ideal = ideal_efficiency_g_per_w(thrust, diameter_mm, rho)
        fom = eff / ideal
        r["fom"] = round(fom, 3)
        if worst is None or fom > worst[0]:
            worst = (fom, r)
        if fom > 1.0:
            violations += 1
    if worst is None:
        return
    print("\nActuator-disk QC (%.0f mm disk, rho %.4f):" % (diameter_mm, rho))
    print("  peak figure of merit %.2f at %d%% throttle  (typical whoop 0.2-0.3)"
          % (worst[0], worst[1]["throttle_pct"]))
    if violations:
        print("  *** %d rows exceed the physical limit (FoM > 1.0). ***" % violations)
        print("  That is impossible, so something upstream is wrong -- most likely")
        print("  the shunt value, the scale factor, or clipped current.")
    elif worst[0] > 0.45:
        print("  Higher than expected for this class; worth double-checking the")
        print("  shunt value and load-cell calibration.")


def _unique_thrust_raw(link, seconds):
    """Unique HX711 conversions over a fixed duration, in RAW counts.

    Raw rather than grams because calibration is exactly what a MASSCHECK run
    does not have yet: the first one on a new stand has no factor at all.
    """
    seen = {}
    deadline = time.time() + seconds
    while time.time() < deadline:
        line = link.readline()
        if not line or line.startswith("#"):
            continue
        row = link.parse_row(line)
        if row is None or row["n_thrust"] in seen:
            continue
        seen[row["n_thrust"]] = row["thrust_raw"]
    return list(seen.values())


def _unique_thrust(link, seconds, callback=None):
    """As above, converted to grams. Meaningless until a factor is known."""
    values = [link.grams({"thrust_raw": r}) for r in _unique_thrust_raw(link, seconds)]
    for v in values:
        if callback:
            callback(v)
    return values


def request_stats(link, timeout=2.0):
    """Ask the firmware for its telemetry counters, motor stopped."""
    link.write_line("STATS")
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = link.readline()
        if not line or not line.startswith("#"):
            continue
        link.absorb_meta(line)
        if not line.startswith("#STATS"):
            continue
        out = {}
        for item in line.split(","):
            key, sep, value = item.partition("=")
            if not sep:
                continue
            try:
                out[key] = float(value)
            except ValueError:
                out[key] = value
        return out
    return None


def watch_idle(link, seconds, interval=5.0):
    """Watch the zero and the telemetry link with the motor stopped.

    The handful of samples --check prints says nothing about drift, and drift
    between sessions dominates the error budget -- it is worth more than any
    refinement of the scale factor. Bracketing the window with STATS also
    separates a live EDT link from one stale frame, which is otherwise
    invisible until a sequence runs.
    """
    before = request_stats(link)
    started = time.time()
    print("\n  Watching the zero for %ds (motor stopped, Ctrl-C to cut short):" % seconds)
    print("    %-7s %-11s %-11s %-11s %s" % ("t (s)", "mean", "p-p", "vs first", "n"))
    points = []
    try:
        while True:
            remaining = seconds - (time.time() - started)
            if remaining <= 0.5:
                break
            values = _unique_thrust(link, min(interval, remaining))
            if not values:
                continue
            now = time.time() - started
            mean = statistics.mean(values)
            points.append((now, mean, max(values) - min(values)))
            print("    %-7.0f %-11s %-11s %-11s %d"
                  % (now, "%+.3f g" % mean, "%.3f g" % points[-1][2],
                     "%+.3f g" % (mean - points[0][1]), len(values)))
    except KeyboardInterrupt:
        print("    (cut short)")
    elapsed = time.time() - started

    if len(points) >= 3:
        # First-vs-last would be phase-sensitive: a zero that oscillates with
        # air currents rather than creeping monotonically gives a large or small
        # difference depending only on where the window happened to stop. Fit a
        # trend for the systematic part and report the spread separately, since
        # they have different causes and different fixes.
        times = [p[0] for p in points]
        means = [p[1] for p in points]
        mt, mm = statistics.mean(times), statistics.mean(means)
        denom = sum((t - mt) ** 2 for t in times)
        slope = sum((t - mt) * (m - mm) for t, m in zip(times, means)) / denom if denom else 0.0
        trend = slope * (times[-1] - times[0])
        span = max(means) - min(means)
        noise = statistics.median([p[2] for p in points])
        # Judge the trend as a rate projected over a session, not as the total
        # over this window: a few minutes of watching understates a creep that
        # runs for the twenty a real sweep-plus-calibration session takes.
        session_s = 600.0
        projected = slope * session_s
        print("\n    trend      %+.3f g over %.0fs (%+.3f g/min, least squares)"
              % (trend, elapsed, slope * 60.0))
        print("               -> %+.3f g over a 10 min session if it continues" % projected)
        print("    wander     %.3f g between window means, after the trend" % span)
        print("    short-term %.3f g typical peak-to-peak within a window" % noise)

        if abs(projected) > 0.1:
            print("\n    The zero is creeping in one direction at a rate that matters")
            print("    over a session, even though it is small over this window.")
            print("    Either let it warm up until the rate falls, or re-tare between")
            print("    runs -- MASSCHECK and every sequence already re-tare, so the")
            print("    exposure is one run's duration, not the whole session. No")
            print("    sensitivity correction touches a moving zero.")
        elif span > 0.1:
            print("\n    The zero wanders without a consistent direction. It will not")
            print("    accumulate over a session, but it is the floor on any single")
            print("    reading. Draught-shield the cell if you need better.")
        else:
            print("\n    No systematic creep. The wander figure above is the floor on")
            print("    any single thrust reading -- quote results no finer than that,")
            print("    and prefer calibration masses large enough that it is a small")
            print("    fraction of them.")

    after = request_stats(link)
    if not (before and after):
        print("\n  (no #STATS -- firmware predates the STATS command, reflash to see"
              " telemetry counters)")
        return
    ok = after.get("telem_ok", 0) - before.get("telem_ok", 0)
    bad = after.get("telem_bad", 0) - before.get("telem_bad", 0)
    edt = after.get("edt_frames", 0) - before.get("edt_frames", 0)
    total = ok + bad
    probe = after.get("i2c_probe_reads")
    if probe is not None:
        reads = int(probe) - int(before.get("i2c_probe_reads", 0))
        badp = int(after.get("i2c_probe_bad", 0)) - int(before.get("i2c_probe_bad", 0))
        print("\n  I2C integrity over the same window:")
        print("    probe reads       %-8d (revision register 0x1F)" % reads)
        print("    mangled           %-8d (%.3f%%)"
              % (badp, 100.0 * badp / reads if reads else 0.0))
        if badp:
            # A wrong constant can only be a wrong transfer, so this settles
            # what a bad ADC sample means without any further argument.
            print("    last bad byte     %s" % after.get("i2c_probe_last", "?"))
            print("\n    The bus is mangling transfers. A register that cannot change")
            print("    came back wrong, so a corrupt thrust sample is a corrupt READ,")
            print("    not a corrupt conversion.")
            # Naming what was already excluded by measurement, so nobody spends
            # an evening re-excluding it. Bench-specific: on the reference stand
            # the fault was measured (2026-09-01) at ~0.018% with the ESC
            # disconnected, then removed by the 2026-09-02 rewire with the
            # mechanism never identified -- treat "it is the supply" as that
            # bench's history, not a law. See BENCH_NOTES.md.
            print("    On the reference bench this was the supply: 0.018%% with the")
            print("    ESC disconnected and no current flowing. Ruled out there by")
            print("    measurement -- bus speed, the DRDY pin, the DShot signal wire,")
            print("    and pull-ups -- then removed by a rewire with the mechanism")
            print("    unidentified (2026-09-02). Suspect common-mode coupling, but")
            print("    re-measure rather than assuming this bench is the same.")
        else:
            print("\n    No mangled transfers. Weaker evidence than a hit would be --")
            print("    the ADC read is 3 bytes to this one, so it is more exposed per")
            print("    transaction -- but on equal counts a corrupt thrust sample")
            print("    would point inside the part, not at the bus.")

    print("\n  Telemetry over the same window:")
    print("    frames decoded    %-8d (%.0f/s)" % (ok, ok / elapsed if elapsed else 0))
    print("    checksum failures %-8d (%.1f%% of %d)"
          % (bad, 100.0 * bad / total if total else 0.0, total))
    if not edt_requested(link):
        print("    EDT               not requested (--edt to ask for it)")
    else:
        print("    EDT frames        %-8d (%.0f/s)"
              % (edt, edt / elapsed if elapsed else 0))
        print("    esc_temp_c        %-8s esc_max_stress %s"
              % (after.get("esc_temp_c", "?"), after.get("esc_max_stress", "?")))
        if edt == 0:
            print("\n    NO EDT FRAMES ARRIVED. Extended telemetry was asked for and")
            print("    is not running, so esc_stress -- the desync canary -- is dead,")
            print("    and any esc_temp_c shown is a stale artifact of one frame that")
            print("    passed checksum by chance. Either the ESC ignored DShot command")
            print("    13 (it may not be Bluejay/BLHeli_S with EDT support, or it may")
            print("    drop EDT once armed) or the telemetry line is too noisy. Do not")
            print("    trust esc_* columns until this is non-zero.")
    # Independent of the EDT result: these are different faults, and chaining
    # them meant a dead EDT link suppressed the report of a bad signal wire.
    if total and bad > total * 0.05:
        print("\n    Link error rate above 5%. Check the signal wire and grounding")
        print("    before recording -- lost frames bias RPM-derived metrics.")


def wait_stable(link, window=8, timeout=20.0, tol_counts=None):
    """Block until the reading stops drifting.

    Mechanical settling after applying a mass takes seconds and dwarfs the
    sensor's own noise -- averaging blindly measures the transient instead of
    the value. Tolerance scales with the reading, since creep is proportional
    to load rather than absolute.

    Returns "settled", or the reason it gave up: "drifting" or "noisy". Those
    two are different faults with different fixes and must not be merged.
    Waiting longer cures a transient and does nothing whatever for a noise
    floor, so reporting the wrong one sends you to re-seat weights that were
    already at rest.
    """
    recent = []
    link.ser.reset_input_buffer()
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = link.readline()
        if not line or line.startswith("#"):
            continue
        row = link.parse_row(line)
        if row is None:
            continue
        recent.append(row["thrust_raw"])
        if len(recent) > window:
            recent.pop(0)
        if len(recent) == window:
            mean = sum(recent) / window
            # The criterion is the same 0.05 g / 0.2% it always was, expressed
            # in counts so it survives having no calibration yet. Without a
            # factor the absolute floor has to be measured instead of assumed.
            if tol_counts is not None:
                floor = tol_counts
            elif link.scale:
                floor = 0.05 * abs(link.scale)
            else:
                floor = 0.0
            tol = max(floor, 0.002 * abs(mean))
            if tol > 0 and (max(recent) - min(recent)) <= tol:
                return "settled"
    return _why_unstable(recent)


def _why_unstable(recent):
    """Name what a failed wait_stable() window shows: drift or noise.

    Fit a line and ask whether its slope is real. Comparing the trend against
    the leftover spread is not enough: over eight samples a least-squares slope
    absorbs some of the noise, so pure noise fits a slope large enough to look
    like drift. Weigh the slope against its own standard error instead, which
    is what tells a settling transient from a noise floor.
    """
    n = len(recent)
    if n < 4:
        return "noisy"
    mean = sum(recent) / n
    xm = (n - 1) / 2.0
    denom = sum((i - xm) ** 2 for i in range(n))
    if not denom:
        return "noisy"
    slope = sum((i - xm) * (v - mean) for i, v in enumerate(recent)) / denom
    ss_resid = sum((v - (mean + slope * (i - xm))) ** 2
                   for i, v in enumerate(recent))
    if ss_resid <= 0:
        # A perfect straight line is nothing but drift, unless it is flat --
        # and a flat window never reaches here, because it passed the check.
        return "drifting" if slope else "noisy"
    stderr = (ss_resid / (n - 2) / denom) ** 0.5
    # 3 sigma. A real settling transient clears it by a wide margin and a flat
    # noisy window sits near zero, so the exact threshold changes nothing.
    return "drifting" if abs(slope) > 3.0 * stderr else "noisy"


# The two ways a point can fail, and the fix for each. Kept together so the
# bench message and the report cannot drift apart.
UNSETTLED_NOTE = {
    "drifting": "NEVER SETTLED - reading still drifting, point unusable",
    "noisy": "TOO NOISY - spread exceeds tolerance at this load, point unusable",
}


def measure_noise_floor(link, seconds=3.0):
    """Peak-to-peak of the cell at rest, in raw counts.

    Used as the settling tolerance during a first calibration, when there is no
    factor to express 0.05 g in. Measuring it beats assuming a value that would
    be wrong for any cell but the one it was written for.
    """
    values = _unique_thrust_raw(link, seconds)
    if len(values) < 4:
        return None
    return max(values) - min(values)


def read_thrust_avg(link, seconds=2.0):
    """Mean thrust over unique conversions: (grams, sd_grams, n, mean_counts, n_mangled).

    Raw counts travel alongside the grams so the calibration fit can work
    without a factor, and so the run stays re-derivable afterwards.

    Corrupt transfers are rejected here for the same reason they are rejected
    in Accumulator, and it matters more here: this is what a calibration point
    is built from, so one mangled sample would be fitted as if it were a
    weight, and the resulting factor is wrong everywhere afterwards.
    """
    raw = _unique_thrust_raw(link, seconds)
    if not raw:
        return 0.0, 0.0, 0, 0.0, 0
    raw, n_mangled = reject_mangled(raw)
    if not raw:
        return 0.0, 0.0, 0, 0.0, n_mangled
    raw_mean = statistics.mean(raw)
    grams = [link.grams({"thrust_raw": r}) for r in raw]
    if len(grams) == 1:
        return grams[0], 0.0, 1, raw_mean, n_mangled
    return (statistics.mean(grams), statistics.stdev(grams), len(grams),
            raw_mean, n_mangled)


def settling_tolerance(link):
    """Counts of movement to accept as 'at rest'.

    With a factor known this is just the usual 0.05 g. Without one -- a first
    calibration -- there is no way to express 0.05 g in counts, so the cell's
    own noise at rest is measured and used instead. Assuming a number here
    would be assuming a cell.
    """
    if link.scale:
        return None
    print("  No calibration yet: measuring the cell's noise floor to set the")
    print("  settling tolerance (3 s, do not touch the stand)...")
    noise = measure_noise_floor(link)
    if not noise:
        print("  [WARN] could not measure it; settling checks will be relative only.")
        return None
    print("  noise floor %.0f counts peak-to-peak -> tolerance %.0f counts\n"
          % (noise, 2.0 * noise))
    return 2.0 * noise


def run_masscheck_unload(link, masses, descend):
    """Verification for stands where thrust UNLOADS the cell.

    With the motor pointing up, thrust reduces the cell's load. Placing weights
    therefore exercises the opposite direction from the one that matters.
    Loading with ballast, taring, then removing weights reproduces thrust: each
    removal unloads the cell by a known amount and should read positive.
    """
    total = sum(masses)
    tol = settling_tolerance(link)
    print("\nLoad cell check, unloading geometry (thrust unloads the cell).")
    print("Removing ballast reproduces thrust: each removal should read POSITIVE.\n")
    input("  Place ALL masses on the motor (%.2f g total), then press Enter: " % total)

    # Tare only once the ballast has come to rest. Taring mid-transient offsets
    # every later point by the same amount, and unlike a slope error that is
    # invisible in the residuals -- the line stays straight, just displaced.
    if not wait_stable(link, tol_counts=tol):
        print("     *** stand still drifting; taring anyway, but the zero point")
        print("         below will show how much of an offset that left.")
    link.write_line("TARE")
    time.sleep(1.5)
    print("     tared with %.2f g ballast\n" % total)

    points = []
    link.partial_rows = points
    removed = 0.0
    t0 = time.time()

    # Record the freshly-tared, fully-ballasted state as the run's zero. Nothing
    # is moved for it, but without it the fit has no measured zero to reference
    # against -- every mass_g in this procedure is a removal -- so a tare offset
    # would tilt the slope instead of being subtracted out.
    print("  Zero (nothing removed yet), measuring...")
    status = wait_stable(link, tol_counts=tol)
    settled = status == "settled"
    g, sd, n, raw, n_mangled = read_thrust_avg(link)
    points.append({"t_s": round(time.time() - t0, 1), "mass_g": 0.0,
                   "reading_g": round(g, 4), "reading_sd_g": round(sd, 4),
                   "reading_raw": round(raw, 1), "n_mangled": n_mangled,
                   "n": n, "direction": "up", "settled": int(settled),
                   "settle_status": status})
    if link.scale:
        print("     read %8.3f g   (sd %.3f, n=%2d)   error %+.3f g%s"
              % (g, sd, n, g, "   *** tare offset, not a measurement error"
                 if abs(g) > 0.15 else ""))
    else:
        print("     read %+10.0f counts   (n=%2d)   uncalibrated" % (raw, n))

    steps = [(m, "up") for m in masses]
    if descend:
        # Put the weights back in reverse order of removal, so the down leg
        # revisits the same cumulative removals the up leg measured. Anything
        # else lands on masses the up leg never visited and no hysteresis can
        # be computed at all.
        steps += [(m, "down") for m in reversed(masses)]

    for m, direction in steps:
        if direction == "up":
            removed += m
            prompt = "  Remove %.2f g (cumulative %.2f g)" % (m, removed)
        else:
            removed -= m
            prompt = "  Put back %.2f g (cumulative removed %.2f g)" % (m, removed)
        input("%s, then press Enter: " % prompt)
        status = wait_stable(link, tol_counts=tol)
        settled = status == "settled"
        g, sd, n, raw, n_mangled = read_thrust_avg(link)
        points.append({"t_s": round(time.time() - t0, 1), "mass_g": round(removed, 3),
                       "reading_g": round(g, 4), "reading_sd_g": round(sd, 4),
                       "reading_raw": round(raw, 1), "n_mangled": n_mangled,
                       "n": n, "direction": direction, "settled": int(settled),
                       "settle_status": status})
        note = ""
        if not settled:
            note = "   *** " + UNSETTLED_NOTE[status]
        elif sd > max(0.05, 0.005 * abs(g)):
            note = "   *** unstable"
        # Appended, not exclusive: a point can be both unsettled and have had a
        # transfer rejected, and a calibration point that lost samples to the
        # bus is worth seeing even when everything else about it was fine.
        if n_mangled:
            note += "   *** %d corrupt transfer(s) rejected" % n_mangled
        if link.scale:
            print("     read %8.3f g   (sd %.3f, n=%2d)   error %+.3f g%s"
                  % (g, sd, n, g - removed, note))
        else:
            # No factor yet, so grams would all read zero. Counts are the only
            # honest feedback during a first calibration.
            print("     read %+10.0f counts   (n=%2d)   uncalibrated%s" % (raw, n, note))
    return points


def run_masscheck(link, masses, descend, expected=1.0):
    """Multi-point load cell verification against known masses.

    A single-point check confirms the scale factor at one load. It says nothing
    about linearity, which is what matters once thrust moves outside the range
    the factor was originally derived at.

    `expected` is the sign of the reading an applied mass produces: +1 for a
    cell that loads with thrust, -1 when thrust unloads it (--thrust-unloads).
    It only affects the per-point "error" display; the fit derives its own
    slope from raw counts and reports against its own measured zero.
    """
    tol = settling_tolerance(link)
    print("\nLoad cell check. Apply each mass ALONG THE THRUST AXIS -- the way the")
    print("prop pulls. A cell calibrated in one axis reads differently in another.\n")

    sequence = [0.0] + list(masses)
    if descend:
        sequence += list(reversed(masses[:-1])) + [0.0]

    points = []
    link.partial_rows = points
    t0 = time.time()
    for i, m in enumerate(sequence):
        direction = "down" if descend and i > len(masses) else "up"
        prompt = "  Remove all masses" if m == 0 else "  Apply %.2f g" % m
        input("%s, then press Enter: " % prompt)
        status = wait_stable(link, tol_counts=tol)
        settled = status == "settled"
        g, sd, n, raw, n_mangled = read_thrust_avg(link)
        points.append({"t_s": round(time.time() - t0, 1), "mass_g": m,
                       "reading_g": round(g, 4), "reading_sd_g": round(sd, 4),
                       "reading_raw": round(raw, 1), "n_mangled": n_mangled,
                       "n": n, "direction": direction, "settled": int(settled),
                       "settle_status": status})
        note = ""
        if not settled:
            note = "   *** " + UNSETTLED_NOTE[status]
        elif sd > max(0.05, 0.005 * abs(g)):
            note = "   *** unstable (sd high for this load)"
        # Appended, not exclusive: a point can be both unsettled and have had a
        # transfer rejected, and a calibration point that lost samples to the
        # bus is worth seeing even when everything else about it was fine.
        if n_mangled:
            note += "   *** %d corrupt transfer(s) rejected" % n_mangled
        if link.scale:
            print("     read %8.3f g   (sd %.3f, n=%2d)   error %+.3f g%s"
                  % (g, sd, n, g - expected * m, note))
        else:
            print("     read %+10.0f counts   (n=%2d)   uncalibrated%s" % (raw, n, note))
    return points


def warn_if_partial(path):
    """Warn when `path` holds a run that stopped early. True if it did.

    An aborted run keeps the rows it collected, so it stays readable and is
    worth re-examining. What it is not is a complete protocol: a sweep can be
    missing its whole down leg, a MASSCHECK half its masses. Anything that
    fits, rescales or plots one has to say so rather than presenting it as a
    finished run. Files written before this marker existed report nothing,
    which is the honest answer for them.
    """
    status, reason = None, ""
    with open(path) as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            if line.startswith("# result,"):
                head, _, rest = line[len("# result,"):].strip().partition(",")
                status = head.partition("=")[2]
                reason = rest.partition("=")[2]
    if status != "aborted":
        return False
    print("[WARN] %s is a PARTIAL run (%s). It holds only the rows collected "
          "before the run stopped -- do not read it as a full protocol."
          % (os.path.basename(path), reason or "no reason recorded"),
          file=sys.stderr)
    return True


def read_kv_csv(path):
    """Points from a saved KV run, numeric, top DShot level only.

    Only the highest setpoint is returned because pairing assumes duty 1, where
    the motor sees the bus voltage. A part-throttle point has an unknown duty
    and its V_motor is not the recorded bus voltage, so mixing them in would
    quietly bias every pairing.
    """
    warn_if_partial(path)
    with open(path) as fh:
        lines = fh.readlines()
    rows = list(csv.DictReader([l for l in lines if not l.startswith("#")]))
    if not rows:
        sys.exit("%s has no data rows" % path)

    points = []
    for row in rows:
        try:
            point = {
                "v_set": float(row.get("v_set") or 0),
                "dshot": int(float(row.get("dshot") or 2000)),
                "volts_mean": float(row["volts_mean"]),
                "amps_mean": float(row["amps_mean"]),
                "rpm_mean": float(row["rpm_mean"]),
                "n_rpm": int(float(row.get("n_rpm") or 0)),
                "n_ina": int(float(row.get("n_ina") or 0)),
                "flags": int(float(row.get("flags") or 0)),
                "flag_names": row.get("flag_names", ""),
            }
        except (KeyError, ValueError):
            sys.exit("%s does not look like a KV run (missing v_set/volts/amps/rpm)"
                     % path)
        points.append(point)

    top = max(p["dshot"] for p in points)
    kept = [p for p in points if p["dshot"] == top]
    dropped = len(points) - len(kept)
    if dropped:
        print("  %s: using %d points at dshot %d, ignoring %d at lower setpoints "
              "(pairing needs duty 1)" % (os.path.basename(path), len(kept), top,
                                          dropped))
    return kept


def flag_speed_limited(points):
    """Mark no-load points where speed stopped rising with voltage.

    An ESC that has run out of commutation headroom holds the motor at a
    ceiling while the bus keeps climbing, and every such point still looks
    entirely plausible on its own -- the measured symptom is that Kv computed
    from it falls steadily as voltage rises. Comparing the local dRPM/dV
    against the best seen in the series catches it without needing to know the
    ESC's limit in advance.
    """
    ordered = sorted(points, key=lambda p: p["volts_mean"])
    slopes = []
    for a, b in zip(ordered, ordered[1:]):
        dv = b["volts_mean"] - a["volts_mean"]
        slopes.append((b, (b["rpm_mean"] - a["rpm_mean"]) / dv if dv > 1e-9 else 0.0))
    if not slopes:
        return
    best = max(s for _, s in slopes)
    for point, slope in slopes:
        # Well under half the healthy slope means the motor is no longer
        # following the voltage. Kv from such a point is meaningless.
        if best > 0 and slope < 0.4 * best:
            point["speed_limited"] = True


def solve_kv_pair(loaded, unloaded):
    """Kv and R from two points at duty 1: same model, very different current.

    RPM1 = Kv*(V1 - I1*R) and RPM2 = Kv*(V2 - I2*R), solved exactly. This is the
    only well-conditioned way to separate the two on this bench: a voltage sweep
    at a fixed prop keeps V and I collinear, so the load has to change instead.
    """
    n1, n2 = loaded["rpm_mean"], unloaded["rpm_mean"]
    if n2 <= 0:
        return None
    k = n1 / n2
    denom = loaded["amps_mean"] - k * unloaded["amps_mean"]
    if abs(denom) < 1e-9:
        return None
    r_ohm = (loaded["volts_mean"] - k * unloaded["volts_mean"]) / denom
    back_emf = loaded["volts_mean"] - loaded["amps_mean"] * r_ohm
    if back_emf <= 0:
        return None
    return {"r_ohm": r_ohm, "kv_rpm_per_v": n1 / back_emf,
            "delta_i": loaded["amps_mean"] - unloaded["amps_mean"]}


def report_kv_pair(loaded_path, noload_path, min_delta_i=0.5):
    """Pair a loaded KV run against a no-load one and report Kv and R."""
    print("\nPairing %s (loaded)\n    against %s (no load)"
          % (loaded_path, noload_path))
    loaded = read_kv_csv(loaded_path)
    unloaded = read_kv_csv(noload_path)

    for group in (loaded, unloaded):
        for point in group:
            ok, why = usable_point(point)
            if not ok:
                point["skip"] = why
    # Only over points that ran. A failed start sits at a few hundred RPM, so
    # the step up to the first real point looks like an enormous dRPM/dV and
    # makes every genuine slope look flat by comparison.
    flag_speed_limited([p for p in unloaded if "skip" not in p])

    usable_loaded = [p for p in loaded if "skip" not in p]
    usable_unloaded = [p for p in unloaded
                       if "skip" not in p and not p.get("speed_limited")]

    for point in loaded + unloaded:
        if "skip" in point:
            print("  skipped %.2f V: %s" % (point["v_set"], point["skip"]))
    for point in unloaded:
        if point.get("speed_limited"):
            print("  skipped %.2f V (no load): speed stopped rising with voltage "
                  "-- ESC limited, Kv from it would be meaningless"
                  % point["v_set"])

    if not usable_loaded or not usable_unloaded:
        print("\n[WARN] nothing left to pair.")
        return

    results = []
    for point in usable_unloaded:
        # Nearest in bus voltage: the closer the two sit, the more purely the
        # pair differs in current, which is the axis being exploited.
        partner = min(usable_loaded,
                      key=lambda p: abs(p["volts_mean"] - point["volts_mean"]))
        solved = solve_kv_pair(partner, point)
        if solved is None:
            print("  %.2f V: degenerate pairing, skipped" % point["v_set"])
            continue
        if abs(solved["delta_i"]) < min_delta_i:
            print("  %.2f V: only %.2f A of current separation, skipped "
                  "(needs >= %.1f A to separate Kv from R)"
                  % (point["v_set"], abs(solved["delta_i"]), min_delta_i))
            continue
        solved["unloaded"], solved["loaded"] = point, partner
        # How far apart the pair sits in bus voltage. The solve assumes both
        # points share one Kv and one R; R is known to vary with operating
        # point, so a badly matched pair carries that variation into the
        # answer. Reported rather than filtered on: a poorly matched pairing is
        # weak evidence, not bad data, and the cure is to record the two runs at
        # matching voltages rather than to discard what disagrees.
        solved["v_mismatch"] = abs(partner["volts_mean"] - point["volts_mean"]) \
            / point["volts_mean"]
        results.append(solved)

    if not results:
        print("\n[WARN] no usable pairings.")
        return

    print("\n%-24s %-24s %-8s %-7s %-9s %-8s"
          % ("no load (V, A, RPM)", "loaded (V, A, RPM)", "dI (A)", "dV %",
             "R ohm", "Kv"))
    print("-" * 86)
    for res in results:
        u, l = res["unloaded"], res["loaded"]
        print("%-24s %-24s %-8.2f %-7.1f %-9.4f %-8.0f"
              % ("%.3f %.3f %.0f" % (u["volts_mean"], u["amps_mean"], u["rpm_mean"]),
                 "%.3f %.3f %.0f" % (l["volts_mean"], l["amps_mean"], l["rpm_mean"]),
                 res["delta_i"], 100 * res["v_mismatch"], res["r_ohm"],
                 res["kv_rpm_per_v"]))

    worst = max(results, key=lambda r: r["v_mismatch"])
    if worst["v_mismatch"] > 0.05:
        print("\n  [WARN] worst voltage match is %.1f%%. The solve assumes both"
              % (100 * worst["v_mismatch"]))
        print("         points share one Kv and one R, and R varies with operating")
        print("         point, so a badly matched pair carries that variation into")
        print("         the answer. Record the no-load run at the same setpoints as")
        print("         the loaded one rather than dropping the pairing -- the cure")
        print("         is matched data, not a shorter list.")

    kvs = [r["kv_rpm_per_v"] for r in results]
    rs = [r["r_ohm"] for r in results]
    mean_kv = sum(kvs) / len(kvs)
    mean_r = sum(rs) / len(rs)
    spread_kv = max(kvs) - min(kvs)
    spread_r = max(rs) - min(rs)
    print("\n  Kv = %.0f RPM/V   spread %.0f over %d pairings (%.2f%%)"
          % (mean_kv, spread_kv, len(results), 100 * spread_kv / mean_kv))
    print("  R  = %.0f mOhm     spread %.0f mOhm (%.1f%%)"
          % (mean_r * 1000, spread_r * 1000, 100 * spread_r / mean_r))

    # Kv is a motor constant and should hold across the pairings; R lumps ESC
    # conduction, iron and windage losses that do not scale as I*R, so it is a
    # local value. Saying which is which is the point of reporting both spreads.
    if spread_kv / mean_kv > 0.02:
        print("\n  [WARN] Kv varies more than 2% across pairings. It is a motor")
        print("         constant, so that is a measurement problem rather than a")
        print("         property -- suspect a point near the ESC's start or speed")
        print("         limit, or a thermal drift between the two runs.")
    if spread_r / mean_r > 0.05:
        print("\n  R is not constant across the pairings, which is expected. It is")
        print("  an effective resistance, not a DC one: the fit uses bus-average")
        print("  current while copper loss follows winding RMS, so R carries a")
        print("  waveform factor that itself depends on commutation and load.")
        print("  Quote it with the current it was measured at, and do not")
        print("  extrapolate it to full throttle.")


def rescale_csv(path, factor):
    """Re-derive thrust in a saved run under a different calibration factor.

    This is what the raw counts are for. A run taken under a factor that later
    turns out to be wrong is not lost: every thrust column is recomputed from
    the counts the sensor actually produced, and the file records both the old
    and the new factor so the correction is auditable.
    """
    warn_if_partial(path)
    header, rows, fieldnames = [], [], None
    with open(path) as fh:
        lines = fh.readlines()
    header = [l for l in lines if l.startswith("#")]
    body = [l for l in lines if not l.startswith("#")]
    reader = csv.DictReader(body)
    fieldnames = reader.fieldnames or []
    rows = list(reader)

    raw_cols = [c for c in ("thrust_raw_mean", "thrust_raw") if c in fieldnames]
    if not raw_cols:
        print("%s has no raw thrust column, so it cannot be rescaled.\n"
              "The run has only scaled grams, not the counts behind them." % path)
        return 1
    raw_col = raw_cols[0]
    old = None
    for line in header:
        if line.startswith("# calibration"):
            for item in line.split(","):
                k, _, v = item.partition("=")
                if k.strip() == "hx711_scale":
                    try:
                        old = float(v)
                    except ValueError:
                        pass

    changed = 0
    for row in rows:
        try:
            raw = float(row[raw_col])
        except (TypeError, ValueError):
            continue
        grams = raw / factor
        for col, digits in (("thrust_g_mean", 3), ("thrust_g", 3)):
            if col in row and row[col] not in ("", None):
                row[col] = round(grams, digits)
                changed += 1
        # Anything derived from thrust is now stale rather than merely wrong.
        for col in ("thrust_g_iso", "fom", "eff_g_per_w", "thrust_g_std"):
            if col in row:
                row[col] = ""

    out = os.path.splitext(path)[0] + ".rescaled.csv"
    with open(out, "w", newline="") as fh:
        for line in header:
            fh.write(line)
        fh.write("# rescaled,from=%s,to=%.4f,source=%s\n"
                 % ("%.4f" % old if old else "unknown", factor, os.path.basename(path)))
        writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore", restval="")
        writer.writeheader()
        writer.writerows(rows)
    print("Rescaled %d thrust values in %d rows" % (changed, len(rows)))
    print("  from %s to %.4f counts/g" % ("%.4f" % old if old else "unknown", factor))
    print("  wrote %s" % out)
    print("\n  Efficiency, density-normalised thrust and figure of merit were")
    print("  blanked: they are derived from thrust and must be recomputed.")
    return 0


def refit_masscheck(path, expected_slope):
    """Re-run the fit on a saved MASSCHECK CSV, no hardware needed.

    Declaring the geometry wrong costs nothing but the flag: the readings are
    already recorded, so the fit can be redone against the other sign rather
    than re-weighing everything. Also useful for re-examining an old run after
    the factor has moved.
    """
    warn_if_partial(path)
    rows, factor = [], None
    with open(path) as handle:
        for line in handle:
            if not line.startswith("#"):
                continue
            if line.startswith("# scale"):
                for item in line.split(","):
                    key, _, value = item.strip().partition("=")
                    if key == "factor":
                        try:
                            factor = float(value)
                        except ValueError:
                            pass
    with open(path) as handle:
        body = [l for l in handle if not l.startswith("#")]
    for raw in csv.DictReader(body):
        if "mass_g" not in raw or "reading_g" not in raw:
            print("%s is not a MASSCHECK run (no mass_g/reading_g columns)" % path)
            return 1
        row = {"mass_g": float(raw["mass_g"]), "reading_g": float(raw["reading_g"]),
               "reading_sd_g": float(raw.get("reading_sd_g") or 0.0),
               "n": int(raw.get("n") or 0), "direction": raw.get("direction", "up"),
               "settled": int(raw.get("settled") or 1),
               "settle_status": raw.get("settle_status")
               or ("settled" if int(raw.get("settled") or 1) else "drifting")}
        # Runs recorded before t_s existed simply cannot have drift separated
        # from hysteresis; the fit degrades to reporting the raw gap.
        if raw.get("t_s"):
            row["t_s"] = float(raw["t_s"])
        rows.append(row)
    print("Refitting %d points from %s" % (len(rows), path))
    print("  recorded factor %s, expected slope %+.0f"
          % ("%.4f" % factor if factor else "unknown", expected_slope))
    report_masscheck(rows, fit_masscheck(rows, factor, expected_slope))
    return 0


def fit_masscheck(points, current_factor, expected_slope=1.0):
    """Derive the calibration factor from known masses.

    The factor is fitted from RAW COUNTS, so this works with no prior
    calibration -- which a first run on a new stand necessarily has. The
    quality figures below (linearity, hysteresis, drift) are then expressed in
    grams under the factor just derived, and the existing factor, if there is
    one, is reported only as an error against it.

    expected_slope is +1 when an applied mass loads the cell the same way
    thrust does, and -1 when the motor points up so thrust unloads it. It sets
    the sign of the factor, and is an assertion about the mechanics that the
    mass data cannot verify on a first calibration.
    """
    pts = [p for p in points if p["n"] > 0]
    if len(pts) < 3 or not all("reading_raw" in p for p in pts):
        return None

    # Counts per gram, through the run's own measured zero.
    raw_zeros = [p["reading_raw"] for p in pts if p["mass_g"] == 0]
    raw_ref = sum(raw_zeros) / len(raw_zeros) if raw_zeros else 0.0
    sxx_r = sum(p["mass_g"] ** 2 for p in pts)
    sxy_r = sum(p["mass_g"] * (p["reading_raw"] - raw_ref) for p in pts)
    if sxx_r <= 0:
        return None
    counts_per_g = sxy_r / sxx_r
    if abs(counts_per_g) < 1e-9:
        return None
    suggested = counts_per_g / expected_slope

    # Quality figures in grams must be expressed under the factor just derived
    # -- that is what makes linearity/hysteresis/drift meaningful on a first
    # calibration -- but the caller's points are not to be touched. The saved
    # CSV's reading_g column must stay under the factor its own
    # `# calibration,hx711_scale=` header declares, or the file contradicts
    # itself. Fit on a re-expressed copy instead.
    pts = [dict(p, reading_g=p["reading_raw"] / suggested) for p in pts]
    n = len(pts)
    sx = sum(p["mass_g"] for p in pts)
    sy = sum(p["reading_g"] for p in pts)
    sxx = sum(p["mass_g"] ** 2 for p in pts)
    sxy = sum(p["mass_g"] * p["reading_g"] for p in pts)
    det = n * sxx - sx * sx
    if abs(det) < 1e-12:
        return None
    a = (n * sxy - sx * sy) / det
    b = (sy * sxx - sx * sxy) / det
    resid = [p["reading_g"] - (a * p["mass_g"] + b) for p in pts]
    span = max(p["mass_g"] for p in pts) or 1.0
    # Sensitivity is measured against the run's own zero, not against an assumed
    # zero: a tare that has drifted would otherwise tilt the line. Referencing
    # first, then forcing through the origin, keeps curvature out of the slope.
    zeros = [p["reading_g"] for p in pts if p["mass_g"] == 0]
    zero_ref = sum(zeros) / len(zeros) if zeros else 0.0
    sxy = sum(p["mass_g"] * (p["reading_g"] - zero_ref) for p in pts)
    sxx2 = sum(p["mass_g"] ** 2 for p in pts)
    a0 = (sxy / sxx2) if sxx2 > 0 else a
    resid0 = [(p["reading_g"] - zero_ref) - a0 * p["mass_g"] for p in pts]

    ratio = a0 / expected_slope
    out = {"slope": a0, "slope_free": a, "expected_slope": expected_slope,
           "ratio": ratio, "offset_g": b, "zero_ref_g": zero_ref,
           "max_resid_g": max(abs(r) for r in resid0),
           "linearity_pct_fs": 100 * max(abs(r) for r in resid0) / span,
           "counts_per_g": counts_per_g,
           "suggested_factor": suggested,
           "current_factor": current_factor,
           "n_points": n}
    if current_factor:
        # How far the factor in force was off. Positive means it over-read.
        out["scale_error"] = suggested / current_factor - 1.0
        out["sign_flip"] = (suggested < 0) != (current_factor < 0)
    # Up leg vs down leg at the same mass. This gap is only hysteresis if the
    # zero held still: a zero that walks during the run shifts every down-leg
    # point by the same amount and masquerades as load-dependent behaviour.
    # Real hysteresis grows with load; drift does not, so subtracting a linear
    # zero trend separates them -- but only when the points carry timestamps.
    ups = {p["mass_g"]: p["reading_g"] for p in pts if p["direction"] == "up"}
    downs = {p["mass_g"]: p["reading_g"] for p in pts if p["direction"] == "down"}
    common = set(ups) & set(downs)
    if common:
        out["max_gap_g"] = max(abs(downs[m] - ups[m]) for m in common)

    zeros = [p for p in pts if p["mass_g"] == 0]
    if len(zeros) >= 2:
        out["zero_drift_g"] = zeros[-1]["reading_g"] - zeros[0]["reading_g"]

    if common and all("t_s" in p for p in pts) and len(zeros) >= 2:
        span_s = zeros[-1]["t_s"] - zeros[0]["t_s"]
        if span_s > 0:
            rate = out["zero_drift_g"] / span_s
            base = zeros[0]["reading_g"] - rate * zeros[0]["t_s"]
            corr = {}
            for p in pts:
                corr.setdefault(p["direction"], {})[p["mass_g"]] = \
                    p["reading_g"] - (base + rate * p["t_s"])
            out["max_hysteresis_g"] = max(
                abs(corr["down"][m] - corr["up"][m]) for m in common)
            out["drift_rate_g_min"] = rate * 60.0
    return out


def report_masscheck(points, fit):
    unsettled = [p for p in points if not p.get("settled", 1)]
    if unsettled:
        print("\n*** %d of %d points failed the stability check, so the factor below"
              % (len(unsettled), len(points)))
        print("    is PROVISIONAL and has not been saved.")
        drifting = [p for p in unsettled if p.get("settle_status") != "noisy"]
        noisy = [p for p in unsettled if p.get("settle_status") == "noisy"]
        if drifting:
            print("    %d still drifting: the stand had not come to rest. Re-run and"
                  % len(drifting))
            print("      wait longer before pressing Enter." )
        if noisy:
            print("    %d too noisy (%s): the spread is the sensor's own noise at that"
                  % (len(noisy),
                     ", ".join("%g g" % abs(p["mass_g"]) for p in noisy)))
            print("      load, not a transient, so waiting longer cannot help. Drop")
            print("      those masses from --masses, or change the ADC setting.")
    if not fit:
        print("\n[WARN] not enough points to fit")
        return
    if fit.get("sign_flip"):
        print("\n*** The derived factor (%.4f) has the opposite sign to the one in"
              % fit["suggested_factor"])
        print("    force (%.4f). Either the geometry flag differs from last time"
              % fit["current_factor"])
        print("    (--thrust-unloads), or the cell has been remounted the other way.")
        print("    Resolve that before accepting this factor -- a sign error reads")
        print("    masses correctly and thrust backwards.\n")

    initial = not fit.get("current_factor")
    print("\n%s: %.4f counts/g   (through origin, %d points, expected slope %+.0f)"
          % ("Initial calibration" if initial else "Fit",
             fit["suggested_factor"], fit["n_points"], fit["expected_slope"]))
    print("      referenced to measured zero of %+.4f g%s"
          % (fit["zero_ref_g"],
             "  <-- tare drifted, worth re-taring" if abs(fit["zero_ref_g"]) > 0.15 else ""))
    if initial:
        print("  The sign follows from the geometry you declared, which the masses")
        print("  cannot check. Verify it with a brief spin: thrust must read POSITIVE.")
    else:
        err = fit["scale_error"]
        print("  scale error   %+.2f%%   %s   (factor in force was %.4f)"
              % (100 * err, "good" if abs(err) < 0.01 else "worth re-deriving",
                 fit["current_factor"]))
    print("  linearity     %.3f g worst deviation (%.2f%% of full scale)"
          % (fit["max_resid_g"], fit["linearity_pct_fs"]))
    if "max_gap_g" in fit:
        print("  up/down gap   %.3f g between legs at the same mass" % fit["max_gap_g"])
    if "zero_drift_g" in fit:
        print("  zero drift    %+.3f g from first zero point to last%s"
              % (fit["zero_drift_g"],
                 " (%+.3f g/min)" % fit["drift_rate_g_min"]
                 if "drift_rate_g_min" in fit else ""))
    if "max_hysteresis_g" in fit:
        # The gap minus the zero trend. Quoting the raw gap as hysteresis blames
        # the cell for what is usually the mount settling or warming.
        print("  hysteresis    %.3f g once that drift is removed -- %s"
              % (fit["max_hysteresis_g"],
                 "genuine load hysteresis" if fit["max_hysteresis_g"] > 0.5 *
                 fit.get("max_gap_g", 0) else "the gap above is mostly drift, not the cell"))
    elif "max_gap_g" in fit:
        print("                (no timestamps in this run, so drift and hysteresis")
        print("                 cannot be separated -- re-run to get t_s)")



# t_s is what separates zero drift from load hysteresis after the fact: without
# it, a zero that walks during the run is indistinguishable from a cell that
# reads differently loading and unloading, and the two have different fixes.
MASS_FIELDS = ["t_s", "mass_g", "reading_g", "reading_raw", "reading_sd_g",
               "n", "n_mangled", "direction", "settled", "settle_status"]


# =========================================================================
# Output
# =========================================================================

def check_rotation_sign(rows, rotation):
    """Cheap bench-side catch for a wrong --reverse/--prop-hand.

    The consuming site fails its build when a declared rotation disagrees with
    the sign the load cell actually recorded -- this is the same check, run
    here so a mismatch is a line printed at the bench instead of a build
    failure found a day later. Not a MASSCHECK check: that mode has its own
    sign convention via --thrust-unloads/--ballast, unrelated to rotation.
    """
    if not rows:
        return
    key = "thrust_g_mean" if "thrust_g_mean" in rows[0] else (
          "thrust_g" if "thrust_g" in rows[0] else None)
    if key is None:
        return
    peak = max(rows, key=lambda r: abs(float(r.get(key) or 0.0)))
    val = float(peak.get(key) or 0.0)
    if abs(val) < 1.0:
        return  # too small to mean anything against noise
    negative = val < 0
    if negative != (rotation == "reversed"):
        print("\n[WARN] thrust was %s while rotation=%s (peak %.2f g) -- "
              "check --reverse/--prop-hand"
              % ("negative" if negative else "positive", rotation, val),
              file=sys.stderr)


def write_csv(path, rows, fields, link, mode, notes, extra_meta=None):
    # data/ is gitignored, so it does not exist in a fresh clone; the documented
    # quick-start writes there.
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", newline="") as fh:
        fh.write("# schema,%d\n" % SCHEMA_VERSION)
        fh.write("# run,utc=%s,mode=%s\n"
                 % (datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"), mode))
        if notes:
            fh.write("# notes,%s\n" % notes.replace("\n", " "))
        # The factor that turned raw counts into the grams below. Firmware no
        # longer knows it, so the run has to carry it or the numbers are
        # uninterpretable -- and --rescale needs it to report what changed.
        fh.write("# calibration,hx711_scale=%s,source=host\n"
                 % ("%.4f" % link.scale if link.scale else "none"))
        for key, value in (extra_meta or {}).items():
            fh.write("# %s,%s\n" % (key, value))
        # "stats" carries the telemetry link health for the run. Without it the
        # CSV records what produced the numbers but not whether the ESC link was
        # working, which is the difference between a real esc_stress column and
        # a frozen one.
        for key in ("fw", "board", "esc", "scale", "ina", "flags", "phase",
                    "ambient", "stats", "warn", "error"):
            # Multi-valued keys accumulate legitimately (ambient banner/start/end),
            # but an ID reprint re-emits the banner line, so drop exact repeats
            # rather than writing the same reading into the header twice.
            seen = set()
            for value in link.meta.get(key, []):
                if value in seen:
                    continue
                seen.add(value)
                fh.write("# %s,%s\n" % (key, value))
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore", restval="")
        writer.writeheader()
        writer.writerows(rows)
    print("\nSaved %d rows to %s" % (len(rows), path))


def report(rows, mode, link, detail_codes=False, step_levels=None):
    if not rows:
        return

    flagged = [r for r in rows if r["flags"]]
    if flagged:
        print("\n*** %d of %d rows carry sensor flags ***" % (len(flagged), len(rows)))
        seen = {}
        for r in flagged:
            seen[r["flag_names"]] = seen.get(r["flag_names"], 0) + 1
        for name, count in sorted(seen.items(), key=lambda kv: -kv[1]):
            print("      %-40s %d rows" % (name, count))
        print("    Saturated or stale samples are NOT valid measurements.")

    if mode in ("TRANSIENT", "RESPONSE", "COASTDOWN"):
        fresh = sum(r["new_thrust"] for r in rows)
        print("\n%d rows printed, %d carry a fresh thrust sample (%.0f Hz effective)"
              % (len(rows), fresh, fresh / max(rows[-1]["time_s"], 1e-6)))
        print("%-10s %-9s %-8s %-11s %-9s %-8s" % ("Time(s)", "Throttle", "RPM", "Thrust(g)", "Volts", "Amps"))
        print("-" * 62)
        step = max(1, len(rows) // 30)
        for r in rows[::step]:
            print("%-10.4f %-9d %-8d %-11.2f %-9.3f %-8.4f"
                  % (r["time_s"], r["throttle_pct"], r["rpm"], r["thrust_g"], r["volts"], r["amps"]))
        # Per-row here rather than per-step: a coastdown holds one throttle the
        # whole way down, so grouping on throttle pools it into a single entry
        # and the value is in the pooled period spread. That is the run that
        # crosses a suspect RPM band with no throttle command at all.
        groups = {}
        for r in rows:
            if not r.get("new_rpm") or r.get("erpm_raw") in (None, ""):
                continue
            counts, rpms = groups.setdefault(r["throttle_pct"], (collections.Counter(), []))
            counts[period_us_from_code(r["erpm_raw"])] += 1
            rpms.append(r["rpm"])
        report_period_codes([("thr %d%%" % t, c, statistics.mean(v) if v else 0.0)
                             for t, (c, v) in sorted(groups.items())], detail_codes)
        if mode in ("RESPONSE", "COASTDOWN"):
            report_step_metrics(step_metrics(rows, step_levels))
        return

    print("\n%-4s %-9s %-16s %-16s %-9s %-9s %-8s %s"
          % ("dir", "Thr(%)", "RPM (+-s, n)", "Thrust g (+-s, n)", "Volts", "Amps", "g/W", "flags"))
    print("-" * 97)
    for r in rows:
        stale = []
        if r.get("n_rpm_stale"):
            stale.append("%d rpm_stale" % r["n_rpm_stale"])
        if r.get("n_thrust_stale"):
            stale.append("%d thrust_stale" % r["n_thrust_stale"])
        # Loud, not silent. A missed conversion is a real sample that no row ever
        # carried, so it cannot be recovered from the CSV afterwards.
        if r.get("n_thrust_missed"):
            stale.append("%d thrust_MISSED" % r["n_thrust_missed"])
        if r.get("n_ina_missed"):
            stale.append("%d ina_MISSED" % r["n_ina_missed"])
        # Rejected as corrupt transfers. Shown per step because the fault is
        # bursty: a rate quoted over a whole run hides that one step lost three
        # samples and its neighbours lost none.
        if r.get("n_thrust_mangled"):
            stale.append("%d thrust_MANGLED" % r["n_thrust_mangled"])
        if r.get("n_ina_mangled"):
            stale.append("%d ina_MANGLED" % r["n_ina_mangled"])
        note = r["flag_names"]
        if stale:
            note = (note + "|" if note else "") + "|".join(stale)
        print("%-4s %-9d %-16s %-16s %-9.3f %-9.4f %-8.2f %s"
              % (r["direction"], r["throttle_pct"],
                 "%d+-%d n=%d" % (r["rpm_mean"], r["rpm_std"], r["n_rpm"]),
                 "%.2f+-%.2f n=%d" % (r["thrust_g_mean"], r["thrust_g_std"], r["n_thrust"]),
                 r["volts_mean"], r["amps_mean"], r["eff_g_per_w"], note))

    steps = []
    for r in rows:
        counts = collections.Counter()
        for item in (r.get("erpm_raw_hist") or "").split("|"):
            code, _, n = item.partition(":")
            if n:
                counts[period_us_from_code(code)] += int(n)
        label = ("%s %d%%" % (r.get("direction", ""), r["throttle_pct"])
                 if "throttle_pct" in r else "%.2f V" % r.get("v_set", 0.0))
        steps.append((label.strip(), counts, r["rpm_mean"]))
    report_period_codes(steps, detail_codes)
    report_missed_samples(rows)
    report_mangled_samples(rows)

    if mode == "SWEEP":
        report_hysteresis(rows)


def main():
    parser = argparse.ArgumentParser(description="Thrust stand host link (schema 1)")
    modes = ["STATIC", "TRANSIENT", "SWEEP", "RESPONSE", "COASTDOWN", "KV", "MASSCHECK",
             "LINKTEST", "FINEWALK"]
    # No default. A default mode turned every mistyped or mode-less invocation
    # into a full sweep: `measure.py --tare` once tared and then swept, at
    # whatever direction and supply voltage the bench had been left in.
    parser.add_argument("-m", "--mode", default=None,
                        choices=modes + [m.lower() for m in modes],
                        help="sequence to run. Omit it and nothing spins: "
                             "--check, --tare and --watch still work on their own")
    parser.add_argument("--codes", action="store_true",
                        help="Print the raw eRPM period each step landed on. The "
                             "resolution of period-based telemetry degrades as RPM^2, "
                             "so this is what says whether a flat patch at the top of "
                             "a sweep is a measurement or the telemetry running out")
    parser.add_argument("--base", type=int, default=40,
                        help="RESPONSE: throttle %% to step from (default 40)")
    parser.add_argument("--high", type=int, default=70,
                        help="RESPONSE: throttle %% to step to (default 70)")
    parser.add_argument("--reps", type=int, default=5,
                        help="RESPONSE: repetitions (default 5)")
    parser.add_argument("--from-throttle", type=int, default=70,
                        help="COASTDOWN: throttle %% to decay from (default 70)")
    parser.add_argument("--masses", default="5,10,20,30,50",
                        help="MASSCHECK: known masses in grams, comma separated")
    parser.add_argument("--thrust-unloads", action="store_true",
                        help="MASSCHECK: motor points up, so thrust UNLOADS the cell and an "
                             "applied mass reads negative. Expected slope becomes -1.")
    parser.add_argument("--ballast", action="store_true",
                        help="MASSCHECK: ballast-removal procedure (needs several weights "
                             "stacked at once); implies --thrust-unloads")
    parser.add_argument("--descend", action="store_true",
                        help="MASSCHECK: also unload in reverse, to measure hysteresis")
    parser.add_argument("--throttles", default="10,20,40,60,80,100",
                        help="LINKTEST: throttles to hold while counting telemetry outcomes")
    parser.add_argument("--dshot-range", default="1900,1990,10", metavar="LO,HI,STEP",
                        help="FINEWALK: raw DShot setpoints to walk, LO,HI,STEP over "
                             "0-2000. Integer percent reaches only 100 of those 2000 "
                             "steps, which near full throttle is coarser than the "
                             "telemetry itself (default 1900,1990,10)")
    parser.add_argument("--walk-reps", type=int, default=3,
                        help="FINEWALK: visits per setpoint. Order is randomised "
                             "across all visits so the reported period's drift with "
                             "running time does not masquerade as a throttle response "
                             "(default 3)")
    parser.add_argument("--voltages", default="3.0,3.4,3.8,4.2",
                        help="KV: supply voltages to step through, comma separated")
    parser.add_argument("--hold-ms", type=int, default=1000,
                        help="KV: logging window per voltage point (default 1000)")
    parser.add_argument("--kv-settle-ms", type=int, default=1500,
                        help="KV: time at the setpoint before logging starts. This is "
                             "motor running time, so with the prop OFF cut it right "
                             "down -- an unloaded rotor is at speed in tens of ms and "
                             "the default is a second of needless overspeed "
                             "(default 1500)")
    parser.add_argument("--kv-dshots", default="2000,1700",
                        help="KV: raw DShot setpoints held at every voltage, highest "
                             "first. Two or more are needed for the fit to be "
                             "determined at all: with one, a fixed prop makes current "
                             "a function of voltage, the regressors are collinear to "
                             "r > 0.99, and Kv trades off against R instead of being "
                             "measured. The highest level is the duty=1 reference; "
                             "lower ones carry a fitted duty (default 2000,1700)")
    parser.add_argument("--psu-port", default=None, metavar="DEV",
                        help="KV: drive a SCPI bench supply on this port instead of "
                             "prompting for each voltage (e.g. /dev/ttyUSB0). Opt-in, "
                             "so emulator runs and manual sweeps are unaffected. "
                             "Disconnect any parallel cell first -- the first step "
                             "checks for one and aborts rather than fitting a slope "
                             "with no voltage span")
    parser.add_argument("--esc-firmware", default=None,
                        help="ESC firmware actually flashed, e.g. 'Bluejay 0.21.0 stock' or "
                             "'Bluejay 0.21.0 + remainder-carry patch'. Overrides stand.json "
                             "and is recorded in the CSV header. Nothing can read this back "
                             "from the ESC: a patched build reports the same version as stock")
    parser.add_argument("--esc-pwm-khz", default=None,
                        help="ESC PWM frequency in kHz, e.g. 96 or 24. Overrides stand.json")
    parser.add_argument("--psu", default=None,
                        help="Bench supply in use. Overrides stand.json")
    parser.add_argument("--psu-voltage", default=None,
                        help="Supply setpoint in volts, as set on the front panel. This is a "
                             "declaration; the rail the motor actually saw is measured per row")
    parser.add_argument("--prop-hand", default=None, choices=("normal", "reversed"),
                        help="Bench-level fact, not a per-run one: which ESC spin direction "
                             "is this prop batch's own design-forward direction. One value "
                             "for the whole prop set in service -- set it in stand.json when "
                             "new props go in, not per run. Overrides stand.json. Combined "
                             "with --reverse to compute the DShot command actually sent: "
                             "esc_dir = prop_hand XOR reverse. Required (no default) before "
                             "any mode but --check will run")
    parser.add_argument("--specimen", default=None,
                        help="Which physical prop was mounted, e.g. '1219s-a' -- matches "
                             "whatever is written on the bag. A model is not a specimen: two "
                             "samples of the same model need this to be told apart. CLI-only, "
                             "no stand.json fallback -- it changes at every prop swap, so a "
                             "persisted value would go stale after the first one. Optional in "
                             "every mode; omit it and assign the specimen later in the run "
                             "record instead")
    parser.add_argument("--edt", action="store_true",
                        help="Ask the ESC for extended telemetry (temperature, stress, "
                             "status). The firmware's own default is "
                             "EDT_REQUEST_DEFAULT in config.h; pass this to force it on "
                             "for one run. The esc_* columns stay present and empty "
                             "when it is off, so runs stay column-compatible")
    parser.add_argument("--no-edt", action="store_true",
                        help="Do not ask for extended telemetry, overriding the "
                             "firmware default for one run. Use it on an ESC that never "
                             "answers: support is an ESC firmware property nothing can "
                             "read back, and asking an ESC that will not answer puts "
                             "edt_stale on every row of every run, which is how flags "
                             "stop being read at all. Not the same as "
                             "tools/fake_stand.py --no-edt, which emulates an ESC "
                             "without EDT support rather than declining to ask. "
                             "Persists until the board reboots or something sends "
                             "EDT,1, so a later run passing no flag inherits it")
    parser.add_argument("--reverse", action="store_true",
                        help="Declare that the prop was spun against its own design "
                             "direction for this run -- not the ESC's spin direction. "
                             "Always recorded, every run: # setup,rotation=reversed with "
                             "this flag, rotation=normal without it. Combined with "
                             "--prop-hand/stand.json's esc_dir_forward to work out which "
                             "way the ESC actually needs to spin (esc_dir = prop_hand XOR "
                             "this flag), sent as Bluejay's temporary spin-direction DShot "
                             "command (not 3D mode -- DShot 0 stays stop, not persisted, "
                             "reverts on ESC power cycle) and recorded as what was "
                             "commanded -- not confirmed, Bluejay does not acknowledge it "
                             "-- in # esc,dir=normal|reversed. Sent every run regardless of "
                             "this flag, so direction is never inherited from a previous "
                             "invocation")
    parser.add_argument("--sps", type=int, default=None, metavar="N",
                        help="Load cell conversion rate for this run, sent as "
                             "SPS,<n>. NAU7802 only (10/20/40/80/320); the "
                             "HX711's rate is the RATE pin and the firmware "
                             "refuses it. Use it to record the same event at "
                             "two rates: a real mount resonance sits at one "
                             "frequency in both, an alias moves.")
    parser.add_argument("--poles", type=int, default=None,
                        help="Rotor magnet count. Overrides stand.json; sent to the firmware "
                             "and recorded in the CSV. Must be even")
    parser.add_argument("--scale", type=float, default=None,
                        help="Load cell factor (raw counts per gram). Overrides stand.json. "
                             "Derive it with -m MASSCHECK rather than guessing")
    parser.add_argument("--no-save", action="store_true",
                        help="MASSCHECK: report the corrected factor without writing it "
                             "to stand.json")
    parser.add_argument("--kv-pair", nargs=2, metavar=("LOADED_CSV", "NOLOAD_CSV"),
                        help="Combine a loaded KV run with a prop-off one and report "
                             "Kv and R, then exit. A loaded voltage sweep alone cannot "
                             "separate them -- at a fixed prop, current is a function "
                             "of voltage, so the regressors are collinear and Kv trades "
                             "off against R. Pairing points at the same bus voltage but "
                             "very different current is what identifies them. Record the "
                             "no-load run at the SAME --voltages as the loaded one: the "
                             "solve assumes both points share one R, so a voltage "
                             "mismatch carries R's variation into the answer")
    parser.add_argument("--rescale", nargs=2, metavar=("CSV", "FACTOR"),
                        help="Re-derive thrust in a saved run under a different calibration "
                             "factor, using the raw counts it recorded, and exit")
    parser.add_argument("--prop-diameter-mm", type=float, default=None,
                        help="Prop disk diameter, enables the actuator-disk QC gate")
    parser.add_argument("--temp-c", type=float, default=None,
                        help="Ambient temperature if no BMP280 is fitted")
    parser.add_argument("--pressure-hpa", type=float, default=None,
                        help="Ambient pressure if no BMP280 is fitted")
    parser.add_argument("-o", "--output", default="thrust_data.csv")
    parser.add_argument("-p", "--port", default=SERIAL_PORT)
    parser.add_argument("--notes", default="",
                        help="Free text recorded in the CSV header (motor, prop, pack, ambient)")
    parser.add_argument("--tare", action="store_true",
                        help="With --check: zero the load cell first, for check-mass verification")
    parser.add_argument("--refit", metavar="CSV",
                        help="Re-fit a saved MASSCHECK CSV under the geometry given by "
                             "--thrust-unloads/--ballast and exit. Recovers a run whose "
                             "geometry was declared wrong, without re-weighing anything")
    parser.add_argument("--watch", type=int, default=0, metavar="SECONDS",
                        help="With --check: watch the zero and the telemetry link for "
                             "this long with the motor stopped, then report drift")
    parser.add_argument("--check", action="store_true",
                        help="Print the firmware banner and ambient reading, then exit "
                             "without spinning the motor")
    parser.add_argument("--no-tare", action="store_true",
                        help="Skip the pre-run tare (default is to re-tare before every run)")
    args = parser.parse_args()

    # Checked before the port is opened, so a typo costs nothing and does not
    # leave a connected board mid-setup.
    if args.edt and args.no_edt:
        parser.error("--edt and --no-edt are contradictory")

    # The prop's own frame, independent of which way the ESC was actually
    # told to spin (that is esc_reversed, computed once connected -- see the
    # --prop-hand block below). Computed here, not inside the try block, so
    # it is available even on a path that never reaches that block.
    rotation = "reversed" if args.reverse else "normal"

    mode = args.mode.upper() if args.mode else None
    link = None
    rows = []
    failure = None
    exit_code = 0
    capture_started = False
    # Chosen from the mode, not from the branch that ran, because a run that
    # fails partway still has to be written out and there is no branch to have
    # set it. One source, so the two cannot drift apart.
    fields = {"TRANSIENT": TRANSIENT_FIELDS, "RESPONSE": TRANSIENT_FIELDS,
              "COASTDOWN": TRANSIENT_FIELDS, "MASSCHECK": MASS_FIELDS,
              "KV": KV_FIELDS, "LINKTEST": LINK_FIELDS,
              "FINEWALK": WALK_FIELDS}.get(mode, STEADY_FIELDS)

    if mode is None and not (args.check or args.tare or args.watch > 0
                             or args.refit or args.rescale or args.kv_pair):
        parser.error("nothing to do: pass -m MODE to run a sequence, or "
                     "--check, --tare or --watch for an action that never "
                     "spins the motor")

    if args.kv_pair:
        return report_kv_pair(args.kv_pair[0], args.kv_pair[1])

    if args.rescale:
        return rescale_csv(args.rescale[0], float(args.rescale[1]))

    if args.refit:
        # Same rule as the live path: ballast removal reads positive, a motor
        # pointing up reads negative, anything else reads positive.
        return refit_masscheck(
            args.refit, -1.0 if (args.thrust_unloads and not args.ballast) else 1.0)

    try:
        print("Connecting to %s..." % args.port)
        link = Link(args.port, BAUD_RATE)
        link.handshake()
        cfg = load_stand_config()
        link.scale = args.scale if args.scale is not None else cfg.get("hx711_scale")
        print("--> firmware: %s" % (link.meta.get("fw", ["?"])[0]))
        if link.scale:
            print("--> scale:    %.4f counts/g (%s)"
                  % (link.scale, "--scale" if args.scale is not None else "stand.json"))
        elif mode != "MASSCHECK":
            print("\n[WARN] No load cell calibration. Thrust will read 0.")
            print("       Run: python measure.py -m MASSCHECK --masses 10,20,30,50")
            print("       See docs/CALIBRATION.md.")
        print("--> ina:      %s" % (link.meta.get("ina", ["?"])[0]))
        # Declared setup, echoed before anything spins. See print_setup_banner.
        setup_entries = setup_provenance(cfg, args)
        # --prop-hand is enum-checked by argparse, but a stand.json value
        # bypasses that, and the site build fails on anything other than
        # exactly "normal" or "reversed" -- so normalise and validate the
        # stand.json path too.
        normed = []
        for field, value, source, label in setup_entries:
            if field == "prop_hand" and value is not None:
                value = str(value).strip().lower()
                if value not in ("normal", "reversed"):
                    sys.exit("prop_hand must be 'normal' or 'reversed' (got %r from %s)"
                              % (value, source))
            normed.append((field, value, source, label))
        setup_entries = normed
        print_setup_banner(setup_entries)
        prop_hand = next((v for f, v, _s, _l in setup_entries if f == "prop_hand"), None)
        # Unlike the other five declared fields, an unset prop_hand refuses to
        # start a real run rather than warning and defaulting: a wrong value
        # here mis-declares an entire session, not one prop, and both this
        # banner warning and the site's sign check only catch it after the
        # runs already exist. --check is exempt -- it never spins the motor
        # or writes a CSV, so there is nothing for a wrong default to corrupt.
        if prop_hand is None and not args.check and mode is not None:
            sys.exit("esc_dir_forward is not set in stand.json and --prop-hand was "
                      "not passed. This is a one-time, bench-level setting -- set it "
                      "once before a session, not per run. See README.md.")
        # Before the --check branch: that branch returns, and a pole count that
        # only applied to measurement runs would make `--check --poles N` print
        # a banner contradicting the run that follows.
        poles = args.poles if args.poles is not None else cfg.get("motor_poles")
        if poles is not None:
            # It changes what every RPM in the run means, and the banner reprint
            # is what lands the value in the CSV.
            link.write_line("POLES,%d" % poles)
            time.sleep(0.3)
            link.write_line("ID")
            time.sleep(0.5)
            deadline = time.time() + 2.0
            while time.time() < deadline:
                line = link.readline()
                if line and line.startswith("#"):
                    link.absorb_meta(line)
                    if line.startswith("#ERROR,poles"):
                        sys.exit("firmware rejected poles=%d (%s)" % (poles, line))
            print("--> poles:    %s" % (link.meta.get("esc", ["?"])[-1]))

        # Same shape as poles, and before --check returns for the same reason:
        # the rate changes what every thrust number means, and the banner
        # reprint is what lands it in the CSV.
        if args.sps is not None:
            link.write_line("SPS,%d" % args.sps)
            time.sleep(0.3)
            link.write_line("ID")
            time.sleep(0.5)
            deadline = time.time() + 2.0
            while time.time() < deadline:
                line = link.readline()
                if line and line.startswith("#"):
                    link.absorb_meta(line)
                    # Two refusals, two different problems. Reporting the
                    # rate as wrong when the ADC never initialised sends you to
                    # check a number that was fine.
                    if line.startswith("#ERROR,sps_unavailable"):
                        sys.exit("the load cell ADC did not initialise, so the "
                                 "rate cannot be changed (%s).\nThe board is "
                                 "running and the part is not converting: check "
                                 "its supply first.\nOn this bench that was VIN "
                                 "left unconnected, with the part still "
                                 "answering on the bus." % line)
                    if line.startswith("#ERROR,sps"):
                        sys.exit("firmware rejected sps=%d (%s) -- the NAU7802 "
                                 "takes 10/20/40/80/320 and the HX711 takes "
                                 "none of them" % (args.sps, line))
            print("--> rate:     %s" % (link.meta.get("scale", ["?"])[-1]))

        # Opt-in, and recorded the same way poles is: the banner reprint is what
        # puts edt=1 in the CSV, so a run with empty esc_* columns says whether
        # it asked. The firmware refuses this mid-sequence, which cannot happen
        # here -- nothing has been triggered yet.
        # Both directions are sent explicitly. The firmware default moves with
        # whichever ESC is fitted, so a run that says nothing inherits it; a run
        # that says something must be able to say either thing, or the default
        # becomes a reflash.
        if args.edt or args.no_edt:
            want = bool(args.edt)
            link.write_line("EDT,%d" % want)
            time.sleep(0.3)
            link.write_line("ID")
            time.sleep(0.5)
            deadline = time.time() + 2.0
            while time.time() < deadline:
                line = link.readline()
                if line and line.startswith("#"):
                    link.absorb_meta(line)
                    if line.startswith("#ERROR,edt"):
                        sys.exit("firmware rejected EDT,%d (%s)" % (want, line))
            if edt_requested(link) != want:
                print("[WARN] asked for EDT,%d but the banner still reports edt=%d"
                      % (want, not want), file=sys.stderr)
            else:
                print("--> EDT:      %s" % ("requested" if want else "not requested"))

        # esc_dir is derived, not declared: --reverse states the prop's own
        # frame (what rotation= records), prop_hand states which ESC
        # direction that frame's "normal" actually is for this batch, and
        # XORing them gives the DShot command that achieves it. Sent every
        # run, not only when --reverse is passed -- unlike EDT, the failure
        # mode here is not a stale reading but the motor spinning the wrong
        # way, so the direction is always asserted rather than left to
        # whatever a previous script invocation last commanded on this
        # board. This is what was COMMANDED; Bluejay gives no
        # acknowledgement, so it is not proof the ESC obeyed -- same
        # limitation as edt=.
        esc_reversed = args.reverse != (prop_hand == "reversed")
        link.write_line("SPIN,%d" % (1 if esc_reversed else 0))
        time.sleep(0.3)
        link.write_line("ID")
        time.sleep(0.5)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            line = link.readline()
            if line and line.startswith("#"):
                link.absorb_meta(line)
                if line.startswith("#ERROR,spin"):
                    sys.exit("firmware rejected SPIN,%d (%s)"
                              % (1 if esc_reversed else 0, line))
        print("--> spin dir: %s" % (link.meta.get("esc", ["?"])[-1]))
        if esc_reversed:
            print("    commanded, not confirmed -- Bluejay does not "
                  "acknowledge direction commands")

        if args.check:
            if args.tare:
                link.write_line("TARE")
                time.sleep(1.5)
                print("  (load cell tared)\n")
            for key in ("schema", "fw", "board", "esc", "scale", "ina", "ambient"):
                for value in link.meta.get(key, []):
                    print("  %-9s %s" % (key, value))

            # Show what is actually on the bus. A missing sensor and a sensor at
            # an unexpected address look the same from the host otherwise.
            link.write_line("SCAN")
            print("\n  I2C bus:")
            deadline = time.time() + 5.0
            while time.time() < deadline:
                line = link.readline()
                if not line:
                    continue
                if line.startswith("#SCAN,found"):
                    addr = line.split(",")[2]
                    known = {"0x76": "BMP280/BME280", "0x77": "BMP280/BME280 (alt)"}
                    hint = known.get(addr.lower(), "INA226" if addr.lower().startswith("0x4") else "")
                    print("    %s  %s" % (addr, hint))
                elif line.startswith("#SCAN,end"):
                    if line.endswith("count=0"):
                        print("    (nothing responded -- check SDA/SCL and pull-ups)")
                    break

            # Live idle samples. The firmware prints at 10 Hz when not running a
            # sequence, so this exercises the whole chain -- load cell, power
            # monitor, telemetry link -- with the motor stopped.
            print("\n  Live samples (motor stopped):")
            print("    %-11s %-9s %-11s %-9s %-7s %-8s %s"
                  % ("thrust", "rpm", "volts", "amps", "escC", "stress", "flags"))
            link.ser.reset_input_buffer()
            seen = 0
            deadline = time.time() + 6.0
            while seen < 5 and time.time() < deadline:
                line = link.readline()
                row = link.parse_row(line) if line else None
                if row is None:
                    continue
                seen += 1
                print("    %-11s %-9d %-11s %-9s %-7s %-8s %s"
                      % ("%.3f g" % link.grams(row), row["rpm"],
                         "%.3f V" % (row["bus_mv"] / 1000.0),
                         "%.4f A" % (row["bus_ua"] / 1e6),
                         row.get("esc_temp_c", "-"), row.get("esc_stress", "-"),
                         link.describe_flags(row["flags"]) or "-"))
            if not seen:
                print("    (no data rows -- firmware is not emitting telemetry)")
            conditions = link.conditions(args.temp_c, args.pressure_hpa)
            if conditions:
                temp_c, press_pa, source = conditions
                print("\n  ambient OK (%s): %.2f C, %.2f hPa -> rho %.4f kg/m3"
                      % (source, temp_c, press_pa / 100.0, air_density(temp_c, press_pa)))
            else:
                print("\n  NO AMBIENT SENSOR DETECTED.")
                print("  Check CSB is tied high (selects I2C), SDO is tied high or low,")
                print("  and that SDA/SCL reach GP4/GP5. The INA226 answering is not")
                print("  evidence the BMP280 is wired correctly -- they are separate parts.")
            if args.watch > 0:
                watch_idle(link, args.watch)
            return

        # No mode means no sequence. --tare and --watch still do their job;
        # what used to happen here was a sweep nobody asked for.
        if mode is None:
            if args.tare:
                link.write_line("TARE")
                time.sleep(1.5)
                print("  (load cell tared)")
            if args.watch > 0:
                watch_idle(link, args.watch)
            return

        link.begin_capture(args.output)
        capture_started = True
        link.journal.write("#RUN,mode=%s\n" % mode)
        link.journal.write("#SETUP,%s,rotation=%s\n" %
                           (format_setup_meta(setup_entries), rotation))
        link.start_keepalive()

        if not args.no_tare:
            link.write_line("TARE")
            time.sleep(1.0)

        if mode == "TRANSIENT":
            rows = collect_samples(link, "TRANSIENT", "START_TRANSIENT",
                                   "END_TRANSIENT", "transient")
        elif mode == "RESPONSE":
            rows = collect_samples(link, "RESPONSE,%d,%d,%d" % (args.base, args.high, args.reps),
                                   "START_RESPONSE", "END_RESPONSE", "response")
        elif mode == "COASTDOWN":
            rows = collect_samples(link, "COASTDOWN,%d" % args.from_throttle,
                                   "START_COASTDOWN", "END_COASTDOWN", "coastdown")
        elif mode == "MASSCHECK":
            masses = [float(m) for m in args.masses.split(",") if m.strip()]
            if args.ballast:
                rows = run_masscheck_unload(link, masses, args.descend)
            else:
                expected = -1.0 if args.thrust_unloads else 1.0
                rows = run_masscheck(link, masses, args.descend, expected)
        elif mode == "KV":
            voltages = [float(v) for v in args.voltages.split(",") if v.strip()]
            # Highest first: the top setpoint is the duty=1 reference the lower
            # levels are fitted against.
            dshots = sorted({int(d) for d in args.kv_dshots.split(",") if d.strip()},
                            reverse=True)
            if not dshots:
                sys.exit("--kv-dshots needs at least one setpoint")
            if len(dshots) < 2:
                print("[WARN] one DShot level only: Kv and R will not be separately"
                      " identified. See --kv-dshots.", file=sys.stderr)
            rows = run_kv(link, voltages, args.hold_ms, args.psu_port, dshots,
                          args.kv_settle_ms)
        elif mode == "LINKTEST":
            throttles = [int(t) for t in args.throttles.split(",") if t.strip()]
            rows = run_link_test(link, throttles, args.hold_ms)
        elif mode == "FINEWALK":
            lo, hi, step = (int(v) for v in args.dshot_range.split(","))
            if step < 1 or hi < lo:
                raise ValueError("--dshot-range needs LO,HI,STEP with HI>=LO, STEP>=1")
            values = list(range(lo, hi + 1, step))
            rows = run_fine_walk(link, values, args.walk_reps, args.hold_ms)
        elif mode == "SWEEP":
            rows = run_sweep(link)
        else:
            rows = run_static(link)

    except (Exception, KeyboardInterrupt) as exc:
        failure = ("interrupted" if isinstance(exc, KeyboardInterrupt)
                   else "%s: %s" % (type(exc).__name__, exc))
        exit_code = 130 if isinstance(exc, KeyboardInterrupt) else 1
        print("\n[ERROR] %s" % failure, file=sys.stderr)
    finally:
        print("\nSafety: commanding motor stop.")
        if link is not None:
            try:
                link.write_line("0")
                link.write_line("STOP")
            except Exception:
                pass
            try:
                link.close()
            except Exception as exc:
                failure = failure or "close failed: %s" % exc
                exit_code = exit_code or 1

    if not capture_started:
        return exit_code
    if failure:
        try:
            rows = link.recovered_rows()
        except Exception as exc:
            print("[WARN] Could not summarize partial data: %s; raw capture retained."
                  % exc, file=sys.stderr)
            rows = link.partial_rows
    elif not rows:
        failure = "no measurement rows collected"
        exit_code = 1

    extra_meta = {"result": "status=%s,reason=%s" % (
        "aborted" if failure else "complete",
        (failure or "completed").replace("\n", " ").replace("\r", " ").replace(",", ";")),
        "raw_capture": "file=" + os.path.basename(link.journal_path)}
    # Declared setup goes in ahead of the measured conditions. `setup_entries`
    # is bound in the try block above, which every path reaching here has run.
    # rotation is folded in unconditionally -- unlike the other four setup
    # fields it has no "not recorded" state. The consuming site treats a
    # missing rotation as an undocumented controlled dimension and fails any
    # strict comparison that includes the run, so every run needs a value,
    # and "normal" (no --reverse) is always a real, known value to give it.
    setup_line = format_setup_meta(setup_entries)
    rotation_field = "rotation=%s" % rotation
    extra_meta["setup"] = (setup_line + "," + rotation_field) if setup_line else rotation_field

    # Ambient applies to a partial run too. The rows it did collect are real
    # measurements, and density is the one correction that cannot be applied
    # afterwards -- withholding it from an aborted run would leave the samples
    # saved but not comparable, which is half a rescue.
    conditions = link.conditions(args.temp_c, args.pressure_hpa)
    if conditions:
        temp_c, press_pa, source = conditions
        rho = air_density(temp_c, press_pa)
        extra_meta["conditions"] = ("source=%s,temp_c=%.2f,press_pa=%.1f,rho=%.4f"
                                   % (source, temp_c, press_pa, rho))
        print("\nAmbient (%s): %.1f C, %.1f hPa -> rho %.4f kg/m3 (%.1f%% off ISA)"
              % (source, temp_c, press_pa / 100.0, rho, 100 * (rho / ISA_RHO - 1)))
        apply_density(rows, rho)
    else:
        rho = ISA_RHO
        print("\n[WARN] No ambient data: no BMP280 detected and no --temp-c/--pressure-hpa.")
        print("       Thrust is proportional to air density, so without this the run")
        print("       cannot be compared against one taken on another day or bench.")
        print("       This is not recoverable after the fact -- re-run with ambient.")

    if failure:
        # Save before analysis: partial protocols must never update calibration
        # or produce an apparently complete fit/report.
        if mode == "RESPONSE":
            extra_meta["response"] = "base=%d,high=%d,reps=%d" % (
                args.base, args.high, args.reps)
        write_csv(args.output, rows, fields, link, mode, args.notes, extra_meta)
        print("Incomplete run; raw samples retained in %s" % link.journal_path,
              file=sys.stderr)
        return exit_code

    if mode == "MASSCHECK":
        # The factor in force comes from stand.json, not the firmware banner.
        factor = link.scale
        expected = -1.0 if (args.thrust_unloads or args.ballast) else 1.0
        if args.ballast:
            expected = 1.0   # removing ballast reads positive
        fit = fit_masscheck(rows, factor, expected)
        if fit:
            # slope is computed under the factor this run derived, so it reports
            # linearity, not agreement with the stored factor -- the agreement
            # lives in scale_error, and both belong in the file, because the
            # run's console output is not preserved with the CSV.
            mass_parts = ["slope=%.5f" % fit["slope"],
                          "offset_g=%.4f" % fit["offset_g"],
                          "max_resid_g=%.4f" % fit["max_resid_g"],
                          "linearity_pct_fs=%.3f" % fit["linearity_pct_fs"],
                          "suggested_factor=%.4f" % fit["suggested_factor"]]
            if fit.get("current_factor"):
                mass_parts += ["current_factor=%.4f" % fit["current_factor"],
                               "scale_error=%+.4f" % fit["scale_error"]]
            extra_meta["mass_fit"] = ",".join(mass_parts)
        write_csv(args.output, rows, fields, link, mode, args.notes, extra_meta)
        report_masscheck(rows, fit)
        bad = [p for p in rows if not p.get("settled", 1)]
        if bad:
            # Warning about a point and then fitting through it anyway is how a
            # settling transient ends up in stand.json looking like a calibration.
            print("\n  NOT saved: %d of %d points failed the stability check."
                  % (len(bad), len(rows)))
            print("  Re-run, or set the factor by hand with --scale.")
        elif fit and fit.get("suggested_factor") and not args.no_save:
            # Close the loop: the next run uses this without a reflash or a
            # copy-paste, which is the whole point of moving calibration here.
            save_stand_config({"hx711_scale": round(fit["suggested_factor"], 4)})
            print("\n  Saved hx711_scale = %.4f to %s"
                  % (fit["suggested_factor"], os.path.basename(STAND_CONFIG)))
            print("  Re-run MASSCHECK to verify; it should now read +-1.000.")
        return

    if mode == "LINKTEST":
        check_rotation_sign(rows, rotation)
        write_csv(args.output, rows, fields, link, mode, args.notes, extra_meta)
        report_link_test(link, rows, args.hold_ms)
        return

    if mode == "FINEWALK":
        check_rotation_sign(rows, rotation)
        write_csv(args.output, rows, fields, link, mode, args.notes, extra_meta)
        report_fine_walk(rows)
        return

    if mode == "KV":
        fit = fit_kv(rows)
        if fit:
            extra_meta["kv_fit"] = ("kv_rpm_per_v=%.0f,r_ohm=%.4f,rms_rpm=%.0f,n=%d"
                                    % (fit["kv_rpm_per_v"], fit["r_ohm"],
                                       fit["rms_rpm"], fit["n_points"]))
            # The fitted duty per level is what the fit solved on -- the column
            # it belongs in has existed since the field set was written, but was
            # never filled. Points whose level was excluded from the fit (a
            # failed start) get no duty and keep the column blank.
            for p in rows:
                duty = fit["duty"].get(int(p.get("dshot") or 2000))
                if duty is not None:
                    p["duty_fitted"] = round(duty, 4)
        check_rotation_sign(rows, rotation)
        write_csv(args.output, rows, fields, link, mode, args.notes, extra_meta)
        report_kv(rows, fit)
        return

    if fields is STEADY_FIELDS:
        qc_actuator_disk(rows, args.prop_diameter_mm, rho)

    if mode == "RESPONSE":
        # The two commanded levels, so a reader can pick step edges exactly
        # instead of inferring them from sample counts -- which is what
        # currently lets the firmware's 400 ms spindown tail leak into fall
        # times. key=value only: a bare positional token would read as a label
        # to the site's header parser and flatten the line differently.
        extra_meta["response"] = "base=%d,high=%d,reps=%d" % (args.base, args.high, args.reps)

    check_rotation_sign(rows, rotation)
    write_csv(args.output, rows, fields, link, mode, args.notes, extra_meta)
    # RESPONSE alternates between two known throttles, so the step report is
    # told which they are rather than inferring them from the trace.
    step_levels = (args.base, args.high) if mode == "RESPONSE" else None
    report(rows, mode, link, detail_codes=args.codes, step_levels=step_levels)


if __name__ == "__main__":
    sys.exit(main())
