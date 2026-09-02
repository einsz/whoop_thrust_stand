#!/usr/bin/env python3
# Thrust stand -- host: OWON HDS242 handheld oscilloscope over USB
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

"""Capture traces from an OWON HDS242 so a waveform can be recorded, not retyped.

Some measurements on this bench are shapes rather than numbers -- whether the
ESC chops at full throttle, what the commutation looks like, how far the bus
actually rises during a spin-down. A photograph of a handheld screen is a poor
record of those, and the INA226 converts every 8.8 ms so it cannot see them at
all.

Verified against an HDS242 reporting `OWON,HDS242,22420256,V8.0.1`. Four things
about it shaped this module:

**It enumerates oddly but ends up as a plain serial port.** USB 5345:1234,
interface class 5 ("Physical Interface Device"), which is neither CDC-ACM nor
USBTMC -- yet the kernel binds a serial driver and it appears as /dev/ttyUSB*.
No libusb needed. Note the CH340 of the bench supply also lands in ttyUSB, so
the two are told apart by ID_MODEL rather than by number: see find_port().

**Terminators do not matter.** It answers to LF, CRLF, CR and even to no
terminator at all -- the opposite of the bench supply, which is silent to
anything but CRLF.

**Command abbreviation does matter, inconsistently.** `:CH1:SCALe?` answers and
`:CHAN1:SCAL?` is silent; `:MEAS:CH1:MAX?` answers and `:MEASure:CH1:MAX?` is
silent; `:HORIzontal:SCALe?` answers and `:TIMebase:SCALe?` is silent. There is
no rule -- the forms below are the ones observed to work, so prefer editing them
over inventing new ones.

**Block replies carry a 4-byte little-endian length prefix**, then the payload.
`:DATA:WAVE:SCREen:HEAD?` returns JSON describing scales, coupling, probe and
sample rate, so a capture is self-describing and does not depend on the scope's
front-panel state being remembered separately.

Vertical decoding is confirmed rather than assumed: screen samples are signed
bytes at **25 counts per division**, zero at screen centre. With nothing
connected and 2.00 V/div, the largest sample was 2 and `:MEAS:CH1:MAX?`
independently reported 160.0 mV -- and 2 * 2.00/25 = 160 mV exactly.

**Horizontal scaling is confirmed: the screen is 12 divisions wide**, measured
against the 1 kHz compensation output, which put exactly 250.00 samples in a
1 ms period with zero spread -- 4 us per sample at 200 us/div, and 600 samples
spanning 12.00 divisions. So for screen data:

    seconds per sample = timebase_per_div * 12 / 600 = timebase_per_div / 50

**Do not use the header's SAMPLERATE for that.** It reports the *acquisition*
rate into the 8K deep memory (2.5 MSa/s at 200 us/div), while the 600 screen
points are decimated from it -- a factor of ten apart at that setting. Both
numbers are correct and they describe different things; only the timebase
relation above applies to what `capture()` returns.

**Writing the timebase over SCPI is unreliable and the rule is not known.**
`:HORI:SCAL 200us` was accepted once, moving the scope from 2.0ms; every later
write has been silently ignored, including the identical string, at both
RUNSTATUS "TRIG" and "AUTo". Run state was the first hypothesis and it does not
hold. Reads are always reliable, so set_timebase() writes, reads back, and
raises on a mismatch -- the time axis is *derived* from the timebase, so an
ignored write yields a wrong axis rather than an obviously missing one. In
practice, use the front-panel knob and let the capture's header record where it
ended up.

**This model has no signal generator.** `:FUNC?`, `:FUNC:FREQ?` and friends
answer with plausible values, but the HDS242 has only a fixed ~1 kHz
probe-compensation output -- the generator belongs to the HDS242S and the
firmware is shared. A reply from this instrument is not evidence that the
feature exists, which is worth remembering before building anything on one.
The compensation output is still what calibrates the time axis: a known
frequency is exactly what the unknown axis needs.

Usage:
    python tools/scope.py --probe
    python tools/scope.py --capture CH1 -o data/phase.csv
    python tools/scope.py --calibrate       # probe on the 1 kHz comp output
"""

import argparse
import json
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("scope.py needs pyserial: pip install pyserial")

BAUD = 115200
COUNTS_PER_DIV = 25          # confirmed against :MEAS:CH1:MAX?, see module docstring
# Screen width, confirmed against the 1 kHz compensation output: 250.00 samples
# per 1 ms period at 200 us/div, i.e. 600 samples over exactly 12 divisions.
SCREEN_DIVISIONS = 12
USB_VID, USB_PID = 0x5345, 0x1234


class ScopeError(RuntimeError):
    pass


def find_port():
    """The scope's port, by USB id rather than by ttyUSB number.

    The bench supply's CH340 also enumerates as ttyUSB, and which one gets 0
    depends on plug order. Guessing wrong means talking SCPI at a power supply,
    so this matches on the vendor/product id instead.
    """
    for port in list_ports.comports():
        if port.vid == USB_VID and port.pid == USB_PID:
            return port.device
    raise ScopeError("no OWON scope found on USB (looked for %04x:%04x). "
                     "Is it powered on and connected?" % (USB_VID, USB_PID))


class Scope:
    def __init__(self, port=None, timeout=1.5):
        self.port = port or find_port()
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=timeout)
        except serial.SerialException as exc:
            raise ScopeError("cannot open %s: %s" % (self.port, exc))
        time.sleep(0.15)
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # -- io ---------------------------------------------------------------
    def ask(self, cmd, settle=0.35):
        """A short text reply. Empty means the scope did not recognise it."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        time.sleep(settle)
        return self.ser.read(self.ser.in_waiting or 1).decode(errors="replace").strip()

    def send(self, cmd, settle=0.35):
        self.ser.write((cmd + "\n").encode())
        time.sleep(settle)

    def ask_block(self, cmd, settle=0.6):
        """A length-prefixed block reply: 4-byte little-endian count, then data."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        time.sleep(settle)
        raw = self.ser.read(self.ser.in_waiting or 4)
        # A slow or large block can still be arriving; keep reading until the
        # declared length is satisfied rather than truncating it silently.
        if len(raw) >= 4:
            want = struct.unpack("<I", raw[:4])[0] + 4
            deadline = time.time() + 2.0
            while len(raw) < want and time.time() < deadline:
                chunk = self.ser.read(want - len(raw))
                if not chunk:
                    break
                raw += chunk
        if len(raw) < 4:
            raise ScopeError("no block reply to %r" % cmd)
        length = struct.unpack("<I", raw[:4])[0]
        body = raw[4:4 + length]
        if len(body) != length:
            raise ScopeError("%r declared %d bytes, got %d" % (cmd, length, len(body)))
        return body

    # -- state ------------------------------------------------------------
    def identify(self):
        return self.ask("*IDN?")

    def header(self):
        """The capture's own description: scales, coupling, probe, sample rate."""
        try:
            return json.loads(self.ask_block(":DATA:WAVE:SCREen:HEAD?").decode())
        except ValueError as exc:
            raise ScopeError("could not parse the header JSON: %s" % exc)

    def channel_state(self, channel="CH1"):
        head = self.header()
        for entry in head.get("CHANNEL", []):
            if entry.get("NAME") == channel:
                return entry, head
        raise ScopeError("%s not present in the header" % channel)

    def set_coupling(self, channel="CH1", coupling="DC"):
        """DC is what a duty-cycle or supply-rail measurement needs.

        The scope powers up AC-coupled, which silently removes exactly the part
        of a PWM waveform being measured.
        """
        self.send(":%s:COUP %s" % (channel, coupling))
        state, _ = self.channel_state(channel)
        if state.get("COUPLING", "").upper() != coupling.upper():
            raise ScopeError("asked for %s coupling on %s, scope reports %s"
                             % (coupling, channel, state.get("COUPLING")))
        return state["COUPLING"]

    # -- capture ----------------------------------------------------------
    def capture(self, channel="CH1"):
        """One screen of samples, in volts, with the state that produced them.

        Volts come from the header's own scale and probe factor, so a capture
        cannot be misread later because the front panel was changed in between.
        `dt_s` comes from the timebase and the confirmed 12-division screen
        width, not from the header's SAMPLERATE -- that field describes the
        acquisition into deep memory, which is ten times faster than the screen
        decimation at some settings.
        """
        state, head = self.channel_state(channel)
        raw = self.ask_block(":DATA:WAVE:SCREen:%s?" % channel)
        samples = struct.unpack("<%db" % len(raw), raw)   # signed, centre = 0

        volts_per_div = _parse_scale(state["SCALE"])
        probe = _parse_probe(state.get("PROBE", "1X"))
        step = volts_per_div * probe / COUNTS_PER_DIV
        offset_counts = state.get("OFFSET", 0) or 0
        volts = [(s - offset_counts) * step for s in samples]

        per_div = _parse_time(head.get("TIMEBASE", {}).get("SCALE"))
        dt = per_div * SCREEN_DIVISIONS / len(samples) if per_div else None

        return {
            "channel": channel,
            "volts": volts,
            "counts": list(samples),
            "volts_per_div": volts_per_div,
            "probe": probe,
            "coupling": state.get("COUPLING"),
            "timebase": head.get("TIMEBASE", {}).get("SCALE"),
            "sample_rate": head.get("SAMPLE", {}).get("SAMPLERATE"),
            "dt_s": dt,
            "header": head,
        }

    def set_timebase(self, value):
        """Set seconds/div, verified. Raises if the scope ignored it.

        It often does: writes are accepted in some run states and silently
        dropped in others, with no error either way. An unverified write means
        a capture at a timebase nobody chose, and since the time axis is
        derived from it, that is a wrong time axis rather than a missing one.
        """
        self.send(":HORI:SCAL %s" % value, settle=0.8)
        got = self.ask(":HORIzontal:SCALe?")
        if got != value:
            raise ScopeError("asked for %s/div, scope reports %s -- writes are "
                             "ignored in some run states; set it on the front "
                             "panel" % (value, got))
        return got


def _parse_scale(text):
    """'2.00V' / '500mV' -> volts per division."""
    text = text.strip()
    for suffix, factor in (("mV", 1e-3), ("uV", 1e-6), ("V", 1.0)):
        if text.endswith(suffix):
            return float(text[:-len(suffix)]) * factor
    raise ScopeError("cannot parse vertical scale %r" % text)


def _parse_probe(text):
    """'1X' / '10X' -> multiplier."""
    try:
        return float(text.strip().rstrip("Xx"))
    except ValueError:
        raise ScopeError("cannot parse probe factor %r" % text)


def probe_report(scope):
    print("identity      %s" % scope.identify())
    print("port          %s" % scope.port)
    head = scope.header()
    tb = head.get("TIMEBASE", {})
    sm = head.get("SAMPLE", {})
    print("timebase      %s/div" % tb.get("SCALE"))
    print("sample        %s, %s points, depth %s"
          % (sm.get("SAMPLERATE"), sm.get("DATALEN"), sm.get("DEPMEM")))
    print("run status    %s" % head.get("RUNSTATUS"))
    for entry in head.get("CHANNEL", []):
        print("%-13s %s, %s coupling, probe %s, display %s"
              % (entry["NAME"], entry["SCALE"], entry["COUPLING"],
                 entry["PROBE"], entry["DISPLAY"]))
        if entry["COUPLING"].upper() == "AC":
            print("              [note] AC coupling removes the DC content -- set DC "
                  "before measuring a rail or a duty cycle")


def write_capture(path, cap, extra_meta=None):
    import csv
    with open(path, "w", newline="") as fh:
        fh.write("# scope,%s\n" % cap["header"].get("MODEL", "?"))
        fh.write("# channel,%s,coupling=%s,scale=%gV/div,probe=%gX\n"
                 % (cap["channel"], cap["coupling"], cap["volts_per_div"], cap["probe"]))
        fh.write("# timebase,%s/div,sample_rate=%s\n"
                 % (cap["timebase"], cap["sample_rate"]))
        # Recorded verbatim so a capture can be re-derived if the decoding is
        # ever found to be wrong -- the same reason every CSV here carries raw
        # counts beside scaled values.
        fh.write("# timeaxis,dt_s=%s,divisions=%d\n"
                 % (("%.6g" % cap["dt_s"]) if cap["dt_s"] else "unknown",
                    SCREEN_DIVISIONS))
        # Whatever the caller knows about the conditions this trace was taken
        # under. A waveform is uninterpretable without the machine's state at
        # the moment of capture, and only the caller has that.
        for line in (extra_meta or []):
            fh.write("# %s\n" % line)
        writer = csv.writer(fh)
        writer.writerow(["sample", "t_s", "counts", "volts"])
        dt = cap["dt_s"]
        for i, (c, v) in enumerate(zip(cap["counts"], cap["volts"])):
            writer.writerow([i, ("%.9g" % (i * dt)) if dt else "", c, "%.6g" % v])
    print("wrote %d samples to %s" % (len(cap["volts"]), path))


def calibrate_horizontal(scope, reference_hz=1000.0):
    """Pin seconds-per-sample using the probe-compensation output.

    The HDS242 has no signal generator -- the `:FUNC...` commands answer because
    the firmware is shared with the HDS242S, which is a trap worth knowing about
    since a supported-looking reply means nothing here. What it does have is a
    fixed ~1 kHz compensation square wave, and a known *frequency* is exactly
    what the unknown axis needs: vertical was already confirmed against
    `:MEAS:CH1:MAX?`, so no known amplitude is required.

    Counts samples between rising edges, which turns 1 kHz into a measured
    seconds-per-sample, and reports the implied screen width so the header's
    timebase can be used directly from then on.
    """
    print("Connect the CH1 probe to the probe-compensation output, then press Enter.")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        return 1
    scope.set_coupling("CH1", "DC")
    time.sleep(0.5)
    cap = scope.capture("CH1")
    volts = cap["volts"]
    lo, hi = min(volts), max(volts)
    span = hi - lo
    if span <= 0:
        sys.exit("[ERROR] flat trace -- is the probe on the compensation pad?")

    # Hysteresis around the midpoint, so noise on a slow edge cannot produce a
    # burst of false crossings.
    mid = (hi + lo) / 2.0
    high_th, low_th = mid + 0.15 * span, mid - 0.15 * span
    raw_edges, armed = [], False
    for i, v in enumerate(volts):
        if v < low_th:
            armed = True
        elif v > high_th and armed:
            raw_edges.append(i)
            armed = False

    # Hysteresis alone is not enough on a fast edge: overshoot can dip back
    # under the low threshold and re-arm within a couple of samples, so one
    # transition is found twice and the apparent period collapses. Merge
    # crossings that are far too close together to be separate periods.
    debounce = max(4, len(volts) // 50)
    edges = []
    for e in raw_edges:
        if not edges or e - edges[-1] > debounce:
            edges.append(e)
    if len(raw_edges) != len(edges):
        print("  (merged %d ringing double-detections)" % (len(raw_edges) - len(edges)))

    print("\ncaptured  %d samples, %.3f V .. %.3f V (%.3f Vpp)"
          % (len(volts), lo, hi, span))
    print("scope says frequency %s" % scope.ask(":MEAS:CH1:FREQ?"))
    if len(edges) < 2:
        print("\nonly %d rising edge(s) found -- set a faster timebase so at least"
              % len(edges))
        print("two full periods fit on screen, then run this again.")
        return 1

    periods = [b - a for a, b in zip(edges, edges[1:])]
    mean_period = sum(periods) / len(periods)
    dt = 1.0 / reference_hz / mean_period
    print("rising edges at samples %s" % edges[:8])
    print("period %.2f samples (spread %d..%d over %d periods)"
          % (mean_period, min(periods), max(periods), len(periods)))
    print("\n  seconds per sample = %.4g s  (%.4g Sa/s)" % (dt, 1.0 / dt))
    per_div = _parse_time(cap["timebase"])
    if per_div:
        print("  screen spans %.4g s at %s/div -> %.2f divisions"
              % (dt * len(volts), cap["timebase"], dt * len(volts) / per_div))
        print("\n  A whole number of divisions confirms the screen-width constant")
        print("  capture() derives dt from, so captures carry a real time axis.")
        print("  If it is not a whole number, set SCREEN_DIVISIONS for this model.")
    return 0


def _parse_time(text):
    """'2.0ms' -> seconds per division, or None if unparseable."""
    if not text:
        return None
    text = text.strip()
    for suffix, factor in (("ns", 1e-9), ("us", 1e-6), ("ms", 1e-3), ("s", 1.0)):
        if text.endswith(suffix):
            try:
                return float(text[:-len(suffix)]) * factor
            except ValueError:
                return None
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=None,
                    help="Serial port; found by USB id if omitted")
    ap.add_argument("--probe", action="store_true", help="Print the scope's state")
    ap.add_argument("--capture", metavar="CH", default=None, help="CH1 or CH2")
    ap.add_argument("--coupling", default=None, choices=["AC", "DC"],
                    help="Set coupling on the captured channel first")
    ap.add_argument("--calibrate", action="store_true",
                    help="Pin the time axis against the 1 kHz compensation output")
    ap.add_argument("-o", "--output", default=None, help="CSV path for --capture")
    args = ap.parse_args()

    if not any((args.probe, args.capture, args.calibrate)):
        args.probe = True

    scope = None
    try:
        scope = Scope(args.port)
        if args.calibrate:
            return calibrate_horizontal(scope)
        if args.probe:
            probe_report(scope)
        if args.capture:
            if args.coupling:
                print("coupling -> %s" % scope.set_coupling(args.capture, args.coupling))
            cap = scope.capture(args.capture)
            lo, hi = min(cap["volts"]), max(cap["volts"])
            print("%s: %d samples, %.4f V .. %.4f V (%s coupling, %gV/div, probe %gX)"
                  % (cap["channel"], len(cap["volts"]), lo, hi, cap["coupling"],
                     cap["volts_per_div"], cap["probe"]))
            if args.output:
                write_capture(args.output, cap)
    except ScopeError as exc:
        sys.exit("[ERROR] %s" % exc)
    finally:
        if scope is not None:
            scope.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
