#!/usr/bin/env python3
# Thrust stand -- firmware emulator for host-side testing
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

"""Firmware emulator for host-side development, speaking telemetry schema 1.

Creates a pty and runs measure.py against it, so the host link can be exercised
without the bench. It reproduces the timing that motivated the schema: samples
are printed at 250 Hz while the HX711 updates at 80 Hz and the INA at 100 Hz, so
a correct host must deduplicate on the sequence counters rather than averaging
printed rows.

Usage:
    python tools/fake_stand.py -m SWEEP -o /tmp/out.csv
    python tools/fake_stand.py --serve      # just expose a pty and print its path
    python tools/fake_stand.py -m MASSCHECK --mass-slope 1.03
"""

import argparse
import math
import os
import pty
import random
import re
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

BANNER = [
    "#SCHEMA,1",
    "#FW,thrust_stand,2.0.0,emulated",
    "#BOARD,rp2040",
    "#ESC,pin=15,proto=DSHOT600,poles=12",
    "#SCALE,hx711,dt=16,sck=17,rate_sps=80",
    "#INA,name=INA226,shunt_uohm=10000,max_a=8,conv_us=1100,avg=4,shunt_fs_uv=81920",
    "#FLAGS,1=ina_shunt_sat,2=ina_i_sat,4=thrust_stale,8=rpm_stale,16=watchdog,32=esc_alert,64=edt_stale",
    "#AMBIENT,banner,present=1,temp_c=21.30,press_pa=100851.0",
    "#PHASE,0=idle,1=sweep_hunt,2=sweep_up,3=transient,4=sweep_down,5=response,6=coastdown,7=hold,8=spindown",
    "#COLS,t_us,phase,throttle_pct,dshot,rpm,erpm,erpm_raw,n_rpm,thrust_raw,"
    "n_thrust,thrust_age_us,bus_mv,bus_ua,shunt_uv,n_ina,ina_age_us,"
    "esc_temp_c,esc_stress,esc_dbg1,esc_dbg2,n_edt,edt_age_us,flags",
]

SHUNT_FS_UV = 81920  # +-81.92 mV, fixed property of INA226/INA231

# --- eRPM telemetry, as the wire actually carries it -----------------------
# Bidirectional DShot does not send a speed. It sends the period of one
# electrical revolution in microseconds, packed as a 9-bit mantissa shifted by a
# 3-bit exponent, and the host divides to get back to RPM. Emulating that
# faithfully matters because it is the only source of the coarseness at the top
# of a sweep: resolution degrades as RPM^2, so a step near full throttle can be
# smaller than one representable period. An emulator that sends exact RPM makes
# every host-side resolution check pass that should not.
#
# Measured ESCs quantise the period on top of the encoding, reporting only
# values of ceil(LATTICE_US * n) below a threshold period. That is an empirical
# observation from decoded runs, not part of any DShot specification, so it is a
# model of the hardware rather than of the protocol -- and it is coarser than
# the encoding alone, which is exactly why it must be modelled here.
ERPM_LATTICE_US = 1.5
ERPM_LATTICE_BELOW_US = 384    # = 1.5 * 256, an 8-bit boundary in the ESC
ERPM_STOPPED_CODE = 0xFFF


def encode_erpm(period_us):
    """Pack a period in microseconds into the 12-bit telemetry word."""
    if period_us <= 0:
        return ERPM_STOPPED_CODE
    period_us = int(period_us)
    exponent = 0
    while (period_us >> exponent) > 0x1FF and exponent < 7:
        exponent += 1
    return (exponent << 9) | ((period_us >> exponent) & 0x1FF)


def decode_erpm(code):
    """The eRPM a host recovers from that word -- the firmware's own decode."""
    if code == ERPM_STOPPED_CODE:
        return 0
    period = (code & 0x1FF) << (code >> 9)
    return (60000000 + 50 * period) // period if period else 0
HX711_HZ = 80.0
INA_HZ = 100.0
# Row rate per fitted part, tracking firmware.ino's PRINT_FAST_US. Nominal
# rather than achieved: the emulator has none of the firmware's loop overhead,
# so it models the ceiling. Keeping these in step matters because a host check
# that passes here and fails on the bench is worse than no check.
PRINT_HZ_BY_PART = {"hx711": 250.0, "nau7802": 667.0}
PRINT_HZ = PRINT_HZ_BY_PART["hx711"]
# The emulated cell's true sensitivity, in raw counts per gram. The firmware no
# longer knows any factor -- it reports tared raw counts and the host converts --
# so this is the ground truth that the host's MASSCHECK fit has to recover from
# known masses. It no longer needs to track anything in the firmware.
SCALE_FACTOR = -17155.7
MASS_TAU = 0.25        # s, mechanical settling after a weight is placed
EDT_STALE_US = 3000000  # must match EDT_STALE_US in firmware/firmware.ino
EDT_HZ = 4.0            # EDT frames per second, as measured on the bench
# Must match firmware/firmware.ino. The ramped stop exists because a hard step
# makes the ESC brake into a supply that cannot sink; see rampThrottleDown().
SPINDOWN_STEP_DSHOT = 100
SPINDOWN_STEP_MS = 20
SPINDOWN_TAIL_MS = 100

# Ground truth for the KV protocol test. The host fit should recover these.
KV_TRUE = 23500.0      # RPM/V
R_TRUE = 0.85          # ohm, lumped motor + ESC + wiring
KQ = 3.26e-11          # prop torque coefficient, N*m*s^2/rad^2


def solve_at_duty(volts, duty=1.0):
    """Steady state at a given PWM duty: back-EMF and prop torque balanced.

    RPM = Kv*(V_motor - I_motor*R)  and  Kt*I_motor = KQ*omega^2, Kt = 9.5493/Kv

    Below duty 1 the motor does not see the bus. It sees the chopped average,
    and the bus carries the current scaled the other way:

        V_motor = duty * V_bus        I_bus = duty * I_motor

    Returned current is the *bus* current, because that is what the INA226
    measures and therefore what the host has to work back from. Modelling this
    honestly is what makes the emulator able to check a multi-throttle Kv fit:
    a host that assumed the motor saw the bus would land on a wrong Kv here
    exactly as it would on the bench.
    """
    kt = 9.5493 / KV_TRUE
    v_motor = duty * volts
    rpm = KV_TRUE * v_motor
    for _ in range(200):                       # fixed-point iteration
        omega = rpm * 2.0 * math.pi / 60.0
        current = KQ * omega * omega / kt
        new_rpm = KV_TRUE * (v_motor - current * R_TRUE)
        if new_rpm < 0:
            new_rpm = 0.0
        if abs(new_rpm - rpm) < 1e-6:
            rpm = new_rpm
            break
        rpm += 0.5 * (new_rpm - rpm)           # damped, for stability
    omega = rpm * 2.0 * math.pi / 60.0
    return rpm, duty * KQ * omega * omega / kt


def solve_full_throttle(volts):
    return solve_at_duty(volts, 1.0)


class Model:
    """Crude but physically-shaped whoop-motor model: RPM ~ linear in throttle,
    thrust ~ RPM^2, current ~ RPM^3, with a first-order spin-up lag."""

    def __init__(self, shunt_uohm=10000, mass_sign=1, mass_slope=1.0):
        self.rpm = 0.0
        self.supply_v = 4.0
        self.full_throttle_hold = False
        # Duty of the current Kv hold. 1.0 at the reference level; the lower
        # levels are what give the host's fit a second axis to work with.
        self.kv_duty = 1.0
        # The shunt sets the current ceiling: 81920 uV / shunt. At 100 mOhm that
        # is 0.819 A -- a mismatched shunt is how a current channel silently
        # rails partway up every sweep.
        self.shunt_uohm = shunt_uohm
        # Load cell check: mass_sign is the stand geometry (-1 when the motor
        # points up, so thrust unloads the cell and a placed weight reads
        # negative); mass_slope is the cell's true sensitivity against the factor
        # the host currently holds, so 1.03 emulates a stored factor 3% off.
        self.mass_sign = mass_sign
        self.mass_slope = mass_slope
        self.mass_target_g = 0.0   # what the operator has placed
        self.mass_g = 0.0          # what the cell has settled to
        self.tare_g = 0.0
        # Set from Firmware's SPIN,<0|1> handler, not commanded here directly
        # -- the model has no notion of the DShot command, only its effect.
        self.spin_reversed = False

    def step(self, throttle, dt):
        target = 0.0 if throttle <= 0 else 500.0 * throttle
        tau = 0.045
        self.rpm += (target - self.rpm) * min(1.0, dt / tau)
        if self.rpm < 1.0:
            self.rpm = 0.0
        # Weights settle mechanically over seconds, not instantly. Reading
        # before this has decayed measures the transient, which is exactly the
        # mistake measure.py's wait_stable() exists to prevent.
        self.mass_g += (self.mass_target_g - self.mass_g) * min(1.0, dt / MASS_TAU)
        return self.rpm

    def cell_raw_g(self):
        """Untared cell load: prop thrust plus anything the operator placed.

        A fixed-pitch prop spun backward makes much less thrust, in the
        opposite direction -- not zero, not full magnitude flipped. Measured
        on the bench with a matched forward/reverse pair:
        remarkably flat ~48-51% of forward magnitude across the whole
        throttle range, so 0.5 here is a real number, not a guess. Without
        this, check_rotation_sign() and every other rotation-vs-thrust-sign
        test would pass against the emulator regardless of whether the sign
        logic is actually right -- a reversed run always read positive.
        """
        n = self.rpm / 50000.0
        prop_g = 20.0 * n * n
        if self.spin_reversed:
            prop_g *= -0.5
        return prop_g + self.mass_sign * self.mass_slope * self.mass_g

    def tare(self):
        self.tare_g = self.cell_raw_g()

    def sample(self, t):
        rpm = self.rpm
        n = rpm / 50000.0
        thrust_g = self.cell_raw_g() - self.tare_g
        if self.full_throttle_hold:
            # KV protocol: bus voltage is whatever the operator set, and current
            # follows from the torque balance rather than a fitted curve.
            _, amps = solve_at_duty(self.supply_v, self.kv_duty)
            volts = self.supply_v
        else:
            amps = 2.2 * (n ** 3)
            volts = 4.00 - 0.25 * (n ** 2)
        # vibration + quantization noise
        thrust_g += 0.02 * math.sin(t * 900.0)
        flags = 0
        shunt_uv = int(amps * self.shunt_uohm)
        if abs(shunt_uv) >= SHUNT_FS_UV:
            shunt_uv = SHUNT_FS_UV
            amps = shunt_uv / self.shunt_uohm
        if abs(shunt_uv) >= SHUNT_FS_UV * 98 // 100:
            flags |= 1
        return rpm, thrust_g, volts, amps, shunt_uv, flags


class Firmware:
    def __init__(self, fd, shunt_uohm=10000, mass_sign=1, mass_slope=1.0,
                 edt=True, has_bmp=True, rpm_lattice=True, period_bias=None,
                 thrust_sps=HX711_HZ, load_cell="hx711", mangle_rate=0.0,
                 mangle_ina_rate=0.0, loadcell_dead=False):
        self.fd = fd
        # Fraction of thrust samples delivered as a corrupted I2C transfer. The
        # bench fault this models puts 0xAA in the top byte of the 24-bit word
        # and varies the rest, which reads as roughly +314 g. See
        # tools/bus_check.py. Exists so measure.py's rejection can be tested
        # without a faulty bus to hand.
        self.mangle_rate = float(mangle_rate)
        self.n_mangled = 0
        # The same fault on the INA226's reads. Modelled separately from the
        # thrust rate so a test can isolate one channel: they are the same bus
        # and the same corruption, but the symptom is not the same size.
        self.mangle_ina_rate = float(mangle_ina_rate)
        self.n_mangled_ina = 0
        # The 2026-09-02 bench failure: the ADC answers on the bus but never
        # converts, because begin() failed. Modelled because the host has to
        # cope with it and because SPS,<n> must be refused in that state -- the
        # firmware guard for that has no other way of being exercised.
        self.loadcell_dead = bool(loadcell_dead)
        self.shunt_uohm = shunt_uohm
        # The load cell ADC's conversion rate. Above PRINT_HZ it outruns the row
        # rate and rows stop carrying every conversion, which is the NAU7802 case
        # -- 320 SPS against 250 rows/s. The host must notice; see
        # Accumulator._missed in measure.py.
        self.thrust_sps = float(thrust_sps)
        # Which ADC the emulated firmware reports. Nothing host-side parses the
        # part name today, but the banner is the protocol and a third
        # implementation that quietly disagrees with the other two is how the
        # three drift apart.
        self.load_cell = load_cell
        self.print_hz = PRINT_HZ_BY_PART.get(load_cell, PRINT_HZ)
        self.edt = edt
        self.has_bmp = has_bmp
        self.rpm_lattice = rpm_lattice
        self.period_bias = period_bias      # (lo_us, hi_us, percent) or None
        self.model = Model(shunt_uohm, mass_sign, mass_slope)
        self.throttle = 0
        self.stale_rpm_every = 0
        self.t0 = time.time()
        self.n_rpm = 0
        self.n_thrust = 0
        self.n_ina = 0
        self.last_thrust = 0.0
        self.last_ina = (4000, 0, 0)
        self.thrust_t = 0.0
        self.ina_t = 0.0
        self.n_edt = 0
        self.last_edt = (24, 0)
        self.edt_t = 0.0
        self.edt_hz = EDT_HZ
        self.rows = 0
        # Whether the *host* asked for EDT, which is separate from whether this
        # emulated ESC would answer (--no-edt). Tracks EDT_REQUEST_DEFAULT in
        # firmware/config.h, which became 1 on 2026-09-06. Unasked, no frames
        # flow and nothing is flagged stale, so this default decides what a
        # host that says nothing gets.
        self.edt_requested = True
        # Mirrors firmware's spinReversed: default normal, only a live SPIN,1
        # changes it, always present in the banner (unlike edt=, which is only
        # meaningful once asked).
        self.spin_reversed = False
        self.poles = 12
        self.kv_voltages = []
        # Set only for a KV run. FINEWALK also drives raw DShot setpoints near
        # full throttle, and those must not be mistaken for Kv reference holds
        # or every walk step would step the emulated supply.
        self.kv_mode = False
        self.kv_top_dshot = None
        self.buf = b""
        self.stop = False

    def stats_line(self, tag):
        """Counters advance with wall time, so a host that brackets a window
        with two STATS requests sees a real rate rather than a constant.

        Under --no-edt the EDT counter stays pinned at zero while esc_temp_c
        holds a nonsense value: that is what an ESC which ignored DShot command
        13 looks like after one noise frame has landed in the temperature slot,
        and it is indistinguishable from a healthy link without this counter.
        That case needs the host to have asked (`--edt`); unasked, the channel
        is simply idle and there is nothing for the host to report.
        """
        t = max(0.001, time.time() - self.t0)
        bad_scale = 1.0 + 40.0 * (self.model.rpm / 50000.0) ** 2
        edt = int(t * self.edt_hz) if self.edt_live() else 0
        ok = int(t * 4000) + edt
        bad = int(t * 8 * bad_scale)
        # Split the same way the firmware does: silence rises with RPM because
        # a software DShot ESC runs out of time to answer between commutations,
        # while corruption stays near zero on a healthy line.
        silent = int(bad * (0.2 + 0.8 * min(1.0, self.model.rpm / 50000.0)))
        return ("#STATS,%s,telem_ok=%d,telem_bad=%d,telem_corrupt=%d,"
                "telem_silent=%d,telem_rate_pct=%.2f,"
                "edt_frames=%d,esc_max_stress=%d,esc_alerts=0,rows_dropped=0,"
                # A healthy emulated bus never mangles a transfer, so the probe
                # reads clean. It is reported anyway: a host that only sees the
                # field when it is broken cannot tell a clean bus from firmware
                # too old to have the probe at all.
                "i2c_probe_reads=%d,i2c_probe_bad=0,i2c_probe_last=0x0,"
                "esc_temp_c=%d,esc_volts=%.2f"
                % (tag, ok, bad, bad - silent, silent, 100.0 * ok / (ok + bad), edt,
                   3 if self.edt_live() else 0, self.n_thrust,
                   27 if self.edt_live() else 213, 4.0))

    def edt_live(self):
        """Frames only flow when the host asked and this ESC answers."""
        return self.edt and self.edt_requested

    def banner(self):
        return [l.replace("shunt_uohm=10000", "shunt_uohm=%d" % self.shunt_uohm)
                 .replace("#SCALE,hx711,dt=16,sck=17",
                          "#SCALE,nau7802,addr=0x2A,drdy=16"
                          if self.load_cell == "nau7802" else
                          "#SCALE,hx711,dt=16,sck=17")
                 .replace("rate_sps=80", "rate_sps=%d" % round(self.thrust_sps))
                 .replace("poles=12", "poles=%d,edt=%d,dir=%s"
                          % (self.poles, 1 if self.edt_requested else 0,
                             "reversed" if self.spin_reversed else "normal"))
                for l in BANNER
                if self.has_bmp or not l.startswith("#AMBIENT")]

    # -- io ---------------------------------------------------------------
    def send(self, line):
        os.write(self.fd, (line + "\n").encode())

    def poll(self):
        """Non-blocking read of any pending host commands."""
        import select
        while select.select([self.fd], [], [], 0)[0]:
            try:
                chunk = os.read(self.fd, 4096)
            except OSError:
                self.stop = True
                return
            if not chunk:
                self.stop = True
                return
            self.buf += chunk
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                self.command(line.decode(errors="replace").strip())

    def command(self, cmd):
        if not cmd:
            return
        if cmd == "PING":
            return
        if cmd == "ID":
            for line in self.banner():
                self.send(line)
            return
        if cmd == "TARE":
            self.model.tare()
            self.send("#TARE,ok")
            return
        if cmd == "STOP":
            self.throttle = 0
            return
        if cmd.startswith("POLES,"):
            try:
                n = int(cmd.split(",", 1)[1])
            except ValueError:
                n = 0
            if n < 2 or n > 32 or n % 2:
                self.send("#ERROR,poles_invalid,%d" % n)
            else:
                self.poles = n
                self.send("#POLES,%d" % n)
            return
        if cmd.startswith("SPS,"):
            # Third implementation of the protocol, after the firmware and
            # measure.py. The emulated
            # part follows --load-cell: an emulated HX711 refuses the command
            # exactly as the firmware does, so a host that assumes every board
            # can change rate fails here rather than on the bench.
            try:
                n = int(cmd.split(",", 1)[1])
            except ValueError:
                n = -1
            if self.loadcell_dead:
                # Distinct from sps_invalid on purpose: the rate is fine, the
                # part is not. Reaching an unconfigured ADC is what hung the
                # real board, so the refusal is the fix being tested.
                self.send("#ERROR,sps_unavailable,loadcell_init_failed")
            elif self.load_cell != "nau7802" or n not in (10, 20, 40, 80, 320):
                self.send("#ERROR,sps_invalid,%d" % n)
            else:
                self.thrust_sps = float(n)
                for line in self.banner():
                    self.send(line)
            return
        if cmd.startswith("EDT,"):
            try:
                n = int(cmd.split(",", 1)[1])
            except ValueError:
                n = -1
            if n not in (0, 1):
                self.send("#ERROR,edt_invalid,%d" % n)
            else:
                self.edt_requested = bool(n)
                # An ESC that answers starts doing so from now, not from boot,
                # so the counter is rebased -- otherwise the first row would
                # claim a backlog of frames that were never sent.
                self.edt_t = time.time() - self.t0
                self.send("#EDT,%d" % n)
            return
        if cmd.startswith("SPIN,"):
            try:
                n = int(cmd.split(",", 1)[1])
            except ValueError:
                n = -1
            if n not in (0, 1):
                self.send("#ERROR,spin_invalid,%d" % n)
            else:
                self.spin_reversed = bool(n)
                self.model.spin_reversed = self.spin_reversed
                self.send("#SPIN,%d" % n)
            return
        if cmd == "STATS":
            self.send(self.stats_line("idle"))
            return
        if cmd == "SCAN":
            self.send("#SCAN,begin")
            found = ["0x40"] + (["0x76"] if self.has_bmp else [])
            for addr in found:
                self.send("#SCAN,found," + addr)
            self.send("#SCAN,end,count=%d" % len(found))
            return
        if cmd == "SWEEP":
            self.sweep()
            return
        if cmd == "TRANSIENT":
            self.transient()
            return
        if cmd.startswith("RESPONSE"):
            a = [int(x) for x in cmd.split(",")[1:]] or [40, 70, 5]
            self.response(*(a + [40, 70, 5][len(a):]))
            return
        if cmd.startswith("DHOLD"):
            a = [int(x) for x in cmd.split(",")[1:]] or [1400, 1000, 1500]
            a = a + [1400, 1000, 1500][len(a):]
            self.hold(0, a[1], a[2], dshot_raw=a[0])
            return
        if cmd.startswith("HOLD"):
            a = [int(x) for x in cmd.split(",")[1:]] or [70, 1000, 1500]
            a = a + [70, 1000, 1500][len(a):]
            self.hold(*a)
            return
        if cmd.startswith("DSHOT,"):
            self.set_dshot_raw(cmd.split(",", 1)[1])
            return
        if cmd.startswith("COASTDOWN"):
            a = [int(x) for x in cmd.split(",")[1:]] or [70]
            self.coastdown(a[0])
            return
        if cmd[0].isdigit():
            self.throttle = max(0, min(100, int(cmd)))

    # -- throttle ---------------------------------------------------------
    # Two ways in: a percent (the historical one) or a raw DShot value, which
    # reaches the ~20 steps per percent that the percent map skips. `throttle` is
    # a property so that assigning a percent anywhere -- and the sequences do it
    # in a dozen places -- always clears a raw setpoint. Leaving one latched would
    # make the emulator silently ignore the next percent it was given.
    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, percent):
        self._throttle = percent
        self._dshot_raw = None

    def set_dshot_raw(self, value):
        value = max(0, min(2000, int(value)))
        self._throttle = self.pct_from_dshot(value)
        self._dshot_raw = value

    @staticmethod
    def pct_from_dshot(value):
        """Throttle percent equivalent to a DShot value, kept fractional.

        The firmware rounds this to fill in throttle_pct, but the model needs the
        fraction: rounding here would quantise the emulated speed back to
        1%-throttle granularity and hide the very resolution a raw walk exists
        to exercise.
        """
        return 1.0 + (value - 1) * 99.0 / 1999.0 if value else 0.0

    # -- telemetry --------------------------------------------------------
    def dshot(self):
        if self._dshot_raw is not None:
            return self._dshot_raw
        if self.throttle == 0:
            return 0
        return int(1 + (self.throttle - 1) * (2000 - 1) / (100 - 1))

    def erpm_word(self, rpm):
        """The 12-bit word the ESC would send for this speed.

        Goes through the period, because that is what the ESC measures and
        transmits. A host that only ever sees exact RPM cannot be tested against
        the coarseness that period-based telemetry actually delivers.
        """
        erpm = rpm * self.poles / 2.0
        if erpm < 1.0:
            return ERPM_STOPPED_CODE
        period = 60000000.0 / erpm
        if self.period_bias:
            lo, hi, pct = self.period_bias
            if lo <= period <= hi:
                period *= 1.0 + pct / 100.0
        if self.rpm_lattice and period < ERPM_LATTICE_BELOW_US:
            steps = round(period / ERPM_LATTICE_US)
            period = math.ceil(ERPM_LATTICE_US * steps)
        return encode_erpm(round(period))

    def emit(self, phase):
        t = time.time() - self.t0
        rpm, thrust_g, volts, amps, shunt_uv, flags = self.model.sample(t)

        # channel cadences
        n_thrust = int(t * self.thrust_sps)
        n_ina = int(t * INA_HZ)
        n_rpm = int(t * 3000)
        if self.loadcell_dead:
            # Addressable, not converting: the sequence counter never advances,
            # so every row reads stale and thrust holds its boot value.
            n_thrust = self.n_thrust
        if n_thrust != self.n_thrust:
            self.n_thrust = n_thrust
            self.last_thrust = thrust_g
            self.thrust_t = t
        if n_ina != self.n_ina:
            self.n_ina = n_ina
            self.last_ina = (int(volts * 1000), int(amps * 1e6), shunt_uv)
            self.ina_t = t
        self.n_rpm = n_rpm

        thrust_raw = int(self.last_thrust * SCALE_FACTOR)
        if self.mangle_rate and random.random() < self.mangle_rate:
            # Top byte 0xAA, lower 16 bits arbitrary, interpreted signed over 24
            # bits -- exactly the shape the bench produces. Applied to the
            # printed value only: the model's own state is untouched, because
            # the bus corrupts the transfer and not the conversion.
            thrust_raw = (0xAA0000 | random.getrandbits(16)) - (1 << 24)
            self.n_mangled += 1
        mv, ua, suv = self.last_ina
        if self.mangle_ina_rate and random.random() < self.mangle_ina_rate:
            # A volt or two off a 4 V rail, either way. Deliberately NOT a wild
            # value: the point of this channel is that a corrupt read still
            # looks like a voltage, which is why it needs rejecting rather than
            # eyeballing.
            mv += random.choice((-1, 1)) * random.randint(1000, 2500)
            self.n_mangled_ina += 1
        erpm_raw = self.erpm_word(rpm)
        erpm = decode_erpm(erpm_raw)
        # rpm is reported as the host will derive it, not as the model knows it:
        # everything downstream sees only what survived the period encoding.
        rpm_out = erpm // (self.poles // 2)
        # Emulated EDT: ESC warms with load, stress rises near full throttle.
        # Frames arrive a few times a second, not per row, so the values are held
        # between them exactly as the firmware holds them -- a host that averages
        # or maxes over printed rows counts one frame hundreds of times.
        n_edt = int(t * self.edt_hz) if self.edt_live() else self.n_edt
        if n_edt != self.n_edt:
            self.n_edt = n_edt
            self.last_edt = (int(24 + 40 * (rpm / 50000.0) ** 2),
                             int(min(255, 60 * (rpm / 50000.0) ** 4)))
            self.edt_t = t
        esc_temp, esc_stress = self.last_edt
        # Staleness is only a fault if the channel was asked for, which is how
        # the firmware computes it: unasked, silence is the expected state.
        if self.edt_requested and (t - self.edt_t) * 1e6 > EDT_STALE_US:
            flags |= 64                                    # edt_stale
        if self.stale_rpm_every and self.rows % self.stale_rpm_every == 0:
            flags |= 8                                     # rpm_stale
        self.rows += 1
        self.send("D,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"
                  # Rounded, not truncated: matches setThrottleRaw() in the
                  # firmware, which rounds when it fills in throttle_pct.
                  % (int(t * 1e6), phase, round(self.throttle), self.dshot(),
                     rpm_out, erpm,
                     erpm_raw, self.n_rpm, thrust_raw, self.n_thrust,
                     int((t - self.thrust_t) * 1e6), mv, ua, suv,
                     self.n_ina, int((t - self.ina_t) * 1e6),
                     esc_temp, esc_stress,
                     # Stock Bluejay's debug frames carry these constants. A
                     # patched ESC sends Comm_Period4x here instead -- see
                     # tools/patch_bluejay_debug.py -- but the emulator models
                     # the stock firmware.
                     0x88, 0xAA, self.n_edt,
                     int((t - self.edt_t) * 1e6), flags))

    def ramp_throttle_down(self):
        """Mirror of rampThrottleDown() in the firmware.

        The stop is logged under phase 8 rather than being dead time, so a host
        that must exclude those rows from a step's statistics has something to
        exclude when run against the emulator.
        """
        value = self.dshot()
        while value > 0:
            value = max(0, value - SPINDOWN_STEP_DSHOT)
            self.set_dshot_raw(value)
            self.run_window(SPINDOWN_STEP_MS / 1000.0, 8)
        self.throttle = 0
        self.run_window(SPINDOWN_TAIL_MS / 1000.0, 8)

    def run_window(self, duration, phase, log_after=0.0):
        start = time.time()
        last_log = 0.0
        last_step = start
        while time.time() - start < duration and not self.stop:
            now = time.time()
            self.model.step(self.throttle, now - last_step)
            last_step = now
            self.poll()
            elapsed = now - start
            if elapsed >= log_after and now - last_log >= 1.0 / self.print_hz:
                last_log = now
                self.emit(phase)
            time.sleep(0.0005)

    # -- sequences --------------------------------------------------------
    def sweep(self):
        self.throttle = 0
        self.send("#AMBIENT,start,present=1,temp_c=21.40,press_pa=100851.0")
        self.send("START_SWEEP")
        self.send("MIN_RELIABLE_IDLE,7")
        for t in range(7, 101):
            self.throttle = t
            self.run_window(0.20, 2, 0.10)
        for t in range(99, 6, -1):
            self.throttle = t
            self.run_window(0.20, 4, 0.10)
        self.ramp_throttle_down()
        self.send(self.stats_line("sweep"))
        self.send("#AMBIENT,end,present=1,temp_c=23.10,press_pa=100849.0")
        self.send("END_SWEEP")

    def transient(self):
        self.throttle = 0
        self.send("#AMBIENT,start,present=1,temp_c=21.40,press_pa=100851.0")
        self.send("START_TRANSIENT")
        self.throttle = 100
        self.run_window(0.6, 3)
        self.throttle = 0
        self.run_window(0.6, 3)
        self.send(self.stats_line("transient"))
        self.send("#AMBIENT,end,present=1,temp_c=21.60,press_pa=100849.0")
        self.send("END_TRANSIENT")

    def response(self, base, high, reps):
        self.throttle = 0
        self.send("#AMBIENT,start,present=1,temp_c=21.40,press_pa=100851.0")
        self.send("START_RESPONSE,%d,%d,%d" % (base, high, reps))
        self.throttle = base
        self.run_window(1.0, 5)
        for _ in range(reps):
            self.throttle = high
            self.run_window(0.3, 5)
            self.throttle = base
            self.run_window(0.3, 5)
        self.ramp_throttle_down()
        self.send(self.stats_line("response"))
        self.send("#AMBIENT,end,present=1,temp_c=22.30,press_pa=100849.0")
        self.send("END_RESPONSE")

    def coastdown(self, from_throttle):
        self.throttle = 0
        self.send("#AMBIENT,start,present=1,temp_c=21.40,press_pa=100851.0")
        self.send("START_COASTDOWN,%d" % from_throttle)
        self.throttle = from_throttle
        self.run_window(1.5, 6)
        self.run_window(0.2, 6)
        self.throttle = 0
        self.run_window(1.5, 6)
        self.send(self.stats_line("coastdown"))
        self.send("#AMBIENT,end,present=1,temp_c=21.90,press_pa=100849.0")
        self.send("END_COASTDOWN")

    def hold(self, throttle, log_ms, settle_ms, dshot_raw=None):
        if dshot_raw is not None:
            self.set_dshot_raw(dshot_raw)
            throttle = self.throttle
        self.send("#AMBIENT,start,present=1,temp_c=21.40,press_pa=100850.0")
        self.send("START_HOLD,%d,%d,%d,%d"
                  % (round(throttle), log_ms, settle_ms, self.dshot()))
        # A Kv hold, at the reference level or below it. Only recognised in KV
        # mode: FINEWALK walks raw DShot values near full throttle too, and
        # those must stay ordinary holds against the ordinary model.
        kv_hold = self.kv_mode and (throttle >= 100 or dshot_raw is not None)
        if kv_hold:
            level = dshot_raw if dshot_raw is not None else 2000
            # The host issues the highest setpoint first at each voltage, so a
            # hold at the top level is what marks the start of a new voltage
            # group -- that is where the operator (or the PSU driver) has
            # stepped the supply. Lower levels stay on the voltage just set.
            if self.kv_top_dshot is None or level > self.kv_top_dshot:
                self.kv_top_dshot = level
            if level >= self.kv_top_dshot and self.kv_voltages:
                self.model.supply_v = self.kv_voltages.pop(0)
            self.model.kv_duty = level / 2000.0
            self.model.full_throttle_hold = True
            self.model.rpm = solve_at_duty(self.model.supply_v,
                                           self.model.kv_duty)[0]
            start = time.time()
            while time.time() - start < (settle_ms + log_ms) / 1000.0 and not self.stop:
                self.poll()
                if time.time() - start >= settle_ms / 1000.0:
                    self.emit(7)
                time.sleep(1.0 / self.print_hz)
            self.model.full_throttle_hold = False
        else:
            # Only assign when driven by percent: set_dshot_raw() has already set
            # the throttle, and the property setter would clear the raw setpoint.
            if dshot_raw is None:
                self.throttle = throttle
            self.run_window((settle_ms + log_ms) / 1000.0, 7, settle_ms / 1000.0)
        self.ramp_throttle_down()
        self.send(self.stats_line("hold"))
        self.send("#AMBIENT,end,present=1,temp_c=21.55,press_pa=100849.0")
        self.send("END_HOLD")

    def idle(self, until):
        last_log = 0.0
        last_step = time.time()
        while time.time() < until and not self.stop:
            now = time.time()
            self.model.step(self.throttle, now - last_step)
            last_step = now
            self.poll()
            if now - last_log >= 0.1:
                last_log = now
                self.emit(0)
            time.sleep(0.0005)


class MassOperator(threading.Thread):
    """Emulated human for MASSCHECK, which is the one mode a pty alone cannot
    drive: measure.py prompts for a weight and blocks on Enter.

    The prompts carry the mass in their text, so this reads them back off
    measure.py's stdout and puts exactly what was asked for on the emulated
    cell rather than replaying a sequence of its own. That way --masses,
    --descend and --ballast need no second implementation here, and a prompt
    this does not recognise fails loudly instead of silently answering an
    empty pan.

    Enter is pressed immediately after the weight lands, so the settling
    transient is real and wait_stable() is actually exercised. The exception is
    the ballast step, which measure.py follows with a TARE: a human waits for
    the stand to come to rest before taring, and so does this.
    """

    PROMPT = "press Enter: "

    def __init__(self, proc, model, settle_before_tare=True):
        super().__init__(daemon=True)
        self.proc = proc
        self.model = model
        self.settle_before_tare = settle_before_tare
        self.total_g = 0.0

    def run(self):
        buf = ""
        while True:
            try:
                chunk = self.proc.stdout.read(1)
            except (ValueError, OSError):
                return
            if not chunk:
                return
            text = chunk.decode("utf-8", "replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            buf += text
            if not buf.endswith(self.PROMPT):
                continue
            self.answer(buf.rsplit("\n", 1)[-1])
            buf = ""

    def answer(self, prompt):
        nums = [float(n) for n in re.findall(r"-?\d+\.\d+", prompt)]
        settle_first = False
        if "Place ALL masses" in prompt:
            self.total_g = nums[0]
            applied = self.total_g
            settle_first = True          # measure.py tares right after this one
        elif "Remove all masses" in prompt:
            applied = 0.0
        elif prompt.lstrip().startswith("Apply"):
            applied = nums[0]
        elif "cumulative" in prompt:
            applied = self.total_g - nums[-1]   # removed so far, not this step
        else:
            sys.stderr.write("\n[fake_stand] unrecognised prompt: %r\n" % prompt)
            return
        self.model.mass_target_g = applied
        if settle_first and self.settle_before_tare:
            # Wait for the ballast to come to rest, not a fixed number of time
            # constants: taring mid-transient offsets every later point.
            # --tare-early skips this to check the host still catches it.
            deadline = time.time() + 5.0
            while (time.time() < deadline
                   and abs(self.model.mass_g - applied) > 0.002):
                time.sleep(0.02)
        try:
            self.proc.stdin.write(b"\n")
            self.proc.stdin.flush()
        except (ValueError, OSError, BrokenPipeError):
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", default="SWEEP")
    parser.add_argument("-o", "--output", default="/tmp/fake_out.csv")
    parser.add_argument("--shunt-uohm", type=int, default=10000,
                        help="Emulated shunt in micro-ohms; 100000 reproduces the old railing sensor")
    parser.add_argument("--thrust-sps", type=float, default=HX711_HZ,
                        help="Load cell ADC conversion rate. The default 80 is the "
                             "fitted HX711; 320 is the NAU7802, which outruns the "
                             "250 rows/s print rate so rows carry only a subsample. "
                             "The host must report the gap, not average over it")
    parser.add_argument("--load-cell", choices=("hx711", "nau7802"), default="hx711",
                        help="Which ADC the emulated firmware reports in #SCALE. "
                             "Pair with --thrust-sps 320 for the NAU7802 case")
    parser.add_argument("--mangle-rate", type=float, default=0.0, metavar="P",
                        help="Corrupt this fraction of thrust samples the way "
                             "the bench's I2C fault does: top byte 0xAA, reading "
                             "about +314 g. Use it to check that measure.py "
                             "rejects them before forming a mean")
    parser.add_argument("--mangle-ina-rate", type=float, default=0.0, metavar="P",
                        help="Corrupt this fraction of INA226 bus-voltage reads "
                             "by 1-2.5 V. Still a plausible voltage, which is the "
                             "point: the host must reject it against the step "
                             "median rather than rely on it looking wrong")
    parser.add_argument("--loadcell-dead", action="store_true",
                        help="ADC answers on the bus but never converts, as on "
                             "the bench when VIN was unconnected. Rows read "
                             "stale and SPS,<n> must be refused with "
                             "sps_unavailable rather than reaching the part")
    parser.add_argument("--serve", action="store_true", help="Expose a pty and idle")
    parser.add_argument("--kv-voltages", default="3.0,3.4,3.8,4.2",
                        help="Voltages the emulated operator dials in per HOLD")
    parser.add_argument("--tare-early", action="store_true",
                        help="MASSCHECK --ballast: press Enter before the ballast has "
                             "settled, so the host tares mid-transient. The fit should "
                             "survive it; an offset in every point means it did not")
    parser.add_argument("--mass-slope", type=float, default=1.0,
                        help="MASSCHECK: true cell sensitivity against the factor the "
                             "firmware assumes. 1.03 emulates a factor 3%% off, which "
                             "the host should report and correct in one pass")
    parser.add_argument("--no-edt", action="store_true",
                        help="Emulate an ESC that ignored the EDT enable command: no "
                             "EDT frames, and a nonsense esc_temp_c left behind by one "
                             "noise frame. Only visible if the host asks, so pair it "
                             "with `-- --edt`; --check --watch should call it out")
    parser.add_argument("--no-bmp", action="store_true",
                        help="Drop the BMP280 off the emulated I2C bus")
    parser.add_argument("--no-rpm-lattice", action="store_true",
                        help="Send every representable period instead of the %.1f us "
                             "lattice measured ESCs quantise to. Use to see how much "
                             "of a run's RPM coarseness is the ESC and how much is "
                             "the DShot encoding" % ERPM_LATTICE_US)
    parser.add_argument("--period-bias", metavar="LO,HI,PCT",
                        help="Stretch the reported eRPM period by PCT%% while it lies "
                             "between LO and HI microseconds, i.e. make the ESC "
                             "under-report speed over one RPM band only. For testing "
                             "whether the host notices")
    parser.add_argument("--stale-rpm-every", type=int, default=0, metavar="N",
                        help="Flag every Nth printed row rpm_stale (ESC declines to "
                             "answer that poll). A step with a few stale rows is "
                             "expected behaviour: the host must keep the step and "
                             "record the count in n_rpm_stale rather than discarding "
                             "the whole throttle")
    parser.add_argument("extra", nargs="*", default=[],
                        help="Extra args forwarded to measure.py")
    args = parser.parse_args()

    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)

    # The stand geometry is declared to measure.py, so take it from the same
    # flags rather than keeping a second switch that can disagree with it.
    unloads = "--thrust-unloads" in args.extra or "--ballast" in args.extra
    bias = None
    if args.period_bias:
        lo, hi, pct = (float(v) for v in args.period_bias.split(","))
        bias = (lo, hi, pct)
    fw = Firmware(master, args.shunt_uohm, -1 if unloads else 1, args.mass_slope,
                  edt=not args.no_edt, has_bmp=not args.no_bmp,
                  rpm_lattice=not args.no_rpm_lattice, period_bias=bias,
                  thrust_sps=args.thrust_sps, load_cell=args.load_cell,
                  mangle_rate=args.mangle_rate,
                  mangle_ina_rate=args.mangle_ina_rate,
                  loadcell_dead=args.loadcell_dead)
    fw.stale_rpm_every = args.stale_rpm_every
    fw.kv_voltages = [float(v) for v in args.kv_voltages.split(",") if v.strip()]
    # --serve hands the port to whoever is driving, so the mode is not known
    # here; allow Kv behaviour there and gate it on the mode otherwise.
    fw.kv_mode = args.serve or args.mode.upper() == "KV"
    for line in fw.banner():
        fw.send(line)
    if fw.loadcell_dead:
        # At boot only, exactly as the firmware does it. Reprinting it on every
        # ID would misrepresent a one-time init failure as a standing state.
        fw.send("#ERROR,%s_no_response"
                % ("nau7802" if fw.load_cell == "nau7802" else "hx711"))
    fw.send("SYSTEM_READY")

    if args.serve:
        print("emulated stand on %s (ctrl-c to stop)" % slave_name)
        fw.idle(time.time() + 3600)
        return 0

    mode = args.mode.upper()
    interactive = mode in ("KV", "MASSCHECK")
    extra = list(args.extra)
    # A MASSCHECK against the emulated load cell must never land in the real
    # stand.json: it would overwrite a factor measured on the bench with one
    # derived from a model, and the file is the only record of it. The fit is
    # still reported, which is the whole point of the test.
    if mode == "MASSCHECK" and "--no-save" not in extra:
        extra.append("--no-save")
    proc = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "measure.py"),
         "-m", args.mode, "-o", args.output, "-p", slave_name,
         "--notes", "emulated bench run"] + extra,
        cwd=HERE, bufsize=0,
        stdin=subprocess.PIPE if interactive else None,
        # MASSCHECK needs its prompts read back to know which weight to place;
        # KV only needs Enter, so leave its output on the terminal untouched.
        stdout=subprocess.PIPE if mode == "MASSCHECK" else None)
    if mode == "MASSCHECK":
        MassOperator(proc, fw.model, not args.tare_early).start()
    elif interactive:
        try:
            proc.stdin.write(b"\n" * 40)
            proc.stdin.flush()
        except Exception:
            pass

    while proc.poll() is None and not fw.stop:
        fw.idle(time.time() + 0.05)
    os.close(master)
    os.close(slave)
    return proc.wait()


if __name__ == "__main__":
    sys.exit(main())
