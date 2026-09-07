// Thrust stand -- per-build configuration
// Copyright (C) 2026 Julian Jakobs
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details. You should have received a copy of the GNU General Public
// License along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-or-later

// =========================================================================
// How this board is built: which pins things are on, and which parts are
// fitted. Edit it once when you assemble the stand, and again only if you
// rewire it or swap a component.
//
// This is deliberately NOT where the measurement settings live. Anything that
// is a property of the stand rather than the electronics -- the load cell
// calibration factor, the pole count of the motor currently mounted -- belongs
// in stand.json on the host, because those change between sessions and motors
// and must never require a reflash to update.
//
//     config.h     changes when you pick up a soldering iron
//     stand.json   changes when you recalibrate or swap the motor
//
// Get a value here wrong and the data will look perfectly plausible while
// being wrong by a fixed factor -- INA_SHUNT_UOHM especially. Every value is
// emitted in the boot banner and recorded in every CSV, so a run always says
// what produced it.
// =========================================================================
#pragma once

// --- ESC / DShot ---------------------------------------------------------
// Single signal wire, bidirectional DShot: the MCU drives the throttle frame,
// then releases the line and the ESC answers on the same wire. The line idles
// HIGH and the ESC pulls it low, so if you fit an external pull resistor it
// must be a pull-UP -- but you almost certainly do not need one, as the RP2040
// internal pull-up is enabled by the DShot library.
const uint ESC_PIN    = 15;    // GPIO number, not header pin number
const int  DSHOT_RATE = 600;   // 150 / 300 / 600 / 1200

// Extended DShot Telemetry (temperature, voltage, current, stress, status
// frames interleaved with the eRPM ones), off by default.
//
// It is off because support is an ESC-firmware property that nothing can read
// back, and a channel that never answers is worse than no channel: edt_stale
// lands on every row of every run, and a flag that is always set is a flag
// nobody reads. On the ESC this stand was developed against, EDT stops once
// the motor has armed and no amount of re-requesting revives it.
//
// Turn it on if your ESC supports it properly -- esc_stress is a genuine
// desync canary when it works. Both directions have a runtime override that
// needs no reflash: `measure.py --edt` forces it on, `--no-edt` forces it off.
//
// **Default changed from 0 to 1 on 2026-09-06**, when the fitted ESC changed to
// one that answers. The old default was set against a Bluejay build that stops
// sending EDT once it arms, where asking gained nothing and put edt_stale on
// every row. If your ESC is like that, set this back to 0 or pass --no-edt:
// flags nobody can trust are worse than no flags.
//
// This only controls whether the stand *asks* for EDT. Frames are always
// decoded if they arrive: they are valid telemetry, and treating them as
// anything else counts every one as a link error.
#define EDT_REQUEST_DEFAULT 1

// NOTE: the motor's pole count is not here. It is a property of whatever motor
// is mounted, not of the board, so it lives in stand.json (or `--poles N`) and
// can change per run without a reflash.

// --- Load cell ADC: which part is fitted ---------------------------------
// 0 = HX711 (bit-banged, 80 SPS), 1 = NAU7802 (I2C, up to 320 SPS).
//
// Both are kept because the swap is physical: the module changes, so reverting
// means rewiring, and a build that can only talk to the part you just removed
// is no way to compare the two.
//
// Changing this REQUIRES a fresh MASSCHECK. The parts have different gain
// paths, so counts per gram changes, and the sign may flip with the wiring.
// The factor lives host-side in stand.json, so this is a calibration run, not
// a reflash -- see docs/CALIBRATION.md.
#define LOADCELL_NAU7802 1

#if LOADCELL_NAU7802

// --- Load cell (NAU7802) -------------------------------------------------
// I2C, so it joins SDA/SCL below alongside the INA226 and BMP280. Address is
// fixed at 0x2A on this part and collides with neither.
const uint8_t NAU7802_ADDR = 0x2A;

// DRDY broken out to a GPIO, or -1 if it is not wired.
//
// This matters more than it looks. The library's available() is an I2C
// register read, and serviceThrust() is called every pass of a loop that runs
// far faster than 320 Hz -- polling it that way would flood the bus and starve
// the INA226 and BMP280 that share it. A GPIO read costs nothing and keeps
// serviceThrust() as cheap as the HX711's is_ready() is.
//
// With DRDY unwired the code falls back to a time-gated available() poll. That
// works, but it is a bus transaction per conversion period, so wire the pin if
// the breakout brings it out.
const int NAU7802_DRDY_PIN = 16;   // reuses the freed HX711 DT pin

// Conversion rate: 10, 20, 40, 80 or 320. There is no 160.
//
// 320 is the reason for fitting this part -- the HX711's 80 SPS with ~19 ms of
// sinc group delay barely resolves a 60 ms RPM rise. At 320 the delay is
// ~4.7 ms.
//
// It is a register, not a pin, so it can be changed at runtime if 320 ever
// costs more resolution than the bandwidth is worth.
const int NAU7802_SPS = 320;

// Internal LDO. Powered from 3V3, so 3.0 V is the highest setting that leaves
// the regulator headroom to actually regulate.
//
// This is the whole point of the part on this bench. The HX711's on-chip
// regulator targets ~4.3 V and cannot reach it from 3V3, so it saturates and
// its AVDD just follows the Pico's switching rail with no supply rejection at
// all -- measured 3.23 V at E+ where it should read 4.3.
//
// Do NOT raise this by powering the part from 5 V: the I2C pull-ups would then
// sit at 5 V into non-tolerant Pico pins.
#define NAU7802_LDO_SETTING NAU7802_3V0

// PGA gain. 128 matches what the HX711 ran at, so the two are comparable.
#define NAU7802_GAIN_SETTING NAU7802_GAIN_128

#else

// --- Load cell (HX711) ---------------------------------------------------
const int HX711_DT_PIN  = 16;
const int HX711_SCK_PIN = 17;

// NOTE: there is deliberately no scale factor here. The firmware reports tared
// RAW counts and the host converts them to grams, so recalibrating never means
// reflashing -- which matters, because the factor is a property of the mounting
// and can move between sessions. Run `measure.py -m MASSCHECK`; it stores the
// factor host-side. See docs/CALIBRATION.md.
//
// Taring stays here: it is an offset in counts and needs no calibration.

// Sample rate, set by the HX711's RATE pin: 80 if tied high, 10 if left
// floating or tied low. This only tells the firmware what to expect -- it does
// not configure the chip. If thrust is constantly flagged stale, suspect that
// pin. 10 SPS is too slow to resolve a spin-up; tie RATE high.
const int HX711_SPS = 80;

#endif  // LOADCELL_NAU7802

// --- I2C (power monitor + ambient) ---------------------------------------
const int I2C_SDA_PIN = 4;
const int I2C_SCL_PIN = 5;

// --- Power monitor (INA226 / INA219 / INA23x) ----------------------------
// MUST match the shunt resistor actually fitted to your board. This is a pure
// scale factor on every current, watt and g/W number in the dataset: a wrong
// value produces smooth, believable curves rather than an obvious failure.
//
// Read it off the board -- the large 2512 resistor is marked R100, R010 etc.
// Common values, and the current ceiling each gives against the INA226's fixed
// +-81.92 mV shunt input:
//
//   R100 = 100000 uOhm -> 0.819 A   <-- what most breakouts ship with, and far
//                                       too little for a propeller
//   R010 =  10000 uOhm -> 8.19 A    <-- what you want
//   R005 =   5000 uOhm -> 16.4 A
//
// See docs/HARDWARE.md before buying a breakout.
const uint32_t INA_SHUNT_UOHM = 10000;

// Full-scale current. Two independent ceilings apply and are best kept aligned
// so there is only one number to reason about:
//   - Analog: the chip's shunt full scale (INA_SHUNT_FS_UV below) divided by
//     the shunt resistance. At +-81.92 mV across 10 mOhm that is 8.19 A.
//   - Digital: the library derives current_LSB = INA_MAX_AMPS / 32767, so the
//     current register saturates here.
// Set INA_MAX_AMPS at or just below the analog ceiling. Raising it costs
// resolution only once the LSB exceeds what the shunt ADC can resolve.
const uint16_t INA_MAX_AMPS = 8;

// Shunt ADC full scale, a fixed property of the chip. INA226/INA231: 81920 uV.
// Check your datasheet if you fit something else -- this is what the firmware
// uses to flag saturation, so a wrong value means clipped current goes unnoticed.
const int32_t INA_SHUNT_FS_UV = 81920;

// Conversion time and hardware averaging. 2 * CONV_US * AVERAGING is the time
// for one full bus+shunt update, so these set how fresh the current reading is.
const uint16_t INA_CONV_US   = 1100;
const uint16_t INA_AVERAGING = 4;      // 2 * 1100us * 4 = 8.8 ms per update
