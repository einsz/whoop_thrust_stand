// Thrust stand -- RP2040 firmware, telemetry schema 1
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

#include <Arduino.h>
#include <Wire.h>
#include <PIO_DShot.h>
#include <INA.h>
#include <Adafruit_BMP280.h>

#include "config.h"

#if LOADCELL_NAU7802
#include <Adafruit_NAU7802.h>
#else
#include <HX711.h>
#endif

// =========================================================================
// Thrust stand firmware -- telemetry schema 1
//
// Four design rules run through this file. They exist because the usual way a
// bench like this fails is not a crash but a channel that quietly saturates or
// a constant that is quietly wrong, producing smooth, plausible, useless data:
//
//  1. Every scaled value is logged next to the RAW sensor value it came from
//     (erpm, hx711 counts, shunt microvolts). A wrong calibration constant is
//     then recoverable in post instead of destroying the run.
//  2. Every sample carries per-channel sequence counters, so the host can tell
//     a genuinely new reading from a re-print of a stale one. The sensors run
//     at ~80 Hz (HX711) and ~100 Hz (INA) while rows print at 250 Hz.
//  3. Saturation and staleness are detected in firmware and flagged per sample,
//     never left for a human to notice later.
//  4. The configuration that determines what the numbers MEAN (shunt value,
//     scale factor, pole count, chip identity) is emitted at boot so the host
//     can record it with the run.
//
// Per-build settings live in config.h -- that is the only file you should need
// to touch to bring this up on your own hardware.
// =========================================================================

#define FW_NAME        "thrust_stand"
#define FW_VERSION     "2.0.0"
#define SCHEMA_VERSION 1

// Rotor magnet count: RPM = eRPM / (poles/2). A wrong value scales every RPM
// figure and any Kv derived from it, with nothing downstream able to detect it.
//
// This is only a fallback for a board nobody has talked to yet; the host sets
// the real value with `POLES,<n>` at the start of every run, from stand.json or
// --poles. 12 is near-universal for whoop-class motors (9N12P), 14 common on
// larger ones (12N14P). Because raw `erpm` is logged beside `rpm`, a run
// recorded under the wrong count is still correctable afterwards.
const uint8_t MOTOR_POLES_FALLBACK = 12;
volatile uint8_t motorPoles = MOTOR_POLES_FALLBACK;

// True only once the load cell ADC has been brought up AND has actually
// produced conversions. Not a diagnostic: it gates `SPS,<n>`, which reaches
// straight into the part. Sending that to an ADC whose begin() failed talks to
// an unconfigured device and hung the board on 2026-09-02, recoverable only by
// a power cycle -- core 1's backstop is disarmed at zero throttle, which is
// exactly where the command is legal.
bool loadCellInitOk = false;

// The conversion rate actually in force. config.h sets the boot value; `SPS,<n>`
// moves it for the session, which the NAU7802 allows because its rate is a
// register. The HX711's is the RATE pin, so there it is read-only and the
// command is refused rather than silently ignored.
//
// It is a runtime value and not a constant because the rate has to be swept to
// tell a real mount resonance from an alias -- a real mode sits at the same
// frequency at 80 and 320 SPS, an alias moves -- and reflashing between the two
// halves of that comparison changes more than the rate.
// I2C integrity probe. Register 0x1F holds a constant whose low nibble is 0xF,
// delivered over the same bus, from the same address, in the same shape of
// transaction as the ADC data. Reading it alongside the data separates the two
// things a bad sample can mean: a corrupted TRANSFER, which shows up here, from
// a corrupted CONVERSION, which does not.
//
// It is weaker evidence clean than dirty. The ADC read is three bytes and this
// is one, so it has less exposure per transaction -- read the counts together,
// never the mismatch alone.
volatile uint32_t i2cProbeReads = 0;
volatile uint32_t i2cProbeBad   = 0;
volatile uint8_t  i2cProbeLast  = 0;   // the last wrong byte, so the pattern is visible

int loadCellSps =
#if LOADCELL_NAU7802
    NAU7802_SPS;
#else
    HX711_SPS;
#endif

// --- Sample cadence ---
const uint32_t INA_PERIOD_US   = 10000;   // just slower than the 8.8ms conversion
const uint32_t PRINT_IDLE_US   = 100000;  // 10 Hz when idle
// Rows must be printed FASTER than the ADC converts, or conversions are lapped
// and simply never reach a row -- thrust is a latest-value register and
// emitSample() prints whichever sample is current.
//
// **Nominal is not achieved.** The loop also polls serial, services sensors and
// feeds the watchdog, which measured ~357 us of overhead on 2026-09-02: 2500 us
// gave ~350 rows/s rather than 400. Against 320 SPS that was a margin of 1.09,
// too thin to absorb jitter, and 2.6-3.3% of conversions were lost on every run
// that day. 1500 us measured 593 rows/s on the bench, a margin of 1.82, and cut
// the loss from 3.0% to 0.2% with rows_dropped still 0. It does not reach zero:
// even at 1.8x, jitter occasionally laps a conversion.
//
// Size this against the MARGIN, not against the nominal rate. At 80 SPS against
// 250 Hz there is ample margin; at 320 SPS there is none, so the rate goes up with the
// part. The host counts any residual loss as n_thrust_missed and warns.
#if LOADCELL_NAU7802
const uint32_t PRINT_FAST_US   = 1500;    // 667 Hz nominal, 593 Hz measured
#else
const uint32_t PRINT_FAST_US   = 4000;    // 250 Hz, clear of the HX711's 80 SPS
#endif

// --- Staleness thresholds (us) ---
const uint32_t THRUST_STALE_US = 25000;
const uint32_t INA_STALE_US    = 25000;
const uint32_t RPM_STALE_US    = 20000;
// EDT frames are interleaved with eRPM ones at a few per second at best, so this
// is orders of magnitude looser than the others. It exists to catch EDT stopping
// altogether -- which some ESCs do once armed -- rather than to time frames.
const uint32_t EDT_STALE_US    = 3000000;
// How often core 1 re-asks for EDT when it was requested and no frames are
// arriving. The enable command is only valid stopped, so this only ever fires
// with the throttle at zero and never during a run. Five seconds is slow enough
// that an ESC which will never answer costs ~10 command frames per 20,000, and
// fast enough that an ESC powered after the Pico starts reporting promptly.
const uint32_t EDT_RETRY_US    = 5000000;

// --- Safety ---
// Armed by the first PING. A human at a serial monitor is present and can react,
// so we do not fight them; an automated host that dies mid-run is the real hazard.
const uint32_t WATCHDOG_MS = 1000;

// How long core 0 may go without checking in before core 1 stops the motor
// itself. This is the backstop for a core 0 that has stopped advancing the
// throttle at all -- see the comment by core0HeartbeatMs.
//
// Sized against the longest legitimate gap between heartbeats, which is one
// iteration of a sequence's inner loop (sub-millisecond) now that every wait
// inside a sequence goes through sequenceDelay(). 500 ms is three orders of
// magnitude of headroom and still stops a propeller before anyone can reach
// the bench.
const uint32_t CORE0_STALL_MS = 500;

// --- Sample flags ---
const uint8_t FLAG_INA_SHUNT_SAT = 1 << 0;
const uint8_t FLAG_INA_I_SAT     = 1 << 1;
const uint8_t FLAG_THRUST_STALE  = 1 << 2;
const uint8_t FLAG_RPM_STALE     = 1 << 3;
const uint8_t FLAG_WATCHDOG      = 1 << 4;
const uint8_t FLAG_ESC_ALERT     = 1 << 5;
const uint8_t FLAG_EDT_STALE     = 1 << 6;

// Extended DShot Telemetry: the ESC interleaves temperature, voltage, current,
// stress and status frames with the eRPM ones. Enabled by DShot command 13 sent
// at least six times with the motor stopped; an ESC without support ignores it.
//
// Treat whatever arrives as opportunistic. Support varies widely between ESC
// firmwares -- fields may never be sent, may report implausible values, and the
// whole channel may stop once the motor arms. Everything here is plumbed so
// that those cases are visible (edt_stale, blank columns) rather than silently
// frozen, but do not build a measurement on EDT. See docs/HARDWARE.md.
//
// Because of that, asking for it at all is a setting: EDT_REQUEST_DEFAULT in
// config.h, or `EDT,<0|1>` at runtime. Off, the enable command is never sent
// and FLAG_EDT_STALE is never raised -- an ESC that will never answer should
// not flag every row of every run. Decoding is unconditional either way; see
// loop1().
const uint16_t DSHOT_CMD_EDT_ENABLE = 13;
const uint8_t  EDT_ENABLE_REPEATS   = 10;
volatile bool  edtRequested         = EDT_REQUEST_DEFAULT;

// Bluejay's "temporary reverse" commands (src/Modules/DShot.asm): they flip
// which way the commutation sequence spins without touching the ESC's saved
// Pgm_Direction and without SAVE_SETTINGS, so a power cycle reverts it. This
// is the same mechanism Betaflight uses for crash-flip/turtle mode, and it is
// unrelated to 3D mode -- DShot 0 stays "stop" and 48-2047 stay the one-sided
// throttle range. Reasserted at boot and at the start of every sequence
// (rather than only when the host asks) so a direction commanded in a
// previous session can never survive silently into one that never asked --
// getting this wrong spins the motor the wrong way, not just a stale reading.
// DSHOT_CMD_SPIN_DIRECTION_NORMAL/_REVERSED come from PIO_DShot.h's own
// DShotCommand enum -- no need to redeclare the raw values 20/21 here.
const uint8_t  SPIN_CMD_REPEATS = 10;  // Bluejay requires 6 in a row; matches EDT's margin
volatile bool  spinReversed  = false;  // default is always normal; only a live SPIN,1 changes it

// --- Phase codes ---
const uint8_t PHASE_IDLE      = 0;
const uint8_t PHASE_SWEEP_HUNT= 1;
const uint8_t PHASE_SWEEP     = 2;
const uint8_t PHASE_TRANSIENT = 3;
const uint8_t PHASE_SWEEP_DOWN= 4;
const uint8_t PHASE_RESPONSE  = 5;
const uint8_t PHASE_COAST     = 6;
const uint8_t PHASE_HOLD      = 7;
// The ramped stop at the end of a sequence. Logged rather than silent: it is
// the most electrically stressful moment of a run -- the ESC brakes into a
// supply that cannot sink -- and it used to be the only part with no data at
// all. Hosts must exclude this phase from a sequence's statistics; the rows
// belong to the stop, not to the setpoint that preceded it.
const uint8_t PHASE_SPINDOWN  = 8;

// Ambient. Thrust is directly proportional to air density, so without these two
// numbers a run cannot be compared against one taken on a different day, let
// alone against another person's bench. Humidity is not measured yet: over any
// realistic indoor range it moves density by ~0.35%, against ~2% for a few
// degrees and ~3% for ordinary weather, and is smaller than the BMP280's own
// temperature error contributes. A BME280 swap is a possible but minor upgrade
// (docs/HARDWARE.md).
const uint32_t AMBIENT_PERIOD_US = 1000000;

// --- Objects ---
BidirDShotX1 *esc;
#if LOADCELL_NAU7802
Adafruit_NAU7802 scale;
// The NAU7802 has no offset register, so the tare is kept here. The HX711
// library owns its own, which is why this exists only on this branch.
long thrustOffset = 0;
#else
HX711 scale;
#endif
INA_Class INA;
Adafruit_BMP280 bmp;

// --- Inter-core shared state (core 0 -> core 1) ---
volatile uint16_t currentDshotValue = 0;

// --- Inter-core shared state (core 1 -> core 0) ---
// The 12-bit telemetry word exactly as the ESC sent it, not a speed. Bidirectional
// DShot reports the *period* of one electrical revolution (9-bit mantissa shifted
// by a 3-bit exponent), so every RPM figure here is a division of this word and
// its resolution degrades as RPM^2 -- near full throttle one representable period
// is worth hundreds of RPM. Storing the word and deriving erpm/rpm at print time
// is what makes "the ESC sent a different number" distinguishable from "the ESC
// sent the same number again", which a rounded RPM hides completely.
const uint32_t ERPM_RAW_STOPPED = 0xFFF;   // protocol's "motor stopped" report
volatile uint32_t currentErpmRaw   = ERPM_RAW_STOPPED;
volatile uint32_t rpmSeq           = 0;
volatile uint32_t lastRpmUpdateUs  = 0;
volatile uint32_t telemOkCount     = 0;
volatile uint32_t telemBadCount    = 0;
volatile uint32_t telemCorruptCount= 0;   // answered, but failed checksum
volatile uint32_t telemSilentCount = 0;   // no response at all

// EDT state (core 1 -> core 0). Values are in the units the spec defines:
// temperature 1 C/step, voltage 0.25 V/step, current 1 A/step, stress 0-255.
volatile uint8_t  escTempC         = 0;
volatile uint8_t  escStress        = 0;
volatile uint8_t  escStatus        = 0;
// EDT debug frame payloads. Meaningless on stock firmware (constants), but a
// patched ESC can put internal state here -- see tools/patch_bluejay_debug.py.
volatile uint8_t  escDebug1        = 0;
volatile uint8_t  escDebug2        = 0;
volatile uint16_t escVoltsQuarter  = 0;
volatile uint16_t escAmps          = 0;
volatile uint32_t edtFrameCount    = 0;
volatile uint32_t escMaxStress     = 0;
volatile uint32_t escAlertCount    = 0;
// Sequence counter and timestamp for the EDT channel, mirroring what rpm/thrust/
// ina already carry. Rows print at 250 Hz while EDT frames arrive at a few per
// second, so without a counter the host cannot tell a fresh reading from the
// same one repeated hundreds of times -- which lets a single stress sample be
// reported once per step for the length of a run as if it had been measured.
volatile uint32_t edtSeq           = 0;
volatile uint32_t lastEdtUpdateUs  = 0;
// When core 1 last re-asked for EDT. Separate from lastEdtUpdateUs so a silent
// ESC is retried on a fixed cadence rather than every single frame.
volatile uint32_t lastEdtRetryUs   = 0;
// Set by core 1 when a status frame carries error/warning/alert bits, cleared by
// core 0 on the next row printed. A lost or doubled flag across the core
// boundary costs one row's marking; escAlertCount is the authoritative tally.
volatile bool     escAlertPending  = false;
// Core 0 asks, core 1 sends: the ESC object belongs to core 1, and DShot command
// frames are only valid with the motor stopped. Some ESCs stop sending EDT once
// they have armed, so requesting it only in setup1() would leave every esc_*
// column frozen at its last pre-arm value for the rest of the session.
volatile uint8_t  edtEnablePending = 0;
// Same core split and same "motor stopped" constraint as edtEnablePending;
// see the comment by spinReversed for why this is reasserted unconditionally
// rather than only on request.
volatile uint8_t  spinCmdPending   = 0;

// --- Core 0 liveness, watched by core 1 ---
//
// Core 1 pushes currentDshotValue forever and is the only writer to the ESC, so
// if core 0 stops advancing that variable the motor holds its last commanded
// throttle indefinitely -- with a perfectly healthy DShot link, so the ESC never
// sees signal loss and never times out either. Every host-side safety path is
// downstream of core 0, which means none of them can act: pollSerial() only runs
// between sequences, so STOP is never read, and serviceWatchdog() lives on the
// same core that has stopped.
//
// That is not hypothetical. HX711::read() calls wait_ready(), an unbounded
// `while (!is_ready())` -- the library's own comment says it "will halt the
// sketch until a load cell is connected". serviceThrust() gates on is_ready()
// first, but that is check-then-act: if DOUT goes high in the window between the
// two, core 0 spins there forever with the prop at whatever the sequence last
// commanded.
//
// So core 1 is the backstop, and deliberately the only one that cannot be
// defeated by a core 0 fault. Core 0 stamps core0HeartbeatMs everywhere it can
// make progress; core 1 stops the motor when that goes stale.
volatile uint32_t core0HeartbeatMs = 0;
// Latched, never cleared except by a reboot. A core 0 that recovers must not be
// allowed to spin the motor back up on the next loop iteration as though nothing
// had happened -- the run is already invalid and the operator has not looked at
// the stand yet.
volatile bool     core0StallLatched = false;

// Set to abort a running sequence: by STOP arriving mid-sequence, by the host
// watchdog timing out, or by core 1 having stopped the motor underneath us. The
// sequence unwinds to its normal exit so the host still gets its END_ sentinel
// and the rows collected so far, rather than blocking forever waiting for one.
volatile bool     sequenceAbort    = false;

// --- Core 0 sensor state ---
long     thrustRaw         = 0;
uint32_t thrustSeq         = 0;
uint32_t lastThrustUpdateUs= 0;

uint16_t busMilliVolts     = 0;
int32_t  busMicroAmps      = 0;
int32_t  shuntMicroVolts   = 0;
uint32_t inaSeq            = 0;
uint32_t lastInaUpdateUs   = 0;
uint32_t lastInaReadUs     = 0;

bool     ambientPresent    = false;
float    ambientTempC      = 0.0f;
float    ambientPressPa    = 0.0f;
uint32_t lastAmbientReadUs = 0;

// --- Core 0 control state ---
int      targetThrottlePercent = 0;
uint32_t lastPrintUs           = 0;
bool     inSequence            = false;
// Rows discarded because the USB buffer was full -- see emitSample. Counted per
// sequence and reported in #STATS: a dropped row is a known gap, and the host's
// deduplication counters already tell it what it missed, but a silent gap would
// look like a sensor that stopped converting.
uint32_t rowsDropped           = 0;
bool     watchdogArmed         = false;
bool     watchdogTripped       = false;
uint32_t lastHostMsgMs         = 0;

char inputBuf[32];
uint8_t inputLen = 0;
char lineBuf[224];

void runTransientSequence();
void runSweepSequence();
void emitAmbient(const char *tag);
void runResponseSequence(int base, int high, int reps);
void runCoastdownSequence(int fromThrottle);
void runHoldSequence(int throttle, int logMs, int settleMs, int dshotRaw = -1);

// Reads comma-separated integer arguments following a command keyword,
// e.g. "RESPONSE,40,70,5". Leaves defaults untouched when absent.
int parseArgs(const char *cmd, int *out, int maxArgs) {
  int n = 0;
  const char *p = strchr(cmd, ',');
  while (p && n < maxArgs) {
    out[n++] = atoi(p + 1);
    p = strchr(p + 1, ',');
  }
  return n;
}

// =========================================================================
// Telemetry
// =========================================================================

void emitBanner() {
  Serial.print(F("#SCHEMA,")); Serial.println(SCHEMA_VERSION);
  Serial.print(F("#FW,")); Serial.print(F(FW_NAME)); Serial.print(',');
  Serial.print(F(FW_VERSION)); Serial.print(','); Serial.println(F(__DATE__));
  Serial.println(F("#BOARD,rp2040"));

  Serial.print(F("#ESC,pin=")); Serial.print(ESC_PIN);
  Serial.print(F(",proto=DSHOT")); Serial.print(DSHOT_RATE);
  Serial.print(F(",poles=")); Serial.print(motorPoles);
  // Recorded because it explains the esc_* columns: unasked and empty is a
  // choice, asked and empty is a finding about the ESC.
  Serial.print(F(",edt=")); Serial.print(edtRequested ? 1 : 0);
  // Always present, unlike edt=: spinReversed is reasserted at boot and every
  // sequence start, so there is always a definite commanded value, not merely
  // a requested one. This is what was COMMANDED, not confirmed -- Bluejay
  // gives no acknowledgement, so an ESC that silently ignores DShot command
  // 20/21 would still read "normal" here despite never having been told to be
  // anything else, and one that ignores 21 would misreport "reversed".
  Serial.print(F(",dir=")); Serial.println(spinReversed ? F("reversed") : F("normal"));

#if LOADCELL_NAU7802
  // The part is named because the columns mean the same thing but the counts do
  // not: a CSV taken on one factor and read as the other is silently wrong.
  Serial.print(F("#SCALE,nau7802,addr=0x"));
  Serial.print(NAU7802_ADDR, HEX);
  Serial.print(F(",drdy=")); Serial.print(NAU7802_DRDY_PIN);
  Serial.print(F(",rate_sps=")); Serial.println(loadCellSps);
#else
  Serial.print(F("#SCALE,hx711,dt=")); Serial.print(HX711_DT_PIN);
  Serial.print(F(",sck=")); Serial.print(HX711_SCK_PIN);
  Serial.print(F(",rate_sps=")); Serial.println(loadCellSps);
#endif

  Serial.print(F("#INA,name=")); Serial.print(INA.getDeviceName());
  Serial.print(F(",shunt_uohm=")); Serial.print(INA_SHUNT_UOHM);
  Serial.print(F(",max_a=")); Serial.print(INA_MAX_AMPS);
  Serial.print(F(",conv_us=")); Serial.print(INA_CONV_US);
  Serial.print(F(",avg=")); Serial.print(INA_AVERAGING);
  Serial.print(F(",shunt_fs_uv=")); Serial.println(INA_SHUNT_FS_UV);

  Serial.println(F("#FLAGS,1=ina_shunt_sat,2=ina_i_sat,4=thrust_stale,8=rpm_stale,"
                   "16=watchdog,32=esc_alert,64=edt_stale"));
  Serial.println(F("#PHASE,0=idle,1=sweep_hunt,2=sweep_up,3=transient,"
                   "4=sweep_down,5=response,6=coastdown,7=hold,8=spindown"));
  emitAmbient("banner");
  Serial.println(F("#COLS,t_us,phase,throttle_pct,dshot,rpm,erpm,erpm_raw,n_rpm,"
                   "thrust_raw,n_thrust,thrust_age_us,"
                   "bus_mv,bus_ua,shunt_uv,n_ina,ina_age_us,"
                   "esc_temp_c,esc_stress,esc_dbg1,esc_dbg2,n_edt,edt_age_us,flags"));
}

// Both derived quantities come from one read of the raw word. Storing them
// instead meant core 1 wrote them as separate stores and core 0 could read the
// set across that gap -- producing rows where rpm belonged to the next erpm.
// Deriving makes erpm == decode(erpm_raw) and rpm == erpm/(poles/2) true in
// every row by construction, which is what lets a run be re-derived under a
// corrected pole count afterwards.
//
// convertFromRaw() is the library's own decode and is `static`: it touches no
// ESC state, so core 0 may call it even though the esc object belongs to core 1.
// Reusing it keeps the mantissa/exponent unpacking in one place rather than a
// second copy here that could drift from the one the library uses.
static inline uint32_t erpmFromRaw(uint32_t raw) {
  return BidirDShotX1::convertFromRaw(raw, BidirDshotTelemetryType::ERPM);
}

static inline int32_t rpmFromErpm(uint32_t erpm) {
  uint8_t p = motorPoles;
  return (p >= 2) ? (int32_t)(erpm / (p / 2)) : 0;
}

// `consume` is false when the caller may still discard the row. FLAG_ESC_ALERT
// is consume-once -- flagged rows equal alert events -- so clearing it for a row
// that is then dropped would lose the event entirely.
uint8_t computeFlags(uint32_t now, bool consume = true) {
  uint8_t f = 0;
  if (abs(shuntMicroVolts) >= (INA_SHUNT_FS_UV * 98) / 100)      f |= FLAG_INA_SHUNT_SAT;
  if (abs(busMicroAmps) >= (int32_t)INA_MAX_AMPS * 980000L)      f |= FLAG_INA_I_SAT;
  if (now - lastThrustUpdateUs > THRUST_STALE_US)                f |= FLAG_THRUST_STALE;
  if (now - lastRpmUpdateUs > RPM_STALE_US)                      f |= FLAG_RPM_STALE;
  if (watchdogTripped)                                           f |= FLAG_WATCHDOG;
  // Only meaningful if we asked for EDT. Unasked, "stale" is the correct and
  // permanent state of a channel nobody requested, and flagging it buries the
  // flags that mean something.
  if (edtRequested && now - lastEdtUpdateUs > EDT_STALE_US)      f |= FLAG_EDT_STALE;
  // Consume-once: an alerting status frame flags exactly the next row printed,
  // so the number of flagged rows equals the number of alert events. Latching it
  // until the following status frame -- seconds later, or never once EDT stops
  // -- marked an entire 40 s sweep from one event, and since the plotter drops
  // flagged rows by default that silently deleted every row in the run.
  if (escAlertPending) {
    if (consume) escAlertPending = false;
    f |= FLAG_ESC_ALERT;
  }
  return f;
}

// Core 0 telling core 1 it is still making progress. Call it from anywhere core
// 0 can be spinning for a while with the motor turning; the cost is one 32-bit
// store. See core0HeartbeatMs for why core 1 is the one holding the stopwatch.
//
// Milliseconds rather than micros(): core 1 compares this against millis(), and
// a micros() stamp would wrap every 71 minutes and read as a stall.
static inline void heartbeat() {
  core0HeartbeatMs = millis();
}

// True once anything has asked the running sequence to stop early. Kept as a
// function so the reason for the stop stays in one place: core 1 having already
// killed the motor is as good a reason to unwind as an explicit STOP, and a
// sequence that kept stepping the throttle afterwards would be commanding a
// motor that is latched off.
static inline bool abortRequested() {
  return sequenceAbort || core0StallLatched;
}

// Rows are printed faster than the slow channels convert, so each one reports how
// old its thrust and INA samples are. Without this, print time is mistaken for
// sample time and the error is charged to the sensor: the HX711's ~33 ms sinc
// group delay is only separable from print staleness if the age is known.
//
// The row is dropped rather than written if it does not fit in the USB buffer.
// Serial.write() blocks while the host is not draining -- bounded at one second
// per call by the core, but a sequence prints at 250 Hz, so even bounded stalls
// of that size stop a run dead with the throttle held wherever it was. Logging
// is not worth blocking the loop that owns the motor.
void emitSample(uint8_t phase) {
  uint32_t now = micros();
  uint32_t rawNow  = currentErpmRaw;       // read once, derive both from that copy
  uint32_t erpmNow = erpmFromRaw(rawNow);
  int32_t  rpmNow  = rpmFromErpm(erpmNow);
  // Read without consuming: whether the ESC alert may be cleared depends on
  // whether this row actually gets written, which is not known yet.
  uint8_t flags = computeFlags(now, false);
  int n = snprintf(lineBuf, sizeof(lineBuf),
    "D,%lu,%u,%d,%u,%ld,%lu,%lu,%lu,%ld,%lu,%lu,%u,%ld,%ld,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%u\n",
    (unsigned long)now, phase, targetThrottlePercent, currentDshotValue,
    (long)rpmNow, (unsigned long)erpmNow, (unsigned long)rawNow,
    (unsigned long)rpmSeq,
    thrustRaw, (unsigned long)thrustSeq,
    (unsigned long)(now - lastThrustUpdateUs),
    busMilliVolts, (long)busMicroAmps, (long)shuntMicroVolts, (unsigned long)inaSeq,
    (unsigned long)(now - lastInaUpdateUs),
    escTempC, escStress, escDebug1, escDebug2, (unsigned long)edtSeq,
    (unsigned long)(now - lastEdtUpdateUs),
    flags);
  if (n <= 0) return;
  // Measured against the formatted length, not a worst-case row. Reserving the
  // full buffer instead dropped ~5% of rows against a merely slow reader on the
  // bench, which is a measurement regression rather than a safety margin.
  if (Serial.availableForWrite() < n) {
    rowsDropped++;
    return;
  }
  Serial.write(lineBuf, n);
  // Committed, so the alert this row carries may now be consumed -- and only if
  // this row is the one carrying it. A dropped row leaves escAlertPending set
  // and the next row inherits the event, which is what keeps "flagged rows equal
  // alert events" true whether or not any rows were dropped.
  if (flags & FLAG_ESC_ALERT) escAlertPending = false;
}

// Call at the start of every sequence, with the throttle already at zero.
// escMaxStress and escAlertCount were previously never cleared, so #STATS
// reported a since-boot maximum while claiming to report the sequence's.
void resetSequenceCounters() {
  telemOkCount = 0;
  telemBadCount = 0;
  telemCorruptCount = 0;
  telemSilentCount = 0;
  edtFrameCount = 0;
  escMaxStress = 0;
  escAlertCount = 0;
  escAlertPending = false;
  rowsDropped = 0;
  // A trip belongs to the run it happened in. watchdogTripped is otherwise
  // cleared only by a bare throttle number or DSHOT, so one trip flagged every
  // row of every later sequence -- and watchdog is a fatal flag, so the plotter
  // dropped all of them and usable_point rejected the run. Seen on the bench
  // 2026-08-20: a deliberate watchdog test poisoned the next sweep completely.
  watchdogTripped = false;
  // Some ESCs drop EDT once armed, so ask again while we are stopped.
  if (edtRequested) edtEnablePending = EDT_ENABLE_REPEATS;
  // Unconditional, unlike the EDT re-request above: direction must never be
  // inherited from whatever a previous sequence (or a previous script run
  // that never power-cycled the ESC) left latched in the ESC's RAM.
  spinCmdPending = SPIN_CMD_REPEATS;
}

void emitStats(const char *tag) {
  uint32_t ok = telemOkCount, bad = telemBadCount;
  uint32_t tot = ok + bad;
  Serial.print(F("#STATS,")); Serial.print(tag);
  Serial.print(F(",telem_ok=")); Serial.print(ok);
  Serial.print(F(",telem_bad=")); Serial.print(bad);
  Serial.print(F(",telem_corrupt=")); Serial.print(telemCorruptCount);
  Serial.print(F(",telem_silent=")); Serial.print(telemSilentCount);
  Serial.print(F(",telem_rate_pct=")); Serial.print(tot ? (100.0 * ok / tot) : 0.0, 2);
  Serial.print(F(",edt_frames=")); Serial.print(edtFrameCount);
  Serial.print(F(",esc_max_stress=")); Serial.print(escMaxStress);
  Serial.print(F(",esc_alerts=")); Serial.print(escAlertCount);
  Serial.print(F(",rows_dropped=")); Serial.print(rowsDropped);
  Serial.print(F(",i2c_probe_reads=")); Serial.print(i2cProbeReads);
  Serial.print(F(",i2c_probe_bad=")); Serial.print(i2cProbeBad);
  Serial.print(F(",i2c_probe_last=0x")); Serial.print(i2cProbeLast, HEX);
  Serial.print(F(",esc_temp_c=")); Serial.print(escTempC);
  Serial.print(F(",esc_volts=")); Serial.println(escVoltsQuarter * 0.25, 2);
}

// =========================================================================
// Sensors
// =========================================================================

#if LOADCELL_NAU7802
// config.h carries the rate as a plain SPS number, because that is the figure
// every other part of this project reasons about -- group delay, row rate, the
// #SCALE banner. The register wants an enum whose values are not the rate, so
// the translation lives here rather than making the config lie about itself.
// Note there is no 160 SPS step.
NAU7802_SampleRate nau7802RateEnum(int sps) {
  switch (sps) {
    case 10:  return NAU7802_RATE_10SPS;
    case 20:  return NAU7802_RATE_20SPS;
    case 40:  return NAU7802_RATE_40SPS;
    case 320: return NAU7802_RATE_320SPS;
    default:  return NAU7802_RATE_80SPS;
  }
}
#endif

// --- Load cell abstraction ----------------------------------------------
// Four operations, so serviceThrust() and tareWithTimeout() read the same on
// both parts and the safety analysis above stays in one piece. Note the
// blocking behaviour differs and the backstop covers both: HX711::read() is
// unbounded, while the NAU7802's reads go through Wire and cap at ~1 s each.
// Bounded is not harmless at 250+ rows/s, so do NOT read this as licence to
// remove the core 1 backstop -- see the comment above.

// True when a fresh conversion is waiting. Called every pass of loop(), so it
// must stay cheap.
bool loadCellReady() {
#if LOADCELL_NAU7802
  if (NAU7802_DRDY_PIN >= 0) return digitalRead(NAU7802_DRDY_PIN) == HIGH;
  // No DRDY pin wired: fall back to the register, but time-gate it. available()
  // is an I2C transaction, and polling it every loop pass would flood the bus
  // the INA226 and BMP280 share.
  static uint32_t lastPollUs = 0;
  uint32_t now = micros();
  if (now - lastPollUs < (1000000UL / (uint32_t)loadCellSps) / 2) return false;
  lastPollUs = now;
  return scale.available();
#else
  return scale.is_ready();
#endif
}

// Tared raw counts. Conversion to grams is the host's job -- see config.h.
long loadCellReadTared() {
#if LOADCELL_NAU7802
  return scale.read() - thrustOffset;
#else
  return scale.get_value(1);
#endif
}

long loadCellReadRaw() {
#if LOADCELL_NAU7802
  return scale.read();
#else
  return scale.read();
#endif
}

void loadCellSetOffset(long offset) {
#if LOADCELL_NAU7802
  thrustOffset = offset;
#else
  scale.set_offset(offset);
#endif
}

// Move the conversion rate without a reflash. False means the rate was not one
// the fitted part offers, or that the part has no say -- the HX711's rate is
// the RATE pin. The caller must reprint the banner on success: rate_sps is what
// a reader uses to judge group delay and whether rows outpaced conversions, so
// a CSV recording the compiled rate after a change would be wrong in the one
// field that explains its own noise.
// Read the revision register and score it. Called on the same cadence as a
// conversion so the two counts are comparable. Cheap: one byte over I2C.
void loadCellProbeI2C() {
#if LOADCELL_NAU7802
  Wire.beginTransmission(NAU7802_ADDR);
  Wire.write(0x1F);
  if (Wire.endTransmission(false) != 0) {   // repeated start, as the reads do
    i2cProbeReads++; i2cProbeBad++; i2cProbeLast = 0;
    return;
  }
  if (Wire.requestFrom((uint8_t)NAU7802_ADDR, (uint8_t)1) != 1) {
    i2cProbeReads++; i2cProbeBad++; i2cProbeLast = 0;
    return;
  }
  uint8_t v = Wire.read();
  i2cProbeReads++;
  if ((v & 0x0F) != 0x0F) { i2cProbeBad++; i2cProbeLast = v; }
#endif
}

bool loadCellSetRate(int sps) {
#if LOADCELL_NAU7802
  // Defensive: callers are expected to check loadCellInitOk and report the
  // reason themselves, since "bad rate" and "dead ADC" need different messages.
  // This is here so a future caller that forgets cannot reach calibrate().
  if (!loadCellInitOk) return false;
  if (sps != 10 && sps != 20 && sps != 40 && sps != 80 && sps != 320) return false;
  scale.setRate(nau7802RateEnum(sps));
  // A rate move invalidates the front end's calibration, which is why setup1()
  // runs one straight after setting gain and rate. Same reason here.
  if (!scale.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println(F("#WARN,nau7802_calibrate_failed"));
  }
  loadCellSps = sps;
  return true;
#else
  (void)sps;
  return false;
#endif
}

// Polled as fast as the loop allows; loadCellReady() gates on the ADC's own
// conversion cadence, so we catch each conversion with minimal added latency.
void serviceThrust() {
  if (!loadCellReady()) return;
  thrustRaw = loadCellReadTared();
  // One probe per conversion, so i2c_probe_reads and the number of ADC reads
  // are the same order and the two rates can be compared directly.
  loadCellProbeI2C();
  thrustSeq++;
  lastThrustUpdateUs = micros();
}

void serviceIna() {
  uint32_t now = micros();
  if (now - lastInaReadUs < INA_PERIOD_US) return;
  lastInaReadUs = now;
  busMilliVolts   = INA.getBusMilliVolts();
  busMicroAmps    = INA.getBusMicroAmps();
  shuntMicroVolts = INA.getShuntMicroVolts();
  inaSeq++;
  lastInaUpdateUs = now;
}

// Every blocking call in setup() gets a deadline, and each step announces itself
// first. Losing the serial link costs every diagnostic there is: the board runs,
// never reaches SYSTEM_READY, says nothing about why, and BOOTSEL is the only
// way back. The library's own tare() waits forever for a load cell that is not
// answering, which is exactly that failure.
bool tareWithTimeout(uint32_t timeoutMs, uint8_t samples = 10) {
  uint32_t deadline = millis() + timeoutMs;
  int64_t sum = 0;
  uint8_t got = 0;
  while (got < samples && (int32_t)(millis() - deadline) < 0) {
    if (loadCellReady()) {
      sum += loadCellReadRaw();
      got++;
    }
  }
  if (got == 0) return false;
  loadCellSetOffset((long)(sum / got));
  return true;
}

void serviceAmbient() {
  if (!ambientPresent) return;
  uint32_t now = micros();
  if (now - lastAmbientReadUs < AMBIENT_PERIOD_US) return;
  lastAmbientReadUs = now;
  ambientTempC = bmp.readTemperature();
  ambientPressPa = bmp.readPressure();
}

void serviceSensors() {
  serviceThrust();
  serviceIna();
  serviceAmbient();
}

// Raw temperature and pressure only -- air density is derived host-side so the
// formula has exactly one implementation.
void emitAmbient(const char *tag) {
  Serial.print(F("#AMBIENT,")); Serial.print(tag);
  if (!ambientPresent) {
    Serial.println(F(",present=0"));
    return;
  }
  Serial.print(F(",present=1,temp_c="));
  Serial.print(ambientTempC, 2);
  Serial.print(F(",press_pa="));
  Serial.println(ambientPressPa, 1);
}

void setThrottle(int percent) {
  targetThrottlePercent = constrain(percent, 0, 100);
  currentDshotValue = (targetThrottlePercent == 0)
                      ? 0
                      : map(targetThrottlePercent, 1, 100, 1, 2000);
}

// Commands a DShot value directly, bypassing the percent map above.
//
// DShot carries 2000 throttle steps and integer percent reaches only 100 of
// them, so ~20 steps go unused per percent. That is invisible in the middle of a
// sweep and decisive at the top: near full throttle one percent is several
// hundred RPM, coarser than the telemetry's own 1.5 us lattice, which means a
// 1%-stepped sweep cannot resolve where a plateau begins or ends. `dshot` is
// already logged in every row, so a run driven this way is indexed on the exact
// commanded value; targetThrottlePercent is still filled in, rounded, purely so
// the throttle_pct column stays readable.
void setThrottleRaw(int dshotValue) {
  dshotValue = constrain(dshotValue, 0, 2000);
  currentDshotValue = (uint16_t)dshotValue;
  // Inverse of map(pct, 1, 100, 1, 2000), rounded rather than truncated.
  targetThrottlePercent = dshotValue ? 1 + ((dshotValue - 1) * 99 + 999) / 1999 : 0;
}

// Ends a sequence by walking the throttle down instead of dropping it.
//
// A step to zero makes the ESC brake hard, and its braking behaves as a boost
// converter: the rotor's energy goes into the bus. A lab supply sources but
// cannot sink, so the rail rises. Measured at 6.2 V from 56 kRPM *with a prop
// fitted*; with the prop off there is no aerodynamic sink at all, and a
// spin-down from 80 kRPM tripped an 8 V supply OVP -- against an ESC rated 8.4.
//
// The stored energy goes as speed squared and the peak regen power as that over
// the stopping time, so spreading the same joules over ~400 ms instead of a few
// tens of milliseconds is most of the problem solved. Windage and bearing drag
// absorb a good share of it once the deceleration is gentle enough.
//
// Fixed decrement rather than a fixed number of steps, so ending a sweep at 7%
// costs 40 ms while ending a full-throttle hold costs 400 ms.
//
// NOT used where the spin-down is itself the measurement -- TRANSIENT and
// COASTDOWN both log the decay from a hard step, and ramping those would
// measure the ramp. With no prop and a non-sinking supply, those two sequences
// are the ones that can still drive the rail up; run them with the prop on.
//
// Also not used by STOP or the watchdog: those are the paths a person or a dead
// host uses to stop a spinning propeller, and they stay immediate.
// Logged at the normal sequence rate under PHASE_SPINDOWN, so the stop is
// visible instead of being the one unmeasured moment in a run. The INA updates
// every 8.8 ms, so a 400 ms ramp carries ~45 fresh bus readings -- enough to
// see the shape of the rise, not just whether a supply cutout fired. Tuning the
// rate against a curve rather than against a trip flag is the whole point.
const uint16_t SPINDOWN_STEP_DSHOT = 100;
const uint16_t SPINDOWN_STEP_MS    = 20;
// Logged after the setpoint reaches zero: the rotor is still turning there, and
// that is where the last of the energy arrives.
const uint16_t SPINDOWN_TAIL_MS    = 100;

void runWindow(uint32_t durationUs, uint8_t phase, uint32_t logAfterUs);

void rampThrottleDown() {
  int value = (int)currentDshotValue;
  while (value > 0) {
    value -= SPINDOWN_STEP_DSHOT;
    if (value < 0) value = 0;
    setThrottleRaw(value);
    runWindow((uint32_t)SPINDOWN_STEP_MS * 1000UL, PHASE_SPINDOWN, 0);
  }
  setThrottle(0);
  runWindow((uint32_t)SPINDOWN_TAIL_MS * 1000UL, PHASE_SPINDOWN, 0);
}

// =========================================================================
// CORE 0: user input, sensors, PC logging
// =========================================================================
void setup() {
  Serial.begin(115200);
  uint32_t serialTimeout = millis();
  while (!Serial && (millis() - serialTimeout < 3000)) delay(10);

  Serial.println(F("#INIT,i2c"));
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  // Bound every I2C transaction and reset the peripheral on a timeout. The
  // default is a second per transfer, which core 0 spends inside serviceIna()
  // or serviceAmbient() with the motor turning and nothing being serviced. The
  // bus has hung with the motor rail live before and it is still
  // unexplained; 25 ms is far longer than any read here needs and short
  // enough that a hung bus degrades the sample rate instead of the run.
  Wire.setTimeout(25, true);

  Serial.println(F("#INIT,ina"));
  if (!INA.begin(INA_MAX_AMPS, INA_SHUNT_UOHM)) {
    Serial.println(F("#ERROR,ina_not_found"));
  }
  INA.setBusConversion(INA_CONV_US);
  INA.setShuntConversion(INA_CONV_US);
  INA.setAveraging(INA_AVERAGING);

  // BMP280 breakouts ship strapped to either address; try both rather than
  // making the wiring a build-time decision.
  Serial.println(F("#INIT,bmp280"));
  ambientPresent = bmp.begin(0x76) || bmp.begin(0x77);
  if (ambientPresent) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,   // temperature
                    Adafruit_BMP280::SAMPLING_X16,  // pressure
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_500);
    ambientTempC = bmp.readTemperature();
    ambientPressPa = bmp.readPressure();
  }

#if LOADCELL_NAU7802
  Serial.println(F("#INIT,nau7802"));
  if (!scale.begin(&Wire)) {
    Serial.println(F("#ERROR,nau7802_no_response"));
  } else {
    // Order matters: the LDO must come up before a conversion is meaningful,
    // and the part needs a fresh internal calibration after gain or rate move.
    scale.setLDO(NAU7802_LDO_SETTING);
    scale.setGain(NAU7802_GAIN_SETTING);
    scale.setRate(nau7802RateEnum(loadCellSps));
    // Internal offset calibration. Reported by the part, not by us, and it is
    // separate from the tare below: this zeroes the ADC's own front end, the
    // tare zeroes the mounted stand.
    if (!scale.calibrate(NAU7802_CALMOD_INTERNAL)) {
      Serial.println(F("#WARN,nau7802_calibrate_failed"));
    }
    if (NAU7802_DRDY_PIN >= 0) pinMode(NAU7802_DRDY_PIN, INPUT);
    // Conversions, not just a configured part: a tare that times out means
    // nothing is converting, and SPS would then reach a part that cannot
    // answer. begin() succeeding is necessary and not sufficient.
    loadCellInitOk = tareWithTimeout(3000);
    if (!loadCellInitOk) {
      Serial.println(F("#ERROR,nau7802_no_response"));
    }
  }
#else
  Serial.println(F("#INIT,hx711"));
  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  loadCellInitOk = tareWithTimeout(3000);
  if (!loadCellInitOk) {
    Serial.println(F("#ERROR,hx711_no_response"));
  }
#endif

  Serial.println(F("#INIT,done"));
  emitBanner();
  Serial.println(F("SYSTEM_READY"));
  lastHostMsgMs = millis();
}

void handleCommand(const char *cmd) {
  lastHostMsgMs = millis();

  // Sequences now poll the serial line while they run, which means commands can
  // arrive somewhere they never could before. Only the two that can make a
  // spinning motor safer are honoured there; everything else is refused rather
  // than acted on, because a sequence owns the throttle and the stopped/running
  // state machine for its whole duration. Without this, a SWEEP arriving
  // mid-SWEEP would recurse into a second one sharing the same globals.
  //
  // PING is honoured because refusing it would starve the host watchdog that
  // now runs during sequences, and STOP because it is the abort.
  if (inSequence && strcmp(cmd, "PING") && strcmp(cmd, "STOP")) {
    Serial.print(F("#ERROR,busy,")); Serial.println(cmd);
    return;
  }

  if (!strcmp(cmd, "PING")) {
    if (!watchdogArmed) {
      watchdogArmed = true;
      Serial.println(F("#WD,armed"));
    }
    return;
  }
  if (!strcmp(cmd, "ID")) { emitBanner(); return; }

  // Bus scan. "The sensor is missing" and "the sensor is at an address we do
  // not probe" look identical from the outside; this separates them.
  if (!strcmp(cmd, "SCAN")) {
    Serial.println(F("#SCAN,begin"));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print(F("#SCAN,found,0x"));
        if (addr < 16) Serial.print('0');
        Serial.println(addr, HEX);
        found++;
      }
    }
    Serial.print(F("#SCAN,end,count="));
    Serial.println(found);
    return;
  }
  // Telemetry counters on demand. #STATS otherwise appears only after a
  // sequence, so with the motor stopped there is no way to tell a working EDT
  // link from a single noise frame that happened to pass checksum -- and a
  // stale esc_temp_c or esc_stress looks exactly like a live one.
  // Rotor magnet count. Rejected for odd or out-of-range values: RPM =
  // eRPM/(poles/2) is integer division, so an odd count would silently truncate
  // rather than fail. Mid-sequence arrivals are already refused above.
  if (!strncmp(cmd, "POLES,", 6)) {
    int n = atoi(cmd + 6);
    if (n < 2 || n > 32 || (n % 2)) {
      Serial.print(F("#ERROR,poles_invalid,")); Serial.println(n);
    } else {
      motorPoles = (uint8_t)n;
      Serial.print(F("#POLES,")); Serial.println(motorPoles);
    }
    return;
  }
  // Conversion rate for this session. Only the NAU7802 can honour it; on the
  // HX711 the rate is the RATE pin and the command is refused, because silently
  // accepting it would leave the host believing a rate the part is not running.
  // The calibration inside loadCellSetRate() disturbs the zero, so re-tare
  // afterwards -- every sequence already does, and `--check --tare` does.
  if (!strncmp(cmd, "SPS,", 4)) {
    int n = atoi(cmd + 4);
    // Two different failures, kept apart on purpose. sps_invalid means the rate
    // is not one this part offers and the board is fine; sps_unavailable means
    // the ADC never came up, and the command is refused rather than allowed to
    // reach an unconfigured part.
    if (!loadCellInitOk) {
      Serial.println(F("#ERROR,sps_unavailable,loadcell_init_failed"));
    } else if (!loadCellSetRate(n)) {
      Serial.print(F("#ERROR,sps_invalid,")); Serial.println(n);
    } else {
      emitBanner();
    }
    return;
  }
  // Extended telemetry on or off for this session, so an ESC that supports it
  // does not need a reflash to be asked and one that does not is silent rather
  // than flagging every row. Enabling takes effect immediately: the request is
  // only valid with the motor stopped, which is exactly where we are.
  if (!strncmp(cmd, "EDT,", 4)) {
    int n = atoi(cmd + 4);
    if (n != 0 && n != 1) {
      Serial.print(F("#ERROR,edt_invalid,")); Serial.println(n);
    } else {
      edtRequested = (n == 1);
      edtEnablePending = edtRequested ? EDT_ENABLE_REPEATS : 0;
      Serial.print(F("#EDT,")); Serial.println(edtRequested ? 1 : 0);
    }
    return;
  }
  // Bluejay's temporary spin-direction commands (20/21) -- see the comment by
  // spinReversed. The send happens only while stopped; mid-sequence arrivals
  // are refused by the guard at the top of this function.
  if (!strncmp(cmd, "SPIN,", 5)) {
    int n = atoi(cmd + 5);
    if (n != 0 && n != 1) {
      Serial.print(F("#ERROR,spin_invalid,")); Serial.println(n);
    } else {
      spinReversed = (n == 1);
      spinCmdPending = SPIN_CMD_REPEATS;
      Serial.print(F("#SPIN,")); Serial.println(spinReversed ? 1 : 0);
    }
    return;
  }
  if (!strcmp(cmd, "STATS")) { emitStats("idle"); return; }
  // Immediate, never ramped -- STOP is the path a person uses to stop a
  // spinning propeller, and the regen ramp is a measurement nicety by
  // comparison. Mid-sequence it is also the abort: zeroing the throttle alone
  // would last until the sequence commanded its next step 100 ms later.
  if (!strcmp(cmd, "STOP")) {
    setThrottle(0);
    if (inSequence) sequenceAbort = true;
    return;
  }
  if (!strcmp(cmd, "TARE")) {
    setThrottle(0);
    if (tareWithTimeout(3000)) {
      Serial.println(F("#TARE,ok"));
    } else {
      Serial.println(F("#ERROR,hx711_no_response"));
    }
    return;
  }
  if (!strcmp(cmd, "SWEEP")) { runSweepSequence(); return; }
  if (!strcmp(cmd, "TRANSIENT")) { runTransientSequence(); return; }

  if (!strncmp(cmd, "RESPONSE", 8)) {
    int args[3] = {40, 70, 5};   // base %, high %, repetitions
    parseArgs(cmd, args, 3);
    runResponseSequence(args[0], args[1], args[2]);
    return;
  }
  // Raw-DShot hold, for walking a band finer than integer percent allows. Tested
  // before HOLD purely for readability -- the prefixes differ in the first
  // character, so they cannot be confused by strncmp.
  if (!strncmp(cmd, "DHOLD", 5)) {
    int args[3] = {1400, 1000, 1500};   // dshot value, log ms, settle ms
    parseArgs(cmd, args, 3);
    runHoldSequence(0, args[1], args[2], args[0]);
    return;
  }
  if (!strncmp(cmd, "HOLD", 4)) {
    int args[3] = {70, 1000, 1500};   // throttle %, log ms, settle ms
    parseArgs(cmd, args, 3);
    runHoldSequence(args[0], args[1], args[2]);
    return;
  }
  // Direct DShot setpoint, the raw-value counterpart of a bare throttle number.
  if (!strncmp(cmd, "DSHOT,", 6)) {
    setThrottleRaw(atoi(cmd + 6));
    watchdogTripped = false;
    return;
  }
  if (!strncmp(cmd, "COASTDOWN", 9)) {
    int args[1] = {70};          // throttle % to decay from
    parseArgs(cmd, args, 1);
    runCoastdownSequence(args[0]);
    return;
  }

  if (cmd[0] >= '0' && cmd[0] <= '9') {
    setThrottle(atoi(cmd));
    watchdogTripped = false;
  }
}

// Re-entrant, and it has to be: a sequence command dispatched from here runs the
// whole sequence, and sequences poll serial themselves, so this function calls
// itself one level deep.
//
// The command is therefore copied out and inputLen reset BEFORE dispatch. With
// the reset after the call, as it was originally, the inner poll appended its
// characters to a buffer still holding the outer command -- `STOP` sent during a
// hold arrived as `HOLD,0,8000,500STOP`, was refused as an unknown busy command,
// and the abort never happened. Caught on the bench 2026-08-20; the whole
// mid-sequence STOP path was inert until this was fixed.
//
// Recursion is bounded at one level: only PING and STOP are accepted while
// inSequence, and neither starts a sequence.
void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLen > 0) {
        char cmd[sizeof(inputBuf)];
        memcpy(cmd, inputBuf, inputLen);
        cmd[inputLen] = '\0';
        inputLen = 0;
        handleCommand(cmd);
      }
    } else if (inputLen < sizeof(inputBuf) - 1) {
      inputBuf[inputLen++] = c;
    }
  }
}

// Runs during sequences too, now that runWindow() and sequenceDelay() poll the
// serial line. It used to bail on inSequence, which exempted it from the whole
// 40 s of a sweep -- exactly the window where a dead host leaves a propeller
// spinning with nobody watching, and where the operator cannot intervene either.
//
// A trip inside a sequence has to abort it as well as zero the throttle: the
// sequence would otherwise command the next step 100 ms later and undo the stop.
void serviceWatchdog() {
  if (!watchdogArmed) return;
  // One shot per silence: cleared by the next throttle command from the host.
  // Without this the end-of-sequence ramp, which commands a falling throttle 20
  // times, would re-trip and re-print on every step once the host had gone.
  if (watchdogTripped) return;
  if (targetThrottlePercent == 0) return;
  if (millis() - lastHostMsgMs > WATCHDOG_MS) {
    setThrottle(0);
    watchdogTripped = true;
    if (inSequence) sequenceAbort = true;
    Serial.println(F("#WARN,watchdog_timeout"));
  }
}

void loop() {
  heartbeat();
  pollSerial();
  serviceSensors();
  serviceWatchdog();

  uint32_t now = micros();
  if (now - lastPrintUs >= PRINT_IDLE_US) {
    lastPrintUs = now;
    emitSample(PHASE_IDLE);
  }
}

// =========================================================================
// Sequence helper: run sensors + logging for a fixed window
// =========================================================================
// Returns early once an abort is requested, so a sequence stops at the next
// sample rather than at the end of the current step. The spin-down is exempt:
// rampThrottleDown() runs on this same path and is the safe stop itself, so
// aborting it would leave the throttle wherever the ramp had reached.
void runWindow(uint32_t durationUs, uint8_t phase, uint32_t logAfterUs) {
  uint32_t windowStart = micros();
  uint32_t lastLog = 0;
  while (micros() - windowStart < durationUs) {
    heartbeat();
    pollSerial();
    serviceSensors();
    serviceWatchdog();
    if (phase != PHASE_SPINDOWN && abortRequested()) return;
    uint32_t elapsed = micros() - windowStart;
    if (elapsed >= logAfterUs) {
      uint32_t now = micros();
      if (now - lastLog >= PRINT_FAST_US) {
        lastLog = now;
        emitSample(phase);
      }
    }
  }
}

// delay() with the loop still alive: heartbeat fed, serial polled, watchdog
// serviced, sensors kept current. Every wait inside a sequence goes through
// this rather than delay().
//
// Bare delay() was the reason the settling waits could not be interrupted -- a
// 10 s DHOLD settle was 10 s during which STOP was not read and the core 1
// backstop would have fired on a perfectly healthy board. Nothing here logs a
// row: these are the windows a sequence has deliberately chosen to discard.
void sequenceDelay(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    heartbeat();
    pollSerial();
    serviceSensors();
    serviceWatchdog();
    if (abortRequested()) return;
  }
}

// The common ending for every sequence, abort or not.
//
// An aborted sequence still emits its stats, its closing ambient snapshot and
// its END_ sentinel, in the same order as a completed one. The host blocks on
// that sentinel with no deadline of its own, so a sequence that returned
// without printing one would hang the run rather than end it -- and the rows
// already collected would be lost with it. What the host gets instead is a
// short run plus the #WARN saying why.
//
// The spin-down ramp is skipped on abort: an abort has already set the throttle
// to zero, deliberately and immediately, and ramping from zero does nothing.
void endSequence(const char *statsTag, const __FlashStringHelper *sentinel) {
  bool aborted = abortRequested();
  if (!aborted) rampThrottleDown();
  setThrottle(0);
  if (aborted) {
    // The three are worth telling apart in a CSV: a stall means the board needs
    // a power cycle, a watchdog trip means the host died, a stop means a person
    // decided to end the run.
    if (core0StallLatched)   Serial.println(F("#WARN,sequence_aborted,core0_stall"));
    else if (watchdogTripped) Serial.println(F("#WARN,sequence_aborted,watchdog"));
    else                      Serial.println(F("#WARN,sequence_aborted,stop"));
  }
  emitStats(statsTag);
  emitAmbient("end");
  Serial.println(sentinel);
  inSequence = false;
  sequenceAbort = false;
}

// =========================================================================
// SWEEP: 1% throttle steps, 100ms settle (discarded) + 100ms logged
// =========================================================================
void runSweepSequence() {
  inSequence = true;
  setThrottle(0);
  sequenceDelay(500);

  resetSequenceCounters();

  emitAmbient("start");
  Serial.println(F("START_SWEEP"));

  // Phase A: find the lowest throttle that sustains rotation
  int reliableStartThrottle = 0;
  for (int t = 1; t <= 15; t++) {
    setThrottle(t);
    sequenceDelay(350);
    if (abortRequested()) break;

    bool stable = true;
    for (int check = 0; check < 10; check++) {
      sequenceDelay(10);
      if (rpmFromErpm(erpmFromRaw(currentErpmRaw)) < 2500) { stable = false; break; }
    }
    if (stable) {
      reliableStartThrottle = t;
      Serial.print(F("MIN_RELIABLE_IDLE,"));
      Serial.println(reliableStartThrottle);
      break;
    }
  }

  if (reliableStartThrottle == 0) {
    // Previously this silently fell back to 7 and produced a sweep that looked
    // normal but started below the motor's actual reliable idle. An abort during
    // the hunt lands here too, and reports itself through endSequence().
    if (!abortRequested()) Serial.println(F("#ERROR,idle_hunt_failed"));
    setThrottle(0);
    endSequence("sweep", F("END_SWEEP"));
    return;
  }

  // Phase B: 1% steps up
  for (int t = reliableStartThrottle; t <= 100; t++) {
    setThrottle(t);
    runWindow(200000, PHASE_SWEEP, 100000);
    if (abortRequested()) break;
  }

  // Phase C: the same steps back down. A one-directional sweep always carries a
  // settling-lag bias in the direction it travels (low on the way up); averaging
  // the two legs cancels it, and the up/down gap is a free check on whether
  // 100 ms of settling is actually enough. The down leg runs hotter, so part of
  // any gap is thermal drift rather than lag -- report it, do not average it away
  // without looking.
  for (int t = 99; t >= reliableStartThrottle && !abortRequested(); t--) {
    setThrottle(t);
    runWindow(200000, PHASE_SWEEP_DOWN, 100000);
  }

  endSequence("sweep", F("END_SWEEP"));
}

// =========================================================================
// TRANSIENT: 0 -> 100% -> 0 step response
// =========================================================================
void runTransientSequence() {
  inSequence = true;
  setThrottle(0);
  sequenceDelay(500);

  resetSequenceCounters();

  const uint32_t PHASE_US = 600000;

  emitAmbient("start");
  Serial.println(F("START_TRANSIENT"));
  setThrottle(100);
  runWindow(PHASE_US, PHASE_TRANSIENT, 0);
  setThrottle(0);
  runWindow(PHASE_US, PHASE_TRANSIENT, 0);

  endSequence("transient", F("END_TRANSIENT"));
}

// =========================================================================
// RESPONSE: repeated steps between two non-zero throttles
//
// The TRANSIENT sequence steps from a standstill, where ~200 ms of the response
// is the ESC's arm-and-start sequence -- something that never happens in flight.
// A PID loop commands steps around hover, so that is what this measures.
// =========================================================================
void runResponseSequence(int base, int high, int reps) {
  inSequence = true;
  base = constrain(base, 1, 99);
  high = constrain(high, base + 1, 100);
  reps = constrain(reps, 1, 20);

  setThrottle(0);
  sequenceDelay(500);
  resetSequenceCounters();

  emitAmbient("start");
  Serial.print(F("START_RESPONSE,"));
  Serial.print(base); Serial.print(',');
  Serial.print(high); Serial.print(',');
  Serial.println(reps);

  setThrottle(base);
  sequenceDelay(1000);  // settle at base before the first edge

  for (int i = 0; i < reps && !abortRequested(); i++) {
    setThrottle(high);
    runWindow(300000, PHASE_RESPONSE, 0);
    if (abortRequested()) break;
    setThrottle(base);
    runWindow(300000, PHASE_RESPONSE, 0);
  }

  endSequence("response", F("END_RESPONSE"));
}

// =========================================================================
// COASTDOWN: unpowered deceleration
//
// Gives an ESC-independent spin-down time constant, and with a known drag law
// the rotor+prop inertia. Whether it is genuinely unpowered depends on the ESC's
// brake setting -- with braking enabled this measures the brake, not the drag.
// Record the ESC config with the run.
// =========================================================================
void runCoastdownSequence(int fromThrottle) {
  inSequence = true;
  fromThrottle = constrain(fromThrottle, 10, 100);

  setThrottle(0);
  sequenceDelay(500);
  resetSequenceCounters();

  emitAmbient("start");
  Serial.print(F("START_COASTDOWN,"));
  Serial.println(fromThrottle);

  setThrottle(fromThrottle);
  sequenceDelay(1500);                  // reach a stable speed
  runWindow(200000, PHASE_COAST, 0);    // log the steady state we decay from
  setThrottle(0);
  runWindow(1500000, PHASE_COAST, 0);   // the decay itself

  endSequence("coastdown", F("END_COASTDOWN"));
}

// =========================================================================
// HOLD: settle at one throttle, log a window, stop.
//
// Deliberately generic. The KV protocol is built on it host-side (hold at 100%
// while the operator steps the bench supply), as are check-mass and calibration
// runs, so none of those need their own firmware mode.
// =========================================================================
// One implementation behind both entry points: HOLD takes a throttle percent,
// DHOLD a raw DShot value (dshotRaw >= 0). Everything after the throttle is set
// is identical, and duplicating it would let the two drift apart in precisely
// the places -- settling time, window length, counter reset -- where a
// difference would be invisible in the resulting data.
void runHoldSequence(int throttle, int logMs, int settleMs, int dshotRaw) {
  inSequence = true;
  throttle = constrain(throttle, 0, 100);
  logMs = constrain(logMs, 100, 10000);
  settleMs = constrain(settleMs, 0, 10000);

  resetSequenceCounters();

  if (dshotRaw >= 0) setThrottleRaw(dshotRaw);
  else               setThrottle(throttle);

  emitAmbient("start");
  // The commanded DShot value is reported alongside the percent: with DHOLD the
  // percent is a rounded label and the raw value is the actual setpoint.
  Serial.print(F("START_HOLD,"));
  Serial.print(targetThrottlePercent); Serial.print(',');
  Serial.print(logMs); Serial.print(',');
  Serial.print(settleMs); Serial.print(',');
  Serial.println(currentDshotValue);

  sequenceDelay(settleMs);
  if (!abortRequested()) runWindow((uint32_t)logMs * 1000UL, PHASE_HOLD, 0);

  endSequence("hold", F("END_HOLD"));
}

// =========================================================================
// CORE 1: dedicated DShot transmission + telemetry decode
// =========================================================================
void setup1() {
  delay(500);
  esc = new BidirDShotX1(ESC_PIN, DSHOT_RATE);
  uint32_t startTime = millis();
  while (millis() - startTime < 3000) {
    esc->sendThrottle(0);
    delay(2);
  }

  // Request extended telemetry, if this build or this session asked for it.
  // Must be sent repeatedly with the motor stopped; an ESC that does not
  // support it simply ignores the command. Re-requested at the start of every
  // sequence too -- see resetSequenceCounters().
  if (edtRequested) edtEnablePending = EDT_ENABLE_REPEATS;
  // Assert "normal" (the boot default of spinReversed) before anything can
  // spin, in case the ESC has a temporary reverse latched in RAM from a
  // previous session that never power-cycled it. Re-asserted every sequence
  // too -- see resetSequenceCounters() and the comment by spinReversed.
  spinCmdPending = SPIN_CMD_REPEATS;
}

// Every EDT frame advances the same counter, whatever its type: the host uses it
// only to tell a fresh esc_* reading from the previous one held over, and every
// frame type proves the link is alive.
static inline void markEdtFrame() {
  edtFrameCount++;
  edtSeq++;
  lastEdtUpdateUs = micros();
  telemOkCount++;
}

// Core 1's own safety check, and the only one in the firmware that survives a
// core 0 that has stopped running. Deliberately kept ahead of every other
// decision in loop1: whatever else is pending, a motor whose commanded value
// nobody is updating any more must be stopped.
//
// Only armed while the motor is actually turning. Core 0 legitimately blocks
// for seconds at a time with the throttle at zero -- tareWithTimeout() waits up
// to 3 s for the HX711, and setup() waits on the USB host -- and none of that is
// dangerous.
static inline bool core0Stalled() {
  if (core0StallLatched) return true;
  if (currentDshotValue == 0) return false;
  if (millis() - core0HeartbeatMs <= CORE0_STALL_MS) return false;
  core0StallLatched = true;
  return true;
}

void loop1() {
  // The stall override replaces only what is SENT. Telemetry is still decoded
  // below, on purpose: discarding it froze currentErpmRaw at its last value, so
  // a latched board reported the speed it had been doing when core 0 died and
  // read as a motor still spinning. That cost a wrong verdict on the bench
  // (2026-08-20) before the current channel settled it, and it would mislead
  // anyone reading the rows after a stall just as badly. rpm_stale alone is not
  // enough of a hint when the number beside it looks plausible.
  if (core0Stalled()) {
    // Not rampThrottleDown(): that runs on core 0, which is by definition not
    // running. The regen spike this risks is the lesser hazard by a wide
    // margin, and the ESC's own braking is all that is left to us here.
    esc->sendThrottle(0);
  }
  // A command frame where a throttle frame belongs is only safe while stopped;
  // with the motor turning it would be read as a throttle value.
  else if (edtEnablePending && currentDshotValue == 0) {
    esc->sendRaw11Bit(DSHOT_CMD_EDT_ENABLE);
    edtEnablePending--;
  } else if (spinCmdPending && currentDshotValue == 0) {
    esc->sendRaw11Bit(spinReversed ? DSHOT_CMD_SPIN_DIRECTION_REVERSED
                                    : DSHOT_CMD_SPIN_DIRECTION_NORMAL);
    spinCmdPending--;
  } else {
    esc->sendThrottle(currentDshotValue);
  }
  delayMicroseconds(200);

  // Raw rather than decoded: eRPM frames are stored as the transmitted word and
  // decoded on core 0 (see erpmFromRaw), so erpm_raw is logged beside erpm and a
  // plateau in an RPM curve can be checked against what the ESC actually sent.
  // For every other frame type the raw word is the value, masked to 8 bits, so
  // this costs those paths nothing.
  uint32_t raw = 0;
  BidirDshotTelemetryType ret = esc->getTelemetryRaw(&raw);
  uint32_t value = raw & 0xFF;

  // ERPM with value 0 is a valid "motor stopped" report, and EDT frames are
  // valid telemetry rather than failures -- decoding only eRPM counted every
  // temperature and stress frame as a link error.
  switch (ret) {
    case BidirDshotTelemetryType::ERPM:
      // A zero mantissa encodes no period at all, so it cannot be a speed --
      // getTelemetryPacket() reported that as a checksum error and this keeps
      // the same accounting rather than storing a word that decodes to a
      // division by zero. Checked here so core 0 only ever sees a valid word.
      if (raw != ERPM_RAW_STOPPED && (raw & 0x1FF) == 0) {
        telemBadCount++;
        telemCorruptCount++;
        break;
      }
      currentErpmRaw = raw;
      rpmSeq++;
      lastRpmUpdateUs = micros();
      telemOkCount++;
      break;
    case BidirDshotTelemetryType::TEMPERATURE:
      escTempC = (uint8_t)value; markEdtFrame();
      break;
    case BidirDshotTelemetryType::VOLTAGE:
      escVoltsQuarter = (uint16_t)value; markEdtFrame();
      break;
    case BidirDshotTelemetryType::CURRENT:
      escAmps = (uint16_t)value; markEdtFrame();
      break;
    case BidirDshotTelemetryType::STRESS:
      escStress = (uint8_t)value;
      if (value > escMaxStress) escMaxStress = value;
      markEdtFrame();
      break;
    case BidirDshotTelemetryType::STATUS:
      escStatus = (uint8_t)value;
      if (value & (ESC_STATUS_ERROR_MASK | ESC_STATUS_WARNING_MASK |
                   ESC_STATUS_ALERT_MASK)) {
        escAlertCount++;
        escAlertPending = true;
      }
      markEdtFrame();
      break;
    // Stock Bluejay sends constants here (0x88 / 0xAA), so the values were
    // discarded. tools/patch_bluejay_debug.py rewrites those two stubs to send
    // Comm_Period4x_L and _H instead -- the ESC's own commutation-period
    // accumulator, which is the one thing that separates a wrong period
    // measurement from a wrong encoding. Storing them costs nothing when the
    // frames are still carrying constants.
    case BidirDshotTelemetryType::DEBUG_FRAME_1:
      escDebug1 = (uint8_t)value; markEdtFrame();
      break;
    case BidirDshotTelemetryType::DEBUG_FRAME_2:
      escDebug2 = (uint8_t)value; markEdtFrame();
      break;
    case BidirDshotTelemetryType::OTHER_VALUE:
      markEdtFrame();
      break;
    // These two mean opposite things and were previously indistinguishable.
    // A corrupt frame says the ESC is talking and the line is noisy; silence
    // says it is not talking at all -- wrong pin, broken wire, a pull-up strong
    // enough that the ESC cannot pull the line low, or an ESC not in
    // bidirectional mode. Only one of those is a signal-integrity problem.
    case BidirDshotTelemetryType::CHECKSUM_ERROR:
      telemBadCount++;
      telemCorruptCount++;
      break;
    default:   // NO_PACKET
      telemBadCount++;
      telemSilentCount++;
      break;
  }

  // Re-ask for EDT when it was requested and nothing is answering.
  //
  // The enable is queued in setup1() and in resetSequenceCounters(), and both
  // are events rather than states: boot the Pico against a dead ESC rail and
  // those frames go nowhere, and `--check` triggers no sequence, so nothing
  // re-requests. The banner then reads edt=1 beside zero frames, which is
  // indistinguishable from an ESC that has no EDT support -- the one thing the
  // edt= field exists to tell apart. Measured on the bench 2026-09-06: the same
  // --check gave 0 frames booted against a dead rail and 1,345 in 8 s booted
  // against a live one.
  //
  // This does not paper over a real finding. An ESC that never answers still
  // ends the run with edt_frames=0 and FLAG_EDT_STALE set, because retrying
  // changes nothing about what came back. What it removes is the case where the
  // stand never asked in a way the ESC could hear.
  //
  // Gated on currentDshotValue == 0 for the same reason the send chain above is:
  // a command frame where a throttle frame belongs would be read as a throttle
  // value. So this can never fire during a sequence.
  if (edtRequested && !edtEnablePending && currentDshotValue == 0) {
    uint32_t nowUs = micros();
    // Both comparisons are on unsigned differences, so the ~72 minute micros()
    // wrap needs no special case.
    if (nowUs - lastEdtUpdateUs > EDT_STALE_US &&
        nowUs - lastEdtRetryUs > EDT_RETRY_US) {
      edtEnablePending = EDT_ENABLE_REPEATS;
      lastEdtRetryUs = nowUs;
    }
  }
}
