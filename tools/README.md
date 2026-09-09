# tools/

Everything here is optional. `measure.py` and `plot_comparison.py` in the
repository root are the stand; these are the instruments around it, the
emulator that lets you work without it, and the one-off experiments that
answered specific questions about the hardware.

They fall into six groups:

| tool | group | what it is for |
|---|---|---|
| [`fake_stand.py`](#fake_standpy) | testing | firmware emulator over a pty, to exercise the host with no bench |
| [`fake_psu.py`](#fake_psupy) | testing | pty that answers like the bench supply, so the Kv PSU path runs without hardware |
| [`kv_dry_run.py`](#kv_dry_runpy) | testing | end-to-end dry run of a Kv sweep, asserting when the mid-hold reading is taken |
| [`psu.py`](#psupy) | instrument | drive a SCPI bench supply, so a Kv sweep needs no knob |
| [`scope.py`](#scopepy) | instrument | capture waveforms from an OWON HDS handheld scope |
| [`phase_probe.py`](#phase_probepy) | instrument | scope captures inside a logged hold: phase voltage at full throttle, or the INA-sense-to-ESC span |
| [`blade_tone.py`](#blade_tonepy) | reference | rotor RPM from sound, with no part of the ESC involved |
| [`tone_hold.py`](#tone_holdpy) | reference | one uninterrupted hold, to record a tone against |
| [`soak_test.py`](#soak_testpy) | experiment | does reported speed decay with running time, and recover on rest |
| [`approach_test.py`](#approach_testpy) | experiment | does the direction of approach change the reported speed |
| [`spindown_probe.py`](#spindown_probepy) | experiment | what the rail does while the motor gives its energy back, to tune the ramp rate |
| [`bus_check.py`](#bus_checkpy) | analysis | count I2C transfers the bus mangled, live or in a saved run |
| [`block_compare.py`](#block_comparepy) | analysis | one block of repeated sweeps against another: block means, spread and step-feature deviation |
| [`branch_probe.py`](#branch_probepy) | experiment | provoke and detect the half-speed branch, the open drive defect |
| [`sweep_monotonic.py`](#sweep_monotonicpy) | analysis | flag sweeps where RPM falls as throttle rises, before their numbers get used |
| [`patch_bluejay_debug.py`](#patch_bluejay_debugpy) | ESC | patch a released Bluejay hex to stream its internal commutation period |
| [`am32.py`](#am32py) | ESC | read, change and flash an AM32 ESC over 4-Way, so its firmware and settings stop being a declared value |
| [`flash_board.py`](#flash_boardpy) | ESC | swap the board between the stand firmware and the ESC passthrough in one command |

## Conventions

**Run them from the repository root**, as `python tools/<name>.py`. Several
import `measure.py` from the parent directory or hand it a port to drive, and
they locate it relative to their own file rather than the working directory.

**Serial ports.** Everything that talks to the stand takes `-p/--port` and
defaults to the same device `measure.py` does. The two instrument tools address
their own hardware: `psu.py` defaults to a fixed port, while `scope.py` finds
the scope by USB vendor/product id. That is worth knowing, because a USB-serial
supply and a USB-serial scope both land in `/dev/ttyUSB*` and which one gets `0`
depends on the order you plugged them in.

**Dependencies.** `pyserial` for anything touching hardware, `numpy` for
`blade_tone.py`, `matplotlib` only for its `--plot`. All in the root
`requirements.txt`.

**Anything that spins the motor** inherits the same two safety mechanisms as
`measure.py`: a keepalive thread arming the firmware watchdog, and a `finally`
that commands zero throttle. If you write another tool here, keep both.

---

## Testing without hardware

The emulator covers a protocol that runs to completion. The failure paths live
in `tests/` at the repo root, which drives `measure.py` through an in-memory
serial peer instead of a pty: a board that goes quiet, a firmware abort, a
Ctrl-C partway through a mass check. Run them with
`python -m unittest discover -s tests`. Neither is a firmware test; that is
still hardware work.

### `fake_stand.py`

A firmware emulator. It opens a pty, speaks telemetry schema 1, and runs the
real `measure.py` against it, so the entire host path can be exercised, and
regressions caught, with nothing connected.

It is not a stub. It reproduces the timing that motivated the schema in the
first place: rows print at 250 Hz while the emulated HX711 updates at 80 Hz and
the INA at 100 Hz, so a host that averages printed rows instead of
deduplicating on the sequence counters gets visibly wrong answers.
`--thrust-sps` inverts that relationship, which is the case a faster ADC brings:
above the row rate, conversions are lapped and never reach a row at all, and the
host has to say so rather than quietly average the survivors. It carries
ground-truth motor and load-cell models, so the Kv fit and the MASSCHECK fit can
be checked against known values rather than eyeballed. The model also tracks
`SPIN,<0|1>` and reverses the sign of the thrust it reports (at ~50% of
forward magnitude, the ratio measured on the real bench)
rather than leaving thrust positive regardless of commanded direction; without
that, any host-side check comparing declared rotation against the load cell's
sign (`measure.py`'s `check_rotation_sign`) would pass against the emulator no
matter whether the check itself is right.

```bash
python tools/fake_stand.py -m SWEEP -o /tmp/out.csv
python tools/fake_stand.py -m KV                      # fit should recover the true Kv
python tools/fake_stand.py --serve                    # expose a pty, print its path, idle
python tools/fake_stand.py -m SWEEP -- --prop-diameter-mm 31   # args after -- go to measure.py
python tools/fake_stand.py -m SWEEP -- --reverse       # thrust reads negative, ~50% magnitude
python tools/fake_stand.py -m SWEEP --load-cell nau7802 --thrust-sps 320  # faster ADC than row rate
```

Fault injection is the point of it. Each of these is a failure the host is
supposed to notice and report:

| flag | emulates |
|---|---|
| `--shunt-uohm 100000` | a mismatched shunt railing the current channel partway up every sweep |
| `--mass-slope 1.03` | a load cell 3% off the stored factor; MASSCHECK should say so and correct it |
| `--tare-early` | taring mid-transient during a mass check; the fit should survive it |
| `--no-edt` | an ESC that ignored the EDT enable command (pair with `-- --edt`) |
| `--no-bmp` | the ambient sensor missing from the I²C bus |
| `--no-rpm-lattice` | an ESC without the period quantisation Bluejay shows. Use it to model an AM32 ESC, which reports consecutive integer microseconds with no lattice at all |
| `--period-bias LO,HI,PCT` | speed under-reported over one RPM band |
| `--stale-rpm-every N` | flag every Nth printed row `rpm_stale`. The host must keep the step and record the count in `n_rpm_stale` rather than discarding it |
| `--load-cell nau7802` | the other supported ADC in the `#SCALE` banner. Pair with `--thrust-sps 320` for the full NAU7802 case |
| `--thrust-sps 800` | a load cell ADC converting faster than rows are printed, so rows carry only a subsample. The host must report `n_thrust_missed` and warn, not average over the gap. **320 no longer does this**: `PRINT_FAST_US` moved to 1500 µs on 2026-09-02 and the emulator's row rate follows it |
| `--mangle-rate P` | an I²C bus corrupting that fraction of thrust transfers, top byte `0xAA`, reading about +314 g. The host must reject them before forming a mean and count them into `n_thrust_mangled` |
| `--mangle-ina-rate P` | the same bus corrupting that fraction of INA226 bus-voltage reads by 1 to 2.5 V. Still a plausible voltage, so the host must reject it against the step median rather than because it looks wrong |
| `--loadcell-dead` | an ADC that answers on the bus but never converts, as on the bench when VIN was left unconnected. Rows read stale and `SPS,<n>` must be refused with `sps_unavailable`, never passed to the part |

`--serve` is the one to use when driving the emulator from something other than
`measure.py`.

### `fake_psu.py`

A pty that answers like the Kiprim DC310S, so the `--psu-port` branch of
`run_kv` can run without a supply. `fake_stand.py` deliberately emulates the
firmware and not the supply, which left that branch with no hardware-free test;
this is the missing half.

```bash
python tools/fake_psu.py --log /tmp/psu.log     # prints the pty path, then serves
python tools/fake_psu.py --cell 3.7             # model a parallel cell holding the rail up
python tools/fake_psu.py --output-off           # start with the output off
```

Every command is logged with a timestamp. It exists to check *when* the host
talks to the supply: the one mid-hold reading must land one settle delay after
the supply settles, not one settle delay plus one log window (which is where it
lands if it is taken after `END_HOLD`). It answers only to CRLF, like the real
unit.

It models the output cap as a first-order lag so `settle()` has something real
to converge on, and the setpoint's ~20 mV optimism against the supply's own
meter. `--cell` holds the rail up like a parallel 18650, which is what makes
the low end of a Kv sweep unreachable with one fitted.

### `kv_dry_run.py`

The test that goes with `fake_psu.py`: wires the firmware emulator and the fake
supply to the real `measure.py`, runs one Kv sweep, then reads the supply's
command log back to prove *when* the mid-hold reading was taken. One second of
separation between the fixed behaviour and the bug, which is not a subtle call.

```bash
python tools/kv_dry_run.py
```

The sweep is pinned to one DShot level (`--kv-dshots 2000`), so each voltage
step has exactly one mid-hold reading. The default two levels would put two
readings in one step and confuse the gap split the assertion is built on.

Exits nonzero if any step shows more than one reading after the settle gap, or
a reading that lands past `END_HOLD`. Writes `data/psu_cmds.log` and
`data/kv_dry.csv`.

---

## Instrument control

### `psu.py`

Drives a programmable bench supply over SCPI, so a Kv sweep sets and verifies
each voltage itself instead of depending on an operator dialling a knob and
pressing Enter. `measure.py -m KV --psu-port <dev>` uses it directly; the CLI
here is for setup and diagnosis.

```bash
python tools/psu.py --probe                  # identity, setpoints, and a check against stand.json
python tools/psu.py --set-voltage 3.4
python tools/psu.py --set-current 5
python tools/psu.py --output on
```

Verified against a Kiprim DC310S. Three of its behaviours are worth knowing
before trusting any supply here:

- **Every write is read back**, because the supply has no usable error queue.
  `SYST:ERR?` answers `0x0000` unconditionally, and a rejected setpoint silently
  leaves the previous one in place. In a Kv sweep that is a step that did not
  happen, producing a plausible wrong slope rather than an error.
- **Its over-voltage limit cannot protect a 1S cell.** `VOLT:LIM` enforces a
  1.0 V margin above the setpoint, so the tightest available limit at 4.0 V is
  5.0 V, well past a lithium cell's 4.2 V ceiling. That guard therefore lives
  in this module, as `--max-volts`, and refuses rather than warns.
- **It answers only to CRLF.** A bare newline gets total silence, which looks
  exactly like a dead device.

`--probe` compares the supply against `stand.json`'s declared `psu_voltage_v`,
which is the one declared value that can be read back from hardware.

### `scope.py`

Captures traces from an OWON HDS handheld. Some questions on this bench are
about shape rather than magnitude: whether the ESC chops at full throttle, what
commutation looks like, how far the bus rises during a spin-down. The INA226
converts every 8.8 ms, so it cannot see any of them.

```bash
python tools/scope.py --probe                              # state of both channels
python tools/scope.py --capture CH1 --coupling DC -o data/phase.csv
python tools/scope.py --calibrate                          # probe on the 1 kHz comp output
```

Every capture carries the scope's own header JSON (scales, coupling, probe
factor), so a trace cannot be misread later because the front panel moved in
between.

Two things to know:

- **Set `--coupling DC` for anything with a DC component.** The scope powers up
  AC-coupled, which silently removes exactly the part of a PWM waveform or a
  supply rail you are trying to measure.
- **Both axes are calibrated, and neither was assumed.** Vertical is signed
  bytes at 25 counts per division, cross-checked against the scope's own
  `:MEAS:CH1:MAX?`. Horizontal was measured with `--calibrate` against the fixed
  1 kHz compensation output: exactly 250.00 samples per 1 ms period, zero
  spread, giving a 12-division screen and `dt = timebase_per_div / 50`. Captures
  carry real seconds. Re-run `--calibrate` if you use a different model.
- **Ignore the header's `SAMPLERATE` when working out time.** It is the
  acquisition rate into the 8K deep memory, 2.5 MSa/s at 200 µs/div, while the
  600 screen points are decimated from it, ten times slower at that setting.
  Both figures are right and they describe different things.
- **Setting the timebase over SCPI is unreliable**, so `set_timebase()` reads it
  back and raises rather than capturing at a timebase nobody chose. Writes were
  accepted with run status `AUTo` and silently ignored with `TRIG`, including a
  string that had worked moments earlier. Use the knob.

A general warning that came out of building this: on the HDS242, `:FUNC?` and
`:FUNC:FREQ?` answer with plausible values, but the model has **no signal
generator**; that firmware is shared with the HDS242S. A reply from an
instrument is not evidence the feature exists.

### `phase_probe.py`

Runs a logged `DHOLD` and takes scope captures from inside the logged window,
so the trace and the telemetry describe the same event, for the same reason
`run_kv` reads the supply mid-hold rather than after `END_HOLD`. Two
measurements use it: whether the ESC chops at full throttle (phase voltage at
duty 1), and the INA-sense-to-ESC span (two channels, differenced).

```bash
python tools/phase_probe.py --channels CH1,CH2 --dshot 2000
python tools/phase_probe.py --dshot 384 --log-ms 10000 --trigger-below 13500
```

Captures are spaced through the window rather than all at once, so a one-off
artefact is distinguishable from the steady waveform.

**Filenames carry a UTC timestamp unless you pass `--tag`.** The fixed default
silently overwrote a healthy baseline on 2026-09-06, and a trace of a fault that
fades is not something you can go back and retake.

**`--trigger-below N` waits for a state instead of sampling on a schedule.** Use
it for anything the machine does briefly and unpredictably: a round fires on the
first fresh telemetry frame below `N`, then re-arms, so several excursions in one
window each get a trace. Rows repeating a stale `rpm` cannot arm it, and neither
can a failed start under `MIN_RUNNING_RPM`.

Three things to hold onto when using it:

- **Trigger deep, not close.** A threshold on the shoulder of an excursion fires
  while the machine is still in the state you were trying to leave behind.
- **It improves the odds; it does not guarantee the trace.** `capture()` reads
  whatever acquisition is on the scope's screen, because the instrument offers
  no single-shot arm-and-wait, so a capture can still describe the state either
  side of a brief event.
- **A window that never fired is one quiet window, not a fix.** The run says so
  explicitly and prints the minimum speed it saw, so a quiet result carries its
  own margin.

**Every capture is stamped with the machine's state while it was being
digitised**: `# state,rpm_mean=...,rpm_min=...,rpm_max=...,amps=...` in the
trace's own header, from the rows that arrived between the start and end of that capture.
This is not bookkeeping. A motor that moves between operating points *within* a
hold makes the run's mean describe no single state, and an unstamped trace then
cannot be attributed to either one. The run warns when the speed was not steady,
and marks any capture the machine changed state during. Three traps:

- **Take the zero offsets first with the rail off.** Both channels read their
  offset against a true 0 V, and that calibration is what makes the differenced
  span trustworthy. The `verify-*-zero.csv` captures in `data/` are the worked
  example.
- **A round's two channels are captured back to back, not simultaneously.** At
  the default timebase each capture takes about a second, so CH1 and CH2
  describe nearby but not identical commutation cycles: difference means, not
  samples. A round taken late in a short window can also land after the motor
  has stopped; check the trace before using it.

---

## An RPM reference the ESC has no part in

Every speed figure the stand produces comes from the ESC's own telemetry, so
telemetry cannot arbitrate a question about itself. A propeller turning at *n*
rev/s with *B* blades radiates a tone at *n·B*, which is a property of the air:

```
RPM = 60 · f_peak / blades
```

### `blade_tone.py`

Extracts that tone from a recording. Any recorder will do; the point is that
nothing in the chain passes through the ESC.

```bash
python tools/blade_tone.py hold.wav --blades 3 --expect-rpm 55000
python tools/blade_tone.py hold.wav --compare data/hold.csv     # telemetry beside the acoustic answer
python tools/blade_tone.py hold.wav --plot tone.png
```

Two ways to get a wrong answer from it, both avoidable:

- **Track in a band only a few percent wide.** The blade tone is *not* the
  loudest thing in the spectrum. A 2-per-rev component sits below it and is
  stronger, so an over-wide `--tolerance` can rail at a band edge and invent a
  slope. The default is deliberately wider than the effects being chased but no
  wider.
- **Segment the recording to the burst first.** Frames of silence track noise.
  `--skip` discards spool-up from the start.

`--blades` wrong by a factor is every RPM wrong by the same factor.

### `tone_hold.py`

The other half: one uninterrupted logged hold, so a recording and the telemetry
cover the same physical event.

```bash
python tools/tone_hold.py -o data/tone-hold.csv     # start recording at the countdown
python tools/blade_tone.py hold.wav --compare data/tone-hold.csv
```

Two slopes measured over one continuous event need no clock alignment between
recorder and stand, which is what makes the comparison airtight. The firmware
caps a logged hold at 10 s, and this refuses a longer `--log-ms` rather than
letting the board clamp it silently.

---

## Experiments

> Skimmable. These are one-off investigations kept because the questions recur;
> none of them is part of the measurement workflow, so skip them entirely on
> first read. Each is a worked example of how to interrogate the stand.

### `soak_test.py`

Repeats one setpoint many times, rests, and repeats again, to turn "the speed
seems to sag over a run" into a curve: how fast it decays, how far, and how
quickly rest undoes it.

```bash
python tools/soak_test.py --dshot 2000 --holds 8 --rests 60,240
```

Thrust is the arbiter here, not power. Thrust goes as *n²* so it measures true
speed directly, whereas power at constant voltage looks nearly identical whether
the rotor slowed or the telemetry drifted. The cell is re-tared before every
hold, because `HOLD` does not re-tare and the zero walks.

It stops on a stall and on a configurable decay limit (`--abort-drop-pct`). An
early version drove two full-throttle holds into a desynced motor.

### `approach_test.py`

Holds one setpoint but arrives at it from above in one case and from below in
the other, with everything else identical. A commutation-filter dead zone
predicts the two will report different speeds while voltage and current stay
put; a real speed difference would move the current too.

```bash
python tools/approach_test.py --targets 1930,1940,1950
```

Approach is set up with `DSHOT`, a direct setpoint that does not spin down, then
`DHOLD` logs the target without passing through zero.

### `branch_probe.py`

Provokes the half-speed branch and says whether it appeared. The motor
intermittently mis-commutates into a slower branch: 100-400 ms excursions from
~18,000 to ~12,000 RPM at **higher** current. Current rising through the dip is
what separates it from lost telemetry.

```bash
python tools/branch_probe.py                                   # th 20, 10 attempts
python tools/branch_probe.py --throttles 14,16,18,22 --attempts 3
```

**When to run it is an open question, so record the conditions.** On 2026-08-20
provocability faded within a session: 4 of 8, then 1 of 10, then 0 of 12 across
an hour with nothing changed. On 2026-09-06 it did the opposite, arriving after
ninety minutes of sweeps and stall testing and gone fifteen minutes later. Each
result now carries its own ESC temperature span so the two can eventually be
compared; neither of those sessions recorded it, which is why they cannot be.

It jumps to the setpoint from a standstill every time, and that is the point
rather than an implementation detail: arriving by a 1% staircase from idle, or
by stepping down from th 50, suppressed the fault entirely across 36 s.

**A start that never happens is reported as `FAILED START`, not as a dip.** A
motor crawling at a few hundred RPM would otherwise register excursions against
its own median and the run would declare the fault live; that happened on
2026-09-06 at `startup_power=50`. Anything whose upper branch is under 2,000 RPM
is treated as a failed start.

**A clean run of attempts is not a fix**, an error made twice on this bench.
When the fault is live, take the two measurements that need it: the old ~19 kKV
motor swap, and a scope trace via `phase_probe.py` triggered well below the
upper branch the probe just reported. Trigger deep rather than close: about
11,000 RPM when that branch sits near 15,000.

### `spindown_probe.py`

Reads the bus during a ramped stop, so the ramp rate can be tuned on data.
`-m KV` and friends write per-step summaries, so the phase-8 rows the firmware
emits during the stop never reach a CSV; this drives one `DHOLD` directly and
keeps every row.

```bash
python tools/spindown_probe.py --volts 3.5 --dshot 2000
```

Reports the peak the rail reaches while the motor gives its energy back, and
the headroom to the 8 V OVP the ramp exists to avoid. `--no-psu` skips the
supply, for a battery-run stop. As with every other serial tool, `-p/--port`
chooses the stand and `--psu-port` the supply.

---

## Checking a run before you trust it

### `bus_check.py`

Counts I2C transfers the bus mangled. On this stand the bus corrupted whenever
the bench supply was switched on -- about one transfer in a thousand, at zero
load, with the ESC disconnected and no current flowing. That was measured in
2026-09-01 and removed by the 2026-09-02 rewire, with the mechanism never
identified, so treat it as that bench's history rather than a live fault. This
tool is how that was measured and how any fix gets scored.

```bash
python tools/bus_check.py --seconds 300              # one window, motor stopped, rail ON
python tools/bus_check.py --seconds 300 --repeat 4   # what a comparison actually needs
python tools/bus_check.py --csv data/sweep.csv       # score a saved SWEEP
```

Live, it counts two independent things. The **probe** is register 0x1F of the
NAU7802, a constant whose low nibble is 0xF, read once per conversion by the
firmware and reported as `i2c_probe_reads` / `i2c_probe_bad` in `#STATS`. A
wrong constant can only be a wrong *transfer*, which is what separates a
corrupted read from a corrupted conversion. The **outliers** are corrupted
thrust samples themselves, printed in hex because the pattern is the clue: on
this bench the top byte is 0xAA every time while the lower bytes vary.

**A clean result is weaker than a dirty one**, because the ADC read is three
bytes to the probe's one and so is more exposed per transaction. Read the two
counts together, never the mismatch alone.

**One window proves nothing.** The rate fluctuates run to run beyond Poisson:
four runs in an evening under the same conditions gave 37, 37, 24 and 11 where
a stable process near 28 has a standard deviation of 5.3. Use `--repeat` and
compare totals with the printed error bar.

**The rail must be ON.** With the supply off the fault does not occur, so a
clean result there is the control and measures nothing.

The `--csv` mode needs an aggregated run with a `thrust_g_std` column, because a
corrupted sample raises its own step's stdev by orders of magnitude. It reads
columns by *name*: a `RESPONSE` file has a different column order, and an
index-based scan silently compares the wrong two columns. On the sweeps taken
2026-09-01 it finds 12 suspect steps in the NAU7802 run and none in the HX711
one.

Do not re-test bus speed, the DRDY pin, the DShot signal wire or the pull-ups;
all four are ruled out by measurement.

### `sweep_monotonic.py`

Flags sweeps where RPM *falls* while commanded throttle *rises*. A healthy sweep
is monotonic within each leg; a step that goes the wrong way means the motor
answered a higher setpoint with a lower speed, which is either mis-commutation
or lost drive on a phase. Either way the numbers around it are not usable.

```bash
python tools/sweep_monotonic.py data/*.csv          # exits non-zero if any flag
python tools/sweep_monotonic.py run.csv --dip-pct 5 --min-rpm 8000
```

Reads only sweep CSVs (`direction`, `rpm_mean`, `rpm_std`, `watts_mean`) and
says so rather than guessing if handed something else.

**Why per-leg, and not up-versus-down.** Comparing the two legs at the same
throttle is the obvious test and it does not work: a motor spins up more easily
on the way down, so 10-25% gaps just above idle are ordinary and a sweep that is
clean by every other measure will show several of them. Judged that way, healthy
runs fail. Within a single leg there is no such effect to subtract, so the check
needs no cross-leg comparison and cannot be fooled by hysteresis.

Two gates keep it honest. `--min-rpm` excludes the bottom of the sweep, where a
percentage is taken on near-zero numbers and the idle hunt alone produces tens
of percent. `--dip-pct` sets the drop worth reporting. Put it below the
run-to-run scatter of your rig and everything flags.

`asym` counts steps where the legs disagree *and* the slower one draws more
power. That inversion, more energy in and less speed out, is what separates a
commutation fault from settling lag or thermal drift.

**Both measures are needed and neither is sufficient.** A leg that enters a
collapsed state and stays there is internally monotonic: throttle falls, speed
falls, just far too little of it. So `dips` sees nothing and only the
comparison against the other leg reveals it. A large, power-inverted asymmetry
therefore flags on its own, at `--asym-flag-pct`, set well above the 10–25% that
ordinary hysteresis produces near idle. A handful of small `asym` steps below
that threshold is normal and does not affect the verdict. `scatter` is the worst
per-step `rpm_std/rpm` over the top half of the speed range, which catches a
motor dithering between two commutation states inside each step before that
becomes a dip.

Calibrate the thresholds against runs you already trust before relying on the
verdict. The defaults were set so that known-good sweeps report zero dips and
known-bad ones report several.

---

## ESC firmware

### `patch_bluejay_debug.py`

Rewrites two instructions in a released Bluejay hex so its debug EDT frames
carry `Comm_Period4x`, the ESC's own internal commutation-period accumulator,
instead of hardcoded constants.

```bash
python tools/patch_bluejay_debug.py in.hex --verify    # locate the stubs, write nothing
python tools/patch_bluejay_debug.py in.hex -o out.hex
```

It needs no assembler and no toolchain. `MOV direct,#imm` and `MOV
direct,direct` are both three bytes on the 8051, so each stub is rewritten in
place with no relocation and no length change, and the frame-id stores are left
alone so the frames still decode normally.

What it buys: the period the ESC *believes* internally, streamed out beside the
period it *reports*, which is what separates a wrong period measurement from a
wrong encoding.

Flashing Bluejay is done from a flight controller. Bluejay runs on a SiLabs
EFM8, and this repository has no route to that bootloader, so reaching it is
out of scope. That is a fact about Bluejay rather than a policy: on an ARM ESC
(AM32, BLHeli_32) the passthrough route is open again, and `am32.py` below is
it. See also `docs/HARDWARE.md`.

### `am32.py`

Reads, changes and flashes an AM32 ESC through the vendored 4-Way passthrough.

```bash
python tools/am32.py read                     # decode the settings page
python tools/am32.py read --raw               # and print all 192 bytes
python tools/am32.py set variable_pwm=0 motor_kv=40
python tools/am32.py dump -o shipped.bin      # back up the application image
python tools/am32.py flash AM32_x_2.21.hex --verify-only
python tools/am32.py flash AM32_x_2.21.hex
```

It needs the board running `vendor/BlHeli-Passthrough/rp2040/` rather than
`firmware/`, and the ESC powered. Flash the stand firmware back afterwards.
The passthrough is a separate sketch on purpose: integrating it would hand over
core 1's PIO pin, block core 0 and share the host's serial line.

What it buys: `esc_firmware` in `stand.json` is a **declared** value, because
nothing in bidirectional DShot carries a firmware version. A stale declaration
is recorded confidently into every CSV and looks right forever. On an ARM ESC
the settings page can be read back and checked, which is the only thing that
makes it falsifiable.

Four things worth knowing before you use it.

**Back up before you flash.** `dump` writes the application region to a file.
The bootloader is never erased so a failed write is recoverable, but a vendor's
shipped image is not downloadable if it predates the public releases.

**The settings page has layout versions and is not self-describing.** Byte 1
says which, and this tool decodes version 4, from AM32 2.21's `Inc/eeprom.h`.
`show` warns on any other layout, and `set` refuses -- before erasing anything
-- because writing version-4 field offsets onto a version-1/2/3 page would put
every value in the wrong byte and the verify would still pass. On anything else
read `--raw` and check the field names against the firmware you flashed rather
than trusting the labels.

**Check every field after a firmware update.** AM32 2.21 migrates an older page
in `loadEEpromSettings`, and that migration is incomplete: it clears the bytes
where layout 1 kept a 12-byte firmware name string, but not the last two, which
layout 4 reads as `input_type` and `auto_advance`. An ESC can come up with a
feature enabled by a leftover ASCII character.

**Some settings decide whether a measurement means anything**, and the tool
prints a note beside those. `brake_on_stop` and `brake_on_zero_throttle` have
to be 0 or `COASTDOWN` measures the brake rather than the rotor's free decay.
`variable_pwm` has to be 0 or the switching frequency moves with throttle.
`motor_kv` is not a motor property at all: it scales AM32's low-RPM duty
ceiling, so setting it to a high-KV motor's real value can cap a sweep partway
up while throttle keeps rising.

### `flash_board.py`

Swaps the board between the two sketches, one command per direction.

```bash
python tools/flash_board.py esc      # then: python tools/am32.py read
python tools/flash_board.py stand    # then: python measure.py --check
```

Reading or changing one ESC setting means running the passthrough and then
putting the stand firmware back. Two flashes per change is enough friction to
discourage checking a setting at all, which is the wrong incentive for a value
that is otherwise only declared.

It exists in this shape because **`arduino-cli upload` cannot finish on a host
with no auto-mounter**. It resets the board into its bootloader correctly and
then fails with "No drive to deploy", because the RPI-RP2 volume is never
mounted and `udisksctl` has no polkit agent over ssh. So the two halves are
done separately: a 1200 baud open-and-close puts the RP2040 in BOOTSEL, and
`picotool load` writes over USB without mounting anything.

**The wait for re-enumeration at the end is the point, not padding.**
`picotool` returns as soon as it has rebooted the board, well before the USB
CDC device is back. Anything that opens the port immediately after gets "No
such file or directory", which reads like a dead board rather than an impatient
script. That cost an evening once.

Builds go to a directory under the system temp dir, so nothing lands in the
repository. `--build-dir` overrides it.


## block_compare.py

Plots one block of repeated sweeps against another and puts the ratio in its
own panel.

    block_compare.py "A=data/branchbase_[0-9]_*.csv" "B=data/motorb_[0-9]_*.csv"

`plot_comparison.py` answers whether a curve has the right shape. This answers
how two sets of runs differ, which is a different job and needs a different
figure: sixteen overlaid traces that differ by two percent are unreadable, and
the same two percent is obvious as a ratio.

**The ratio panel is the tool.** Motor A's 66% throttle step was invisible in
the overlaid curves and in a matched-RPM table, and unmissable as a ratio.

Two or three blocks, with ratios taken against the first, so name the reference
block first. The per-step panel marks every throttle step deviating 35% or more
from its local trend, which is where PWM entrainment locks show up; the trace
behind it is context, the dots are the finding.

**A fourth block needs a hue that passes the colour-vision check**, not the next
one that looks different. The obvious green fails deuteranopia separation
against the orange already in use.

Current against RPM is plotted because the load cell is not in that path, so
two blocks taken at different scale factors, which any pair either side of a
mount swap will be, can still be compared honestly. Power against thrust is
plotted for what the setup delivers, calibration included.

Steps at or above `--amp-limit` are dropped as supply-limited and the excluded
fraction is printed. A current-limited top end is a property of the supply, and
the lossier motor sags the rail further and reads as if it were weaker.
