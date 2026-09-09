# Thrust stand

An open propeller and motor test stand. An RP2040 drives an ESC over
bidirectional DShot600 while logging thrust, RPM, voltage, current and ambient
conditions; Python scripts on the host command test sequences and record CSV.

Built for 1S tiny-whoop hardware, but nothing in it is specific to that class.
The sizes of the load cell and current shunt are configuration, not assumptions.

The goal is a **reproducible, publishable motor/propeller database**: real
measured Kv, efficiency at a given thrust, and spin-up/spin-down responsiveness.
That goal drives most of the design decisions here: measurement provenance
matters more than convenience, and a run that cannot say what produced it is not
worth keeping.

## What do you want to do?

This repository is the *how* behind the results published on the accompanying
website: the build, the calibration and the measurement methods. Pick a path:

- **Build your own stand** → follow [Getting started](#getting-started) in
  order. The six steps take you from a bare bench to your first CSV.
- **Understand how the published numbers were produced** → read
  [Why the measurements are shaped this way](#why-the-measurements-are-shaped-this-way)
  and [Output format](#output-format). The short version: every value ships with
  the raw sensor reading behind it and the conditions it was taken under, so a
  published result can be checked rather than trusted.
- **Just want the results** → they live on the website; this repo is the stand
  that produced them. You do not need any of it to read the numbers.

Everything past the build steps is depth, not a prerequisite. Skip what you do
not need and come back when a question points you here.

---

## What it measures

| mode | what it gives you |
|---|---|
| `SWEEP` | 1% throttle steps up **and back down**, 100 ms settle + 100 ms logged per step |
| `RESPONSE` | repeated steps between two non-zero throttles → rise time and τ |
| `COASTDOWN` | unpowered deceleration → spin-down time constant |
| `KV` | supply-voltage sweep at full throttle → Kv and effective resistance (pair with a prop-off run; see below) |
| `TRANSIENT` | step from standstill (characterises the ESC's start-up, not the motor) |
| `MASSCHECK` | load cell calibration and verification against known weights |
| `LINKTEST` | telemetry link quality against throttle (diagnostics, not data) |
| `FINEWALK` | walks raw DShot setpoints at finer than 1% spacing, visits randomised → where the reported speed steps |
| `STATIC` | legacy host-timed 10% steps, kept for comparison with older runs |

### Addressing throttle by raw DShot value

`measure.py` takes throttle in whole percent, and `map(pct, 1, 100, 1, 2000)`
reaches only 100 of the 2000 values DShot can carry. Near full throttle one
percent is several hundred RPM, which is coarser than the telemetry itself, so a
percent-stepped sweep cannot resolve where a step in the reported speed begins.

Three firmware commands address the setpoint directly. Send them over the serial
link, or let `FINEWALK` drive them for you:

| command | what it does |
|---|---|
| `HOLD,<pct>,<log_ms>,<settle_ms>` | settle at one throttle percent, log a window, stop |
| `DHOLD,<dshot>,<log_ms>,<settle_ms>` | the same, addressed by raw DShot value |
| `DSHOT,<value>` | set a raw setpoint and leave it there |

`log_ms` is clamped to 100–10000. `HOLD` and `DHOLD` run the same code, so their
settling and window behaviour cannot drift apart. With `DHOLD` the CSV's
`throttle_pct` is a rounded label and `dshot` is the real setpoint.

`HOLD` is deliberately generic, and the Kv, check-mass and link-test protocols
are all built on it host-side rather than each having its own firmware mode.

---

## Getting started

### 1. Build the hardware

See **[docs/HARDWARE.md](docs/HARDWARE.md)** for the bill of materials, wiring,
and the several connections that fail in ways that look like a dead component.

### 2. Install the toolchain

The firmware uses the [arduino-pico](https://github.com/earlephilhower/arduino-pico)
core. With `arduino-cli`:

```bash
arduino-cli core install rp2040:rp2040
arduino-cli lib install "HX711 Arduino Library" "INA2xx" "Adafruit BMP280 Library"
# ...or, if you fitted a NAU7802 instead of an HX711:
arduino-cli lib install "Adafruit NAU7802 Library"
```

That pulls in `Adafruit BusIO` and `Adafruit Unified Sensor` as dependencies.
`Pico_Bidir_DShot` is installed from source:

```bash
git clone https://github.com/bastian2001/pico-bidir-dshot \
    ~/Arduino/libraries/Pico_Bidir_DShot
```

| library | source |
|---|---|
| HX711 Arduino Library | <https://github.com/bogde/HX711> |
| Adafruit NAU7802 Library *(alternative to the HX711)* | <https://github.com/adafruit/Adafruit_NAU7802> |
| INA2xx | <https://github.com/Zanduino/INA> |
| Adafruit BMP280 Library | <https://github.com/adafruit/Adafruit_BMP280_Library> |
| Pico_Bidir_DShot | <https://github.com/bastian2001/pico-bidir-dshot> |

Host side:

```bash
pip install -r requirements.txt
```

### 3. Configure for your build

Settings are split by how often they change, so that the ones that change often
never require a reflash:

| file | holds | changes when |
|---|---|---|
| **`firmware/config.h`** | pin assignments, DShot rate, which load cell ADC is fitted and its sample rate, which INA and shunt are fitted | you pick up a soldering iron |
| **`stand.json`** (host) | load cell factor, motor pole count, declared setup (ESC firmware, PWM frequency, supply) | you recalibrate, swap the motor, reflash the ESC or change the supply |

Edit `config.h` once when you assemble the stand. The value most worth
double-checking is `INA_SHUNT_UOHM`. It must match the shunt actually fitted,
and a wrong one scales every current and efficiency figure without ever looking
wrong.

The load cell factor in `stand.json` is written for you by `MASSCHECK`. Five
fields in it are declarations you have to keep honest yourself:

```json
{
  "esc_firmware": "AM32 2.21",
  "esc_pwm_khz": 48,
  "hx711_scale": -17669.5711,
  "motor_poles": 12,
  "psu": "Kiprim DC310S",
  "psu_voltage_v": 4.0,
  "esc_dir_forward": "normal"
}
```

They go into every CSV as a `# setup,` header line, and `--esc-firmware`,
`--esc-pwm-khz`, `--psu`, `--psu-voltage` and `--prop-hand` override them for
one run. None of them can be read back over DShot, which carries no firmware
version at all, and **a patched Bluejay reports the same version string as a
stock one**. That is why `measure.py` echoes all five before the motor spins,
and warns about any that are unset. A blank field is recoverable from your
notes; a confidently wrong one is not.

On an ARM ESC (AM32, BLHeli_32) you can do better than declaring them.
`tools/am32.py` reads the firmware version and the whole settings page back off
the ESC over 4-Way, so the declaration becomes checkable rather than trusted.
See [docs/HARDWARE.md](docs/HARDWARE.md).

`esc_dir_forward` is a bench-level fact, not a per-run one. It names which ESC
spin direction is the *whole current prop set's* own design-forward direction:
one value for every prop in service, set once when new props go in.
`--reverse` is the per-run flag (see below); the two combine to work out
which way the ESC actually needs to spin.

Unlike the other four, leaving it unset does not just warn: **`measure.py`
refuses to start any mode except `--check` until it is set.** A wrong value
here mis-declares an entire session rather than one run, and nothing catches
it until the runs already exist. Getting it right once beats defaulting
quietly.

A sixth field, `--specimen`, has no `stand.json` counterpart on purpose. It
answers a different question: not which ESC direction is forward, but which
*physical prop* was mounted (e.g. `1219s-a`, matching whatever is written on
the bag). A model name is not a specimen id, and this is the only field that
tells two samples of the same model apart.

It changes at every prop swap. Unlike the other five, a persisted default
would go stale after the first swap and silently mislabel every run after it.
That is worse than leaving it blank, which can still be filled in later. It is
always optional, in every mode: bench first, work out which sample it was
afterwards.

### 4. Flash and verify

```bash
arduino-cli compile --fqbn rp2040:rp2040:rpipico firmware
arduino-cli upload  --fqbn rp2040:rp2040:rpipico -p /dev/ttyACM0 firmware

python measure.py --check                      # does not spin the motor
```

`--check` prints the boot banner, scans the I²C bus, shows live samples and
reports the ambient reading. Everything should answer before you go further.

**`-m` has no default, so a command without one never spins the motor.**
`--check`, `--tare` and `--watch` work on their own; anything that drives the
motor has to name its mode.

```bash
python measure.py --check --tare --watch 180   # zero drift + link quality
```

On Windows the port looks like `COM3`, on macOS `/dev/cu.usbmodem*`; pass it
with `-p`.

### 5. Calibrate the load cell

Nothing is configured yet, and nothing needs to be. The factor is fitted from
raw counts against known masses:

```bash
python measure.py -m MASSCHECK --masses 10,20,30,50 -o data/cal.csv
```

It writes the result to `stand.json` and you re-run to verify; the second pass
should report a scale error near 0.00%. No reflash at any point. Full procedure,
including how to declare your stand's geometry, in
**[docs/CALIBRATION.md](docs/CALIBRATION.md)**.

Re-derive it at the start of each session rather than trusting a stored value.
The factor is a property of the mounting and can move.

### 6. Measure

```bash
python measure.py -m SWEEP -o data/run1.csv \
    --prop-diameter-mm 31 --notes "0702 26kKV, 31mm tri-blade, 1S 450mAh"

python plot_comparison.py data/run1.csv
python plot_comparison.py data/run1.csv data/run2.csv --save compare.png
```

`--save` writes the figure instead of opening a window, and forces a headless
backend, so it works over ssh. The file extension picks the format.

Pass `--prop-diameter-mm` whenever you know it: it enables a physics-based QC
gate that catches wrong constants without your having to anticipate the failure.
Use `--poles N` if your motor is not the 12-pole default.

**A run that fails still writes its data.** If the board drops off the bus, the
firmware aborts, or you hit Ctrl-C, `measure.py` saves the rows it had, marks
the file `# result,status=aborted,reason=...` and exits non-zero (130 for
Ctrl-C). Beside the CSV it leaves a raw log of every line the board sent, named
in `# raw_capture`. Nothing is fitted or reported from a partial run, and an
interrupted `MASSCHECK` never writes a calibration factor. Tools that read one
say so: `plot_comparison.py` labels it "(partial)". See
[docs/COLUMNS.md](docs/COLUMNS.md#partial-runs).

To look at a narrow band in more detail than whole percent allows:

```bash
python measure.py -m FINEWALK --dshot-range 1900,1990,10 --walk-reps 3 \
    -o data/walk.csv
```

Visits are randomised across all repetitions, so drift with running time cannot
masquerade as a response to throttle.

**Measure motor Kv.** Two runs and a pair solve, because a voltage sweep alone
cannot separate Kv from resistance:

```bash
python measure.py -m KV --voltages 3.0,3.4,3.8,4.2 --kv-dshots 2000 \
    -o data/kv_loaded.csv --prop-diameter-mm 31 --notes "26kKV, 31mm tri-blade, prop ON"
# remove the propeller, keep everything else identical:
python measure.py -m KV --voltages 3.0,3.4,3.8,4.2 --kv-dshots 2000 \
    -o data/kv_noload.csv --notes "26kKV, prop OFF"
python measure.py --kv-pair data/kv_loaded.csv data/kv_noload.csv
```

The loaded run holds full throttle at each supply voltage with the prop on.
Full throttle means duty ≈ 1, and the prop keeps an unloaded whoop motor from
chasing ~100 kRPM. The no-load run repeats the same voltages with the prop off.
`--kv-pair` matches each no-load point to the nearest loaded one by bus voltage:
same V, very different I, which is the axis that identifies R.

Use the **same `--voltages` for both runs**, and keep `--kv-dshots` at one level
(`2000`). Pairing assumes duty 1 and ignores everything below the top DShot
level. Don't take no-load points below the loaded run's lowest bus voltage:
they have nothing to pair with. With a bench supply, add `--psu-port` and the
sweep sets and verifies each voltage itself. A single `-m KV` sweep alone fits
an ill-conditioned model whose coefficients look normal while the answer slides
along a ridge. See [Why the measurements are shaped this way](#why-the-measurements-are-shaped-this-way).

---

## Testing without hardware

### An RPM reference the ESC has no part in

Every speed figure the stand produces comes from the ESC's own telemetry, so it
cannot arbitrate a question about that telemetry. A prop turning at n rev/s with
B blades radiates a tone at n*B, which is a property of the air:

```bash
ffmpeg -i clip.m4a -ac 1 -ar 48000 clip.wav      # phones record m4a
python tools/blade_tone.py clip.wav --expect-rpm 47000
python tools/blade_tone.py clip.wav --compare data/hold.csv --plot out.png
```

It reports rotor RPM against time and, given a run CSV, puts its drift slope next
to the telemetry's. Comparing slopes needs no clock alignment between recording
and CSV, only that both cover the same hold. Acoustic rather than an optical
tachometer because tape on a 31 mm prop is a real fraction of its mass on one
blade, which unbalances what you are measuring.

Trim the recording to the hold before analysing it. Then narrow `--tolerance`
until the implied blade count it prints lands on the real one. The blade tone
is not necessarily the loudest thing in its neighbourhood. A wide band can rail
on something else and report a confident slope that is not the rotor. The
tool's docstring has the details.

`tools/tone_hold.py` produces the matching run: one uninterrupted hold logged per
sample, so the recording and the telemetry cover exactly the same event.

`tools/fake_stand.py` emulates the firmware over a pty and runs the real
`measure.py` against it, so host changes can be developed and validated with
nothing connected. It carries a ground-truth motor model, so the Kv fit is
checkable against known values, and a load cell model, so `MASSCHECK` is too.

```bash
python tools/fake_stand.py -m SWEEP -o /tmp/out.csv
python tools/fake_stand.py -m SWEEP --shunt-uohm 100000   # simulate a railing sensor
python tools/fake_stand.py -m MASSCHECK --mass-slope 1.03 # cell 3% off; host should say so
python tools/fake_stand.py --no-edt -- --check --watch 8 --edt  # dead ESC telemetry link
python tools/fake_stand.py --serve                        # expose a pty, print its path
```

Arguments after `--` are forwarded to `measure.py`.

`tests/` holds host tests that need no pty at all: they drive `measure.py`
through an in-memory serial peer to check what happens when a run fails. Run
them with `python -m unittest discover -s tests`. They cover host behaviour
only. Firmware safety is verified on hardware, and the emulator remains the
check for a full protocol end to end.

---

## Why the measurements are shaped this way

> Skip this on first read if you are building a stand. It is for interpreting
> results. Each point explains a measurement choice that otherwise shows up as a
> quirk in the data.

- **Sweeps run both directions.** A one-directional sweep carries a settling-lag
  bias in the direction it travels. Averaging the legs cancels it, and the
  up/down gap is a free check on whether 100 ms of settling is enough. The down
  leg does run hotter, though, so part of any gap is thermal drift rather than
  lag.
- **Responsiveness is measured on RPM, never thrust.** The HX711 contributes
  ~30 ms of filter group delay and resolves only ~8 samples across a typical
  90 ms rise, while DShot telemetry updates at kHz. `RESPONSE` also steps
  between *non-zero* throttles, because a step from standstill spends ~200 ms in
  the ESC's arm-and-start sequence, which never happens in flight.
- **Kv needs the load changed, not just the voltage.** `KV` sweeps supply
  voltage at full throttle, prop fitted, and fits `RPM = Kv·(V − I·R)`. Full
  throttle means duty ≈ 1, which removes the `I_motor = I_bat / duty` model.
  Keeping the prop on holds RPM in its normal band, rather than letting an
  unloaded whoop motor chase ~100 kRPM.

  **That sweep alone cannot separate Kv from R, and the failure is silent.** At
  a fixed prop, load torque goes as n² and n goes as V, so current is a
  *function* of voltage. The two regressors are collinear to r > 0.99 and the
  fit slides along a ridge. Sub-percent changes in RPM then swing Kv by several
  percent while the residuals barely move.

  Repeating one sweep four times here gave a 14.6% spread in Kv from data whose
  RPM repeated to 1.3%. Extra throttle setpoints (`--kv-dshots`) do *not* rescue
  it. Below duty 1 the motor simply sees a lower voltage, so those points land
  on the same one-dimensional operating curve.

  What works is a second run with the **prop off**, paired against a loaded
  point at the same bus voltage: same V, very different I, which is what
  identifies R. `measure.py --kv-pair loaded.csv noload.csv` does that
  arithmetic. The prop swap cannot be automated, but nothing else about it
  needs doing by hand. It pairs on bus voltage and uses only the top DShot
  level, because pairing assumes duty 1. It also rejects failed starts, and
  points where speed stopped rising with voltage, an ESC running out of
  commutation headroom.

  **Record the no-load run at the same `--voltages` as the loaded one.** The
  solve assumes both points share one Kv and one R. R varies with operating
  point, so a voltage mismatch between the pair carries that variation straight
  into the answer. Matching them took the spread here from 4.2% to 0.8% without
  discarding any data. Don't take no-load points below the loaded run's lowest
  bus voltage either. They have nothing to pair with.

  `report_kv` prints `corr(V_motor, I_motor)` on every fit and warns above 0.99,
  because an ill-conditioned fit looks entirely normal from its coefficients.
  Quote a loaded-sweep Kv with the spread across repeats, or quote RPM/V at full
  throttle with the prop named. The latter needs no fit and repeats far better.
- **Ambient conditions are not optional.** Thrust is directly proportional to air
  density. Humidity is not measured yet: across any realistic indoor range it
  moves density only ~0.35%, against ~2% for a few degrees of temperature and
  ~3% for ordinary weather. A BME280 swap is a possible but minor upgrade; see
  [docs/HARDWARE.md](docs/HARDWARE.md#possible-future-upgrades). Thrust is
  normalised to ISA density at matched RPM; electrical power is left
  uncorrected, because aerodynamic power scales with density but motor I²R
  losses do not.
- **Throttle % is not a comparable axis** across packs, ESC firmware or
  sessions. Steady-state plots default to RPM, and results are best compared at
  matched *thrust*. The question a pilot actually has is "at my hover thrust,
  which combination draws least power and responds fastest".

### Extended DShot Telemetry

> Optional and advanced. EDT is a low-rate ESC status channel; you do not need
> it to take a sweep.

**Whether the stand asks for EDT is a setting, not a fixed answer**, because
whether an ESC replies is a property of its firmware. `EDT_REQUEST_DEFAULT` in
`firmware/config.h` sets what a build does, and `measure.py --edt` and
`--no-edt` override it for one run without a reflash. Asked, the firmware sends
DShot command 13 at boot and at every sequence start, and the banner records
`edt=1` so a run says whether it asked.

**Check your own ESC before trusting the default.** The reference build ships
with it on, because the fitted AM32 ESC answers readily. An earlier Bluejay ESC
on this same stand stopped sending EDT the moment it armed and never resumed
without a power cycle, and on an ESC like that you want `--no-edt` or a build
default of 0.

The reason it is a choice at all is that support is an ESC-firmware property
that nothing can read back. A channel that never answers is worse than no channel: `edt_stale` lands
on every row of every run. A flag that is always set is a flag nobody reads.
Empty `esc_*` columns then mean two opposite things: unasked says nothing about
your ESC, asked-and-empty is a finding about it. That is exactly what the banner
field separates.

Frames are always *decoded* if they arrive, whether or not they were asked for.
They are valid telemetry, and counting them as anything else charges every
temperature and stress frame to the link error rate.

**Treat whatever arrives as opportunistic.** Support varies widely between ESC
firmwares: fields may never be sent, may report implausible values, and the
whole channel may stop once the motor arms. All EDT types also share one budget
of a few frames per second against 250 printed rows/s, which is far too sparse
to attribute anything to a particular throttle step.

Everything is plumbed so those cases are *visible* rather than silently frozen.
The `esc_*` columns carry `n_edt`/`edt_age_us` and are deduplicated like every
other channel. A step reports them only if a frame actually landed in it, and a
dead link raises `edt_stale` with `edt_frames=0`. The columns stay in every CSV
either way, present and empty, so runs taken with and without remain
column-compatible.

But do not build a measurement on EDT. For catching a bad run, rely on the
actuator-disk QC gate, RPM/throttle monotonicity and thrust/RPM² consistency,
which come from channels sampling at 80–250 Hz.

---

## Design principles

Most of what follows exists to make one class of failure impossible to miss.
The failure is not a crash, but a channel that quietly saturates or a constant
that is quietly wrong, producing smooth, plausible, useless data.

1. **Every scaled value ships beside its raw sensor value**: `erpm_raw` beside
   `erpm` beside `rpm`, `thrust_raw_mean` beside `thrust_g_mean`, `shunt_uv`
   beside `bus_ua`. A wrong calibration constant is then genuinely recoverable
   rather than theoretically so: `--rescale` re-derives any past run from the
   counts the sensor actually produced. Load cell scaling happens on the host for
   the same reason, so the factor is never baked into a binary.

   RPM is the case where this earns its keep twice over. Bidirectional DShot
   does not send a speed. It sends the period of one electrical revolution,
   packed as a 9-bit mantissa and a 3-bit exponent, and every RPM figure here is
   a division of that. So resolution degrades as RPM², and near the top of a
   sweep a throttle step can be *smaller than one representable period*. That is
   a flat patch in an RPM curve that is arithmetic, not aerodynamics.

   Logging the transmitted word (`erpm_raw`, summarised per step as
   `erpm_raw_hist`) is what separates the three cases after the fact. Those
   cases are the telemetry too coarse to express the change, the ESC repeating
   one value, and the speed genuinely not moving. `measure.py --codes` prints it
   per step; `rpm_per_us` in every steady-state row is the finest change the
   telemetry could have expressed there. Read any plateau against that number
   before believing it.
2. **Per-channel sequence counters.** Rows print at 250 Hz while the HX711
   converts at 80 Hz and the INA at 100 Hz, so the host deduplicates on these
   counters. Averaging printed rows would count each physical reading 2–3× and
   deflate the standard deviation by roughly √3.
3. **Saturation and staleness are flagged in firmware**, per sample, rather than
   left for a human to notice later. A flag marks the sample it belongs to and
   nothing else. `esc_alert` is cleared as soon as it is printed: latching it
   until the next ESC status frame can mark a whole run as suspect from a single
   event. Plots drop a row only for flags that invalidate what is being
   plotted; ESC-link flags are reported and kept.
4. **Every CSV records what produced it**: chip identity, shunt value, scale
   factor, pole count, firmware version, telemetry link health and the ambient
   conditions. A run without provenance cannot be merged with anyone else's.
5. **A physics-based QC gate.** `--prop-diameter-mm` enables an actuator-disk
   check that rejects any row claiming better efficiency than an ideal disk of
   that diameter could achieve. It catches wrong shunt values, wrong scale
   factors and clipped current *without* having to anticipate the failure.
   Typical whoop figure of merit is 0.2–0.3; above 1.0 is impossible.
6. **A loss percentage means nothing on its own.** `LINKTEST` splits telemetry
   failures into `corrupt` (the ESC answered and the answer arrived damaged:
   electrical, worth chasing) and `silent` (it did not answer: the ESC's own
   scheduling). It then compares what survives against the print rate. Frames
   are polled at a few kHz while rows print at 250 Hz, so even a large loss
   percentage often costs no samples at all.

---

## Output format

CSVs are schema 1: a `#`-comment metadata header carrying the firmware banner
and run conditions, then named columns. Steady-state, transient, Kv, mass-check,
link-test and fine-walk runs have different column sets. `plot_comparison.py` reads by
column name and refuses anything older. [docs/COLUMNS.md](docs/COLUMNS.md) is the
reference for every column in every set.

Runs are not tracked in this repository. `.gitignore` excludes `data/` and
`*.csv`, because CSVs diff badly and a dataset outgrows git as motors and props
are added.

Each CSV is self-describing. The header carries firmware version, calibration
factor, shunt value, pole count and the ambient conditions at the time. A
`# setup,` line adds the ESC firmware, PWM frequency, supply and pole hand
declared in `stand.json`, plus the rotation for the run (`--reverse`). When
given, the specimen (`--specimen`) is recorded there too.

`RESPONSE` runs additionally carry `# response,base=40,high=70,reps=5`, the
commanded step levels, so a reader can pick step edges exactly rather than
inferring them from sample counts. A file can then be interpreted without any
accompanying document.

Per-sample files also carry a `phase` column: which part of the sequence each
row belongs to, as a code the `# phase,` header names. On a `RESPONSE` run it
separates the steps from the staircase down to zero that follows them, which is
the tail that used to leak into fall times.

`# setup,specimen=<id>` is the only place that says which physical prop was
mounted, and it is optional in every mode. A run without it is normal, not
an error. Assign it after the fact by adding `specimen:` to the run's own
record instead (`data/testruns/<id>.yaml` on the consuming site). A file
should carry a specimen in exactly one of those two places, never both.

`# setup,rotation=normal|reversed` is on **every** run, not only reversed ones.
A run with no rotation recorded at all is an undocumented controlled dimension,
and a strict comparison has to reject it. The field means the prop turned the
way *it* is built to turn, not which way the ESC was told to spin it. It is the
only field a consumer should read for rotation.

`# esc,dir=normal|reversed` is a separate fact, always present too: which way
the ESC was actually *commanded* to spin this run (not confirmed; Bluejay
never acknowledges it). For an odd-handed prop (`--prop-hand reversed`, or
`esc_dir_forward` in `stand.json`) measured in its own normal direction, the
two legitimately disagree: `rotation=normal` with `dir=reversed`. The ESC has
to be told to spin backward for *that* prop to turn forward.

Some things the file cannot know, and they are worth stating alongside any
published set:

- the motor: make, size, Kv, **pole count**
- the propeller
- whether the supply voltage was held constant
- when the load cell was last calibrated and verified
- the measured error budget for your stand

---

## Layout

```
firmware/firmware.ino       the firmware
firmware/config.h           per-build settings -- edit this one
stand.json                  per-stand calibration, written by MASSCHECK (not tracked)
measure.py                  host: commands sequences, records CSV
plot_comparison.py          host: plots one or more runs
tools/README.md             what each tool in tools/ does, and how to drive it
tools/fake_stand.py         firmware emulator over a pty, for testing without the bench
tools/blade_tone.py         host: rotor RPM from the blade-pass tone, no ESC involved
tools/tone_hold.py          host: one uninterrupted hold, to record against that tone
tools/psu.py                host: SCPI control of a programmable bench supply
tools/scope.py              host: waveform capture from an OWON HDS handheld scope
tools/soak_test.py          host: repeated holds with a re-tare, for the RPM drift
tools/approach_test.py      host: does approach direction change the reported period
docs/HARDWARE.md            bill of materials, wiring, gotchas
docs/CALIBRATION.md         load cell calibration procedure
docs/COLUMNS.md             what every CSV column means
tests/                      host tests for the failure paths, no hardware needed
tools/am32.py               host: read, change and flash AM32 ESC settings
tools/flash_board.py        host: swap the board between stand firmware and passthrough
tools/block_compare.py      host: one block of repeated sweeps against another
vendor/BlHeli-Passthrough/  4-way passthrough sketch, vendored, GPL-3.0
LICENSE                     GPL-3.0
data/                       measurement output (not tracked)
```

## Safety

`PING` arms a firmware watchdog; once armed, the motor stops if the host goes
silent for 1 second, so a crashed script cannot leave a propeller spinning. It
deliberately does not arm until the first `PING`, so an interactive serial
monitor is not fought. The host also zeroes throttle in a `finally`. **Keep both
guarantees in any new host code.**

## Licence

Code and documentation are **GPL-3.0-or-later**, see [LICENSE](LICENSE). You may
use, study, modify and redistribute it; if you distribute a modified version,
that version must be released under the same terms with its source available.

**Measurements you take with it are yours.** The licence covers this software,
not its output. The numbers your stand produces are your data, to publish or
license however you see fit, or not at all. If you do publish them, a CC licence
is the usual fit for data; the GPL is not designed for it. But that is your
call to make, not this project's.

### Dependencies

| library | licence |
|---|---|
| [Pico_Bidir_DShot](https://github.com/bastian2001/pico-bidir-dshot) | **GPL-3.0** |
| [INA2xx](https://github.com/Zanduino/INA) | **GPL-3.0** |
| [HX711 Arduino Library](https://github.com/bogde/HX711) | MIT |
| [Adafruit NAU7802](https://github.com/adafruit/Adafruit_NAU7802) | BSD-3-Clause |
| [Adafruit BMP280](https://github.com/adafruit/Adafruit_BMP280_Library), [BusIO](https://github.com/adafruit/Adafruit_BusIO) | MIT |
| [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) | Apache-2.0 |
| pyserial | BSD-3-Clause |
| matplotlib | PSF-based (BSD-compatible) |

The firmware links two GPL-3.0 libraries, so **any compiled binary is a combined
work under GPL-3.0**. This repository distributes source only. If you publish a
built `.uf2`, you are conveying that combined work and must do so under GPL-3.0,
including offering corresponding source for the libraries as well.

## Contributing

The serial protocol is the real interface between the three pieces, and
`tools/fake_stand.py` is a third implementation of it, so a protocol change
means updating firmware, host and emulator together. Run the emulator across every
mode before sending a change; it is the only end-to-end test there is.
