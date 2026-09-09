# CSV columns

Every CSV is schema 1: a `#`-comment metadata header, then named columns. Which
columns appear depends on the run type. A sweep, a transient, a Kv run, a mass
check, a link test and a fine walk each carry the columns they need and no
others. This file is the reference for both.

The header carries everything needed to interpret the file on its own:

- the firmware banner: `#FW`, `#BOARD`, `#ESC`, `#SCALE`, `#INA`, `#FLAGS`,
  `#PHASE`, `#COLS`
- the ambient conditions: `#AMBIENT`
- the calibration factor: `# calibration`
- the declared setup, run rotation and (when given) the specimen: `# setup`
- the telemetry link health for the run: `#STATS`
- whether the run finished, and where its raw wire log went: `# result`,
  `# raw_capture`

`RESPONSE` runs additionally carry the commanded step levels as
`# response,base=..,high=..,reps=..`. `plot_comparison.py` reads columns by
name and refuses any file with a schema other than 1.

**Rotation and spin direction are two different fields; a consumer should not
read `dir=` for rotation.** `# setup,rotation=normal|reversed` is the only
field that says whether the prop turned in its own design direction. It is on
every run (`measure.py --reverse`, defaulting to `normal`), never
"not recorded" the way the other `# setup,` fields can be. So treat a file with
no `rotation=` at all as pre-dating this field, rather than as an undeclared
"normal."

`# esc,dir=normal|reversed` is a separate, also-always-present fact: which way
the ESC was actually *commanded* to spin this run. On a Bluejay ESC that is
real, via DShot's temporary spin-direction command: commanded, not confirmed,
since Bluejay never acknowledges it. These are not a priority pair; they answer
different questions and can legitimately disagree. A run on an odd-handed prop
(`--prop-hand reversed`, or `esc_dir_forward` in `stand.json`) measured in its
own normal direction carries `rotation=normal` with `dir=reversed`. That prop's
own forward direction needs the ESC spun backward.

The consuming site reads only `rotation=`, and uses it two ways. It decides
whether to negate the thrust column for display. A reversed run's raw thrust is
signed by the load cell's own convention, not by "how much thrust did this prop
make", so negating it makes cross-comparisons sane. It also fails its build if a
run's declared rotation disagrees with the sign the load cell actually
recorded, say `rotation=normal` on a run whose thrust reads negative.

That build failure is the safety net for a wrong `--reverse` or `--prop-hand` at
the bench. `measure.py` prints the same check itself at the end of a run, so the
mistake shows up immediately rather than a day later on someone else's build.

**`# setup,specimen=<id>` names the physical prop, not the model, and is
optional in every mode.** A model (e.g. HQProp Ultralight 1.2x1x3) can have
several physical samples in service at once: the bench's golden reference and a
working sample are the same model. `specimen` is the only field that tells them
apart, matching whatever id is written on the sample itself (`1219s-a` and so
on).

It has no `stand.json` fallback. Unlike the other `# setup,` fields it changes
at every prop swap, so a persisted value would silently mislabel every run after
the first swap. That is worse than a missing one, which can still be filled in.
A run without `specimen=` is not an error. Bench first, work out which sample
it was later.

The consuming site reads `# setup,specimen=` first, and falls back to a
`specimen:` field in the run's own record only when the header omits it. A run
record that restates a specimen the file already declares fails the site build.
`specimen` is a controlled dimension only among runs naming the same prop model,
since it varies by definition across different models.

## Partial runs

`# result` says whether the run completed its protocol: `status=complete`, or
`status=aborted,reason=...` naming what stopped it. A run that fails, loses the
board or is stopped with Ctrl-C still writes the rows it collected. An aborted
file is a real measurement of a shorter run, not a corrupt one.

Read those rows, but not as a protocol. A sweep can be missing its entire down
leg, a mass check half its masses, a Kv run every voltage above the first. So
anything that assumes a complete set has to check the marker first.
`measure.py --rescale`, `--refit` and `--kv-pair` warn on one, and
`plot_comparison.py` appends "(partial)" to the run's label, because a figure
gets shared and a console warning does not travel with it.

An aborted file still carries the calibration factor, the declared setup and
rotation, the ambient conditions and the density-normalised thrust column.
Density cannot be applied after the fact, so it is applied whatever the
outcome. It also carries the firmware's own account of the ending: the
`# warn,sequence_aborted,<reason>` line and the `#STATS` telemetry health for
the shortened run.

What it does not carry is any conclusion. No fit, no report, no updated
`stand.json`: an interrupted MASSCHECK never writes a calibration factor.

`# raw_capture,file=<name>` names the wire journal written beside the CSV,
holding every line the board sent during the run. Successful runs get one too.
It is a debugging record rather than an input to any tool, it is gitignored,
and it can be deleted once the run is understood.

## Firmware columns vs CSV columns

The firmware prints rows named by its `#COLS` banner:

```
t_us, phase, throttle_pct, dshot, rpm, erpm, erpm_raw, n_rpm,
thrust_raw, n_thrust, thrust_age_us, bus_mv, bus_ua, shunt_uv,
n_ina, ina_age_us, esc_temp_c, esc_stress, esc_dbg1, esc_dbg2,
n_edt, edt_age_us, flags
```

The host renames some of these (`bus_mv` → `volts`, `bus_ua` → `amps`,
`t_us` → `time_s`) and derives the rest from them. The raw values stay in the
CSV so a run can be re-derived later: `--rescale` with a corrected load-cell
factor, `--poles` with a corrected pole count. `erpm_raw` is the 12-bit
telemetry word, a *period* rather than a speed. `erpm` and `rpm` are decoded from it
in the same row, so `rpm == erpm / (poles/2)` holds in every row by
construction.

## Reading a summary row

Rows print faster than the sensors convert, 250 Hz against the HX711's 80 SPS
and the INA's 100 SPS, so **summary columns are computed over distinct physical
readings, not printed rows**. The host deduplicates on the sequence counters
(`n_rpm`, `n_thrust`, `n_ina`, `n_edt`); means and standard deviations are over
real samples, and the `n_*` counts show how many stand behind each.
`rows_printed` is the printed row count and is *not* the sample support.

Two counts describe what went wrong at the edges of that scheme, and they mean
opposite things:

- **`n_*_stale`**: a printed row repeated a sample already counted. Harmless,
  because the dedup removed it from the means; counted rather than flagged so a
  long window is not condemned for one gap.
- **`n_*_missed`**: a conversion happened and *no* row ever carried it. This
  can only occur when the ADC outruns the row rate, which is why `PRINT_FAST_US`
  moves with the fitted part. Those samples are not recoverable from the CSV,
  and the loss is systematic rather than random, so it can alias periodic
  content, prop-order vibration above all, into means that still look well
  behaved. Anything other than zero deserves investigation before the numbers
  are used.

## Column sets by run type

| run | column set |
|---|---|
| `SWEEP`, bare-number static | steady-state |
| `TRANSIENT`, `RESPONSE`, `COASTDOWN` | transient |
| `LINKTEST` | steady-state + link |
| `FINEWALK` | walk |
| `KV` | Kv |
| `MASSCHECK` | mass |

## Steady-state set

`SWEEP` and static runs. One row per step (a sweep step is ~25 printed rows
collapsed into one summary).

| column | meaning |
|---|---|
| `mode` | run type: `sweep`, `static`, `kv`, `finewalk`, `linktest`, ... |
| `direction` | `up` or `down` sweep leg; blank for static |
| `throttle_pct` | commanded throttle in percent. A rounded label for DShot-addressed runs; `dshot` is the real setpoint |
| `dshot` | the raw DShot setpoint the firmware echoed back |
| `rpm_mean` | mean mechanical RPM over the step's distinct readings |
| `rpm_std` | standard deviation of those readings |
| `n_rpm` | number of distinct RPM readings |
| `erpm_period_us` | the most common raw telemetry period, decoded to microseconds: what the ESC actually sent |
| `rpm_per_us` | RPM per microsecond of period at this step (`rpm_mean / erpm_period_us`). The finest change the telemetry can express here; grows as RPM², so it must be read alongside any step near the top of a sweep |
| `n_codes` | number of distinct raw period codes seen this step |
| `erpm_raw_hist` | histogram of raw period codes as `code:count`, `|`-separated |
| `thrust_g_mean` | mean thrust in grams |
| `thrust_g_std` | standard deviation of thrust |
| `n_thrust` | distinct thrust readings |
| `thrust_raw_mean` | mean raw ADC counts, what the grams were derived from, so a run stays re-derivable under a corrected factor. The `# scale` header line names the part that produced them |
| `volts_mean`, `volts_std` | bus voltage in volts from the INA226 |
| `amps_mean`, `amps_std` | bus current in amps from the INA226 |
| `n_ina` | distinct INA readings |
| `watts_mean` | `volts_mean × amps_mean` (ratio of means, not mean of ratios) |
| `eff_g_per_w` | `thrust_g_mean / watts_mean` |
| `thrust_g_iso` | thrust normalised to ISA air density, so measurements taken on different days are comparable at matched RPM |
| `fom` | actuator-disk figure of merit: actual efficiency over the ideal disk's. Present only when `--prop-diameter-mm` was passed; a value above 1.0 is physically impossible |
| `esc_temp_max_c` | maximum EDT temperature reading this step, only if a fresh frame landed; blank otherwise |
| `esc_stress_max` | maximum EDT stress reading, same freshness rule |
| `n_edt` | number of EDT frames received this step |
| `rows_printed` | printed rows accumulated (before dedup) |
| `n_rpm_stale` | printed rows whose RPM repeated an already-counted sample |
| `n_thrust_stale` | printed rows whose thrust repeated an already-counted sample |
| `n_thrust_missed` | thrust conversions no row ever carried; see above. Zero on a healthy run |
| `n_thrust_mangled` | thrust samples rejected as corrupt I²C transfers before any mean was formed. The third of a set: stale was printed twice, missed was never printed, mangled was printed and was not a reading. Non-zero means the bus is faulty, not that the file is bad |
| `n_ina_mangled` | INA readings rejected as corrupt I²C transfers. The quieter half of the same fault: a corrupt bus voltage lands a volt or two out on a 4 V rail and reads as a plausible number, so it moves watts and `eff_g_per_w` without looking wrong |
| `n_ina_missed` | INA conversions no row ever carried. Zero on a healthy run |
| `flags` | bitmask of sensor flags, see below |
| `flag_names` | the same flags as names |

## Transient set

`TRANSIENT`, `RESPONSE` and `COASTDOWN`. One row per printed sample. The
`new_*` columns are the dedup markers: rows print at 250 Hz while the sensors
convert slower, so consecutive rows repeat the same reading. A `new_` of 0 means
"this row repeats the previous value".

A `RESPONSE` run's header additionally carries `# response,base=B,high=H,reps=N`,
with the throttles and repeat count actually commanded (`--base`/`--high`/
`--reps`). Step edges can then be read off the header exactly, rather than
inferred from sample counts, which is how the firmware's ~400 ms spindown tail
leaked into fall-time figures.

Every row also carries `phase`, the firmware's own label for the part of the
sequence the row belongs to, decoded by the `# phase,` line in the header. On a
`RESPONSE` file that separates the steps (`5=response`) from the staircase down
to zero that follows them (`8=spindown`), so a consumer selects the rows it
wants instead of inferring them. Files written before the column existed do not
carry it, and the header pair above stays the fallback for those.

| column | meaning |
|---|---|
| `mode` | `transient`, `response` or `coastdown` |
| `time_s` | seconds since the first logged row |
| `phase` | which part of the sequence this row belongs to, as a code the `# phase,` header names |
| `throttle_pct` | commanded throttle in percent |
| `dshot` | raw DShot setpoint |
| `rpm` | mechanical RPM this row |
| `erpm` | electrical RPM = `rpm × poles/2` |
| `erpm_raw` | the 12-bit telemetry word (a period) |
| `new_rpm` | 1 if this row carries a fresh RPM reading |
| `thrust_g` | thrust in grams this row |
| `thrust_raw` | raw ADC counts, from whichever part the `# scale` header names |
| `new_thrust` | 1 if fresh |
| `thrust_age_us` | age of the thrust reading |
| `thrust_t_s` | when the load cell conversion actually happened (row time minus age) |
| `volts`, `amps` | bus voltage and current this row |
| `shunt_uv` | INA shunt voltage in microvolts |
| `new_ina` | 1 if fresh |
| `ina_age_us` | age of the INA reading |
| `esc_temp_c`, `esc_stress` | EDT temperature and stress, sample-and-hold |
| `esc_dbg1`, `esc_dbg2` | ESC debug bytes: constants on stock Bluejay, the ESC's internal `Comm_Period4x` low/high bytes on a build patched by `tools/patch_bluejay_debug.py` |
| `new_edt` | 1 if this row carries a fresh EDT frame |
| `edt_age_us` | age of the EDT reading |
| `flags`, `flag_names` | sensor flags |

## Link-test additions

`LINKTEST` carries the steady-state period and power columns plus the telemetry
link counts. `telem_corrupt` and `telem_silent` mean opposite things. Corrupt
means the ESC answered and the checksum failed: electrical, so wiring and
grounding. Silent means no answer at all: ESC scheduling, wrong pin, broken
wire. Read the split, never the sum.

| column | meaning |
|---|---|
| `telem_ok` | telemetry frames decoded |
| `telem_corrupt` | frames that failed the checksum |
| `telem_silent` | commands with no answer |
| `silent_pct`, `corrupt_pct` | the same as percentages of all frames |
| `edt_frames` | EDT frames received |

## Walk additions

`FINEWALK` holds raw DShot setpoints in randomised order, repeating each. The
columns record *when* each hold happened, which is what makes the running-time
drift separable from the throttle response afterwards.

| column | meaning |
|---|---|
| `dshot_set` | the commanded DShot value |
| `dshot` | what the firmware echoed back; should equal `dshot_set`, and a disagreement means the setpoint did not take |
| `visit` | visit number (order was randomised) |
| `rep` | repeat within a visit |

## Kv additions

`KV` holds one or more DShot setpoints at each supply voltage. `dshot` is the
second axis of the fit; `v_set` is the commanded supply voltage.

| column | meaning |
|---|---|
| `v_set` | the commanded supply voltage |
| `duty_fitted` | the fitted duty for this DShot level: 1.0 for the top level, lower levels fitted by the Kv solve. Blank for points whose level was excluded from the fit (a failed start) |
| `v_psu`, `a_psu` | the supply's own meter, read once mid-hold. At the supply's terminals rather than the stand's, so `v_psu - volts_mean` is the harness drop and `a_psu` is an independent check on the shunt value. Blank unless `--psu-port` drove the sweep |
| `esc_temp_max_c`, `esc_stress_max`, `n_edt` | as in the steady-state set. Added 2026-09-07: start behaviour near the motor's lower voltage limit changes as the ESC warms, and a Kv run had no way to record that. Read `n_edt` alongside the temperature, since a stale frame holds its last value |

## Mass-check set

`MASSCHECK` weighs known masses and fits the factor. One row per placed weight.

| column | meaning |
|---|---|
| `t_s` | seconds since the first reading |
| `mass_g` | the applied mass |
| `reading_g` | the cell's settled reading in grams |
| `reading_raw` | the cell's settled reading in raw counts |
| `reading_sd_g` | standard deviation of the settled readings |
| `n` | number of samples |
| `direction` | `up` or `down` |
| `settled` | 1 if the point settled within tolerance |

## Flags

From the `#FLAGS` banner: `1=ina_shunt_sat, 2=ina_i_sat, 4=thrust_stale,
8=rpm_stale, 16=watchdog, 32=esc_alert, 64=edt_stale`.

- `ina_shunt_sat`: shunt voltage at 98% of full scale; current may be clipped.
- `ina_i_sat`: current register at its ceiling.
- `thrust_stale`, `rpm_stale`: the sensor did not update before this row
  printed. On a summary, staleness only means one printed row repeated a value;
  the means are built from fresh samples (see `n_*`).
- `watchdog`: the host keepalive lapsed.
- `esc_alert`: the ESC raised a status alert. Consume-once: flagged rows equal
  alert events, so a row count of flagged rows is an event count, not a
  duration.
- `edt_stale`: EDT was requested (the `#ESC` banner carries `edt=1`) but no
  frame has landed this step. With `edt=0` the columns are simply unasked and
  the flag is never raised.