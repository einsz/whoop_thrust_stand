# Calibrating the load cell

The load cell factor converts raw ADC counts to grams. It belongs to the
load cell **and** the amplifier fitted, so swapping between an HX711 and a
NAU7802 means recalibrating: the gain paths differ, and the sign may flip with
the wiring. It multiplies every thrust and g/W number in the dataset, and a
wrong value produces smooth, plausible curves rather than an obvious failure.

**It is not in the firmware.** The board reports tared raw counts and
`measure.py` does the conversion, so recalibrating is never a reflash. That
matters, because the factor is a property of the *mounting* and can move
between sessions. The factor lives in `stand.json` next to `measure.py`, and
`MASSCHECK` writes it there for you.

Nothing needs to be configured before your first calibration. The factor is
fitted directly from raw counts against known masses, so a stand with no
calibration at all can derive one from scratch in a single pass.

## Declaring your stand's geometry

How your cell is mounted determines the sign of the result, and the tool needs
to be told which case you are in.

| your stand | flag | an applied mass reads |
|---|---|---|
| Applying a mass loads the cell the same way thrust does | *(none)* | positive |
| Motor points up, so thrust *unloads* the cell | `--thrust-unloads` | negative |
| You can stack several weights at once | `--ballast` | positive (implies `--thrust-unloads`) |

The geometry sets the **sign** of the factor, and on a first calibration the mass
data cannot check it for you, because applying weights looks the same either
way. The tool says so, and asks you to confirm with a brief spin: thrust must
read positive. On any later calibration a sign disagreement with the stored factor is
flagged loudly, since it means either the flag changed or the cell was remounted.

Getting it wrong costs nothing. The readings are already in the CSV, so refit
rather than re-weigh:

```bash
python measure.py --refit data/cal.csv --thrust-unloads
```

## Running it

```bash
python measure.py -m MASSCHECK --masses 10,20,30,50 -o data/cal.csv
```

You get one prompt per weight. Place it, let the stand settle, press Enter. Add
`--descend` to walk back down as well, which measures hysteresis and takes
roughly twice as long.

If you can stack weights, `--ballast` is the better method where the mechanics
allow it: load them all, tare, then remove one at a time. That reproduces
thrust in both direction and range. If your stand only holds one weight at a
time, use the plain form.

The factor is written to `stand.json` automatically. **Re-run to verify.** The
second pass should report a scale error near 0.00%. Use `--no-save` if you want
to see the number without storing it, or `--scale <factor>` to override the
stored value for one run.

On a first calibration the readings print as raw counts rather than grams,
since there is no factor yet to convert them with. The settling tolerance is
then set from the cell's own measured noise instead of a fixed number of grams.

## Choosing masses

- **Span your actual thrust range.** Calibrating over 10–50 g and then measuring
  200 g is extrapolation.
- **Prefer larger masses.** Zero noise is a fixed number of grams, so it is a
  much larger fraction of a 5 g point than a 50 g one. Small points also
  contribute almost nothing to a through-origin fit.
- **Repeating one mass** (`--masses 20,20,20,20,20`) measures *your* placement
  repeatability, which is the floor on everything else.

## Reading the result

```
Fit: -17155.6706 counts/g   (through origin, 9 points, expected slope -1)
      referenced to measured zero of -0.0478 g
  scale error   -0.00%   good   (factor in force was -17155.4)
  linearity     0.068 g worst deviation (0.14% of full scale)
  up/down gap   0.112 g between legs at the same mass
  zero drift    -0.079 g from first zero point to last (-0.026 g/min)
  hysteresis    0.020 g once that drift is removed
```

- **The factor is fitted from raw counts**, not as a correction to an existing
  one. A first calibration therefore needs no starting value, and a later one is
  not biased by whatever was stored before. The `scale error` line compares the
  freshly derived factor against the one that was in force.
- **Sensitivity is fitted through the origin**, referenced to the run's own
  measured zero, so a tare that has drifted does not tilt the line. Under
  `--ballast` that zero is recorded right after taring, with nothing removed
  yet. Without it there would be no measured zero in a procedure where every
  other point is a removal.
- **The up/down gap is not automatically hysteresis.** A zero that walks during
  the run displaces every down-leg point by the same amount, which looks
  load-dependent but is not. Rows carry `t_s` so the fit subtracts a linear zero
  trend and reports the two separately. Real hysteresis grows with load, drift
  does not.
- **Points that never settled are marked unusable.** Measuring the settling
  transient rather than the value is good for 20–30% errors.

## Things worth knowing

**Check the zero is not walking before touching weights.**

```bash
python measure.py --check --tare --watch 180
```

Three minutes of drift measurement. Sensitivity is worth re-deriving only once
the zero holds still. Nothing in the fit corrects a zero that moves, and on some
mounts that term dominates the entire error budget.

**After any mechanical change, calibrate twice and keep the second.** Touching
the rig, whether re-torquing, remounting or changing a bracket, leaves the
joints to bed in, and the first run afterwards measures that settling rather
than the cell. On the reference build the first calibration after re-torquing every
screw reported 0.065 g of hysteresis and a 2.70% sensitivity shift; repeating
it unchanged gave 0.017 g and 0.07%.

**Judge settling by the shape numbers, not by the factor.** Two runs either
side of a bedding-in cycle can report almost the same sensitivity while being
worlds apart in quality. What moves is linearity (0.300 g → 0.041 g),
hysteresis (0.361 g → 0.048 g) and zero drift (−0.203 g/min → +0.009 g/min).
Comparing factors alone would have called both runs settled.

A bedding-in run can also mimic a hardware fault convincingly. The first one
slipped at its top mass and left the zero displaced by 0.42 g, which reads
exactly like a loose fastener and was not one.

A related symptom, useful because it costs nothing to look for: **high zero
drift straight after mechanical work means unbedded joints, not a loose screw.**
`--check --tare --watch` will show it before any weight goes on. One load cycle
clears it.

**But two runs an hour apart are not enough, and this is the trap.** Those two
post-re-torque calibrations agreed to 0.07%, which reads as settled and is not. Thirty-six hours and a session of running later, with nothing adjusted,
the factor had relaxed to within 0.01% of its *pre*-re-torque value. The 2.70%
was a transient the whole time, and both runs sat inside it.

Agreement between two nearby runs measures repeatability, not settling. Treat a
factor measured soon after a mechanical change as provisional, and re-check it
the next time you use the rig.

**Calibrate at both ends of a session.** One check at the start tells you the
factor was right when you began; it says nothing about whether it still was
when you finished. Two bracketing runs cost a few minutes and turn an unknown
into a bound. If they agree, everything between them is good. If they do not,
you know by how much and can re-derive the affected runs from their raw counts
with `--rescale`.

Without a closing check, a session's absolute thrust cannot be falsified, only
trusted.

**Re-derive per session, not once.** On the reference build a freshly derived
factor verified at +0.40%, and the same procedure one day later found it 3.66%
low, with nothing knowingly touched in between. A mount that can shift several
percent without visibly moving makes the factor a per-session measurement, not
a property of the cell. Check yours before trusting a stored value.

**Record your error budget.** Run the calibration a few times and write down
what you get: scale error, within-session repeatability, zero drift across a
run, hysteresis. Quote thrust to that, rather than to the resolution of the ADC.

**A run taken under a wrong factor is not lost.** Every CSV records the raw
counts alongside the grams and names the factor used, so it can be re-derived:

```bash
python measure.py --rescale data/run1.csv -17155.7
```

This writes `run1.rescaled.csv` and blanks efficiency, normalised thrust and
figure of merit, which are derived from thrust and must be recomputed.

**Cross-check against physics.** Pass `--prop-diameter-mm` on a sweep to enable
the actuator-disk QC gate. It rejects any row claiming better efficiency than
an ideal disk of that diameter could achieve. That catches a wrong scale factor,
a wrong shunt value and clipped current, without your having to anticipate the
failure. A whoop-class figure of merit lands around 0.2–0.3; above 1.0 is
physically impossible and means a constant is wrong.
