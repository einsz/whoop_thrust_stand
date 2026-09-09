#!/usr/bin/env python3
"""Capture the motor phase voltage during a sustained full-throttle hold.

The decisive measurement for what the fitted R actually is. If the ESC chops at
full throttle then duty < 1, the model's V_motor = V_bus assumption is wrong,
and the waveform factor that reconciles a 0.225 ohm winding with a ~0.5 ohm
fitted R is real. If instead the phase shows a clean 6-step trapezoid, duty is
1, I_rms/I_avg is ~1.05, and that reconciliation is largely wrong.

Runs one DHOLD and takes scope captures from inside the logged window, so the
trace and the telemetry describe the same event.

Each capture is stamped with the machine's state while it was being digitised --
`# state,rpm_mean=...,rpm_min=...,rpm_max=...,amps=...` in the trace's own
header, taken from the rows that arrived between the start and end of that
capture. A run's mean operating point describes no single state if the motor
moved between operating points during the window, and an unstamped trace then
cannot be attributed to either one. The run warns when the speed was not steady
and marks any capture the machine changed state during.

`--trigger-below N` waits for the machine to reach a state instead of sampling
the window on a schedule. It exists for the half-speed branch, which is not a
sustained state but a 100-400 ms dip happening a few times in a 10 s hold: a
capture takes about a second to digitise, so evenly spaced rounds are unlikely
to land inside one. Armed, a round fires on the first logged row whose speed
falls below N and re-arms afterwards, so several dips in one window each get a
trace.

Trigger deep rather than close. A threshold on the shoulder of a dip fires while
the motor is still healthy -- 15,500 RPM against a ~18,000 healthy point and a
~12,000 collapsed one returned a fully healthy trace on 2026-08-20. The state a
trace caught is decided afterwards by the trace's own electrical period, never
by the threshold that fired it and never by the timestamp.

**The trigger improves the odds; it does not guarantee the trace.** `capture()`
reads whatever acquisition is on the scope's screen at that moment -- there is
no single-shot arm-and-wait in this instrument's command set -- so what comes
back is the last screen the scope acquired, not one acquired on the trigger.
Between the ESC's telemetry frame and the SCPI read sit the row, USB and the
host, so a capture can still describe the healthy state either side of a dip.
That is precisely why every trace is verified by its own period, and why a
capture that fired below the threshold is not by itself a collapsed trace.
"""
import argparse
import datetime
import importlib.util
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def arms_trigger(row, last_n_rpm, threshold, floor):
    """Should this row fire a capture?

    Split out from the read loop so it can be exercised without a scope, a
    board or a spinning motor -- the three reasons the trigger went untested
    the first time it was needed.

    Two rows are refused that a naive speed test would accept. A row repeating
    the previous telemetry frame is refused because rows print at 250 Hz over a
    much slower frame rate, so a stale `rpm` describes a state the motor may
    already have left. A row below `floor` is refused because a motor that
    never started sits at a few hundred RPM, under any threshold worth setting.
    """
    if not threshold:
        return False
    if row["n_rpm"] == last_n_rpm:
        return False
    return floor <= row["rpm"] < threshold


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dshot", type=int, default=2000)
    ap.add_argument("--log-ms", type=int, default=5000)
    ap.add_argument("--settle-ms", type=int, default=1500)
    ap.add_argument("--captures", type=int, default=3)
    ap.add_argument("--trigger-below", type=int, default=0, metavar="RPM",
                    help="Capture only once the reported speed drops below "
                         "this, instead of spacing captures through the "
                         "window. Set it well inside the state you are after, "
                         "not on its shoulder.")
    ap.add_argument("--channels", default="CH1",
                    help="Comma separated, e.g. CH1,CH2")
    # Timestamped by default. A fixed tag silently overwrote a healthy baseline
    # on 2026-09-06, and a scope capture of a fault that fades is not something
    # you can go back and retake.
    ap.add_argument("--tag", default=None,
                    help="filename prefix; defaults to phase-<UTC timestamp>")
    ap.add_argument("--outdir", default=os.path.join(ROOT, "data"))
    args = ap.parse_args()
    if args.log_ms > 10000:
        # The firmware clamps the logged window to 10 s (runHoldSequence), so a
        # longer ask would silently become a shorter run -- refuse up front, the
        # way tone_hold.py does.
        sys.exit("--log-ms %d exceeds the firmware cap of 10000 ms; the board "
                 "would clamp it and the run would be shorter than asked."
                 % args.log_ms)

    measure = load(os.path.join(ROOT, "measure.py"), "measure")
    scopemod = load(os.path.join(ROOT, "tools", "scope.py"), "scope")

    scope = scopemod.Scope()
    print("scope: %s" % scope.identify())
    channels = [c.strip() for c in args.channels.split(",") if c.strip()]
    for ch in channels:
        state, head = scope.channel_state(ch)
        print("       %s %s/div, %s coupling, probe %s, timebase %s/div"
              % (ch, state["SCALE"], state["COUPLING"], state["PROBE"],
                 head["TIMEBASE"]["SCALE"]))
        if state["COUPLING"].upper() != "DC":
            sys.exit("[ERROR] %s is AC coupled -- that removes the DC content, "
                     "which is the measurement. Set DC first." % ch)

    link = measure.Link("/dev/ttyACM0", measure.BAUD_RATE)
    link.write_line("ID")
    deadline = time.time() + 3.0
    while time.time() < deadline and link.cols is None:
        line = link.readline()
        if line.startswith("#"):
            link.absorb_meta(line)
    if link.cols is None:
        sys.exit("[ERROR] no banner from the firmware")
    link.start_keepalive()

    captures, rows = [], []
    stop_capturing = threading.Event()
    triggered = threading.Event()
    trigger_rpm = [0]

    def wait_for_trigger():
        """Block until a row comes in below the threshold. False to give up."""
        while not stop_capturing.is_set():
            if triggered.wait(0.05):
                return True
        return False

    def capture_worker():
        # Spaced through the logged window rather than all at once, so a
        # one-off artefact is distinguishable from the steady waveform. With
        # --trigger-below the spacing is the machine's instead: each round
        # waits for the state to arrive.
        for i in range(args.captures):
            if args.trigger_below:
                if not wait_for_trigger():
                    return
                fired_at = trigger_rpm[0]
                print("   round %d armed at %d RPM, capturing..."
                      % (i + 1, fired_at))
            elif stop_capturing.wait(0.2):
                return
            try:
                # Both channels inside one round, back to back, so the pair
                # describes the same commutation cycles as closely as the
                # instrument allows. They are still not simultaneous -- see the
                # note in the analysis before differencing them sample by
                # sample.
                for ch in channels:
                    # Wall clock either side of the capture, so the rows that
                    # arrived while the instrument was digitising can be found
                    # afterwards. Without it a trace carries no record of what
                    # the motor was doing, and on a machine that changes state
                    # within a hold that makes the trace uninterpretable.
                    t0 = time.time()
                    cap = scope.capture(ch)
                    cap["t_start"], cap["t_end"] = t0, time.time()
                    cap["round"] = i + 1
                    if args.trigger_below:
                        cap["trigger_rpm"] = fired_at
                    captures.append(cap)
                    lo, hi = min(cap["volts"]), max(cap["volts"])
                    print("   round %d %s: %.3f .. %.3f V" % (i + 1, ch, lo, hi))
            except scopemod.ScopeError as exc:
                print("   round %d failed: %s" % (i + 1, exc))
            finally:
                # Re-arm. A dip is over long before the transfer finishes, so
                # the next round has to wait for the next one rather than fire
                # on this one's stale event.
                triggered.clear()

    try:
        print("\nholding dshot %d for %d ms (settle %d)..."
              % (args.dshot, args.log_ms, args.settle_ms))
        if args.trigger_below:
            print("armed: capturing when the reported speed drops below %d RPM"
                  % args.trigger_below)
        link.write_line("DHOLD,%d,%d,%d" % (args.dshot, args.log_ms, args.settle_ms))
        recording = False
        worker = None
        last_n_rpm = None
        # Bounded like every other motor-driving harvest: a board that stalls
        # without printing END_HOLD must end the run, not hang it with the
        # watchdog fed and the motor held.
        quiet = measure.SilenceTimer("END_HOLD")
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
            row["_t"] = time.time()
            rows.append(row)
            if args.trigger_below and not triggered.is_set():
                if arms_trigger(row, last_n_rpm, args.trigger_below,
                                measure.MIN_RUNNING_RPM):
                    trigger_rpm[0] = row["rpm"]
                    triggered.set()
                last_n_rpm = row["n_rpm"]
            if worker is None:
                # The first logged row is proof the window is running; the
                # settle delay prints nothing.
                worker = threading.Thread(target=capture_worker, daemon=True)
                worker.start()
        stop_capturing.set()
        if worker:
            worker.join(timeout=5)
    finally:
        link.write_line("STOP")
        time.sleep(0.2)
        link.close()
        scope.close()

    hold = [r for r in rows if r["phase"] != 8]
    if hold:
        rpm = sum(r["rpm"] for r in hold) / len(hold)
        volts = sum(r["bus_mv"] for r in hold) / len(hold) / 1000.0
        amps = sum(r["bus_ua"] for r in hold) / len(hold) / 1e6
        lo, hi = min(r["rpm"] for r in hold), max(r["rpm"] for r in hold)
        print("\noperating point: %.0f RPM, %.3f V bus, %.3f A  (%d rows)"
              % (rpm, volts, amps, len(hold)))
        # A mean is not a state. If the motor moved between operating points
        # during the window this says so, and every per-capture figure below
        # matters rather than being a formality.
        if lo and (hi - lo) / lo > 0.05:
            print("[WARN] speed was NOT steady: %d..%d RPM, %.0f%% spread."
                  % (lo, hi, (hi - lo) / lo * 100.0))
            print("       The mean above describes no single state. Read each"
                  " capture against its own stamp.")

    def state_at(cap):
        """The machine's state while this trace was being digitised."""
        seg = [r for r in hold
               if cap["t_start"] <= r.get("_t", 0) <= cap["t_end"]]
        if not seg:
            return None
        rpms = [r["rpm"] for r in seg]
        return {
            "n": len(seg),
            "rpm": sum(rpms) / len(rpms),
            "rpm_min": min(rpms),
            "rpm_max": max(rpms),
            "amps": sum(r["bus_ua"] for r in seg) / len(seg) / 1e6,
        }

    os.makedirs(args.outdir, exist_ok=True)
    if args.tag is None:
        args.tag = "phase-" + datetime.datetime.now(
            datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    print()
    for cap in captures:
        st = state_at(cap)
        meta = []
        if "trigger_rpm" in cap:
            # What armed the round, kept apart from the state stamp on purpose.
            # The threshold that fired says what was being hunted; only the
            # trace's own electrical period says what was caught.
            meta.append("trigger,below=%d,fired_at_rpm=%d"
                        % (args.trigger_below, cap["trigger_rpm"]))
        if st:
            meta.append("state,rpm_mean=%.0f,rpm_min=%d,rpm_max=%d,amps=%.4f,rows=%d"
                        % (st["rpm"], st["rpm_min"], st["rpm_max"], st["amps"], st["n"]))
            steady = st["rpm_min"] and (st["rpm_max"] - st["rpm_min"]) / st["rpm_min"] < 0.05
            meta.append("state_steady,%s" % ("yes" if steady else "no"))
            print("   round %d %s: %.0f RPM (%d..%d), %.3f A%s"
                  % (cap["round"], cap["channel"], st["rpm"], st["rpm_min"],
                     st["rpm_max"], st["amps"], "" if steady else "   <-- CHANGED MID-CAPTURE"))
        else:
            meta.append("state,unknown")
            print("   round %d %s: no telemetry overlapped this capture"
                  % (cap["round"], cap["channel"]))
        path = os.path.join(args.outdir, "%s-%s-%d.csv"
                            % (args.tag, cap["channel"].lower(), cap["round"]))
        scopemod.write_capture(path, cap, extra_meta=meta)
    if not captures:
        if args.trigger_below:
            # Say which of the two happened. A window the fault never entered
            # and a window it entered without a capture landing are different
            # results, and neither is evidence the fault has gone away.
            floor = min((r["rpm"] for r in hold), default=0)
            print("[INFO] trigger never fired: nothing below %d RPM in this "
                  "window (minimum %d)." % (args.trigger_below, floor))
            print("       That is one quiet window, not a fix.")
        else:
            print("[WARN] no scope captures were taken")
    elif args.trigger_below:
        fired = sum(1 for c in captures if c["channel"] == channels[0])
        print("\n%d of %d rounds fired below %d RPM."
              % (fired, args.captures, args.trigger_below))
        print("Read each trace by its own electrical period before calling it "
              "collapsed -- ~573 us is 18,000 RPM, ~860 us is 12,000.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
