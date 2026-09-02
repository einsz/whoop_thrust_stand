#!/usr/bin/env python3
"""End-to-end dry run of the Kv PSU path, and a timing assertion on the fix.

Wires the firmware emulator and the fake supply to the real measure.py, then
reads the supply's command log back to prove *when* the one mid-hold reading
was taken.

The firmware's hold is settle_ms of silence followed by log_ms of rows. So,
measured from the moment settle() stops polling:

    ~settle_ms          the reading came from inside the logging window  (fixed)
    ~settle_ms+log_ms   the reading came after END_HOLD, motor coasting  (bug)

One second of separation between the two, which is not a subtle call.

The sweep is pinned to one DShot level (--kv-dshots 2000). The default
"2000,1700" does two holds per voltage, so two mid-hold readings land in one
step ~3 s apart; the gap split below would then mistake the first reading for
the last settle poll and every step would fail. One level keeps the model
one-reading-per-step that the assertion is written for.
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
PTY_RE = re.compile(r"(/dev/pts/\d+)")

SETTLE_MS = 1500        # run_kv hard-codes HOLD,100,<log_ms>,1500
LOG_MS = 1000           # --hold-ms default
DSHOTS = "2000"         # one level: each voltage step gets exactly one hold,
                        # so one mid-hold reading per step. The default
                        # "2000,1700" does two holds per step, and the second
                        # reading lands ~3 s after the first -- the tool's
                        # gap split then mistakes reading 1 for the last
                        # settle poll and every step fails.


def spawn_and_get_pty(cmd, name):
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, bufsize=1)
    deadline = time.time() + 10
    while time.time() < deadline:
        line = proc.stdout.readline()
        if not line:
            break
        match = PTY_RE.search(line)
        if match:
            print("%-14s %s" % (name, match.group(1)))
            return proc, match.group(1)
    proc.kill()
    sys.exit("could not get a pty from %s" % name)


def main():
    log_path = os.path.join(PROJECT, "data", "psu_cmds.log")
    csv_path = os.path.join(PROJECT, "data", "kv_dry.csv")
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    # -u throughout: a child's stdout is a pipe here, so print() would be block
    # buffered and the pty path would never arrive.
    psu_proc, psu_pty = spawn_and_get_pty(
        [sys.executable, "-u", os.path.join(HERE, "fake_psu.py"), "--log", log_path],
        "fake supply")
    fw_proc, fw_pty = spawn_and_get_pty(
        [sys.executable, "-u", os.path.join(PROJECT, "tools", "fake_stand.py"), "--serve"],
        "fake stand")

    try:
        result = subprocess.run(
            [sys.executable, os.path.join(PROJECT, "measure.py"),
             "-m", "KV", "-p", fw_pty, "--psu-port", psu_pty,
             "-o", csv_path, "--hold-ms", str(LOG_MS),
             "--kv-dshots", DSHOTS,
             "--notes", "dry run of the Kv PSU path"],
            cwd=PROJECT, timeout=180)
        print("\nmeasure.py exited %d" % result.returncode)
    finally:
        psu_proc.terminate()
        fw_proc.terminate()
        time.sleep(0.3)

    print("\n=== supply command timing ===")
    events = []
    with open(log_path) as fh:
        for line in fh:
            parts = line.split(None, 1)
            if len(parts) == 2:
                events.append((float(parts[0]), parts[1].strip()))

    # Group into steps: each VOLT write opens one.
    steps, current = [], None
    for t, cmd in events:
        if cmd.upper().startswith("VOLT "):
            if current:
                steps.append(current)
            current = []
        elif cmd.upper() == "MEAS:ALL?" and current is not None:
            current.append(t)
    if current:
        steps.append(current)

    verdicts = []
    for i, reads in enumerate(steps):
        if len(reads) < 2:
            print("step %d: only %d reads, cannot judge" % (i + 1, len(reads)))
            continue
        # settle() polls back to back; the mid-hold reading is the one after
        # the long gap. Find the largest gap and split there.
        gaps = [(reads[j + 1] - reads[j], j) for j in range(len(reads) - 1)]
        biggest, at = max(gaps)
        lone = len(reads) - at - 1
        print("step %d: %d settle polls, gap %.3f s, then %d reading(s)"
              % (i + 1, at + 1, biggest, lone))
        verdicts.append((biggest, lone))

    if not verdicts:
        sys.exit("no steps to judge")

    print("\n=== verdict ===")
    expect_fixed = SETTLE_MS / 1000.0
    expect_bug = (SETTLE_MS + LOG_MS) / 1000.0
    ok = True
    for i, (gap, lone) in enumerate(verdicts):
        near_fixed = abs(gap - expect_fixed) < abs(gap - expect_bug)
        if lone != 1:
            print("step %d: FAIL -- %d readings after the gap, expected exactly 1"
                  % (i + 1, lone))
            ok = False
        elif near_fixed:
            print("step %d: PASS -- %.3f s after settling, inside the %.1f s log "
                  "window" % (i + 1, gap, LOG_MS / 1000.0))
        else:
            print("step %d: FAIL -- %.3f s after settling, i.e. past END_HOLD "
                  "with the motor coasting" % (i + 1, gap))
            ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
