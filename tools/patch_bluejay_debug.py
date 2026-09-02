#!/usr/bin/env python3
"""Patch a released Bluejay hex so its debug EDT frames carry Comm_Period4x.

Why this exists: Bluejay assembles with Keil AX51/LX51 and the CI toolchain
archive is password-protected, so reaching for a build is a detour. It is not a
dead end -- Silicon Labs licenses the full Keil PK51 free for their own MCUs,
which removes the evaluation build's 2 KB cap -- but the change we want for
diagnosis needs no reassembly at all, and this works on any released hex with no
toolchain of any kind.

Bluejay v0.21.0 ships two *live* EDT scheduler steps that send hardcoded stubs
(`src/Modules/Scheduler.asm`):

    scheduler_steps_odd_debug1_frame:   mov Ext_Telemetry_L, #088h
                                        mov Ext_Telemetry_H, #08h   ; frame id
    scheduler_steps_odd_debug2_frame:   mov Ext_Telemetry_L, #0AAh
                                        mov Ext_Telemetry_H, #0Ah   ; frame id

`MOV direct,#imm` and `MOV direct,direct` are both three bytes on the 8051, so
each stub can be rewritten in place with no relocation and no length change:

    75 65 88  ->  85 42 65     Ext_Telemetry_L = Comm_Period4x_L
    75 65 AA  ->  85 43 65     Ext_Telemetry_L = Comm_Period4x_H

The frame-id stores are left alone, so the frames keep arriving as DEBUG_FRAME_1
and DEBUG_FRAME_2 and decode normally. The result is the ESC's *internal*
commutation-period accumulator, streamed out beside the period it reports --
which is what separates a wrong period measurement from a wrong encoding, and
tests the timebase hypothesis directly.

IRAM addresses come from the `DSEG AT 30h` block in `src/Bluejay.asm`:
Comm_Period4x_L=0x42, Comm_Period4x_H=0x43, Ext_Telemetry_L=0x65,
Ext_Telemetry_H=0x66. The stub pattern itself confirms the Ext_Telemetry pair,
and `--verify` cross-checks the Comm_Period4x pair against other instructions.

**These addresses are for a stock release hex.** They are assigned by order of
declaration, so any source change that adds a byte to that block shifts every
variable below it. The remainder-carry fix for the period filter does exactly
that: it declares one byte immediately after Comm_Period4x_H, moving
Ext_Telemetry and everything after it up by one. Run `--verify` before trusting
a patch against any hex that is not a stock release, or this writes the ESC's
telemetry into the wrong variable and reports it as success.

Caveats worth knowing before flashing:

  * L and H are sent by different scheduler steps, so they are sampled at
    slightly different times. At high RPM the high byte is ~1 and nearly static,
    so this is not a practical problem, but do not treat the pair as atomic.
  * On the reference ESC, EDT stops once the motor has armed and only a power
    cycle revives it, so expect the debug frames only during the first sequence
    after power-up. One long hold per power cycle.
  * This is a diagnostic build. It does not fix anything, and it changes what
    the debug frames mean, so do not leave it on an ESC you fly.

Usage:
    python tools/patch_bluejay_debug.py IN.hex -o OUT.hex
    python tools/patch_bluejay_debug.py IN.hex --verify        # inspect only
"""

import argparse
import sys

# (name, search pattern, replacement for the first 3 bytes)
PATCHES = [
    ("debug1 -> Comm_Period4x_L", "756588756608", "854265"),
    ("debug2 -> Comm_Period4x_H", "7565AA75660A", "854365"),
]

# Bluejay puts its EEPROM copy and bootloader above the application. Patching
# into either would be a different kind of mistake than a wrong opcode, so it is
# checked rather than assumed. 0x1A00 is the lowest CSEG_EEPROM of any variant.
APP_LIMIT = 0x1A00


def parse_hex(path):
    """Returns (records, image) where records keep the original file structure."""
    records = []
    image = {}
    for lineno, raw in enumerate(open(path), 1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError("%s:%d is not an Intel HEX record" % (path, lineno))
        body = bytes.fromhex(line[1:])
        count, addr, rtype = body[0], (body[1] << 8) | body[2], body[3]
        data = bytearray(body[4:4 + count])
        if (sum(body[:-1]) + body[-1]) & 0xFF:
            raise ValueError("%s:%d bad record checksum" % (path, lineno))
        records.append({"count": count, "addr": addr, "type": rtype, "data": data})
        if rtype == 0:
            for i, value in enumerate(data):
                image[addr + i] = value
    return records, image


def find(image, pattern):
    """Byte-offset(s) of `pattern` in the contiguous parts of `image`."""
    want = bytes.fromhex(pattern)
    hits = []
    for start in sorted(image):
        if all(image.get(start + i) == want[i] for i in range(len(want))):
            hits.append(start)
    return hits


def emit(records, path):
    with open(path, "w") as fh:
        for rec in records:
            body = bytes([rec["count"], rec["addr"] >> 8, rec["addr"] & 0xFF,
                          rec["type"]]) + bytes(rec["data"])
            fh.write(":%s%02X\n" % (body.hex().upper(), (-sum(body)) & 0xFF))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="Official Bluejay .hex for YOUR layout/MCU/deadtime/PWM")
    ap.add_argument("-o", "--output", help="Where to write the patched hex")
    ap.add_argument("--verify", action="store_true",
                    help="Locate the stubs and cross-check addresses, write nothing")
    args = ap.parse_args()

    records, image = parse_hex(args.input)
    print("%s: 0x%04X..0x%04X, %d code bytes"
          % (args.input, min(image), max(image), len(image)))

    # Independent evidence that Comm_Period4x really is at 0x42/0x43 in this
    # build: these opcodes should appear if the DSEG layout matches the source.
    print("\nAddress cross-check (expect all non-zero):")
    for label, pat in (("mov Comm_Period4x_L,A  F5 42", "F542"),
                       ("mov A,Comm_Period4x_L  E5 42", "E542"),
                       ("mov A,Comm_Period4x_H  E5 43", "E543")):
        print("  %-30s %d site(s)" % (label, len(find(image, pat))))

    targets = []
    print("\nStub locations:")
    for name, pattern, replacement in PATCHES:
        hits = find(image, pattern)
        print("  %-28s %s" % (name, [hex(h) for h in hits] or "NOT FOUND"))
        if len(hits) != 1:
            sys.exit("[ERROR] expected exactly one site for %s, found %d. This hex "
                     "may be a different Bluejay version -- do not patch it blind."
                     % (name, len(hits)))
        if hits[0] >= APP_LIMIT:
            sys.exit("[ERROR] %s sits at 0x%04X, at or above the EEPROM/bootloader "
                     "boundary 0x%04X. Refusing." % (name, hits[0], APP_LIMIT))
        targets.append((name, hits[0], bytes.fromhex(replacement)))

    if args.verify:
        print("\nVerify only, nothing written. Re-run with -o to patch.")
        return
    if not args.output:
        sys.exit("[ERROR] give -o OUT.hex, or use --verify")

    # Patch inside the original records so the file structure is untouched and
    # only the affected records' checksums change.
    changed = 0
    for name, addr, replacement in targets:
        before = bytes(image[addr + i] for i in range(len(replacement)))
        for rec in records:
            if rec["type"] != 0:
                continue
            for i in range(len(replacement)):
                off = addr + i - rec["addr"]
                if 0 <= off < rec["count"]:
                    rec["data"][off] = replacement[i]
                    changed += 1
        print("  0x%04X  %s -> %s   (%s)"
              % (addr, before.hex().upper(), replacement.hex().upper(), name))
    if changed != sum(len(r) for _, _, r in targets):
        sys.exit("[ERROR] patched %d bytes, expected %d -- a stub straddles a record "
                 "boundary in an unexpected way. Refusing to write."
                 % (changed, sum(len(r) for _, _, r in targets)))

    emit(records, args.output)
    print("\nWrote %s" % args.output)
    print("Keep the original: reflashing it undoes this completely.")
    print("Debug frames now carry Comm_Period4x, not 0x88/0xAA.")


if __name__ == "__main__":
    main()
