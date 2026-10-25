#!/usr/bin/env python3
"""Undefined-symbol audit of the CP firmware objects.

The firmware image is freestanding: no libc, and (until crt0 and the
linker script) nothing links it.  This script is the stand-in for the missing
link step: given the cross-compiled objects it runs `nm` and checks that every
undefined symbol is either

  * defined by one of the other objects (the usual case),
  * a compiler-emitted mem helper (memcpy/memset/memmove/memcmp): the image
    start-up code must provide these; reported as NOTE, or
  * a libgcc integer helper (__udivdi3, __umoddi3, ...): needs -lgcc for the
    rv32imc multilib at image link time; reported as NOTE.

Anything else (printf, malloc, strlen, ...) is a libc dependency and fails.

  python tools/check_rv_objs.py --nm riscv64-unknown-elf-nm a.o b.o ...
  python tools/check_rv_objs.py --selftest

Python >= 3.8, stdlib only.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys

MEM_HELPERS = {"memcpy", "memset", "memmove", "memcmp"}
LIBGCC_RE = re.compile(
    r"^__(?:u?(?:div|mod|mul)(?:si|di|ti)3|(?:ashl|ashr|lshr)(?:di|ti)3|"
    r"clz(?:si|di)2|ctz(?:si|di)2|popcount(?:si|di)2|bswap(?:si|di)2|"
    r"u?divmod(?:si|di)4|cmpdi2|ucmpdi2)$"
)


def parse_nm(text):
    """Return (defined, undefined) symbol sets from `nm` (BSD format) output."""
    defined, undefined = set(), set()
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in ("U", "w", "v"):
            if parts[0] == "U":
                undefined.add(parts[1])
        elif len(parts) == 3 and parts[1] not in ("U",):
            defined.add(parts[2])
    return defined, undefined


def classify(defined, undefined):
    """Split unresolved symbols into (notes_mem, notes_libgcc, forbidden)."""
    unresolved = sorted(undefined - defined)
    mem = [s for s in unresolved if s in MEM_HELPERS]
    gcc = [s for s in unresolved if LIBGCC_RE.match(s)]
    bad = [s for s in unresolved if s not in MEM_HELPERS and not LIBGCC_RE.match(s)]
    return mem, gcc, bad


def selftest():
    sample = (
        "00000000 T ngfw_memset\n"
        "         U ngfw_memset\n"
        "         U memcpy\n"
        "         U __umoddi3\n"
        "         U printf\n"
        "00000010 t local\n"
        "         w __gmon_start__\n"
    )
    d, u = parse_nm(sample)
    assert d == {"ngfw_memset", "local"}, d
    assert u == {"ngfw_memset", "memcpy", "__umoddi3", "printf"}, u
    mem, gcc, bad = classify(d, u)
    assert mem == ["memcpy"] and gcc == ["__umoddi3"] and bad == ["printf"], (mem, gcc, bad)
    d2, u2 = parse_nm("         U ngfw_a\n00000000 T ngfw_a\n")
    assert classify(d2, u2) == ([], [], [])
    print("check_rv_objs selftest OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--nm", default="nm", help="nm tool (default: nm)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("objects", nargs="*")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.objects:
        ap.error("no objects given")
    defined, undefined = set(), set()
    for obj in args.objects:
        out = subprocess.run(
            [args.nm, obj], check=True, stdout=subprocess.PIPE, universal_newlines=True
        ).stdout
        d, u = parse_nm(out)
        defined |= d
        undefined |= u
    mem, gcc, bad = classify(defined, undefined)
    for s in mem:
        print("NOTE: compiler-emitted %s: image start-up code must provide it" % s)
    for s in gcc:
        print("NOTE: libgcc helper %s: image link needs -lgcc (rv32imc multilib)" % s)
    for s in bad:
        print("ERROR: unresolved non-freestanding symbol: %s" % s)
    print(
        "%d object(s), %d defined, %d unresolved (%d mem, %d libgcc, %d forbidden)"
        % (len(args.objects), len(defined), len(undefined - defined), len(mem), len(gcc), len(bad))
    )
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
