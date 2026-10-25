#!/usr/bin/env python3
"""Convert a raw little-endian binary (objcopy -O binary of the CP firmware ELF)
into the $readmemh word file used to preload the Ibex IMEM in simulation
(xg_cp_ibex_wrap IMAGE_FILE): one 32-bit word per line, zero padded to 4 bytes.

  python tools/bin2hex.py fw.bin fw_imem.hex [--words N]

--words pads the file with zero words up to N lines (default: no padding).
Python >= 3.8, stdlib only.
"""
import argparse
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("hex")
    ap.add_argument("--words", type=int, default=0)
    a = ap.parse_args()
    data = open(a.bin, "rb").read()
    data += b"\0" * (-len(data) % 4)
    words = [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]
    if a.words:
        if len(words) > a.words:
            print("image has %d words, limit %d" % (len(words), a.words), file=sys.stderr)
            return 1
        words += [0] * (a.words - len(words))
    with open(a.hex, "w", newline="\n") as f:
        for w in words:
            f.write("%08x\n" % w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
