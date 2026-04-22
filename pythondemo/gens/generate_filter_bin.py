#!/usr/bin/env python3
"""
Generate a binary FIR filter bank file for the ESP32 SD card.

Reads the MATLAB-generated filter_matrix.txt and writes a compact binary
file (filters.bin) that the firmware loads directly into PSRAM.

Binary layout (little-endian):
  magic       uint32  0x46495242 ("FIRB")
  num_bands   uint32
  num_taps    uint32
  min_freqs   float32[num_bands]
  max_freqs   float32[num_bands]
  center_freqs float32[num_bands]
  coeffs      float32[num_bands * num_taps]   (row-major)

Usage:
  python generate_filter_bin.py                     # default: filter_matrix.txt -> filters.bin
  python generate_filter_bin.py input.txt out.bin   # custom paths
"""

import struct
import sys
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]

PATTERN = re.compile(
    r"Filter\s+(\d+):\s+fmin\s+=\s+([0-9.]+)\s+Hz,\s+fmax\s+=\s+([0-9.]+)\s+Hz,\s+fcenter\s+=\s+([0-9.]+)\s+Hz\s*\n((?:-?[0-9]+\.[0-9]+\n)+)",
    re.MULTILINE,
)

MAGIC = 0x46495242  # "FIRB"


def parse_filters(text: str):
    filters = []
    for match in PATTERN.finditer(text):
        index = int(match.group(1))
        fmin = float(match.group(2))
        fmax = float(match.group(3))
        fcenter = float(match.group(4))
        coeffs = [float(line) for line in match.group(5).strip().splitlines()]
        filters.append((index, fmin, fmax, fcenter, coeffs))
    return filters


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "filter_matrix.txt"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "filters.bin"

    filters = parse_filters(src.read_text(encoding="ascii"))
    if len(filters) == 0:
        raise ValueError(f"No filters found in {src}")

    num_bands = len(filters)
    num_taps = len(filters[0][4])

    inconsistent = [index for index, *_, coeffs in filters if len(coeffs) != num_taps]
    if inconsistent:
        raise ValueError(f"Inconsistent tap counts for filters: {inconsistent}")

    print(f"Packing {num_bands} bands × {num_taps} taps")

    with open(out, "wb") as f:
        # Header
        f.write(struct.pack("<III", MAGIC, num_bands, num_taps))

        # Frequency arrays
        for _, fmin, _, _, _ in filters:
            f.write(struct.pack("<f", fmin))
        for _, _, fmax, _, _ in filters:
            f.write(struct.pack("<f", fmax))
        for _, _, _, fcenter, _ in filters:
            f.write(struct.pack("<f", fcenter))

        # Coefficient matrix (row-major: band 0 all taps, band 1 all taps, ...)
        for _, _, _, _, coeffs in filters:
            f.write(struct.pack(f"<{num_taps}f", *coeffs))

    size_kb = out.stat().st_size / 1024
    print(f"Wrote {out} ({size_kb:.1f} KB)")
    print(f"Copy to SD card as: filters.bin")


if __name__ == "__main__":
    main()
