from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "filter_matrix.txt"
OUT = ROOT / "USFMH" / "main" / "filter_bank_coeffs.h"

PATTERN = re.compile(
    r"Filter\s+(\d+):\s+fmin\s+=\s+([0-9.]+)\s+Hz,\s+fmax\s+=\s+([0-9.]+)\s+Hz,\s+fcenter\s+=\s+([0-9.]+)\s+Hz\s*\n((?:-?[0-9]+\.[0-9]+\n)+)",
    re.MULTILINE,
)


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
    filters = parse_filters(SRC.read_text(encoding="ascii"))
    if len(filters) != 32:
        raise ValueError(f"Expected 32 filters, found {len(filters)}")
    tap_count = len(filters[0][4])
    inconsistent = [index for index, *_, coeffs in filters if len(coeffs) != tap_count]
    if inconsistent:
        raise ValueError(f"Inconsistent tap counts for filters: {inconsistent}")

    lines = [
        "#pragma once",
        "",
        "#define FILTER_BANK_NUM_BANDS 32",
        f"#define FILTER_BANK_NUM_TAPS {tap_count}",
        "",
        "static const float filter_bank_min_freqs[FILTER_BANK_NUM_BANDS] = {",
    ]
    for _, fmin, _, _, _ in filters:
        lines.append(f"    {fmin:.2f}f,")
    lines.extend([
        "};",
        "",
        "static const float filter_bank_max_freqs[FILTER_BANK_NUM_BANDS] = {",
    ])
    for _, _, fmax, _, _ in filters:
        lines.append(f"    {fmax:.2f}f,")
    lines.extend([
        "};",
        "",
        "static const float filter_bank_center_freqs[FILTER_BANK_NUM_BANDS] = {",
    ])
    for _, _, _, fcenter, _ in filters:
        lines.append(f"    {fcenter:.2f}f,")
    lines.extend([
        "};",
        "",
        "static const float filter_bank_coeffs[FILTER_BANK_NUM_BANDS][FILTER_BANK_NUM_TAPS] = {",
    ])

    for index, _, _, _, coeffs in filters:
        lines.append(f"    /* Filter {index} */ {{")
        for start in range(0, len(coeffs), 6):
            chunk = ", ".join(f"{value:.16f}f" for value in coeffs[start:start + 6])
            lines.append(f"        {chunk},")
        lines.append("    },")

    lines.append("};")
    OUT.write_text("\n".join(lines) + "\n", encoding="ascii")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
