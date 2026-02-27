#!/usr/bin/env python3
"""Extract code arrays from reference C files into compact .bin files.

Each .bin is a flat array of little-endian u64 values.
"""

import re
import struct
import sys
from pathlib import Path

REF_DIR = Path(__file__).parent.parent / "docs" / "reference-detection"
OUT_DIR = Path(__file__).parent.parent / "apriltag-gen" / "families"

FAMILIES = [
    "tag16h5",
    "tag25h9",
    "tag36h11",
    "tagCircle21h7",
    "tagCircle49h12",
    "tagCustom48h12",
    "tagStandard41h12",
    "tagStandard52h13",
]

def extract_codes(c_file: Path) -> list[int]:
    """Extract the codes array from a C file."""
    text = c_file.read_text()
    # Find the codes array
    match = re.search(r'uint64_t\s+\w+\[\d*\]\s*=\s*\{([^}]+)\}', text, re.DOTALL)
    if not match:
        raise ValueError(f"No codes array found in {c_file}")

    codes_text = match.group(1)
    codes = []
    for m in re.finditer(r'0x([0-9a-fA-F]+)', codes_text):
        codes.append(int(m.group(1), 16))
    return codes

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    for name in FAMILIES:
        c_file = REF_DIR / f"{name}.c"
        if not c_file.exists():
            print(f"WARNING: {c_file} not found, skipping")
            continue

        codes = extract_codes(c_file)
        bin_file = OUT_DIR / f"{name}.bin"

        with open(bin_file, "wb") as f:
            for code in codes:
                f.write(struct.pack("<Q", code))

        print(f"{name}: {len(codes)} codes -> {bin_file} ({bin_file.stat().st_size} bytes)")

if __name__ == "__main__":
    main()
