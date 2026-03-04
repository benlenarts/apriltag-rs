#!/usr/bin/env python3
"""Analyze a samply Firefox Profiler JSON profile.

Produces three views:
  - Self-time (leaf functions) — where CPU time is actually spent
  - Inclusive time (on-stack) — which functions are on the critical path
  - Top collapsed stacks — full call chains for deep analysis
"""
import json
import sys
from collections import defaultdict


def load_profile(path="/tmp/apriltag-profile.json"):
    with open(path) as f:
        return json.load(f)


def load_syms(path="/tmp/apriltag-profile.json.syms.json"):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return None


def build_sym_table(syms):
    """Build a map from hex address string to symbol name."""
    if not syms:
        return {}
    table = {}
    string_table = syms.get("string_table", [])
    for lib in syms.get("data", []):
        for entry in lib.get("symbol_table", []):
            rva = entry.get("rva", 0)
            size = entry.get("size", 0)
            sym_idx = entry.get("symbol", 0)
            name = string_table[sym_idx] if sym_idx < len(string_table) else f"<unknown_{sym_idx}>"
            for addr in range(rva, rva + max(size, 1)):
                hex_addr = f"0x{addr:x}"
                table[hex_addr] = name
    return table


def resolve_name(string_array, func_table, frame_table, frame_idx, sym_table):
    """Resolve a frame index to a symbol name."""
    func_idx = frame_table["func"][frame_idx]
    name_idx = func_table["name"][func_idx]
    name = string_array[name_idx]
    if name.startswith("0x") and sym_table:
        resolved = sym_table.get(name)
        if resolved:
            return resolved
    return name


def walk_stack(stack_table, stack_idx):
    """Walk a stack from leaf to root, yielding stack indices."""
    while stack_idx is not None:
        yield stack_idx
        stack_idx = stack_table["prefix"][stack_idx]


def analyze(profile_path="/tmp/apriltag-profile.json", syms_path=None):
    if syms_path is None:
        syms_path = profile_path + ".syms.json"

    profile = load_profile(profile_path)
    syms = load_syms(syms_path)
    sym_table = build_sym_table(syms)

    thread = profile["threads"][0]
    samples = thread["samples"]
    stack_table = thread["stackTable"]
    frame_table = thread["frameTable"]
    func_table = thread["funcTable"]
    string_array = thread["stringArray"]

    self_time = defaultdict(int)
    inclusive_time = defaultdict(int)
    collapsed_stacks = defaultdict(int)

    sample_stacks = samples["stack"]
    total_samples = len(sample_stacks)

    for stack_idx in sample_stacks:
        if stack_idx is None:
            continue

        frames = []
        for si in walk_stack(stack_table, stack_idx):
            fi = stack_table["frame"][si]
            name = resolve_name(string_array, func_table, frame_table, fi, sym_table)
            frames.append(name)

        if not frames:
            continue

        # Self time = leaf function
        self_time[frames[0]] += 1

        # Inclusive time = all functions in the stack
        seen = set()
        for name in frames:
            if name not in seen:
                inclusive_time[name] += 1
                seen.add(name)

        # Collapsed stack (root to leaf, semicolon-separated)
        collapsed = ";".join(reversed(frames))
        collapsed_stacks[collapsed] += 1

    # Print results
    print(f"=== SELF TIME (leaf functions) — {total_samples} total samples ===")
    print(f"{'%':>6}  {'samples':>8}  function")
    print("-" * 80)
    for name, count in sorted(self_time.items(), key=lambda x: -x[1])[:30]:
        pct = 100.0 * count / total_samples
        print(f"{pct:5.1f}%  {count:8d}  {name}")

    print(f"\n=== INCLUSIVE TIME (on-stack) — {total_samples} total samples ===")
    print(f"{'%':>6}  {'samples':>8}  function")
    print("-" * 80)
    for name, count in sorted(inclusive_time.items(), key=lambda x: -x[1])[:30]:
        pct = 100.0 * count / total_samples
        print(f"{pct:5.1f}%  {count:8d}  {name}")

    print(f"\n=== TOP COLLAPSED STACKS ===")
    for stack, count in sorted(collapsed_stacks.items(), key=lambda x: -x[1])[:20]:
        pct = 100.0 * count / total_samples
        parts = stack.split(";")
        if len(parts) > 8:
            display = ";".join(parts[:3]) + ";...;" + ";".join(parts[-4:])
        else:
            display = stack
        print(f"{pct:5.1f}%  {count:8d}  {display}")


def filter_function(profile_path, syms_path, func_name):
    """Print collapsed stacks containing a specific function."""
    if syms_path is None:
        syms_path = profile_path + ".syms.json"

    profile = load_profile(profile_path)
    syms = load_syms(syms_path)
    sym_table = build_sym_table(syms)

    thread = profile["threads"][0]
    samples = thread["samples"]
    stack_table = thread["stackTable"]
    frame_table = thread["frameTable"]
    func_table = thread["funcTable"]
    string_array = thread["stringArray"]

    stacks = defaultdict(int)
    total = 0

    for stack_idx in samples["stack"]:
        if stack_idx is None:
            continue
        total += 1

        frames = []
        for si in walk_stack(stack_table, stack_idx):
            fi = stack_table["frame"][si]
            name = resolve_name(string_array, func_table, frame_table, fi, sym_table)
            frames.append(name)

        collapsed = ";".join(reversed(frames))
        if func_name in collapsed:
            stacks[collapsed] += 1

    print(f"=== Stacks containing '{func_name}' — {sum(stacks.values())}/{total} samples ===")
    for stack, count in sorted(stacks.items(), key=lambda x: -x[1])[:30]:
        pct = 100.0 * count / total
        print(f"{pct:5.1f}%  {count:8d}  {stack}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--filter":
        func_name = sys.argv[2]
        p = sys.argv[3] if len(sys.argv) > 3 else "/tmp/apriltag-profile.json"
        s = sys.argv[4] if len(sys.argv) > 4 else None
        filter_function(p, s, func_name)
    else:
        p = sys.argv[1] if len(sys.argv) > 1 else "/tmp/apriltag-profile.json"
        s = sys.argv[2] if len(sys.argv) > 2 else None
        analyze(p, s)
