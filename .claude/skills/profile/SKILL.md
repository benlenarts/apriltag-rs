---
name: profile
description: Profile Rust binaries with samply and analyze results
allowed-tools: Bash(python *), Bash(samply *), Bash(cargo build *), Bash(gunzip *)
---

# /profile — Samply Profiling Skill

Profile Rust binaries using samply and analyze the results.

## Usage

```
/profile [command-to-profile]
```

If no command is given, defaults to the apriltag noise profiling loop.

## Workflow

### 1. Build and record

```bash
cargo build -p apriltag-bench --release
samply record --save-only --unstable-presymbolicate \
  -o /tmp/apriltag-profile.json.gz \
  -- ./target/release/apriltag-bench profile --noise 20 --iterations 1000
```

Or with a catalog scenario:

```bash
samply record --save-only --unstable-presymbolicate \
  -o /tmp/apriltag-profile.json.gz \
  -- ./target/release/apriltag-bench profile --scenario noise-sigma20 --iterations 1000
```

Via justfile:

```bash
just sim profile --noise 20 --iterations 1000
just sim profile --scenario noise-sigma20 --iterations 1000
```

### 2. Decompress

```bash
gunzip -f /tmp/apriltag-profile.json.gz
```

### 3. Analyze

Run the bundled analysis script at [analyze.py](analyze.py):

```bash
python3 .claude/skills/profile/analyze.py /tmp/apriltag-profile.json
```

This produces three views:
- **Self-time** (leaf functions) — where CPU time is actually spent
- **Inclusive time** (on-stack) — which functions are on the critical path
- **Top collapsed stacks** — full call chains for deep analysis

### 4. Drill into a specific function

```bash
python3 .claude/skills/profile/analyze.py --filter gradient_clusters /tmp/apriltag-profile.json
```

### 5. Compare before/after

Run the full workflow before and after your change, then compare self-time percentages for hot functions.

## Tips

- Increase iterations for more samples: `profile --iterations 5000`
- Look at self-time first to find where CPU time is actually spent
- Look at inclusive-time to understand the critical path
- Use `--filter` to drill into specific functions by name
