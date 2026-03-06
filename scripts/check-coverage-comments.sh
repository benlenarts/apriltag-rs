#!/usr/bin/env bash
# Check that every uncovered line has a nearby COVERAGE: comment explaining
# why it is acceptable. Intended for use in CI / `just check-coverage`.
#
# Usage: ./check-coverage-comments.sh [scope]
#   scope — optional path substring to filter files (e.g. "apriltag/src/")
#
# Exit 0 if all uncovered lines are documented, 1 otherwise.

set -euo pipefail

# Number of lines above an uncovered line to search for a COVERAGE: comment.
# A single comment can cover a block of consecutive uncovered lines.
LOOKBACK=10

# Optional scope filter: only check files whose path contains this string.
# Pass as first argument, e.g. ./check-coverage-comments.sh "apriltag/src/"
SCOPE="${1:-}"

# Run coverage and extract "Uncovered Lines:" section.
# Format: /path/to/file.rs: 10, 20, 30
raw=$(cargo llvm-cov \
  --ignore-filename-regex '(apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/(main\.rs|bin/|report\.rs))' \
  --show-missing-lines 2>&1 \
  | grep "^/" || true)

if [ -n "$SCOPE" ]; then
  uncovered=$(echo "$raw" | grep "$SCOPE" || true)
else
  uncovered="$raw"
fi

if [ -z "$uncovered" ]; then
  echo "No uncovered lines — nothing to check."
  exit 0
fi

fail=0

while IFS= read -r entry; do
  file="${entry%%:*}"
  lines_csv="${entry#*: }"

  IFS=', ' read -ra line_nums <<< "$lines_csv"

  for lineno in "${line_nums[@]}"; do
    [ -z "$lineno" ] && continue

    # Search the LOOKBACK lines above (inclusive) for a COVERAGE: comment
    start=$((lineno - LOOKBACK))
    [ "$start" -lt 1 ] && start=1

    if sed -n "${start},${lineno}p" "$file" | grep -q "COVERAGE:"; then
      continue
    fi

    # Also check: is this a closing brace or blank line? Those are
    # instrumentation artifacts — allow them without a comment.
    content=$(sed -n "${lineno}p" "$file" | sed 's/^[[:space:]]*//')
    if [ "$content" = "}" ] || [ -z "$content" ]; then
      continue
    fi

    echo "UNCOVERED without COVERAGE comment: ${file##*/}:$lineno: $content"
    fail=1
  done
done <<< "$uncovered"

if [ "$fail" -eq 0 ]; then
  echo "All uncovered lines are documented with COVERAGE: comments."
else
  echo ""
  echo "Add a '// COVERAGE: <reason>' comment within $LOOKBACK lines above each uncovered line."
  exit 1
fi
