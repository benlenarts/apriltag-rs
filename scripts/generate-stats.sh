#!/usr/bin/env bash
# Generate badge JSON files from current project stats.
# Run by CI on push to main; badges are committed back to the repo.
#
# Usage: ./scripts/generate-stats.sh [BADGES_DIR]
#   BADGES_DIR — directory for badge JSON files (default: .github/badges)

set -euo pipefail

BADGES_DIR="${1:-.github/badges}"
mkdir -p "$BADGES_DIR"

# Helper: write a shields.io endpoint badge JSON
write_badge() {
  local file="$1" label="$2" message="$3" color="$4"
  cat > "$BADGES_DIR/$file" <<EOF
{"schemaVersion": 1, "label": "$label", "message": "$message", "color": "$color"}
EOF
}

# --- Tests ---
echo "Counting tests..."
test_output=$(cargo test --workspace --features parallel 2>&1)
test_count=$(echo "$test_output" | grep "^test result:" | awk '{sum += $4} END {print sum}')
failed=$(echo "$test_output" | grep "^test result:" | awk '{sum += $6} END {print sum}')

if [ "$failed" -eq 0 ]; then
  write_badge "tests.json" "tests" "$test_count passed" "brightgreen"
else
  write_badge "tests.json" "tests" "$failed failed" "red"
fi

# --- Coverage ---
echo "Computing coverage..."
cov_exclude="--ignore-filename-regex (apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/(main\.rs|report\.rs))"
cov_line=$(cargo llvm-cov $cov_exclude 2>&1 | grep "^TOTAL" || true)

if [ -n "$cov_line" ]; then
  # Extract line coverage percentage (column after lines/missed)
  cov_pct=$(echo "$cov_line" | awk '{for(i=1;i<=NF;i++) if($i ~ /%/) {print $i; exit}}')
  cov_num=$(echo "$cov_pct" | tr -d '%')

  if (( $(echo "$cov_num >= 99" | bc -l) )); then
    color="brightgreen"
  elif (( $(echo "$cov_num >= 90" | bc -l) )); then
    color="green"
  elif (( $(echo "$cov_num >= 80" | bc -l) )); then
    color="yellow"
  else
    color="red"
  fi
  write_badge "coverage.json" "coverage" "$cov_pct" "$color"
fi

# --- Regression scenarios ---
echo "Running regression gate..."
regression_output=$(just verify-func 2>&1 || true)
total_line=$(echo "$regression_output" | grep "^Total:" || true)

if [ -n "$total_line" ]; then
  total=$(echo "$total_line" | grep -oP 'Total: \K[0-9]+')
  passed=$(echo "$total_line" | grep -oP 'Passed: \K[0-9]+')
  failed_reg=$(echo "$total_line" | grep -oP 'Failed: \K[0-9]+')

  if [ "$failed_reg" -eq 0 ]; then
    write_badge "regression.json" "regression" "$passed/$total passed" "brightgreen"
  else
    write_badge "regression.json" "regression" "$failed_reg/$total failed" "red"
  fi
fi

# --- Unsafe code ---
echo "Checking unsafe code..."
unsafe_count=$(grep -r "unsafe " --include="*.rs" \
  --exclude-dir=target \
  --exclude="reference.rs" \
  --exclude="alloc_comparison.rs" \
  -l . 2>/dev/null | wc -l || echo "0")

if [ "$unsafe_count" -eq 0 ]; then
  write_badge "unsafe.json" "unsafe" "0 lines" "brightgreen"
else
  write_badge "unsafe.json" "unsafe" "$unsafe_count files" "orange"
fi

echo "Badge files written to $BADGES_DIR:"
ls -la "$BADGES_DIR"/*.json
