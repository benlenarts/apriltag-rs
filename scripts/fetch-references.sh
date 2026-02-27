#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

PAPERS_DIR="$PROJECT_ROOT/docs/papers"
REF_DETECT_DIR="$PROJECT_ROOT/docs/reference-detection"
REF_GENERATE_DIR="$PROJECT_ROOT/docs/reference-generation"

# AprilTag papers
PAPERS=(
  "https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf"
  "https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf"
  "https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf"
  "https://cs.brown.edu/people/pfelzens/papers/seg-ijcv.pdf"
)

# Reference implementations
REF_DETECT_REPO="https://github.com/AprilRobotics/apriltag.git"
REF_GENERATE_REPO="https://github.com/AprilRobotics/apriltag-generation.git"

echo "==> Downloading papers to $PAPERS_DIR"
mkdir -p "$PAPERS_DIR"
for url in "${PAPERS[@]}"; do
  filename="$(basename "$url")"
  if [[ -f "$PAPERS_DIR/$filename" ]]; then
    echo "    $filename (already exists, skipping)"
  else
    echo "    $filename"
    curl -sL -o "$PAPERS_DIR/$filename" "$url"
  fi
done

echo "==> Cloning reference detection to $REF_DETECT_DIR"
if [[ -d "$REF_DETECT_DIR/.git" ]]; then
  echo "    Already cloned, pulling latest"
  git -C "$REF_DETECT_DIR" pull --ff-only
else
  rm -rf "$REF_DETECT_DIR"
  git clone --depth 1 "$REF_DETECT_REPO" "$REF_DETECT_DIR"
fi

echo "==> Cloning reference generation to $REF_GENERATE_DIR"
if [[ -d "$REF_GENERATE_DIR/.git" ]]; then
  echo "    Already cloned, pulling latest"
  git -C "$REF_GENERATE_DIR" pull --ff-only
else
  rm -rf "$REF_GENERATE_DIR"
  git clone --depth 1 "$REF_GENERATE_REPO" "$REF_GENERATE_DIR"
fi

echo "==> Done"
