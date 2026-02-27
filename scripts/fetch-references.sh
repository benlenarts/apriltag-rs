#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

PAPERS_DIR="$PROJECT_ROOT/docs/papers"
REF_IMPL_DIR="$PROJECT_ROOT/docs/reference-implementation"

# AprilTag papers
PAPERS=(
  "https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf"
  "https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf"
  "https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf"
  "https://cs.brown.edu/people/pfelzens/papers/seg-ijcv.pdf"
)

# Reference C implementation
REF_REPO="https://github.com/AprilRobotics/apriltag.git"

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

echo "==> Cloning reference implementation to $REF_IMPL_DIR"
if [[ -d "$REF_IMPL_DIR/.git" ]]; then
  echo "    Already cloned, pulling latest"
  git -C "$REF_IMPL_DIR" pull --ff-only
else
  rm -rf "$REF_IMPL_DIR"
  git clone --depth 1 "$REF_REPO" "$REF_IMPL_DIR"
fi

echo "==> Done"
