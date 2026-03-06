# Release Process

## Versioning Policy

- This project follows [Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html).
- **Pre-1.0:** minor version bumps for breaking changes, patch bumps for fixes and new features.
- All workspace crates share the same version number and are released together.

## Tag Format

Tags use the format `vMAJOR.MINOR.PATCH` (e.g., `v0.2.0`).

## Steps to Release

### 1. Update the changelog

Move items from `[Unreleased]` in `CHANGELOG.md` to a new version heading:

```markdown
## [X.Y.Z] - YYYY-MM-DD
```

### 2. Bump crate versions

Update the `version` field in every workspace member's `Cargo.toml`:

- `apriltag/Cargo.toml`
- `apriltag-gen/Cargo.toml`
- `apriltag-gen-cli/Cargo.toml`
- `apriltag-detect-cli/Cargo.toml`
- `apriltag-wasm/Cargo.toml`
- `apriltag-bench/Cargo.toml`
- `apriltag-bench-wasm/Cargo.toml`

Also update any intra-workspace dependency version requirements if they use exact versions.

### 3. Run the full CI suite locally

```bash
just ci
```

### 4. Commit the version bump

```bash
git add -A
git commit -m "Release vX.Y.Z"
```

### 5. Tag the release

```bash
git tag -a vX.Y.Z -m "Release vX.Y.Z"
```

### 6. Push

```bash
git push origin main --follow-tags
```

### 7. Create a GitHub Release

The `release.yml` workflow automatically creates a draft GitHub Release when a `v*` tag is pushed. Review the draft, paste the relevant changelog section into the release notes, and publish.

### 8. (Future) Publish to crates.io

When the project is ready for crate publication, publish in dependency order:

1. `apriltag`
2. `apriltag-gen`
3. `apriltag-gen-cli`
4. `apriltag-detect-cli`
5. `apriltag-wasm`
6. `apriltag-bench`
7. `apriltag-bench-wasm`

```bash
cargo publish -p apriltag
cargo publish -p apriltag-gen
# ... etc.
```
