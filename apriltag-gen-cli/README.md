# apriltag-gen-cli

CLI for generating and rendering AprilTag families.

## Install

```bash
cargo install --path apriltag-gen-cli
```

## Commands

### List built-in families

```bash
apriltag-gen list
```

### Show family details

```bash
apriltag-gen info --family tag36h11
```

### Render tags

Render individual tags as PNG or PDF:

```bash
apriltag-gen render --family tag36h11 --ids 0-9 --scale 20 --output tags/
apriltag-gen render --family tag36h11 --ids 0,3,5 --format pdf --output tags/
```

### Render a mosaic

Render all tags in a family as a grid:

```bash
apriltag-gen mosaic --family tag36h11 --scale 10 --columns 10 --output mosaic.png
```

### Generate codes for a custom family

Define a family in a `.toml` config file, then generate its codes:

```bash
apriltag-gen generate --family my_family.toml
```

This produces a `.bin` file containing the generated tag codes.

### Verify built-in families

Check that regenerated codes match the built-in data:

```bash
apriltag-gen verify --family tag36h11
```

## Custom families

The `--family` argument accepts either a built-in family name (e.g. `tag36h11`) or a path to a `.toml` family config file. When using a `.toml` file, the CLI looks for a matching `.bin` file alongside it for pre-generated codes.
