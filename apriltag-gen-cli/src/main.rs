use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

mod render_pdf;
mod render_png;

/// AprilTag generation and rendering CLI
#[derive(Parser)]
#[command(name = "apriltag-gen", version)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List all built-in tag families
    List,
    /// Show detailed info about a tag family
    Info {
        /// Family name (built-in) or path to .toml config
        #[arg(long)]
        family: String,
    },
    /// Render individual tags as PNG or PDF
    Render {
        /// Family name (built-in) or path to .toml config
        #[arg(long)]
        family: String,
        /// Tag IDs to render (e.g. "0", "0-9", "0,3,5")
        #[arg(long, default_value = "0")]
        ids: String,
        /// Output format
        #[arg(long, default_value = "png")]
        format: String,
        /// Pixels per tag cell
        #[arg(long, default_value = "10")]
        scale: usize,
        /// White border width in cells around the tag
        #[arg(long, default_value = "1")]
        border: usize,
        /// Output directory
        #[arg(short, long, default_value = ".")]
        output: String,
    },
    /// Render a mosaic of all tags in a family
    Mosaic {
        /// Family name (built-in) or path to .toml config
        #[arg(long)]
        family: String,
        /// Output format
        #[arg(long, default_value = "png")]
        format: String,
        /// Pixels per tag cell
        #[arg(long, default_value = "10")]
        scale: usize,
        /// Spacing between tags in cells
        #[arg(long, default_value = "2")]
        spacing: usize,
        /// Number of columns in the grid
        #[arg(long, default_value = "10")]
        columns: usize,
        /// Output file path
        #[arg(short, long, default_value = "mosaic.png")]
        output: String,
    },
    /// Generate codes for a tag family config
    Generate {
        /// Family name (built-in) or path to .toml config
        #[arg(long)]
        family: String,
    },
    /// Verify that regenerated codes match the built-in .bin data
    Verify {
        /// Built-in family name
        #[arg(long)]
        family: String,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Command::List => cmd_list(),
        Command::Info { family } => cmd_info(&family),
        Command::Render {
            family,
            ids,
            format,
            scale,
            border,
            output,
        } => cmd_render(&family, &ids, &format, scale, border, &output),
        Command::Mosaic {
            family,
            format,
            scale,
            spacing,
            columns,
            output,
        } => cmd_mosaic(&family, &format, scale, spacing, columns, &output),
        Command::Generate { family } => cmd_generate(&family),
        Command::Verify { family } => cmd_verify(&family),
    }
}

/// Load a family by name (built-in) or path (.toml file).
fn load_family(name_or_path: &str) -> Result<apriltag_gen::family::TagFamily> {
    if let Some(family) = apriltag_gen::family::builtin_family(name_or_path) {
        return Ok(family);
    }

    // Try loading as a TOML file path
    let toml_path = std::path::Path::new(name_or_path);
    if toml_path.exists() {
        let toml_str = std::fs::read_to_string(toml_path)
            .with_context(|| format!("reading {}", toml_path.display()))?;
        let bin_path = toml_path.with_extension("bin");
        let bin_data = if bin_path.exists() {
            std::fs::read(&bin_path).with_context(|| format!("reading {}", bin_path.display()))?
        } else {
            Vec::new()
        };
        let family = apriltag_gen::family::TagFamily::from_toml_and_bin(&toml_str, &bin_data)
            .with_context(|| format!("parsing family from {}", toml_path.display()))?;
        return Ok(family);
    }

    anyhow::bail!(
        "unknown family '{}'. Use 'list' to see built-in families, or provide a .toml path.",
        name_or_path
    );
}

/// Parse an ID specification like "0", "0-9", "0,3,5", "0-4,7,10-12".
fn parse_ids(spec: &str, max_id: usize) -> Result<Vec<usize>> {
    let mut ids = Vec::new();
    for part in spec.split(',') {
        let part = part.trim();
        if let Some((start, end)) = part.split_once('-') {
            let start: usize = start.trim().parse().context("invalid ID range start")?;
            let end: usize = end.trim().parse().context("invalid ID range end")?;
            anyhow::ensure!(
                end < max_id,
                "ID {} exceeds max {} for this family",
                end,
                max_id - 1
            );
            ids.extend(start..=end);
        } else {
            let id: usize = part.parse().context("invalid ID")?;
            anyhow::ensure!(
                id < max_id,
                "ID {} exceeds max {} for this family",
                id,
                max_id - 1
            );
            ids.push(id);
        }
    }
    Ok(ids)
}

fn cmd_list() -> Result<()> {
    println!(
        "{:<22} {:>5} {:>7} {:>8}",
        "Family", "Bits", "Hamming", "Codes"
    );
    println!("{}", "-".repeat(46));
    for name in apriltag_gen::family::BUILTIN_NAMES {
        let family = apriltag_gen::family::builtin_family(name).unwrap();
        println!(
            "{:<22} {:>5} {:>7} {:>8}",
            family.config.name,
            family.layout.nbits,
            family.config.min_hamming,
            family.codes.len(),
        );
    }
    Ok(())
}

fn cmd_info(name: &str) -> Result<()> {
    let family = load_family(name)?;
    println!("Family:        {}", family.config.name);
    println!("Data bits:     {}", family.layout.nbits);
    println!("Min hamming:   {}", family.config.min_hamming);
    if let Some(mc) = family.config.min_complexity {
        println!("Min complex:   {}", mc);
    }
    println!(
        "Grid size:     {}x{}",
        family.layout.grid_size, family.layout.grid_size
    );
    println!("Border width:  {}", family.layout.border_width);
    println!("Reversed:      {}", family.layout.reversed_border);
    println!("Code count:    {}", family.codes.len());
    println!();

    // Show layout visualization
    println!("Layout:");
    let ds = family.layout.data_string();
    let size = family.layout.grid_size;
    for y in 0..size {
        print!("  ");
        for x in 0..size {
            let ch = ds.as_bytes()[y * size + x] as char;
            print!("{}", ch);
        }
        println!();
    }
    Ok(())
}

fn cmd_render(
    name: &str,
    id_spec: &str,
    format: &str,
    scale: usize,
    border: usize,
    output_dir: &str,
) -> Result<()> {
    let family = load_family(name)?;
    let ids = parse_ids(id_spec, family.codes.len())?;

    std::fs::create_dir_all(output_dir)
        .with_context(|| format!("creating output directory '{}'", output_dir))?;

    for &id in &ids {
        let tag = apriltag_gen::render::render(&family.layout, family.codes[id]);
        let filename = format!("{}_{:04}.{}", family.config.name, id, format);
        let path = std::path::Path::new(output_dir).join(&filename);

        match format {
            "png" => {
                render_png::write_tag_png(&tag, scale, border, &path)?;
                println!("wrote {}", path.display());
            }
            "pdf" => {
                render_pdf::write_tag_pdf(&tag, border, &path.to_string_lossy())?;
                println!("wrote {}", path.display());
            }
            _ => anyhow::bail!("unknown format '{}', use 'png' or 'pdf'", format),
        }
    }
    Ok(())
}

fn cmd_mosaic(
    name: &str,
    format: &str,
    scale: usize,
    spacing: usize,
    columns: usize,
    output_path: &str,
) -> Result<()> {
    let family = load_family(name)?;

    match format {
        "png" => {
            render_png::write_mosaic_png(&family, scale, spacing, columns, output_path)?;
            println!("wrote {}", output_path);
        }
        "pdf" => {
            render_pdf::write_mosaic_pdf(&family, spacing, columns, output_path)?;
            println!("wrote {}", output_path);
        }
        _ => anyhow::bail!("unknown format '{}', use 'png' or 'pdf'", format),
    }
    Ok(())
}

fn cmd_generate(name: &str) -> Result<()> {
    let family = load_family(name)?;

    let codes = if matches!(
        family.config.layout,
        apriltag_gen::family::LayoutConfig::Classic { .. }
    ) {
        generate_classic(&family)?
    } else {
        generate_era2(&family)?
    };

    println!("Generated {} codes.", codes.len());

    // Write .bin file
    let bin_path = format!("{}.bin", family.config.name);
    let mut bin_data = Vec::with_capacity(codes.len() * 8);
    for &code in &codes {
        bin_data.extend_from_slice(&code.to_le_bytes());
    }
    std::fs::write(&bin_path, &bin_data).with_context(|| format!("writing {}", bin_path))?;
    println!("Wrote {} codes to {}", codes.len(), bin_path);

    Ok(())
}

/// Generate codes for a classic family by upgrading old row-major codes.
fn generate_classic(family: &apriltag_gen::family::TagFamily) -> Result<Vec<u64>> {
    let old_codes =
        apriltag_gen::upgrade::classic_old_codes(&family.config.name).with_context(|| {
            format!(
                "classic family '{}' has no known old codes — classic families cannot be \
             regenerated algorithmically, they require the original row-major codes \
             from the apriltag-generation Java source",
                family.config.name
            )
        })?;

    let data_size = (family.layout.nbits as f64).sqrt() as usize;

    println!(
        "Upgrading {} old codes for {} (nbits={}, data_size={})...",
        old_codes.len(),
        family.config.name,
        family.layout.nbits,
        data_size,
    );

    Ok(apriltag_gen::upgrade::upgrade_codes(
        old_codes,
        &family.bit_locations,
        data_size,
    ))
}

fn cmd_verify(name: &str) -> Result<()> {
    let family = apriltag_gen::family::builtin_family(name).with_context(|| {
        format!(
            "'{}' is not a built-in family — verify only works with built-in families",
            name
        )
    })?;

    let codes = if matches!(
        family.config.layout,
        apriltag_gen::family::LayoutConfig::Classic { .. }
    ) {
        generate_classic(&family)?
    } else {
        generate_era2(&family)?
    };

    if codes == family.codes {
        println!(
            "PASS: {} — regenerated {} codes match built-in data",
            family.config.name,
            codes.len()
        );
        Ok(())
    } else {
        // Find first mismatch for diagnostics
        let mut msg = format!(
            "FAIL: {} — regenerated codes differ from built-in data\n  expected {} codes, got {}",
            family.config.name,
            family.codes.len(),
            codes.len()
        );
        for (i, (expected, got)) in family.codes.iter().zip(codes.iter()).enumerate() {
            if expected != got {
                msg.push_str(&format!(
                    "\n  first mismatch at index {}: expected {:#x}, got {:#x}",
                    i, expected, got
                ));
                break;
            }
        }
        anyhow::bail!(msg);
    }
}

/// Generate codes for an Era 2 family using the lexicode algorithm.
fn generate_era2(family: &apriltag_gen::family::TagFamily) -> Result<Vec<u64>> {
    let min_complexity = family
        .config
        .min_complexity
        .context("min_complexity is required in the family config for code generation")?;

    println!(
        "Generating codes for {} (nbits={}, min_hamming={}, min_complexity={})...",
        family.config.name, family.layout.nbits, family.config.min_hamming, min_complexity
    );

    let codes = apriltag_gen::codegen::generate_with_progress(
        &family.layout,
        family.config.min_hamming,
        min_complexity,
        {
            let mut last_print = std::time::Instant::now();
            let mut decimals = None;
            move |iter, total, codes_found| {
                let d = *decimals
                    .get_or_insert_with(|| ((total as f64).log10() - 8.0).ceil().max(1.0) as usize);
                let now = std::time::Instant::now();
                if iter == 0 || now.duration_since(last_print).as_millis() >= 100 {
                    let pct = iter as f64 / total as f64 * 100.0;
                    eprint!(
                        "\r  {:>width$.prec$}% searched, {} codes found",
                        pct,
                        codes_found,
                        width = d + 4,
                        prec = d
                    );
                    last_print = now;
                }
            }
        },
    );
    eprintln!();

    Ok(codes)
}
