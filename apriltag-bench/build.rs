fn main() {
    #[cfg(feature = "reference")]
    {
        build_reference();
    }
}

#[cfg(feature = "reference")]
fn build_reference() {
    let ref_dir = std::path::Path::new("../docs/reference-detection");

    if !ref_dir.exists() {
        panic!(
            "Reference C implementation not found at {:?}. \
             Run scripts/fetch-references.sh first.",
            ref_dir
        );
    }

    let source_files = [
        "apriltag.c",
        "apriltag_pose.c",
        "apriltag_quad_thresh.c",
        "tag36h11.c",
        "tag25h9.c",
        "tag16h5.c",
        "tagStandard41h12.c",
        "tagStandard52h13.c",
        "tagCircle21h7.c",
        "tagCircle49h12.c",
        "tagCustom48h12.c",
        "common/g2d.c",
        "common/getopt.c",
        "common/homography.c",
        "common/image_u8.c",
        "common/image_u8_parallel.c",
        "common/image_u8x3.c",
        "common/image_u8x4.c",
        "common/matd.c",
        "common/pam.c",
        "common/pjpeg.c",
        "common/pjpeg-idct.c",
        "common/pnm.c",
        "common/string_util.c",
        "common/svd22.c",
        "common/time_util.c",
        "common/unionfind.c",
        "common/workerpool.c",
        "common/zarray.c",
        "common/zhash.c",
        "common/zmaxheap.c",
    ];

    let mut build = cc::Build::new();
    build
        .include(ref_dir)
        .include(ref_dir.join("common"))
        .define("_GNU_SOURCE", None)
        .flag_if_supported("-std=gnu99")
        .flag_if_supported("-w"); // suppress warnings from C code

    for src in &source_files {
        let path = ref_dir.join(src);
        if path.exists() {
            build.file(&path);
        } else {
            eprintln!("Warning: reference source file not found: {:?}", path);
        }
    }

    // Add our bridge file
    build.file("src/reference_bridge.c");

    build.compile("apriltag_reference");

    println!("cargo:rustc-link-lib=static=apriltag_reference");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=m");
}
