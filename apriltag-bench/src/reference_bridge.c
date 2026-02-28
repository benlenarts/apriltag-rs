/**
 * Thin C bridge between Rust FFI and the reference apriltag3 library.
 *
 * Provides a simple flat interface that Rust can call without dealing with
 * the full C API's complex struct layouts.
 */
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"

#include <stdlib.h>
#include <string.h>

/**
 * Result struct for a single detection, laid out for easy FFI.
 */
typedef struct {
    int id;
    int hamming;
    float decision_margin;
    /* corners: [TL, TR, BR, BL] each as (x, y) */
    double corners[8];
    double center[2];
} bench_detection_t;

/**
 * Detect tags in a grayscale image using the reference C implementation.
 *
 * Parameters:
 *   buf       - pixel data (row-major, grayscale)
 *   width     - image width
 *   height    - image height
 *   stride    - row stride in bytes
 *   family    - tag family name (e.g. "tag36h11")
 *   quad_decimate - quad decimation factor (e.g. 2.0)
 *   nthreads  - number of threads (1 for single-threaded)
 *   out_count - (output) number of detections
 *
 * Returns: heap-allocated array of bench_detection_t (caller must free with bench_free_detections)
 */
bench_detection_t* bench_reference_detect(
    const uint8_t* buf,
    int width,
    int height,
    int stride,
    const char* family,
    float quad_decimate,
    int nthreads,
    int* out_count
) {
    /* Create tag family */
    apriltag_family_t* tf = NULL;
    if (strcmp(family, "tag36h11") == 0) {
        tf = tag36h11_create();
    } else if (strcmp(family, "tag25h9") == 0) {
        tf = tag25h9_create();
    } else if (strcmp(family, "tag16h5") == 0) {
        tf = tag16h5_create();
    } else if (strcmp(family, "tagStandard41h12") == 0) {
        tf = tagStandard41h12_create();
    } else if (strcmp(family, "tagStandard52h13") == 0) {
        tf = tagStandard52h13_create();
    } else if (strcmp(family, "tagCircle21h7") == 0) {
        tf = tagCircle21h7_create();
    } else if (strcmp(family, "tagCircle49h12") == 0) {
        tf = tagCircle49h12_create();
    } else if (strcmp(family, "tagCustom48h12") == 0) {
        tf = tagCustom48h12_create();
    } else {
        *out_count = 0;
        return NULL;
    }

    /* Create detector */
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = quad_decimate;
    td->nthreads = nthreads;

    /* Create image (non-owning copy of the buffer) */
    image_u8_t im = {
        .width = width,
        .height = height,
        .stride = stride,
        .buf = (uint8_t*)buf  /* apriltag_detect doesn't modify the image */
    };

    /* Detect */
    zarray_t* detections = apriltag_detector_detect(td, &im);
    int n = zarray_size(detections);
    *out_count = n;

    bench_detection_t* results = NULL;
    if (n > 0) {
        results = (bench_detection_t*)calloc(n, sizeof(bench_detection_t));
        for (int i = 0; i < n; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            results[i].id = det->id;
            results[i].hamming = det->hamming;
            results[i].decision_margin = det->decision_margin;
            results[i].center[0] = det->c[0];
            results[i].center[1] = det->c[1];
            /* Corners: det->p[0..3] each is [x, y] */
            for (int j = 0; j < 4; j++) {
                results[i].corners[j * 2] = det->p[j][0];
                results[i].corners[j * 2 + 1] = det->p[j][1];
            }
        }
    }

    /* Cleanup */
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);

    /* Destroy family */
    if (strcmp(family, "tag36h11") == 0) {
        tag36h11_destroy(tf);
    } else if (strcmp(family, "tag25h9") == 0) {
        tag25h9_destroy(tf);
    } else if (strcmp(family, "tag16h5") == 0) {
        tag16h5_destroy(tf);
    } else if (strcmp(family, "tagStandard41h12") == 0) {
        tagStandard41h12_destroy(tf);
    } else if (strcmp(family, "tagStandard52h13") == 0) {
        tagStandard52h13_destroy(tf);
    } else if (strcmp(family, "tagCircle21h7") == 0) {
        tagCircle21h7_destroy(tf);
    } else if (strcmp(family, "tagCircle49h12") == 0) {
        tagCircle49h12_destroy(tf);
    } else if (strcmp(family, "tagCustom48h12") == 0) {
        tagCustom48h12_destroy(tf);
    }

    return results;
}

/**
 * Free the detection array returned by bench_reference_detect.
 */
void bench_free_detections(bench_detection_t* detections) {
    free(detections);
}
