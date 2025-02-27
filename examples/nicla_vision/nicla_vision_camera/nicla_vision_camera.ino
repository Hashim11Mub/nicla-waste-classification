
/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <Arduino.h>
#include <_lk_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "camera.h"
#include "gc2145.h"
#include <ea_malloc.h>

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_RAW_FRAME_BYTE_SIZE             2

#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

/* Function Declarations -------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize);
bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
static bool is_ll_initialised = false;

GC2145 galaxyCore;
Camera cam(galaxyCore);
FrameBuffer fb;

static uint8_t *ei_camera_capture_out = NULL;
static uint8_t *ei_camera_frame_mem;
static uint8_t *ei_camera_frame_buffer;

/*  Setup Function - Initialize Camera & LEDs */
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    malloc_addblock((void*)0x30000000, 288 * 1024);

    if (!ei_camera_init()) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    // ** Initialize Nicla Vision LEDs**
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    // ** LED TEST at startup**
    Serial.println("Testing LEDs...");
    digitalWrite(LEDR, LOW); delay(500); digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW); delay(500); digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW); delay(500); digitalWrite(LEDB, HIGH);
    Serial.println("LED Test Complete.");
}

/*  Loop Function - Perform Inference */
void loop() {
    ei_printf("\nStarting inferencing in 2 seconds...\n");
    if (ei_sleep(5000) != EI_IMPULSE_OK) {
        return;
    }

    ei_printf("Taking photo...\n");

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Free camera memory before capturing
    ei_camera_deinit();
    if (!ei_camera_init()) {
        ei_printf("ERROR: Failed to reinitialize camera!\n");
        return;
    }

    if (!ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, NULL)) {
        ei_printf("ERROR: Camera capture failed! Resetting...\n");
        return;
    }

    ei_printf("Image captured, running inference...\n");

    // Run the classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERROR: Failed to run classifier (%d)\n", err);
        return;
    }

    // Print prediction probabilities
    float prob_R = 0.0, prob_N = 0.0, prob_A = 0.0;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: %.5f\r\n", ei_classifier_inferencing_categories[i], result.classification[i].value);

        if (strcmp(ei_classifier_inferencing_categories[i], "R") == 0) {
            prob_R = result.classification[i].value;
        }
        if (strcmp(ei_classifier_inferencing_categories[i], "N") == 0) {
            prob_N = result.classification[i].value;
        }
        if (strcmp(ei_classifier_inferencing_categories[i], "A") == 0) {
            prob_A = result.classification[i].value;
        }
    }

    // **Debugging: Print probability values**
    ei_printf("DEBUG: R=%.5f, N=%.5f, A=%.5f\n", prob_R, prob_N, prob_A);

    //  **LED Logic (Active LOW Control)**
    digitalWrite(LEDR, HIGH); // Red OFF
    digitalWrite(LEDG, HIGH); // Green OFF
    digitalWrite(LEDB, HIGH); // Blue OFF

    if (prob_R > prob_N && prob_R > prob_A) {
        ei_printf("✅ Recyclable detected: GREEN LED ON\n");
        digitalWrite(LEDG, LOW);  // Green ON
    } 
    else if (prob_N > prob_R && prob_N > prob_A) {
        ei_printf("🚨 Non-recyclable detected: RED LED ON\n");
        digitalWrite(LEDR, LOW);  // Red ON
    } 
    else if (prob_A > prob_R && prob_A > prob_N) {
        ei_printf("⚠️ Anomaly detected: YELLOW LED ON\n");
        digitalWrite(LEDR, LOW);  // Red ON
        digitalWrite(LEDG, LOW);  // Green ON
    }

    ei_sleep(1000); // Keep LED on for a second

    // **Turn off all LEDs before next iteration**
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    ei_printf("\nLoop completed. Ready for next inference...\n");

    delay(500);  // Small delay to stabilize the loop
}

/*  Camera Initialization Function */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    if (!is_ll_initialised) {
        if (!cam.begin(CAMERA_R320x240, CAMERA_RGB565, -1)) {
            ei_printf("ERR: Failed to initialize camera\r\n");
            return false;
        }

        ei_camera_frame_mem = (uint8_t *)ei_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_RAW_FRAME_BYTE_SIZE + 32);
        if (!ei_camera_frame_mem) {
            ei_printf("Failed to allocate camera memory\r\n");
            return false;
        }
        ei_camera_frame_buffer = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);

        fb.setBuffer(ei_camera_frame_buffer);
        is_initialised = true;
    }
    return true;
}

/*  Camera Deinitialization Function */
void ei_camera_deinit(void) {
    ei_free(ei_camera_frame_mem);
    ei_camera_frame_mem = NULL;
    ei_camera_frame_buffer = NULL;
    is_initialised = false;
}

/*  Camera Data Retrieval Function */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    if (!ei_camera_capture_out) {
        ei_printf("ERR: No captured image available.\n");
        return -1;
    }

    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (ei_camera_capture_out[pixel_ix] << 16) +
                              (ei_camera_capture_out[pixel_ix + 1] << 8) +
                              ei_camera_capture_out[pixel_ix + 2];

        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

/*  Camera Capture Function */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;
    bool do_crop = false;

    ei_camera_capture_out = (uint8_t*)ea_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32);
    ei_camera_capture_out = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_capture_out, 32);

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    int snapshot_response = cam.grabFrame(fb, 100);
    if (snapshot_response != 0) {
        ei_printf("ERR: Failed to get snapshot (%d)\r\n", snapshot_response);
        return false;
    }

    bool converted = RBG565ToRGB888(ei_camera_frame_buffer, ei_camera_capture_out, cam.frameSize());

    if (!converted) {
        ei_printf("ERR: Conversion failed\n");
        ei_free(ei_camera_frame_mem);
        return false;
    }

    uint32_t resize_col_sz;
    uint32_t resize_row_sz;
    int res = calculate_resize_dimensions(img_width, img_height, &resize_col_sz, &resize_row_sz, &do_resize);
    if (res) {
        ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
        return false;
    }

    if ((img_width != resize_col_sz) || (img_height != resize_row_sz)) {
        do_crop = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            ei_camera_capture_out,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            ei_camera_capture_out,
            resize_col_sz,
            resize_row_sz);
    }

    ea_free(ei_camera_capture_out);
    return true;
}

/*  RGB565 to RGB888 Conversion Function */
bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len) {
    uint8_t hb, lb;
    uint32_t pix_count = src_len / 2;

    for (uint32_t i = 0; i < pix_count; i++) {
        hb = *src_buf++;
        lb = *src_buf++;

        *dst_buf++ = hb & 0xF8;                          // Red
        *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3; // Green
        *dst_buf++ = (lb & 0x1F) << 3;                   // Blue
    }
    return true;
}
/*  Define ei_device_resize_resolutions_t Struct */
typedef struct {
    size_t width;
    size_t height;
} ei_device_resize_resolutions_t;
/*  Function to Determine Resize Dimensions */
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize) {
    size_t list_size = 5;

    // **Use the Struct to Store Resolutions**
    const ei_device_resize_resolutions_t list[] = {
        {64, 64},
        {96, 96},
        {160, 120},
        {160, 160},
        {320, 240},
    };

    // **Default Values**
    *resize_col_sz = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    *resize_row_sz = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    *do_resize = false;

    for (size_t ix = 0; ix < list_size; ix++) {
        if ((out_width <= list[ix].width) && (out_height <= list[ix].height)) {
            *resize_col_sz = list[ix].width;
            *resize_row_sz = list[ix].height;
            *do_resize = true;
            break;
        }
    }
    return 0;
}