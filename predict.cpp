/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * 
 * This examples captures data from an analog microphone using a sample
 * rate of 8 kHz and prints the sample values over the USB serial
 * connection.
 */
#include <stdio.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "pico/stdlib.h"
#include "analog_microphone.h"
#include "analog_microphone.c"
#include "tusb.h"
#include <ctime>
#include <iostream>

char ssid[] = "";
char pass[] = "";

// configuration
#define INSIZE 1024
const struct analog_microphone_config config = {
    // GPIO to use for input, must be ADC compatible (GPIO 26 - 28)
    .gpio = 26,

    // bias voltage of microphone in volts
    .bias_voltage = 1.25,

    // sample rate in Hz
    .sample_rate = 16000,

    // number of samples to buffer
    .sample_buffer_size = INSIZE,
};

// variables
int16_t sample_buffer[INSIZE];
volatile int samples_read = 0;

float features[2700];

const int minutes = .3;
const int threshold = 10;
const float thresh = 0.9;

int count_label_on = 0;
std::time_t start_time = std::time(0);

void on_analog_samples_ready() {
    // callback from library when all the samples in the library
    // internal sample buffer are ready for reading 
    samples_read = analog_microphone_read(sample_buffer, INSIZE);
}


int raw_feature_get_data(size_t offset, size_t length, float * out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void wait_for_usb() {
    while (!tud_cdc_connected()) {
        printf(".");
        sleep_ms(500);
    }
    printf("usb host detected\n");
}   

int main(void) {
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();

    wait_for_usb();

    printf("connected\n");

    ei_impulse_result_t result = {
        nullptr
    };

    ei_printf("Edge Impulse standalone inferencing (Raspberry Pi Pico)\n");

    // initialize the analog microphone
    if (analog_microphone_init( & config) < 0) {
        ei_printf("analog microphone initialization failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    analog_microphone_set_samples_ready_handler(on_analog_samples_ready);

    // start capturing data from the analog microphone
    if (analog_microphone_start() < 0) {
        ei_printf("Analog microphone start failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    printf("sdssdsdsd");

    //if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
    //{
    //  ei_printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
    //            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
    //  return 1;
    //}
    
    while (1) {
        // wait for new samples
        while (samples_read == 0) {
            tight_loop_contents();
        }

        // store and clear the samples read from the callback
        int sample_count = samples_read;
        samples_read = 0;
        printf("sample_count returned: %d\n", sample_count);

        // loop through any new collected samples
        for (int i = 0; i < sample_count; i++) { //sample_count returned: 1024
            features[i] = (float)sample_buffer[i];
            //printf("%f \n", (float)sample_buffer[i]);
        }

        printf("size of features/floats: %d\n", sizeof(features) / sizeof(float));
        printf("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE: %.3f\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        printf("EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME: %.3f\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        printf("size of features returned: %d\n", sizeof(features));
        printf("total_length: %d\n", sizeof(features) / sizeof(features[0]));

        //sample_count returned: 16
        //size of features/floats: 2700
        //EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE: 0.000
        //EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME: 0.000
        //size of features returned: 10800
        //total_length: 2700
        //run_classifier returned: 0


        // the features are stored into flash, and we don't want to load everything into RAM
        signal_t features_signal;
        features_signal.total_length = sizeof(features) / sizeof(features[0]);
        features_signal.get_data = &raw_feature_get_data;


        // invoke the impulse
        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

        //ei_printf("run_classifier returned: %d\n", res);

        if (res != 0)
            return 1;

        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

        // print the predictions
        ei_printf("[");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%.5f\n", result.classification[ix].value);

            if (ix == 1 && result.classification[ix].value > thresh) {
              count_label_on++;
            } else {
              //count_label_on = 0;
            }

            if (result.classification[0].value > result.classification[1].value) {
              count_label_on = 0;
            }

            if (count_label_on >= threshold) {
              std::time_t current_time = std::time(0);
              if (current_time - start_time >= minutes * 60) {
                printf("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\n");
                count_label_on = 0;
                start_time = std::time(0);
              }
            } 

            #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf(", ");
            #else
            if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
                ei_printf(", ");
            }
            #endif
        }
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
        printf("%.3f", result.anomaly);
        #endif
        printf("]\n");
        
        

        //sleep_ms(2000);

    }

    return 0;
}
