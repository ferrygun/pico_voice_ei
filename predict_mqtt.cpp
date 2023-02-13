#include <stdio.h>
#include "pico/stdlib.h"
#include "ei_run_classifier.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"

#include "hardware/structs/rosc.h"

#include <string.h>
#include <time.h>

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"

#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"

#include "lwip/apps/mqtt_priv.h"

#include "tusb.h"
#include <string>

#include "analog_microphone.h"
#include "analog_microphone.c"
#include <ctime>
#include <iostream>


#define DEBUG_printf printf

#define MQTT_TLS 0 // needs to be 1 for AWS IoT
//#define CRYPTO_AWS_IOT
#define CRYPTO_MOSQUITTO_LOCAL
#include "/home/fd/picovoice/source/crypto_consts.h"


#if MQTT_TLS
#ifdef CRYPTO_CERT
const char * cert = CRYPTO_CERT;
#endif
#ifdef CRYPTO_CA
const char * ca = CRYPTO_CA;
#endif
#ifdef CRYPTO_KEY
const char * key = CRYPTO_KEY;
#endif
#endif

char ssid[] = "";
char pass[] = "";

typedef struct MQTT_CLIENT_T_ {
  ip_addr_t remote_addr;
  mqtt_client_t * mqtt_client;
  u8_t receiving;
  u32_t received;
  u32_t counter;
  u32_t reconnect;
}
MQTT_CLIENT_T;

err_t mqtt_test_connect(MQTT_CLIENT_T * state);

/* cribbed from https://github.com/peterharperuk/pico-examples/tree/add_mbedtls_example */
/* Function to feed mbedtls entropy. May be better to move it to pico-sdk */
int mbedtls_hardware_poll(void * data, unsigned char * output, size_t len, size_t * olen) {
  /* Code borrowed from pico_lwip_random_byte(), which is static, so we cannot call it directly */
  static uint8_t byte;

  for (int p = 0; p < len; p++) {
    for (int i = 0; i < 32; i++) {
      // picked a fairly arbitrary polynomial of 0x35u - this doesn't have to be crazily uniform.
      byte = ((byte << 1) | rosc_hw->randombit) ^ (byte & 0x80u ? 0x35u : 0);
      // delay a little because the random bit is a little slow
      busy_wait_at_least_cycles(30);
    }
    output[p] = byte;
  }

  *olen = len;
  return 0;
}

// Perform initialisation
static MQTT_CLIENT_T * mqtt_client_init() {
  MQTT_CLIENT_T * state = new MQTT_CLIENT_T();
  if (!state) {
    DEBUG_printf("failed to allocate state\n");
    return nullptr;
  }
  state -> receiving = 0;
  state -> received = 0;
  return state;
}

void dns_found(const char * name,
  const ip_addr_t * ipaddr, void * callback_arg) {
  MQTT_CLIENT_T * state = (MQTT_CLIENT_T * ) callback_arg;
  DEBUG_printf("DNS query finished with resolved addr of %s.\n", ip4addr_ntoa(ipaddr));
  state -> remote_addr = * ipaddr;
}

void run_dns_lookup(MQTT_CLIENT_T * state) {
  DEBUG_printf("Running DNS query for %s.\n", MQTT_SERVER_HOST);

  cyw43_arch_lwip_begin();
  err_t err = dns_gethostbyname(MQTT_SERVER_HOST, & (state -> remote_addr), dns_found, state);
  cyw43_arch_lwip_end();

  if (err == ERR_ARG) {
    DEBUG_printf("failed to start DNS query\n");
    return;
  }

  if (err == ERR_OK) {
    DEBUG_printf("no lookup needed\n");
    return;
  }

  while (state -> remote_addr.addr == 0) {
    cyw43_arch_poll();
    sleep_ms(1);
  }
}

static void mqtt_connection_cb(mqtt_client_t * client, void * arg, mqtt_connection_status_t status) {
  MQTT_CLIENT_T * state = (MQTT_CLIENT_T * ) arg;
  if (status != 0) {
    DEBUG_printf("Error during connection: err %d.\n", status);
  } else {
    DEBUG_printf("MQTT connected.\n");
  }
}

void mqtt_pub_request_cb(void * arg, err_t err) {
  MQTT_CLIENT_T * state = (MQTT_CLIENT_T * ) arg;
  DEBUG_printf("mqtt_pub_request_cb: err %d\n", err);
  state -> receiving = 0;
  state -> received++;
}

err_t mqtt_test_publish(MQTT_CLIENT_T * state,
  const char * message) {
  char buffer[128];

  #if MQTT_TLS
  #define TLS_STR "TLS"
  #else
  #define TLS_STR ""
  #endif

  sprintf(buffer, message, state -> received, state -> counter, TLS_STR);

  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0;
  cyw43_arch_lwip_begin();
  printf("MQTT PUBLISH\n");
  err = mqtt_publish(state -> mqtt_client, "picow", buffer, strlen(buffer), qos, retain, mqtt_pub_request_cb, state);
  cyw43_arch_lwip_end();
  if (err != ERR_OK) {
    DEBUG_printf("Publish err: %d\n", err);
  }

  return err;
}

void mqtt_test_conn_config_cb(void * conn) {
  #if MQTT_TLS
  mbedtls_ssl_set_hostname(altcp_tls_context(static_cast < struct altcp_pcb * > (conn)), MQTT_SERVER_HOST);
  #endif
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state) {
    DEBUG_printf("mqtt_test_connect inside\n");
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "pico";
    ci.client_user = "mqtt";
    ci.client_pass = "";
    ci.keep_alive = 60;
    ci.will_topic = "picow";
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    #if MQTT_TLS

    struct altcp_tls_config *tls_config;
  
    #if defined(CRYPTO_CA) && defined(CRYPTO_KEY) && defined(CRYPTO_CERT)
    DEBUG_printf("Setting up TLS with 2wayauth.\n");
    tls_config = altcp_tls_create_config_client_2wayauth(
        (const u8_t *)ca, 1 + strlen((const char *)ca),
        (const u8_t *)key, 1 + strlen((const char *)key),
        (const u8_t *)"", 0,
        
        (const u8_t *)cert, 1 + strlen((const char *)cert)
    );
    #elif defined(CRYPTO_CERT)
    DEBUG_printf("Setting up TLS with cert.\n");
    tls_config = altcp_tls_create_config_client((const u8_t *) cert, 1 + strlen((const char *) cert));
    #endif

    if (tls_config == NULL) {
        DEBUG_printf("Failed to initialize config\n");
        return -1;
    }

    ci.tls_config = tls_config;
    #endif

    //err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr), MQTT_SERVER_PORT, mqtt_connection_cb, state, &ci, mqtt_test_conn_config_cb);
    DEBUG_printf("mqtt_client_connect\n");
    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr), MQTT_SERVER_PORT, mqtt_connection_cb, state, &ci);

    if (err != ERR_OK) {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }
    DEBUG_printf("ERR_OK\n");
    return err;
}


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

float features[10800]; 

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

void mqtt_run_test(MQTT_CLIENT_T * state) {
  DEBUG_printf("mqtt_run_test\n");

  state -> mqtt_client = mqtt_client_new();

  const char * message = "";

  if (state -> mqtt_client == NULL) {
    DEBUG_printf("Failed to create new mqtt client\n");
    return;
  }

  if (mqtt_test_connect(state) == ERR_OK) {
    while (true) {
      //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

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
        //printf("sample_count returned: %d\n", sample_count);

        // loop through any new collected samples
        for (int i = 0; i < sample_count; i++) { //sample_count returned: 16
          features[i] = (float) sample_buffer[i];
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

        ei_printf("run_classifier returned: %d\n", res);

        //if (res != 0)
        //  return 1;

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

      //return 0;

    }
  }

}

void wait_for_usb() {
  while (!tud_cdc_connected()) {
    DEBUG_printf(".");
    sleep_ms(500);
  }
  DEBUG_printf("usb host detected\n");
}

int main(void) {
  // initialize stdio and wait for USB CDC connect
  stdio_init_all();

  wait_for_usb();

  if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
    DEBUG_printf("failed to initialise\n");
    return 1;
  }

  DEBUG_printf("initialised\n");

  cyw43_arch_enable_sta_mode();

  if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    DEBUG_printf("failed to  connect.\n");
    return 1;
  } else {
    DEBUG_printf("Connected.\n");
  }

  MQTT_CLIENT_T * state = mqtt_client_init();

  run_dns_lookup(state);

  mqtt_run_test(state);

  cyw43_arch_deinit();

  return 0;
}
