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

#define DEBUG_printf printf

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
//#define MQTT_PORT 1883


typedef struct MQTT_CLIENT_DATA_T {
    mqtt_client_t *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    uint8_t data[MQTT_OUTPUT_RINGBUF_SIZE];
    uint8_t topic[100];
    uint32_t len;
    bool playing;
    bool newTopic;
} MQTT_CLIENT_DATA_T;

MQTT_CLIENT_DATA_T *mqtt;

struct mqtt_connect_client_info_t mqtt_client_info =
{
  "ws2812",
  "mqtt", /* user */
  "xxx", /* pass */
  0,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

// configuration
#define INSIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE //2729

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

float features[INSIZE];

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

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}



err_t example_publish(mqtt_client_t *client, void *arg)
{
  const char *pub_payload= "PubSubHubLubJub";
  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  cyw43_arch_lwip_begin();
  err = mqtt_publish(client, "pub_topic", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  cyw43_arch_lwip_end();
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
  return err;
}


static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("mqtt_incoming_data_cb");
    MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
    LWIP_UNUSED_ARG(data);

    //strncpy(mqtt_client->data, data, len);
    strncpy((char*)mqtt_client->data, (char*)data, len);

    mqtt_client->len=len;
    mqtt_client->data[len]='\0';
    
    mqtt_client->newTopic=true;
    mqtt->playing=false;
 
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
  printf("mqtt_incoming_publish_cb");
  MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
  //strcpy(mqtt_client->topic, topic);
  strcpy((char*)mqtt_client->topic, (char*)topic);
}

static void mqtt_request_cb(void *arg, err_t err) {
  MQTT_CLIENT_DATA_T* mqtt_client = ( MQTT_CLIENT_DATA_T*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" request cb: err %d\n", mqtt_client->mqtt_client_info.client_id, (int)err));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
  LWIP_UNUSED_ARG(client);

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" connection cb: status %d\n", mqtt_client->mqtt_client_info.client_id, (int)status));

  if (status == MQTT_CONNECT_ACCEPTED) {
    printf("MQTT_CONNECT_ACCEPTED");


    //example_publish(client, arg);
    //mqtt_disconnect(client);

    
    mqtt_sub_unsub(client,
            "start", 0,
            mqtt_request_cb, arg,
            1);
    mqtt_sub_unsub(client,
            "stop", 0,
            mqtt_request_cb, arg,
            1);
    
    
  }
}

int main()
{
    stdio_init_all();
    int sm = 0;


    mqtt=(MQTT_CLIENT_DATA_T*)calloc(1, sizeof(MQTT_CLIENT_DATA_T));

    if (!mqtt) {
        printf("mqtt client instant ini error\n");
        return 0;
    }
    mqtt->playing=false;
    mqtt->newTopic=false;
    mqtt->mqtt_client_info = mqtt_client_info;

    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect\n");
        return 1;
    }

    ip_addr_t addr;
    if (!ip4addr_aton("192.168.XX.XX", &addr)) {
        printf("ip error\n");
        return 0;
    }

   
    mqtt->mqtt_client_inst = mqtt_client_new();
    mqtt_set_inpub_callback(mqtt->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, mqtt);

    err_t err = mqtt_client_connect(mqtt->mqtt_client_inst, &addr, MQTT_PORT, &mqtt_connection_cb, mqtt, &mqtt->mqtt_client_info);
    if (err != ERR_OK) {
      printf("connect error\n");
      return 0;
    }

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

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
    {
      ei_printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
                EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
      return 1;
    }

        printf("size of features/floats: %d\n", sizeof(features) / sizeof(float));
        printf("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE:%d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        printf("EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME:%d\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        printf("size of features returned: %d\n", sizeof(features));
        printf("total_length: %d\n", sizeof(features) / sizeof(features[0]));
        printf("\n");


    while(1) {

      // wait for new samples
        while (samples_read == 0) {
            tight_loop_contents();
        }


        ei_printf("\nStarting inferencing in 3 seconds...\n");
        sleep_ms(3000);

        // store and clear the samples read from the callback
        int sample_count = samples_read;
        samples_read = 0;
        

        // loop through any new collected samples        
        for (int i = 0; i < sample_count; i++) { 
            features[i] = sample_buffer[i]; 
        }

        // the features are stored into flash, and we don't want to load everything into RAM
        signal_t features_signal;
        features_signal.total_length = sizeof(features) / sizeof(features[0]);
        features_signal.get_data = &raw_feature_get_data;

         
        // invoke the impulse
        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

        ei_printf("run_classifier returned: %d\n", res);

        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

        // print the predictions
        ei_printf("[");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%s: %.5f", result.classification[ix].label, result.classification[ix].value);

           
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

        example_publish(mqtt->mqtt_client_inst, mqtt);
        sleep_ms(1000);
      /*
      if (mqtt->newTopic) { 
          mqtt->newTopic=false;
          //if (strcmp(mqtt->topic, "start")==0) {
          if (strcmp((const char*)mqtt->topic, "start") == 0) {
            printf("START");
          }
          //if (strcmp(mqtt->topic, "stop")==0) {
          if (strcmp((const char*)mqtt->topic, "stop") == 0) {
              printf("STOP");
          }
      }
      */
   }
   

   
    return 0;
}
