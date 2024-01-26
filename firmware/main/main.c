#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "audiobit.h"

#include "chirp_lr_17000_20000_20500_23500_48000_16.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          GPIO_NUM_21
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_1_BIT
#define LEDC_DUTY               (1)
#define LEDC_FREQUENCY          (8050005)

#define SEND_SAMPLES 1152

#define SAMPLE_RATE     (48000)
#define I2S_NUM_MIC         (0)
#define I2S_BCK_IO      (GPIO_NUM_13)
#define I2S_WS_IO       (GPIO_NUM_14)
#define I2S_DI_IO       (GPIO_NUM_15)
static int32_t i2s_readraw_buff[(SEND_SAMPLES*2+1)];
static int32_t tcp_readraw_buff[(SEND_SAMPLES*2+1)];
static int16_t i2s_readraw_buff_16bit[(SEND_SAMPLES*2+2)];
size_t bytes_read;

#define WIFI_SSID       "<SSID>"
#define WIFI_PASS       "<PASSWORD>"
#define UDP_SERVER_IP  "192.168.137.1"
#define UDP_SERVER_PORT 12345
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static int s_retry_num = 0;
#define DEST_BDADDR "01:23:45:67:89:AB"

StreamBufferHandle_t xStreamBuffer = NULL;
TaskHandle_t xTaskToNotify = NULL;

static bool index_receive = true;
static bool index_send = true;

void init_speaker_lr(void){
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_16 | 1ULL << GPIO_NUM_17);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 0);

}
void init_microphone(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = 0,
    };

    // Set the pinout configuration (set using menuconfig)
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DI_IO,
    };

    // Call driver installation function before any I2S R/W operation.
    i2s_driver_install(I2S_NUM_MIC, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_MIC, &pin_config);
    // ESP_ERROR_CHECK( i2s_set_clk(I2S_NUM_MIC, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO) );
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));


    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("wifi_init_sta finished.\n");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        printf("connected to ap");
    } else if (bits & WIFI_FAIL_BIT) {
        printf("Failed to connect");
    } else {
        printf("UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void play_read_task(void *pvParameter){

    const char *ptr = (const char *)audiobit_music;
    unsigned int offset = 0;
    int32_t tcp_idx = 0;

    signed short samples[SEND_SAMPLES*2], i;
    size_t written;

    unsigned int buff_size = sizeof(samples);

    while (offset<NUM_ELEMENTS*2){
        memcpy (samples, ptr+offset, buff_size);

        // write to speaker
        i2s_write (I2S_NUM, (const char*) samples, buff_size, &written, portMAX_DELAY);
        
        offset = (offset + buff_size) % (NUM_ELEMENTS*2);

        // read from microphone
        i2s_read(I2S_NUM_MIC, (char *)i2s_readraw_buff, SEND_SAMPLES*4*2, &bytes_read, portMAX_DELAY);

        for (int i = 0; i < SEND_SAMPLES*2; i++) {
            i2s_readraw_buff_16bit[i] = i2s_readraw_buff[i] >> 16;
        }

        // add index
        i2s_readraw_buff_16bit[SEND_SAMPLES*2] = tcp_idx & 0xffff;
        i2s_readraw_buff_16bit[SEND_SAMPLES*2+1] = tcp_idx >> 16;
        tcp_idx++;

        xStreamBufferSend(xStreamBuffer, i2s_readraw_buff_16bit, SEND_SAMPLES*2*2+4, 2); 
    }
}

int send_subtask(int sock){
    int64_t start_time = esp_timer_get_time();
    xStreamBufferReceive(xStreamBuffer,(char *)tcp_readraw_buff,SEND_SAMPLES*2*2+4, portMAX_DELAY);
    int len = send(sock, tcp_readraw_buff, SEND_SAMPLES*2*2+4, 0);
    int64_t end_time = esp_timer_get_time();
    return len;
}

void send_tcp_task(void *pvParameter){

    ESP_LOGI(TAG, "Tcp_task");

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
    }

    while(connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to connect");
        // vTaskDelete(NULL);
        vTaskDelay(100);
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    }

    int len;
    while (1){
       len = send_subtask(sock);
       if (len == -1) {    // If the connection is lost, attempt to reconnect.
            while(connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
                ESP_LOGE(TAG, "Failed to connect");
                // vTaskDelete(NULL);
                vTaskDelay(100);
                sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            }
       }
    }
}

void app_main(void)
{
    esp_rom_delay_us(300000);

    // Initialize speaker output
    audiobit_i2s_init ();
    init_speaker_lr();

    esp_rom_delay_us(1000);

    // Initialize SGTL5000 codec
    audiobit_i2c_init ();
    if (audiobit_playback_init () == ESP_OK)
        printf("I2S and I2C setup is completed for playback!\n");
    else
        printf("Seems like AudioBit is not connected properly!\n");
    i2c_driver_delete(AUDIOBIT_I2C_NUM);

    // Initialize microphone input
    init_microphone();

    // Initialize WiFi
    wifi_init_sta();

    xStreamBuffer = xStreamBufferCreate((SEND_SAMPLES*2*2+4)*40, 0);  // Buffer size = 40 packets

    // Launch tasks
    xTaskCreate(send_tcp_task, "send_tcp_task", 8192, NULL, 1, NULL);
    xTaskCreate(play_read_task, "play_read_task", 8192, NULL, 1, NULL);
    

}
