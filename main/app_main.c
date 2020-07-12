//
/*
MIT License

Copyright (c) 2019 Olof Astrand (Ebiroll)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
//#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <sys/time.h>
#include "read_chars.h"
#include "esp32_sump.h" 
#include "scpi_server.h"
#include "analog.h"
#include <driver/rmt.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "app-config.h"
#include "Header.h"
#include "ota_server.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"




#define STEP_PIN  GPIO_NUM_21

static const char *TAG = "sigrok";

TaskHandle_t xHandlingTask;

EventGroupHandle_t ota_event_group;

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD


TaskHandle_t xTaskList[20];
uint8_t xtaskListCounter = 0;

xTaskHandle TaskHandle_tmp;

 void KillAllThreads(void)
 {
    uint8_t list;

    printf("\nKilling A Total of %u Threads\r\n", xtaskListCounter);

    for(list=0; list < xtaskListCounter; list++)
    {
      // Use the handle to delete the task.
      if( xTaskList[list] != NULL )
      {
          printf("Killed Task[%u] Complete\r\n", list);
          vTaskDelete( xTaskList[list] );
      }
      else
      {
          printf("Could not Kill Task[%u] \r\n", list);
      }
    }
 }



#define BUF_SIZE 512

char echoLine[BUF_SIZE];

#if UART_TEST_OUTPUT
static void init_uart_1()
{
  uart_port_t uart_num = UART_NUM_1;                                     //uart port number


  uart_config_t uart_config = {
      .baud_rate = 9600,                    //baudrate was 9600 (2400=)
      .data_bits = UART_DATA_8_BITS,          //data bit mode
      .parity = UART_PARITY_DISABLE,          //parity mode
      .stop_bits = UART_STOP_BITS_1,          //stop bit mode
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //hardware flow control(cts/rts)
      .rx_flow_ctrl_thresh = 122,             //flow control threshold
  };
  ESP_LOGI(TAG, "Setting UART configuration number %d...", uart_num);
  ESP_ERROR_CHECK( uart_param_config(uart_num, &uart_config));
  QueueHandle_t uart_queue;
  // Use default pins for the uart
  ESP_ERROR_CHECK( uart_set_pin(uart_num, UART_OUTPUT_PIN, UART_RX_PIN , -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 2, 512 * 2, 10,  &uart_queue,0));

}

static void uartWRITETask(void *inpar) {
  //printf("sump UUU");  
  uart_port_t uart_num = UART_NUM_1;    
  echoLine[0]='s';
  echoLine[1]='u';
  echoLine[2]='m';
  echoLine[3]='p';
  echoLine[4]=' ';
  echoLine[5]='U';
  echoLine[6]='U';
  echoLine[7]='U';

  while(true) {
    (void) uart_write_bytes(uart_num, (const char *)echoLine, 8);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
#endif

#if defined(SUMP_OVER_UART)
static void init_uart()
{
  uart_port_t uart_num = UART_NUM_0;                                     //uart port number


  uart_config_t uart_config = {
      .baud_rate = 115200,                    //baudrate
      .data_bits = UART_DATA_8_BITS,          //data bit mode
      .parity = UART_PARITY_DISABLE,          //parity mode
      .stop_bits = UART_STOP_BITS_1,          //stop bit mode
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //hardware flow control(cts/rts)
      .rx_flow_ctrl_thresh = 122,             //flow control threshold
  };
  ESP_LOGI(TAG, "Setting UART configuration number %d...", uart_num);
  ESP_ERROR_CHECK( uart_param_config(uart_num, &uart_config));
  QueueHandle_t uart_queue;
  // Use default pins for the uart
  //ESP_ERROR_CHECK( uart_set_pin(uart_num, 1, 3, -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 4, 512 * 4, 20,  &uart_queue,0));

}

#endif


/* Function to initialize SPIFFS */
static esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/www",
      .partition_label = NULL,
      .max_files = 5,   // This decides the maximum number of files that can be created on the storage
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

/* Declare the function which starts the file server.
 * Implementation of this function is to be found in
 * file_server.c */
esp_err_t start_file_server(const char *base_path);


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(ota_event_group, OTA_CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        xEventGroupClearBits(ota_event_group, OTA_CONNECTED_BIT);
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
         esp_wifi_connect();
    }
}

// Similar to uint32_t system_get_time(void)
uint32_t get_usec() {
 struct timeval tv;
 gettimeofday(&tv,NULL);
 return (tv.tv_sec*1000000 + tv.tv_usec);
 //uint64_t tmp=get_time_since_boot();
 //uint32_t ret=(uint32_t)tmp;
 //return ret;
}

rmt_config_t config;
rmt_item32_t items[1];
 

void send_remote_pulses() {

  config.rmt_mode = RMT_MODE_TX;
  config.channel = RMT_CHANNEL_0;
  config.gpio_num = PULSE_PIN; 
  config.mem_block_num = 1;
  config.tx_config.loop_en = 1;
  config.tx_config.carrier_en = 0;
  config.tx_config.idle_output_en = 1;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  config.clk_div = 80; // 80MHz / 80 = 1MHz 0r 1uS per count
 
  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
   
  items[0].duration0 = 1500;  // 0.5 ms (500)
  items[0].level0 = 1;
  items[0].duration1 = 3000;   // 0.25 ms  (250)
  items[0].level1 = 0;  

}


static void remoteTask(void *inpar) {
    rmt_write_items(config.channel, items, 1, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

void sump_server_init(void);


// Only wait for sample task to finnish
void test_sample_task(void *param) {
    unsigned int ulNotifiedValue;

    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
    BaseType_t xResult=0;

      /* Wait to be notified when sampling is complete. */

     while (xResult != pdPASS ) {

        xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
                            ULONG_MAX,        /* Clear all bits on exit. */
                            &ulNotifiedValue, /* Stores the notified value. */
                            xMaxBlockTime );

        if( xResult == pdPASS ) {
            // Task finished
            printf("Sample task finished\n");
        }
     }

    vTaskDelete(NULL);
}

void example_i2s_adc_dac(void*arg);




#if 0
void pwm(int gpioNum, uint32_t frequency) {
    
	ledc_timer_config_t timer_conf;
	timer_conf.duty_resolution    = 8;
	timer_conf.freq_hz    = frequency;
	timer_conf.speed_mode = 0; //High speed mode should be 0...  LEDC_LOW_SPEED_MODE;  // LEDC_HIGH_SPEED_MODE;
	timer_conf.timer_num  = LEDC_TIMER_3;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
	ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
   
	ledc_channel_config_t ledc_conf;
	ledc_conf.channel    = LEDC_CHANNEL_0;
	ledc_conf.duty       = 2;
	ledc_conf.gpio_num   = gpioNum;
    ledc_conf.hpoint = 0;
	ledc_conf.intr_type  = LEDC_INTR_DISABLE;
	ledc_conf.speed_mode = 0; //LEDC_HIGH_SPEED_MODE;
	ledc_conf.timer_sel  = LEDC_TIMER_3;
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_conf));

}

#endif

//192.168.1.127
void setup();

void app_main(void)
{
    nvs_flash_init();
#if defined(SUMP_OVER_UART)
    init_uart();
#endif
    
ota_event_group = xEventGroupCreate();

#if defined(SUMP_ON_NETWORK) ||  defined (SCPI_ON_NETWORK) || defined(DEBUG_SUMP)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    //esp_event_handler_register_instance

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                         &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
/*
      EventBits_t bits = xEventGroupWaitBits(ota_event_group,
            OTA_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
    // happened. 
    if (bits & OTA_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
*/


    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

#endif

//setup();

// Start OTA Server
#ifdef OTA_RUN_SERVER
	xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, 15, NULL);
#endif
    //size_t free8start=heap_caps_get_free_size(MALLOC_CAP_8BIT);
    //size_t free32start=heap_caps_get_free_size(MALLOC_CAP_32BIT);
    //ESP_LOGI(TAG,"free mem8bit: %d mem32bit: %d\n",free8start,free32start);
    //printf("free mem8bit: %d mem32bit: %d\n",free8start,free32start);

    #if 0
        // If you want to check the pixel clock
        gpio_set_direction(PIXEL_LEDC_PIN, GPIO_MODE_INPUT);
        pwm(PIXEL_LEDC_PIN,4000);
    #endif

    //gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);

    //gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    //gpio_set_level(GPIO_NUM_2, 0);


    //gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);	    
    //gpio_set_level(GPIO_NUM_4, 0);


#ifdef RMT_PULSES
    gpio_set_direction(PULSE_PIN, GPIO_MODE_OUTPUT);
    send_remote_pulses();
    rmt_write_items(config.channel, items, 1, 0);
#endif

 //gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);

#ifdef UART_TEST_OUTPUT
    // To look at test UART data
    init_uart_1();
    xTaskCreatePinnedToCore(&uartWRITETask, "uartw", 4096, NULL, 20, &TaskHandle_tmp, 0);
    xTaskList[xtaskListCounter++] = TaskHandle_tmp;

#endif

  // Analouge out, however this interferes with analogue in
  //xTaskCreate(example_i2s_adc_dac, "example_i2s_adc_dac", 1024 * 2, NULL, 21, NULL);

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  /* Initialize file storage */
  ESP_ERROR_CHECK(init_spiffs());


#ifdef SCPI_ON_NETWORK
    scpi_server_init(&xHandlingTask);
#endif
#if defined(SUMP_ON_NETWORK) || defined(SUMP_OVER_UART)
    sump_init();
#endif
#if defined(SUMP_ON_NETWORK) || defined(DEBUG_SUMP)
    sump_server_init();
#endif
#if defined (SUMP_OVER_UART)
    sump_uart();
#endif

 ESP_ERROR_CHECK(start_file_server("/www"));

#if 0
  ledc_fade_func_install(0);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

    printf("Fade to 4000\n");

    ledc_set_fade_with_time(0,
                    LEDC_CHANNEL_0, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(0 /*Speed mode */,
                    LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);

  vTaskDelay(4000 / portTICK_PERIOD_MS);
    printf("Fade to 0\n");
    ledc_set_fade_with_time(0,
                    LEDC_CHANNEL_0, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(0 /*Speed mode */,
                    LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);


  vTaskDelay(4000 / portTICK_PERIOD_MS);

    printf("Fade to 4000\n");

    ledc_set_fade_with_time(0,
                    LEDC_CHANNEL_0, LEDC_TEST_DUTY, 1000);
            ledc_fade_start(0 /*Speed mode */,
                    LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);

#endif
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  vTaskDelay(2000 / portTICK_PERIOD_MS);

    vTaskDelete(NULL);

}
