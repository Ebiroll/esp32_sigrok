//
/*
MIT License

Copyright (c) 2017 Olof Astrand (Ebiroll)

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
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <sys/time.h>
#include "read_chars.h"
#include "esp32_sump.h" 
#include "scpi_server.h"

#include <driver/rmt.h>
 
#define STEP_PIN  GPIO_NUM_21

static const char *TAG = "uart";


#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#define BUF_SIZE 512

char echoLine[BUF_SIZE];

#if 0
static void init_uart_1()
{
  uart_port_t uart_num = UART_NUM_1;                                     //uart port number


  uart_config_t uart_config = {
      .baud_rate = 9600,                    //baudrate
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
  ESP_ERROR_CHECK( uart_set_pin(uart_num, 18, 19 , -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 2, 512 * 2, 10,  &uart_queue,0));

}

static void uartWRITETask(void *inpar) {
  uart_port_t uart_num = UART_NUM_1;    
  echoLine[0]='s';
  echoLine[1]='u';
  echoLine[2]='m';
  echoLine[3]='p';

  while(true) {
    int size = uart_write_bytes(uart_num, (const char *)echoLine, 4);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
#endif

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


// This task only prints what is received on UART1
static void uartECHOTask(void *inpar) {
  uart_port_t uart_num = UART_NUM_0;                                     //uart port number

  printf("ESP32 uart echo\n");
  int level=0;

  while(1) {
     int size = uart_read_bytes(uart_num, (unsigned char *)echoLine, 1, 500/portTICK_PERIOD_MS);
     if (size<=0) {
         echoLine[0]='T';
     }
    gpio_set_level(GPIO_NUM_2, level);
    if (level==1) 
    {
        level=0;
    } else {
        level=1;
    }     
     uart_write_bytes(uart_num, (char *)echoLine, 1);
     //printf("%c",echoLine[0]);
     uart_flush(0);
  }
  //sump_init();
  //sump_uart();
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
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
  config.gpio_num = 13; //STEP_PIN;, we use pin 14 directly, this way no cable is needed.
  config.mem_block_num = 1;
  config.tx_config.loop_en = 1;
  config.tx_config.carrier_en = 0;
  config.tx_config.idle_output_en = 1;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  config.clk_div = 80; // 80MHx / 80 = 1MHz 0r 1uS per count
 
  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
   
  items[0].duration0 = 30000;  // 30.000 us
  items[0].level0 = 1;
  items[0].duration1 = 15000;   // 15 ms
  items[0].level1 = 0;  

}


static void remoteTask(void *inpar) {
    rmt_write_items(config.channel, items, 1, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

void sump_server_init(void);

void app_main(void)
{
    nvs_flash_init();
    init_uart();

#if 1
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    scpi_server_init();
#endif

    size_t free8start=heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t free32start=heap_caps_get_free_size(MALLOC_CAP_32BIT);
    ESP_LOGI(TAG,"free mem8bit: %d mem32bit: %d\n",free8start,free32start);
    printf("free mem8bit: %d mem32bit: %d\n",free8start,free32start);

    //gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);


    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);	    
    gpio_set_level(GPIO_NUM_4, 0);

#if 0
    // RGB leds on wrover kit
        gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

	    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
		gpio_set_level(GPIO_NUM_0, 0);
        gpio_set_level(GPIO_NUM_2, 0);
	    gpio_set_level(GPIO_NUM_4, 0);

	}
#endif


    // esp_err_t rmt_write_items(rmt_channel_t channel, rmt_item32_t *rmt_item, int item_num, bool wait_tx_done)
    send_remote_pulses();
    rmt_write_items(config.channel, items, 1, 0);


    //xTaskCreatePinnedToCore(&remoteTask, "remote", 4096, NULL, 20, NULL, 0);

    // To look at test data for 
    //init_uart_1();
    //xTaskCreatePinnedToCore(&uartWRITETask, "uartw", 4096, NULL, 20, NULL, 1);

    sump_init();
    sump_server_init();
    sump_uart();

    //xTaskCreatePinnedToCore(&uartECHOTask, "echo", 4096, NULL, 20, NULL, 0);
#if 0
    int test=0;
    for(;;) 
    {
        //int *GPIO_STRAP_TEST=(int *)0x3ff44038;
        //printf( "GPIO STRAP REG=%08X\n", *GPIO_STRAP_TEST);

        int *GPIO_IN_REG_TEST=(int *)0x3ff4403C;
        if (test!=*GPIO_IN_REG_TEST) {
            test=*GPIO_IN_REG_TEST;
            printf( "GPIO_IN_REG=%08X\n", *GPIO_IN_REG_TEST);

        }

        //vTaskDelay(200 / portTICK_PERIOD_MS);
    }
#endif


}
