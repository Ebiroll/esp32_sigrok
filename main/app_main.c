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
#include "analog.h"
#include <driver/rmt.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "app-config.h"
#include "Header.h"
#include "ota_server.h"

#ifdef RUN_IN_QEMU
#include "emul_ip.h"
#endif

#ifdef CONFIG_EXAMPLE_USE_TFT
#include "tftspi.h"
#include "tft.h"

// ==========================================================
// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
#define SPI_BUS TFT_HSPI_HOST
// ==========================================================

#endif



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
      .baud_rate = 2400,                    //baudrate was 9600
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
  uart_port_t uart_num = UART_NUM_1;    
  echoLine[0]='s';
  echoLine[1]='u';
  echoLine[2]='m';
  echoLine[3]='p';

  while(true) {
    (void) uart_write_bytes(uart_num, (const char *)echoLine, 4);
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}
#endif

#if 0
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
    level=!level;
    //uart_write_bytes(uart_num, (char *)echoLine, 1);
    printf("%c",echoLine[0]);
    uart_flush(0);
  }
}
#endif

esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
        printf("Connectiing To SSID:%s : Pass:%s\r\n", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
        printf("got ip:%s\r\n",	ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		xEventGroupSetBits(ota_event_group, OTA_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
        printf("SYSTEM_EVENT_STA_DISCONNECTED\r\n");
		esp_wifi_connect();
		xEventGroupClearBits(ota_event_group, OTA_CONNECTED_BIT);
		break;
	default:
		break;
	}
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
  config.gpio_num = PULSE_PIN; //STEP_PIN;, we use pin 17 directly, this way no cable is needed.
  config.mem_block_num = 1;
  config.tx_config.loop_en = 1;
  config.tx_config.carrier_en = 0;
  config.tx_config.idle_output_en = 1;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  config.clk_div = 80; // 80MHz / 80 = 1MHz 0r 1uS per count
 
  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
   
  items[0].duration0 = 1000;  // 1 ms (30)
  items[0].level0 = 1;
  items[0].duration1 = 500;   // 0.5 ms  (15)
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



#ifdef CONFIG_EXAMPLE_USE_TFT

void drawSampleData(int* in_data,int num_samples);

//=============
void tft_init()
{
    //test_sd_card();
    // ========  PREPARE DISPLAY INITIALIZATION  =========

    esp_err_t ret;

    // === SET GLOBAL VARIABLES ==========================

    // ===================================================
    // ==== Set display type                         =====
    //tft_disp_type = DEFAULT_DISP_TYPE;
	tft_disp_type = DISP_TYPE_ILI9341;
	//tft_disp_type = DISP_TYPE_ILI9488;
	//tft_disp_type = DISP_TYPE_ST7735B;
    //tft_disp_type =  DISP_TYPE_ST7735:
    // DISP_TYPE_ST7735R,

    // ===================================================

	// ===================================================
	// === Set display resolution if NOT using default ===
	// === DEFAULT_TFT_DISPLAY_WIDTH &                 ===
    // === DEFAULT_TFT_DISPLAY_HEIGHT                  ===
	_width = DEFAULT_TFT_DISPLAY_WIDTH;  // smaller dimension
	_height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
	//_width = 128;  // smaller dimension
	//_height = 160; // larger dimension
	// ===================================================

	// ===================================================
	// ==== Set maximum spi clock for display read    ====
	//      operations, function 'find_rd_speed()'    ====
	//      can be used after display initialization  ====
	max_rdclock = 8000000;
	// ===================================================

    // ====================================================================
    // === Pins MUST be initialized before SPI interface initialization ===
    // ====================================================================
    TFT_PinsInit();

    // ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

    spi_lobo_device_handle_t spi;
	
    spi_lobo_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,				// set SPI MISO pin
        .mosi_io_num=PIN_NUM_MOSI,				// set SPI MOSI pin
        .sclk_io_num=PIN_NUM_CLK,				// set SPI CLK pin
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
		.max_transfer_sz = 6*1024,
    };
    spi_lobo_device_interface_config_t devcfg={
        .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
        .mode=0,                                // SPI mode 0
        .spics_io_num=-1,                       // we will use external CS pin
		.spics_ext_io_num=PIN_NUM_CS,           // external CS pin
		.flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };


    // ====================================================================================================================


    vTaskDelay(500 / portTICK_RATE_MS);
	printf("==============================\r\n");
    printf("Pins used: miso=%d, mosi=%d, sck=%d, cs=%d\r\n", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
	printf("==============================\r\n\r\n");

	// ==================================================================
	// ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

	ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    assert(ret==ESP_OK);
	printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
	disp_spi = spi;

	// ==== Test select/deselect ====
	ret = spi_lobo_device_select(spi, 1);
    assert(ret==ESP_OK);
	ret = spi_lobo_device_deselect(spi);
    assert(ret==ESP_OK);

	printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
	printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");


	// ================================
	// ==== Initialize the Display ====

	printf("SPI: display init...\r\n");
	TFT_display_init();
    printf("OK\r\n");
	
	// ---- Detect maximum read speed ----
	max_rdclock = find_rd_speed();
	printf("SPI: Max rd speed = %u\r\n", max_rdclock);

    // ==== Set SPI clock used for display operations ====
	spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
	printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

    printf("\r\n---------------------\r\n");
	printf(" TFT started\r\n");

	printf("---------------------\r\n");

    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_setFont(DEFAULT_FONT, NULL);
	TFT_resetclipwin();

    char Buff[120];
    sprintf(Buff,"To start, press boot");

    TFT_print(Buff,20,40);

/*
==============================
Pins used: miso=25, mosi=23, sck=19, cs=22
==============================

SPI: display device added to spi bus (2)
SPI: attached display device, speed=8000000
SPI: bus uses native pins: false
SPI: display init...

*/

}

static void tft_trig_task(void *inpar) {


    int level=gpio_get_level(0);
    // Check boot button and trigger sampling
    while(1) {
      // You need to connect the debugger for this to work
      //while(gpio_get_level(0)==level) {
      //      vTaskDelay(100 / portTICK_RATE_MS);
      //  }
        TFT_fillWindow(TFT_BLACK);

        printf("Start sampling\n");
        vTaskDelay(50 / portTICK_RATE_MS);

       while(true) {  // gpio_get_level(0)==level
         start_sampling();
         if (samples_finnished()) {
             int *samp=get_sample_values();
             drawSampleData(samp,NUM_SAMPLES);
             //printf("s=%d,%d\n",*samp,*(samp+1));

             /*
             int num_in_loop=0;
             // Try trigger on rising edge
             while(num_in_loop < NUM_SAMPLES-320) {
                 if (*samp < *(samp+1) ) {
                     if (*(samp+1) > 20) {
                        drawSampleData(samp,NUM_SAMPLES-num_in_loop);
                        printf("s=%d\n",num_in_loop);
                        break;
                     }
                     samp++;
                     num_in_loop++;
                 }
             }
             if (num_in_loop > NUM_SAMPLES-340) {
                 int *samp2=get_sample_values();
                 drawSampleData(samp2,NUM_SAMPLES);
             }
             */
         } else {
             printf("NOT FINISHED!!!!\n");
             int *samp=get_sample_values();
             drawSampleData(samp,NUM_SAMPLES);
         }
      }
      level=gpio_get_level(0);

    }
}


#endif


void app_main(void)
{
    nvs_flash_init();
    //init_uart();

#if 0
    xTaskCreatePinnedToCore(&test_sample_task, "test_sample_task", 4096, NULL, 10, &xHandlingTask, 0);

    // Test analog thread
    xTaskCreatePinnedToCore(&sample_thread, "sample_thread", 4096, &xHandlingTask, 20, NULL, 0);
#endif
ota_event_group = xEventGroupCreate();

#if RUN_IN_QEMU
   if (is_running_qemu()) {
          tcpip_adapter_init();
          ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	      xTaskCreatePinnedToCore(task_lwip_init, "loop", 3*4096, NULL, 14, NULL, 0);
    }
#else
#if defined(SUMP_ON_NETWORK) ||  defined (SCPI_ON_NETWORK) || defined(DEBUG_SUMP)
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

  
#endif
#endif


// Start OTA Server
#ifdef OTA_RUN_SERVER
	xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, 15, NULL);
#endif
    //size_t free8start=heap_caps_get_free_size(MALLOC_CAP_8BIT);
    //size_t free32start=heap_caps_get_free_size(MALLOC_CAP_32BIT);
    //ESP_LOGI(TAG,"free mem8bit: %d mem32bit: %d\n",free8start,free32start);
    //printf("free mem8bit: %d mem32bit: %d\n",free8start,free32start);

    //gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);

    //gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    //gpio_set_level(GPIO_NUM_2, 0);


    //gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);	    
    //gpio_set_level(GPIO_NUM_4, 0);

#if 0
    // RGB leds on wrover kit
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_0, 0);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_4, 0);
#endif

#ifdef RMT_PULSES
    gpio_set_direction(PULSE_PIN, GPIO_MODE_OUTPUT);
    send_remote_pulses();
    rmt_write_items(config.channel, items, 1, 0);
#endif

#ifdef UART_TEST_OUTPUT
    // To look at test UART data 2400 Baud on pin 18
    init_uart_1();
    xTaskCreatePinnedToCore(&uartWRITETask, "uartw", 4096, NULL, 20, &TaskHandle_tmp, 0);
    xTaskList[xtaskListCounter++] = TaskHandle_tmp;

#endif

    // Analouge out, however this interferes with analogue in
    //xTaskCreate(example_i2s_adc_dac, "example_i2s_adc_dac", 1024 * 2, NULL, 21, NULL);

#ifdef CONFIG_EXAMPLE_USE_TFT
    tft_init();
    xTaskCreatePinnedToCore(&tft_trig_task, "trig", 4096, NULL, 20, &xHandlingTask, 1);
    xTaskList[xtaskListCounter++] = xHandlingTask;
#endif

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

    vTaskDelete(NULL);

}
