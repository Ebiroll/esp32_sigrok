#if 0

#include <stdbool.h>
#include <stdio.h>
#include "analog.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "nvs_flash.h"
#include "sampler.h"
#include "driver/ledc.h"
#include "app-config.h"

static const char *TAG = "sampler";

#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define LEDC_HS_TIMER          LEDC_TIMER_3
#define LEDC_HS_MODE           0
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
// LEDC_HIGH_SPEED_MODE
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif

// 1400/4
#define SAMPLE_LENGTH 140
#define MAX_DMA       10
//DMA_ATTR uint32_t dma_buff[MAX_DMA][SAMPLE_LENGTH*2];
//DMA_ATTR lldesc_t dma_descriptor[MAX_DMA];

uint32_t *dma_buff;
lldesc_t *dma_descriptor;

TrigState_t g_trig_state=Stopped;

TrigState_t get_trig_state(){
    return(g_trig_state);
}

bool buffers_allocated=false;

int trig_pin;

bool ledPinVal=false;
int isRcounter=0;
intr_handle_t isrHandle;

void setupSimple(void);

// Not sure why this wont trigger
#if 1
static void IRAM_ATTR i2s_trigger_isr(void) {

  if (ledPinVal)
     gpio_set_level(ledPin, 1);
  else
     gpio_set_level(ledPin, 0);

  ledPinVal=!ledPinVal;
  isRcounter++;

  //I2S0.conf.rx_start = 1;

  //I2S0.rx_eof_num = readCount/2

  I2S0.int_clr.val = I2S0.int_raw.val;   

  I2S0.conf.rx_start = 1;

}
#endif


static int maxSamples=NUM_SAMPLES;

void set_mem_depth(int depth) {
    if (depth<NUM_SAMPLES) {
        maxSamples=depth;
    }
}



int stop_aq=false;
void stop_aquisition() {
    stop_aq=true;
}

void i2s_conf_reset(){
  // Toggle some reset bits in LC_CONF register
  I2S0.lc_conf.in_rst = 1;
  I2S0.lc_conf.in_rst = 0;
  I2S0.lc_conf.ahbm_rst = 1;
  I2S0.lc_conf.ahbm_rst = 0;
  I2S0.lc_conf.ahbm_fifo_rst = 1;
  I2S0.lc_conf.ahbm_fifo_rst = 0;

  // Toggle some reset bits in CONF register
  I2S0.conf.rx_reset = 1;
  I2S0.conf.rx_reset = 0;
  I2S0.conf.rx_fifo_reset = 1;
  I2S0.conf.rx_fifo_reset = 0;
  I2S0.conf.tx_reset = 1;
  I2S0.conf.tx_reset = 0;
  I2S0.conf.tx_fifo_reset = 1;
  I2S0.conf.tx_fifo_reset = 0;
  //while (I2S0.state.rx_fifo_reset_back) {}
  }


void enable_out_clock( int freq_in_hz ) {

  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/peripherals/ledc.html
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT, // resolution of PWM duty
        .freq_hz =    freq_in_hz,                      // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,                        // timer mode, only low speed available on esp32s2
        .timer_num = LEDC_TIMER_2,            // timer index
        .clk_cfg   = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);


#if defined CONFIG_IDF_TARGET_ESP32S2
 ledc_channel_config_t ledc_channel = 
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 1,
            .gpio_num   = PIXEL_LEDC_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_2,
            .intr_type  = LEDC_INTR_DISABLE
        };
#else
    ledc_channel_config_t ledc_channel = 
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 1,
            .gpio_num   = PIXEL_LEDC_PIN,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        };
#endif
   ledc_channel_config(&ledc_channel);
}

void setupScale(float timescale) {
  double rate = 1000.0 / timescale ;
  // Slow down 100 times for debugging, rate=rate/100;
  enable_out_clock((int)rate);
  printf("Capture Speed : %.2f Khz\r\n", rate/1000.0);
}


void setTimescale(float scale){
  setupScale(scale);
  printf("setTimescale=%.3f\n",scale);

}



void allocate_buffers() {
//uint32_t *dma_buff[MAX_DMA];
//lldesc_t *dma_descriptor;
  dma_descriptor=(lldesc_t *)heap_caps_malloc(MAX_DMA * sizeof(lldesc_t), MALLOC_CAP_DMA);
  dma_buff=(uint32_t *) heap_caps_malloc(MAX_DMA * SAMPLE_LENGTH* 2* sizeof(uint8_t), MALLOC_CAP_DMA);

  uint16_t* tmp=(uint16_t*)dma_buff;
  for (int j=0;j<1400;j++) {
    tmp[j]=j;
  }

}

void start_sampling(){
  if (!buffers_allocated) {
     buffers_allocated=true;
     allocate_buffers();
  }

  gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
  setupSimple();

}


void setupSimple(void) {

   periph_module_enable(PERIPH_I2S0_MODULE);

    i2s_conf_reset();
    vTaskDelay(50 / portTICK_PERIOD_MS);

   for(int i=0;i<MAX_DMA-1;i++) {
    dma_descriptor[i].length =0; // sizeof(dma_buff[0]); //number of byte written to the buffer
    dma_descriptor[i].size = 140*2; // sizeof(dma_buff[0]); //max size of the buffer in bytes
    dma_descriptor[i].owner = 1;
    dma_descriptor[i].sosf = 0;
    dma_descriptor[i].buf = (uint8_t*) (uint8_t*) dma_buff + i * SAMPLE_LENGTH * 2;
    dma_descriptor[i].offset = 0;
    dma_descriptor[i].empty = (uint32_t) (lldesc_t *) &dma_descriptor[i+1];
    dma_descriptor[i].eof = 0;
    //pointer to the next descriptor
    //dma_descriptor[i].qe.stqe_next = &dma_descriptor[i+1];
   }


    //buff 1
    dma_descriptor[MAX_DMA-1].length = 0; //number of byte written to the buffer
    dma_descriptor[MAX_DMA-1].size = sizeof(dma_buff[0]); //max size of the buffer in bytes
    dma_descriptor[MAX_DMA-1].owner = 1;
    dma_descriptor[MAX_DMA-1].sosf = 1;
    dma_descriptor[MAX_DMA-1].buf = (uint8_t*) dma_buff + ( MAX_DMA-1) * SAMPLE_LENGTH * 2;
    dma_descriptor[MAX_DMA-1].offset = 0;
    dma_descriptor[MAX_DMA-1].empty = 0;
    dma_descriptor[MAX_DMA-1].eof = 1; //Mark last DMA desc as end of stream.
    //pointer to the next descriptor, end here
    dma_descriptor[9].qe.stqe_next = 0; 

    //data inputs
    #ifndef UART_TEST_OUTPUT
    //gpio_set_direction(17, GPIO_MODE_INPUT);
    #endif
    gpio_set_direction(13, GPIO_MODE_INPUT);
    gpio_matrix_in(13,    I2S0I_DATA_IN0_IDX, false);
    gpio_set_direction(14, GPIO_MODE_INPUT);
    gpio_matrix_in(14,    I2S0I_DATA_IN1_IDX, false); 
    #ifndef RMT_PULSES
    gpio_set_direction(15, GPIO_MODE_INPUT);
    #endif    
    gpio_matrix_in(15,    I2S0I_DATA_IN2_IDX, false);
    gpio_set_direction(16, GPIO_MODE_INPUT);
    gpio_matrix_in(16,    I2S0I_DATA_IN3_IDX, false);
    #ifndef UART_TEST_OUTPUT
    gpio_set_direction(17, GPIO_MODE_INPUT);
    #endif
    gpio_matrix_in(17,    I2S0I_DATA_IN4_IDX, false);
    gpio_set_direction(18, GPIO_MODE_INPUT);
    gpio_matrix_in(18,    I2S0I_DATA_IN5_IDX, false);
    gpio_set_direction(19, GPIO_MODE_INPUT);
    gpio_matrix_in(19,    I2S0I_DATA_IN6_IDX, false);
    // Not sure why 20 is missing???
    gpio_set_direction(21, GPIO_MODE_INPUT);
    gpio_matrix_in(21,    I2S0I_DATA_IN7_IDX, false);



    //gpio_set_direction(config->pin.vsync, GPIO_MODE_INPUT);
    //gpio_set_pull_mode(config->pin.vsync, GPIO_FLOATING);


    //for i2s in parallel camera input mode data is receiver only when V_SYNC = H_SYNC = H_ENABLE = 1. We don't use these inputs so simply set them High
    gpio_matrix_in(0x38, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);  //0x30 sends 0, 0x38 sends 1
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);

    // Pixel clock is generated from pwm, should be connected internally
    //gpio_set_direction(PIXEL_LEDC_PIN, GPIO_MODE_INPUT);
    // Pullup
// PIXEL_LEDC_PIN
// Allow connecting pins externally
 #define MY_PXCLK   33
    //gpio_set_pull_mode(MY_PXCLK,GPIO_FLOATING);
    //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[MY_PXCLK],PIN_FUNC_GPIO);
    //gpio_set_direction(MY_PXCLK, GPIO_MODE_INPUT);
    gpio_matrix_in(MY_PXCLK, I2S0I_WS_IN_IDX, false);  // PXCLK 
    
    // Enable and configure I2S peripheral
    //periph_module_enable(PERIPH_I2S0_MODULE);

    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1; 
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;

    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.sample_rate_conf.rx_bck_div_num = 2; //i2s clk is divided before reaches BCK output 
       
    //I2S0.conf2.i_v_sync_filter_en = 0;
    //I2S0.conf2.i_v_sync_filter_thres = 0;
    I2S0.conf1.rx_pcm_bypass = 1;

    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    // FIFO configuration
    //two bytes per dword packing
    I2S0.fifo_conf.rx_fifo_mod = 1;   // 16-bit single channel dat
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 1;
    I2S0.conf.rx_short_sync = 1;
    I2S0.timing.val = 0;
    I2S0.sample_rate_conf.val = 0;

    I2S0.sample_rate_conf.rx_bits_mod = 16;
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.rx_bck_div_num = 1;


  //Setup I2S DMA Interrupt
  
  esp_err_t err = esp_intr_alloc( ETS_I2S0_INTR_SOURCE,
                    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                    &i2s_trigger_isr,  NULL, &isrHandle);
  
  //Enable the Interrupt
  ESP_ERROR_CHECK(esp_intr_enable(isrHandle));
  

    //start i2s
    I2S0.rx_eof_num = 1400;
    I2S0.in_link.addr = ((uint32_t)&dma_descriptor[0]) & 0xfffff;
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;

    // To enable interrups
    //I2S0.int_ena.val = 1;
    //I2S0.int_ena.in_done = 1;

    // Print values
    samples_finnished();

    I2S0.lc_conf.check_owner = 0;
    I2S0.int_clr.val = ~0;

    //start i2s + dma
    I2S0.conf.rx_start = 1;
    gpio_set_level(ledPin, 1);
    printf("<====> DMA INT Number %d Status 0x%x \n", isRcounter, I2S0.int_raw.val);
    gpio_set_level(ledPin, 0);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    //stop i2s + dma
    I2S0.conf.rx_start = 0;
    g_trig_state=Triggered;
}

bool samples_finnished() 
{
  printf("(%d)\n",I2S0.fifo_conf.rx_data_num);
  printf("<====> DMA INT Number %d Status 0x%x \n", isRcounter, I2S0.int_raw.val);
  ESP_LOGD(TAG, "DMA INT take_data? %d", I2S0.int_raw.rx_take_data );
  ESP_LOGD(TAG, "DMA INT in_dscr_empty? %d", I2S0.int_raw.in_dscr_empty );
  ESP_LOGD(TAG, "DMA INT in_done? %d", I2S0.int_raw.in_done );
  ESP_LOGD(TAG, "DMA INT in_suc_eof? %d", I2S0.int_raw.in_suc_eof );
  ESP_LOGD(TAG, "DMA INT rx_rempty? %d", I2S0.int_raw.rx_rempty );

  ESP_LOGD(TAG, "DMA INT in_err_eof? %d", I2S0.int_raw.in_err_eof );
  ESP_LOGD(TAG, "DMA INT in_dscr_err? %d", I2S0.int_raw.in_dscr_err );
  ESP_LOGD(TAG, "DMA INT in_dscr_empty? %d\r\n", I2S0.int_raw.in_dscr_empty );

  //stop i2s + dma
  I2S0.conf.rx_start = 0;
  //ESP_LOGD( "\r\n" );
  
  return true;
}


uint16_t* get_digital_values() {

  // Test data
  //uint16_t* tmp=(uint16_t*)dma_buff;
  //for (int j=0;j<1400;j++) {
  //  tmp[j]=j;
  // }

  return((uint16_t*)dma_buff);
}

// This should return the analog values
uint8_t* get_values() {
  //return((uint16_t*)s_state->dma_buf[0]);
  return((uint8_t*)dma_buff);
}
#endif
