
#include <stdint.h>
#include "soc/i2s_struct.h"
#include "config.h"
#include "rom/lldesc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"


#define CAPTURE_SIZE 14000

#define ledPin 21 //Led on while running and Blinks while transfering data.


typedef enum {
  I2S_PARALLEL_BITS_8   = 8,
  I2S_PARALLEL_BITS_16  = 16,
  I2S_PARALLEL_BITS_32  = 32,
} i2s_parallel_cfg_bits_t;

typedef struct {
  //void* memory;
  size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct {
  int gpio_bus[24];
  int gpio_clk;
  int clkspeed_hz;
  i2s_parallel_cfg_bits_t bits;
  i2s_parallel_buffer_desc_t* buf;
} i2s_parallel_config_t;

typedef struct {
  volatile lldesc_t* dmadesc;
  int desccount;
} i2s_parallel_state_t;

void i2s_conf_reset();

void enable_out_clock( int freq_in_hz );

void start_dma_capture(void);


#define DMA_MAX (4096-4)



//Calculate the amount of dma descs needed for a buffer desc
static int calc_needed_dma_descs_for(i2s_parallel_buffer_desc_t *desc) {
  int ret = (desc->size + DMA_MAX - 1) / DMA_MAX;
  return ret;
}

typedef union {
    struct {
        uint8_t sample2;
        uint8_t unused2;
        uint8_t sample1;
        uint8_t unused1;
      };
    struct{
      uint16_t val2;
      uint16_t val1;
      };
    uint32_t val;
} dma_elem_t;

typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

typedef struct {
    lldesc_t *dma_desc;
    dma_elem_t **dma_buf;
    bool dma_done;
    size_t dma_desc_count;
    size_t dma_desc_cur;
    int dma_desc_triggered;
    size_t dma_received_count;
    size_t dma_filtered_count;
    size_t dma_buf_width;
    size_t dma_sample_count;
    size_t dma_val_per_desc;
    size_t dma_sample_per_desc;
    i2s_sampling_mode_t sampling_mode;
//    dma_filter_t dma_filter;
    intr_handle_t i2s_intr_handle;
//    QueueHandle_t data_ready;
//    SemaphoreHandle_t frame_ready;
//    TaskHandle_t dma_filter_task;
} camera_state_t;


void i2s_parallel_setup( const i2s_parallel_config_t *cfg);


#define CHANPIN GPIO.in

uint8_t channels_to_read=3;

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

#define MAX_CAPTURE_SIZE CAPTURE_SIZE




