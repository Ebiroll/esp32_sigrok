
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

#define ledPin 27 //Led on while running and Blinks while transfering data.


void enable_out_clock( int freq_in_hz );


#define DMA_MAX (4096-4)

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


uint8_t* get_values();

uint16_t* get_digital_values();

int* get_sample_values();


// Param is task handle of task to notify
void sample_thread(void *param);

void setTimescale(float scale);

void start_sampling();

bool samples_finnished();




