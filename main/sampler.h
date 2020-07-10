
#include <stdint.h>
#include "soc/i2s_struct.h"
#include "config.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "rom/lldesc.h"
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/lldesc.h"
#endif
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"


#define CAPTURE_SIZE 14000

#define ledPin 39 //Led on while running and Blinks while transfering data.


void enable_out_clock( int freq_in_hz );


#define DMA_MAX (4096-4)


/*
 *  SLC2 DMA Desc struct, aka lldesc_t
 *
 * --------------------------------------------------------------
 * | own | EoF | sub_sof | 5'b0   | length [11:0] | size [11:0] |
 * --------------------------------------------------------------
 * |            buf_ptr [31:0]                                  |
 * --------------------------------------------------------------
 * |            next_desc_ptr [31:0]                            |
 * --------------------------------------------------------------
 */

/* this bitfield is start from the LSB!!! */
#if 0
typedef struct lldesc_s {
    volatile uint32_t size  :12,
                        length:12,
                        offset: 5, /* h/w reserved 5bit, s/w use it as offset in buffer */
                        sosf  : 1, /* start of sub-frame */
                        eof   : 1, /* end of frame */
                        owner : 1; /* hw or sw */
    volatile const uint8_t *buf;       /* point to buffer data */
    union{
        volatile uint32_t empty;
        STAILQ_ENTRY(lldesc_s) qe;  /* pointing to the next desc */
    };
} lldesc_t;
#endif

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

//uint8_t* get_sample_values();


// Param is task handle of task to notify
void sample_thread(void *param);

void setTimescale(float scale);

void start_sampling();

bool samples_finnished();




