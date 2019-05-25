// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*  Also look here for newer implementation of this
 *  https://github.com/Spritetm/printercart_simple/tree/master/components/printcart
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "rom/lldesc.h"
#include "esp_heap_caps.h"
#include "anim.h"
#include "val2pwm.h"
#include "i2s_parallel.h"
#include "driver/gpio.h"

typedef struct {
    volatile lldesc_t *dmadesc_a, *dmadesc_b;
    int desccount_a, desccount_b;
} i2s_parallel_state_t;

static i2s_parallel_state_t *i2s_state[2]={NULL, NULL};

#define DMA_MAX (4096-4)

//Calculate the amount of dma descs needed for a buffer desc
static int calc_needed_dma_descs_for(i2s_parallel_buffer_desc_t *desc) {
    int ret=0;
    for (int i=0; desc[i].memory!=NULL; i++) {
        ret+=(desc[i].size+DMA_MAX-1)/DMA_MAX;
    }
    return ret;
}

static void fill_dma_desc(volatile lldesc_t *dmadesc, i2s_parallel_buffer_desc_t *bufdesc) {
    int n=0;
    for (int i=0; bufdesc[i].memory!=NULL; i++) {
        int len=bufdesc[i].size;
        uint8_t *data=(uint8_t*)bufdesc[i].memory;
        while(len) {
            int dmalen=len;
            if (dmalen>DMA_MAX) dmalen=DMA_MAX;
            dmadesc[n].size=dmalen;
            dmadesc[n].length=dmalen;
            dmadesc[n].buf=data;
            dmadesc[n].eof=0;
            dmadesc[n].sosf=0;
            dmadesc[n].owner=1;
            dmadesc[n].qe.stqe_next=(lldesc_t*)&dmadesc[n+1];
            dmadesc[n].offset=0;
            len-=dmalen;
            data+=dmalen;
            n++;
        }
    }
    //Loop last back to first
    dmadesc[n-1].qe.stqe_next=(lldesc_t*)&dmadesc[0];
    printf("fill_dma_desc: filled %d descriptors\n", n);
}

static void gpio_setup_out(int gpio, int sig) {
    if (gpio==-1) return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(gpio, sig, false, false);
}


static void gpio_setup_in(int gpio, int sig) {
    if (gpio==-1) return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_DEF_INPUT);
    gpio_matrix_in(gpio, sig, false);
}


static void dma_reset(i2s_dev_t *dev) {
    dev->lc_conf.in_rst=1; dev->lc_conf.in_rst=0;
    dev->lc_conf.out_rst=1; dev->lc_conf.out_rst=0;
}

static void fifo_reset(i2s_dev_t *dev) {
    dev->conf.rx_fifo_reset=1; dev->conf.rx_fifo_reset=0;
    dev->conf.tx_fifo_reset=1; dev->conf.tx_fifo_reset=0;
}

static int i2snum(i2s_dev_t *dev) {
    return (dev==&I2S0)?0:1;
}

void i2s_parallel_setup(i2s_dev_t *dev, const i2s_parallel_config_t *cfg) {
    //Figure out which signal numbers to use for routing
    printf("Setting up parallel I2S bus at I2S%d\n", i2snum(dev));
    int sig_data_base, sig_clk;
    if (dev==&I2S0) {
        sig_data_base=I2S0I_DATA_IN0_IDX;
        sig_clk=I2S0I_WS_IN_IDX;
    } else {
        if (cfg->bits==I2S_PARALLEL_BITS_32) {
            sig_data_base=I2S1I_DATA_IN0_IDX;
        } else {
            //Because of... reasons... the 16-bit values for i2s1 appear on d8...d23
            // Not sure if this applies to input???
            sig_data_base=I2S1I_DATA_IN8_IDX;
        }
        sig_clk=I2S1I_WS_IN_IDX;
    }
    
    //Route the signals
    for (int x=0; x<cfg->bits; x++) {
        gpio_setup_in(cfg->gpio_bus[x], sig_data_base+x);
    }
    //ToDo: Clk/WS may need inversion?
    gpio_setup_in(cfg->gpio_clk, sig_clk);
    
    //Power on dev
    if (dev==&I2S0) {
        periph_module_enable(PERIPH_I2S0_MODULE);
    } else {
        periph_module_enable(PERIPH_I2S1_MODULE);
    }
    //Initialize I2S dev
    dev->conf.rx_reset=1; dev->conf.rx_reset=0;
    dev->conf.tx_reset=1; dev->conf.tx_reset=0;
    dma_reset(dev);
    fifo_reset(dev);
    
    /*
    8.3 The Clock of I2S Module(Reference Manual)
        I2Sn_CLK, as the master clock of I2S module, 
        is derived from the 160 MHz clock PLL_D2_CLK 
        or the configurable analog PLL output clock APLL_CLK. 
        PLL_D2_CLK is used as the clock source for I2Sn, bydefault. 
    */
    // Enable slave mode (sampling clock is external)
    // We will connect the output of the remote device (RMT) to the pixel clock.
    dev->conf.rx_slave_mod = 1; 
    // Enable parallell mode
    //dev->conf2.val=0;
    dev->conf2.lcd_en=1;
    
    // Use HSYNC/VSYNC/HREF to control sampling
    dev->conf2.camera_en = 1;

    // Configure clock divider
    dev->clkm_conf.clkm_div_a = 1;
    dev->clkm_conf.clkm_div_b = 0;
    dev->clkm_conf.clkm_div_num = 2;
    dev->sample_rate_conf.rx_bck_div_num = 2; //i2s clk is divided before reaches BCK output 

    // FIFO will sink data to DMA
    dev->fifo_conf.dscr_en = 1;
    dev->fifo_conf.tx_fifo_mod_force_en = 1;
    dev->fifo_conf.rx_fifo_mod_force_en = 1;
    // FIFO configuration
    //two bytes per dword packing
    dev->fifo_conf.rx_fifo_mod = 1;
    dev->conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    dev->sample_rate_conf.rx_bits_mod = 0;
    dev->conf.rx_right_first = 0;
    dev->conf.rx_msb_right = 0;
    dev->conf.rx_msb_shift = 0;
    dev->conf.rx_mono = 0;
    dev->conf.rx_short_sync = 0;
    dev->timing.val = 0;
    //start i2s
    //dev->rx_eof_num = 0xFFFFFFFF;
    //dev->in_link.addr = (uint32_t)&dma_descriptor[0];
    //dev->in_link.start = 1;
    //dev->int_clr.val = dev->int_raw.val;
    //dev->int_ena.val = 0;

    //start i2s + dma
    //I2S0.conf.rx_start = 1;

    //delay(10000);

    // stop i2s + dma
    // I2S0.conf.rx_start = 0;

/*
    dev->sample_rate_conf.val=0;
    dev->sample_rate_conf.rx_bits_mod=cfg->bits;
    dev->sample_rate_conf.tx_bits_mod=cfg->bits;
    dev->sample_rate_conf.rx_bck_div_num=4; //1 for 20Mhz
    dev->sample_rate_conf.tx_bck_div_num=4;  //1 for 20mhz
    
    dev->clkm_conf.val=0;
    dev->clkm_conf.clka_en=0;
    dev->clkm_conf.clkm_div_a=1; //63
    dev->clkm_conf.clkm_div_b=1; //63
    //We ignore the possibility for fractional division here.
    dev->clkm_conf.clkm_div_num=80000000L/cfg->clkspeed_hz;
    
    dev->fifo_conf.val=0;
    dev->fifo_conf.rx_fifo_mod_force_en=1;
    dev->fifo_conf.tx_fifo_mod_force_en=1;
    dev->fifo_conf.tx_fifo_mod=1;
    dev->fifo_conf.tx_fifo_mod=1;
    dev->fifo_conf.rx_data_num=32; //Thresholds. 
    dev->fifo_conf.tx_data_num=32;
    dev->fifo_conf.dscr_en=1;
    
    dev->conf1.val=0;
    dev->conf1.tx_stop_en=0;
    dev->conf1.tx_pcm_bypass=1;
    
    dev->conf_chan.val=0;
    dev->conf_chan.tx_chan_mod=1;
    dev->conf_chan.rx_chan_mod=1;
    
    //Invert ws to be active-low... ToDo: make this configurable
    dev->conf.tx_right_first=1;
    dev->conf.rx_right_first=1;
    
    dev->timing.val=0;
    */

    //Allocate DMA descriptors
    i2s_state[i2snum(dev)]=malloc(sizeof(i2s_parallel_state_t));
    i2s_parallel_state_t *st=i2s_state[i2snum(dev)];
    st->desccount_a=calc_needed_dma_descs_for(cfg->bufa);
    st->desccount_b=calc_needed_dma_descs_for(cfg->bufb);
    st->dmadesc_a=heap_caps_malloc(st->desccount_a*sizeof(lldesc_t), MALLOC_CAP_DMA);
    st->dmadesc_b=heap_caps_malloc(st->desccount_b*sizeof(lldesc_t), MALLOC_CAP_DMA);
    
    //and fill them
    fill_dma_desc(st->dmadesc_a, cfg->bufa);
    fill_dma_desc(st->dmadesc_b, cfg->bufb);
    
    //Reset FIFO/DMA -> needed? Doesn't dma_reset/fifo_reset do this?
    dev->lc_conf.in_rst=1; dev->lc_conf.out_rst=1; dev->lc_conf.ahbm_rst=1; dev->lc_conf.ahbm_fifo_rst=1;
    dev->lc_conf.in_rst=0; dev->lc_conf.out_rst=0; dev->lc_conf.ahbm_rst=0; dev->lc_conf.ahbm_fifo_rst=0;
    dev->conf.tx_reset=1; dev->conf.tx_fifo_reset=1; dev->conf.rx_fifo_reset=1;
    dev->conf.tx_reset=0; dev->conf.tx_fifo_reset=0; dev->conf.rx_fifo_reset=0;
    
    //Start dma on front buffer
                     // I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
    dev->lc_conf.val=   I2S_INDSCR_BURST_EN  ;
    dev->in_link.addr=((uint32_t)(&st->dmadesc_a[0]));
    dev->in_link.start=1;
    // Start receive
    dev->conf.rx_start=1;
}

void i2s_parallel_start(i2s_dev_t *dev) {
    dev->conf.rx_start=1;
}
void i2s_parallel_stop(i2s_dev_t *dev) {
    dev->conf.rx_start=1;
    dev->conf.rx_reset=1;
}


//Flip to a buffer: 0 for bufa, 1 for bufb
void i2s_parallel_flip_to_buffer(i2s_dev_t *dev, int bufid) {
    int no=i2snum(dev);
    if (i2s_state[no]==NULL) return;
    lldesc_t *active_dma_chain;
    if (bufid==0) {
        active_dma_chain=(lldesc_t*)&i2s_state[no]->dmadesc_a[0];
    } else {
        active_dma_chain=(lldesc_t*)&i2s_state[no]->dmadesc_b[0];
    }

    i2s_state[no]->dmadesc_a[i2s_state[no]->desccount_a-1].qe.stqe_next=active_dma_chain;
    i2s_state[no]->dmadesc_b[i2s_state[no]->desccount_b-1].qe.stqe_next=active_dma_chain;
}

#if 0

#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/i2s.h"
#include "rom/lldesc.h"

DMA_ATTR uint32_t dma_buff[2][100];
DMA_ATTR lldesc_t dma_descriptor[2];

void setup() {
    Serial.begin(115200);
    Serial.println(ESP.getSdkVersion());
  
    //buff 0
    dma_descriptor[0].length = 0; //number of byte written to the buffer
    dma_descriptor[0].size = sizeof(dma_buff[0]); //max size of the buffer in bytes
    dma_descriptor[0].owner = 1;
    dma_descriptor[0].sosf = 1;
    dma_descriptor[0].buf = (uint8_t*)&dma_buff[0][0];
    dma_descriptor[0].offset = 0;
    dma_descriptor[0].empty = 0;
    dma_descriptor[0].eof = 0;
    //pointer to the next descriptor
    dma_descriptor[0].qe.stqe_next = &dma_descriptor[1];

    //buff 1
    dma_descriptor[1].length = 0; //number of byte written to the buffer
    dma_descriptor[1].size = sizeof(dma_buff[1]); //max size of the buffer in bytes
    dma_descriptor[1].owner = 1;
    dma_descriptor[1].sosf = 1;
    dma_descriptor[1].buf = (uint8_t*)&dma_buff[1][0];
    dma_descriptor[1].offset = 0;
    dma_descriptor[1].empty = 0;
    dma_descriptor[1].eof = 0;
    //pointer to the next descriptor
    dma_descriptor[1].qe.stqe_next = &dma_descriptor[0];

    //data inputs
    pinMode(18, INPUT);
    gpio_matrix_in(18,    I2S0I_DATA_IN0_IDX, false);
    pinMode(23, INPUT);
    gpio_matrix_in(23,    I2S0I_DATA_IN1_IDX, false);
    pinMode(19, INPUT);
    gpio_matrix_in(19,    I2S0I_DATA_IN2_IDX, false);
    //for i2s in parallel camera input mode data is receiver only when V_SYNC = H_SYNC = H_ENABLE = 1. We don't use these inputs so simply set them High
    gpio_matrix_in(0x38, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);  //0x30 sends 0, 0x38 sends 1
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);

    pinMode(15, INPUT);
    gpio_matrix_in(15/*XCLK*/, I2S0I_WS_IN_IDX, false);
    
    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);

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
       
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    // FIFO configuration
    //two bytes per dword packing
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    //start i2s
    I2S0.rx_eof_num = 0xFFFFFFFF;
    I2S0.in_link.addr = (uint32_t)&dma_descriptor[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;

    //start i2s + dma
    I2S0.conf.rx_start = 1;

    delay(10000);

    //stop i2s + dma
    I2S0.conf.rx_start = 0;
    
    for(uint8_t i = 0; i<sizeof(dma_buff[0])/2; i++) {
      Serial.println(*(((uint16_t *)&dma_buff[0][0]) + i));
      Serial.print(' ');
      delay(10);
    }
}

void loop() {
  // put your main code here, to run repeatedly:

}
```cpp

## Other items : 
The attached picture shows an irregular graph instead of an expected even ramp signal.

==========================================================================================

// We will want to run i2s as input for sigrok
static void i2s_init()
{
    camera_config_t* config = &s_state->config;

    // Configure input GPIOs
    gpio_num_t pins[] = {
            config->pin_d7,
            config->pin_d6,
            config->pin_d5,
            config->pin_d4,
            config->pin_d3,
            config->pin_d2,
            config->pin_d1,
            config->pin_d0,
            config->pin_vsync,
            config->pin_href,
            config->pin_pclk
    };
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(config->pin_href, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
    // Toggle some reset bits in CONF register
    i2s_conf_reset();
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
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
            &i2s_isr, NULL, &s_state->i2s_intr_handle);
}

static void IRAM_ATTR signal_dma_buf_received(bool* need_yield)
{
    size_t dma_desc_filled = s_state->dma_desc_cur;
    s_state->dma_desc_cur = (dma_desc_filled + 1) % s_state->dma_desc_count;
    s_state->dma_received_count++;
    BaseType_t higher_priority_task_woken;
    BaseType_t ret = xQueueSendFromISR(s_state->data_ready, &dma_desc_filled, &higher_priority_task_woken);
    if (ret != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "queue send failed (%d), dma_received_count=%d", ret, s_state->dma_received_count);
    }
    *need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
}


static void IRAM_ATTR i2s_isr(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;
    bool need_yield;
    signal_dma_buf_received(&need_yield);
    ESP_EARLY_LOGV(TAG, "isr, cnt=%d", s_state->dma_received_count);
    if (s_state->dma_received_count == s_state->height * s_state->dma_per_line) {
        i2s_stop();
    }
    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

#endif
