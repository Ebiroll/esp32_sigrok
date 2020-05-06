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
#include "esp_log.h"
#include "nvs_flash.h"
#include "collector.h"
//#include "esp_netif.h"
#if 0

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

#define RECEIVER_PORT_NUM 6001
#define SENDER_PORT_NUM 6000

uint16_t trigger = 0;

int trig_pin;
// #define NUM_SAMPLES 14000

static int maxSamples=NUM_SAMPLES;
int stop_at_desc=-1;
unsigned int logicIndex = 0;
unsigned int triggerIndex = 0;
uint32_t readCount = CAPTURE_SIZE;
unsigned int delayCount = 0;
uint16_t trigger_values = 0;
unsigned int useMicro = 0;
unsigned int delayTime = 0;
unsigned long divider = 0;
uint32_t clock_per_read = 0;

camera_state_t *s_state;

static i2s_parallel_state_t* i2s_state[2] = {NULL, NULL};


static void IRAM_ATTR i2s_trigger_isr(void) {
  //I2S0.conf.rx_start = 1;
}

void dma_serializer( dma_elem_t *dma_buffer ){
  for ( int i = 0 ; i < s_state->dma_buf_width/4 ; i++ ){
     uint8_t y =  dma_buffer[i].sample2;
     dma_buffer[i].sample2 = dma_buffer[i].sample1;
     dma_buffer[i].sample1 = y;
   }
}

i2s_parallel_buffer_desc_t bufdesc;

uint16_t* get_digital_values() {
  return((uint16_t*)s_state->dma_buf[0]);
  //return((uint16_t*)bufdesc.memory);
}


uint8_t* get_values() {
  return((uint16_t*)s_state->dma_buf[0]);
}

// This task sends data
#if 0
static void udp_sender(void *pvParameters)
{
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    // This will stress all clients on your wifi network
    #define RECEIVER_IP_ADDR "255.255.255.255"

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(RECEIVER_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(RECEIVER_PORT_NUM);
        addr_family = PF_INET;
        ip_protocol = IPPROTO_IP;

        //struct sockaddr_in6 dest_addr = { 0 };
        //ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));



        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);


        //#define sendto(s,dataptr,size,flags,to,tolen)     lwip_sendto_r(s,dataptr,size,flags,to,tolen)
        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
/*
Raw/UDP

struct udp_pcb * udp_new(void)
Creates a new UDP pcb which can be used for UDP communication. The pcb is not active until it has either been bound to a local address or connected to a remote address.

void udp_remove(struct udp_pcb * pcb)
Removes and deallocates the pcb.

err_t udp_bind(struct udp_pcb * pcb, struct ip_addr * ipaddr, u16_t port)
Binds the pcb to a local address. The IP-address argument "ipaddr" can be IP_ADDR_ANY to indicate that it should listen to any local IP address. The function returns ERR_USE if the specified port is already in use, otherwise ERR_OK.

err_t udp_connect(struct udp_pcb * pcb, struct ip_addr * ipaddr, u16_t port)
Sets the remote end of the pcb. This function does not generate any network traffic, but only set the remote address of the pcb. It binds the pcb to a local address if it is not already bound. It returns ERR_USE if no port is available, ERR_RTE if there is no route to the destination, or ERR_OK. Connecting is only needed when using udp_send(). For unconnected pcbs, udp_sendto() can be used to send to any specified remote address. Connected pcbs only receive data from the connected remote address, while unconnected pcbs receive datagrams from any remote address.

void udp_disconnect(struct udp_pcb * pcb)
Remove the remote end of the pcb. This function does not generate any network traffic, but only removes the remote address of the pcb.

err_t udp_send(struct udp_pcb * pcb, struct pbuf * p)
Sends the pbuf p to the remote address set using udp_connect(). The pbuf is not deallocated.

err_t udp_sendto(struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *dst_ip, u16_t dst_port);
Same as udp_send(), but sends to any remote address.

void udp_recv(struct udp_pcb * pcb,
               void (* recv)(void * arg, struct udp_pcb * upcb,
                                         struct pbuf * p,
                                         struct ip_addr * addr,
                                         u16_t port),
               void * recv_arg)
Specifies a callback function that should be called when a UDP datagram is received on the specified connection. The callback function is responsible for deallocating the pbuf.
*/
#endif 
/*
This example shows how to send some UDP data to UDP port 7000 on a remote host with IP address 10.0.0.1.
Send the data in the netbuf buf on the UDP connection conn. The data in the netbuf should not be too large if IP fragmentation support is disabled. If IP fragmentation support is disabled, the data should not be larger than the maximum transmission unit (MTU) of the outgoing network interface, less the space required for link layer, IP and UDP headers. No checking is necessarily made of whether the data is sufficiently small and sending very large netbufs might give undefined results.
int
main()
{
    struct netconn *conn;
    struct netbuf *buf;
    struct ip_addr addr;
    char *data;
    char text[] = "A static text";
    int i;

    // create a new connection 
    conn = netconn_new(NETCONN_UDP);

    // set up the IP address of the remote host 
    addr.addr = htonl(0x0a000001);

    // connect the connection to the remote host 
    netconn_connect(conn, addr, 7000);

    // create a new netbuf 
    buf = netbuf_new();
    data = netbuf_alloc(buf, 10);

    // create some arbitrary data 
    for(i = 0; i < 10; i++)
        data[i] = i;

    // send the arbitrary data 
    netconn_send(conn, buf);

    // reference the text into the netbuf 
    netbuf_ref(buf, text, sizeof(text));

    // send the text 
    netconn_send(conn, buf);

    // deallocate connection and netbuf 
    netconn_delete(conn);
    netconn_delete(buf);
}
*/

// We preallocate
// Start sending received buffers as fast as possible

void initUDP() {
 /* Start sending like a madlad */
 //xTaskCreate(udp_sender, "udp_sender", 4096, NULL, 5, NULL);
}

void initNormal() {


}

void set_mem_depth(int depth) {
    if (depth<NUM_SAMPLES) {
        maxSamples=depth;
    }
}


int stop_aq=false;
void stop_aquisition() {
    stop_aq=true;
}


void setupDelay(float divider) {
  double rate = 100000000.0 / (divider + 1.0);
  enable_out_clock((int)rate);
  printf("Capture Speed : %.2f Mhz\r\n", rate/1000000.0);
}

void setTimescale(float scale){
  setupDelay(scale);
  printf("setTimescale=%.3f\n",scale);

}

void captureData() {
  uint32_t a, b, c, d;
  /*
  printf("FreeHeap         :%u\r\n", getFreeHeap());
  printf("FreeHeap 64 Byte :%u\r\n", heap_caps_get_largest_free_block(64) );
  printf("Triger Values 0x%X\r\n", trigger_values);
  printf("Triger        0x%X\r\n", trigger);
  printf("Running on CORE #%d\r\n", xPortGetCoreID());
  printf("Reading %d Samples\r\n", readCount);
  */
  gpio_set_level(ledPin, 1);

  ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);
  start_dma_capture();

  while (! s_state->dma_done ) 
    vTaskDelay(100 / portTICK_PERIOD_MS);
		

  taskYIELD();

  gpio_set_level(ledPin, 0);


  printf("ReadCount:  %d\r\n",readCount);
  printf("DMA Desc Current: %d\r\n",  s_state->dma_desc_cur);

  ESP_LOGD(TAG, "Copying buffer.");

  int filled_desc = ((readCount/2) / s_state->dma_sample_per_desc);
  int filled_sample_offset = ((readCount/2) % s_state->dma_sample_per_desc); //((readCount - 1) % s_state->dma_val_per_desc) % s_state->dma_sample_per_desc;
  int filled_full_sample_offset = s_state->dma_sample_per_desc;
  
  int tx_count = 0;
  printf("used_desc = %d\r\n", filled_desc);
  printf("used_sample_offset = %d\r\n", filled_sample_offset);  
  printf( "\r\nDone\r\n" );
  //flush();
  filled_desc--;
  filled_full_sample_offset--;
  if( filled_sample_offset-- == 0)
    filled_sample_offset = filled_full_sample_offset;


/*
  dma_elem_t cur;

  printf("\r\nRAW BlocX \r\n");
  for ( int i = 0 ; i < 100 ; i++ ){
     cur = (dma_elem_t&)s_state->dma_buf[0][i];
     printf("0x%X, ", cur.sample2);
     printf("0x%X, ", cur.sample1);
   }

  printf("\r\nRAW Block InpuX:\r\n");
  for ( int i = 0 ; i < 400 ; i+=2 ){
     uint8_t *crx = (uint8_t*)s_state->dma_buf[0];
     printf("0x%X, ", *(crx+i) );
     }
   println();
*/


  if(s_state->dma_desc_triggered < 0){ //if not triggered mode,
    s_state->dma_desc_triggered=0;  //first desc is 0
      ESP_LOGD(TAG, "Normal TX");
    }
  else
  {
    ESP_LOGD(TAG, "Triggered TX");
  }

/*
    for(int i=0; i<32 ; i++){
     printf("Processing DMA Desc: %d", i);
     fast_rle_block_encode_asm( (uint8_t*)s_state->dma_buf[i], s_state->dma_buf_width);
     if( rle_buff_p-rle_buff > rle_size - 4000 )
      break;
     }
*/   

//brexit:
  ESP_LOGD(TAG, "End. TX: %d", tx_count);
  gpio_set_level(ledPin, 0);
}


void start_sampling(){
   captureData();
}

bool samples_finnished() 
{
  return true;
}


void start_dma_capture(void) {
  s_state->dma_done = false;
  s_state->dma_desc_cur = 0;
  s_state->dma_received_count = 0;
  s_state->dma_filtered_count = 0;
  s_state->dma_desc_triggered = 0;

  ESP_ERROR_CHECK(esp_intr_disable(s_state->i2s_intr_handle));
  i2s_conf_reset();

  I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
  I2S0.in_link.start = 1;
  I2S0.int_clr.val = I2S0.int_raw.val;

  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;

  ESP_LOGD(TAG, "DMA Tigger : 0x%X", trigger );
  if (trigger ) {
    stop_at_desc = -1;
    I2S0.rx_eof_num = s_state->dma_buf_width;
    I2S0.int_ena.in_suc_eof = 0;
    I2S0.int_ena.rx_take_data = 0;
    lldesc_t* pd = &s_state->dma_desc[s_state->dma_desc_count - 1];
    pd->eof = 0;
    pd->qe.stqe_next = &s_state->dma_desc[0];
  }
  else {
    s_state->dma_desc_triggered=-1;
    I2S0.rx_eof_num = readCount/2;
    I2S0.int_ena.in_suc_eof = 1;
    I2S0.int_ena.rx_take_data = 1;
    lldesc_t* pd = &s_state->dma_desc[s_state->dma_desc_count - 1];
    pd->eof = 1;
    pd->qe.stqe_next = 0x0;
  }

  //clear dma buffers
  for (int i = 0; i < s_state->dma_desc_count; ++i)
    memset( s_state->dma_buf[i], 0, s_state->dma_desc[i].length);

  //Enable the Interrupt
  ESP_ERROR_CHECK(esp_intr_enable(s_state->i2s_intr_handle));

  //attachInterrupt(0, i2s_trigger_isr, RISING);
  I2S0.conf.rx_start = 1;
}

static esp_err_t dma_desc_init(int raw_byte_size){
    s_state = (camera_state_t*) malloc (sizeof(camera_state_t));
    assert(raw_byte_size % 4 == 0);

    ESP_LOGD(TAG, "Buffer Total (for DMA): %d bytes", raw_byte_size);
    size_t dma_desc_count = 1;
    size_t buf_size = raw_byte_size;
    /*
    while (buf_size >= 4096)
    {
        buf_size /= 2;
        dma_desc_count *= 2;
    }
    s_state->dma_buf_width = buf_size;
    s_state->dma_val_per_desc = buf_size/2;
    s_state->dma_sample_per_desc = buf_size/4;
    s_state->dma_desc_count = dma_desc_count;
*/
    s_state->dma_buf_width = buf_size = 4000;
    s_state->dma_val_per_desc = 2000;
    s_state->dma_sample_per_desc = 1000;
    s_state->dma_desc_count = dma_desc_count = raw_byte_size/4000;
    
    ESP_LOGD(TAG, "DMA buffer size: %d", buf_size);
    ESP_LOGD(TAG, "DMA buffer count: %d", dma_desc_count);
    ESP_LOGD(TAG, "DMA buffer total: %d bytes", buf_size * dma_desc_count);
    
    s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
    if (s_state->dma_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
    if (s_state->dma_desc == NULL) {
        return ESP_ERR_NO_MEM;
    }
    size_t dma_sample_count = 0;
    for (int i = 0; i < dma_desc_count; ++i) {
        ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
        if (buf == NULL) {
            ESP_LOGD(TAG, "NO_MEM!");  
            return ESP_ERR_NO_MEM;
        }
        s_state->dma_buf[i] = buf;
        ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);

        lldesc_t* pd = &s_state->dma_desc[i];
        pd->length = buf_size/2;
        dma_sample_count += buf_size / 2; // indeed /4 because each sample is 4 bytes
        pd->size = buf_size;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = (uint8_t*) buf;
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 0;
        pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
        if( i+1 == dma_desc_count ){
           pd->eof = 1;
           pd->qe.stqe_next = 0x0;
           //pd->eof = 0;
           //pd->qe.stqe_next = &s_state->dma_desc[0];
          }
    }
    s_state->dma_done = false;
    s_state->dma_sample_count = dma_sample_count;
    ESP_LOGD(TAG, "DMA dma_sample_count: %d", dma_sample_count);
    return ESP_OK;
}

static void gpio_setup_in(int gpio, int sig, int inv) {
  if (gpio == -1) return;
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
  gpio_set_direction( (gpio_num_t)gpio, (gpio_mode_t)GPIO_MODE_DEF_INPUT);
  //gpio_matrix_in(gpio, sig, inv, false);
  gpio_matrix_in(gpio, sig, false);
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
  while (I2S0.state.rx_fifo_reset_back) {}
  }

/*
static void IRAM_ATTR iggr_isr(void* arg)
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
*/

static void IRAM_ATTR i2s_isr(void* arg) {
  if(trigger==0){
    ESP_LOGD(TAG, "DMA INT Number %d Status 0x%x", s_state->dma_desc_cur, I2S0.int_raw.val );
  
     /*
     ESP_LOGD(TAG, "DMA INT take_data? %d", I2S0.int_raw.rx_take_data );
     ESP_LOGD(TAG, "DMA INT in_dscr_empty? %d", I2S0.int_raw.in_dscr_empty );
     ESP_LOGD(TAG, "DMA INT in_done? %d", I2S0.int_raw.in_done );
     ESP_LOGD(TAG, "DMA INT in_suc_eof? %d", I2S0.int_raw.in_suc_eof );
     ESP_LOGD(TAG, "DMA INT rx_rempty? %d", I2S0.int_raw.rx_rempty );
     ESP_LOGD( "\r\n" );
     */
  
    //ESP_LOGD(TAG, "Executing DMA ISR on core %d", xPortGetCoreID() );
  }
  
  //gpio_set_level(, 1); //Should show a pulse on the logic analyzer when an interrupt occurs
  //gpio_set_level(, 0);
  if(I2S0.int_raw.in_done){ //filled desc
        
    if (stop_at_desc==-1){
        ESP_LOGD(TAG, "DMA INT Number %d Status 0x%x ", s_state->dma_desc_cur, I2S0.int_raw.val);


        stop_at_desc= (s_state->dma_desc_cur + (readCount / (s_state->dma_buf_width/2))-1) % s_state->dma_desc_count;
        ESP_LOGD(TAG, "DMA BREAK! Stop at desc %d", stop_at_desc);
        
        s_state->dma_desc_triggered = s_state->dma_desc_cur;
        I2S0.rx_eof_num = readCount/2 ;//- s_state->dma_buf_width/4000; // Total capacity - desc capacity
        I2S0.int_ena.in_suc_eof = 1;
        }        
        trigger=0;
        I2S0.int_clr.val = I2S0.int_raw.val;
       s_state->dma_desc_cur++;
    }

    
    //s_state->dma_desc_cur=0;
    esp_intr_disable(s_state->i2s_intr_handle);
    i2s_conf_reset();
    I2S0.conf.rx_start = 0;
    s_state->dma_done = true;
 
     I2S0.int_clr.val = I2S0.int_raw.val;   
  }


void i2s_parallel_setup(const i2s_parallel_config_t *cfg) {

  //Figure out which signal numbers to use for routing
  ESP_LOGI(TAG, "Setting up parallel I2S bus at I2S%d\n", 0);
  int sig_data_base, sig_clk;

  sig_data_base = I2S0I_DATA_IN0_IDX;
  sig_clk       = I2S0I_WS_IN_IDX;
  //sig_clk       = I2S0I_BCK_IN_IDX;

  //Route the signals
  gpio_setup_in(cfg->gpio_bus[0], sig_data_base + 0, false); // D0
  gpio_setup_in(cfg->gpio_bus[1], sig_data_base + 1, false); // D1
  gpio_setup_in(cfg->gpio_bus[2], sig_data_base + 2, false); // D2
  gpio_setup_in(cfg->gpio_bus[3], sig_data_base + 3, false); // D3
  gpio_setup_in(cfg->gpio_bus[4], sig_data_base + 4, false); // HS
  gpio_setup_in(cfg->gpio_bus[5], sig_data_base + 5, false); // VS
  gpio_setup_in(cfg->gpio_bus[6], sig_data_base + 6, false); // VS
  gpio_setup_in(cfg->gpio_bus[7], sig_data_base + 7, false); // VS

  gpio_setup_in(cfg->gpio_bus[8], sig_data_base + 8, false); // VS
  gpio_setup_in(cfg->gpio_bus[9], sig_data_base + 9, false); // VS
  gpio_setup_in(cfg->gpio_bus[10], sig_data_base + 10, false); // VS
  gpio_setup_in(cfg->gpio_bus[11], sig_data_base + 11, false); // VS
  gpio_setup_in(cfg->gpio_bus[12], sig_data_base + 12, false); // VS
  gpio_setup_in(cfg->gpio_bus[13], sig_data_base + 13, false); // VS
  gpio_setup_in(cfg->gpio_bus[14], sig_data_base + 14, false); // VS
  gpio_setup_in(cfg->gpio_bus[15], sig_data_base + 15, false); // VS

  gpio_setup_in(cfg->gpio_clk, sig_clk, false);
//gpio_matrix_in(0x38,    I2S0I_WS_IN_IDX, false);

  gpio_matrix_in(0x38,    I2S0I_V_SYNC_IDX, false);
  gpio_matrix_in(0x38,    I2S0I_H_SYNC_IDX, false);
  gpio_matrix_in(0x38,    I2S0I_H_ENABLE_IDX, false);

  // Enable and configure I2S peripheral
  periph_module_enable(PERIPH_I2S0_MODULE);
  
  
  
  //Initialize I2S dev
  // Toggle some reset bits in LC_CONF register
  // Toggle some reset bits in CONF register
  i2s_conf_reset();
    
  // Enable slave mode (sampling clock is external)
  I2S0.conf.rx_slave_mod = 1;

  //Enable LCD mode
  I2S0.conf2.val = 0;

  // Enable parallel mode
  I2S0.conf2.lcd_en = 1;
  
  // Use HSYNC/VSYNC/HREF to control sampling
  I2S0.conf2.camera_en = 1;

  // f i2s = fpll / (Num + b/a )) where fpll=80Mhz
  // Configure clock divider
  I2S0.clkm_conf.val = 0;

  I2S0.clkm_conf.clka_en = 0;    // select PLL_D2_CLK. Digital Multiplexer that select between APLL_CLK or PLL_D2_CLK.
  //I2S0.clkm_conf.clk_en = 1;
  
  
  I2S0.clkm_conf.clkm_div_a = 1;
  I2S0.clkm_conf.clkm_div_b = 0;
  I2S0.clkm_conf.clkm_div_num = 4;
  
  /*
  I2S0.clkm_conf.clkm_div_a = 3;//  24Mhz
  I2S0.clkm_conf.clkm_div_b = 1;
  I2S0.clkm_conf.clkm_div_num = 3;
  */
  
  // FIFO will sink data to DMA
  I2S0.fifo_conf.dscr_en = 1;
  
  // FIFO configuration
  //I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
  I2S0.fifo_conf.rx_fifo_mod = 1;// SM_0A0B_0C0D = 1,
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
  //dev->conf_chan.val = 0;
  I2S0.conf_chan.rx_chan_mod = 1;
  
  // Clear flags which are used in I2S serial mode
  I2S0.sample_rate_conf.rx_bits_mod = 0;
  I2S0.conf.rx_right_first = 1;
  I2S0.conf.rx_msb_right = 0;
  I2S0.conf.rx_msb_shift = 0;
  I2S0.conf.rx_mono = 1;
  I2S0.conf.rx_short_sync = 1;
//  I2S0.conf.rx_mono = 0;
//  I2S0.conf.rx_short_sync = 0;
  I2S0.timing.val = 0;
//  I2S0.timing.rx_dsync_sw = 1;

  I2S0.sample_rate_conf.val = 0;
  // Clear flags which are used in I2S serial mode
  //I2S0.sample_rate_conf.rx_bits_mod = 8;
  I2S0.sample_rate_conf.rx_bits_mod = 16;
  //dev->sample_rate_conf.rx_bck_div_num = 16; //ToDo: Unsure about what this does...
  I2S0.sample_rate_conf.rx_bck_div_num = 1;  // datasheet says this must be 2 or greater (but 1 seems to work)
  
  // this combination is 20MHz
  //dev->sample_rate_conf.tx_bck_div_num=1;
  //dev->clkm_conf.clkm_div_num=3; // datasheet says this must be 2 or greater (but lower values seem to work)

  //Allocate DMA descriptors
  i2s_state[0] = (i2s_parallel_state_t*)malloc(sizeof(i2s_parallel_state_t));
  i2s_parallel_state_t *st = i2s_state[0];
  
  s_state->dma_done = false;
  s_state->dma_desc_cur = 0;
  s_state->dma_received_count = 0;
  s_state->dma_filtered_count = 0;
  //esp_intr_disable(s_state->i2s_intr_handle);
  // i2s_conf_reset();


  ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);
  I2S0.rx_eof_num = s_state->dma_sample_count;
  I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
  I2S0.in_link.start = 1;
  I2S0.int_clr.val = I2S0.int_raw.val;
  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;

  //Setup I2S DMA Interrupt
  esp_err_t err = esp_intr_alloc( ETS_I2S0_INTR_SOURCE,
                    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                    &i2s_isr,  NULL, &s_state->i2s_intr_handle );
  
  //Enable the Interrupt
  //ESP_ERROR_CHECK(esp_intr_enable(s_state->i2s_intr_handle));
  
//  I2S0.conf.rx_start = 1;
}

void enable_out_clock( int freq_in_hz ) {
    //ledcSetup(0, freq_in_hz, 1);
    //ledcAttachPin(cfg.gpio_clk, 0);
    //ledcWrite( 0, 1);
    //delay(10);
    
    
    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_config_t timer_conf;
    // Name change... timer_conf.bit_num = LEDC_TIMER_1_BIT;
    //timer_conf.timer_num  = LEDC_TIMER_3;
    #if ESP_IDF_VERSION_MAJOR >= 4 && ESP_IDF_VERSION_MINOR >= 1
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    #endif

    //timer_conf.freq_hz = I2S_HZ;
    timer_conf.freq_hz = freq_in_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.timer_sel = LEDC_TIMER_0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 1;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num =  23; //cfg.gpio_clk;  //s_config.pin_xclk; 
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    }
    
}

i2s_parallel_config_t cfg;

// Check this also
// https://github.com/espressif/esp-idf/issues/2251

DMA_ATTR uint32_t dma_buff[2][100];
DMA_ATTR lldesc_t dma_descriptor[2];

void setupSimple(void) {

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
     gpio_set_direction(18, GPIO_MODE_INPUT);
     gpio_matrix_in(18,    I2S0I_DATA_IN0_IDX, false);
    gpio_set_direction(23, GPIO_MODE_INPUT);
    gpio_matrix_in(23,    I2S0I_DATA_IN1_IDX, false);
    gpio_set_direction(19, GPIO_MODE_INPUT);
    gpio_matrix_in(19,    I2S0I_DATA_IN2_IDX, false);
    //for i2s in parallel camera input mode data is receiver only when V_SYNC = H_SYNC = H_ENABLE = 1. We don't use these inputs so simply set them High
    gpio_matrix_in(0x38, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);  //0x30 sends 0, 0x38 sends 1
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);

    gpio_set_direction(15, GPIO_MODE_INPUT);
    gpio_matrix_in(15, I2S0I_WS_IN_IDX, false);  // XCLK
    
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

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //stop i2s + dma
    I2S0.conf.rx_start = 0;
}



void setup(void) {


  //WiFi.mode(WIFI_OFF);
  //btStop();
    
  gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
  dma_desc_init(CAPTURE_SIZE);

  cfg.gpio_bus[0] = 0;
  cfg.gpio_bus[1] = 18; //GPIO01 used for UART 0 RX
  cfg.gpio_bus[2] = 2;
  cfg.gpio_bus[3] = 19; //GPIO03 used for UART 0 TX
  cfg.gpio_bus[4] = 4;
  cfg.gpio_bus[5] = 5;
  cfg.gpio_bus[6] = -1; //GPIO06 used for SCK, bootloop
  cfg.gpio_bus[7] = -1; //GPIO07 used for SDO, bootloop

  cfg.gpio_bus[8] = -1; //GPIO8 used for SDI, bootloop
  cfg.gpio_bus[9] = -1; //GPIO9 lead SW_CPU_RESET on WROOVER module
  cfg.gpio_bus[10] = -1; //GPI10 lead SW_CPU_RESET on WROOVER module
  cfg.gpio_bus[11] = 22; //GPIO11 used for CMD, bootloop
  cfg.gpio_bus[12] = 12;
  cfg.gpio_bus[13] = 13;
  cfg.gpio_bus[14] = 14;
  cfg.gpio_bus[15] = 15;


  cfg.gpio_clk = 23; // Pin23 used for XCK input from LedC
  cfg.bits = I2S_PARALLEL_BITS_16;
  cfg.clkspeed_hz = 2 * 1000 * 1000; //resulting pixel clock = 1MHz
  cfg.buf = &bufdesc;

  //enable_out_clock(I2S_HZ);
  //fill_dma_desc( bufdesc );
  i2s_parallel_setup(&cfg);
}

#endif