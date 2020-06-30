/*
 *
 * Copyright (C) 2014-2015 Benjamin VERNOUX
 * Copyright (C) 2015 Nicolas OBERLI
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <stdio.h>
#include "esp32_sump.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "driver/gpio.h"
#include "read_chars.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "freertos/ringbuf.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "app-config.h"

#ifdef SUMP_OVER_UART

#define STATES_LEN 8192

const char *tim_name="samp_tim";
esp_timer_handle_t   sample_timer;


//static uint16_t *buffer = (uint16_t *)g_sbuf;
//static uint16_t *buffer;
uint8_t *buffer;

static uint16_t INDEX = 0;
//static TIM_HandleTypeDef htim;
static sump_config sconfig;

// Event interrupt handling to get a steady flow of samples

/*
 * A sample structure to pass samples
 * from the timer handler to the main program.
 */
typedef struct {
	uint16_t        sample;
} timer_event_t;

xQueueHandle timer_queue;

void sump_debug(char *str,unsigned int value);


static int level=1;
	
//    xQueueSendFromISR(timer_queue, &evt, NULL);


uint16_t getSample() {
	// 0x3FF4403C, GPIO_IN_REG
	// TODO, there should be a more direct way to read this

    uint16_t ret=0;
	ret=gpio_get_level(17) | 
	        (gpio_get_level(13) << 1) |  
			(gpio_get_level(14) << 2) |
			(gpio_get_level(15) << 3);

	return (ret);
			// ||
			//(gpio_get_level(16) << 4));
}


static void portc_init(void)
{
	    gpio_num_t pins[] = {
			// JTAG, on the wrover kit
            12,
            13,
            14,
            15,
			// Other
            //16,
            //17
            //18,
            //19,
            //20,
            //21,
            //22,
            //23,
            //24,
            //25,
            //26,
            //27
    };
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

	//int level0=gpio_get_level(0);
	//int level16=gpio_get_level(16);

#if 0
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Speed = GPIO_SPEED_HIGH;
	gpio_init.Pull = GPIO_PULLDOWN;
#endif
}


// High res timer
void high_res_timer(void *para) {
	//if (config_state == SUMP_STATE_TRIGGED) {
    //
	//}

	level++;
	if (level<1000) 
	{
		//gpio_set_level(GPIO_NUM_4, 1);
		//gpio_set_level(GPIO_NUM_14, 1);
	} else if (level <2000) {
		//gpio_set_level(GPIO_NUM_4, 0);
		//gpio_set_level(GPIO_NUM_14, 0);
	} else {
		level=0;
	}

	if (sconfig.state !=SUMP_STATE_IDLE) {
		//gpio_set_level(GPIO_NUM_17, 1);

		//static unsigned int go_test=0;

		timer_event_t evt;

		evt.sample=getSample();
		//evt.sample=go_test++;
		xQueueSend( timer_queue, ( void * ) &evt, ( TickType_t ) 0 );

		//gpio_set_level(GPIO_NUM_17, 0);

	}

}



static void tim_init(void)
{
	esp_timer_create_args_t  high_res_args;

	high_res_args.callback=high_res_timer;
	high_res_args.arg=NULL;
	high_res_args.dispatch_method=ESP_TIMER_TASK; // TODO, from ISR??
	high_res_args.name=tim_name;

	if (ESP_OK!=esp_timer_create(&high_res_args,&sample_timer)) {
		printf("!!!!!! Failed to create timer, samp_tim!!!!!\n");
	}
														// 1000 microsec.. Send sample each ms 1kHz
	esp_timer_start_periodic(sample_timer, 100);        // 10 microsec, 100kHz

#if 0
	htim.Instance = TIM4;

	htim.Init.Period = 21 - 1;
	htim.Init.Prescaler = 2*(sconfig.divider) - 1;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;

	HAL_TIM_Base_MspInit(&htim);
	__TIM4_CLK_ENABLE();
	HAL_TIM_Base_Init(&htim);
	TIM4->SR &= ~TIM_SR_UIF;  //clear overflow flag
#endif
}

#if 0
static void tim_set_prescaler(void)
{
	//  f = clock / (x + 1)
    int timer_idx=0;
    timer_config_t config;
    config.divider = 2*(sconfig.divider) - 1;;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;  // Auto reload
    timer_init(TIMER_GROUP_0, timer_idx, &config);
#if 0
	HAL_TIM_Base_DeInit(&htim);
	htim.Init.Prescaler = 2*(config.divider) - 1;
	HAL_TIM_Base_Init(&htim);
#endif
}
#endif
extern void setup_adcSampler(void);


void sump_init(void)
{
    buffer=malloc(STATES_LEN*sizeof(uint16_t));
    timer_queue = xQueueCreate(9000, sizeof(timer_event_t));
	portc_init();
	//setup_adcSampler();
	tim_init();

	sconfig.read_count=100;
}

extern unsigned char* get_dma_samples(); 
extern uint32_t  get_frame_length();

static void get_samples(void)
{
	uint32_t config_state;
	config_state = sconfig.state;


	sump_debug("get_samples=",1);
	/* Lock Kernel for logic analyzer */
	//chSysLock();
#if 0
	// Experimental aquisition with parallell i2s aquisition
    static unsigned int go_test=0;

	unsigned char* data;
	uint32_t ix;
	for (int y=0;y<3;y++) {
		data=get_dma_samples();
		uint32_t length=get_frame_length();
		sump_debug("get_length=",length);
		ix=0;
		unsigned char test=0;
		while(ix<3000) { 
			timer_event_t evt;
			evt.sample=*data;
			//evt.sample=1; // test++;
			data++;
			ix++;
			//evt.sample=go_test++;
			xQueueSend( timer_queue, ( void * ) &evt, ( TickType_t ) 0 );
		}
	}

	sump_debug("config_state=",config_state);
#endif

	//gpio_set_level(GPIO_NUM_2, 1);
	//HAL_TIM_Base_Start(&htim);

	if(config_state == SUMP_STATE_ARMED)
	{
		uint32_t config_trigger_value;
		uint32_t config_trigger_mask;

		config_trigger_value =  sconfig.trigger_values[0];
		config_trigger_mask = sconfig.trigger_masks[0];

		

		while(1)
		{
			// Wait for timer
			timer_event_t        evt;
            xQueueReceive(timer_queue, &evt, portMAX_DELAY);

			*(buffer+INDEX) = evt.sample;
			//sump_debug("val=",evt.sample);

			//if (INDEX>8700) {
			//	config_state = SUMP_STATE_TRIGGED;
			//	break;
			//}
			//*(buffer+INDEX) = GPIOC->IDR;
			//TIM4->SR &= ~TIM_SR_UIF;  //clear overflow flag
			if ( !((*(buffer+INDEX) ^ config_trigger_value) & config_trigger_mask) ) {
				config_state = SUMP_STATE_TRIGGED;
				sump_debug("trig=",INDEX);
				break;
			}
			INDEX++;
			INDEX &= STATES_LEN-1;
		}
	}

	if(config_state == SUMP_STATE_TRIGGED)
	{
		register uint32_t config_delay_count = sconfig.delay_count;

		while(config_delay_count > 0)
		{
			timer_event_t        evt;
            xQueueReceive(timer_queue, &evt, portMAX_DELAY);

			*(buffer+INDEX) = evt.sample;

			//*(buffer+INDEX) = GPIOC->IDR;
			//TIM4->SR &= ~TIM_SR_UIF;  //clear overflow flag
			config_delay_count--;
			INDEX++;
			INDEX &= STATES_LEN-1;
		}
	}
	//gpio_set_level(GPIO_NUM_4, 0);
	//gpio_set_level(GPIO_NUM_2, 0);

	//chSysUnlock();
	sconfig.state = SUMP_STATE_IDLE;
	//HAL_TIM_Base_Stop(&htim);
}

static void sump_deinit(void)
{
	//GPIO_TypeDef *hal_gpio_port;
	//hal_gpio_port =(GPIO_TypeDef*)GPIOC;
	uint8_t gpio_pin;

	//HAL_TIM_Base_Stop(&htim);
	//HAL_TIM_Base_DeInit(&htim);
	//__TIM4_CLK_DISABLE();
	for(gpio_pin=0; gpio_pin<15; gpio_pin++) {
		//HAL_GPIO_DeInit(hal_gpio_port, 1 << gpio_pin);
	}
}


char reply[10];

int uart_read_timeout(sump_context_t * context,unsigned char * data,unsigned int len, size_t timeout)
{
	size_t size;
	int num_read=0;
	unsigned char *ptr = data;
	//printf("+");

	{
	    size = uart_read_bytes(0, (unsigned char *)ptr, 1, timeout); // portMAX_DELAY);
		//printf("%c\n",*ptr);
		if (size == 1) {
			num_read++;
			ptr++;
			return num_read;
		} else {
			//printf(".");
            return num_read;
		}						
		// End of read a character
	} // End of loop
	return num_read;
};

int uart_read_chars (sump_context_t * context,  unsigned char * data,unsigned int len)
{
	int size;
	int ret=len;
	unsigned char *ptr = data;
	while(len>0) {
		size = uart_read_bytes(context->uart_portno, ptr, 1, portMAX_DELAY);
		if (size >0) {
			len-=size;
			ptr+=size;
		} // End of read a character
	} // End of loop
	return (ret);
};

int uart_write (sump_context_t * context, const char * data, size_t len)
{
	int  num_written= uart_write_bytes(context->uart_portno, data ,len);
	if (num_written<len) {

	}
	return num_written;
};

int uart_error_callback(sump_context_t * context, int_fast16_t error) {
	// Nothing here yet!!
	return 0;
};

int uart_sump_flush(sump_context_t * context) {
  uart_flush(context->uart_portno);
  return 0;
}

sump_interface_t uart_interface = {
    .error = uart_error_callback,
    .write = uart_write,
	.read_timeout=uart_read_timeout,
	.read_chars=uart_read_chars,
	.flush=uart_sump_flush,
};




void sump(sump_context_t *context,sump_interface_t *io)
{

	//sump_init();
	sconfig.state = SUMP_STATE_IDLE;

	unsigned char  sump_command;
	uint8_t sump_parameters[4] = {0};
	uint32_t index=0;
	int size;
	//printf("SUMP ENTER\n");
	io->flush(context);
	
	// Clear buffer
    while (io->read_timeout(context, &sump_parameters, 4, 10/portTICK_PERIOD_MS)>0){

	}

	// !(gpio_get_level(0)==0)
	while (true) {
		if(io->read_timeout(context, &sump_command, 1, 500/portTICK_PERIOD_MS)) {
		    sump_debug("cmd=",sump_command);

			switch(sump_command) {
			case SUMP_RESET:			
				break;
			case SUMP_ID:
				reply[0]='1';
				reply[1]='A';
				reply[2]='L';
				reply[3]='S';
			    size = io->write(context, (const char *)reply, 4);
				io->flush(context);

				//printf( "1ALS");
				break;
			case SUMP_RUN:
				INDEX=0;
				xQueueReset(timer_queue);
				sconfig.state = SUMP_STATE_ARMED;
				get_samples();

				while(sconfig.read_count > 0) {
					if (INDEX == 0) {
						INDEX = STATES_LEN-1;
					} else {
						INDEX--;
					}
					reply[0]=*(buffer+INDEX) & 0xff;
					/*					
					// TDOO, fix this sconfig.channels stuff

					switch (sconfig.channels) {	
					case 1:
						reply[0]=*(buffer+INDEX) & 0xff;				
				        reply[1]=0x00;
				        reply[2]=0x00;
				        reply[3]=0x00;
						//sprintf( reply,"%c%c%c%c", *(buffer+INDEX) & 0xff,zero,zero,zero);
						break;
					case 2:
						//sprintf( reply,"%c%c%c%c", (*(buffer+INDEX) & 0xff00)>>8,zero,zero,zero);
						reply[0]=(*(buffer+INDEX) & 0xff00)>>8;				
				        reply[1]=0x00;
				        reply[2]=0x00;
				        reply[3]=0x00;
						break;
					case 3:
						reply[0]=*(buffer+INDEX) & 0xff;				
				        reply[1]=(*(buffer+INDEX) & 0xff00)>>8;
				        reply[2]=0x00;
				        reply[3]=0x00;
						//sprintf( reply,"%c%c%c%c", *(buffer+INDEX) & 0xff, (*(buffer+INDEX) & 0xff00)>>8,zero,zero);
						break;
					}
					*/
					//printf("%s",reply);
					size = io->write(context, (const char *)reply, 1);  // WAS 4
					io->flush(context);
					sconfig.read_count--;
				}
				break;
			case SUMP_DESC:
				// device name string
				//sprintf( reply,"%c%s%c", 0x01,"Esp32",0x00);
				reply[0]=0x01;								
				reply[1]='E';
				reply[2]='S';
				reply[3]='P';
				reply[4]='3';
				reply[5]='2';
				reply[6]=0x0;
				size = io->write(context, (const char *)reply, 7);

				// Usable probes -- 8
				reply[0]=0x20;								
				reply[1]=0x00;
				reply[2]=0x00;
				reply[3]=0x00;
				reply[4]=0x08;
				size = io->write(context, (const char *)reply, 5);


				//sample memory (8192)
				reply[0]=0x21;								
				reply[1]=0x00;
				reply[2]=0x00;
				reply[3]=0x20;
				reply[4]=0x00;
				size = io->write(context, (const char *)reply, 5);
				//sample rate (2MHz)
				//reply[0]=0x23;								
				//reply[1]=0x00;
				//reply[2]=0x1E;
				//reply[3]=0x84;
				//reply[4]=0x80;
			    //sample rate (10kHz)
				reply[0]=0x23;								
				reply[1]=0x00;
				reply[2]=0x00;
				reply[3]=0x27;
				reply[4]=0x10;
				size = io->write(context, (const char *)reply, 5);

				//number of probes (8)
				//reply[0]=0x40;								
				//reply[1]=0x08;
				//size = io->write(context, (const char *)reply, 2);
				//protocol version (2)
				reply[0]=0x41;								
				reply[1]=0x02;
				size = io->write(context, (const char *)reply, 2);

				// End
				reply[0]=0x00;
				size = io->write(context, (const char *)reply, 1);
				io->flush(context);
				break;
			case SUMP_XON:
			case SUMP_XOFF:
				/* not implemented */
				break;
			default:
				// Other commands take 4 bytes as parameters
				if(io->read_chars(context, (unsigned char *)sump_parameters, 4) == 4) {
						sump_debug("0=",sump_parameters[0]);
						sump_debug("1=",sump_parameters[1]);
						sump_debug("2=",sump_parameters[2]);
						sump_debug("3=",sump_parameters[3]);

					switch(sump_command) {
					case SUMP_TRIG_1:
					case SUMP_TRIG_2:
					case SUMP_TRIG_3:
					case SUMP_TRIG_4:
						// Get the trigger index
						index = (sump_command & 0x0c) >> 2;
						sconfig.trigger_masks[index] = sump_parameters[3];
						sconfig.trigger_masks[index] <<= 8;
						sconfig.trigger_masks[index] |= sump_parameters[2];
						sconfig.trigger_masks[index] <<= 8;
						sconfig.trigger_masks[index] |= sump_parameters[1];
						sconfig.trigger_masks[index] <<= 8;
						sconfig.trigger_masks[index] |= sump_parameters[0];

						sump_debug("trigger_masks[]=",sconfig.trigger_masks[index]);

						break;
					case SUMP_TRIG_VALS_1:
					case SUMP_TRIG_VALS_2:
					case SUMP_TRIG_VALS_3:
					case SUMP_TRIG_VALS_4:
						// Get the trigger index
						index = (sump_command & 0x0c) >> 2;
						sconfig.trigger_values[index] = sump_parameters[3];
						sconfig.trigger_values[index] <<= 8;
						sconfig.trigger_values[index] |= sump_parameters[2];
						sconfig.trigger_values[index] <<= 8;
						sconfig.trigger_values[index] |= sump_parameters[1];
						sconfig.trigger_values[index] <<= 8;
						sconfig.trigger_values[index] |= sump_parameters[0];
						sump_debug("trigger_values[]=",sconfig.trigger_values[index]);

						break;
					case SUMP_CNT:
						sconfig.delay_count = sump_parameters[3];
						sconfig.delay_count <<= 8;
						sconfig.delay_count |= sump_parameters[2];
						sconfig.delay_count <<= 2; /* values are multiples of 4 */
						sconfig.read_count = sump_parameters[1];
						sconfig.read_count <<= 8;
						sconfig.read_count |= sump_parameters[0];
						sconfig.read_count++;
						sconfig.read_count <<= 2; /* values are multiples of 4 */
						if (sconfig.read_count>STATES_LEN-1) {
							sconfig.read_count=STATES_LEN-1;
						}
						if (sconfig.delay_count>STATES_LEN-1) {
							sconfig.delay_count=STATES_LEN-1;
						}


						sump_debug("read_count=",sconfig.read_count);
						sump_debug("delay_count=",sconfig.delay_count);
						break;
					case SUMP_DIV:
						sconfig.divider = sump_parameters[2];
						sconfig.divider <<= 8;
						sconfig.divider |= sump_parameters[1];
						sconfig.divider <<= 8;
						sconfig.divider |= sump_parameters[0];
						sconfig.divider /= 50; /* Assuming 100MHz base frequency */
						sconfig.divider++;
						sump_debug("divider=",sconfig.read_count);
						unsigned int freq=1000000/(1+sconfig.divider);
						sump_debug("freq=",freq);

						//esp_timer_stop(sample_timer);
						//esp_timer_start_periodic(sample_timer, 1000);
						//  the sampling frequency is set to f = clock / (x + 1)
						// Dont do this yet
						//tim_set_prescaler();
						break;
					case SUMP_FLAGS:
						sconfig.channels = (~sump_parameters[0] >> 2) & 0x0f;
						/* not implemented */
						break;
					default:
						break;
					}
				}
				break;
			}
		}
	}
	//printf("SUMP EXIT\n");
	sump_deinit();
}




void sump_uart() {
  sump_context_t context;
  context.uart_portno=0;
  sump(&context,&uart_interface);
}

#endif
