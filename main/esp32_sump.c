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

#include <stdint.h>
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

#define STATES_LEN 8192

// Timer variables to use for 
#define TIMER_DIVIDER         8  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (0.001) // sample test interval for the first timer 1kHz

#define TIMER_GROUP  0
#define TIMER_IDX    TIMER_0

//static uint16_t *buffer = (uint16_t *)g_sbuf;
static uint16_t *buffer;
static uint16_t INDEX = 0;
//static TIM_HandleTypeDef htim;
static sump_config sconfig;

// Event interrupt handling to get a steady flow of samples

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;


/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}


/*
 * The main task of this example program
 */
static void timer_example_evt_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        printf("Group[%d], timer[%d] alarm event\n", TIMER_GROUP, evt.timer_idx);

        /* Print the timer values passed by event */
        printf("------- EVENT TIME --------\n");
        print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value(TIMER_GROUP, evt.timer_idx, &task_counter_value);
        print_timer_counter(task_counter_value);
    }
}


void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        TIMERG0.int_clr_timers.t0 = 1;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init(int timer_idx,  double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;  // No auto reload
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

uint16_t getSample() {
	// 0x3FF4403C
	// TODO, there should be a more direct way to read this
	return (gpio_get_level(12) || 
	        (gpio_get_level(13) << 1) ||  
			(gpio_get_level(14) << 2) );

}

static void portc_init(void)
{
	    gpio_num_t pins[] = {
            12,
            13,
            14,
            15,
            16,
            17,
            18,
            19,
            20,
            21,
            22,
            23,
            24,
            25,
            26,
            27
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

	int level0=gpio_get_level(0);
	int level16=gpio_get_level(16);

#if 0
	GPIO_InitTypeDef gpio_init;
	GPIO_TypeDef *hal_gpio_port;
	hal_gpio_port =(GPIO_TypeDef*)GPIOC;

	uint8_t gpio_pin;

	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Speed = GPIO_SPEED_HIGH;
	gpio_init.Pull = GPIO_PULLDOWN;
	gpio_init.Alternate = 0; /* Not used */

	for(gpio_pin=0; gpio_pin<15; gpio_pin++) {
		HAL_GPIO_DeInit(hal_gpio_port, 1 << gpio_pin);
		gpio_init.Pin = 1 << gpio_pin;
		HAL_GPIO_Init(hal_gpio_port, &gpio_init);
	}
#endif
}

static void tim_init(void)
{
	example_tg0_timer_init(TIMER_0, TIMER_INTERVAL0_SEC);
	// Test the timer events
	xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
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
    config.auto_reload = 1;  // No auto reload
    timer_init(TIMER_GROUP_0, timer_idx, &config);
#if 0
	HAL_TIM_Base_DeInit(&htim);
	htim.Init.Prescaler = 2*(config.divider) - 1;
	HAL_TIM_Base_Init(&htim);
#endif
}

static void sump_init(void)
{
    buffer=malloc(STATES_LEN*sizeof(uint16_t));
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
	portc_init();
	tim_init();
}

static void get_samples(void)
{
	uint32_t config_state;
	config_state = sconfig.state;

	/* Lock Kernel for logic analyzer */
	//chSysLock();

	//HAL_TIM_Base_Start(&htim);

	if(config_state == SUMP_STATE_ARMED)
	{
		uint32_t config_trigger_value;
		uint32_t config_trigger_mask;

		config_trigger_value =  sconfig.trigger_values[0];
		config_trigger_mask = sconfig.trigger_masks[0];

		while(1)
		{
			//while( !(TIM4->SR & TIM_SR_UIF)) {
				//Wait for timer...
			//}
			timer_event_t evt;
            xQueueReceive(timer_queue, &evt, portMAX_DELAY);

			*(buffer+INDEX) = getSample();
			//*(buffer+INDEX) = GPIOC->IDR;
			//TIM4->SR &= ~TIM_SR_UIF;  //clear overflow flag
			if ( !((*(buffer+INDEX) ^ config_trigger_value) & config_trigger_mask) ) {
				config_state = SUMP_STATE_TRIGGED;
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
			//while( !(TIM4->SR & TIM_SR_UIF)) {
				//Wait for timer...
			//}
			timer_event_t evt;
            xQueueReceive(timer_queue, &evt, portMAX_DELAY);

			*(buffer+INDEX) = getSample();

			//*(buffer+INDEX) = GPIOC->IDR;
			//TIM4->SR &= ~TIM_SR_UIF;  //clear overflow flag
			config_delay_count--;
			INDEX++;
			INDEX &= STATES_LEN-1;
		}
	}

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

int cmd_sump()
{
	printf( "Interrupt by pressing boot.\r\n");
	printf( "\r\n");

	sump();

	return 1;
}

void sump()
{

	sump_init();
	sconfig.state = SUMP_STATE_IDLE;

	uint8_t sump_command;
	uint8_t sump_parameters[4] = {0};
	uint32_t index=0;

	while (!(gpio_get_level(0)==1)) {
		if(char_read_timeout( &sump_command, 1, 1)) {
			switch(sump_command) {
			case SUMP_RESET:
				break;
			case SUMP_ID:
				printf( "1ALS");
				break;
			case SUMP_RUN:
				INDEX=0;
				sconfig.state = SUMP_STATE_ARMED;
				get_samples();

				while(sconfig.read_count > 0) {
					char zero=0;
					if (INDEX == 0) {
						INDEX = STATES_LEN-1;
					} else {
						INDEX--;
					}
					switch (sconfig.channels) {						
					case 1:
						printf( "%c%c%c%c", *(buffer+INDEX) & 0xff,zero,zero,zero);
						break;
					case 2:
						printf( "%c%c%c%c", (*(buffer+INDEX) & 0xff00)>>8,zero,zero,zero);
						break;
					case 3:
						printf( "%c%c%c%c", *(buffer+INDEX) & 0xff, (*(buffer+INDEX) & 0xff00)>>8,zero,zero);
						break;
					}
					sconfig.read_count--;
				}
				break;
			case SUMP_DESC:
				// device name string
				printf( "%c", 0x01);
				printf( "Esp32");
				printf( "%c", 0x00);
				//sample memory (8192)
				printf( "%c", 0x21);
				printf( "%c", 0x00);
				printf( "%c", 0x00);
				printf( "%c", 0x20);
				printf( "%c", 0x00);
				//sample rate (2MHz)
				printf( "%c", 0x23);
				printf( "%c", 0x00);
				printf( "%c", 0x1E);
				printf( "%c", 0x84);
				printf( "%c", 0x80);
				//number of probes (16)
				printf( "%c", 0x40);
				printf( "%c", 0x10);
				//protocol version (2)
				printf( "%c", 0x41);
				printf( "%c", 0x02);
				printf( "%c", 0x00);
				break;
			case SUMP_XON:
			case SUMP_XOFF:
				/* not implemented */
				break;
			default:
				// Other commands take 4 bytes as parameters
				if(char_read( sump_parameters, 4) == 4) {
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
						break;
					case SUMP_DIV:
						sconfig.divider = sump_parameters[2];
						sconfig.divider <<= 8;
						sconfig.divider |= sump_parameters[1];
						sconfig.divider <<= 8;
						sconfig.divider |= sump_parameters[0];
						sconfig.divider /= 50; /* Assuming 100MHz base frequency */
						sconfig.divider++;
						//  the sampling frequency is set to f = clock / (x + 1)
						tim_set_prescaler();
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
	sump_deinit();
}

