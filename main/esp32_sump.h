#ifndef __ESP32_SUMP
#define __ESP32_SUMP

/*
 * HydraBus/HydraNFC
 *  ESP32 sigrok 
 *
 * Copyright (C) 2015 Nicolas OBERLI
 *               2017 Olof Astrand
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
#include "lwip/tcpip.h"
#include "freertos/FreeRTOS.h"
//#include "task.h"
//#include "esp_system.h"
//#include "lwip/tcp.h"
//#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/api.h"
#include <stdint.h>


#define SUMP_RESET	0x00
#define SUMP_RUN	0x01
#define SUMP_ID		0x02
#define SUMP_DESC	0x04
#define SUMP_XON	0x11
#define SUMP_XOFF	0x13
#define SUMP_DIV	0x80
#define SUMP_CNT	0x81
#define SUMP_FLAGS	0x82
#define SUMP_TRIG_1	0xc0
#define SUMP_TRIG_2	0xc4
#define SUMP_TRIG_3	0xc8
#define SUMP_TRIG_4	0xcc
#define SUMP_TRIG_VALS_1  0xc1
#define SUMP_TRIG_VALS_2  0xc5
#define SUMP_TRIG_VALS_3  0xc9
#define SUMP_TRIG_VALS_4  0xcd

#define SUMP_STATE_IDLE		0
#define SUMP_STATE_ARMED	1
#define SUMP_STATE_RUNNNING	2
#define SUMP_STATE_TRIGGED	3

typedef struct {
	uint32_t trigger_masks[4];
	uint32_t trigger_values[4];
	uint32_t read_count;
	uint32_t delay_count;
	uint32_t divider;
	uint8_t state;
	uint8_t channels;
} sump_config;

typedef struct _sump_t {
	int            uart_portno;
	struct netconn *io;
	struct netconn *io_listen;

	xQueueHandle   evtQueue;
} sump_context_t;


typedef int(*sump_read_timeout_t)(sump_context_t * context, unsigned char * data,unsigned int len, size_t timeout);
typedef int(*sump_read_chars_t)(sump_context_t * context, unsigned char * data,unsigned int len);
typedef int(*sump_write_t)(sump_context_t * context, const char * data, size_t len);
typedef int (*sump_error_callback_t)(sump_context_t * context, int_fast16_t error);
typedef int (*sump_flush_t)(sump_context_t * context);


struct _sump_interface_t {
        sump_error_callback_t 	error;
        sump_write_t 			write;
		sump_read_timeout_t  	read_timeout;
		sump_read_chars_t		read_chars;
		sump_flush_t            flush;
    };

typedef int SUMP_result_t;

typedef struct _sump_interface_t sump_interface_t;

enum _sump_result_t {
	SUMP_RES_OK = 1,
	SUMP_RES_ERR = -1
};

void sump_init();
void sump_uart();
void sump(sump_context_t *context,sump_interface_t *io);
#endif