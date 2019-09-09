/*-
 * Copyright (c) 2017 Olof Åstrand,
 *
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   sump_server.c
 * @date   Thu Dec 27 10:58:45 UTC 2017
 *
 * @brief  TCP/IP SCPI Server
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "scpi/scpi.h"
#include "scpi-def.h"

#include "lwip/tcpip.h"
#include "freertos/FreeRTOS.h"
//#include "task.h"
#include "esp_system.h"
#include "lwip/tcp.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/api.h"
#include "esp32_sump.h"

#define SUMP_PORT 5566



#define SUMP_THREAD_PRIO (tskIDLE_PRIORITY + 3)

#define SUMP_MSG_TIMEOUT                0
#define SUMP_MSG_TEST                   1
#define SUMP_MSG_IO_LISTEN              2
#define SUMP_MSG_IO                     4

//typedef struct {
//    struct netconn *io_listen;
//    struct netconn *io;
//    xQueueHandle evtQueue;
    /* FILE * fio; */
    /* fd_set fds; */
//} user_data_t;

struct _queue_event_t {
    uint8_t cmd;
    uint8_t param1;
    int16_t param2;
} __attribute__((__packed__));
typedef struct _queue_event_t queue_event_t;

queue_event_t evt;
sump_context_t context;

static int processIoListen(sump_context_t* context);
static int processIo(sump_context_t* context,char *data,int maxlen);
static void waitServer(sump_context_t* context, queue_event_t * evt);

/* a global output buffer to collect output data until it will be 'flushed' */
#define SUMP_OUPUT_BUFFER_SIZE      (256)
char SUMP_outputBuffer[SUMP_OUPUT_BUFFER_SIZE];
unsigned int SUMP_outputBuffer_idx = 0;

/* a global output buffer to collect input data until it will be 'flushed' */
#define SUMP_INPUT_BUFFER_SIZE      (256)
char SUMP_inputBuffer[SUMP_INPUT_BUFFER_SIZE];
unsigned int SUMP_inputBuffer_idx = 0;
unsigned int SUMP_inputBuffer_size = 0;


int SUMP_Write(sump_context_t * context, const char * data, size_t len) {

    if ((SUMP_outputBuffer_idx + len) > (SUMP_OUPUT_BUFFER_SIZE - 1)) {
        len = (SUMP_OUPUT_BUFFER_SIZE - 1) - SUMP_outputBuffer_idx; /* limit length to left over space */
        /* apparently there is no mechanism to cope with buffers that are too small */
    }
    memcpy(&SUMP_outputBuffer[SUMP_outputBuffer_idx], data, len);
    SUMP_outputBuffer_idx += len;

    SUMP_outputBuffer[SUMP_outputBuffer_idx] = '\0';
 /*   
    if (context->user_context != NULL) {
        user_data_t * u = (user_data_t *) (context->user_context);
        if (u->io) {
            return (netconn_write(u->io, data, len, NETCONN_NOCOPY) == ERR_OK) ? len : 0;
        }
    }
*/
    return 0;
}

int SUMP_Flush(sump_context_t * context) {
    if (context->io) {
        SUMP_outputBuffer[SUMP_outputBuffer_idx] = 0x0a;
        SUMP_outputBuffer_idx++;
        //SUMP_outputBuffer[SUMP_outputBuffer_idx] = 0x0; 
        //SUMP_outputBuffer_idx++;
        int tmp=SUMP_outputBuffer_idx;
        SUMP_outputBuffer_idx=0;
        netconn_write(context->io, SUMP_outputBuffer, tmp, NETCONN_NOCOPY); 
        return SUMP_RES_OK;
    }
    return SUMP_RES_OK;
}


void sump_debug(char *str,unsigned int value) {
 #ifdef  DEBUG_SUMP
    printf("%s 0x%X\n",str,value);   
#endif
#if 0
    strcpy(&SUMP_outputBuffer[SUMP_outputBuffer_idx], str);
    SUMP_outputBuffer_idx +=  strlen(str);
    char Buffer[10];
    sprintf(Buffer,"0x%X\n",value);

    strcpy(&SUMP_outputBuffer[SUMP_outputBuffer_idx], Buffer);
    SUMP_outputBuffer_idx +=  strlen(Buffer);

    SUMP_Flush(&context);
#endif
} 


int SUMP_Error(sump_context_t * context, int_fast16_t err) {
    (void) context;
    /* BEEP */
    printf("**ERROR: %d, \"%d\"\r\n", (int32_t) err, (err));
    if (err != 0) {
        /* New error */
        /* Beep */
        /* Error LED ON */
    } else {
        /* No more errors in the queue */
        /* Error LED OFF */
    }
    return 0;
};


int SUMP_read_timeout(sump_context_t * context, char * data,unsigned int len, size_t timeout) {

//unsigned int SUMP_inputBuffer_idx = 0;
//unsigned int SUMP_inputBuffer_size = 0;
    if (SUMP_inputBuffer_idx!=SUMP_inputBuffer_size) {
        int tocopy=len;
        while (tocopy-->0 && SUMP_inputBuffer_idx<SUMP_inputBuffer_size) {
            *data=SUMP_inputBuffer[SUMP_inputBuffer_idx];
            SUMP_inputBuffer_idx++;
        }
        SUMP_inputBuffer_idx-=len;
        return len;
    }



    //printf("Waiting for server\n");

    waitServer(context, &evt);
    //printf("Waited for server\n");


    if (evt.cmd == SUMP_MSG_TIMEOUT) { /* timeout */
        //SUMP_Input(&SUMP_context, NULL, 0);
        return 0;
    }

    if ((context->io_listen != NULL) && (evt.cmd == SUMP_MSG_IO_LISTEN)) {
        processIoListen(context);
    }

    if ((context->io != NULL) && (evt.cmd == SUMP_MSG_IO)) {
        return (processIo(context,data,len));
    }


    return 0;

}
int SUMP_read_chars(sump_context_t * context, char * data,unsigned int len) {    
   if (SUMP_inputBuffer_idx!=SUMP_inputBuffer_size) {
        int tocopy=len;
        while (tocopy-->0 && SUMP_inputBuffer_idx<SUMP_inputBuffer_size) {
            *data=SUMP_inputBuffer[SUMP_inputBuffer_idx];
            SUMP_inputBuffer_idx++;
        }
        return len;
    }
    return (SUMP_read_timeout(context,data,len,0));
}


sump_interface_t sump_network_interface = {
    .error = SUMP_Error,
    .write = SUMP_Write,
	.read_timeout=SUMP_read_timeout,
	.read_chars=SUMP_read_chars,
	.flush=SUMP_Flush,
};


SUMP_result_t SUMP_Reset(sump_context_t * context) {
    (void) context;
    iprintf("**Reset\r\n");
    return SUMP_RES_OK;
}


void SUMP_netconn_callback(struct netconn * conn, enum netconn_evt evt, u16_t len) {
    queue_event_t msg;
    (void) len;

    printf("SUMP_netconn_callback\n");

    if (evt == NETCONN_EVT_RCVPLUS) {
        msg.cmd = SUMP_MSG_TEST;
        if (conn == context.io) {
            msg.cmd = SUMP_MSG_IO;
        } else if (conn == context.io_listen) {
            msg.cmd = SUMP_MSG_IO_LISTEN;
        } 

        xQueueSend(context.evtQueue, &msg, 1000);
    }
}

static struct netconn * createSumpServer(int port) {
    struct netconn * conn;
    err_t err;

    printf("creating sump server: %d\n",port);

    conn = netconn_new_with_callback(NETCONN_TCP, SUMP_netconn_callback);
    if (conn == NULL) {
        return NULL;
    }

    err = netconn_bind(conn, NULL, port);
    if (err != ERR_OK) {
        printf("Bind failed: err=%d\n",err);

        netconn_delete(conn);
        return NULL;
    }

    //printf("start listen\n");


    netconn_listen(conn);

    //printf("Listen ok!\n");


    return conn;
}

static void waitServer(sump_context_t* context, queue_event_t * evt) {
    /* 0.5 s timeout */
    if (xQueueReceive(context->evtQueue, evt, 500 * portTICK_RATE_MS) != pdPASS) {
        evt->cmd = SUMP_MSG_TIMEOUT;
    }
}

static int processIoListen(sump_context_t *context) {
    struct netconn *newconn;

    if (netconn_accept(context->io_listen, &newconn) == ERR_OK) {
        if (context->io) {
            /* Close unwanted connection */
            netconn_close(context->io);
            netconn_delete(context->io);
            context->io = newconn;
        } else {
            /* connection established */
            iprintf("***SUMP Connection established %s\r\n", inet_ntoa(newconn->pcb.ip->remote_ip));
            context->io = newconn;
        }
    }

    return 0;
}


static void closeIo(sump_context_t * context) {
    /* connection closed */
    netconn_close(context->io);
    netconn_delete(context->io);
    context->io = NULL;
    iprintf("***Connection closed\r\n");
}


static int processIo(sump_context_t * context,char *data, int maxlen) {
    struct netbuf *inbuf;
    char* buf;
    u16_t buflen;
    int ret=0;

    if (netconn_recv(context->io, &inbuf) != ERR_OK) {
        goto fail1;
    }
    if (netconn_err(context->io) != ERR_OK) {
        goto fail2;
    }

    netbuf_data(inbuf, (void**) &buf, &buflen);

    if (buflen > 0) {
        if (buflen<maxlen) {
           memcpy(data,buf,buflen);
           ret=buflen;
        } else  {
              memcpy(data,buf,maxlen);
              ret=maxlen;
              // TODO! SOME of the message is list
        }
        //SUMP_Input(&SUMP_context, buf, buflen);
    } else {
        /* goto fail2; */
    }

    netbuf_delete(inbuf);

    return ret;

fail2:
    netbuf_delete(inbuf);
fail1:
    closeIo(context);

    return 0;
}


//void sump_network(sump_interface_t *net_if)
//{
// 
//  sump(&context,&uart_interface);
//}

/*
 *
 */
static void sump_server_thread(void *arg) {
  

    (void) arg;

    context.evtQueue = xQueueCreate(10, sizeof (queue_event_t));
    context.io_listen = createSumpServer(SUMP_PORT);


    while (1) {
        sump(&context,&sump_network_interface);
    }

    vTaskDelete(NULL);
}

extern TaskHandle_t xTaskList[20];
extern uint8_t xtaskListCounter;

void sump_server_init(void) {
    printf("Server thread\n");
    sys_thread_t tmp=sys_thread_new("SUMP", sump_server_thread, NULL, 2 * 4096, SUMP_THREAD_PRIO);
    xTaskList[xtaskListCounter]=tmp;
}