/*-
 * Copyright (c) 2012-2013 Jan Breuer,
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
 * @file   scpi_server.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  TCP/IP SCPI Server
 *
 *
 */
#if 0

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


#define DEVICE_PORT 5555
#define CONTROL_PORT 5556


#define SCPI_THREAD_PRIO (tskIDLE_PRIORITY + 8)

#define SCPI_MSG_TIMEOUT                0
#define SCPI_MSG_TEST                   1
#define SCPI_MSG_IO_LISTEN              2
#define SCPI_MSG_CONTROL_IO_LISTEN      3
#define SCPI_MSG_IO                     4
#define SCPI_MSG_CONTROL_IO             5
#define SCPI_MSG_SET_ESE_REQ            6
#define SCPI_MSG_SET_ERROR              7

typedef struct {
    struct netconn *io_listen;
    struct netconn *control_io_listen;
    struct netconn *io;
    struct netconn *control_io;
    xQueueHandle evtQueue;
    /* FILE * fio; */
    /* fd_set fds; */
} user_data_t;

struct _queue_event_t {
    uint8_t cmd;
    uint8_t param1;
    int16_t param2;
} __attribute__((__packed__));
typedef struct _queue_event_t queue_event_t;


user_data_t user_data = {
    .io_listen = NULL,
    .io = NULL,
    .control_io_listen = NULL,
    .control_io = NULL,
    .evtQueue = 0,
};

/* a global output buffer to collect output data until it will be 'flushed' */
#define SCPI_OUPUT_BUFFER_SIZE      (4096+30)
char SCPI_outputBuffer[SCPI_OUPUT_BUFFER_SIZE];
unsigned int SCPI_outputBuffer_idx = 0;


size_t SCPI_Write(scpi_t * context, const char * data, size_t len) {

    if ((SCPI_outputBuffer_idx + len) > (SCPI_OUPUT_BUFFER_SIZE - 1)) {
        printf("SCPI_Write , space limited");
        len = (SCPI_OUPUT_BUFFER_SIZE - 1) - SCPI_outputBuffer_idx; /* limit length to left over space */
        /* apparently there is no mechanism to cope with buffers that are too small */
    }
    memcpy(&SCPI_outputBuffer[SCPI_outputBuffer_idx], data, len);
    SCPI_outputBuffer_idx += len;

    SCPI_outputBuffer[SCPI_outputBuffer_idx] = '\0';
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

scpi_result_t SCPI_Flush(scpi_t * context) {
    //printf("SCPI_Flush\n");
    if (context->user_context != NULL) {
        user_data_t * u = (user_data_t *) (context->user_context);
        if (u->io) {
            //SCPI_outputBuffer[SCPI_outputBuffer_idx] = 0x0a;
            //SCPI_outputBuffer_idx++;
            //SCPI_outputBuffer[SCPI_outputBuffer_idx] = 0x0; 
            //SCPI_outputBuffer_idx++;
            int tmp=SCPI_outputBuffer_idx;
            SCPI_outputBuffer_idx=0;
            netconn_write(u->io, SCPI_outputBuffer, tmp, NETCONN_NOCOPY); 
            return SCPI_RES_OK;
        }
    }
    return SCPI_RES_OK;
}

int SCPI_Error(scpi_t * context, int_fast16_t err) {
    (void) context;
    /* BEEP */
    printf("**ERROR: %d, \"%s\"\r\n", (int32_t) err, SCPI_ErrorTranslate(err));
    if (err != 0) {
        /* New error */
        /* Beep */
        /* Error LED ON */
    } else {
        /* No more errors in the queue */
        /* Error LED OFF */
    }
    return 0;
}

scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    char b[16];

    if (SCPI_CTRL_SRQ == ctrl) {
        iprintf("**SRQ: 0x%X (%d)\r\n", val, val);
    } else {
        iprintf("**CTRL %02x: 0x%X (%d)\r\n", ctrl, val, val);
    }

    if (context->user_context != NULL) {
        user_data_t * u = (user_data_t *) (context->user_context);
        if (u->control_io) {
            snprintf(b, sizeof (b), "SRQ%d\r\n", val);
            return netconn_write(u->control_io, b, strlen(b), NETCONN_NOCOPY) == ERR_OK ? SCPI_RES_OK : SCPI_RES_ERR;
        }
    }
    return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t * context) {
    (void) context;
    printf("**Reset\r\n");
    return SCPI_RES_OK;
}

scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t * context) {
    SCPI_ResultInt(context, CONTROL_PORT);
    return SCPI_RES_OK;
}

static void setEseReq(void) {
    SCPI_RegSetBits(&scpi_context, SCPI_REG_ESR, ESR_REQ);
}

static void setError(int16_t err) {
    SCPI_ErrorPush(&scpi_context, err);
}

void SCPI_RequestControl(void) {
    queue_event_t msg;
    msg.cmd = SCPI_MSG_SET_ESE_REQ;

    /* Avoid sending evtQueue message if ESR_REQ is already set
    if((SCPI_RegGet(&scpi_context, SCPI_REG_ESR) & ESR_REQ) == 0) {
        xQueueSend(user_data.evtQueue, &msg, 1000);
    }
     */

    xQueueSend(user_data.evtQueue, &msg, 1000);
}

void SCPI_AddError(int16_t err) {
    queue_event_t msg;
    msg.cmd = SCPI_MSG_SET_ERROR;
    msg.param2 = err;

    xQueueSend(user_data.evtQueue, &msg, 1000);
}

void scpi_netconn_callback(struct netconn * conn, enum netconn_evt evt, u16_t len) {
    queue_event_t msg;
    (void) len;


    if (evt == NETCONN_EVT_RCVPLUS) {
        msg.cmd = SCPI_MSG_TEST;
        if (conn == user_data.io) {
            msg.cmd = SCPI_MSG_IO;
        } else if (conn == user_data.io_listen) {
            msg.cmd = SCPI_MSG_IO_LISTEN;
        } else if (conn == user_data.control_io) {
            msg.cmd = SCPI_MSG_CONTROL_IO;
        } else if (conn == user_data.control_io_listen) {
            msg.cmd = SCPI_MSG_CONTROL_IO_LISTEN;
        }
        xQueueSend(user_data.evtQueue, &msg, 1000);
    } else if (evt == NETCONN_EVT_RCVMINUS) {
         if (conn == user_data.io) {
             printf(".\n");
         }
    }
}

static struct netconn * createServer(int port) {
    struct netconn * conn;
    err_t err;

    printf("creating spci server: %d\n",port);

    conn = netconn_new_with_callback(NETCONN_TCP, scpi_netconn_callback);
    if (conn == NULL) {
        return NULL;
    }

    err = netconn_bind(conn, NULL, port);
    if (err != ERR_OK) {
        netconn_delete(conn);
        return NULL;
    }


    netconn_listen(conn);

    return conn;
}

static void waitServer(user_data_t * user_data, queue_event_t * evt) {
    /* 5s timeout */
    if (xQueueReceive(user_data->evtQueue, evt, 5000 * portTICK_RATE_MS) != pdPASS) {
        evt->cmd = SCPI_MSG_TIMEOUT;
    }
}

static int processIoListen(user_data_t * user_data) {
    struct netconn *newconn;

    if (netconn_accept(user_data->io_listen, &newconn) == ERR_OK) {
        if (user_data->io) {
            /* Close previous unwanted connection */
            printf("***Connection already established, closing");
            netconn_close(user_data->io);
            netconn_delete(user_data->io); 
            user_data->io = newconn;           
        } else {
            /* connection established */
            //iprintf("***Connection established %s\r\n", inet_ntoa(newconn->pcb.ip->remote_ip));
            user_data->io = newconn;
        }
    }

    return 0;
}

static int processSrqIoListen(user_data_t * user_data) {
    struct netconn *newconn;

    if (netconn_accept(user_data->control_io_listen, &newconn) == ERR_OK) {
        if (user_data->control_io) {
            printf("***Control connection already established, closing");
            netconn_close(user_data->control_io);
            netconn_delete(user_data->control_io);
        } else {
            /* control connection established */
            printf("***Control Connection established %s\r\n", inet_ntoa(newconn->pcb.ip->remote_ip));
            user_data->control_io = newconn;
        }
    }

    return 0;
}

static void closeIo(user_data_t * user_data) {
    /* connection closed */
    netconn_close(user_data->io);
    netconn_delete(user_data->io);
    user_data->io = NULL;
    SCPI_outputBuffer_idx=0;
    //iprintf("***Connection closed\r\n");
}

static void closeSrqIo(user_data_t * user_data) {
    /* control connection closed */
    netconn_close(user_data->control_io);
    netconn_delete(user_data->control_io);
    user_data->control_io = NULL;
    //iprintf("***Control Connection closed\r\n");
}

static int processIo(user_data_t * user_data) {
    struct netbuf *inbuf;
    char* buf;
    u16_t buflen;

    if (netconn_recv(user_data->io, &inbuf) != ERR_OK) {
        goto fail1;
    }
    if (netconn_err(user_data->io) != ERR_OK) {
        goto fail2;
    }

    netbuf_data(inbuf, (void**) &buf, &buflen);

    if (buflen > 0) {
        buf[buflen]=0;
        printf("%s",buf);
        SCPI_Input(&scpi_context, buf, buflen);
    } else {
        /* goto fail2; */
    }

    netbuf_delete(inbuf);

    return 0;

fail2:
    netbuf_delete(inbuf);
fail1:
    closeIo(user_data);

    return 0;
}

static int processSrqIo(user_data_t * user_data) {
    struct netbuf *inbuf;
    char* buf;
    u16_t buflen;

    if (netconn_recv(user_data->control_io, &inbuf) != ERR_OK) {
        goto fail1;
    }
    if (netconn_err(user_data->control_io) != ERR_OK) {
        goto fail2;
    }

    netbuf_data(inbuf, (void**) &buf, &buflen);


    if (buflen > 0) {
        buf[buflen]=0;
        printf("i%s",buf);
        SCPI_Input(&scpi_context, buf, buflen);

        /* TODO process control */
    } else {
        /* goto fail2; */
    }

    netbuf_delete(inbuf);

    return 0;

fail2:
    netbuf_delete(inbuf);
fail1:
    closeSrqIo(user_data);

    return 0;
}

/*
 *
 */
static void scpi_server_thread(void *arg) {
    queue_event_t evt;

    (void) arg;

    user_data.evtQueue = xQueueCreate(10, sizeof (queue_event_t));

    /* user_context will be pointer to socket */
    SCPI_Init(&scpi_context,
            scpi_commands,
            &scpi_interface,
            scpi_units_def,
            SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
            scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
            scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);

    scpi_context.user_context = &user_data;

    user_data.io_listen = createServer(DEVICE_PORT);
    //user_data.control_io_listen = createServer(CONTROL_PORT);
    user_data.control_io_listen = NULL;

    while (1) {
        waitServer(&user_data, &evt);

        if (evt.cmd == SCPI_MSG_TIMEOUT) { /* timeout */
            printf("Timeout\n");
            SCPI_Input(&scpi_context, NULL, 0);
        }

        if ((user_data.io_listen != NULL) && (evt.cmd == SCPI_MSG_IO_LISTEN)) {
            processIoListen(&user_data);
        }

        if ((user_data.control_io_listen != NULL) && (evt.cmd == SCPI_MSG_CONTROL_IO_LISTEN)) {
            processSrqIoListen(&user_data);
        }

        if ((user_data.io != NULL) && (evt.cmd == SCPI_MSG_IO)) {
            processIo(&user_data);
        }

        if ((user_data.control_io != NULL) && (evt.cmd == SCPI_MSG_CONTROL_IO)) {
            processSrqIo(&user_data);
        }

        if (evt.cmd == SCPI_MSG_SET_ESE_REQ) {
            setEseReq();
        }

        if (evt.cmd == SCPI_MSG_SET_ERROR) {
            setError(evt.param2);
        }

    }

    vTaskDelete(NULL);
}

extern TaskHandle_t xTaskList[20];
extern uint8_t xtaskListCounter;


void scpi_server_init(TaskHandle_t *pvCreatedTask) {
    printf("scpi server thread\n");
    sys_thread_t tmp=sys_thread_new("SCPI", scpi_server_thread, NULL, 5 * DEFAULT_THREAD_STACKSIZE, SCPI_THREAD_PRIO);
    xTaskList[xtaskListCounter]=tmp;

}
#endif