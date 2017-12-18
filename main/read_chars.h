#ifndef READ_CHARS_H
#define READ_CHARS_H 1
#include "driver/uart.h"

extern int char_read_timeout(unsigned char *buff, int num_bytes, int timeout);
extern int char_read(unsigned char *buff, int num_bytes);

extern char *readLine(uart_port_t uart,char *line,int len);
extern char *pollLine(uart_port_t uart,char *line,int len);

#endif