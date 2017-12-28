#include "driver/uart.h"
#include <stdio.h>

#define ESP_REG(addr) *((volatile uint32_t *)(addr))
#include "read_chars.h"

char *readLine(uart_port_t uart,char *line,int len) {
	int size;
	char *ptr = line;
	while(1) {
		size = uart_read_bytes(uart, (unsigned char *)ptr, 1, portMAX_DELAY);
		if (size == 1) {
			if (*ptr == '\n') {
				*ptr = 0;
				return line;
			}
			ptr++;
		} // End of read a character
	} // End of loop
} // End of readLine


// This function polls uart data 
char *pollLine(uart_port_t uart,char *line,int len) {
	//int size;
	volatile uint32_t* FIFO=0;
	volatile uint32_t* RX_STATUS=0;

	char *ptr = line;
	switch (uart) {
		case UART_NUM_0:
		    FIFO=(uint32_t*)0x3ff40000;
			RX_STATUS=(uint32_t*)0x3ff4001c;
		break;
		case UART_NUM_1:
			FIFO=(uint32_t*)0x3ff50000;
			RX_STATUS=(uint32_t*)0x3ff5001c;
		break;
		case UART_NUM_2:
			FIFO=(uint32_t*)0x3ff6e000;	
			RX_STATUS=(uint32_t*)0x3ff6e01c;			
		break;		
		default:
		break;
	}

	while(1) {
		if (ESP_REG(RX_STATUS)>0) {
			*ptr=(ESP_REG(FIFO) & 0x000000ff);
			//printf("GOT:%c\n",(char)*ptr);
			if (*ptr == '\n') {
				*ptr = 0;
				return line;
			}
			ptr++;
		} 
		vTaskDelay(100 / portTICK_PERIOD_MS);
	} 
} 


int char_read_timeout(unsigned char *buff, int num_bytes, int timeout) {
	int size;
	int num_read=0;
	unsigned char *ptr = buff;
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
}

int char_read(unsigned char *buff, int num_bytes) {
	int size;
	int ret=num_bytes;
	unsigned char *ptr = buff;
	while(num_bytes>0) {
		size = uart_read_bytes(0, ptr, 1, portMAX_DELAY);
		if (size == 1) {
			num_bytes--;
			ptr++;
		} // End of read a character
	} // End of loop
	return (ret);
}

