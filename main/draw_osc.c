#include <time.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_heap_caps.h"


#if  CONFIG_EXAMPLE_USE_TFT


#include "tftspi.h"
#include "tft.h"
#include "analog.h"



const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;
const int DOTS_DIV = 30;

#define BLACK (color_t){ 00, 00, 00 }
#define GREY (color_t){ 70, 70, 80 }
#define CYAN (color_t){ 07, 7F, 0 }
#define YELLOW   (color_t){ 255, 180, 0 }


//#define BLACK           0x0000
//#define RED             0xF800
//#define CYAN            0x07FF
//#define YELLOW          0xFFE0 
//#define WHITE           0xFFFF
//#define GREY            0x7BEF

#define CH1COLOR YELLOW
#define CH2COLOR CYAN
#define  SAMPLES  320

short data[SAMPLES];


// NUM_SAMPLES 2048

void DrawGrid()
{
	for (int x = 0; x <= SAMPLES; x += 2) // Horizontal Line
	{
		for (int y = 0; y <= LCD_HEIGHT; y += DOTS_DIV)
		{
			TFT_drawPixel(x, y, GREY,1);
		}
		if (LCD_HEIGHT == 240)
		{
			TFT_drawPixel(x, LCD_HEIGHT - 1, GREY,1);
		}
	}
	for (int x = 0; x <= SAMPLES; x += DOTS_DIV) // Vertical Line
	{
		for (int y = 0; y <= LCD_HEIGHT; y += 2)
		{
			TFT_drawPixel(x, y, GREY,1);
		}
	}
}

#if 0
void DrawGrid(int x)
{
	if ((x % 2) == 0)
	{
		for (int y = 0; y <= LCD_HEIGHT; y += DOTS_DIV)
		{
			TFT_drawPixel(x, y, GREY,1);
		}
	}
	if ((x % DOTS_DIV) == 0)
	{
		for (int y = 0; y <= LCD_HEIGHT; y += 2)
		{
			TFT_drawPixel(x, y, GREY,1);
		}
	}
}
#endif

void ClearGraph()
{

	for (int x = 0; x < (SAMPLES - 1); x++)
	{
		TFT_drawLine(x, LCD_HEIGHT - data[x], x + 1, LCD_HEIGHT - data[x + 1], BLACK);
    }

}
void DrawGraph()
{
	for (int x = 0; x < (SAMPLES - 1); x++)
	{
		TFT_drawLine(x, LCD_HEIGHT - data[x], x + 1, LCD_HEIGHT - data[x + 1], CH1COLOR);
	}
}


void drawSampleData(int* data,int num_samples) {


    ClearGraph();

	for (int x = 0; x < SAMPLES ; x++) {
        data[x]=(short) data[x];
    }
    DrawGraph();

}



#endif