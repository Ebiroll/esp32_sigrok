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



#define CH1COLOR TFT_YELLOW
#define CH2COLOR TFT_CYAN
#define  SAMPLES  320

short data[SAMPLES];
short old_data[SAMPLES];

short slow_data[SAMPLES];
short old_slow_data[SAMPLES];


// NUM_SAMPLES 2048

void DrawGrid()
{
	for (int x = 0; x <= SAMPLES; x += 2) // Horizontal Line
	{
		for (int y = 0; y <= LCD_HEIGHT; y += DOTS_DIV)
		{
			TFT_drawPixel(x, y, TFT_DARKGREY,0);
		}
		if (LCD_HEIGHT == 240)
		{
			TFT_drawPixel(x, LCD_HEIGHT - 1, TFT_DARKGREY,0);
		}
	}
	for (int x = 0; x <= SAMPLES; x += DOTS_DIV) // Vertical Line
	{
		for (int y = 0; y <= LCD_HEIGHT; y += 2)
		{
			TFT_drawPixel(x, y, TFT_LIGHTGREY,0);
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

void DrawSlowGraph()
{
	DrawGrid();

	for (int x = 0; x < (SAMPLES - 1); x++)
	{
		TFT_drawLine(x, LCD_HEIGHT/2 - old_slow_data[x], x + 1, 1+LCD_HEIGHT/2 - old_slow_data[x + 1], TFT_BLACK);
		TFT_drawLine(x, LCD_HEIGHT/2 - slow_data[x], x + 1, 1+LCD_HEIGHT/2 - slow_data[x + 1], TFT_GREEN);
	}
}


void DrawGraph()
{
	DrawGrid();

	for (int x = 0; x < (SAMPLES - 1); x++)
	{
		TFT_drawLine(x, LCD_HEIGHT/2 - old_data[x], x + 1, 1+LCD_HEIGHT/2 - old_data[x + 1], TFT_BLACK);
		TFT_drawLine(x, LCD_HEIGHT/2 - data[x], x + 1, 1+LCD_HEIGHT/2 - data[x + 1], TFT_WHITE);
	}
}


void drawSampleData(int* in_data,int num_samples) {

	for (int x = 0; x < SAMPLES ; x++) {
        data[x]=(short) -30 + in_data[x];
    }
    DrawGraph();
	for (int x = 0; x < SAMPLES ; x++) {
        old_data[x]=data[x];
    }

	int skip=SAMPLES;

	for (int x = 0; x < SAMPLES  && skip < num_samples; x++) {
        slow_data[x]=(short) -80 + (in_data[skip]+in_data[skip+1]+in_data[skip+2]+in_data[skip+3])/4;
		skip+=4;
    }
	DrawSlowGraph();
	for (int x = 0; x < SAMPLES ; x++) {
        old_slow_data[x]=slow_data[x];
    }



}



#endif