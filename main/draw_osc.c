#if CONFIG_EXAMPLE_USE_TFT


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
#include "tftspi.h"
#include "tft.h"
#include "analog.h"



const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;

#define BLACK           0x0000
#define RED             0xF800
#define CYAN            0x07FF
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF
#define GREY            0x7BEF

#define CH1COLOR YELLOW
#define CH2COLOR CYAN
const int SAMPLES = 320;

short data[4][SAMPLES];


// NUM_SAMPLES 2048

void DrawGrid()
{
	for (int x = 0; x <= SAMPLES; x += 2) // Horizontal Line
	{
		for (int y = 0; y <= LCD_HEIGHT; y += DOTS_DIV)
		{
			TFT_drawPixel(x, y, GREY);
			CheckSW();
		}
		if (LCD_HEIGHT == 240)
		{
			TFT_drawPixel(x, LCD_HEIGHT - 1, GREY);
		}
	}
	for (int x = 0; x <= SAMPLES; x += DOTS_DIV) // Vertical Line
	{
		for (int y = 0; y <= LCD_HEIGHT; y += 2)
		{
			TFT_drawPixel(x, y, GREY);
		}
	}
}

void DrawGrid(int x)
{
	if ((x % 2) == 0)
	{
		for (int y = 0; y <= LCD_HEIGHT; y += DOTS_DIV)
		{
			TFT_drawPixel(x, y, GREY);
		}
	}
	if ((x % DOTS_DIV) == 0)
	{
		for (int y = 0; y <= LCD_HEIGHT; y += 2)
		{
			TFT_drawPixel(x, y, GREY);
		}
	}
}

void ClearAndDrawGraph()
{
	int clear = 0;

	if (sample == 0)
	{
		clear = 2;
	}
	for (int x = 0; x < (SAMPLES - 1); x++)
	{
		TFT_drawLine(x, LCD_HEIGHT - data[clear + 0][x], x + 1, LCD_HEIGHT - data[clear + 0][x + 1], BLACK);
		TFT_drawLine(x, LCD_HEIGHT - data[clear + 1][x], x + 1, LCD_HEIGHT - data[clear + 1][x + 1], BLACK);
		//if (ch0_mode != MODE_OFF)
		{
			TFT_drawLine(x, LCD_HEIGHT - data[sample + 0][x], x + 1, LCD_HEIGHT - data[sample + 0][x + 1], CH1COLOR);
		}
		//if (ch1_mode != MODE_OFF)
		//{
		//	TFT_drawLine(x, LCD_HEIGHT - data[sample + 1][x], x + 1, LCD_HEIGHT - data[sample + 1][x + 1], CH2COLOR);
		//}
	}
}


//short data[4][SAMPLES];
void drawSampleData(uint8_t* data,int num_samples) {

    // todo set data() !!!
    ClearAndDrawGraph();

}



#endif