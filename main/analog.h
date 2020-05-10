#ifndef __ESP32_ANALOG
#define __ESP32_ANALOG

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SAMPLING_DONE_BIT   1

#define NUM_SAMPLES 2048
//14000

uint8_t* get_values();

uint16_t* get_digital_values();

int* get_sample_values();


// Param is task handle of task to notify
void sample_thread(void *param);

void setTimescale(float scale);

void start_sampling();

bool samples_finnished();


extern TaskHandle_t xHandlingTask;

#endif