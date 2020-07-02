#ifndef __ESP32_ANALOG
#define __ESP32_ANALOG

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SAMPLING_DONE_BIT   1

#define NUM_SAMPLES 14000
//14000

typedef enum TrigState {
    Triggered,
    Auto,
    Running,
    Maiting,
    Stopped
} TrigState_t;

TrigState_t get_trig_state();

uint8_t* get_values();

uint16_t* get_digital_values();

uint8_t*  get_sample_values();


// Param is task handle of task to notify
void sample_thread(void *param);

void setTimescale(float scale);

void start_sampling();

bool samples_finnished();


extern TaskHandle_t xHandlingTask;

#endif