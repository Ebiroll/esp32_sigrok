#pragma once


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//#include "esp_event_loop.h"
#include <esp_system.h>
#include <nvs_flash.h>

#include "ota_server.h"

#define FIRMWARE_REV    " Rev: 0.1"


 void KillAllThreads(void);
 

