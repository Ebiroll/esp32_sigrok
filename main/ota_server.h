#pragma once

#define OTA_LISTEN_PORT 8032
#define OTA_BUFF_SIZE 1024


extern const int OTA_CONNECTED_BIT;


void ota_server_task(void *param);
void ota_server_start(void);
void initialise_wifi(void);

extern EventGroupHandle_t ota_event_group;




