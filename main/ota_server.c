//https://github.com/yanbe/esp-idf-ota-template/blob/master/components/ota_server/ota_server.c
	
#include "Header.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "lwip/sockets.h"


const int OTA_CONNECTED_BIT = BIT0;
static const char * TAG = "OTA";
EventGroupHandle_t ota_event_group;
/*socket*/
static int connect_socket = 0;

void ota_server_task(void *param)
{
	xEventGroupWaitBits(ota_event_group, OTA_CONNECTED_BIT, false, true, portMAX_DELAY);
	ota_server_start();
	vTaskDelete(NULL);
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
        printf("Connectiing To SSID:%s : Pass:%s\r\n", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
        printf("got ip:%s",	ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		xEventGroupSetBits(ota_event_group, OTA_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
        printf("SYSTEM_EVENT_STA_DISCONNECTED\r\n");
		esp_wifi_connect();
		xEventGroupClearBits(ota_event_group, OTA_CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

void initialise_wifi(void)
{
	ota_event_group = xEventGroupCreate();
	
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t sta_config = {
		.sta = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASSWORD,
			.bssid_set = false
			}
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}


static int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
	
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
	
    if (err == -1) 
    {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}

static int show_socket_error_reason(const char *str, int socket)
{
    int err = get_socket_error_code(socket);

    if (err != 0) 
    {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }

    return err;
}

static esp_err_t create_tcp_server()
{
    ESP_LOGI(TAG, "server socket....port=%d", OTA_LISTEN_PORT);
    int server_socket = 0;
    struct sockaddr_in server_addr;
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
	
    if (server_socket < 0) 
    {
        show_socket_error_reason("create_server", server_socket);
        return ESP_FAIL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(OTA_LISTEN_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) 
    {
        show_socket_error_reason("bind_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
	
    if (listen(server_socket, 5) < 0) 
    {
        show_socket_error_reason("listen_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
	
    struct sockaddr_in client_addr;
    unsigned int socklen = sizeof(client_addr);
    connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
	
    if (connect_socket < 0) 
    {
        show_socket_error_reason("accept_server", connect_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

void ota_server_start(void)
{
	uint8_t percent_loaded;
	
    ESP_ERROR_CHECK( create_tcp_server() );
	
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",	update_partition->subtype, update_partition->address);

	//https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/system/log.html
	// I dont want to see all the Log esp_image stuff while its flashing it
	esp_log_level_set("esp_image", ESP_LOG_ERROR);           // set all components to ERROR level  ESP_LOG_NONE


	// We dont want any other thread running during this update. 
	//SuspendAllThreads();
	KillAllThreads();

    int recv_len;
    char ota_buff[OTA_BUFF_SIZE] = {0};
    bool is_req_body_started = false;
    int content_length = -1;
    int content_received = 0;

    esp_ota_handle_t ota_handle; 

	
    do {
        recv_len = recv(connect_socket, ota_buff, OTA_BUFF_SIZE, 0);
	    
        if (recv_len > 0) 
        {
            if (!is_req_body_started) 
            {
                const char *content_length_start = "Content-Length: ";
                char *content_length_start_p = strstr(ota_buff, content_length_start) + strlen(content_length_start);
                sscanf(content_length_start_p, "%d", &content_length);
                ESP_LOGI(TAG, "Detected content length: %d", content_length);
                ESP_ERROR_CHECK( esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle) );
                const char *header_end = "\r\n\r\n";
                char *body_start_p = strstr(ota_buff, header_end) + strlen(header_end);
                int body_part_len = recv_len - (body_start_p - ota_buff);
                esp_ota_write(ota_handle, body_start_p, body_part_len);
                content_received += body_part_len;
                is_req_body_started = true;
            }
	        else 
	        {
                esp_ota_write(ota_handle, ota_buff, recv_len);
                content_received += recv_len;
		        
		        percent_loaded = (((float)content_received / (float)content_length) * 100.00);
		        ESP_LOGI(TAG, "Uploaded %03u%%", percent_loaded);
            }
        }
        else if (recv_len < 0) 
        {
	        ESP_LOGI(TAG, "Error: recv data error! errno=%d", errno);
        }
	    
    } while (recv_len > 0 && content_received < content_length);

//	ESP_LOGI(TAG, "OTA Transferred Finished: %d bytes", content_received);
	
	
	char res_buff[128];
	int send_len;
	send_len = sprintf(res_buff, "200 OK\n\n");
	send(connect_socket, res_buff, send_len, 0);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	close(connect_socket);

    ESP_ERROR_CHECK( esp_ota_end(ota_handle) );
	
    esp_err_t err = esp_ota_set_boot_partition(update_partition);
	
	if (err == ESP_OK) 
	{
		const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
	
		ESP_LOGI(TAG, "***********************************************************");
		ESP_LOGI(TAG, "OTA Successful");
		ESP_LOGI(TAG, "Next Boot Partition Subtype %d At Offset 0x%x", boot_partition->subtype, boot_partition->address);
		ESP_LOGI(TAG, "***********************************************************");
	}
	else
	{
		ESP_LOGI(TAG, "!!! OTA Failed !!!");
	}

	
	for (int x = 3; x >= 1; x--)
	{
		ESP_LOGI(TAG, "Prepare to restart system...%d", x);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

    esp_restart();
}
