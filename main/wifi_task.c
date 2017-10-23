#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "vospi.h"
#include "shared_frame.h"

static const char* TAG = "WiFiTask";

esp_err_t event_handler(void* ctx, system_event_t* event)
{
    return ESP_OK;
}

void wifi_task(c_frame_t* c_frame)
{
  nvs_flash_init();
  tcpip_adapter_init();

  #ifdef WIFI_STANDALONE
  tcpip_adapter_ip_info_t info = { 0, };
  IP4_ADDR(&info.ip, 10, 0, 0, 1);
  IP4_ADDR(&info.gw, 10, 0, 0, 1);
  IP4_ADDR(&info.netmask, 255, 255, 255, 0);
  ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
  ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
  ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  wifi_config_t ap_config = {
    .ap = {
        .ssid = "ThermalCamera",
        .ssid_len = sizeof("ThermalCamera"),
        .password = "password",
        .authmode = WIFI_AUTH_OPEN,
        .ssid_hidden = 0,
        .max_connection = 1,
        .beacon_interval = 100
    }
  };
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  #else
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  wifi_config_t sta_config = {
      .sta = {
          .ssid = "TestNetwork",
          .password = "TestNetwork",
          .bssid_set = false
      }
  };
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect());
  #endif

  // Listen for connections
  struct sockaddr_in server_addr, client_addr;
  int server_sock;
  socklen_t sin_size = sizeof(client_addr);
  bzero(&server_addr, sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(80);
  server_sock = socket(AF_INET, SOCK_STREAM, 0);
  bind(server_sock, (struct sockaddr *)(&server_addr), sizeof(struct sockaddr));
  listen(server_sock, 5);

  for (;;)
  {
    int client_sock = accept(server_sock, (struct sockaddr *) &client_addr, &sin_size);

    // Wait for a frame to be available
    if (xSemaphoreTake(c_frame->sem, 1000) == pdTRUE) {

      // Write the frame out to our client
      char* http_headers = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n";
      write(client_sock, http_headers, strlen(http_headers));

      // Write the frame data
      char id[30];
      sprintf(id, "ID is: %02x", c_frame->frame.segments[0].packets[20].id);
      write(client_sock, id, strlen(id));

      xSemaphoreGive(c_frame->sem);

    } else {
      ESP_LOGW(TAG, "couldn't obtain c_frame sem, can't send a frame right now");
      close(client_sock);
      continue;
    }

    close(client_sock);
  }
}
