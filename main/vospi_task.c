#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "vospi.h"
#include "driver/spi_master.h"
#include "shared_frame.h"

static const char* TAG = "VoSPITask";

void vospi_task(c_frame_t* c_frame)
{
  ESP_LOGI(TAG, "start VoSPI task...");

  // Allocate space (32-bit multiple for DMA)
  ESP_LOGI(TAG, "preallocating space for segments... (%d bytes)", (sizeof(vospi_frame_t)/4)*4);
  vospi_frame_t* frame = heap_caps_malloc((sizeof(vospi_frame_t)/4)*4, MALLOC_CAP_DMA);
  for (int seg = 0; seg < VOSPI_SEGMENTS_PER_FRAME; seg ++) {
    frame->segments[seg].packet_count = VOSPI_PACKETS_PER_SEGMENT_NORMAL;
  }

  // Initialise SPI hardware
  ESP_LOGI(TAG, "starting VoSPI initialisation...");
  vospi_init(12000000);
  vTaskDelay(3000 / portTICK_RATE_MS);


  while(1) {
    // Synchronise and transfer a single frame
    ESP_LOGI(TAG, "aquiring VoSPI synchronisation");
    if (0 == sync_and_transfer_frame(frame)) {
      ESP_LOGE(TAG, "failed to obtain frame from device.");
      continue;
    }
    ESP_LOGI(TAG, "VoSPI stream synchronised");

    while (1) {
      
      if (!transfer_frame(frame)) {
        ESP_LOGI(TAG, "resynchronising...");
        break;
      }

      // Obtain the semaphore to update the current frame, if we can't update
      // the current frame, just drop this one and try again next time
      if (xSemaphoreTake(c_frame->sem, 0) == pdTRUE) {

        // Copy the frame into place
        memcpy(&c_frame->frame, frame, sizeof(vospi_frame_t));
        xSemaphoreGive(c_frame->sem);

      } else {
        ESP_LOGW(TAG, "couldn't obtain c_frame sem, dropping frame");
      }

    }
  }


}
