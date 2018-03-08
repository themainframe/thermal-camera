#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "vospi_task.h"
#include "display_task.h"
#include "gui_task.h"
#include "shared_frame.h"
#include "softpower.h"
#include "cci/cci.h"

static c_frame_t c_frame;

static const char* TAG = "Main";

void app_main()
{
  ESP_LOGI(TAG, "Thermal Camera System startup");

  // Initialise softpower
  softpower_init();

  // Create the semaphore for the current frame
  c_frame.sem = xSemaphoreCreateBinary();

  // The semaphore isn't held to start with
  xSemaphoreGive(c_frame.sem);

  // Turn on the display and camera power
  softpower_pf_on();

  // Start tasks
  // xTaskCreate(&vospi_task, "vospi_task", 30000, &c_frame, 5, NULL);
  // xTaskCreate(&display_task, "display_task", 16000, &c_frame, 5, NULL);
  xTaskCreate(&gui_task, "gui_task", 16000, NULL, 5, NULL);
}
