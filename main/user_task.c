#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "softpower.h"

/*
 * This task handles interaction from the user via the touch screen/buttons.
 */

static const char* TAG = "UserTask";

void user_task()
{
  for(;;) {

    // Sniff the power switch state
    if(!softpower_get_desired_state()) {
      // The power switch is off, immediately go into deep sleep...
      ESP_LOGW(TAG, "power switch has entered state <0> - going to deep sleep");
      softpower_pf_off();
      softpower_deep_sleep();
    }

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}
