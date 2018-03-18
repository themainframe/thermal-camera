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
#include "display/gui/gui.h"
#include "display/render.h"

/*
 * This task handles interaction from the user via the touch screen/buttons.
 */

static const char* TAG = "UserTask";

void user_task()
{
  // Add a rectangle component to the GUI (testing)
  gui_comp_t* heartbeat_pip_comp = malloc(sizeof(gui_comp_t));
  heartbeat_pip_comp->visible = true;
  heartbeat_pip_comp->left = 10;
  heartbeat_pip_comp->top = 10;
  heartbeat_pip_comp->text = NULL;
  heartbeat_pip_comp->rectangle = malloc(sizeof(gui_comp_rectangle_t));
  heartbeat_pip_comp->rectangle->width = 10;
  heartbeat_pip_comp->rectangle->height = 8;
  heartbeat_pip_comp->rectangle->fill_colour = RGB_TO_16BIT(0, 255, 0);
  gui_add_comp(heartbeat_pip_comp);

  for(;;) {

    #if !CONFIG_NO_SOFTPOWER
    // Sniff the power switch state
    if(!softpower_get_desired_state()) {
      // The power switch is off, immediately go into deep sleep...
      ESP_LOGW(TAG, "power switch has entered state <0> - going to deep sleep");
      softpower_pf_off();
      softpower_deep_sleep();
    }
    #endif

    // Flash the heartbeat pip
    heartbeat_pip_comp->visible = !heartbeat_pip_comp->visible;

    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
