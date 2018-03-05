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
#include "cci/cci.h"

/*
 * This task handles user interaction/GUI updating.
 */

static const char* TAG = "GUITask";

void gui_task()
{
  // Add a rectangle component to the GUI (testing)
  gui_comp_t* heartbeat_pip_comp = calloc(1, sizeof(gui_comp_t));
  heartbeat_pip_comp->visible = true;
  heartbeat_pip_comp->left = 10;
  heartbeat_pip_comp->top = 10;
  heartbeat_pip_comp->rectangle = malloc(sizeof(gui_comp_rectangle_t));
  heartbeat_pip_comp->rectangle->width = 10;
  heartbeat_pip_comp->rectangle->height = 8;
  heartbeat_pip_comp->rectangle->fill_colour = RGB_TO_16BIT(0, 255, 0);
  gui_add_comp(heartbeat_pip_comp);

  // Add some text to the display
  gui_comp_t* text_comp = calloc(1, sizeof(gui_comp_t));
  text_comp->visible = true;
  text_comp->left = 30;
  text_comp->top = 10;
  text_comp->text = malloc(sizeof(gui_comp_text_t));
  strcpy(text_comp->text->text, "RUN");
  text_comp->text->colour = RGB_TO_16BIT(0, 255, 0);
  gui_add_comp(text_comp);

  // Set up the CCI
  cci_init();

  for(;;) {

    // Detect the FFC state
    ESP_LOGI(TAG, "FFC state: %d", cci_get_ffc_state());

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

    vTaskDelay(3000 / portTICK_RATE_MS);
  }
}
