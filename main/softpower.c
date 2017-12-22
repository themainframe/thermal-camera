#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "softpower.h"

/*
 * Manages the system software-controlled power state.
 */

static const char* TAG = "SoftPower";

/**
 * Initialise the softpower subsystem.
 */
void softpower_init(bool* pswitch_state)
{
  ESP_LOGI(TAG, "initialise softpower control subsystem");
  gpio_set_direction(PIN_NUM_SP_DISPLAY, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_SP_VOSPI, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_SP_SWITCH, GPIO_MODE_INPUT);

  // Deactivate RTC control of the power switch pin (hang-up from wakeup)
  rtc_gpio_deinit(PIN_NUM_SP_SWITCH);

  // Peripherals off by default
  gpio_set_level(PIN_NUM_SP_DISPLAY, true);
  gpio_set_level(PIN_NUM_SP_VOSPI, true);

  // Get the initial power switch state
  bool power_switch_state = gpio_get_level(PIN_NUM_SP_SWITCH);
  ESP_LOGI(TAG, "INITIAL pswitch state is %d", power_switch_state);
}

/**
 * Get the desired power state.
 * This is influenced by the power switch position, battery charge state etc.
 */
bool softpower_get_desired_state()
{
  return gpio_get_level(PIN_NUM_SP_SWITCH);
}

/**
 * Set up wake-on-pin then enter deep sleep state.
 */
void softpower_deep_sleep()
{
  // Set a wake-up upon power switch -> 1
  esp_sleep_enable_ext0_wakeup(PIN_NUM_SP_SWITCH, 1);

  // Go to sleep (effectively a reset!)
  esp_deep_sleep_start();
}

/**
 * Turn system peripherals on.
 */
void softpower_pf_on()
{
  ESP_LOGI(TAG, "powering ON system peripherals");
  gpio_set_level(PIN_NUM_SP_DISPLAY, false);
  gpio_set_level(PIN_NUM_SP_VOSPI, false);
}

/**
 * Turn system peripherals off.
 */
void softpower_pf_off()
{
  ESP_LOGI(TAG, "powering OFF system peripherals");
  gpio_set_level(PIN_NUM_SP_DISPLAY, true);
  gpio_set_level(PIN_NUM_SP_VOSPI, true);
}
