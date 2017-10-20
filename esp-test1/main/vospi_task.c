#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
static const char* TAG = "VOSPITask";

void vospi_task()
{
    ESP_LOGI(TAG, "start VoSPI task...");
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    int level = 0;
    while (1) {
        ESP_LOGI(TAG, "blink (%d)", level);
        gpio_set_level(GPIO_NUM_5, level = !level);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}
