#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vospi_task.h"

void app_main()
{
  xTaskCreate(&vospi_task, "vospitask", 10000, NULL, 5, NULL);
}
