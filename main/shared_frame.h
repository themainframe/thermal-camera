#ifndef SHARED_FRAME_H
#define SHARED_FRAME_H
#include "vospi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
  vospi_frame_t frame;
  SemaphoreHandle_t sem;
} c_frame_t;

#endif
