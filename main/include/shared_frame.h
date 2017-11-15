#ifndef SHARED_FRAME_H
#define SHARED_FRAME_H
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "vospi/vospi.h"

// The shape of the shared frame that is produced by the VoSPI task and consumed by the display task
typedef struct {
  vospi_frame_t frame;
  SemaphoreHandle_t sem;
} c_frame_t;

#endif
