#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "vospi/vospi.h"
#include "display/display.h"
#include "shared_frame.h"
#include "display/gui/gui.h"
#include "display/palettes/palettes.h"
#include "display/render.h"

/*
 * This task drives the display update pipeline.
 *
 * The frame semaphore is awaited. Once a frame is available, we preprocess it a
 * segment at a time then draw each segment to the display.
 */

static const char* TAG = "DisplayTask";
static vospi_frame_t c_frame_buf;

void display_task(c_frame_t* c_frame)
{
  ESP_LOGI(TAG, "start DisplayTask...");
  display_init();

  for (;;) {

    // Wait for a frame to be available
    if (xSemaphoreTake(c_frame->sem, 1000) == pdTRUE) {

      // Quickly copy the frame into our local buffer to release the sem faster
      memcpy(&c_frame_buf, &c_frame->frame, sizeof(vospi_frame_t));
      xSemaphoreGive(c_frame->sem);

      // Perform a preliminary scan to establish the pixel value range
      uint16_t max = 0, min = UINT16_MAX;
      for (uint8_t seg = 0; seg < VOSPI_SEGMENTS_PER_FRAME; seg ++) {
        for (uint8_t pkt = 0; pkt < VOSPI_PACKETS_PER_SEGMENT_NORMAL; pkt ++) {
          for (uint8_t sym = 0; sym < VOSPI_PACKET_SYMBOLS; sym += 2) {
            uint16_t pix_val = c_frame_buf.segments[seg].packets[pkt].symbols[sym] << 8 |
              c_frame_buf.segments[seg].packets[pkt].symbols[sym + 1];
            max = pix_val > max ? pix_val : max;
            min = pix_val < min ? pix_val : min;
          }
        }
      }

      // Each frame contains 4 segments
      for (uint8_t seg = 0; seg < VOSPI_SEGMENTS_PER_FRAME; seg ++) {

        // Allocate space for pixel values for the display
        // There will be 4x the bytes because we're scaling to fit a display twice the size
        display_segment_t* disp_segment = malloc(sizeof(display_segment_t));

        // Render the VoSPI segment onto a display segment
        render_vospi_segment(&c_frame_buf.segments[seg], disp_segment, &palettes[0], min, max);

        // Draw any graphical additions for this segment ...
        gui_render_to_segment(seg, disp_segment);

        // Set up the graphic RAM page position for this segment
        display_write_segment(seg, disp_segment);

        // Free this segment's pixel values
        free(disp_segment);
      }
    }
  }

  while (true) ;
}
