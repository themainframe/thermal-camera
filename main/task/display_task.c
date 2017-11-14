#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "vospi/vospi.h"
#include "shared_frame.h"
#include "display/display.h"

/*
 * This task drives the display update pipeline.
 *
 * The frame semaphore is awaited. Once a frame is available, we preprocess it a
 * segment at a time then draw each segment to the display.
 */

#define RGB_TO_16BIT(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

static const char* TAG = "DisplayTask";

static vospi_frame_t c_frame_buf;

uint8_t map_to_byte(uint16_t in, uint16_t in_min, uint16_t in_max)
{
  return (in - in_min) * (254 - 20) / (in_max - in_min) + 20;
}

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
        display_segment_t* disp_segment = malloc(sizeof(disp_segment));

        // Each segment contains 30 lines consisting of 2 packets each
        uint16_t offset = 0;
        for (int8_t line = 0; line < VOSPI_PACKETS_PER_SEGMENT_NORMAL / 2; line ++) {
          // Copy data out of the two packets *twice* for this line, scaling them too
          for (uint8_t line_cpy = 0; line_cpy < 2; line_cpy ++) {
            for (uint8_t pkt = 0; pkt < 2; pkt ++) {
              for (uint8_t sym = 0; sym < VOSPI_PACKET_SYMBOLS; sym += 2) {

                uint16_t pix_value = c_frame_buf.segments[seg].packets[line * 2 + pkt].symbols[sym] << 8 |
                  c_frame_buf.segments[seg].packets[line * 2 + pkt].symbols[sym + 1];
                uint8_t scaled_v = 254 - map_to_byte(pix_value, max + 2, min + 1);
                uint16_t colour_v = RGB_TO_16BIT(fc_map[scaled_v][0], fc_map[scaled_v][1], fc_map[scaled_v][2]);

                // Write the same pixel twice, we're doubling width
                disp_segment[offset ++] = colour_v >> 8;
                disp_segment[offset ++] = colour_v & 0xff;
                disp_segment[offset ++] = colour_v >> 8;
                disp_segment[offset ++] = colour_v & 0xff;
              }
            }
          }
        }

        // TODO: Process any graphical additions for this segment

        // Set up the graphic RAM page position for this segment
        ili_command(device, ILI9341_PASET);
        uint8_t paset_d[4] = {
          seg * 60 >> 8,
          seg * 60 & 0xff,
          (seg + 1) * 60 >> 8,
          (seg + 1) * 60 & 0xff
        };
        ili_data(device, paset_d, 4);

        // Draw the pixel values for the segment to the display
        ili_command(device, ILI9341_RAMWR);
        ili_data(device, disp_segment_data, (VOSPI_PACKETS_PER_SEGMENT_NORMAL * VOSPI_PACKET_SYMBOLS * 4));

        // Free this segment's pixel values
        free(disp_segment_data);
      }
    }
  }

  while (true) ;
}
