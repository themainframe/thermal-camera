#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "display/render.h"
#include "vospi/vospi.h"
#include "display/display.h"
#include "display/palettes/palettes.h"

/**
 * Map a single 16-bit unsigned value to a single byte, given an input range.
 */
uint8_t map_to_byte(uint16_t in, uint16_t in_min, uint16_t in_max)
{
  return (in - in_min) * 254 / (in_max - in_min);
}

/**
 * Render a VoSPI segment to a display segment, performing colour mapping and
 * scaling as required.
 *
 * TODO: Describe the configuration as a config struct passed in rather than
 * separate parameters.
 */
void render_vospi_segment(vospi_segment_t* in, display_segment_t* out, palette_t* palette, uint16_t min, uint16_t max)
{
  // Each segment contains 30 lines consisting of 2 packets each
  for (int8_t line = 0; line < VOSPI_PACKETS_PER_SEGMENT_NORMAL / 2; line ++) {   // One iteration per line

    // Copy data out of the two packets
    for (uint8_t pkt = 0; pkt < 2; pkt ++) {
      for (uint8_t sym = 0; sym < VOSPI_PACKET_SYMBOLS; sym += 2) {

        uint16_t pix_value = in->packets[line * 2 + pkt].symbols[sym] << 8 |
          in->packets[line * 2 + pkt].symbols[sym + 1];
        uint8_t scaled_v = 254 - map_to_byte(pix_value, max + 2, min + 1);

        // Use the provided pointer to a colour palette if available, otherwise greyscale
        uint16_t colour_v;
        if (palette) {
          colour_v = RGB_TO_16BIT(
            (*(palette->map_ptr))[scaled_v][0],
            (*(palette->map_ptr))[scaled_v][1],
            (*(palette->map_ptr))[scaled_v][2]
          );
        } else {
          colour_v = RGB_TO_16BIT(scaled_v, scaled_v, scaled_v);
        }

        // Write the same pixel twice, we're doubling width and replicate it onto the line below too
        out->lines[line * 2].half_pixels[(320 * pkt) + (sym * 2)] = colour_v >> 8;
        out->lines[line * 2].half_pixels[(320 * pkt) + (sym * 2) + 1] = colour_v & 0xff;
        out->lines[line * 2].half_pixels[(320 * pkt) + (sym * 2) + 2] = colour_v >> 8;
        out->lines[line * 2].half_pixels[(320 * pkt) + (sym * 2) + 3] = colour_v & 0xff;
      }
    }

    memcpy(&out->lines[line * 2 + 1], &out->lines[line * 2], sizeof(display_line_t));
  }
}
