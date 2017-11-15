#ifndef RENDER_H
#define RENDER_H

#include "vospi/vospi.h"
#include "display/display.h"
#include "display/palettes/palettes.h"

#define RGB_TO_16BIT(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

void render_vospi_segment(vospi_segment_t* in, display_segment_t* out, palette_t* palette, uint16_t min, uint16_t max);

#endif
