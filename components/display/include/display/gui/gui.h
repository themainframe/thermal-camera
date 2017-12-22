#ifndef GUI_H
#define GUI_H

#include <stdint.h>
#include "esp_system.h"
#include "display/display.h"

#define DISPLAY_SEGMENT_H 60
#define SEG_START(seg) DISPLAY_SEGMENT_H * seg
#define SEG_END(seg) DISPLAY_SEGMENT_H * (seg + 1)

typedef struct {
  uint16_t colour;
  char text[64];
} gui_comp_text_t;

typedef struct {
  uint16_t width;
  uint8_t height;
  uint16_t fill_colour;
} gui_comp_rectangle_t;

typedef struct {
  bool visible;
  uint16_t left;
  uint8_t top;
  gui_comp_rectangle_t* rectangle;
  gui_comp_text_t* text;
  void (*touch_cb);
} gui_comp_t;

typedef struct gui_comp_list {
  gui_comp_t* comp;
  struct gui_comp_list* next;
} gui_comp_list_t;

void gui_add_comp(gui_comp_t* comp);
void gui_render_to_segment(uint8_t seg, display_segment_t* disp_segment);
void render_comp(gui_comp_t* comp, uint8_t seg, display_segment_t* disp_segment);
void render_rectangle(gui_comp_t* comp, uint8_t seg, display_segment_t* disp_segment);
void render_text(gui_comp_t* comp, uint8_t seg, display_segment_t* disp_segment);

#endif
