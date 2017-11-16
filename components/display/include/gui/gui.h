#ifndef GUI_H
#define GUI_H

typedef struct {
  gui_comp_t* comp;
  gui_comp_list_t* next;
} gui_comp_list_t;

typedef struct {
  uint16_t left;
  uint16_t top;
  gui_comp_rectangle_t* rectangle;
  void (*touch_cb);
} gui_comp_t;

typedef struct {
  uint16_t width;
  uint16_t height;
  bool is_filled;
  uint16_t line_colour;
  uint16_t fill_colour;
} gui_comp_rectangle_t;

#endif
