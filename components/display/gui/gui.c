#include <stdlib.h>
#include <string.h>
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "display/gui/gui.h"
#include "display/gui/gui.h"
#include "display/render.h"
#include "display/fonts/font8x8_basic.h"

static const char* TAG = "GUI";

// The head of the GUI component list
gui_comp_list_t* gui_comp_head = NULL;

/**
 * Add an item to the current GUI.
 */
void gui_add_comp(gui_comp_t* comp)
{
  // Allocate a new list item and attach the comp to it
  gui_comp_list_t* link = malloc(sizeof(gui_comp_list_t));
  link->comp = comp;
  link->next = gui_comp_head;
  gui_comp_head = link;
}

/**
 * Render any GUI artifacts in the current segment.
 */
void gui_render_to_segment(uint8_t seg, display_segment_t* disp_segment)
{
  // Iterate over the GUI comps in the list and render them
  gui_comp_list_t* current = gui_comp_head;

  while (current != NULL) {
    render_comp(current->comp, seg, disp_segment);
    current = current->next;
  }
}

/**
 * Render a single GUI component in the current segment.
 */
void render_comp(gui_comp_t* comp, uint8_t seg, display_segment_t* disp_segment)
{
  ESP_LOGD(TAG, "drawing comp @ top: %d, left: %d ", comp->top, comp->left);

  if (comp->rectangle != NULL) {
    ESP_LOGD(
      TAG, "...comp has rectangle with dims: width: %d, height: %d",
      comp->rectangle->width, comp->rectangle->height
    );
    render_rectangle(comp, seg, disp_segment);
  }
}

/**
 * Render a rectangle in the current segment.
 */
void render_rectangle(gui_comp_t* comp, uint8_t seg, display_segment_t* disp_segment)
{
  // Is the rectangle entirely outside this segment?
  if (comp->top + comp->rectangle->height < SEG_START(seg) || comp->top > SEG_END(seg)) {
    ESP_LOGD(TAG, "nothing to draw in segment %d", seg);
    return;
  }

  ESP_LOGD(TAG, "drawing segment %d", seg);

  uint8_t start_line = SEG_START(seg) > comp->top ? 0 : comp->top % 60;
  uint8_t end_line = comp->top + comp->rectangle->height >= SEG_END(seg) ?
    60 : ((comp->top + comp->rectangle->height) % 60);

  ESP_LOGD(TAG, "start line: %d, end line: %d", start_line, end_line);

  for (uint8_t line = start_line; line < end_line; line ++) {
    // Fill the pixels covered by the rectangle on this line
    for (uint16_t column = comp->left; column < comp->left + comp->rectangle->width; column ++) {
      // Set the two values for this pixel
      disp_segment->lines[line].half_pixels[column * 2] = comp->rectangle->fill_colour >> 8;
      disp_segment->lines[line].half_pixels[column * 2 + 1] = comp->rectangle->fill_colour & 0xff;
    }
  }
}

/**
 * Render a text string in the current segment.
 */
void render_text(gui_comp_t* comp, uint8_t seg, display_segment_t* display_segment)
{
  // Is the text entirely outside this segment?
  if (comp->top + 8 < SEG_START(seg) || comp->top > SEG_END(seg)) {
    ESP_LOGD(TAG, "nothing to draw in segment %d", seg);
    return;
  }

  uint8_t start_line = SEG_START(seg) > comp->top ? 0 : comp->top % 60;
  uint8_t end_line = comp->top + 8 >= SEG_END(seg) ?
    60 : ((comp->top + 8) % 60);

  // Draw each character
  for (uint8_t c = 0; c < strlen(comp->text->text); c ++) {
    for (uint8_t line = start_line; line < end_line; line ++) {

      // Derive the line of the actual character (0 = top, 7 = last)
    }
  }
}
