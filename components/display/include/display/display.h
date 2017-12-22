#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_system.h"

#define VSPI_PIN_NUM_MISO 19
#define VSPI_PIN_NUM_CLK 18
#define VSPI_PIN_NUM_MOSI 23
#define VSPI_PIN_NUM_CS 5
#define PIN_NUM_DISPLAY_DC 22
#define PIN_NUM_DISPLAY_RESET 21

#define DISPLAY_SPI_SPEED_HZ 32000000

typedef struct {
  uint8_t half_pixels[640];
} display_line_t;

typedef struct {
  display_line_t lines[60];
} display_segment_t;

typedef struct {
  uint8_t command;
  uint8_t data[16];
  uint8_t data_len;
} ili_init_cmd_t;

typedef struct {
  bool dc;
  uint8_t dc_io_num;
} spi_transaction_user_data_t;

void display_init();
void display_write_segment(uint8_t seg_position, display_segment_t* segment);

#endif
