#ifndef PALETTES_H
#define PALETTES_H

#include <stdint.h>

#define PALETTE_COUNT 1

typedef const uint8_t palette_map_t[255][3];

typedef struct {
  char name[32];
  palette_map_t* map_ptr;
} palette_t;

extern palette_t palettes[PALETTE_COUNT];

#endif
