#include "soul_link/utils.h"
#include <stdint.h>

float uint16_to_float(uint16_t x, float x_min, float x_max, int bits) {
  uint32_t span = (1 << bits) - 1;
  float offset = x_max - x_min;
  return offset * x / span + x_min;
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Byte_to_float(uint8_t *bytedata) {
  uint32_t data =
      bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
  float data_float = *(float *)(&data);
  return data_float;
}
