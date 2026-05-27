#pragma once

#include <lvgl.h>

#define LV_DELAY(x)                                   \
  do {                                                \
  uint32_t t = x;                                     \
    while (t--) {                                     \
      lv_timer_handler();                             \
      delay(1);                                       \
    }                                                 \
  } while (0);

typedef struct {
  float * value;
  float step;
  float min_value;
  float max_value;
  lv_obj_t * label;
  const char * name;
} pid_adjust_ctx_t;