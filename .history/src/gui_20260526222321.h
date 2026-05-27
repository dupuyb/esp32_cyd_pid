#pragma once

#include <lvgl.h>

#define LV_DELAY(x)                                   \
  do {                                                \
    uint32_t t = x;                                   \
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

static void create_pid_adjuster (
  lv_obj_t * parent,
  int y,
  const char * caption,
  lv_obj_t ** value_label,
  float * value,
  float step,
  float min_value,
  float max_value,
  pid_adjust_ctx_t * ctx_minus,
  pid_adjust_ctx_t * ctx_plus,
  const char * name
);
static void update_pid_value_label(lv_obj_t * label, float value);
static void pid_adjust_button_event_callback(lv_event_t * e);