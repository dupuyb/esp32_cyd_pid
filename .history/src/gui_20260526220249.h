#pragma once


#define LV_DELAY(x)                                   \
  do {                                                \
  uint32_t t = x;                                     \
    while (t--) {                                     \
      lv_timer_handler();                             \
      delay(1);                                       \
    }                                                 \
  } while (0);
