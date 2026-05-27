#ifndef GUI_H
#define GUI_H

#include <lvgl.h>

// Main GUI builder
void lv_create_main_gui(void);

static lv_color_t color_from_temperature(float temp_c);

// Global LVGL object handles used by the UI
extern lv_obj_t * g_label_control_value;
extern lv_obj_t * g_label_reading_value;
extern lv_obj_t * g_label_humidity_value;
extern lv_obj_t * g_label_pid_state;
extern lv_obj_t * g_label_vmc_state;
extern lv_obj_t * g_label_pwm_value;
extern lv_obj_t * g_led_pwm_state;
extern lv_obj_t * g_label_kp_value;
extern lv_obj_t * g_label_ki_value;
extern lv_obj_t * g_label_kd_value;
extern lv_obj_t * g_label_time;
extern lv_obj_t * g_label_ip_line;
extern lv_obj_t * g_label_mac_line;

#endif
