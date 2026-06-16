#pragma once

#include "Arduino.h"
//#include <DHTesp.h>
#include "dht.h"
#include <lvgl.h>

#define PAGE_MAIN 0
#define PAGE_GRAPH 1
#define PAGE_SAVER 2
#define UI_PAGE_COUNT 3

#define DSP_GRAPH_HISTORY_POINTS 100

extern bool testGraph;
extern bool must_be_saved;
extern uint32_t g_last_pid_compute_ms;

extern lv_obj_t *g_label_control_value;
extern lv_obj_t *g_label_reading_value;
extern lv_obj_t *g_label_humidity_value;
extern lv_obj_t *g_label_pid_state;
extern lv_obj_t *g_label_vmc_state;
extern lv_obj_t *g_label_pwm_value;
extern lv_obj_t *g_led_pwm_state;
extern lv_obj_t *g_label_kp_value;
extern lv_obj_t *g_label_ki_value;
extern lv_obj_t *g_label_kd_value;
extern lv_obj_t *g_switch_pid;
extern lv_obj_t *g_switch_vmc;
extern lv_obj_t *g_label_time;
extern lv_obj_t *g_label_ip_line;
extern lv_obj_t *g_label_mac_line;
extern lv_obj_t *g_label_ss_time;
extern lv_obj_t *g_label_ss_temp;
extern lv_obj_t *g_label_ss_vmc;

extern float g_kp;
extern float g_ki;
extern float g_kd;
extern float g_setpoint_temp_c;
extern float g_pid_integral;
extern float g_pid_prev_error;
extern float g_pwm_percent;
extern bool g_pid_has_prev_error;
extern bool g_pid_enabled;
extern bool g_vmc_manual_on;

extern String currentTime;
extern String rebootTime;

struct TempAndHumidity {
  float temperature;
  float humidity;
};

extern TempAndHumidity sensorData;
extern int pageVisible;

void update_time_label();
void update_pid_value_label(lv_obj_t *label, float value);
void update_access_network_labels(String ip, String mac);
void set_pwm_values();
void update_dht_values();
void update_graph_history();
void switch_callback(lv_obj_t *label, bool is_on);
void control_slider_callback();

void gui_setPage(int page);
void gui_switch_page(int sens);
void lv_create_gui();

void save_pid_settings();
void load_pid_settings_from_file();

// Implemented in main.cpp and used by lv_create_gui() when wiring callbacks.
void vmc_switch_event_callback(lv_event_t *e);
void control_slider_event_callback(lv_event_t *e);
void pid_switch_event_callback(lv_event_t *e);
