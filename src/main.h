#pragma once

#ifndef ESP32_CYD_PID_MAIN_H_
#define ESP32_CYD_PID_MAIN_H_

#include "Arduino.h"
#include <lvgl.h>
#include "FrameWeb.h"

// Local helpers and callbacks defined in src/main.cpp
void log_print(lv_log_level_t level, const char *buf);
void touchscreen_read(lv_indev_t *indev, lv_indev_data_t *data);

static void set_backlight_percent(float percent);
static String formatIpAddress(const IPAddress &ip);
static void append_history_sample(float temperature_c, float pwm_percent);
static void webSocketSendState(uint8_t num, bool include_history);
static void webSocketSendError(uint8_t num, const char *message);
static void set_pwm_percent(float percent);
static void decodePidJsonAndApply(uint8_t num, uint8_t *payload, size_t length);
static void turn_off_rgb_led(void);
void compute_pid_and_drive_output(void);
void update_dht_values();
void control_slider_event_callback(lv_event_t *e);
void pid_switch_event_callback(lv_event_t *e);
void vmc_switch_event_callback(lv_event_t *e);
static void refresh_external_html_tools();

void lv_create_main_gui(void);

// Framework callbacks
void saveConfigCallback();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void configModeCallback(WiFiManager *myWiFiManager);

void IRAM_ATTR startingTask(void *pvParameter);

// Arduino entry points
void setup();
void loop();

#endif // ESP32_CYD_PID_MAIN_H_
