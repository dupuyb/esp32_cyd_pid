

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <DHTesp.h>
#include <math.h>

// ============== Framework and Core Includes ==============
// #define DEBUG_FRAMEWEB 
#include "FrameWeb.h"
FrameWeb frame;  // Web server and configuration framework instance

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen - Note: this library doesn't require further configuration
#include <XPT2046_Touchscreen.h>

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define DHT22_PIN 27
#define PWM_OUTPUT_PIN 22
#define PWM_CHANNEL 0
#define PWM_FREQUENCY_HZ 20000
#define PWM_RESOLUTION_BITS 8
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION_BITS) - 1)

#define VMC_ACTIVE_OUT_PIN 26

// CYD boards often expose a discrete RGB LED on these GPIOs.
#define RGB_LED_R_PIN 4
#define RGB_LED_G_PIN 16
#define RGB_LED_B_PIN 17
#define RGB_LED_ACTIVE_HIGH 0

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// If logging is enabled, it will inform the user about what is happening in the library
void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
}

// ============== Time Configuration ==============
/** @brief UTC offset in seconds (3600 = UTC+1 for Central European Time) */
const long gmtOffset_sec = 3600;

/** @brief Daylight saving offset in seconds (3600 = +1 hour for summer time) */
const int daylightOffset_sec = 3600;  // Central European Summer Time (CEST) offset

/** @brief Broken-down time structure (updated every 5 seconds in loop) */
struct tm timeinfo;

/** @brief NTP server pool for time synchronization */
const char* ntpServer = "pool.ntp.org";

// counter for flow duration in seconds
long previousMillis = 0;

String strTime = "00:00:00";

// Get the Touchscreen data
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    z = p.z;

    data->state = LV_INDEV_STATE_PRESSED;

    // Set the coordinates
    data->point.x = x;
    data->point.y = y;
  }
  else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static lv_obj_t * g_label_control_value;
static lv_obj_t * g_label_reading_value;
static lv_obj_t * g_label_humidity_value;
static lv_obj_t * g_label_pid_state;
static lv_obj_t * g_label_vmc_state;
static lv_obj_t * g_label_pwm_value;
static lv_obj_t * g_led_pwm_state;
static lv_obj_t * g_label_kp_value;
static lv_obj_t * g_label_ki_value;
static lv_obj_t * g_label_kd_value;
static lv_obj_t * g_label_time;
static lv_obj_t * g_label_ip_line;
static lv_obj_t * g_label_mac_line;

static DHTesp g_dht;
static uint32_t g_last_dht_read_ms = 0;
static uint32_t g_last_pid_compute_ms = 0;
static float g_kp = 0.123f;
static float g_ki = 5.545f;
static float g_kd = 11.130f;
static float g_setpoint_temp_c = 30.0f;
static float g_current_temp_c = NAN;
static float g_pid_integral = 0.0f;
static float g_pid_prev_error = 0.0f;
static float g_pwm_percent = 0.0f;
static bool g_pid_has_prev_error = false;
static bool g_pid_enabled = true;
static bool g_vmc_manual_on = true;

#include "gui.h"

static pid_adjust_ctx_t g_ctx_kp_minus;
static pid_adjust_ctx_t g_ctx_kp_plus;
static pid_adjust_ctx_t g_ctx_ki_minus;
static pid_adjust_ctx_t g_ctx_ki_plus;
static pid_adjust_ctx_t g_ctx_kd_minus;
static pid_adjust_ctx_t g_ctx_kd_plus;

static String formatIpAddress(const IPAddress& ip) {
  if(ip == IPAddress((uint32_t)0)) {
    return "not connected";
  }
  return ip.toString();
}

static void set_pwm_percent(float percent) {
  if(percent < 0.0f) {
    percent = 0.0f;
  }
  if(percent > 100.0f) {
    percent = 100.0f;
  }

  g_pwm_percent = percent;
  uint32_t duty = (uint32_t)((percent * (float)PWM_MAX_DUTY / 100.0f) + 0.5f);
  ledcWrite(PWM_CHANNEL, duty);

  set_pwm_percent(duty, VMC_ACTIVE_OUT_PIN, percent );
}

static void turn_off_rgb_led(void) {
#if RGB_LED_ACTIVE_HIGH
  const int off_level = LOW;
#else
  const int off_level = HIGH;
#endif

#if RGB_LED_R_PIN != PWM_OUTPUT_PIN
  pinMode(RGB_LED_R_PIN, OUTPUT);
  digitalWrite(RGB_LED_R_PIN, off_level);
#endif
#if RGB_LED_G_PIN != PWM_OUTPUT_PIN
  pinMode(RGB_LED_G_PIN, OUTPUT);
  digitalWrite(RGB_LED_G_PIN, off_level);
#endif
#if RGB_LED_B_PIN != PWM_OUTPUT_PIN
  pinMode(RGB_LED_B_PIN, OUTPUT);
  digitalWrite(RGB_LED_B_PIN, off_level);
#endif
}

static void compute_pid_and_drive_output(void) {
  if(!g_pid_enabled || isnan(g_current_temp_c)) {
    set_pwm_percent(0.0f);
    return;
  }

  uint32_t now = millis();
  float dt_sec = 0.0f;
  if(g_last_pid_compute_ms == 0) {
    dt_sec = 2.0f;
  } else {
    dt_sec = (float)(now - g_last_pid_compute_ms) / 1000.0f;
    if(dt_sec <= 0.0f) {
      dt_sec = 0.001f;
    }
  }
  g_last_pid_compute_ms = now;

  float error = g_setpoint_temp_c - g_current_temp_c;
  g_pid_integral += error * dt_sec;

  // Anti wind-up for bounded 0-100% output.
  if(g_pid_integral > 100.0f) {
    g_pid_integral = 100.0f;
  }
  if(g_pid_integral < -100.0f) {
    g_pid_integral = -100.0f;
  }

  float derivative = 0.0f;
  if(g_pid_has_prev_error) {
    derivative = (error - g_pid_prev_error) / dt_sec;
  }

  float pid_output = (g_kp * error) + (g_ki * g_pid_integral) + (g_kd * derivative);
  g_pid_prev_error = error;
  g_pid_has_prev_error = true;

  set_pwm_percent(pid_output);
}

static void update_dht_values(bool force_update = false) {
  uint32_t now = millis();
  if(!force_update && (now - g_last_dht_read_ms) < 2000U) {
    return;
  }
  g_last_dht_read_ms = now;

  TempAndHumidity data = g_dht.getTempAndHumidity();
  if(isnan(data.temperature) || isnan(data.humidity)) {
    LV_LOG_USER("DHT22 read failed");
    return;
  }

  update_dht_values(data.temperature, data.humidity);
  g_current_temp_c = data.temperature;

  compute_pid_and_drive_output();
}

static void control_slider_event_callback(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  int control_temp_tenth = (int)lv_slider_get_value(slider);  // 25.0 C to 45.0 C
  char temp_text[12];
  format_temp_1_decimal(temp_text, sizeof(temp_text), control_temp_tenth);
  g_setpoint_temp_c = (float)control_temp_tenth / 10.0f;
  control_slider_callback(temp_text, g_setpoint_temp_c);
  compute_pid_and_drive_output();
  LV_LOG_USER("Control setpoint changed to %s C", temp_text);
}

static void pid_switch_event_callback(lv_event_t * e) {
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  g_pid_enabled = is_on;
  switch_callback(g_label_pid_state, is_on);
  //lv_label_set_text(g_label_pid_state, is_on ? "ON" : "OFF");
  //lv_obj_set_style_text_color(g_label_pid_state, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
  if(!is_on) {
    g_pid_integral = 0.0f;
    g_pid_prev_error = 0.0f;
    g_pid_has_prev_error = false;
    // PID disabled: manual VMC switch controls output as 0% or 100%.
    set_pwm_percent(g_vmc_manual_on ? 100.0f : 0.0f);
  } else {
    g_last_pid_compute_ms = 0;
    compute_pid_and_drive_output();
  }
  LV_LOG_USER("PID %s", is_on ? "enabled" : "disabled");
}

static void vmc_switch_event_callback(lv_event_t * e) {
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  g_vmc_manual_on = is_on;
  switch_callback(g_label_vmc_state, is_on);
  //lv_label_set_text(g_label_vmc_state, is_on ? "ON" : "OFF");
  //lv_obj_set_style_text_color(g_label_vmc_state, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
  // VMC switch commands PWM only when PID mode is disabled.
  if(!g_pid_enabled) {
    set_pwm_percent(is_on ? 100.0f : 0.0f);
    LV_LOG_USER("Manual VMC %s -> PWM %.0f%%", is_on ? "ON" : "OFF", g_pwm_percent);
    return;
  }
  LV_LOG_USER("VMC %s", is_on ? "enabled" : "disabled");
}
/*
void lv_create_main_gui(void) {
  static lv_style_t style_screen;
  static lv_style_t style_card;
  static lv_style_t style_title;
  static bool styles_initialized = false;

  if(!styles_initialized) {
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_hex(0xE8EDF2));
    lv_style_set_bg_grad_color(&style_screen, lv_color_hex(0xDCE4EA));
    lv_style_set_bg_grad_dir(&style_screen, LV_GRAD_DIR_VER);

    lv_style_init(&style_card);
    lv_style_set_bg_color(&style_card, lv_color_hex(0xE5E7EB));
    lv_style_set_border_width(&style_card, 1);
    lv_style_set_border_color(&style_card, lv_color_hex(0x7A8A99));
    lv_style_set_radius(&style_card, 0);
    lv_style_set_pad_all(&style_card, 0);

    lv_style_init(&style_title);
    lv_style_set_text_color(&style_title, lv_color_hex(0xFACC15));
    lv_style_set_bg_opa(&style_title, LV_OPA_COVER);
    lv_style_set_bg_color(&style_title, lv_color_hex(0x1F2937));
    lv_style_set_pad_left(&style_title, 6);
    lv_style_set_pad_right(&style_title, 6);
    lv_style_set_pad_top(&style_title, 2);
    lv_style_set_pad_bottom(&style_title, 2);
    lv_style_set_radius(&style_title, 2);
    lv_style_set_text_font(&style_title, &lv_font_montserrat_14);
    styles_initialized = true;
  }

  lv_obj_t * screen = lv_screen_active();
  lv_obj_add_style(screen, &style_screen, 0);
  lv_obj_set_style_pad_all(screen, 0, 0);

  lv_display_t * display = lv_display_get_default();
  int screen_w = (int)lv_display_get_horizontal_resolution(display);
  int screen_h = (int)lv_display_get_vertical_resolution(display);
  int half_w = screen_w / 2;
  int panel_pid_w = screen_w - half_w;
  int top_h = (screen_h * 60) / 100;
  int bottom_h = screen_h - top_h;
  int pid_bottom_line_y = top_h - 30;
  int top_row_1_y = 36;
  int top_row_4_y = pid_bottom_line_y;
  int top_row_spacing = (top_row_4_y - top_row_1_y) / 3;
  int top_row_2_y = top_row_1_y + top_row_spacing;
  int top_row_3_y = top_row_2_y + top_row_spacing;

  lv_obj_t * panel_temp = lv_obj_create(screen);
  lv_obj_set_pos(panel_temp, 0, 0);
  lv_obj_set_size(panel_temp, half_w, top_h);
  lv_obj_add_style(panel_temp, &style_card, 0);

  lv_obj_t * title_temp = lv_label_create(panel_temp);
  lv_label_set_text(title_temp, "Temperature");
  lv_obj_add_style(title_temp, &style_title, 0);
  lv_obj_set_style_text_font(title_temp, &lv_font_montserrat_16, 0);
  lv_obj_set_width(title_temp, lv_pct(100));
  lv_obj_set_style_text_align(title_temp, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(title_temp, 0, 0);

  lv_obj_t * label_reading = lv_label_create(panel_temp);
  lv_label_set_text(label_reading, "Lecture :");
  lv_obj_set_pos(label_reading, 8, top_row_1_y);

  g_label_reading_value = lv_label_create(panel_temp);
  lv_label_set_text(g_label_reading_value, "--.-");
  lv_obj_set_style_text_color(g_label_reading_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_style_text_font(g_label_reading_value, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(g_label_reading_value, 75, top_row_1_y);

  lv_obj_t * label_reading_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_reading_unit, "°C");
  lv_obj_set_pos(label_reading_unit, 130, top_row_1_y);

  lv_obj_t * label_control = lv_label_create(panel_temp);
  lv_label_set_text(label_control, "Control :");
  lv_obj_set_pos(label_control, 8, top_row_2_y);

  g_label_control_value = lv_label_create(panel_temp);
  lv_label_set_text(g_label_control_value, "30.0");
  lv_obj_set_style_text_color(g_label_control_value, color_from_temperature(30.0f), 0);
  lv_obj_set_style_text_font(g_label_control_value, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(g_label_control_value, 75, top_row_2_y);

  lv_obj_t * label_control_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_control_unit, "°C");
  lv_obj_set_pos(label_control_unit, 130, top_row_2_y);

  lv_obj_t * label_humidity = lv_label_create(panel_temp);
  lv_label_set_text(label_humidity, "Humid. :");
  lv_obj_set_pos(label_humidity, 8, top_row_3_y);

  g_label_humidity_value = lv_label_create(panel_temp);
  lv_label_set_text(g_label_humidity_value, "--");
  lv_obj_set_style_text_color(g_label_humidity_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_style_text_font(g_label_humidity_value, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(g_label_humidity_value, 75, top_row_3_y);

  lv_obj_t * label_humidity_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_humidity_unit, "%");
  lv_obj_set_pos(label_humidity_unit, 130, top_row_3_y);

  lv_obj_t * slider_setpoint = lv_slider_create(panel_temp);
  lv_obj_set_pos(slider_setpoint, 8, top_row_4_y + 4);
  lv_obj_set_size(slider_setpoint, half_w - 16, 10);
  lv_slider_set_range(slider_setpoint, 250, 450);
  lv_slider_set_value(slider_setpoint, 300, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_setpoint, control_slider_event_callback, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xD6DEE5), LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0x3B82F6), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xFFFFFF), LV_PART_KNOB);

  lv_obj_t * panel_pid = lv_obj_create(screen);
  lv_obj_set_pos(panel_pid, half_w, 0);
  lv_obj_set_size(panel_pid, panel_pid_w, top_h);
  lv_obj_add_style(panel_pid, &style_card, 0);

  lv_obj_t * title_pid = lv_label_create(panel_pid);
  lv_label_set_text(title_pid, "P.I.D.");
  lv_obj_add_style(title_pid, &style_title, 0);
  lv_obj_set_style_text_font(title_pid, &lv_font_montserrat_16, 0);
  lv_obj_set_width(title_pid, lv_pct(100));
  lv_obj_set_style_text_align(title_pid, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(title_pid, 0, 0);

  // switch PID
  lv_obj_t * switch_pid = lv_switch_create(panel_pid);
  lv_obj_set_pos(switch_pid, 8, top_row_4_y);
  lv_obj_set_size(switch_pid, 50, 24);
  lv_obj_add_state(switch_pid, LV_STATE_CHECKED);
  lv_obj_add_event_cb(switch_pid, pid_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  create_pid_adjuster(panel_pid, top_row_1_y - 4, "Pro.", &g_label_kp_value, &g_kp, 0.010f, 0.000f, 25.000f, &g_ctx_kp_minus, &g_ctx_kp_plus, "Kp");
  create_pid_adjuster(panel_pid, top_row_2_y - 4, "Int.", &g_label_ki_value, &g_ki, 0.050f, 0.000f, 50.000f, &g_ctx_ki_minus, &g_ctx_ki_plus, "Ki");
  create_pid_adjuster(panel_pid, top_row_3_y - 4, "Dev.", &g_label_kd_value, &g_kd, 0.050f, 0.000f, 50.000f, &g_ctx_kd_minus, &g_ctx_kd_plus, "Kd");

  g_label_pid_state = lv_label_create(panel_pid);
  lv_label_set_text(g_label_pid_state, "ON");
  lv_obj_set_style_text_color(g_label_pid_state, lv_color_hex(0x107C10), 0);
  lv_obj_set_pos(g_label_pid_state, panel_pid_w - 34, top_row_4_y + 4);

  lv_obj_t * panel_vmc = lv_obj_create(screen);
  lv_obj_set_pos(panel_vmc, 0, top_h);
  lv_obj_set_size(panel_vmc, half_w, bottom_h);
  lv_obj_add_style(panel_vmc, &style_card, 0);

  int lower_row_1_y = 34;
  int lower_row_2_y = 64;

  lv_obj_t * label_vmc = lv_label_create(panel_vmc);
  lv_label_set_text(label_vmc, "V.M.C.");
  lv_obj_add_style(label_vmc, &style_title, 0);
  lv_obj_set_style_text_font(label_vmc, &lv_font_montserrat_16, 0);
  lv_obj_set_width(label_vmc, lv_pct(100));
  lv_obj_set_style_text_align(label_vmc, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(label_vmc, 0, 0);

  lv_obj_t * switch_vmc = lv_switch_create(panel_vmc);
  lv_obj_set_pos(switch_vmc, 8, lower_row_1_y);
  lv_obj_set_size(switch_vmc, 50, 24);
  lv_obj_add_state(switch_vmc, LV_STATE_CHECKED);
  lv_obj_add_event_cb(switch_vmc, vmc_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t * label_pwm = lv_label_create(panel_vmc);
  lv_label_set_text(label_pwm, "Vitesse :");
  lv_obj_set_pos(label_pwm, 8, lower_row_2_y);

  g_label_pwm_value = lv_label_create(panel_vmc);
  lv_label_set_text(g_label_pwm_value, "000");
  lv_obj_set_style_text_color(g_label_pwm_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(g_label_pwm_value, 75, lower_row_2_y);

  lv_obj_t * label_pwm_unit = lv_label_create(panel_vmc);
  lv_label_set_text(label_pwm_unit, "%");
  lv_obj_set_style_text_color(label_pwm_unit, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(label_pwm_unit, 130, lower_row_2_y);

  g_led_pwm_state = lv_obj_create(panel_vmc);
  lv_obj_set_size(g_led_pwm_state, 28, 28);
  lv_obj_set_pos(g_led_pwm_state, 110, lower_row_1_y);
  lv_obj_set_style_radius(g_led_pwm_state, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(g_led_pwm_state, 1, 0);
  lv_obj_set_style_border_color(g_led_pwm_state, lv_color_hex(0x7A8A99), 0);
  lv_obj_set_style_bg_opa(g_led_pwm_state, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(g_led_pwm_state, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_pad_all(g_led_pwm_state, 0, 0);

  g_label_vmc_state = lv_label_create(panel_vmc);
  lv_label_set_text(g_label_vmc_state, "ON");
  lv_obj_set_style_text_color(g_label_vmc_state, lv_color_hex(0x107C10), 0);
  lv_obj_set_pos(g_label_vmc_state, 75, lower_row_1_y + 4);

  lv_obj_t * panel_status = lv_obj_create(screen);
  lv_obj_set_pos(panel_status, half_w, top_h);
  lv_obj_set_size(panel_status, screen_w - half_w, bottom_h);
  lv_obj_add_style(panel_status, &style_card, 0);

  lv_obj_t * label_access = lv_label_create(panel_status);
  lv_label_set_text(label_access, "Acces");
  lv_obj_add_style(label_access, &style_title, 0);
  lv_obj_set_style_text_font(label_access, &lv_font_montserrat_16, 0);
  lv_obj_set_width(label_access, lv_pct(100));
  lv_obj_set_style_text_align(label_access, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(label_access, 0, 0);

  g_label_time = lv_label_create(panel_status);
  lv_label_set_text(g_label_time, strTime.c_str());
  lv_obj_set_style_text_font(g_label_time, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(g_label_time, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_width(g_label_time, lv_pct(100));
  lv_obj_set_style_text_align(g_label_time, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(g_label_time, 0, lower_row_1_y);

  g_label_ip_line = lv_label_create(panel_status);
  lv_label_set_text(g_label_ip_line, "IP: not connected");
  lv_obj_set_width(g_label_ip_line, lv_pct(100));
  lv_obj_set_style_text_align(g_label_ip_line, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(g_label_ip_line, 0, lower_row_2_y);

  g_label_mac_line = lv_label_create(panel_status);
  lv_label_set_text(g_label_mac_line, "Mc:--:--:--:--:--:--");
  lv_obj_set_width(g_label_mac_line, lv_pct(100));
  lv_obj_set_style_text_align(g_label_mac_line, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(g_label_mac_line, 0, lower_row_2_y - 15);

  update_access_network_labels(formatIpAddress(WiFi.localIP()) , WiFi.macAddress());

  lv_obj_t * separator_v = lv_obj_create(screen);
  lv_obj_remove_style_all(separator_v);
  lv_obj_set_pos(separator_v, half_w, 0);
  lv_obj_set_size(separator_v, 1, screen_h);
  lv_obj_set_style_bg_color(separator_v, lv_color_hex(0x7A8A99), 0);

  lv_obj_t * separator_h = lv_obj_create(screen);
  lv_obj_remove_style_all(separator_h);
  lv_obj_set_pos(separator_h, 0, top_h);
  lv_obj_set_size(separator_h, screen_w, 1);
  lv_obj_set_style_bg_color(separator_h, lv_color_hex(0x7A8A99), 0);
}

*/

// ============== FrameWeb Framework Callbacks ==============
/** @brief Called when configuration is saved via web interface (not used in this app) */
void saveConfigCallback() {}

/** @brief Called when WebSocket events occur (not used in this app) */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {}

/** @brief Called when WiFi enters AP mode for configuration (not used in this app) */
void configModeCallback (WiFiManager *myWiFiManager) {}

void setup() {
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.begin(115200);
  Serial.println(LVGL_Arduino);

  // Try to keep onboard RGB LED off at startup.
  turn_off_rgb_led();
  
  // Start LVGL
  lv_init();
  // Register print function for debugging
  lv_log_register_print_cb(log_print);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation for 320x240 landscape rendering.
  // If touch is mirrored on your hardware, try touchscreen.setRotation(0).
  touchscreen.setRotation(2);

  // Create a display object
  lv_display_t * disp;
  // Initialize the TFT display using the TFT_eSPI library
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);
    
  // Initialize an LVGL input device object (Touchscreen)
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  // Set the callback function to read Touchscreen input
  lv_indev_set_read_cb(indev, touchscreen_read);

  // Initialize DHT22 sensor on GPIO23
  g_dht.setup(DHT22_PIN, DHTesp::DHT22);

  // Initialize PWM output on GPIO4, duty 0..255 for 0..100%.
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
  pinMode(VMC_ACTIVE_OUT_PIN, OUTPUT);
  digitalWrite(VMC_ACTIVE_OUT_PIN, LOW);
  set_pwm_percent(0.0f);

  // Function to draw the GUI (text, buttons and sliders)
  lv_create_gui();
  update_dht_values(true);

  // Initialize FrameWeb framework (web server, WebSocket, config management)
  //frame.setFlag();
  frame.setup();

  // Configure system time from NTP server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  // Start NTP sync
  // Set timezone: CET-1CEST,M3.5.0,M10.5.0/3 = Central European Time with DST
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();  // Apply timezone settings

  // Read current time from system
  getLocalTime(&timeinfo);
}

void loop() {

  // Update FrameWeb framework (web server, WebSocket, etc.)
  frame.loop();

  update_dht_values();
  lv_task_handler();  // let the GUI do its work
  lv_tick_inc(5);     // tell LVGL how much time has passed
  delay(5);           // let this time pass

  // Alive section, executed every second
  if ( millis() - previousMillis > 1000L) {
    previousMillis = millis();
    // Update time display every second
    if(getLocalTime(&timeinfo)) {
      char temp[9];
      strftime(temp, sizeof(temp), "%H:%M:%S", &timeinfo);
      strTime = String(temp);
      if(g_label_time != NULL) {
        lv_label_set_text(g_label_time, strTime.c_str());
      }
    }

      update_access_network_labels(formatIpAddress(WiFi.localIP()), WiFi.macAddress());


  }
}
