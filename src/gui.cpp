#include "gui.h"
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

bool must_be_saved = false;
static const char *PID_SETTINGS_FILE = "/pid.json";

// Shared runtime state used by the GUI and control logic.
uint32_t g_last_pid_compute_ms = 0;

// Main page widgets.
lv_obj_t *g_label_control_value;
lv_obj_t *g_label_reading_value;
lv_obj_t *g_label_humidity_value;
lv_obj_t *g_label_pid_state;
lv_obj_t *g_label_vmc_state;
lv_obj_t *g_label_pwm_value;
lv_obj_t *g_led_pwm_state;
lv_obj_t *g_label_kp_value;
lv_obj_t *g_label_ki_value;
lv_obj_t *g_label_kd_value;
lv_obj_t *g_switch_pid;
lv_obj_t *g_switch_vmc;
lv_obj_t *g_label_time;
lv_obj_t *g_label_ip_line;
lv_obj_t *g_label_mac_line;
lv_obj_t *g_label_ss_time;
lv_obj_t *g_label_ss_temp;
lv_obj_t *g_label_ss_vmc;
lv_obj_t *chart;
lv_chart_series_t *ser1;
lv_chart_series_t *ser2;

// PID and process state.
float g_kp = 0.123f;
float g_ki = 5.545f;
float g_kd = 11.130f;
float g_setpoint_temp_c = 30.0f;
float g_pid_integral = 0.0f;
float g_pid_prev_error = 0.0f;
float g_pwm_percent = 0.0f;
bool g_pid_has_prev_error = false;
bool g_pid_enabled = true;
bool g_vmc_manual_on = true;

// Graphical data.
String currentTime = "HH:MM:SS";
TempAndHumidity sensorData;

// Generic context passed to PID +/- button callbacks.
typedef struct {
  float *value;
  float step;
  float min_value;
  float max_value;
  lv_obj_t *label;
  const char *name;
} pid_adjust_ctx_t;

// PID +/- buttons contexts.
static pid_adjust_ctx_t g_ctx_kp_minus;
static pid_adjust_ctx_t g_ctx_kp_plus;
static pid_adjust_ctx_t g_ctx_ki_minus;
static pid_adjust_ctx_t g_ctx_ki_plus;
static pid_adjust_ctx_t g_ctx_kd_minus;
static pid_adjust_ctx_t g_ctx_kd_plus;

// Mirrors the current time on both the main page and screen saver page.
void update_time_label() {
  if (g_label_time != NULL) {
    lv_label_set_text(g_label_time, currentTime.c_str());
  }
  if (g_label_ss_time != NULL) {
    lv_label_set_text(g_label_ss_time, currentTime.c_str());
  }
}

void update_pid_value_label(lv_obj_t *label, float value) {
  // Keep all PID labels formatted with 3 decimals for consistent tuning feedback.
  char value_text[16];
  lv_snprintf(value_text, sizeof(value_text), "%.3f", value);
  lv_label_set_text(label, value_text);
}

static void pid_adjust_button_event_callback(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  pid_adjust_ctx_t *ctx = (pid_adjust_ctx_t *)lv_event_get_user_data(e);
  // Generic callback shared by +- buttons for Kp, Ki, Kd.
  float new_value = *(ctx->value) + ctx->step;
  if (new_value < ctx->min_value) {
    new_value = ctx->min_value;
  }
  if (new_value > ctx->max_value) {
    new_value = ctx->max_value;
  }

  *(ctx->value) = new_value;
  update_pid_value_label(ctx->label, new_value);
  must_be_saved = true;
  LV_LOG_USER("%s changed to %.3f", ctx->name, new_value);
}

static void format_temp_1_decimal(char *out, size_t out_size, int tenth_degrees) {
  int whole = tenth_degrees / 10;
  int decimal = tenth_degrees % 10;
  lv_snprintf(out, out_size, "%d.%d", whole, decimal);
}

// UI color ramp used for temperature values.
static lv_color_t color_from_temperature(float temp_c) {
  if (temp_c < 28.0f) {
    return lv_color_hex(0x1565C0);
  }
  if (temp_c <= 35.0f) {
    return lv_color_hex(0x107C10);
  }
  if (temp_c <= 40.0f) {
    return lv_color_hex(0xE67E22);
  }
  return lv_color_hex(0xC62828);
}

static void create_pid_adjuster(lv_obj_t *parent, int y, const char *caption, lv_obj_t **value_label, float *value, float step, float min_value, float max_value,
                                pid_adjust_ctx_t *ctx_minus, pid_adjust_ctx_t *ctx_plus, const char *name) {
  // Reusable PID row builder: caption, value label, minus and plus buttons.
  lv_obj_t *label_caption = lv_label_create(parent);
  lv_label_set_text(label_caption, caption);
  lv_obj_set_pos(label_caption, 8, y + 4);

  *value_label = lv_label_create(parent);
  lv_obj_set_style_text_color(*value_label, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(*value_label, 85, y + 4);
  update_pid_value_label(*value_label, *value);

  lv_obj_t *btn_minus = lv_button_create(parent);
  lv_obj_set_size(btn_minus, 26, 22);
  lv_obj_set_pos(btn_minus, 55, y);
  lv_obj_t *lbl_minus = lv_label_create(btn_minus);
  lv_label_set_text(lbl_minus, "<<");
  lv_obj_center(lbl_minus);

  lv_obj_t *btn_plus = lv_button_create(parent);
  lv_obj_set_size(btn_plus, 26, 22);
  lv_obj_set_pos(btn_plus, 130, y);
  lv_obj_t *lbl_plus = lv_label_create(btn_plus);
  lv_label_set_text(lbl_plus, ">>");
  lv_obj_center(lbl_plus);

  ctx_minus->value = value;
  ctx_minus->step = -step;
  ctx_minus->min_value = min_value;
  ctx_minus->max_value = max_value;
  ctx_minus->label = *value_label;
  ctx_minus->name = name;

  ctx_plus->value = value;
  ctx_plus->step = step;
  ctx_plus->min_value = min_value;
  ctx_plus->max_value = max_value;
  ctx_plus->label = *value_label;
  ctx_plus->name = name;

  lv_obj_add_event_cb(btn_minus, pid_adjust_button_event_callback, LV_EVENT_CLICKED, ctx_minus);
  lv_obj_add_event_cb(btn_plus, pid_adjust_button_event_callback, LV_EVENT_CLICKED, ctx_plus);
}

void update_access_network_labels(String ip, String mac) {
  if (g_label_ip_line == NULL) {
    return;
  }
  String ip_text = "IP: " + ip;
  if (strcmp(ip_text.c_str(), lv_label_get_text(g_label_ip_line))==0)
    return;
  lv_label_set_text(g_label_ip_line, ip_text.c_str());

  if (g_label_mac_line != NULL) {
    // Hide MAC once network and clock are both stable to reduce UI clutter.
    bool show_mac = (ip == "not connected") || (currentTime == "HH:MM:SS");
    if (show_mac) {
      String mac_text = "Ma:" + mac;
      lv_label_set_text(g_label_mac_line, mac_text.c_str());
      lv_obj_clear_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

void set_pwm_values() {
  bool output_active = (g_pwm_percent > 0.0f);

  // Visual PWM feedback:green LED when output is active, white otherwise.
  if (g_led_pwm_state != NULL) {
    lv_color_t led_color = output_active ? lv_color_hex(0x16A34A) : lv_color_hex(0xFFFFFF);
    lv_obj_set_style_bg_color(g_led_pwm_state, led_color, 0);
  }

  // Screen saver state reflects the real output state, not the manual switch.
  if (g_label_ss_vmc != NULL) {
    lv_label_set_text(g_label_ss_vmc, output_active ? "VENT ON" : "VENT OFF");
    lv_obj_set_style_text_color(g_label_ss_vmc, output_active ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
  }

  if (g_label_pwm_value != NULL) {
    char pwm_text[12];
    lv_snprintf(pwm_text, sizeof(pwm_text), "%.0f", g_pwm_percent);
    lv_label_set_text(g_label_pwm_value, pwm_text);
  }
}

void update_dht_values() {
  if (isnan(sensorData.temperature) || isnan(sensorData.humidity)) 
    return;

  // Update main temperature/humidity labels and mirror on screen saver page.
  char temp_text[12];
  lv_snprintf(temp_text, sizeof(temp_text), "%.1f", sensorData.temperature);
  lv_label_set_text(g_label_reading_value, temp_text);
  lv_obj_set_style_text_color(g_label_reading_value, color_from_temperature(sensorData.temperature), 0);

  char hum_text[12];
  lv_snprintf(hum_text, sizeof(hum_text), "%.0f", sensorData.humidity);
  lv_label_set_text(g_label_humidity_value, hum_text);

  if (g_label_ss_temp != NULL) {
    char temp_ss_text[16];
    lv_snprintf(temp_ss_text, sizeof(temp_ss_text), "%s C", temp_text);
    lv_label_set_text(g_label_ss_temp, temp_ss_text);
    lv_obj_set_style_text_color(g_label_ss_temp, color_from_temperature(sensorData.temperature), 0);
  }
}

void update_graph_history() {
  if (chart == NULL || ser1 == NULL || ser2 == NULL) {
    return;
  }
  if (!isfinite(sensorData.temperature) || !isfinite(g_pwm_percent)) {
    return;
  }
  int temp_value = (int)(sensorData.temperature * 10.0f + (sensorData.temperature >= 0.0f ? 0.5f : -0.5f));
  int pwm_value = (int)(g_pwm_percent + 0.5f);
  lv_chart_set_next_value(chart, ser1, temp_value);
  lv_chart_set_next_value(chart, ser2, pwm_value);
  if (pageVisible == PAGE_GRAPH) {
  lv_chart_refresh(chart);
  }
}

void switch_callback(lv_obj_t *label, bool is_on) {
  lv_obj_t *target_switch = NULL;
  if (label == g_label_pid_state) {
    target_switch = g_switch_pid;
  } else if (label == g_label_vmc_state) {
    target_switch = g_switch_vmc;
  }

  if (target_switch != NULL) {
    if (is_on) {
      lv_obj_add_state(target_switch, LV_STATE_CHECKED);
    } else {
      lv_obj_clear_state(target_switch, LV_STATE_CHECKED);
    }
  }

  lv_label_set_text(label, is_on ? "ON" : "OFF");
  lv_obj_set_style_text_color(label, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
}

/* 
Page management
*/
static lv_obj_t *screen_ = NULL;
static lv_obj_t *g_tileview = NULL;
static lv_obj_t *g_tile_page_main = NULL;
static lv_obj_t *g_tile_page_saver = NULL;
static lv_obj_t *g_tile_page_graph = NULL;
int pageVisible = 0;

//void gui_setPage(int page);

static void temperature_title_event_callback(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  gui_setPage(PAGE_GRAPH);
}

static void page_nav_button_event_callback(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  intptr_t page = (intptr_t)lv_event_get_user_data(e);
  gui_setPage((int)page);
}

static void tileview_page_changed_event_callback(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED || g_tileview == NULL) {
    return;
  }
  lv_obj_t *active_tile = lv_tileview_get_tile_active(g_tileview);
  if (active_tile == g_tile_page_main) {
    pageVisible = PAGE_MAIN;
  } else if (active_tile == g_tile_page_graph) {
    pageVisible = PAGE_GRAPH;
  } else if (active_tile == g_tile_page_saver) {
    pageVisible = PAGE_SAVER;
  } else {
    return;
  }
  LV_LOG_USER("Page(swiped):%d", pageVisible);
}

// Page helper for explicit page selection (0: main dashboard, 1: screen saver).
void gui_setPage(int page) {
  if (g_tileview == NULL) 
    return;
  if (page < 0) 
    page = 0;
  if (page >= UI_PAGE_COUNT) 
    page = UI_PAGE_COUNT - 1;
  pageVisible = page;
  lv_tileview_set_tile_by_index(g_tileview, 0, (uint32_t)pageVisible, LV_ANIM_OFF);
  LV_LOG_USER("Page:%d", pageVisible);
}

// Page helper for relative page movement.
void gui_switch_page(int sens) {
  int next_page = pageVisible + sens;
  if (next_page < 0) {
    next_page += UI_PAGE_COUNT;
  }
  next_page %= UI_PAGE_COUNT;
  gui_setPage(next_page);
}

void control_slider_callback() {
  char temp_text[12];
  int temp_tenth = (int)(g_setpoint_temp_c * 10.0f + 0.5f);
  format_temp_1_decimal(temp_text, sizeof(temp_text), temp_tenth);
  lv_label_set_text(g_label_control_value, temp_text);
  lv_obj_set_style_text_color(g_label_control_value, color_from_temperature(g_setpoint_temp_c), 0);
}

void lv_create_gui() {
  // Styles are initialized once, then reused across page rebuilds.
  static lv_style_t style_screen;
  static lv_style_t style_card;
  static lv_style_t style_title;
  static bool styles_initialized = false;

  if (!styles_initialized) {
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

  screen_ = lv_screen_active();
  lv_obj_add_style(screen_, &style_screen, 0);
  lv_obj_set_style_pad_all(screen_, 0, 0);

  g_tileview = lv_tileview_create(screen_);
  lv_obj_set_size(g_tileview, lv_pct(100), lv_pct(100));
  lv_obj_set_scrollbar_mode(g_tileview, LV_SCROLLBAR_MODE_OFF);
  lv_obj_add_event_cb(g_tileview, tileview_page_changed_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  lv_display_t *display = lv_display_get_default();

  // Tileview pages.
  g_tile_page_main = lv_tileview_add_tile(g_tileview, 0, 0, LV_DIR_VER);

  g_tile_page_graph = lv_tileview_add_tile(g_tileview, 0, 1, LV_DIR_VER);
  
  g_tile_page_saver = lv_tileview_add_tile(g_tileview, 0, 2, LV_DIR_VER);

  lv_obj_t *tv1 = g_tile_page_main;
  lv_obj_t *tv2 = g_tile_page_graph;
  lv_obj_t *tv3 = g_tile_page_saver;

  int screen_w = (int)lv_display_get_horizontal_resolution(display);
  int screen_h = (int)lv_display_get_vertical_resolution(display);
  int half_w = screen_w / 2;
  int panel_pid_w = screen_w - half_w;
  int top_h = (screen_h * 60) / 100;
  int bottom_h = screen_h - top_h;
  int graph_header_h = 34;
  int graph_nav_h = 54;
  int graph_chart_h = screen_h - graph_header_h - graph_nav_h;
  // const uint16_t graph_point_count = 180;
  int pid_bottom_line_y = top_h - 30;
  int top_row_1_y = 36;
  int top_row_4_y = pid_bottom_line_y;
  int top_row_spacing = (top_row_4_y - top_row_1_y) / 3;
  int top_row_2_y = top_row_1_y + top_row_spacing;
  int top_row_3_y = top_row_2_y + top_row_spacing;

  // Page 1: main dashboard (temperature, PID, ventilation, network).
  LV_LOG_USER("Page 1");
  // Top-left card: live temperature, humidity and setpoint slider.
  lv_obj_t *panel_temp = lv_obj_create(tv1);
  lv_obj_set_pos(panel_temp, 0, 0);
  lv_obj_set_size(panel_temp, half_w, top_h);
  lv_obj_add_style(panel_temp, &style_card, 0);

  lv_obj_t *title_temp = lv_label_create(panel_temp);
  lv_label_set_text(title_temp, "Temperature");
  lv_obj_add_style(title_temp, &style_title, 0);
  lv_obj_set_style_text_font(title_temp, &lv_font_montserrat_16, 0);
  lv_obj_set_width(title_temp, lv_pct(100));
  lv_obj_set_style_text_align(title_temp, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(title_temp, 0, 0);
  lv_obj_add_flag(title_temp, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(title_temp, temperature_title_event_callback, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label_reading = lv_label_create(panel_temp);
  lv_label_set_text(label_reading, "Lecture :");
  lv_obj_set_pos(label_reading, 8, top_row_1_y);

  g_label_reading_value = lv_label_create(panel_temp);
  lv_label_set_text(g_label_reading_value, "--.-");
  lv_obj_set_style_text_color(g_label_reading_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_style_text_font(g_label_reading_value, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(g_label_reading_value, 75, top_row_1_y);

  lv_obj_t *label_reading_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_reading_unit, "°C");
  lv_obj_set_pos(label_reading_unit, 130, top_row_1_y);

  lv_obj_t *label_control = lv_label_create(panel_temp);
  lv_label_set_text(label_control, "Control :");
  lv_obj_set_pos(label_control, 8, top_row_2_y);

  g_label_control_value = lv_label_create(panel_temp);
  control_slider_callback();
  lv_obj_set_style_text_font(g_label_control_value, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(g_label_control_value, 75, top_row_2_y);

  lv_obj_t *label_control_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_control_unit, "°C");
  lv_obj_set_pos(label_control_unit, 130, top_row_2_y);

  lv_obj_t *label_humidity = lv_label_create(panel_temp);
  lv_label_set_text(label_humidity, "Humid. :");
  lv_obj_set_pos(label_humidity, 8, top_row_3_y);

  g_label_humidity_value = lv_label_create(panel_temp);
  lv_label_set_text(g_label_humidity_value, "--");
  lv_obj_set_style_text_color(g_label_humidity_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_style_text_font(g_label_humidity_value, &lv_font_montserrat_16, 0);
  lv_obj_set_pos(g_label_humidity_value, 75, top_row_3_y);

  lv_obj_t *label_humidity_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_humidity_unit, "%");
  lv_obj_set_pos(label_humidity_unit, 130, top_row_3_y);

  lv_obj_t *slider_setpoint = lv_slider_create(panel_temp);
  lv_obj_set_pos(slider_setpoint, 8, top_row_4_y + 4);
  lv_obj_set_size(slider_setpoint, half_w - 16, 10);
  lv_slider_set_range(slider_setpoint, 250, 450);
  lv_slider_set_value(slider_setpoint, 300, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_setpoint, control_slider_event_callback, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xD6DEE5), LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0x3B82F6), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xFFFFFF), LV_PART_KNOB);

  lv_obj_t *panel_pid = lv_obj_create(tv1);
  lv_obj_set_pos(panel_pid, half_w, 0);
  lv_obj_set_size(panel_pid, panel_pid_w, top_h);
  lv_obj_add_style(panel_pid, &style_card, 0);

  lv_obj_t *title_pid = lv_label_create(panel_pid);
  lv_label_set_text(title_pid, "P.I.D.");
  lv_obj_add_style(title_pid, &style_title, 0);
  lv_obj_set_style_text_font(title_pid, &lv_font_montserrat_16, 0);
  lv_obj_set_width(title_pid, lv_pct(100));
  lv_obj_set_style_text_align(title_pid, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(title_pid, 0, 0);

  // PID enable switch.
  g_switch_pid = lv_switch_create(panel_pid);
  lv_obj_set_pos(g_switch_pid, 8, top_row_4_y);
  lv_obj_set_size(g_switch_pid, 50, 24);
  lv_obj_add_state(g_switch_pid, LV_STATE_CHECKED);
  lv_obj_add_event_cb(g_switch_pid, pid_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  create_pid_adjuster(panel_pid, top_row_1_y - 4, "Pro.", &g_label_kp_value, &g_kp, 0.010f, 0.000f, 25.000f, &g_ctx_kp_minus, &g_ctx_kp_plus, "Kp");
  create_pid_adjuster(panel_pid, top_row_2_y - 4, "Int.", &g_label_ki_value, &g_ki, 0.050f, 0.000f, 50.000f, &g_ctx_ki_minus, &g_ctx_ki_plus, "Ki");
  create_pid_adjuster(panel_pid, top_row_3_y - 4, "Dev.", &g_label_kd_value, &g_kd, 0.050f, 0.000f, 50.000f, &g_ctx_kd_minus, &g_ctx_kd_plus, "Kd");

  // Bottom-left card: manual ventilation control and live PWM state.

  g_label_pid_state = lv_label_create(panel_pid);
  lv_label_set_text(g_label_pid_state, "ON");
  lv_obj_set_style_text_color(g_label_pid_state, lv_color_hex(0x107C10), 0);
  lv_obj_set_pos(g_label_pid_state, panel_pid_w - 34, top_row_4_y + 4);

  lv_obj_t *panel_vmc = lv_obj_create(tv1);
  lv_obj_set_pos(panel_vmc, 0, top_h);
  lv_obj_set_size(panel_vmc, half_w, bottom_h);
  lv_obj_add_style(panel_vmc, &style_card, 0);

  lv_obj_t *label_vmc = lv_label_create(panel_vmc);
  lv_label_set_text(label_vmc, "Ventilation");
  lv_obj_add_style(label_vmc, &style_title, 0);
  lv_obj_set_style_text_font(label_vmc, &lv_font_montserrat_16, 0);
  lv_obj_set_width(label_vmc, lv_pct(100));
  lv_obj_set_style_text_align(label_vmc, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(label_vmc, 0, 0);

  int lower_row_1_y = 33;
  int lower_row_2_y = 66;

  g_switch_vmc = lv_switch_create(panel_vmc);
  lv_obj_set_pos(g_switch_vmc, 8, lower_row_1_y);
  lv_obj_set_size(g_switch_vmc, 50, 24);
  lv_obj_add_state(g_switch_vmc, LV_STATE_CHECKED);
  lv_obj_add_event_cb(g_switch_vmc, vmc_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *label_pwm = lv_label_create(panel_vmc);
  lv_label_set_text(label_pwm, "Vitesse :");
  lv_obj_set_pos(label_pwm, 8, lower_row_2_y);

  g_label_pwm_value = lv_label_create(panel_vmc);
  lv_label_set_text(g_label_pwm_value, "000");
  lv_obj_set_style_text_color(g_label_pwm_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(g_label_pwm_value, 75, lower_row_2_y);

  lv_obj_t *label_pwm_unit = lv_label_create(panel_vmc);
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

  lv_obj_t *panel_status = lv_obj_create(tv1);
  lv_obj_set_pos(panel_status, half_w, top_h);
  lv_obj_set_size(panel_status, screen_w - half_w, bottom_h);
  lv_obj_add_style(panel_status, &style_card, 0);

  lv_obj_t *label_access = lv_label_create(panel_status);
  lv_label_set_text(label_access, "Acces");
  lv_obj_add_style(label_access, &style_title, 0);
  lv_obj_set_style_text_font(label_access, &lv_font_montserrat_16, 0);
  lv_obj_set_width(label_access, lv_pct(100));
  lv_obj_set_style_text_align(label_access, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(label_access, 0, 0);

  g_label_time = lv_label_create(panel_status);
  lv_label_set_text(g_label_time, currentTime.c_str());
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
  lv_obj_set_pos(g_label_mac_line, 0, lower_row_2_y - 16);

  lv_obj_t *separator_v = lv_obj_create(tv1);
  lv_obj_remove_style_all(separator_v);
  lv_obj_set_pos(separator_v, half_w, 0);
  lv_obj_set_size(separator_v, 1, screen_h);
  lv_obj_set_style_bg_color(separator_v, lv_color_hex(0x7A8A99), 0);

  lv_obj_t *separator_h = lv_obj_create(tv1);
  lv_obj_remove_style_all(separator_h);
  lv_obj_set_pos(separator_h, 0, top_h);
  lv_obj_set_size(separator_h, screen_w, 1);
  lv_obj_set_style_bg_color(separator_h, lv_color_hex(0x7A8A99), 0);

  // Page 2: Graph.
  LV_LOG_USER("Page 2");
  lv_obj_t *panel_graph = lv_obj_create(tv2);
  lv_obj_set_pos(panel_graph, 0, 0);
  lv_obj_set_size(panel_graph, screen_w, screen_h);
  lv_obj_set_style_radius(panel_graph, 0, 0);
  lv_obj_set_style_border_width(panel_graph, 0, 0);
  lv_obj_set_style_bg_color(panel_graph, lv_color_hex(0x0F172A), 0);
  lv_obj_set_style_bg_grad_color(panel_graph, lv_color_hex(0x1E293B), 0);
  lv_obj_set_style_bg_grad_dir(panel_graph, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_pad_all(panel_graph, 0, 0);

  lv_obj_t *graph_title = lv_label_create(panel_graph);
  lv_label_set_text(graph_title, "History");
  lv_obj_set_style_text_font(graph_title, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(graph_title, lv_color_hex(0xE5E7EB), 0);
  lv_obj_set_pos(graph_title, 10, 8);

  lv_obj_t *graph_legend_temp = lv_label_create(panel_graph);
  lv_label_set_text(graph_legend_temp, "Temp.");
  lv_obj_set_style_text_font(graph_legend_temp, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(graph_legend_temp, lv_color_hex(0xF87171), 0);
  lv_obj_set_pos(graph_legend_temp, screen_w - 110, 10);

  lv_obj_t *graph_legend_pwm = lv_label_create(panel_graph);
  lv_label_set_text(graph_legend_pwm, "Correction");
  lv_obj_set_style_text_font(graph_legend_pwm, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(graph_legend_pwm, lv_color_hex(0x4ADE80), 0);
  lv_obj_set_pos(graph_legend_pwm, screen_w - 110, 22);

  chart = lv_chart_create(panel_graph);
  lv_obj_set_pos(chart, 0, graph_header_h);
  lv_obj_set_size(chart, screen_w, graph_chart_h);
  lv_obj_set_style_radius(chart, 0, 0);
  lv_obj_set_style_border_width(chart, 0, 0);
  lv_obj_set_style_bg_color(chart, lv_color_hex(0x111827), 0);
  lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(chart, DSP_GRAPH_HISTORY_POINTS);
  lv_chart_set_div_line_count(chart, 4, 8);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_X, 0, DSP_GRAPH_HISTORY_POINTS - 1);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 500);
  lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
  ser1 = lv_chart_add_series(chart, lv_color_hex(0xF87171), LV_CHART_AXIS_PRIMARY_Y);
  ser2 = lv_chart_add_series(chart, lv_color_hex(0x4ADE80), LV_CHART_AXIS_SECONDARY_Y);
  //lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
  //lv_obj_set_style_width(chart, 3, LV_PART_INDICATOR);
  //lv_obj_set_style_height(chart, 3, LV_PART_INDICATOR);
  lv_chart_refresh(chart);

  lv_obj_t *graph_nav = lv_obj_create(panel_graph);
  lv_obj_set_pos(graph_nav, 0, screen_h - graph_nav_h);
  lv_obj_set_size(graph_nav, screen_w, graph_nav_h);
  lv_obj_set_style_radius(graph_nav, 0, 0);
  lv_obj_set_style_border_width(graph_nav, 0, 0);
  lv_obj_set_style_bg_color(graph_nav, lv_color_hex(0x0B1220), 0);
  lv_obj_set_style_pad_all(graph_nav, 4, 0);
  lv_obj_set_flex_flow(graph_nav, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(graph_nav, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  // Button navigation.
  struct nav_button_def_t {
    const char *text;
    int page;
  } nav_buttons[] = {
      {"Main", PAGE_MAIN},
      {"Graph", PAGE_GRAPH},
      {"Saver", PAGE_SAVER},
  };

  for (uint8_t i = 0; i < sizeof(nav_buttons) / sizeof(nav_buttons[0]); ++i) {
    lv_obj_t *btn = lv_btn_create(graph_nav);
    lv_obj_set_size(btn, 70, 30);
    lv_obj_add_event_cb(btn, page_nav_button_event_callback, LV_EVENT_CLICKED, (void *)(intptr_t)nav_buttons[i].page);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, nav_buttons[i].text);
    lv_obj_center(btn_label);
  }

   // Page 3: screen saver (high contrast, large clock and process summary).
  LV_LOG_USER("Page 3");
  lv_obj_set_scrollbar_mode(tv3, LV_SCROLLBAR_MODE_OFF);
  lv_obj_t *panel_screensaver = lv_obj_create(tv3);
  lv_obj_set_pos(panel_screensaver, 0, 0);
  lv_obj_set_size(panel_screensaver, screen_w, screen_h);
  lv_obj_set_style_radius(panel_screensaver, 0, 0);
  lv_obj_set_style_border_width(panel_screensaver, 0, 0);
  lv_obj_set_style_bg_color(panel_screensaver, lv_color_hex(0x111827), 0);
  lv_obj_set_style_bg_grad_color(panel_screensaver, lv_color_hex(0x1F2937), 0);
  lv_obj_set_style_bg_grad_dir(panel_screensaver, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_pad_all(panel_screensaver, 0, 0);

  g_label_ss_time = lv_label_create(panel_screensaver);
  lv_label_set_text(g_label_ss_time, currentTime.c_str());
  lv_obj_set_style_text_font(g_label_ss_time, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(g_label_ss_time, lv_color_hex(0xE5E7EB), 0);
  lv_obj_align(g_label_ss_time, LV_ALIGN_TOP_MID, 0, 22);

  g_label_ss_temp = lv_label_create(panel_screensaver);
  if (isnan(sensorData.temperature)) {
    lv_label_set_text(g_label_ss_temp, "--.- °C");
  } else {
    char temp_ss_text[16];
    lv_snprintf(temp_ss_text, sizeof(temp_ss_text), "%.1f °C", sensorData.temperature);
    lv_label_set_text(g_label_ss_temp, temp_ss_text);
  }
  lv_obj_set_style_text_font(g_label_ss_temp, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(g_label_ss_temp, isnan(sensorData.temperature) ? lv_color_hex(0x9CA3AF) : color_from_temperature(sensorData.temperature), 0);
  lv_obj_align(g_label_ss_temp, LV_ALIGN_CENTER, 0, -8);

  g_label_ss_vmc = lv_label_create(panel_screensaver);
  lv_label_set_text(g_label_ss_vmc, g_vmc_manual_on ? "VENT ON" : "VENT OFF");
  lv_obj_set_style_text_font(g_label_ss_vmc, &lv_font_montserrat_36, 0);
  lv_obj_set_style_text_color(g_label_ss_vmc, g_vmc_manual_on ? lv_color_hex(0x22C55E) : lv_color_hex(0xEF4444), 0);
  lv_obj_align(g_label_ss_vmc, LV_ALIGN_BOTTOM_MID, 0, -28);

}


/*
* Save & Restaure facilities for PID settings, using SPIFFS and JSON serialization.
*/

static bool save_pid_settings_to_file() {

  JsonDocument settings;
  settings["kp"] = g_kp;
  settings["ki"] = g_ki;
  settings["kd"] = g_kd;
  settings["setpoint"] = g_setpoint_temp_c;
  File file = SPIFFS.open(PID_SETTINGS_FILE, FILE_WRITE);

  if (!file) {
    LV_LOG_USER("PID settings save failed: unable to open file");
    return false;
  }
  file.close();
  if (SPIFFS.exists(PID_SETTINGS_FILE) && !SPIFFS.remove(PID_SETTINGS_FILE)) {
    LV_LOG_USER("PID settings save failed: unable to remove old file");
    return false;
  }
  file = SPIFFS.open(PID_SETTINGS_FILE, FILE_WRITE);
  if (!file) {
    LV_LOG_USER("PID settings save failed: unable to recreate file");
    return false;
  }
  if (serializeJson(settings, file) == 0) {
    LV_LOG_USER("PID settings save failed: serialize error");
    file.close();
    return false;
  }
  file.flush();
  file.close();
  
  LV_LOG_USER("PID settings saved: Kp=%.3f Ki=%.3f Kd=%.3f Setpoint=%.1f\n", g_kp, g_ki, g_kd, g_setpoint_temp_c);

  return true;
}

void save_pid_settings() {
  save_pid_settings_to_file();
  must_be_saved = false;
}

static float clamp_pid_value(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void load_pid_settings_from_file() {

  if (!SPIFFS.exists(PID_SETTINGS_FILE)) {
    LV_LOG_USER("PID settings file not found, using defaults");
    return;
  }

  File file = SPIFFS.open(PID_SETTINGS_FILE, FILE_READ);
  if (!file) {
    LV_LOG_USER("PID settings load failed: unable to open file");
    return;
  }

  JsonDocument settings;
  DeserializationError error = deserializeJson(settings, file);
  file.close();
  if (error) {
    LV_LOG_USER("PID settings load failed: invalid JSON");
    return;
  }

  if (settings["kp"].is<float>() || settings["kp"].is<int>()) {
    g_kp = clamp_pid_value(settings["kp"].as<float>(), 0.0f, 25.0f);

  }
  if (settings["ki"].is<float>() || settings["ki"].is<int>()) {
    g_ki = clamp_pid_value(settings["ki"].as<float>(), 0.0f, 50.0f);
  }
  if (settings["kd"].is<float>() || settings["kd"].is<int>()) {
    g_kd = clamp_pid_value(settings["kd"].as<float>(), 0.0f, 50.0f);
  }
  if (settings["setpoint"].is<float>() || settings["setpoint"].is<int>()) {
    g_setpoint_temp_c = clamp_pid_value(settings["setpoint"].as<float>(), 25.0f, 45.0f);
  }

  LV_LOG_USER("PID settings loaded: Kp=%.3f Ki=%.3f Kd=%.3f Setpoint=%.1f\n", g_kp, g_ki, g_kd, g_setpoint_temp_c);
}
