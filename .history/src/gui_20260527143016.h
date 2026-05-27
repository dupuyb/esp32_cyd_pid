#pragma once
#include <Arduino.h>
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

static void update_pid_value_label(lv_obj_t * label, float value) {
  char value_text[16];
  lv_snprintf(value_text, sizeof(value_text), "%.3f", value);
  lv_label_set_text(label, value_text);
}

static void pid_adjust_button_event_callback(lv_event_t * e) {
  if(lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  pid_adjust_ctx_t * ctx = (pid_adjust_ctx_t *)lv_event_get_user_data(e);
  float new_value = *(ctx->value) + ctx->step;
  if(new_value < ctx->min_value) {
    new_value = ctx->min_value;
  }
  if(new_value > ctx->max_value) {
    new_value = ctx->max_value;
  }

  *(ctx->value) = new_value;
  update_pid_value_label(ctx->label, new_value);
  LV_LOG_USER("%s changed to %.3f", ctx->name, new_value);
}

static void format_temp_1_decimal(char * out, size_t out_size, int tenth_degrees) {
  int whole = tenth_degrees / 10;
  int decimal = tenth_degrees % 10;
  lv_snprintf(out, out_size, "%d.%d", whole, decimal);
}

static lv_color_t color_from_temperature(float temp_c) {
  if(temp_c < 28.0f) {
    return lv_color_hex(0x1565C0);
  }
  if(temp_c <= 35.0f) {
    return lv_color_hex(0x107C10);
  }
  if(temp_c <= 40.0f) {
    return lv_color_hex(0xE67E22);
  }
  return lv_color_hex(0xC62828);
}

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
) {
  lv_obj_t * label_caption = lv_label_create(parent);
  lv_label_set_text(label_caption, caption);
  lv_obj_set_pos(label_caption, 8, y + 4);

  *value_label = lv_label_create(parent);
  lv_obj_set_style_text_color(*value_label, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(*value_label, 85, y + 4);
  update_pid_value_label(*value_label, *value);

  lv_obj_t * btn_minus = lv_button_create(parent);
  lv_obj_set_size(btn_minus, 26, 22);
  lv_obj_set_pos(btn_minus, 55, y);
  lv_obj_t * lbl_minus = lv_label_create(btn_minus);
  lv_label_set_text(lbl_minus, "<<");
  lv_obj_center(lbl_minus);

  lv_obj_t * btn_plus = lv_button_create(parent);
  lv_obj_set_size(btn_plus, 26, 22);
  lv_obj_set_pos(btn_plus, 130, y);
  lv_obj_t * lbl_plus = lv_label_create(btn_plus);
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

extern  lv_obj_t * g_label_ip_line;
extern  lv_obj_t * g_label_mac_line;
extern  lv_obj_t * g_label_time;
extern String strTime;

static void update_access_network_labels(String ip, String mac) {
  if(g_label_ip_line == NULL) {
    return;
  }

  //String ip = formatIpAddress(WiFi.localIP());
  String ip_text = "IP: " + ip;
  lv_label_set_text(g_label_ip_line, ip_text.c_str());

  if(g_label_mac_line != NULL) {
    bool show_mac = (ip == "not connected") || (strTime == "00:00:00");
    if(show_mac) {
      String mac_text = "Mc:" + mac; // WiFi.macAddress();
      lv_label_set_text(g_label_mac_line, mac_text.c_str());
      lv_obj_clear_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
    }
    else {
      lv_obj_add_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

extern  lv_obj_t * g_led_pwm_state;
extern  lv_obj_t * g_label_pwm_value; 

static void set_pwm_percent(uint32_t duty, uint8_t pinOF, float percent ){
  if(g_led_pwm_state != NULL) {
    lv_color_t led_color = (duty > 0U) ? lv_color_hex(0x16A34A) : lv_color_hex(0xFFFFFF);
    lv_obj_set_style_bg_color(g_led_pwm_state, led_color, 0);
    digitalWrite(pinOF, (duty > 0U) ? HIGH : LOW);
  }

  if(g_label_pwm_value != NULL) {
    char pwm_text[12];
    lv_snprintf(pwm_text, sizeof(pwm_text), "%.0f", percent);
    lv_label_set_text(g_label_pwm_value, pwm_text);
  }
}

extern  lv_obj_t *g_label_reading_value;
extern  lv_obj_t *g_label_humidity_value;

static void update_dht_values(float temperature, float humidity) {
  char temp_text[12];
  lv_snprintf(temp_text, sizeof(temp_text), "%.1f", temperature);
  lv_label_set_text(g_label_reading_value, temp_text);
  lv_obj_set_style_text_color(g_label_reading_value, color_from_temperature(temperature), 0);
 
  char hum_text[12];
  lv_snprintf(hum_text, sizeof(hum_text), "%.0f", humidity);
  lv_label_set_text(g_label_humidity_value, hum_text);
}

extern lv_obj_t * g_label_control_value;

static void control_slider_event_callback(char* temp_text, float g_setpoint_temp_c) {
  lv_label_set_text(g_label_control_value, temp_text);
  lv_obj_set_style_text_color(g_label_control_value, color_from_temperature(g_setpoint_temp_c), 0);
}

static void switch_event_callback(lv_obj_t * label, bool is_on) {
  lv_label_set_text(label, is_on ? "ON" : "OFF");
  lv_obj_set_style_text_color(label, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
}