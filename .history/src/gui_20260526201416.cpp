#include <lvgl.h>

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

  update_access_network_labels();

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
