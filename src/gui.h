#pragma once
#include <Arduino.h>
#include <lvgl.h>

#define LV_DELAY(x)                                                            \
    do {                                                                       \
        uint32_t t = x;                                                        \
        while (t--) {                                                          \
            lv_timer_handler();                                                \
            delay(1);                                                          \
        }                                                                      \
    } while (0);

static float    g_kp = 0.123 f;
static float    g_ki = 5.545 f;
static float    g_kd = 11.130 f;
static float    g_setpoint_temp_c = 30.0 f;
static float    g_current_temp_c = NAN;
static float    g_pid_integral = 0.0 f;
static float    g_pid_prev_error = 0.0 f;
static float    g_pwm_percent = 0.0 f;
static bool     g_pid_has_prev_error = false;
static bool     g_pid_enabled = true;
static bool     g_vmc_manual_on = true;

typedef struct {
        float          *value;
        float           step;
        float           min_value;
        float           max_value;
        lv_obj_t       *label;
        const char     *name;
}               pid_adjust_ctx_t;

static void 
update_pid_value_label(lv_obj_t * label, float value)
{
        //Keep all PID labels formatted with 3 decimals for consistent tuning
        // feedback.
        char            value_text[16];
        lv_snprintf(value_text, sizeof(value_text), "%.3f", value);
        lv_label_set_text(label, value_text);
}

static void 
pid_adjust_button_event_callback(lv_event_t * e)
{
        if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
                return;
        }

        pid_adjust_ctx_t *ctx = (pid_adjust_ctx_t *) lv_event_get_user_data(e);
        //Generic callback shared by + /-buttons for Kp
                /Ki / Kd.
                        float           new_value = *(ctx->value) + ctx->step;
        if (new_value < ctx->min_value) {
                new_value = ctx->min_value;
        }
        if (new_value > ctx->max_value) {
                new_value = ctx->max_value;
        }

        *(ctx->value) = new_value;
        update_pid_value_label(ctx->label, new_value);
        LV_LOG_USER("%s changed to %.3f", ctx->name, new_value);
}

static void 
format_temp_1_decimal(char *out, size_t out_size,
                      int tenth_degrees)
{
        int             whole = tenth_degrees / 10;
        int             decimal = tenth_degrees % 10;
        lv_snprintf(out, out_size, "%d.%d", whole, decimal);
}

static lv_color_t 
color_from_temperature(float temp_c)
{
        if (temp_c < 28.0 f) {
                return lv_color_hex(0x1565C0);
        }
        if (temp_c <= 35.0 f) {
                return lv_color_hex(0x107C10);
        }
        if (temp_c <= 40.0 f) {
                return lv_color_hex(0xE67E22);
        }
        return lv_color_hex(0xC62828);
}

static void 
create_pid_adjuster(lv_obj_t * parent, int y, const char *caption,
                    lv_obj_t ** value_label, float *value,
                    float step, float min_value, float max_value,
                    pid_adjust_ctx_t * ctx_minus,
                    pid_adjust_ctx_t * ctx_plus, const char *name)
{
        lv_obj_t       *label_caption = lv_label_create(parent);
        lv_label_set_text(label_caption, caption);
        lv_obj_set_pos(label_caption, 8, y + 4);

        *value_label = lv_label_create(parent);
        lv_obj_set_style_text_color(*value_label, lv_color_hex(0x0F3D5E), 0);
        lv_obj_set_pos(*value_label, 85, y + 4);
        update_pid_value_label(*value_label, *value);

        lv_obj_t       *btn_minus = lv_button_create(parent);
        lv_obj_set_size(btn_minus, 26, 22);
        lv_obj_set_pos(btn_minus, 55, y);
        lv_obj_t       *lbl_minus = lv_label_create(btn_minus);
        lv_label_set_text(lbl_minus, "<<");
        lv_obj_center(lbl_minus);

        lv_obj_t       *btn_plus = lv_button_create(parent);
        lv_obj_set_size(btn_plus, 26, 22);
        lv_obj_set_pos(btn_plus, 130, y);
        lv_obj_t       *lbl_plus = lv_label_create(btn_plus);
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

        lv_obj_add_event_cb(btn_minus, pid_adjust_button_event_callback,
                            LV_EVENT_CLICKED, ctx_minus);
        lv_obj_add_event_cb(btn_plus, pid_adjust_button_event_callback,
                            LV_EVENT_CLICKED, ctx_plus);
}

extern lv_obj_t *g_label_ip_line;
extern lv_obj_t *g_label_mac_line;
extern lv_obj_t *g_label_time;
extern String   strTime;

static void 
update_access_network_labels(String ip, String mac)
{
        if (g_label_ip_line == NULL) {
                return;
        }

        //String ip = formatIpAddress(WiFi.localIP());
        String          ip_text = "IP: " + ip;
        lv_label_set_text(g_label_ip_line, ip_text.c_str());

        if (g_label_mac_line != NULL) {
                //Hide MAC once network and clock are both stable to reduce UI clutter.
                        bool show_mac = (ip == "not connected") || (strTime == "00:00:00");
                if (show_mac) {
                        String          mac_text = "Mc:" + mac;
                        //WiFi.macAddress();
                        lv_label_set_text(g_label_mac_line, mac_text.c_str());
                        lv_obj_clear_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
                }
                else {
                        lv_obj_add_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
                }
        }
}

extern lv_obj_t *g_led_pwm_state;
extern lv_obj_t *g_label_pwm_value;

static void 
set_pwm_percent(uint32_t duty, uint8_t pinOF, float percent)
{
//Visual PWM feedback:green LED when output is active, white otherwise.
        if (g_led_pwm_state != NULL) {
                lv_color_t      led_color =
                (duty > 0U) ? lv_color_hex(0x16A34A) : lv_color_hex(0xFFFFFF);
                lv_obj_set_style_bg_color(g_led_pwm_state, led_color, 0);
                digitalWrite(pinOF, (duty > 0U) ? HIGH : LOW);
        }

        if (g_label_pwm_value != NULL) {
                char            pwm_text[12];
                lv_snprintf(pwm_text, sizeof(pwm_text), "%.0f", percent);
                lv_label_set_text(g_label_pwm_value, pwm_text);
        }
}

extern lv_obj_t *g_label_reading_value;
extern lv_obj_t *g_label_humidity_value;

static void 
update_dht_values(float temperature, float humidity)
{
        char            temp_text[12];
        lv_snprintf(temp_text, sizeof(temp_text), "%.1f", temperature);
        lv_label_set_text(g_label_reading_value, temp_text);
        lv_obj_set_style_text_color(g_label_reading_value,
                                    color_from_temperature(temperature), 0);

        char            hum_text[12];
        lv_snprintf(hum_text, sizeof(hum_text), "%.0f", humidity);
        lv_label_set_text(g_label_humidity_value, hum_text);
}

extern lv_obj_t *g_label_control_value;

static void 
control_slider_callback(char *temp_text, float g_setpoint_temp_c)
{
        lv_label_set_text(g_label_control_value, temp_text);
        lv_obj_set_style_text_color(g_label_control_value,
                                    color_from_temperature(g_setpoint_temp_c), 0);
}

static void 
switch_callback(lv_obj_t * label, bool is_on)
{
        lv_label_set_text(label, is_on ? "ON" : "OFF");
        lv_obj_set_style_text_color(
                                    label, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
}

static pid_adjust_ctx_t g_ctx_kp_minus;
static pid_adjust_ctx_t g_ctx_kp_plus;
static pid_adjust_ctx_t g_ctx_ki_minus;
static pid_adjust_ctx_t g_ctx_ki_plus;
static pid_adjust_ctx_t g_ctx_kd_minus;
static pid_adjust_ctx_t g_ctx_kd_plus;

static void     vmc_switch_event_callback(lv_event_t * e);
static void     control_slider_event_callback(lv_event_t * e);
static void     pid_switch_event_callback(lv_event_t * e);

extern lv_obj_t *g_label_vmc_state;

void 
lv_create_gui(void)
{
        static lv_style_t style_screen;
        static lv_style_t style_card;
        static lv_style_t style_title;
        static bool     styles_initialized = false;

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

        lv_obj_t       *screen = lv_screen_active();
        lv_obj_add_style(screen, &style_screen, 0);
        lv_obj_set_style_pad_all(screen, 0, 0);

        lv_display_t   *display = lv_display_get_default();
        int             screen_w = (int)lv_display_get_horizontal_resolution(display);
        int             screen_h = (int)lv_display_get_vertical_resolution(display);
        int             half_w = screen_w / 2;
        int             panel_pid_w = screen_w - half_w;
        int             top_h = (screen_h * 60) / 100;
        int             bottom_h = screen_h - top_h;
        int             pid_bottom_line_y = top_h - 30;
        int             top_row_1_y = 36;
        int             top_row_4_y = pid_bottom_line_y;
        int             top_row_spacing = (top_row_4_y - top_row_1_y) / 3;
        int             top_row_2_y = top_row_1_y + top_row_spacing;
        int             top_row_3_y = top_row_2_y + top_row_spacing;

//Top - left card:live temperature, humidity and setpoint slider.

                lv_obj_t * panel_temp = lv_obj_create(screen);
        lv_obj_set_pos(panel_temp, 0, 0);
        lv_obj_set_size(panel_temp, half_w, top_h);
        lv_obj_add_style(panel_temp, &style_card, 0);

        lv_obj_t       *title_temp = lv_label_create(panel_temp);
        lv_label_set_text(title_temp, "Temperature");
        lv_obj_add_style(title_temp, &style_title, 0);
        lv_obj_set_style_text_font(title_temp, &lv_font_montserrat_16, 0);
        lv_obj_set_width(title_temp, lv_pct(100));
        lv_obj_set_style_text_align(title_temp, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(title_temp, 0, 0);

        lv_obj_t       *label_reading = lv_label_create(panel_temp);
        lv_label_set_text(label_reading, "Lecture :");
        lv_obj_set_pos(label_reading, 8, top_row_1_y);

        g_label_reading_value = lv_label_create(panel_temp);
        lv_label_set_text(g_label_reading_value, "--.-");
        lv_obj_set_style_text_color(g_label_reading_value, lv_color_hex(0x0F3D5E),
                                    0);
        lv_obj_set_style_text_font(g_label_reading_value, &lv_font_montserrat_16,
                                   0);
        lv_obj_set_pos(g_label_reading_value, 75, top_row_1_y);

        lv_obj_t       *label_reading_unit = lv_label_create(panel_temp);
        lv_label_set_text(label_reading_unit, "°C");
        lv_obj_set_pos(label_reading_unit, 130, top_row_1_y);

        lv_obj_t       *label_control = lv_label_create(panel_temp);
        lv_label_set_text(label_control, "Control :");
        lv_obj_set_pos(label_control, 8, top_row_2_y);

        g_label_control_value = lv_label_create(panel_temp);
        lv_label_set_text(g_label_control_value, "30.0");
        lv_obj_set_style_text_color(g_label_control_value,
                                    color_from_temperature(30.0 f), 0);
        lv_obj_set_style_text_font(g_label_control_value, &lv_font_montserrat_16,
                                   0);
        lv_obj_set_pos(g_label_control_value, 75, top_row_2_y);

        lv_obj_t       *label_control_unit = lv_label_create(panel_temp);
        lv_label_set_text(label_control_unit, "°C");
        lv_obj_set_pos(label_control_unit, 130, top_row_2_y);

        lv_obj_t       *label_humidity = lv_label_create(panel_temp);
        lv_label_set_text(label_humidity, "Humid. :");
        lv_obj_set_pos(label_humidity, 8, top_row_3_y);

        g_label_humidity_value = lv_label_create(panel_temp);
        lv_label_set_text(g_label_humidity_value, "--");
        lv_obj_set_style_text_color(g_label_humidity_value, lv_color_hex(0x0F3D5E),
                                    0);
        lv_obj_set_style_text_font(g_label_humidity_value, &lv_font_montserrat_16,
                                   0);
        lv_obj_set_pos(g_label_humidity_value, 75, top_row_3_y);

        lv_obj_t       *label_humidity_unit = lv_label_create(panel_temp);
        lv_label_set_text(label_humidity_unit, "%");
        lv_obj_set_pos(label_humidity_unit, 130, top_row_3_y);

        lv_obj_t       *slider_setpoint = lv_slider_create(panel_temp);
        lv_obj_set_pos(slider_setpoint, 8, top_row_4_y + 4);
        lv_obj_set_size(slider_setpoint, half_w - 16, 10);
        lv_slider_set_range(slider_setpoint, 250, 450);
        lv_slider_set_value(slider_setpoint, 300, LV_ANIM_OFF);
        lv_obj_add_event_cb(slider_setpoint, control_slider_event_callback,
                            LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xD6DEE5),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0x3B82F6),
                                  LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xFFFFFF),
                                  LV_PART_KNOB);

        lv_obj_t       *panel_pid = lv_obj_create(screen);
        lv_obj_set_pos(panel_pid, half_w, 0);
        lv_obj_set_size(panel_pid, panel_pid_w, top_h);
        lv_obj_add_style(panel_pid, &style_card, 0);

        lv_obj_t       *title_pid = lv_label_create(panel_pid);
        lv_label_set_text(title_pid, "P.I.D.");
        lv_obj_add_style(title_pid, &style_title, 0);
        lv_obj_set_style_text_font(title_pid, &lv_font_montserrat_16, 0);
        lv_obj_set_width(title_pid, lv_pct(100));
        lv_obj_set_style_text_align(title_pid, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(title_pid, 0, 0);

        //switch PID
                lv_obj_t * switch_pid = lv_switch_create(panel_pid);
        lv_obj_set_pos(switch_pid, 8, top_row_4_y);
        lv_obj_set_size(switch_pid, 50, 24);
        lv_obj_add_state(switch_pid, LV_STATE_CHECKED);
        lv_obj_add_event_cb(switch_pid, pid_switch_event_callback,
                            LV_EVENT_VALUE_CHANGED, NULL);

        create_pid_adjuster(panel_pid, top_row_1_y - 4, "Pro.", &g_label_kp_value,
                            &g_kp, 0.010 f, 0.000 f, 25.000 f, &g_ctx_kp_minus,
                            &g_ctx_kp_plus, "Kp");
        create_pid_adjuster(panel_pid, top_row_2_y - 4, "Int.", &g_label_ki_value,
                            &g_ki, 0.050 f, 0.000 f, 50.000 f, &g_ctx_ki_minus,
                            &g_ctx_ki_plus, "Ki");
        create_pid_adjuster(panel_pid, top_row_3_y - 4, "Dev.", &g_label_kd_value,
                            &g_kd, 0.050 f, 0.000 f, 50.000 f, &g_ctx_kd_minus,
                            &g_ctx_kd_plus, "Kd");

//Bottom - left card:manual VMC control and live PWM state.

                g_label_pid_state = lv_label_create(panel_pid);
        lv_label_set_text(g_label_pid_state, "ON");
        lv_obj_set_style_text_color(g_label_pid_state, lv_color_hex(0x107C10), 0);
        lv_obj_set_pos(g_label_pid_state, panel_pid_w - 34, top_row_4_y + 4);

        lv_obj_t       *panel_vmc = lv_obj_create(screen);
        lv_obj_set_pos(panel_vmc, 0, top_h);
        lv_obj_set_size(panel_vmc, half_w, bottom_h);
        lv_obj_add_style(panel_vmc, &style_card, 0);

        int             lower_row_1_y = 34;
        int             lower_row_2_y = 64;

        lv_obj_t       *label_vmc = lv_label_create(panel_vmc);
        lv_label_set_text(label_vmc, "V.M.C.");
        lv_obj_add_style(label_vmc, &style_title, 0);
        lv_obj_set_style_text_font(label_vmc, &lv_font_montserrat_16, 0);
        lv_obj_set_width(label_vmc, lv_pct(100));
        lv_obj_set_style_text_align(label_vmc, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(label_vmc, 0, 0);

        lv_obj_t       *switch_vmc = lv_switch_create(panel_vmc);
        lv_obj_set_pos(switch_vmc, 8, lower_row_1_y);
        lv_obj_set_size(switch_vmc, 50, 24);
        lv_obj_add_state(switch_vmc, LV_STATE_CHECKED);
        lv_obj_add_event_cb(switch_vmc, vmc_switch_event_callback,
                            LV_EVENT_VALUE_CHANGED, NULL);

        lv_obj_t       *label_pwm = lv_label_create(panel_vmc);
        lv_label_set_text(label_pwm, "Vitesse :");
        lv_obj_set_pos(label_pwm, 8, lower_row_2_y);

        g_label_pwm_value = lv_label_create(panel_vmc);
        lv_label_set_text(g_label_pwm_value, "000");
        lv_obj_set_style_text_color(g_label_pwm_value, lv_color_hex(0x0F3D5E), 0);
        lv_obj_set_pos(g_label_pwm_value, 75, lower_row_2_y);

        lv_obj_t       *label_pwm_unit = lv_label_create(panel_vmc);
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

        lv_obj_t       *panel_status = lv_obj_create(screen);
        lv_obj_set_pos(panel_status, half_w, top_h);
        lv_obj_set_size(panel_status, screen_w - half_w, bottom_h);
        lv_obj_add_style(panel_status, &style_card, 0);

        lv_obj_t       *label_access = lv_label_create(panel_status);
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

        //update_access_network_labels(formatIpAddress(WiFi.localIP()),
                                       //WiFi.macAddress());

        //Visual separators to preserve card boundaries across displays and
                // resolutions.

                lv_obj_t * separator_v = lv_obj_create(screen);
        lv_obj_remove_style_all(separator_v);
        lv_obj_set_pos(separator_v, half_w, 0);
        lv_obj_set_size(separator_v, 1, screen_h);
        lv_obj_set_style_bg_color(separator_v, lv_color_hex(0x7A8A99), 0);

        lv_obj_t       *separator_h = lv_obj_create(screen);
        lv_obj_remove_style_all(separator_h);
        lv_obj_set_pos(separator_h, 0, top_h);
        lv_obj_set_size(separator_h, screen_w, 1);
        lv_obj_set_style_bg_color(separator_h, lv_color_hex(0x7A8A99), 0);
}
