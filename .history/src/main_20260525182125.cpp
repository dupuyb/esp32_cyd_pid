

#include <lvgl.h>
#include <TFT_eSPI.h>

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

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// If logging is enabled, it will inform the user about what is happening in the library
void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

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

    // Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
    /* Serial.print("X = ");
    Serial.print(x);
    Serial.print(" | Y = ");
    Serial.print(y);
    Serial.print(" | Pressure = ");
    Serial.print(z);
    Serial.println();*/
  }
  else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static lv_obj_t * g_label_control_value;
static lv_obj_t * g_label_pid_state;
static lv_obj_t * g_label_vmc_state;

static void format_temp_1_decimal(char * out, size_t out_size, int tenth_degrees) {
  int whole = tenth_degrees / 10;
  int decimal = tenth_degrees % 10;
  lv_snprintf(out, out_size, "%d.%d", whole, decimal);
}

static void control_slider_event_callback(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  int slider_value = (int)lv_slider_get_value(slider);
  int control_temp_tenth = 200 + (slider_value * 2);  // 20.0 C to 40.0 C
  char temp_text[12];
  format_temp_1_decimal(temp_text, sizeof(temp_text), control_temp_tenth);
  lv_label_set_text(g_label_control_value, temp_text);
  LV_LOG_USER("Control setpoint changed to %s C", temp_text);
}

static void pid_switch_event_callback(lv_event_t * e) {
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  lv_label_set_text(g_label_pid_state, is_on ? "ON" : "OFF");
  lv_obj_set_style_text_color(g_label_pid_state, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
  LV_LOG_USER("PID %s", is_on ? "enabled" : "disabled");
}

static void vmc_switch_event_callback(lv_event_t * e) {
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  lv_label_set_text(g_label_vmc_state, is_on ? "ON" : "OFF");
  lv_obj_set_style_text_color(g_label_vmc_state, is_on ? lv_color_hex(0x107C10) : lv_color_hex(0xC32F27), 0);
  LV_LOG_USER("VMC %s", is_on ? "enabled" : "disabled");
}

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
    lv_style_set_bg_color(&style_card, lv_color_hex(0xF8FAFC));
    lv_style_set_border_width(&style_card, 2);
    lv_style_set_border_color(&style_card, lv_color_hex(0x7A8A99));
    lv_style_set_radius(&style_card, 10);
    lv_style_set_shadow_width(&style_card, 10);
    lv_style_set_shadow_opa(&style_card, LV_OPA_20);
    lv_style_set_shadow_color(&style_card, lv_color_hex(0x1F2937));
    lv_style_set_shadow_ofs_y(&style_card, 2);

    lv_style_init(&style_title);
    lv_style_set_text_color(&style_title, lv_color_hex(0x1F2937));
    lv_style_set_text_font(&style_title, &lv_font_montserrat_16);
    styles_initialized = true;
  }

  lv_obj_t * screen = lv_screen_active();
  lv_obj_add_style(screen, &style_screen, 0);

  lv_obj_t * panel_temp = lv_obj_create(screen);
  lv_obj_set_pos(panel_temp, 6, 6);
  lv_obj_set_size(panel_temp, 168, 108);
  lv_obj_add_style(panel_temp, &style_card, 0);

  lv_obj_t * title_temp = lv_label_create(panel_temp);
  lv_label_set_text(title_temp, "Temperature Interieur");
  lv_obj_add_style(title_temp, &style_title, 0);
  lv_obj_set_pos(title_temp, 10, 8);

  lv_obj_t * label_reading = lv_label_create(panel_temp);
  lv_label_set_text(label_reading, "Lecture :");
  lv_obj_set_pos(label_reading, 10, 38);

  lv_obj_t * label_reading_value = lv_label_create(panel_temp);
  lv_label_set_text(label_reading_value, "33.6");
  lv_obj_set_style_text_color(label_reading_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(label_reading_value, 74, 38);

  lv_obj_t * label_reading_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_reading_unit, "C");
  lv_obj_set_pos(label_reading_unit, 116, 38);

  lv_obj_t * label_control = lv_label_create(panel_temp);
  lv_label_set_text(label_control, "Control :");
  lv_obj_set_pos(label_control, 10, 60);

  g_label_control_value = lv_label_create(panel_temp);
  lv_label_set_text(g_label_control_value, "30.0");
  lv_obj_set_style_text_color(g_label_control_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(g_label_control_value, 74, 60);

  lv_obj_t * label_control_unit = lv_label_create(panel_temp);
  lv_label_set_text(label_control_unit, "C");
  lv_obj_set_pos(label_control_unit, 116, 60);

  lv_obj_t * slider_setpoint = lv_slider_create(panel_temp);
  lv_obj_set_pos(slider_setpoint, 10, 84);
  lv_obj_set_size(slider_setpoint, 148, 10);
  lv_slider_set_range(slider_setpoint, 0, 100);
  lv_slider_set_value(slider_setpoint, 50, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_setpoint, control_slider_event_callback, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xD6DEE5), LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0x3B82F6), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_setpoint, lv_color_hex(0xFFFFFF), LV_PART_KNOB);

  lv_obj_t * panel_pid = lv_obj_create(screen);
  lv_obj_set_pos(panel_pid, 180, 6);
  lv_obj_set_size(panel_pid, 134, 108);
  lv_obj_add_style(panel_pid, &style_card, 0);

  lv_obj_t * title_pid = lv_label_create(panel_pid);
  lv_label_set_text(title_pid, "P.I.D");
  lv_obj_add_style(title_pid, &style_title, 0);
  lv_obj_set_pos(title_pid, 10, 8);

  lv_obj_t * switch_pid = lv_switch_create(panel_pid);
  lv_obj_set_pos(switch_pid, 74, 4);
  lv_obj_set_size(switch_pid, 50, 24);
  lv_obj_add_state(switch_pid, LV_STATE_CHECKED);
  lv_obj_add_event_cb(switch_pid, pid_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t * label_prop = lv_label_create(panel_pid);
  lv_label_set_text(label_prop, "Prop. :");
  lv_obj_set_pos(label_prop, 10, 34);
  lv_obj_t * label_prop_value = lv_label_create(panel_pid);
  lv_label_set_text(label_prop_value, "0.123");
  lv_obj_set_pos(label_prop_value, 72, 34);

  lv_obj_t * label_integ = lv_label_create(panel_pid);
  lv_label_set_text(label_integ, "Integ.:");
  lv_obj_set_pos(label_integ, 10, 56);
  lv_obj_t * label_integ_value = lv_label_create(panel_pid);
  lv_label_set_text(label_integ_value, "5.545");
  lv_obj_set_pos(label_integ_value, 72, 56);

  lv_obj_t * label_deriv = lv_label_create(panel_pid);
  lv_label_set_text(label_deriv, "Deriv.:");
  lv_obj_set_pos(label_deriv, 10, 78);
  lv_obj_t * label_deriv_value = lv_label_create(panel_pid);
  lv_label_set_text(label_deriv_value, "11.130");
  lv_obj_set_pos(label_deriv_value, 72, 78);

  g_label_pid_state = lv_label_create(panel_pid);
  lv_label_set_text(g_label_pid_state, "ON");
  lv_obj_set_style_text_color(g_label_pid_state, lv_color_hex(0x107C10), 0);
  lv_obj_set_pos(g_label_pid_state, 48, 10);

  lv_obj_t * panel_vmc = lv_obj_create(screen);
  lv_obj_set_pos(panel_vmc, 180, 118);
  lv_obj_set_size(panel_vmc, 134, 58);
  lv_obj_add_style(panel_vmc, &style_card, 0);

  lv_obj_t * label_vmc = lv_label_create(panel_vmc);
  lv_label_set_text(label_vmc, "V.M.C");
  lv_obj_add_style(label_vmc, &style_title, 0);
  lv_obj_set_style_text_font(label_vmc, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(label_vmc, 10, 8);

  lv_obj_t * switch_vmc = lv_switch_create(panel_vmc);
  lv_obj_set_pos(switch_vmc, 74, 4);
  lv_obj_set_size(switch_vmc, 50, 24);
  lv_obj_add_state(switch_vmc, LV_STATE_CHECKED);
  lv_obj_add_event_cb(switch_vmc, vmc_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t * label_rpm = lv_label_create(panel_vmc);
  lv_label_set_text(label_rpm, "R.P.M:");
  lv_obj_set_pos(label_rpm, 10, 33);
  lv_obj_t * label_rpm_value = lv_label_create(panel_vmc);
  lv_label_set_text(label_rpm_value, "123");
  lv_obj_set_style_text_color(label_rpm_value, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(label_rpm_value, 66, 33);

  g_label_vmc_state = lv_label_create(panel_vmc);
  lv_label_set_text(g_label_vmc_state, "ON");
  lv_obj_set_style_text_color(g_label_vmc_state, lv_color_hex(0x107C10), 0);
  lv_obj_set_pos(g_label_vmc_state, 44, 10);

  lv_obj_t * panel_status = lv_obj_create(screen);
  lv_obj_set_pos(panel_status, 6, 118);
  lv_obj_set_size(panel_status, 168, 58);
  lv_obj_add_style(panel_status, &style_card, 0);

  lv_obj_t * label_time = lv_label_create(panel_status);
  lv_label_set_text(label_time, "HH:MM:SS");
  lv_obj_set_style_text_font(label_time, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(label_time, lv_color_hex(0x0F3D5E), 0);
  lv_obj_set_pos(label_time, 42, 8);

  lv_obj_t * label_ip_title = lv_label_create(panel_status);
  lv_label_set_text(label_ip_title, "IP:");
  lv_obj_set_pos(label_ip_title, 10, 34);

  lv_obj_t * label_ip_value = lv_label_create(panel_status);
  lv_label_set_text(label_ip_value, "192.168.xxx.yyy");
  lv_obj_set_pos(label_ip_value, 32, 34);
}

void setup() {
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.begin(115200);
  Serial.println(LVGL_Arduino);
  
  // Start LVGL
  lv_init();
  // Register print function for debugging
  lv_log_register_print_cb(log_print);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 0: touchscreen.setRotation(0);
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

  // Function to draw the GUI (text, buttons and sliders)
  lv_create_main_gui();
}

void loop() {
  lv_task_handler();  // let the GUI do its work
  lv_tick_inc(5);     // tell LVGL how much time has passed
  delay(5);           // let this time pass
}