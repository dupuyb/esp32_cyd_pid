

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

#include "gui.h"

static String formatIpAddress(const IPAddress& ip) {
  if(ip == IPAddress((uint32_t)0)) {
    return "not connected";
  }
  return ip.toString();
}

static void set_pwm_percent(float percent) {
  // Clamp command to a valid duty-cycle percentage before converting to timer ticks.
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
  // Fail-safe: if PID is disabled or sensor value is invalid, force output to 0%.
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

  // Classic PID terms computed in engineering units (degC error and seconds).
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
  // DHT22 is relatively slow and should not be polled too fast.
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
  // Slider stores tenth-degrees to avoid float rounding artifacts in UI interactions.
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
  // VMC switch commands PWM only when PID mode is disabled.
  if(!g_pid_enabled) {
    set_pwm_percent(is_on ? 100.0f : 0.0f);
    LV_LOG_USER("Manual VMC %s -> PWM %.0f%%", is_on ? "ON" : "OFF", g_pwm_percent);
    return;
  }
  LV_LOG_USER("VMC %s", is_on ? "enabled" : "disabled");
}

void lv_create_main_gui(void) {
  lv_create_main_gui();
  update_access_network_labels(formatIpAddress(WiFi.localIP()), WiFi.macAddress());
}

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
  // CYD is physically portrait but GUI layout is designed in landscape coordinates.
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
  // LVGL timing loop: process events then advance internal tick base.
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
