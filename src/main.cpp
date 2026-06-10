#include "gui.h"
#include "main.h"
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <math.h>

// ============== Framework and Core Includes ==============
#include "FrameWeb.h"
FrameWeb frame; // Web server and configuration framework instance

#define WIFI_TASK_PRIORITY 5 // Low numbers denote low priority tasks
TaskHandle_t wifiCxHandle = NULL;
#define LVGL_TASK_PRIORITY 12 // Higher priority than WiFi task to keep GUI responsive
SemaphoreHandle_t guiMutex;

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the
// touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen.
// No extra configuration is required for this project.
#include <XPT2046_Touchscreen.h>

// Touchscreen pins
#define XPT2046_IRQ 36  // T_IRQ
#define XPT2046_MOSI 32 // T_DIN
#define XPT2046_MISO 39 // T_OUT
#define XPT2046_CLK 25  // T_CLK
#define XPT2046_CS 33   // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// DHT22 pin to sensor.
#define DHT22_PIN 27
// LED RGB pins
#define RGB_LED_R_PIN 4
#define RGB_LED_G_PIN 16
#define RGB_LED_B_PIN 17
// Backlight control pin
#define BACKLIGHT_PIN 21
// Fan PWM output pin (LEDC attached in setup)
#define VMC_PWM_PIN 22

// LEDC channel used for fan PWM output.
#define PWM_CHANNEL 0
#define PWM_FREQUENCY_HZ 20000
#define PWM_RESOLUTION_BITS 8
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION_BITS) - 1)
// Backlight PWM settings
#define BACKLIGHT_PWM_CHANNEL 1
#define BACKLIGHT_BRIGHT_PERCENT_ACTIVE 100.0f
#define BACKLIGHT_BRIGHT_PERCENT_SCREENSAVER 20.0f

// RGB LED on these GPIOs.
#define RGB_LED_ACTIVE_HIGH 0

// Temperature and humidity control variables
static DHTesp g_dht;
static int g_websocket_client_num = -1;
bool sensorOk = false;
int sensorError = 0;

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// If logging is enabled, it will inform the user about what is happening in the library
void log_print(lv_log_level_t level, const char *buf) { LV_UNUSED(level); }

// ============== Time Configuration ==============
/** @brief UTC offset in seconds (3600 = UTC+1 for Central European Time) */
const long gmtOffset_sec = 3600;

/** @brief Daylight saving offset in seconds (3600 = +1 hour for summer time) */
const int daylightOffset_sec = 3600; // Central European Summer Time (CEST) offset

/** @brief Broken-down time structure (updated every 5 seconds in loop) */
struct tm timeinfo;

/** @brief NTP server pool for time synchronization */
const char *ntpServer = "pool.ntp.org";

// counter for flow duration in seconds
long previousMillis = 0;
static uint32_t g_last_sensor_refresh_ms = 0;
#define SCREEN_SAVER_TIMEOUT_MS (5UL * 60UL * 1000UL)
static uint32_t g_last_user_activity_ms = 0;
// Tracks whether screen saver dimming is currently applied.
static bool g_backlight_dimmed = false;

static void set_backlight_percent(float percent);

static void apply_screen_mode(bool screen_saver_active) {
  int pv = PAGE_MAIN; // Main page by default.
  if (pageVisible!=PAGE_SAVER) 
    pv=pageVisible; // Graph page if it was active before.
  const int target_page = screen_saver_active ? PAGE_SAVER : pv;
  if (pageVisible != target_page) {
    gui_setPage(target_page);
  }
  const float target_brightness = screen_saver_active
                                      ? BACKLIGHT_BRIGHT_PERCENT_SCREENSAVER
                                      : BACKLIGHT_BRIGHT_PERCENT_ACTIVE;
  const bool should_be_dimmed = screen_saver_active;
  // Apply backlight changes only when mode really changes.
  if (g_backlight_dimmed != should_be_dimmed) {
    set_backlight_percent(target_brightness);
    g_backlight_dimmed = should_be_dimmed;
  }
}

static void keep_screen_awake(void) {
  g_last_user_activity_ms = millis();
  apply_screen_mode(false);
}

static void set_backlight_percent(float percent) {
  // Clamp to a safe range before converting to LEDC duty-cycle.
  if (percent < 0.0f) 
    percent = 0.0f;
  if (percent > 100.0f) 
    percent = 100.0f;
  uint32_t duty = (uint32_t)((percent * (float)PWM_MAX_DUTY / 100.0f) + 0.5f);
  ledcWrite(BACKLIGHT_PWM_CHANNEL, duty);
}

// Get the Touchscreen data
void touchscreen_read(lv_indev_t *indev, lv_indev_data_t *data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if (touchscreen.tirqTouched() && touchscreen.touched()) {

    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 220, 3770, 1, SCREEN_WIDTH);
    y = map(p.y, 420, 3950, 1, SCREEN_HEIGHT);
    z = p.z;

    keep_screen_awake();
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static String formatIpAddress(const IPAddress &ip) {
  if (ip == IPAddress((uint32_t)0)) {
    return "not connected";
  }
  return ip.toString();
}

static void webSocketSendState(uint8_t num) {
  JsonDocument readings;
  readings["type"] = "pid_state";
  readings["historyPoints"] = WEB_GRAPH_HISTORY_POINTS;
  readings["setpoint"] = g_setpoint_temp_c;
  readings["temperature"] = sensorData.temperature;
  readings["humidity"] = lv_label_get_text(g_label_humidity_value);
  readings["pwm"] = g_pwm_percent;
  readings["pidEnabled"] = g_pid_enabled;
  readings["vmcManualOn"] = g_vmc_manual_on;
  readings["kp"] = g_kp;
  readings["ki"] = g_ki;
  readings["kd"] = g_kd;

  String json_string;
  serializeJson(readings, json_string);
  frame.webSocket.sendTXT(num, json_string);
}

static void webSocketSendError(uint8_t num, const char *message) {
  JsonDocument response;
  response["type"] = "error";
  response["message"] = message;
  String json_string;
  serializeJson(response, json_string);
  LV_LOG_USER("WS send error -> client:%u msg:%s", (unsigned)num, message);
  frame.webSocket.sendTXT(num, json_string);
}

static void set_pwm_percent(float percent) {
  // Clamp command to a valid duty-cycle percentage before converting to timer ticks.
  if (percent < 0.0f) {
    percent = 0.0f;
  }
  if (percent > 100.0f) {
    percent = 100.0f;
  }
  g_pwm_percent = percent;
  // Map fan command.
  uint32_t pwm_fan = map((long)g_pwm_percent, 0, 100, 1, 254);
  ledcWrite(PWM_CHANNEL, pwm_fan);
}

void compute_pid_and_drive_output(void) {
  // Fail-safe: if PID is disabled or sensor value is invalid, force output to 0%.
  if ( isnan(sensorData.temperature)) {
    set_pwm_percent(0.0f);
    return;
  }

  if (!g_pid_enabled ){
    set_pwm_percent(g_vmc_manual_on ? 100.0f : 0.0f);
    return;
  }

  uint32_t now = millis();
  float dt_sec = 0.0f;
  if (g_last_pid_compute_ms == 0) {
    dt_sec = 2.0f;
  } else {
    dt_sec = (float)(now - g_last_pid_compute_ms) / 1000.0f;
    if (dt_sec <= 0.0f) {
      dt_sec = 0.001f;
    }
  }
  g_last_pid_compute_ms = now;

  // In cooling mode: positive error means room is hotter than setpoint.
  float error = sensorData.temperature - g_setpoint_temp_c;
  g_pid_integral += error * dt_sec;

  // Anti wind-up for bounded 0-100% output.
  if (g_pid_integral > 100.0f) {
    g_pid_integral = 100.0f;
  }
  if (g_pid_integral < -100.0f) {
    g_pid_integral = -100.0f;
  }

  float derivative = 0.0f;
  if (g_pid_has_prev_error) {
    derivative = (error - g_pid_prev_error) / dt_sec;
  }

  float pid_output = (g_kp * error) + (g_ki * g_pid_integral) + (g_kd * derivative);
  g_pid_prev_error = error;
  g_pid_has_prev_error = true;
  set_pwm_percent(pid_output);
}

static void decodePidJsonAndApply(uint8_t num, uint8_t *payload, size_t length) {

  JsonDocument root;
  DeserializationError error = deserializeJson(root, payload, length);
  if (error) {
    webSocketSendError(num, "Invalid JSON payload");
    return;
  }

  bool need_recompute = false;

  if (root["setpoint"].is<float>() || root["setpoint"].is<int>()) {
    float value = root["setpoint"].as<float>();
    if (value < 25.0f) {
      value = 25.0f;
    }
    if (value > 45.0f) {
      value = 45.0f;
    }
    g_setpoint_temp_c = value;
    control_slider_callback();
    need_recompute = true;
    must_be_saved = true;
  }

  if (root["kp"].is<float>() || root["kp"].is<int>()) {
    float value = root["kp"].as<float>();
    if (value < 0.0f) {
      value = 0.0f;
    }
    if (value > 25.0f) {
      value = 25.0f;
    }
    g_kp = value;
    update_pid_value_label(g_label_kp_value, g_kp);
    need_recompute = true;
    must_be_saved = true;
  }

  if (root["ki"].is<float>() || root["ki"].is<int>()) {
    float value = root["ki"].as<float>();
    if (value < 0.0f) {
      value = 0.0f;
    }
    if (value > 50.0f) {
      value = 50.0f;
    }
    g_ki = value;
    update_pid_value_label(g_label_ki_value, g_ki);
    need_recompute = true;
    must_be_saved = true;
  }

  if (root["kd"].is<float>() || root["kd"].is<int>()) {
    float value = root["kd"].as<float>();
    if (value < 0.0f) {
      value = 0.0f;
    }
    if (value > 50.0f) {
      value = 50.0f;
    }
    g_kd = value;
    update_pid_value_label(g_label_kd_value, g_kd);
    need_recompute = true;
    must_be_saved = true;
  }

  if (root["pidEnabled"].is<bool>()) {
    g_pid_enabled = root["pidEnabled"];
    switch_callback(g_label_pid_state, g_pid_enabled);
    if (!g_pid_enabled) {
      g_pid_integral = 0.0f;
      g_pid_prev_error = 0.0f;
      g_pid_has_prev_error = false;
      set_pwm_percent(g_vmc_manual_on ? 100.0f : 0.0f);
    } else {
      g_last_pid_compute_ms = 0;
      need_recompute = true;
    }
  }

  if (root["vmcManualOn"].is<bool>()) {
    g_vmc_manual_on = root["vmcManualOn"];
    switch_callback(g_label_vmc_state, g_vmc_manual_on);
    if (!g_pid_enabled) {
      set_pwm_percent(g_vmc_manual_on ? 100.0f : 0.0f);
    }
  }

  if (need_recompute && g_pid_enabled) {
    compute_pid_and_drive_output();
  }

  webSocketSendState(num);
}

static void turn_off_rgb_led(void) {
  digitalWrite(RGB_LED_R_PIN, HIGH);
  digitalWrite(RGB_LED_G_PIN, HIGH);
  digitalWrite(RGB_LED_B_PIN, HIGH);
}

static void refresh_widget_lcd() {

  if (sensorOk){
    // Sensor
    update_dht_values();
    update_graph_history();

    // Update percent.
    set_pwm_values();

    compute_pid_and_drive_output();

  }

  // update time every 1 second
  update_time_label();

  // update IP & MAC Address.
  update_access_network_labels(formatIpAddress(WiFi.localIP()), WiFi.macAddress());

  // Do not enable screen saver while MAC is visible: network/time init is still ongoing.
  bool mac_label_visible = (g_label_mac_line != NULL) && !lv_obj_has_flag(g_label_mac_line, LV_OBJ_FLAG_HIDDEN);
  if (mac_label_visible) {
      keep_screen_awake();
  } else {
      const uint32_t now = millis();
      if (g_last_user_activity_ms == 0) {
        g_last_user_activity_ms = now;
      }
      const bool screen_saver_active =
          (uint32_t)(now - g_last_user_activity_ms) >= SCREEN_SAVER_TIMEOUT_MS;
      // Screen saver: if no touch for a while, switch to clock/network page and dim.
      apply_screen_mode(screen_saver_active);
  }
}

bool get_dht_values() {
  if (g_dht.getStatus() != DHTesp::ERROR_NONE) {
      LV_LOG_USER("DHT22 bad status.");
    return false;
  }
  // Sample the DHT and update both UI values and control loop input.
  sensorData = g_dht.getTempAndHumidity();
  if (isnan(sensorData.temperature) || isnan(sensorData.humidity)) {
    LV_LOG_USER("DHT22 read failed.");
    return false;
  }
  // Serial.printf("Temperature: %.2f C, Humidity: %.2f%%\n", sensorData.temperature, sensorData.humidity);
  return true;
}

void control_slider_event_callback(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  // Slider stores tenth-degrees to avoid float rounding artifacts in UI interactions.
  int control_temp_tenth = (int)lv_slider_get_value(slider); // 25.0 C to 45.0 C
  g_setpoint_temp_c = (float)control_temp_tenth / 10.0f;
  control_slider_callback();
  must_be_saved = true;
  compute_pid_and_drive_output();
}

void pid_switch_event_callback(lv_event_t *e) {
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
  bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  g_pid_enabled = is_on;
  switch_callback(g_label_pid_state, is_on);
  if (!is_on) {
    g_pid_integral = 0.0f;
    g_pid_prev_error = 0.0f;
    g_pid_has_prev_error = false;
    // PID disabled: manual ventilation switch controls output as 0% or 100%.
    set_pwm_percent(g_vmc_manual_on ? 100.0f : 0.0f);
  } else {
    g_last_pid_compute_ms = 0;
    compute_pid_and_drive_output();
  }
}

void vmc_switch_event_callback(lv_event_t *e) {
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
  bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  g_vmc_manual_on = is_on;
  switch_callback(g_label_vmc_state, is_on);
  // Ventilation switch commands PWM only when PID mode is disabled.
  if (!g_pid_enabled) {
    set_pwm_percent(is_on ? 100.0f : 0.0f);
    LV_LOG_USER("Manual ventilation %s -> PWM %.0f%%", is_on ? "ON" : "OFF", g_pwm_percent);
    return;
  }
}

// ============== FrameWeb Framework Callbacks ==============
/** @brief Called when configuration is saved via web interface (not used in
 * this app) */
void saveConfigCallback() {}

/** @brief Called when WebSocket events occur (not used in this app) */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    // LV_LOG_USER("WS connected: client:%u", (unsigned)num);
    g_websocket_client_num = (int)num;
    webSocketSendState(num);
    break;
  case WStype_DISCONNECTED:
   // LV_LOG_USER("WS disconnected: client:%u", (unsigned)num);
    if (g_websocket_client_num == (int)num) {
      g_websocket_client_num = -1;
    }
    break;
  case WStype_TEXT:
    //LV_LOG_USER("WS text event: client:%u", (unsigned)num);
    g_websocket_client_num = (int)num;
    decodePidJsonAndApply(num, payload, length);
    break;
  default:
    LV_LOG_USER("WS event type:%d client:%u", (int)type, (unsigned)num);
    break;
  }
}

// Called when WiFiManager not used here.
void configModeCallback(WiFiManager *myWiFiManager) {}

void IRAM_ATTR startingTask(void *pvParameter) {
  // Run network/framework boot asynchronously to keep GUI startup responsive.
  delay(2000); // Let the system stabilize before starting WiFi connection
  // Initialize FrameWeb framework (web server, WebSocket, config management)
  frame.setup();
  // Configure system time from NTP server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // Start NTP sync
  // Set timezone: CET-1CEST,M3.5.0,M10.5.0/3 = Central European Time with DST
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset(); // Apply timezone settings
  // Read current time from system
  getLocalTime(&timeinfo);
  // External tools.
  frame.externalHtmlTools = "<div class='action-item'><div>- PID page:</div>"
                            "<div class='button-group'><a class='button' href='/pid.html'>PID Web</a></div></div>";
  vTaskDelete(NULL); // auto destroy
}

// Task dedicated to LVGL processing. Runs the LVGL handler in a loop with a
// delay to target roughly 50 FPS.
void IRAM_ATTR lvglTask(void *pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(20));
    while (1) {
      if (xSemaphoreTake(guiMutex, portMAX_DELAY) == pdTRUE) {
        lv_task_handler();
        lv_tick_inc(20);    // tell LVGL how much time has passed
        xSemaphoreGive(guiMutex);
      }
      // 20ms equivalent to 50 FPS.
      vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setup() {
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.begin(115200);

  delay(200);
  Serial.println(LVGL_Arduino);

  // Initialize DHT sensor on pin GPIO27
  g_dht.setup(DHT22_PIN, DHTesp::DHT22);

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
  lv_display_t *disp;
  // Initialize the TFT display using the TFT_eSPI library
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  // CYD is physically portrait but GUI layout is designed in landscape coordinates.
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

  // Initialize an LVGL input device object (Touchscreen)
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  // Set the callback function to read Touchscreen input
  lv_indev_set_read_cb(indev, touchscreen_read);

  // Display backlight control on GPIO21.
  ledcSetup(BACKLIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(BACKLIGHT_PIN, BACKLIGHT_PWM_CHANNEL);
  set_backlight_percent(BACKLIGHT_BRIGHT_PERCENT_ACTIVE);
  g_last_user_activity_ms = millis();

  // Fan PWM control on GPIO22.
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(VMC_PWM_PIN, PWM_CHANNEL);
  set_pwm_percent(0.0f);

  // Mount SPIFFS and restore PID gains before building the UI.
  bool g_pid_fs_ready = SPIFFS.begin(false);
  if (!g_pid_fs_ready) {
    Serial.println("SPIFFS mount failed, PID settings will not be persisted");
  } else {
    load_pid_settings_from_file();
  }

  // Function to draw the GUI (text, buttons and sliders)
  lv_create_gui();

  // create Mutex
  guiMutex = xSemaphoreCreateMutex();

  // Start the WiFi connection and web server in a separate task to avoid blocking
  LV_LOG_USER("Thread connectTask starting...");
  xTaskCreate(&startingTask, "startingTask", 4096, NULL, WIFI_TASK_PRIORITY, &wifiCxHandle);

  // Start the LVGL task loop in a separate task to keep the GUI responsive.
  LV_LOG_USER("Thread LVGL loop starting...");
  xTaskCreate(&lvglTask, "LVGL", 4096*2, NULL, LVGL_TASK_PRIORITY, NULL);
}

void loop() {

  // Keep FrameWeb services running.
  frame.loop();

  // Alive section, executed every second
  if (millis() - previousMillis > 1000L) {
    previousMillis = millis();

    // Update DHT22 readings and history every 5 seconds to keep the chart aligned with real data points.
    if (g_last_sensor_refresh_ms == 0 || (uint32_t)(previousMillis - g_last_sensor_refresh_ms) >= 5000UL) {
      g_last_sensor_refresh_ms = previousMillis;
      sensorOk = get_dht_values();
      if (!sensorOk) {
        sensorError++;
        if (sensorError > 9) {
          // Initialize DHT sensor on pin GPIO27
          LV_LOG_USER("DHT22 sensor error count exceeded threshold, reinitializing sensor...");
          g_dht.setup(DHT22_PIN, DHTesp::DHT22);
          sensorError = 0;
        }
      } else {
        compute_pid_and_drive_output();
      }

      // Update WebSocket clients with current state when fresh sensor data is available.
      webSocketSendState((uint8_t)g_websocket_client_num);

      if (must_be_saved) {
        save_pid_settings();
      }
    }

    // Update time every second
    if (getLocalTime(&timeinfo)) {
      char temp[9];
      strftime(temp, sizeof(temp), "%H:%M:%S", &timeinfo);
      currentTime = String(temp);
    }

    // update all graphical widgets.
    if (xSemaphoreTake(guiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      refresh_widget_lcd();
      xSemaphoreGive(guiMutex);
    }

  } // end seconde.
}