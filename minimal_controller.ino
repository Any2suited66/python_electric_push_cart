/*
 * Arduino Nano ESP32 - Minimal Controller (Based on Working ESP-NOW Test)
 * 
 * This is a clean, minimal version based on our working ESP-NOW test script
 * with only essential functionality to avoid crashes.
 * 
 * Wiring:
 * - Joystick VCC -> 3.3V, GND -> GND, X -> A0, Y -> A1, Button -> D2
 * - Button 1 -> D3, Button 2 -> D4, Button 3 -> D5, Button 4 -> D6
 */

#include <WiFi.h>
#include <esp_now.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <MPU6050.h>

// Pin definitions (X and Y swapped)
#define JOYSTICK_X_PIN A1        // A1 = GPIO2
#define JOYSTICK_Y_PIN A0        // A0 = GPIO1
#define JOYSTICK_BUTTON_PIN D2   // D2 = GPIO5
#define BUTTON_1_PIN D3          // D3 = GPIO6
#define BUTTON_2_PIN D4          // D4 = GPIO7
#define BUTTON_3_PIN D5          // D5 = GPIO21
#define BUTTON_4_PIN D6          // D6 = GPIO20
#define LOW_BATTERY_PIN D12      // D12 = PowerBoost 1000C LBO pin

// Display pins (separate from MPU6050)
#define DISPLAY_SDA_PIN 8        // GPIO8 for Display SDA (U8g2 software I2C)
#define DISPLAY_SCL_PIN 9        // GPIO9 for Display SCL (U8g2 software I2C)

// MPU6050 I2C pins (separate from display)
#define MPU_SDA_PIN 10           // GPIO10 for MPU6050 SDA
#define MPU_SCL_PIN 11           // GPIO11 for MPU6050 SCL

// ESP-NOW Configuration
#define BROADCAST_MAC_ADDRESS {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// Display object (SSD1306 128x32 using U8g2 software I2C) - Flipped 180 degrees
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C display(U8G2_R2, DISPLAY_SCL_PIN, DISPLAY_SDA_PIN);

// MPU6050 object
MPU6050 mpu;

// Data structure for ESP-NOW communication (with emergency stop)
typedef struct __attribute__((packed)) {
  int16_t joystick_x;
  int16_t joystick_y;
  uint8_t button_states;
  uint8_t battery_level;
  uint8_t operation_mode;
  bool cruise_control_active;
  int16_t cruise_speed;
  bool emergency_stop;  // NEW: Explicit emergency stop flag for wheel locking
} controller_data_t;

// Global variables (minimal set)
controller_data_t controller_data;
uint8_t broadcast_mac[] = BROADCAST_MAC_ADDRESS;
esp_now_peer_info_t broadcast_peer;

// Display state
bool display_connected = false;
unsigned long last_display_update = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500; // Update every 500ms

// MPU6050 state
bool mpu_connected = false;
bool mpu_initialized = false;
bool gesture_control_enabled = false;
unsigned long last_mpu_read = 0;
const unsigned long MPU_READ_INTERVAL = 100; // Read every 100ms

// MPU6050 data
int16_t mpu_accel_x = 0, mpu_accel_y = 0, mpu_accel_z = 0;
int16_t mpu_gyro_x = 0, mpu_gyro_y = 0, mpu_gyro_z = 0;

// Transmission tracking (simplified)
unsigned long last_send_time = 0;
const unsigned long SEND_INTERVAL = 25; // 40Hz
unsigned long consecutive_failures = 0;

// Button states (simplified)
bool last_button_states[5] = {false, false, false, false, false};
bool joystick_button_was_pressed = false;

// Operation modes
enum OperationMode {
  MODE_NORMAL = 0,
  MODE_TURBO = 1,
  MODE_FOLLOW_ME = 2,
  MODE_PARKING = 3
};

OperationMode current_mode = MODE_NORMAL;
bool cruise_control_enabled = false;
int16_t current_cruise_speed = 0;

// Emergency stop
bool emergency_stop_enabled = false;

// Battery monitoring
bool low_battery_detected = false;
uint8_t battery_level = 85; // Default battery level

// Function declarations
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void initializeDisplay();
void updateDisplay();
void displayStatus(const char* message);
void initializeMPU6050();
void readMPU6050();
void handleGestureControl();
void readBatteryStatus();

void setup() {
  // Optimize stack and memory usage
  Serial.begin(115200);
  delay(1000);
  
  // Print memory information
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
  Serial.printf("Chip model: %s\n", ESP.getChipModel());
  Serial.printf("Chip cores: %d\n", ESP.getChipCores());
  
  // Initialize pins
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  pinMode(LOW_BATTERY_PIN, INPUT_PULLUP);  // PowerBoost 1000C LBO pin
  
  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  Serial.println("=================================");
  Serial.println("Minimal Controller - Clean Version");
  Serial.println("=================================");
  
  // Initialize display
  initializeDisplay();
  
  // MPU6050 will be initialized when Button 3 is pressed
  Serial.println("MPU6050 initialization deferred - press Button 3 to activate");
  
  // Initialize WiFi and ESP-NOW (same as working test)
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add broadcast peer
  memcpy(broadcast_peer.peer_addr, broadcast_mac, 6);
  broadcast_peer.channel = 1;
  broadcast_peer.encrypt = false;
  
  if (esp_now_add_peer(&broadcast_peer) != ESP_OK) {
    Serial.println("❌ Failed to add broadcast peer");
  } else {
    Serial.println("✅ Broadcast peer added successfully");
  }
  
  Serial.println("ESP-NOW initialized successfully");
  Serial.printf("📏 Data structure size: %d bytes\n", sizeof(controller_data_t));
  Serial.println("=================================");
  Serial.println("All systems ready!");
  Serial.println("Move joystick and press buttons!");
  Serial.println("=================================");
  
  // Show initial status on display
  displayStatus("Ready!");
}

void loop() {
  // Read joystick (same as working test)
  static int x_raw = analogRead(JOYSTICK_X_PIN);
  static int y_raw = analogRead(JOYSTICK_Y_PIN);
  
  x_raw = analogRead(JOYSTICK_X_PIN);
  y_raw = analogRead(JOYSTICK_Y_PIN);
  
  // Map to -512 to 512 range
  static int16_t joystick_x_raw = 0;
  static int16_t joystick_y_raw = 0;
  
  joystick_x_raw = map(x_raw, 0, 4095, 512, -512); // Reversed X-axis (left/right)
  joystick_y_raw = map(y_raw, 0, 4095, -512, 512);
  
  controller_data.joystick_x = joystick_x_raw;
  controller_data.joystick_y = joystick_y_raw;
  
  // Apply deadzone and minimum threshold (same as working test)
  if (abs(controller_data.joystick_x) < 50) {
    controller_data.joystick_x = 0;
  } else {
    if (controller_data.joystick_x > 0) {
      controller_data.joystick_x = max((int16_t)100, controller_data.joystick_x);
    } else {
      controller_data.joystick_x = min((int16_t)-100, controller_data.joystick_x);
    }
  }
  
  if (abs(controller_data.joystick_y) < 50) {
    controller_data.joystick_y = 0;
  } else {
    if (controller_data.joystick_y > 0) {
      controller_data.joystick_y = max((int16_t)100, controller_data.joystick_y);
    } else {
      controller_data.joystick_y = min((int16_t)-100, controller_data.joystick_y);
    }
  }
  
  // TURBO MODE DISABLED - No boost applied
  // if (current_mode == MODE_TURBO) {
  //   // Apply 1.5x multiplier to joystick values (50% boost)
  //   controller_data.joystick_x = (int16_t)(controller_data.joystick_x * 1.5);
  //   controller_data.joystick_y = (int16_t)(controller_data.joystick_y * 1.5);
  //   
  //   // Cap at maximum values
  //   if (controller_data.joystick_x > 512) controller_data.joystick_x = 512;
  //   if (controller_data.joystick_x < -512) controller_data.joystick_x = -512;
  //   if (controller_data.joystick_y > 512) controller_data.joystick_y = 512;
  //   if (controller_data.joystick_y < -512) controller_data.joystick_y = -512;
  // }
  
  // Read buttons (simplified)
  controller_data.button_states = 0;
  if (digitalRead(BUTTON_1_PIN) == LOW) controller_data.button_states |= (1 << 0);
  if (digitalRead(BUTTON_2_PIN) == LOW) controller_data.button_states |= (1 << 1);
  if (digitalRead(BUTTON_3_PIN) == LOW) controller_data.button_states |= (1 << 2);
  if (digitalRead(BUTTON_4_PIN) == LOW) controller_data.button_states |= (1 << 3);
  if (digitalRead(JOYSTICK_BUTTON_PIN) == LOW) controller_data.button_states |= (1 << 5);
  
  // Handle button press events (simplified)
  static bool button_states[5] = {false, false, false, false, false};
  button_states[0] = (digitalRead(BUTTON_1_PIN) == LOW);
  button_states[1] = (digitalRead(BUTTON_2_PIN) == LOW);
  button_states[2] = (digitalRead(BUTTON_3_PIN) == LOW);
  button_states[3] = (digitalRead(BUTTON_4_PIN) == LOW);
  button_states[4] = (digitalRead(JOYSTICK_BUTTON_PIN) == LOW);
  
  // Check for button press events
  for (int i = 0; i < 4; i++) {
    if (button_states[i] && !last_button_states[i]) {
      Serial.printf("Button %d pressed\n", i + 1);
      
      switch (i) {
        case 0: // Button 1 - Toggle operation mode
          current_mode = (OperationMode)((current_mode + 1) % 4);
          Serial.printf("Operation mode changed to %d\n", current_mode);
          break;
        case 1: // Button 2 - Toggle cruise control
          cruise_control_enabled = !cruise_control_enabled;
          if (cruise_control_enabled) {
            // Check if cart is in reverse - don't allow cruise control in reverse
            if (joystick_y_raw < 0) {
              cruise_control_enabled = false;
              current_cruise_speed = 0;
              Serial.println("🚗 Cruise control not allowed in REVERSE");
              displayStatus("No Cruise REV");
            } else {
              // Use the RAW joystick value (before deadzone/threshold processing)
              current_cruise_speed = joystick_y_raw;
              Serial.printf("🚗 Cruise control ON at speed: %d (raw Y: %d, processed Y: %d)\n", 
                            current_cruise_speed, joystick_y_raw, controller_data.joystick_y);
              displayStatus("Cruise ON");
            }
          } else {
            current_cruise_speed = 0;
            Serial.printf("🚗 Cruise control OFF\n");
            displayStatus("Cruise OFF");
          }
          break;
        case 2: // Button 3 - Summon/resume gestures (handled by Pi in follow mode)
          Serial.println("📍 Button 3 — summon/resume (Pi)");
          displayStatus("Summon/Resume");
          break;
        case 3: // Button 4 - unused
          Serial.println("Button 4 — unused");
          displayStatus("Btn4 unused");
          break;
      }
    }
    last_button_states[i] = button_states[i];
  }
  
  // Handle joystick button for emergency stop
  bool joystick_pressed = button_states[4];
  if (joystick_pressed && !joystick_button_was_pressed) {
    emergency_stop_enabled = !emergency_stop_enabled;
    Serial.printf("🛑 Emergency stop mode %s\n", emergency_stop_enabled ? "ENABLED" : "disabled");
    displayStatus(emergency_stop_enabled ? "E-STOP ON!" : "E-Stop OFF");
    
    // If emergency stop is enabled, disable cruise control immediately
    if (emergency_stop_enabled && cruise_control_enabled) {
      cruise_control_enabled = false;
      current_cruise_speed = 0;
      Serial.println("🚗 Cruise control disabled due to emergency stop");
    }
  }
  joystick_button_was_pressed = joystick_pressed;
  
  // CRUISE CONTROL SPEED ADJUSTMENT - Fine-tune speed while active
  if (cruise_control_enabled) {
    static unsigned long joystick_hold_start = 0;
    static unsigned long last_continuous_adjustment = 0;
    static bool quick_adjustment_made = false;
    
    unsigned long current_time = millis();
    
    // Check if joystick is being pushed forward or backward
    bool joystick_forward = (joystick_y_raw > 300);
    bool joystick_backward = (joystick_y_raw < -300);
    
    if (joystick_forward || joystick_backward) {
      // Start tracking hold time
      if (joystick_hold_start == 0) {
        joystick_hold_start = current_time;
      }
      
      unsigned long hold_duration = current_time - joystick_hold_start;
      
      // Quick adjustment: Single tap (< 1 second) = ±15
      if (hold_duration < 1000 && !quick_adjustment_made) {
        if (current_time - last_continuous_adjustment >= 500) { // 500ms cooldown for quick adjustment
          if (joystick_forward) {
            current_cruise_speed += 15;
            if (current_cruise_speed > 512) current_cruise_speed = 512;
            Serial.printf("🚗 Quick INCREASE: %d\n", current_cruise_speed);
          } else if (joystick_backward) {
            current_cruise_speed -= 15;
            if (current_cruise_speed < -512) current_cruise_speed = -512;
            Serial.printf("🚗 Quick DECREASE: %d\n", current_cruise_speed);
          }
          quick_adjustment_made = true;
          last_continuous_adjustment = current_time;
        }
      }
      // Continuous adjustment: Hold > 1 second = ±5 every 200ms
      else if (hold_duration >= 1000) {
        if (current_time - last_continuous_adjustment >= 200) { // Every 200ms
          if (joystick_forward) {
            current_cruise_speed += 5;
            if (current_cruise_speed > 512) current_cruise_speed = 512;
            Serial.printf("🚗 Continuous INCREASE: %d\n", current_cruise_speed);
          } else if (joystick_backward) {
            current_cruise_speed -= 5;
            if (current_cruise_speed < -512) current_cruise_speed = -512;
            Serial.printf("🚗 Continuous DECREASE: %d\n", current_cruise_speed);
          }
          last_continuous_adjustment = current_time;
        }
      }
    } else {
      // Joystick returned to center - reset tracking
      joystick_hold_start = 0;
      quick_adjustment_made = false;
    }
    
    // Disable cruise control if speed is 0 or very close to 0
    if (abs(current_cruise_speed) < 10) {
      cruise_control_enabled = false;
      current_cruise_speed = 0;
      Serial.println("🚗 Cruise control disabled - cart not moving (speed ~0)");
      displayStatus("Cruise OFF: 0");
    }
  }
  
  // MPU6050/gesture control DISABLED
  // if (mpu_connected && gesture_control_enabled) {
  //   readMPU6050();
  //   handleGestureControl();
  // }
  
  // Read battery status
  readBatteryStatus();
  
  // EMERGENCY STOP OVERRIDE - Set all movement to zero and lock wheels
  if (emergency_stop_enabled) {
    controller_data.joystick_x = 0;
    controller_data.joystick_y = 0;
    controller_data.cruise_control_active = false;
    controller_data.cruise_speed = 0;
    controller_data.emergency_stop = true;  // Signal receiver to lock wheels
  } else {
    controller_data.emergency_stop = false;  // Normal operation
  }
  
  // Set other data fields
  controller_data.battery_level = battery_level;
  controller_data.operation_mode = current_mode;
  
  // Only set cruise control if NOT in emergency stop
  if (!emergency_stop_enabled) {
    controller_data.cruise_control_active = cruise_control_enabled;
    controller_data.cruise_speed = current_cruise_speed;
  } else {
    controller_data.cruise_control_active = false;
    controller_data.cruise_speed = 0;
  }
  
  // Send data via ESP-NOW at 40Hz (same as working test)
  if (millis() - last_send_time >= SEND_INTERVAL) {
    esp_err_t result = esp_now_send(broadcast_mac, (uint8_t*)&controller_data, sizeof(controller_data));
    
    if (result == ESP_OK) {
      // Success - reset failure counter
      if (consecutive_failures > 0) {
        consecutive_failures = 0;
      }
    } else {
      consecutive_failures++;
      if (consecutive_failures % 10 == 1) {
        Serial.printf("❌ Send failed: %d (failures: %lu)\n", result, consecutive_failures);
      }
    }
    
    last_send_time = millis();
  }
  
  // Update display every 500ms
  if (display_connected && millis() - last_display_update >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    last_display_update = millis();
  }
  
  // Print status every 3 seconds
  static unsigned long last_print = 0;
  if (millis() - last_print > 3000) {
    Serial.printf("📡 Status: X=%4d Y=%4d Buttons=0x%02X | Mode: %d | Cruise: %s | E-Stop: %s | TX: %s\n",
                  controller_data.joystick_x,
                  controller_data.joystick_y,
                  controller_data.button_states,
                  current_mode,
                  cruise_control_enabled ? "ON" : "OFF",
                  emergency_stop_enabled ? "ACTIVE" : "off",
                  consecutive_failures < 10 ? "GOOD" : "POOR");
    last_print = millis();
  }
  
  delay(5);
}

// ESP-NOW callback functions (same as working test)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    // Success - reset failure counter
    if (consecutive_failures > 0) {
      consecutive_failures = 0;
    }
  } else {
    consecutive_failures++;
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.printf("📥 Received %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n",
                data_len, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

// Display functions
void initializeDisplay() {
  Serial.println("🖥️ Initializing display...");
  
  display.begin();
  display.clearBuffer();
  display.sendBuffer();
  
  delay(100); // Let display stabilize
  
  display_connected = true;
  Serial.println("✅ Display initialized successfully");
}

void updateDisplay() {
  if (!display_connected) return;
  
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  
  // Line 1: Joystick values
  char joystick_str[32];
  snprintf(joystick_str, sizeof(joystick_str), "X:%4d Y:%4d", 
           controller_data.joystick_x, controller_data.joystick_y);
  display.drawStr(0, 10, joystick_str);
  
  // Line 2: Mode and status (simplified - no turbo mode)
  char mode_str[32];
  const char* mode_names[] = {"NORM", "NORM", "FOLLOW", "PARK"};
  snprintf(mode_str, sizeof(mode_str), "Mode:%s", mode_names[current_mode]);
  display.drawStr(0, 22, mode_str);
  
  // Line 3: Emergency stop, cruise control, or battery status
  char status_str[32];
  if (emergency_stop_enabled) {
    snprintf(status_str, sizeof(status_str), "E-STOP ACTIVE!");
  } else if (cruise_control_enabled) {
    snprintf(status_str, sizeof(status_str), "Cruise:ON %d", current_cruise_speed);
  } else {
    const char* batt_icon = low_battery_detected ? "LOW!" : "OK";
    snprintf(status_str, sizeof(status_str), "Batt:%s TX:%s", 
             batt_icon, consecutive_failures < 10 ? "OK" : "POOR");
  }
  display.drawStr(0, 32, status_str);
  
  display.sendBuffer();
}

void displayStatus(const char* message) {
  if (!display_connected) {
    Serial.printf("Status: %s\n", message);
    return;
  }
  
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB10_tr);
  display.drawStr(0, 16, message);
  display.sendBuffer();
  
  Serial.printf("Status: %s\n", message);
}

// MPU6050 functions
void initializeMPU6050() {
  Serial.println("🔄 Initializing MPU6050...");
  displayStatus("Initializing...");
  
  // Initialize hardware I2C for MPU6050 on separate pins
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(100000); // 100kHz for MPU6050
  delay(100); // Let I2C stabilize
  
  // Test connection
  if (mpu.testConnection()) {
    Serial.println("✅ MPU6050 connection test passed");
    displayStatus("Calibrating...");
    
    // Initialize MPU6050
    mpu.initialize();
    delay(100); // Let initialization complete
    
    // Configure MPU6050
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    delay(100); // Let configuration settle
    
    // Calibration phase - set initial angle while holding controller
    Serial.println("📱 CALIBRATION: Hold controller in desired position for 3 seconds...");
    displayStatus("Hold steady...");
    
    // Collect calibration samples for 3 seconds
    int16_t calib_x = 0, calib_y = 0, calib_z = 0;
    int samples = 0;
    
    for (int i = 0; i < 30; i++) { // 3 seconds at 100ms intervals
      if (mpu.testConnection()) {
        int16_t ax, ay, az;
        // Read accelerometer data
        uint8_t buffer[6];
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // ACCEL_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x68, (size_t)6, (bool)true);
        
        if (Wire.available() == 6) {
          for (int j = 0; j < 6; j++) {
            buffer[j] = Wire.read();
          }
          ax = (buffer[0] << 8) | buffer[1];
          ay = (buffer[2] << 8) | buffer[3];
          az = (buffer[4] << 8) | buffer[5];
          
          calib_x += ax;
          calib_y += ay;
          calib_z += az;
          samples++;
        }
      }
      
      // Show countdown on display
      char countdown[16];
      snprintf(countdown, sizeof(countdown), "Calib: %ds", (30-i)/10);
      display.clearBuffer();
      display.setFont(u8g2_font_ncenB08_tr);
      display.drawStr(0, 16, countdown);
      display.sendBuffer();
      
      delay(100);
    }
    
    // Calculate calibration offsets (center values)
    if (samples > 0) {
      calib_x /= samples;
      calib_y /= samples;
      calib_z /= samples;
      
      Serial.printf("📱 Calibration complete! Center: X=%d Y=%d Z=%d\n", calib_x, calib_y, calib_z);
      displayStatus("Calibrated!");
      delay(1000);
    }
    
    // Mark as connected and initialized
    mpu_connected = true;
    mpu_initialized = true;
    gesture_control_enabled = true; // Auto-enable gesture control after calibration
    Serial.println("✅ MPU6050 initialized and calibrated!");
    displayStatus("MPU6050 Ready!");
    
  } else {
    Serial.println("❌ MPU6050 connection test failed - check wiring on pins SDA=10, SCL=11");
    mpu_connected = false;
    mpu_initialized = false;
    displayStatus("MPU6050 Failed");
  }
}

void readMPU6050() {
  if (!mpu_connected || !mpu_initialized) return;
  
  // Rate limit reading to avoid overwhelming the system
  if (millis() - last_mpu_read < MPU_READ_INTERVAL) return;
  last_mpu_read = millis();
  
  // Test connection before reading
  if (!mpu.testConnection()) {
    Serial.println("⚠️ MPU6050 connection lost");
    mpu_connected = false;
    return;
  }
  
  // Read MPU6050 sensor data
  int16_t ax, ay, az, gx, gy, gz;
  
  // Use direct I2C read to avoid library issues
  uint8_t buffer[14];
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start reading from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (size_t)14, (bool)true);
  
  if (Wire.available() == 14) {
    for (int i = 0; i < 14; i++) {
      buffer[i] = Wire.read();
    }
    
    // Parse the data
    mpu_accel_x = (buffer[0] << 8) | buffer[1];
    mpu_accel_y = (buffer[2] << 8) | buffer[3];
    mpu_accel_z = (buffer[4] << 8) | buffer[5];
    mpu_gyro_x = (buffer[8] << 8) | buffer[9];
    mpu_gyro_y = (buffer[10] << 8) | buffer[11];
    mpu_gyro_z = (buffer[12] << 8) | buffer[13];
    
    // Debug: Print MPU6050 data every 2 seconds
    static unsigned long last_mpu_debug = 0;
    if (millis() - last_mpu_debug > 2000) {
      Serial.printf("📱 MPU6050 - Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n",
                    mpu_accel_x, mpu_accel_y, mpu_accel_z, mpu_gyro_x, mpu_gyro_y, mpu_gyro_z);
      last_mpu_debug = millis();
    }
  }
}

void handleGestureControl() {
  if (!mpu_connected || !gesture_control_enabled) return;
  
  // Simple gesture detection based on accelerometer
  const int GESTURE_THRESHOLD = 8000; // Adjust sensitivity
  
  // Tilt left (negative X acceleration)
  if (mpu_accel_x < -GESTURE_THRESHOLD) {
    static unsigned long last_left_tilt = 0;
    if (millis() - last_left_tilt > 1000) { // Debounce 1 second
      Serial.println("👈 Left tilt detected");
      last_left_tilt = millis();
    }
  }
  
  // Tilt right (positive X acceleration)
  if (mpu_accel_x > GESTURE_THRESHOLD) {
    static unsigned long last_right_tilt = 0;
    if (millis() - last_right_tilt > 1000) { // Debounce 1 second
      Serial.println("👉 Right tilt detected");
      last_right_tilt = millis();
    }
  }
  
  // Tilt forward (negative Y acceleration)
  if (mpu_accel_y < -GESTURE_THRESHOLD) {
    static unsigned long last_forward_tilt = 0;
    if (millis() - last_forward_tilt > 1000) { // Debounce 1 second
      Serial.println("⬆️ Forward tilt detected");
      last_forward_tilt = millis();
    }
  }
  
  // Tilt backward (positive Y acceleration)
  if (mpu_accel_y > GESTURE_THRESHOLD) {
    static unsigned long last_backward_tilt = 0;
    if (millis() - last_backward_tilt > 1000) { // Debounce 1 second
      Serial.println("⬇️ Backward tilt detected");
      last_backward_tilt = millis();
    }
  }
}

// Battery monitoring function
void readBatteryStatus() {
  // Read PowerBoost 1000C LBO pin
  // LBO is LOW when battery is low (below 3.2V)
  // LBO is HIGH when battery is good (above 3.2V)
  bool lbo_state = digitalRead(LOW_BATTERY_PIN);
  
  // LBO is active LOW, so LOW means low battery
  bool new_low_battery = !lbo_state;
  
  // Check for low battery state change
  if (new_low_battery && !low_battery_detected) {
    Serial.println("⚠️ LOW BATTERY WARNING! PowerBoost 1000C LBO triggered");
    displayStatus("BATTERY LOW!");
    delay(2000); // Show warning for 2 seconds
  }
  
  low_battery_detected = new_low_battery;
  
  // Update battery level based on LBO status
  if (low_battery_detected) {
    battery_level = 15; // Set to low battery level
  } else {
    battery_level = 85; // Normal battery level
  }
  
  // Debug: Print battery status every 10 seconds
  static unsigned long last_battery_debug = 0;
  if (millis() - last_battery_debug > 10000) {
    Serial.printf("🔋 Battery Status: Level=%d%%, LBO=%s, Low=%s\n", 
                  battery_level, lbo_state ? "HIGH" : "LOW", 
                  low_battery_detected ? "YES" : "NO");
    last_battery_debug = millis();
  }
}
