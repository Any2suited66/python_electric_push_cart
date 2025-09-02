#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MPU6050.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Pin Definitions
#define JOYSTICK_X_PIN 32  // PS4 joystick X-axis
#define JOYSTICK_Y_PIN 33  // PS4 joystick Y-axis
#define JOYSTICK_BUTTON_PIN 4  // PS4 joystick button press
#define BUTTON_1_PIN 25    // Button 1
#define BUTTON_2_PIN 26    // Button 2
#define BUTTON_3_PIN 27    // Button 3
#define BUTTON_4_PIN 14    // Button 4
#define BUTTON_5_PIN 12    // Button 5

// ESP-NOW Configuration - Broadcast for automatic discovery
#define BROADCAST_MAC_ADDRESS {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // Broadcast to all devices

// Data structure for ESP-NOW communication
typedef struct __attribute__((packed)) {
  int16_t joystick_x;
  int16_t joystick_y;
  uint8_t button_states;
  uint8_t battery_level;
  uint8_t operation_mode;
  bool cruise_control_active;    // Cruise control on/off
  int16_t cruise_speed;         // Speed to maintain when cruise control is active
} controller_data_t;

// Acknowledgment data structure (must match receiver)
typedef struct {
  uint8_t ack_type;      // 0=ack, 1=heartbeat, 2=channel_switch
  uint8_t sequence;      // Sequence number for tracking
  uint8_t rssi;          // Signal strength
  uint8_t connection_quality; // 0-100 quality score
} ack_data_t;

// Global objects
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu;
esp_now_peer_info_t broadcast_peer;

// Global variables
controller_data_t controller_data;
controller_data_t last_sent_data;  // Store last sent data for change detection
ack_data_t received_ack;
uint8_t broadcast_mac[] = BROADCAST_MAC_ADDRESS;
unsigned long last_send_time = 0;
const unsigned long SEND_INTERVAL = 25; // 40Hz update rate for faster response
const unsigned long MIN_SEND_INTERVAL = 10; // Minimum 10ms between sends (100Hz max)
unsigned long last_successful_send = 0;
unsigned long consecutive_failures = 0;
const unsigned long MAX_FAILURES = 10;
bool connection_warning_shown = false;

// FreeRTOS queue for ESP-NOW transmission
QueueHandle_t esp_now_queue;
const int ESP_NOW_QUEUE_SIZE = 8; // Hold 8 packets max

// Queue item structure
typedef struct {
  uint8_t mac[6];
  uint8_t data[sizeof(controller_data_t)];
  size_t data_len;
  bool is_ack; // Whether this is an acknowledgment packet
} queue_item_t;

// Transmission status
volatile bool transmission_in_progress = false;
unsigned long last_transmission_time = 0;
const unsigned long TRANSMISSION_TIMEOUT = 200; // 200ms timeout

// Change threshold for joystick (prevents sending tiny changes)
const int16_t JOYSTICK_CHANGE_THRESHOLD = 30; // Increased to reduce packet frequency

// ESP-NOW Connection Management
unsigned long last_connection_check = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 3000; // Check connection every 3 seconds
unsigned long last_reconnect_attempt = 0;
const unsigned long RECONNECT_COOLDOWN = 10000; // Wait 10 seconds between reconnect attempts
bool esp_now_initialized = false;
bool receiver_connected = false;
unsigned long last_successful_receive = 0;
const unsigned long RECEIVE_TIMEOUT = 5000; // 5 seconds without receive = disconnected

// Connection management
unsigned long last_receiver_response = 0;
const unsigned long RECEIVER_TIMEOUT = 3000; // 3 seconds without response
unsigned long last_ack_time = 0;
const unsigned long ACK_TIMEOUT = 100; // 100ms timeout for acknowledgments
bool waiting_for_ack = false;
uint8_t last_sequence_sent = 0;
uint8_t connection_quality = 100;

// Channel management
uint8_t current_channel = 1;
const uint8_t CHANNELS[] = {1, 6, 11}; // WiFi channels to try
uint8_t current_channel_index = 0;
unsigned long last_channel_switch = 0;
const unsigned long CHANNEL_SWITCH_COOLDOWN = 30000; // 30 seconds between channel switches

// Button states with enhanced debouncing
bool button_states[5] = {false, false, false, false, false};
bool last_button_states[5] = {false, false, false, false, false};
bool joystick_button_state = false;


// Enhanced button debouncing variables
unsigned long button_debounce_times[5] = {0, 0, 0, 0, 0};
unsigned long joystick_button_debounce_time = 0;
const unsigned long BUTTON_DEBOUNCE_DELAY = 50; // 50ms debounce
int button_stable_readings[5] = {0, 0, 0, 0, 0};
int joystick_button_stable_readings = 0;
const int REQUIRED_STABLE_READINGS = 5; // Need 5 stable readings to confirm state

// Button connection status
bool button_connected[5] = {true, true, true, true, true};
bool joystick_button_connected = true;

// Operation modes
enum OperationMode {
  MODE_NORMAL = 0,
  MODE_TURBO = 1,
  MODE_FOLLOW_ME = 2,
  MODE_PARKING = 3
};

OperationMode current_mode = MODE_NORMAL;

// Toggle features
bool emergency_stop_enabled = false;
bool debug_mode_enabled = false;  // Set to false for production (silent operation)
bool auto_reconnect_enabled = true;

// Cruise control variables
bool cruise_control_enabled = false;
int16_t current_cruise_speed = 0;
bool cruise_control_changed = false; // Flag to force immediate transmission
bool performance_mode = false; // Disable heavy operations for instant response

// Gesture control state
bool gesture_control_enabled = false;
bool mpu_initialized = false;
bool mpu_connected = false;
unsigned long mpu_init_start_time = 0;
const unsigned long MPU_INIT_DURATION = 3000; // 3 seconds for calibration

// Battery simulation (replace with actual battery monitoring)
uint8_t controller_battery = 85;
// cart_battery variable removed since cart_battery_level field was removed from struct

// Function declarations
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t sendStatus);
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int data_len);
void displayStatus(const char* message);
void updateDisplay();
bool hasSignificantChange();
void sendData();
void processTransmissionQueue();
void sendAcknowledgment();
void runJoystickHardwareTest();
void debugPrint(const char* message);
void debugPrintf(const char* format, ...);

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  
  // Configure ADC for better joystick readings
  analogReadResolution(12);  // Set ADC resolution to 12 bits (0-4095)
  analogSetAttenuation(ADC_11db);  // Set attenuation for 0-3.3V range
  
  // Add some delay for ADC stabilization
  delay(100);
  
  // Test joystick calibration
  debugPrint("Testing joystick calibration...\n");
  for (int i = 0; i < 10; i++) {
    int x_test = analogRead(JOYSTICK_X_PIN);
    int y_test = analogRead(JOYSTICK_Y_PIN);
    debugPrintf("Calibration test %d: X=%d, Y=%d\n", i+1, x_test, y_test);
    delay(100);
  }
  
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  pinMode(BUTTON_5_PIN, INPUT_PULLUP);
  
  // Check button connections
  checkButtonConnections();
  
  // Initialize joystick button state after connection check
  joystick_button_state = (digitalRead(JOYSTICK_BUTTON_PIN) == LOW);
  Serial.printf("🔍 Initial joystick button state: %s\n", joystick_button_state ? "PRESSED" : "released");
  
  // Initialize I2C for OLED and MPU6050
  Wire.begin();
  
  // Initialize OLED display
  debugPrint("Initializing OLED display...\n");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    debugPrint("SSD1306 allocation failed\n");
    for(;;);
  }
  debugPrint("OLED display initialized successfully\n");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  debugPrint("Display cleared and configured\n");
  
  // Initialize MPU6050 and check connection
  mpu.initialize();
  if (!mpu.testConnection()) {
    debugPrint("MPU6050 connection failed - gesture control disabled\n");
    mpu_connected = false;
  } else {
    debugPrint("MPU6050 hardware connection OK\n");
    mpu_connected = true;
  }
  
  // Initialize ESP-NOW
  WiFi.mode(WIFI_AP_STA);  // Match receiver mode for better compatibility
  
  // Disable WiFi power management for better reliability
  WiFi.setSleep(false);
  
  // Set maximum TX power for better range and reliability
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Initialize FreeRTOS queue for ESP-NOW transmission
  esp_now_queue = xQueueCreate(ESP_NOW_QUEUE_SIZE, sizeof(queue_item_t));
  if (esp_now_queue == NULL) {
    Serial.println("Error creating ESP-NOW queue");
    return;
  }
  Serial.printf("ESP-NOW queue created with size %d\n", ESP_NOW_QUEUE_SIZE);
  
  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add broadcast peer
  memcpy(broadcast_peer.peer_addr, broadcast_mac, 6);
  broadcast_peer.channel = current_channel;
  broadcast_peer.encrypt = false;
  
  // Remove any existing peer first
  esp_now_del_peer(broadcast_mac);
  
  if (esp_now_add_peer(&broadcast_peer) != ESP_OK) {
    Serial.println("Warning: Failed to add broadcast peer");
    Serial.println("Will try to send data anyway...");
  } else {
    Serial.println("Broadcast peer added successfully");
  }
  
  Serial.println("ESP-NOW initialized successfully");
  Serial.printf("Broadcasting to all devices on channel %d\n", current_channel);
  
  Serial.println("Controller initialized successfully");
  Serial.printf("📏 Controller data structure size: %d bytes\n", sizeof(controller_data_t));
  displayStatus("Controller Ready");
  
  // Force display test
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("DISPLAY TEST");
  display.setCursor(0, 20);
  display.println("If you see this,");
  display.setCursor(0, 30);
  display.println("display is working!");
  display.display();
  delay(3000);
  
  esp_now_initialized = true;
  last_successful_receive = millis();
}

void checkConnectionHealth() {
  unsigned long current_time = millis();
  
  if (current_time - last_connection_check < CONNECTION_CHECK_INTERVAL) {
    return;
  }
  
  last_connection_check = current_time;
  
  // In broadcast mode, assume we're connected if transmissions are successful
  if (consecutive_failures < MAX_FAILURES) {
    receiver_connected = true;
    connection_warning_shown = false;
  } else {
    if (!connection_warning_shown) {
      Serial.println("Warning: High transmission failure rate");
      connection_warning_shown = true;
    }
    receiver_connected = false;
  }
  
  // Try channel switching if connection is poor and auto-reconnect is enabled
  if (auto_reconnect_enabled && consecutive_failures >= MAX_FAILURES && 
      current_time - last_channel_switch > CHANNEL_SWITCH_COOLDOWN) {
    switchChannel();
  }
}

void switchChannel() {
  current_channel_index = (current_channel_index + 1) % (sizeof(CHANNELS) / sizeof(CHANNELS[0]));
  current_channel = CHANNELS[current_channel_index];
  
  Serial.printf("Switching to channel %d\n", current_channel);
  
  // Reinitialize ESP-NOW with new channel
  reconnectESPNow();
  
  last_channel_switch = millis();
}

void reconnectESPNow() {
  Serial.println("Reconnecting ESP-NOW...");
  
  // Remove broadcast peer
  esp_now_del_peer(broadcast_mac);
  
  // Reinitialize ESP-NOW
  esp_now_deinit();
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error reinitializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add broadcast peer with new channel
  memcpy(broadcast_peer.peer_addr, broadcast_mac, 6);
  broadcast_peer.channel = current_channel;
  broadcast_peer.encrypt = false;
  
  if (esp_now_add_peer(&broadcast_peer) != ESP_OK) {
    Serial.println("Warning: Failed to add broadcast peer after reconnect");
    Serial.println("Will try to send data anyway...");
  }
  
  // Reset connection state
  waiting_for_ack = false;
  displayStatus("Reconnected");
}

void readJoystick() {
  // Read joystick values (0-4095 for ESP32 ADC)
  int x_raw = analogRead(JOYSTICK_X_PIN);
  int y_raw = analogRead(JOYSTICK_Y_PIN);
  
  // DIAGNOSTIC: Check for hardware issues
  static unsigned long last_diagnostic_time = 0;
  static int x_min = 4095, x_max = 0, y_min = 4095, y_max = 0;
  static int x_center = 2048, y_center = 2048;
  static int consecutive_x_errors = 0, consecutive_y_errors = 0;
  
  // Track min/max values to detect range issues
  if (x_raw < x_min) x_min = x_raw;
  if (x_raw > x_max) x_max = x_raw;
  if (y_raw < y_min) y_min = y_raw;
  if (y_raw > y_max) y_max = y_raw;
  
  // Check for suspicious readings (stuck values, out of range)
  bool x_suspicious = (x_raw == 0 || x_raw == 4095 || x_raw < 100 || x_raw > 3995);
  bool y_suspicious = (y_raw == 0 || y_raw == 4095 || y_raw < 100 || y_raw > 3995);
  
  if (x_suspicious) consecutive_x_errors++;
  else consecutive_x_errors = 0;
  
  if (y_suspicious) consecutive_y_errors++;
  else consecutive_y_errors = 0;
  
  // Print diagnostic info every 5 seconds
  if (millis() - last_diagnostic_time > 5000) {
    Serial.printf("🎮 Joystick Diagnostic - X: raw=%d mapped=%d range=[%d,%d] errors=%d | Y: raw=%d mapped=%d range=[%d,%d] errors=%d\n",
                  x_raw, controller_data.joystick_x, x_min, x_max, consecutive_x_errors,
                  y_raw, controller_data.joystick_y, y_min, y_max, consecutive_y_errors);
    
    // Check for hardware issues
    if (x_max - x_min < 1000) Serial.println("⚠️ X-axis range too small - possible hardware issue");
    if (y_max - y_min < 1000) Serial.println("⚠️ Y-axis range too small - possible hardware issue");
    if (consecutive_x_errors > 5) Serial.println("⚠️ X-axis showing consecutive errors - possible loose connection");
    if (consecutive_y_errors > 5) Serial.println("⚠️ Y-axis showing consecutive errors - possible loose connection");
    
    // SPECIFIC LEFT TURN DEBUGGING
    if (controller_data.joystick_x < -100) {
      Serial.printf("🔍 LEFT TURN DEBUG - Raw X: %d, Mapped X: %d, Range: [%d,%d], Errors: %d\n", 
                    x_raw, controller_data.joystick_x, x_min, x_max, consecutive_x_errors);
    }
    
    last_diagnostic_time = millis();
  }
  
  // Convert to -512 to 512 range for better resolution
  controller_data.joystick_x = map(x_raw, 0, 4095, -512, 512);
  controller_data.joystick_y = map(y_raw, 0, 4095, -512, 512);
  
  // Apply deadzone
  if (abs(controller_data.joystick_x) < 50) controller_data.joystick_x = 0;
  if (abs(controller_data.joystick_y) < 50) controller_data.joystick_y = 0;
}

void checkButtonConnections() {
  // Check if buttons are connected by reading them multiple times
  // If a button always reads the same value, it's likely disconnected
  const int samples = 10;
  
  // Check regular buttons
  for (int i = 0; i < 5; i++) {
    int pin = (i == 0) ? BUTTON_1_PIN : (i == 1) ? BUTTON_2_PIN : 
              (i == 2) ? BUTTON_3_PIN : (i == 3) ? BUTTON_4_PIN : BUTTON_5_PIN;
    
    int first_read = digitalRead(pin);
    bool all_same = true;
    
    for (int j = 1; j < samples; j++) {
      if (digitalRead(pin) != first_read) {
        all_same = false;
        break;
      }
      delay(1);
    }
    
    button_connected[i] = !all_same;
    if (!button_connected[i]) {
      Serial.printf("Button %d appears to be disconnected\n", i + 1);
    }
  }
  
  // Check joystick button - Test if it's actually connected
  Serial.println("🔍 Testing joystick button connection...");
  int joystick_first_read = digitalRead(JOYSTICK_BUTTON_PIN);
  Serial.printf("🔍 Initial joystick button reading: %d\n", joystick_first_read);
  
  bool joystick_all_same = true;
  
  for (int j = 1; j < samples; j++) {
    int current_read = digitalRead(JOYSTICK_BUTTON_PIN);
    Serial.printf("🔍 Sample %d: %d\n", j, current_read);
    if (current_read != joystick_first_read) {
      joystick_all_same = false;
      Serial.printf("🔍 Variation detected at sample %d\n", j);
      break;
    }
    delay(1);
  }
  
  joystick_button_connected = !joystick_all_same;
  Serial.printf("🔍 Joystick button connection test: %s (raw reading: %d, all_same: %s)\n", 
               joystick_button_connected ? "CONNECTED" : "DISCONNECTED", 
               joystick_first_read, 
               joystick_all_same ? "YES" : "NO");
  
  // Force connection for testing if it's detected as disconnected
  if (!joystick_button_connected) {
    Serial.println("🔍 WARNING: Joystick button detected as disconnected, but forcing connection for testing");
    joystick_button_connected = true;
  }
  
  // Initialize joystick button state to match current reading
  joystick_button_state = (digitalRead(JOYSTICK_BUTTON_PIN) == LOW);
  Serial.printf("🔍 Initial joystick button state: %s\n", joystick_button_state ? "PRESSED" : "released");
}

void readButtons() {
  unsigned long current_time = millis();
  
  // Debug: Show that readButtons is being called
  static unsigned long last_read_test = 0;
  if (current_time - last_read_test > 5000) {
    Serial.println("🔍 readButtons() function executing");
    last_read_test = current_time;
  }
  
  // Read regular buttons with enhanced debouncing (only if connected)
  for (int i = 0; i < 5; i++) {
    if (button_connected[i]) {
      int pin = (i == 0) ? BUTTON_1_PIN : (i == 1) ? BUTTON_2_PIN : 
                (i == 2) ? BUTTON_3_PIN : (i == 3) ? BUTTON_4_PIN : BUTTON_5_PIN;
      
      bool raw_button_pressed = !digitalRead(pin); // Button pressed when LOW (due to INPUT_PULLUP)
      
      // Track state changes for debouncing
      if (raw_button_pressed != last_button_states[i]) {
        button_debounce_times[i] = current_time;
        button_stable_readings[i] = 0;
      } else {
        // Same reading - increment stable counter
        button_stable_readings[i]++;
      }
      
      // Enhanced debouncing with stability check
      if ((current_time - button_debounce_times[i]) > BUTTON_DEBOUNCE_DELAY && 
          button_stable_readings[i] >= REQUIRED_STABLE_READINGS) {
        // If the button state has changed and readings are stable
        if (raw_button_pressed != button_states[i]) {
          button_states[i] = raw_button_pressed;
          
          // Debug: Show button state change
          if (button_states[i]) {
            Serial.printf("Button %d PRESSED (stable readings: %d)\n", i + 1, button_stable_readings[i]);
          } else {
            Serial.printf("Button %d released (stable readings: %d)\n", i + 1, button_stable_readings[i]);
          }
        }
      }
      
      // Save the reading for next comparison
      last_button_states[i] = raw_button_pressed;
    }
  }
  
  // Simple joystick button reading - no complex debouncing needed
  if (joystick_button_connected) {
    // Just read the button state for the button_states byte
    joystick_button_state = (digitalRead(JOYSTICK_BUTTON_PIN) == LOW);
  }
  
  // Pack button states into a byte (including joystick button as bit 5)
  controller_data.button_states = 0;
  for (int i = 0; i < 5; i++) {
    if (button_states[i]) {
      controller_data.button_states |= (1 << i);
    }
  }
  if (joystick_button_state) {
    controller_data.button_states |= (1 << 5); // Bit 5 for joystick button
  }
  
  // Debug: Show button_states value when it changes
  static uint8_t last_button_states_sent = 0;
  if (controller_data.button_states != last_button_states_sent) {
    debugPrintf("🔘 Button states sent: 0x%02X (joystick_bit5=%d, cruise_control=%s, cruise_speed=%d, struct_size=%d)\n", 
                 controller_data.button_states, 
                 (controller_data.button_states & (1 << 5)) ? 1 : 0,
                 controller_data.cruise_speed,
                 sizeof(controller_data_t));
    last_button_states_sent = controller_data.button_states;
  }
}

void readMPU6050() {
  // MPU data removed from struct to eliminate alignment issues
  // This function is kept for compatibility but does nothing
}

void handleButtonPresses() {
  // Check for button press events (rising edge detection)
  for (int i = 0; i < 5; i++) {
    if (button_states[i] && !last_button_states[i]) {
      // Button pressed
      Serial.printf("Button %d pressed\n", i + 1);
      
      // Handle button actions
      switch (i) {
        case 0: // Button 1 - Toggle operation mode
          current_mode = (OperationMode)((current_mode + 1) % 4);
          Serial.printf("Operation mode changed to %d\n", current_mode);
          break;
        case 1: // Button 2 - Toggle gesture control
          if (mpu_connected) {
            gesture_control_enabled = !gesture_control_enabled;
            if (gesture_control_enabled) {
              mpu_init_start_time = millis();
              mpu_initialized = false;
              displayStatus("Gesture Control Starting...");
            } else {
              mpu_initialized = false;
              displayStatus("Gesture Control Disabled");
            }
            Serial.printf("Gesture control %s\n", gesture_control_enabled ? "enabled" : "disabled");
          }
          break;
        case 2: // Button 3 - Toggle emergency stop mode
          emergency_stop_enabled = !emergency_stop_enabled;
          Serial.printf("Emergency stop mode %s\n", emergency_stop_enabled ? "ENABLED" : "disabled");
          if (emergency_stop_enabled) {
            displayStatus("Emergency Stop ON");
          } else {
            displayStatus("Emergency Stop OFF");
          }
          break;
        case 3: // Button 4 - Toggle turbo mode
          if (current_mode == MODE_NORMAL) {
            current_mode = MODE_TURBO;
          } else if (current_mode == MODE_TURBO) {
            current_mode = MODE_NORMAL;
          }
          Serial.printf("Turbo mode %s\n", (current_mode == MODE_TURBO) ? "enabled" : "disabled");
          break;
        case 4: // Button 5 - Toggle auto-reconnect
          auto_reconnect_enabled = !auto_reconnect_enabled;
          debugPrintf("Auto-reconnect %s\n", auto_reconnect_enabled ? "enabled" : "disabled");
          if (!auto_reconnect_enabled) {
            displayStatus("Auto-reconnect OFF");
          } else {
            displayStatus("Auto-reconnect ON");
          }
          break;
      }
    }
    last_button_states[i] = button_states[i];
  }
  
  // DEBUG MODE TOGGLE - Button 1 + Button 2 held together for 2 seconds
  static unsigned long debug_toggle_start = 0;
  static bool debug_toggle_triggered = false;
  
  if (button_states[0] && button_states[1] && !debug_toggle_triggered) {
    if (debug_toggle_start == 0) {
      debug_toggle_start = millis();
    } else if (millis() - debug_toggle_start > 2000) { // 2 seconds
      debug_mode_enabled = !debug_mode_enabled;
      debug_toggle_triggered = true;
      displayStatus(debug_mode_enabled ? "Debug ON" : "Debug OFF");
      delay(1000); // Show status for 1 second
    }
  } else if (!button_states[0] || !button_states[1]) {
    debug_toggle_start = 0;
    debug_toggle_triggered = false;
  }
  
  // FAST cruise control toggle - immediate response for safety!
  static bool button_was_pressed = false;
  bool button_is_pressed = (digitalRead(JOYSTICK_BUTTON_PIN) == LOW);
  
  // Detect button press (transition from not pressed to pressed)
  if (button_is_pressed && !button_was_pressed) {
    // IMMEDIATE cruise control toggle - no delays!
    if (!cruise_control_enabled) {
      // Activate cruise control
      cruise_control_enabled = true;
      current_cruise_speed = controller_data.joystick_y;
      cruise_control_changed = true; // Force immediate transmission
      performance_mode = true; // Enable performance mode for instant response
      debugPrintf("🚗 Cruise control ON at speed: %d\n", current_cruise_speed);
      // Update display asynchronously to avoid delay
      displayStatus("Cruise Control ON");
    } else {
      // Deactivate cruise control IMMEDIATELY
      cruise_control_enabled = false;
      current_cruise_speed = 0;
      cruise_control_changed = true; // Force immediate transmission
      performance_mode = false; // Disable performance mode
      debugPrintf("🚗 Cruise control OFF\n");
      displayStatus("Cruise Control OFF");
    }
  }
  
  // CRUISE CONTROL SPEED ADJUSTMENT - Fine-tune while walking
  if (cruise_control_enabled) {
    // Check for maximum forward/backward joystick to adjust cruise speed
    static bool speed_adjustment_made = false;
    static unsigned long last_speed_adjustment = 0;
    const unsigned long SPEED_ADJUSTMENT_COOLDOWN = 500; // 500ms between adjustments
    
    unsigned long current_time = millis();
    
    // Only allow speed adjustment if enough time has passed
    if (current_time - last_speed_adjustment >= SPEED_ADJUSTMENT_COOLDOWN) {
      // Maximum forward (Y > 400) - Speed up cruise control
      if (controller_data.joystick_y > 400 && !speed_adjustment_made) {
        current_cruise_speed += 25; // Increase speed by 25 units
        if (current_cruise_speed > 512) current_cruise_speed = 512; // Cap at maximum
        cruise_control_changed = true; // Force immediate transmission
        speed_adjustment_made = true;
        last_speed_adjustment = current_time;
        debugPrintf("🚗 Cruise speed INCREASED to: %d\n", current_cruise_speed);
        displayStatus("Speed +25");
      }
      // Maximum backward (Y < -400) - Slow down cruise control
      else if (controller_data.joystick_y < -400 && !speed_adjustment_made) {
        current_cruise_speed -= 25; // Decrease speed by 25 units
        if (current_cruise_speed < -512) current_cruise_speed = -512; // Cap at minimum
        cruise_control_changed = true; // Force immediate transmission
        speed_adjustment_made = true;
        last_speed_adjustment = current_time;
        debugPrintf("🚗 Cruise speed DECREASED to: %d\n", current_cruise_speed);
        displayStatus("Speed -25");
      }
    }
    
    // Reset speed adjustment flag when joystick returns to center
    if (abs(controller_data.joystick_y) < 200) {
      speed_adjustment_made = false;
    }
  }
  
  button_was_pressed = button_is_pressed;
}



void sendAcknowledgment() {
  // Prepare acknowledgment queue item
  queue_item_t item;
  memcpy(item.mac, broadcast_mac, 6);
  item.is_ack = true;
    
  ack_data_t ack_data;
  ack_data.ack_type = 0; // Regular acknowledgment
  ack_data.sequence = last_sequence_sent;
  ack_data.rssi = WiFi.RSSI();
  ack_data.connection_quality = connection_quality;
  
  memcpy(item.data, &ack_data, sizeof(ack_data));
  item.data_len = sizeof(ack_data);
  
  // Add to queue (non-blocking)
  if (xQueueSend(esp_now_queue, &item, 0) != pdTRUE) {
    // Queue is full - ack can be dropped safely
    Serial.println("⚠️ Queue full - dropped acknowledgment");
  }
}

void loop() {
  unsigned long current_time = millis();
  
  // Check connection health and reconnect if needed (skip in performance mode)
  if (!performance_mode) {
    checkConnectionHealth();
  }
  
  // Handle MPU initialization timing
  if (gesture_control_enabled && !mpu_initialized) {
    if (current_time - mpu_init_start_time >= MPU_INIT_DURATION) {
      mpu_initialized = true;
      displayStatus("Gesture Control Ready!");
      Serial.println("MPU6050 initialization complete");
      delay(1000); // Show completion message for 1 second
    }
  }
  
  // Read inputs
  readJoystick();
  
  // Debug: Test if readButtons is being called
  static unsigned long last_button_test = 0;
  if (current_time - last_button_test > 3000) {
    debugPrint("🔍 readButtons() function called\n");
    last_button_test = current_time;
  }
  
  readButtons();
  // Skip MPU readings in performance mode for instant response
  if (!performance_mode) {
    readMPU6050();
  }
  handleButtonPresses();
  
  // Update display (skip in performance mode for instant response)
  if (!performance_mode) {
    updateDisplay();
  }
  
  // Send data via ESP-NOW with change threshold
  bool should_send = false;
  
  // Always send at regular intervals (40Hz) to maintain connection
  if (current_time - last_send_time >= SEND_INTERVAL) {
    should_send = true;
  }
  // Also send immediately if there's a significant change (but respect minimum interval)
  else if (current_time - last_send_time >= MIN_SEND_INTERVAL && hasSignificantChange()) {
    should_send = true;
  }
  // Force immediate send if cruise control state changed (safety override)
  else if (current_time - last_send_time >= MIN_SEND_INTERVAL && cruise_control_changed) {
    should_send = true;
    cruise_control_changed = false; // Reset flag
    if (!performance_mode) {
      debugPrintf("🚗 Safety override: forcing immediate cruise control transmission\n");
    }
  }
  
  if (should_send) {
    // Debug: Show why we're sending and queue status (skip in performance mode)
    if (!performance_mode) {
      static unsigned long last_debug_time = 0;
      if (millis() - last_debug_time > 2000) { // Every 2 seconds
        UBaseType_t queue_items = uxQueueMessagesWaiting(esp_now_queue);
        if (current_time - last_send_time >= SEND_INTERVAL) {
          debugPrintf("📡 Sending: Regular interval (40Hz) | Queue: %d/%d | Connected: %s\n", 
                       queue_items, ESP_NOW_QUEUE_SIZE, receiver_connected ? "YES" : "NO");
        } else if (hasSignificantChange()) {
          debugPrintf("📡 Sending: Significant change detected | Queue: %d/%d | Connected: %s\n", 
                       queue_items, ESP_NOW_QUEUE_SIZE, receiver_connected ? "YES" : "NO");
        }
        last_debug_time = millis();
      }
    }
    
    sendData();
    last_send_time = current_time;
    
    // Process transmission queue
    processTransmissionQueue();
    
    // Small delay to prevent overwhelming the system
    delay(2);
  }
  
  // Reduced delay for faster response to cruise control changes
  if (performance_mode) {
    delay(1); // Ultra-fast response when cruise control is active
  } else {
    delay(5); // Normal delay when not in performance mode
  }
  
  // SPECIAL TEST: If joystick is held left for 3 seconds, run hardware test
  static unsigned long left_hold_start = 0;
  static bool left_hold_test_done = false;
  
  if (controller_data.joystick_x < -200) { // Strong left turn
    if (left_hold_start == 0) {
      left_hold_start = millis();
    } else if (millis() - left_hold_start > 3000 && !left_hold_test_done) {
      Serial.println("🔧 Running left turn hardware test...");
      runJoystickHardwareTest();
      left_hold_test_done = true;
    }
    
    // Debug TX queue during left turns (skip in performance mode)
    if (!performance_mode) {
      static unsigned long last_left_debug = 0;
      if (millis() - last_left_debug > 1000) {
        UBaseType_t queue_items = uxQueueMessagesWaiting(esp_now_queue);
        debugPrintf("🔄 LEFT TURN TX DEBUG - Queue: %d/%d, Transmission: %s\n", 
                     queue_items, ESP_NOW_QUEUE_SIZE, transmission_in_progress ? "IN PROGRESS" : "READY");
        last_left_debug = millis();
      }
    }
  } else {
    left_hold_start = 0;
    left_hold_test_done = false;
  }
}

bool hasSignificantChange() {
  // Check if joystick values have changed significantly
  int16_t x_change = abs(controller_data.joystick_x - last_sent_data.joystick_x);
  int16_t y_change = abs(controller_data.joystick_y - last_sent_data.joystick_y);
  
  // Check if button states have changed
  bool buttons_changed = (controller_data.button_states != last_sent_data.button_states);
  
  // Check if MPU values have changed significantly (if gesture control is enabled)
      bool mpu_changed = false;
    // MPU data removed from struct to eliminate alignment issues
  
  // Debug: Show change detection (every 5 seconds, skip in performance mode)
  if (!performance_mode) {
    static unsigned long last_change_debug = 0;
    if (millis() - last_change_debug > 5000) {
      Serial.printf("🔍 Change Detection - X: %d->%d (Δ%d) Y: %d->%d (Δ%d) Threshold: %d\n",
                    last_sent_data.joystick_x, controller_data.joystick_x, x_change,
                    last_sent_data.joystick_y, controller_data.joystick_y, y_change,
                    JOYSTICK_CHANGE_THRESHOLD);
      last_change_debug = millis();
    }
  }
  
  // Return true if any significant change detected
  return (x_change >= JOYSTICK_CHANGE_THRESHOLD || 
          y_change >= JOYSTICK_CHANGE_THRESHOLD || 
          buttons_changed || 
          mpu_changed);
}

void processTransmissionQueue() {
  // Don't process if transmission is in progress
  if (transmission_in_progress) {
    // Check for timeout
    if (millis() - last_transmission_time > TRANSMISSION_TIMEOUT) {
      Serial.printf("⏰ Transmission timeout after %dms - resetting\n", 
                   millis() - last_transmission_time);
      transmission_in_progress = false;
    } else {
      return; // Still waiting for transmission
    }
  }
  
  // Try to get next item from queue
  queue_item_t item;
  if (xQueueReceive(esp_now_queue, &item, 0) == pdTRUE) {
    // Start transmission
    transmission_in_progress = true;
    last_transmission_time = millis();
    
    // Send via ESP-NOW
    esp_err_t result = esp_now_send(item.mac, item.data, item.data_len);
    
    if (result == ESP_OK) {
      // Store the data we just sent for change detection (if it's controller data)
      if (!item.is_ack) {
        memcpy(&last_sent_data, item.data, sizeof(controller_data_t));
      }
      
      // Handle acknowledgments
      if (!item.is_ack) {
        waiting_for_ack = true;
        last_ack_time = millis();
        last_sequence_sent++;
      }
      
      // Debug: Show transmission started
      static unsigned long tx_count = 0;
      tx_count++;
      if (tx_count % 20 == 0) {
        debugPrintf("📤 Transmission started #%d\n", tx_count);
      }
    } else {
      // Transmission failed immediately
      transmission_in_progress = false;
      consecutive_failures++;
      Serial.printf("❌ Transmission failed: %d\n", result);
    }
  }
}

void runJoystickHardwareTest() {
  Serial.println("🎮 Running joystick hardware test...");
  
  // Test X-axis response
  Serial.println("Testing X-axis (left/right)...");
  for (int i = 0; i < 20; i++) {
    int x_raw = analogRead(JOYSTICK_X_PIN);
    int y_raw = analogRead(JOYSTICK_Y_PIN);
    Serial.printf("Sample %d: X=%d, Y=%d\n", i+1, x_raw, y_raw);
    delay(100);
  }
  
  // Test specific left turn readings
  Serial.println("Testing left turn readings...");
  for (int i = 0; i < 10; i++) {
    int x_raw = analogRead(JOYSTICK_X_PIN);
    int mapped_x = map(x_raw, 0, 4095, -512, 512);
    Serial.printf("Left test %d: Raw=%d, Mapped=%d\n", i+1, x_raw, mapped_x);
    delay(200);
  }
  
  Serial.println("Hardware test complete!");
}

void sendData() {
  // Update battery levels (simulate battery drain)
  controller_data.battery_level = controller_battery;
  
  // Update cruise control data
  controller_data.cruise_control_active = cruise_control_enabled;
  controller_data.cruise_speed = current_cruise_speed;
  
  // Cruise control state change is now handled in main loop for immediate response
  
  // Debug: Show cruise control data being sent
  static unsigned long last_cruise_debug = 0;
  if (millis() - last_cruise_debug > 2000) {
    debugPrintf("🚗 Cruise control data: enabled=%s, speed=%d, struct_size=%d\n", 
                 cruise_control_enabled ? "YES" : "NO", 
                 current_cruise_speed, 
                 sizeof(controller_data_t));
    last_cruise_debug = millis();
  }
  
  // Prepare queue item
  queue_item_t item;
  
  // Send via broadcast
  memcpy(item.mac, broadcast_mac, 6);
  item.is_ack = false;
  
  memcpy(item.data, &controller_data, sizeof(controller_data));
  item.data_len = sizeof(controller_data);
  
  // Try to add to queue (non-blocking)
  if (xQueueSend(esp_now_queue, &item, 0) == pdTRUE) {
    // Successfully queued
    static unsigned long send_count = 0;
    send_count++;
    if (send_count % 50 == 0) {
      debugPrintf("📦 Queued data - Mode:BROADCAST X:%d Y:%d Quality:%d%% Size:%d bytes\n", 
                   controller_data.joystick_x, controller_data.joystick_y, connection_quality, sizeof(controller_data_t));
    }
  } else {
    // Queue is full - this prevents packet corruption!
    static unsigned long queue_full_count = 0;
    queue_full_count++;
    if (queue_full_count % 10 == 0) {
      Serial.printf("⚠️ Queue full - dropped packet (count: %d)\n", queue_full_count);
    }
  }
}

void updateDisplay() {
  // Debug: Print display update every 5 seconds
  static unsigned long last_display_debug = 0;
  if (millis() - last_display_debug > 5000) {
    Serial.println("Updating display...");
    last_display_debug = millis();
  }
  
  display.clearDisplay();
  
  // Display title and connection status
  display.setCursor(0, 0);
  display.println("Golf Cart Controller");
  
  // Show connection status
  display.setCursor(0, 9);
  display.print("ESP-NOW: ");
  if (consecutive_failures >= MAX_FAILURES) {
    display.print("FAILED");
  } else if (consecutive_failures > 0) {
    display.print("WEAK");
  } else {
    display.print("OK");
  }
  
  // Show connection status
  display.setCursor(0, 18);
  display.print("Status: ");
  if (receiver_connected) {
    display.print("BROADCAST");
  } else {
    display.print("DISCONNECTED");
  }
  
  display.drawLine(0, 30, 128, 30, SSD1306_WHITE);
  
  // Display battery levels
  display.setCursor(0, 35);
  display.print("Ctrl:");
  display.print(controller_data.battery_level);
  display.print("% Cart:");
  display.print(85); // Default value since cart_battery_level was removed
  display.print("%");
  
  // Display operation mode
  display.setCursor(0, 45);
  display.print("Op Mode: ");
  switch (current_mode) {
    case MODE_NORMAL:
      display.print("Normal");
      break;
    case MODE_TURBO:
      display.print("Turbo");
      break;
    case MODE_FOLLOW_ME:
      display.print("Follow");
      break;
    case MODE_PARKING:
      display.print("Parking");
      break;
  }
  
  // Show toggle states
  display.setCursor(0, 55);
  display.print("ES:");
  display.print(emergency_stop_enabled ? "ON" : "OFF");
  display.print(" CC:");
  display.print(cruise_control_enabled ? "ON" : "OFF");
  display.print(" AR:");
  display.print(auto_reconnect_enabled ? "ON" : "OFF");
  
  // Show cruise speed when active
  if (cruise_control_enabled) {
    display.setCursor(0, 45);
    display.print("Cruise Speed: ");
    display.print(current_cruise_speed);
  }
  
  // Show debug mode status
  display.setCursor(0, 35);
  display.print("Debug: ");
  display.print(debug_mode_enabled ? "ON" : "OFF");
  
  display.display();
}

void displayStatus(const char* message) {
  display.clearDisplay();
  display.setCursor(0, 20);
  display.println(message);
  display.display();
  
  // Only delay for non-critical messages to avoid blocking cruise control
  if (strstr(message, "Cruise Control") == NULL) {
    delay(2000);
  }
  // For cruise control messages, show immediately without delay
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t sendStatus) {
  // Mark transmission as complete
  transmission_in_progress = false;
  
  if (sendStatus == ESP_NOW_SEND_SUCCESS) {
    // Data sent successfully
    last_successful_send = millis();
    if (consecutive_failures > 0) {
      consecutive_failures = 0;
      connection_warning_shown = false;
    }
    
    // Debug: Show successful transmission
    static unsigned long success_count = 0;
    success_count++;
    if (success_count % 20 == 0) { // Every 20 successful sends
      UBaseType_t queue_items = uxQueueMessagesWaiting(esp_now_queue);
      debugPrintf("✅ TX Success #%d - Queue: %d/%d\n", 
                   success_count, queue_items, ESP_NOW_QUEUE_SIZE);
    }
  } else {
    // Send failed
    consecutive_failures++;
    Serial.printf("❌ TX failed: %d\n", sendStatus);
  }
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int data_len) {
  // Update connection status
  last_receiver_response = millis();
  receiver_connected = true;
  
  // Check if this is acknowledgment data
  if (data_len == sizeof(ack_data_t)) {
    ack_data_t *ack_data = (ack_data_t*)data;
    
    if (ack_data->ack_type == 0) { // Regular acknowledgment
      if (waiting_for_ack) {
        waiting_for_ack = false;
        connection_quality = ack_data->connection_quality;
        
        // Connection quality updated
        if (connection_quality > 80) {
          // Good connection quality
        }
      }
    } else if (ack_data->ack_type == 1) { // Heartbeat
      connection_quality = ack_data->connection_quality;
      
      // Update connection status based on quality
      if (connection_quality < 50) {
        // Poor connection quality
      } else if (connection_quality > 80) {
        // Good connection quality
      }
    }
  }
  
  // Send acknowledgment
  sendAcknowledgment();
}

// Debug print functions - only output if debug mode is enabled
void debugPrint(const char* message) {
  if (debug_mode_enabled) {
    Serial.print(message);
  }
}

void debugPrintf(const char* format, ...) {
  if (debug_mode_enabled) {
    Serial.printf(format);
  }
}

