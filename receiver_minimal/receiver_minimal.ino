#include <WiFi.h>
#include <esp_now.h>

// Simple data structure - must match controller exactly
typedef struct __attribute__((packed)) {
  int16_t joystick_x;
  int16_t joystick_y;
  uint8_t button_states;
  uint8_t battery_level;
  uint8_t operation_mode;
  uint8_t cruise_control_active; // Changed from bool to uint8_t for stability
  int16_t cruise_speed;         // Speed to maintain when cruise control is active
} controller_data_t;

// Simple USB data structure - minimal fields only
typedef struct __attribute__((packed)) {
  int16_t throttle;
  int16_t steering;
  uint8_t emergency_stop;
  uint8_t cruise_control;    // Cruise control status
  int16_t cruise_speed;      // Speed to maintain
  uint8_t checksum;
} usb_data_t;

// Global variables
controller_data_t received_data;
usb_data_t usb_data;
bool new_data_received = false;
bool performance_mode = false; // Disable heavy operations for instant response
bool debug_mode = true; // Set to false for production (silent operation)



// Broadcast MAC
uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  if (debug_mode) {
    Serial.println("=== Minimal ESP32 Receiver ===");
  }
  
  // Simple WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  
  if (debug_mode) Serial.println("WiFi initialized");
  
  // Simple ESP-NOW setup
  if (esp_now_init() != ESP_OK) {
    if (debug_mode) Serial.println("ESP-NOW init failed");
    return;
  }
  
  if (debug_mode) Serial.println("ESP-NOW initialized");
  
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add broadcast peer
  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, broadcast_mac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    if (debug_mode) Serial.println("Failed to add peer");
  } else {
    if (debug_mode) Serial.println("Broadcast peer added");
  }
  
  // Initialize USB data
  usb_data.throttle = 0;
  usb_data.steering = 0;
  usb_data.emergency_stop = 0;
  usb_data.cruise_control = 0;
  usb_data.cruise_speed = 0;
  
  if (debug_mode) {
    Serial.println("Setup complete - waiting for data...");
    Serial.printf("📏 Controller data structure size: %d bytes\n", sizeof(controller_data_t));
    Serial.printf("📏 USB data structure size: %d bytes\n", sizeof(usb_data_t));
    Serial.println("🚀 Performance mode: Auto-enabled during cruise control for instant response");
    Serial.println("📊 Note: Using 40Hz (25ms) to prevent Serial Monitor corruption");
  }
}

void loop() {
  if (new_data_received) {
    // Simple data processing
    usb_data.throttle = received_data.joystick_y;
    usb_data.steering = received_data.joystick_x;
    usb_data.emergency_stop = (received_data.button_states & (1 << 5)) ? 1 : 0;
    usb_data.cruise_control = received_data.cruise_control_active;
    usb_data.cruise_speed = received_data.cruise_speed;
    
    // Simple checksum
    usb_data.checksum = 0;
    
    // Send to Pi
    Serial.write(0xAA);
    Serial.write((uint8_t*)&usb_data, sizeof(usb_data));
    Serial.write(0xBB);
    
    // Only show status in normal mode
    if (!performance_mode && debug_mode) {
      Serial.printf("Sent: T=%d, S=%d, E=%d, C=%d\n", 
                   usb_data.throttle, usb_data.steering, 
                   usb_data.emergency_stop, usb_data.cruise_control);
    }
    // In performance mode, send data silently to prevent Serial corruption
    
    new_data_received = false;
  }
  
  // Simple timing - use moderate speed to avoid Serial Monitor corruption
  delay(performance_mode ? 25 : 50); // 40Hz instead of 100Hz to prevent corruption
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int data_len) {
  if (data_len == sizeof(controller_data_t)) {
    // Simple, safe data copy
    memcpy(&received_data, data, sizeof(controller_data_t));
    new_data_received = true;
    
    // Simple performance mode toggle
    if (received_data.cruise_control_active == 1 && !performance_mode) {
      performance_mode = true;
      if (debug_mode) Serial.println("🚀 PERFORMANCE MODE ON");
      // No more Serial.flush() - it was causing delays
    } else if (received_data.cruise_control_active == 0 && performance_mode) {
      performance_mode = false;
      if (debug_mode) Serial.println("📊 NORMAL MODE ON");
    }
    
    // Only show data in normal mode
    if (!performance_mode && debug_mode) {
      Serial.printf("📥 Data: X=%d, Y=%d, Cruise=%d\n", 
                   received_data.joystick_x, received_data.joystick_y, 
                   received_data.cruise_control_active);
    }
  } else {
    // Only show errors in normal mode
    if (!performance_mode && debug_mode) {
      Serial.printf("❌ Wrong size: %d vs %d\n", data_len, sizeof(controller_data_t));
    }
  }
}
