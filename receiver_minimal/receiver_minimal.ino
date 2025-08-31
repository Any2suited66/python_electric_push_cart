#include <WiFi.h>
#include <esp_now.h>

// Simple data structure - must match controller exactly
typedef struct __attribute__((packed)) {
  int16_t joystick_x;
  int16_t joystick_y;
  uint8_t button_states;
  uint8_t battery_level;
  uint8_t operation_mode;
} controller_data_t;

// Simple USB data structure - minimal fields only
typedef struct __attribute__((packed)) {
  int16_t throttle;
  int16_t steering;
  uint8_t emergency_stop;
  uint8_t checksum;
} usb_data_t;

// Global variables
controller_data_t received_data;
usb_data_t usb_data;
bool new_data_received = false;

// Broadcast MAC
uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Minimal ESP32 Receiver ===");
  
  // Simple WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  
  Serial.println("WiFi initialized");
  
  // Simple ESP-NOW setup
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  
  Serial.println("ESP-NOW initialized");
  
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add broadcast peer
  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, broadcast_mac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Broadcast peer added");
  }
  
  // Initialize USB data
  usb_data.throttle = 0;
  usb_data.steering = 0;
  usb_data.emergency_stop = 0;
  
  Serial.println("Setup complete - waiting for data...");
}

void loop() {
  if (new_data_received) {
    // Process data
    usb_data.throttle = received_data.joystick_y;
    usb_data.steering = received_data.joystick_x;
    usb_data.emergency_stop = 0;
    
    // Calculate checksum
    usb_data.checksum = 0;
    uint8_t* data_ptr = (uint8_t*)&usb_data;
    for (int i = 0; i < sizeof(usb_data) - 1; i++) {
      usb_data.checksum ^= data_ptr[i];
    }
    
    // Send to Pi - simple format
    Serial.write(0xAA);
    Serial.write((uint8_t*)&usb_data, sizeof(usb_data));
    Serial.write(0xBB);
    Serial.flush();
    
    Serial.printf("Sent to Pi: Throttle=%d, Steering=%d\n", usb_data.throttle, usb_data.steering);
    
    new_data_received = false;
  }
  
  delay(50); // 20Hz
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int data_len) {
  Serial.printf("Received data: %d bytes\n", data_len);
  
  if (data_len == sizeof(controller_data_t)) {
    memcpy(&received_data, data, sizeof(controller_data_t));
    new_data_received = true;
    Serial.printf("Data processed: X=%d, Y=%d\n", received_data.joystick_x, received_data.joystick_y);
  } else {
    Serial.printf("Wrong data size: expected %d, got %d\n", sizeof(controller_data_t), data_len);
  }
}
