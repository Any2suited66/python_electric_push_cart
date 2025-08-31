# 🏌️ Electric Golf Cart Controller

A wireless remote-controlled 3-wheel electric golf cart using hoverboard motors, Raspberry Pi Zero 2W, and ESP32 microcontrollers.

## 📋 Overview

This project transforms a standard golf push cart into a motorized, remote-controlled vehicle using:
- **Hoverboard motors** for propulsion and steering
- **Raspberry Pi Zero 2W** as the main controller
- **ESP32 microcontrollers** for wireless communication
- **Differential steering** for smooth turning

## 🏗️ Hardware Components

### Core System
- **Raspberry Pi Zero 2W** - Main controller running Python
- **2x Hoverboard Motors** - Brushless DC motors with built-in controllers
- **3-Wheel Golf Cart Frame** - Standard golf cart modified for motorization

### Wireless Control System
- **ESP32 Controller** - Handles joystick input and wireless transmission
- **ESP32 Receiver** - Receives control data and forwards to Raspberry Pi
- **HotRC DS600 Joystick** - Single-axis controller with tank-style steering

### Power & Safety
- **12V Battery Pack** - Powers hoverboard motors
- **5V Power Supply** - Powers Raspberry Pi and ESP32s
- **Emergency Stop System** - RC controller with PWM-based safety cutoff

## 🔧 Hardware Setup

### Motor Connections
```
Hoverboard Motors:
├── Left Motor:  UART TX (GPIO 14) → Motor Controller
├── Right Motor: UART RX (GPIO 15) → Motor Controller
└── Power:       12V Battery → Motor Controllers
```

### ESP32 Pinout
```
Controller ESP32:
├── GPIO 20: X-axis (Steering)
├── GPIO 16: Y-axis (Speed/Throttle)
├── GPIO 21: RC Emergency Stop (PWM)
└── USB:     Power and data to receiver

Receiver ESP32:
├── USB:     Data to Raspberry Pi
└── GPIO 2:  Status LED
```

### Raspberry Pi Connections
```
Raspberry Pi Zero 2W:
├── UART TX (GPIO 14): → Hoverboard Left Motor
├── UART RX (GPIO 15): → Hoverboard Right Motor
├── USB:               → ESP32 Receiver
└── Power:             5V supply
```

## 📁 Project Structure

```
python_uart_cart/
├── controller/
│   └── controller.ino          # ESP32 controller firmware
├── hoverboard_minimal.py       # Main Python control script
├── hoverboard_controller.py    # Reference implementation
├── test_hoverboard.py          # Simple test script
└── README.md                   # This file
```

## 🚀 Quick Start

### 1. Flash ESP32 Firmware

**Controller ESP32:**
```bash
# Upload controller.ino to the joystick controller ESP32
# This handles joystick input and wireless transmission
```

**Receiver ESP32:**
```bash
# Upload receiver_minimal.ino to the receiver ESP32
# This receives data and forwards to Raspberry Pi via USB
```

### 2. Install Dependencies

```bash
# Install Python serial library
pip install pyserial

# Or install system-wide
sudo apt-get install python3-serial
```

### 3. Run the Controller

```bash
# Run the main control script
python3 hoverboard_minimal.py

# Or run the reference implementation
python3 hoverboard_controller.py
```

## 🎮 Control System

### Joystick Operation
- **Y-Axis (Forward/Backward)**: Controls speed and direction
- **X-Axis (Left/Right)**: Controls steering
- **Tank-Style Mixing**: `left_motor = throttle + steering`, `right_motor = throttle - steering`

### Operation Modes
- **Normal Mode**: 70% max speed
- **Turbo Mode**: 100% max speed
- **Follow-Me Mode**: 50% max speed
- **Parking Mode**: 30% max speed

### Safety Features
- **Emergency Stop**: RC controller PWM signal (<1400μs = normal, >1500μs = stop)
- **Signal Loss Protection**: Automatic stop after 3 seconds of no data
- **Joystick Deadzone**: Prevents motor jitter from small movements
- **Smooth Acceleration**: Configurable acceleration/deceleration rates

## 🔧 Configuration

### Motor Control Parameters
```python
# In hoverboard_minimal.py
MAX_SPEED = 400        # Maximum motor speed
MAX_STEERING = 600     # Maximum steering angle
ACCEL_RATE = 5.0       # Acceleration rate (units/second)
DECEL_RATE = 8.0       # Deceleration rate (units/second)
STEER_RATE = 10.0      # Steering rate (units/second)
```

### Communication Settings
```python
# ESP32 Communication
ESP32_BAUDRATE = 115200
HOVERBOARD_BAUDRATE = 9600
COMMAND_RATE = 20      # Hz (commands per second)
```

### Safety Timeouts
```python
DATA_TIMEOUT = 3.0     # Seconds before emergency stop
EMERGENCY_STOP_DURATION = 2.0  # Seconds to clear emergency stop
```

## 🛠️ Troubleshooting

### Common Issues

**Motors Not Responding:**
1. Check UART connections (GPIO 14/15)
2. Verify hoverboard power supply
3. Check motor controller connections
4. Test with `test_hoverboard.py`

**No Wireless Control:**
1. Verify ESP32 firmware is uploaded
2. Check USB connections
3. Monitor serial output for connection status
4. Ensure both ESP32s are powered

**Erratic Movement:**
1. Check for EMI interference (add aluminum foil shielding)
2. Verify joystick calibration
3. Check for loose connections
4. Monitor signal quality in logs

### Debug Commands

```bash
# Test hoverboard communication
python3 test_hoverboard.py

# Monitor USB ports
ls /dev/ttyUSB*

# Check UART permissions
sudo usermod -a -G dialout $USER
```

## 🔒 Safety Considerations

### Critical Safety Features
- **Emergency Stop**: Always functional via RC controller
- **Signal Loss Protection**: Automatic stop on communication failure
- **Speed Limits**: Configurable maximum speeds for each mode
- **EMI Shielding**: Aluminum foil around hoverboard motherboard

### Operating Guidelines
- Always test in open area first
- Keep emergency stop controller accessible
- Monitor battery levels
- Check all connections before use
- Start in low-speed modes

## 📊 Technical Specifications

### Performance
- **Max Speed**: ~15-20 mph (configurable)
- **Control Range**: ~100m (ESP32 wireless)
- **Response Time**: <50ms
- **Battery Life**: 2-4 hours (depending on usage)

### Communication Protocol
- **ESP-NOW**: Direct wireless between ESP32s
- **UART**: 9600 baud to hoverboard motors
- **USB Serial**: 115200 baud to Raspberry Pi
- **PWM**: Emergency stop signal (700-2300μs range)

### Hoverboard Protocol
```
Command Format: <start_frame><steer><speed><checksum>
- start_frame: 0xABCD (16-bit)
- steer: Differential steering (-32767 to 32767)
- speed: Average speed (-32767 to 32767)
- checksum: XOR of start_frame, steer, and speed
```

## 🤝 Contributing

1. Test thoroughly before submitting changes
2. Maintain safety features
3. Document any new features
4. Follow existing code style

## 📄 License

This project is for educational and personal use. Please ensure compliance with local regulations for motorized vehicles.

## ⚠️ Disclaimer

This project involves high-voltage motors and moving parts. Use at your own risk. Always prioritize safety and test in controlled environments.

---

**Built with ❤️ for golf cart automation**

## 🚧 TODO / Future Enhancements

### 🤖 Follow-Me Mode
- **Ultrasonic Sensors**: Distance measurement for following
- **Infrared Beacon**: Wearable IR transmitter for tracking
- **Path Planning**: Smooth following algorithm
- **Obstacle Avoidance**: Prevent collisions while following

### 👁️ Object Detection
- **Camera Integration**: Raspberry Pi Camera Module
- **Computer Vision**: OpenCV for obstacle detection
- **LIDAR/Laser Sensors**: Precise distance measurement
- **Safety Zones**: Configurable detection ranges

### 🎮 Remote Control Designs
- **Custom Controller**: 3D-printed ergonomic design
- **Smartphone App**: Bluetooth/WiFi control interface
- **Voice Commands**: Speech recognition for hands-free operation
- **Gesture Control**: Motion-based steering

### 🔧 Additional Features
- **GPS Navigation**: Autonomous waypoint following
- **Battery Management**: Smart charging and monitoring
- **Speed Profiles**: Terrain-adaptive speed control
- **Data Logging**: Trip recording and analytics