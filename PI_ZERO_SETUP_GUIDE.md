# Raspberry Pi Zero 2 W Push Cart Setup Guide

## 🚗 Complete Electric Push Cart Controller

**Components:**
- Raspberry Pi Zero 2 W
- MPU6050 (tilt/stability sensor)
- HOTRC DS600 RC Controller
- 2x ODESC v4.2 + Hoverboard Motors
- USB Hub (for dual motor connections)

---

## 📋 Required Components

### **Hardware:**
- Raspberry Pi Zero 2 W
- MicroSD card (32GB+ recommended)
- USB Hub (powered, 4+ ports)
- MPU6050 breakout board
- HOTRC DS600 transmitter/receiver
- Jumper wires
- Breadboard or custom PCB

### **Software:**
- Raspberry Pi OS Lite
- Python 3 + required libraries

---

## 🔌 Wiring Connections

### **MPU6050 → Pi Zero 2 W (I2C)**
```
MPU6050    Pi Zero 2 W
VCC    →   3.3V (Pin 1)
GND    →   GND (Pin 6)
SCL    →   GPIO 3 (Pin 5) - I2C Clock
SDA    →   GPIO 2 (Pin 3) - I2C Data
```

### **HOTRC DS600 Receiver → Pi Zero 2 W**
```
DS600 RX   Pi Zero 2 W
Ch1 (Throttle) → GPIO 18 (Pin 12)
Ch2 (Steering)  → GPIO 19 (Pin 35)
VCC        → 5V (Pin 2)
GND        → GND (Pin 6)
```

### **USB Connections**
```
Device                     Connection
Left Motor (ODESC)    →    USB Hub Port 1
Right Motor (ODESC)   →    USB Hub Port 2
USB Hub               →    Pi Zero USB port
```

---

## ⚙️ Software Installation

### **1. Prepare SD Card**
```bash
# Flash Raspberry Pi OS Lite to SD card
# Enable SSH and WiFi in boot partition

# Add to /boot/ssh (empty file)
# Add to /boot/wpa_supplicant.conf:
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YourWiFiName"
    psk="YourWiFiPassword"
}
```

### **2. First Boot Setup**
```bash
# SSH into Pi Zero
ssh pi@raspberrypi.local

# Update system
sudo apt update && sudo apt upgrade -y

# Enable I2C
sudo raspi-config
# → Interface Options → I2C → Enable

# Install required packages
sudo apt install -y python3-pip python3-smbus python3-rpi.gpio git

# Install ODrive library
pip3 install odrive

# Clone your controller code
git clone <your-repo> /home/pi/push_cart
```

### **3. Install Dependencies**
```bash
# Install Python libraries
pip3 install odrive smbus RPi.GPIO

# Test I2C connection
sudo i2cdetect -y 1
# Should show MPU6050 at address 0x68
```

---

## 🔧 Hardware Setup

### **Power Requirements:**
- **Pi Zero 2 W:** 5V/1A minimum
- **USB Hub:** 5V/2A (powered hub recommended)
- **Motors:** Powered independently from main battery
- **MPU6050:** 3.3V from Pi
- **RC Receiver:** 5V from Pi or separate BEC

### **Physical Mounting:**
1. **Mount Pi Zero** securely to cart frame
2. **Position MPU6050** at cart center for accurate tilt detection
3. **Mount RC receiver** with good antenna clearance
4. **Secure USB connections** to prevent disconnection
5. **Protect connections** from moisture/debris

---

## 🧪 Testing and Calibration

### **1. Test Individual Components**
```bash
# Test MPU6050
python3 -c "
from smbus import SMBus
bus = SMBus(1)
bus.write_byte_data(0x68, 0x6B, 0)
print('MPU6050 OK')
"

# Test RC Controller
python3 -c "
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN)
# Move RC stick and watch GPIO changes
"

# Test Motor Connection
python3 -c "
import odrive
odrv = odrive.find_any()
print(f'Motor found: {hex(odrv.serial_number)}')
"
```

### **2. Run Full System**
```bash
cd /home/pi/push_cart
python3 pi_zero_push_cart_controller.py
```

---

## 🛡️ Safety Features

### **Built-in Safety Systems:**
- **Tilt monitoring:** Stops motors if cart tilts >30°
- **RC connection monitoring:** Emergency stop if signal lost
- **Velocity limiting:** Max speed protection
- **Emergency stop:** Immediate motor shutdown capability

### **Safety Configuration:**
```python
# Adjust in controller code:
self.max_velocity = 3.0        # Max speed (rad/s)
self.max_tilt_angle = 30.0     # Max safe tilt (degrees)
```

---

## 📡 RC Controller Setup (HOTRC DS600)

### **Channel Assignment:**
- **Channel 1:** Throttle (forward/reverse)
- **Channel 2:** Steering (left/right)

### **Transmitter Setup:**
1. **Bind transmitter** to receiver
2. **Set stick modes** (Mode 2 recommended)
3. **Calibrate endpoints** (1000-2000μs range)
4. **Test fail-safe** (should return to neutral)

---

## 🚀 Operation

### **Startup Sequence:**
1. **Power on Pi Zero** and wait for boot
2. **Turn on RC transmitter**
3. **Run controller script**
4. **Verify all systems** show "✅ OK"
5. **Test with small movements first**

### **Status Indicators:**
```
📊 T:0.50 S:-0.20 | L:1.5 R:2.1 | Tilt: 2.3°/1.1°
    ↑     ↑         ↑    ↑        ↑
 Throttle Steering Left Right   Pitch/Roll
```

### **Emergency Procedures:**
- **Lost control:** Turn off RC transmitter (triggers failsafe)
- **Tilt warning:** System automatically stops motors
- **Manual stop:** Ctrl+C in terminal

---

## 🔍 Troubleshooting

### **Common Issues:**

**Motors not found:**
```bash
# Check USB connections
lsusb
# Should show ODrive devices

# Check permissions
sudo usermod -a -G dialout pi
```

**MPU6050 not detected:**
```bash
# Check I2C
sudo i2cdetect -y 1
# Enable I2C if not visible
sudo raspi-config
```

**RC not responding:**
```bash
# Check GPIO connections
# Verify receiver is bound and powered
# Check PWM signal with oscilloscope
```

**System performance:**
```bash
# Monitor CPU usage
htop

# Check temperature
vcgencmd measure_temp
```

---

## 🏗️ Advanced Features

### **Auto-Start on Boot:**
```bash
# Add to /etc/rc.local (before exit 0):
cd /home/pi/push_cart
python3 pi_zero_push_cart_controller.py &
```

### **Remote Monitoring:**
```bash
# Install VNC for remote access
sudo apt install realvnc-vnc-server
sudo raspi-config
# → Interface Options → VNC → Enable
```

### **Data Logging:**
```python
# Add to controller for telemetry
import logging
logging.basicConfig(filename='cart.log', level=logging.INFO)
```

---

## ⚡ Performance Optimization

### **CPU Optimization:**
- Use **GPU split**: `sudo raspi-config` → Advanced → Memory Split → 16
- **Disable unused services**: Bluetooth, audio if not needed
- **Overclock safely**: Stay within thermal limits

### **Power Management:**
- Use **quality power supply** (2.5A minimum)
- **Monitor voltage**: `vcgencmd measure_volts`
- **Add capacitors** for motor control stability

---

## 🎯 Next Steps

**You now have a complete, professional-grade electric push cart controller!**

**Features achieved:**
✅ **Dual motor control** via reliable USB  
✅ **RC remote control** with failsafe  
✅ **Tilt safety monitoring** with automatic stop  
✅ **Real-time status feedback**  
✅ **Emergency stop systems**  

**Ready to test your push cart!** 🚗💨 