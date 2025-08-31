#!/usr/bin/env python3
"""
Minimal Hoverboard Controller
Simple, clean implementation to eliminate corruption issues
"""

import serial
import struct
import time
import threading

class MinimalHoverboardController:
    def __init__(self):
        # ESP32 connection
        self.esp32_ser = None
        self.esp32_port = None
        
        # Hoverboard connection
        self.hoverboard_ser = None
        self.hoverboard_connected = False
        
        # Control data
        self.throttle = 0
        self.steering = 0
        self.emergency_stop = 0
        
        # Running state
        self.running = False
        
    def find_esp32_port(self):
        """Find ESP32 port"""
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        for port in ports:
            if any(identifier in port.description.lower() for identifier in ['esp32', 'cp2102', 'cp210x', 'ch340']):
                return port.device
        
        # Fallback to ttyUSB0
        return '/dev/ttyUSB0'
    
    def connect_esp32(self):
        """Connect to ESP32"""
        try:
            self.esp32_port = self.find_esp32_port()
            self.esp32_ser = serial.Serial(self.esp32_port, 115200, timeout=1)
            print(f"✓ Connected to ESP32 on {self.esp32_port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to ESP32: {e}")
            return False
    
    def connect_hoverboard(self):
        """Connect to hoverboard"""
        try:
            self.hoverboard_ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
            self.hoverboard_connected = True
            print("✓ Connected to hoverboard")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to hoverboard: {e}")
            return False
    
    def send_hoverboard_command(self, steer, speed):
        """Send command to hoverboard"""
        if not self.hoverboard_connected:
            return False
            
        # Clamp values to valid range for hoverboard protocol
        steer = max(-32767, min(32767, steer))
        speed = max(-32767, min(32767, speed))
        
        # Use the correct start frame that the hoverboard expects
        start_frame = 0xABCD  # Original hoverboard protocol start frame
        checksum = start_frame ^ steer ^ speed  # Correct checksum from official protocol
        
        # Clamp checksum to valid range for unsigned short
        checksum = checksum & 0xFFFF  # Keep only 16 bits
        
        # Pack: start_frame (H), steer (h), speed (h), checksum (H)
        # Note: 0xABCD (43981) fits in unsigned short range (0-65535)
        command = struct.pack('<HhhH', start_frame, steer, speed, checksum)
        
        try:
            self.hoverboard_ser.write(command)
            self.hoverboard_ser.flush()
            return True
        except:
            return False
    
    def parse_data(self, data):
        """Parse simple packet format"""
        try:
            if len(data) < 6:  # Minimum size
                return False
                
            if data[0] != 0xAA or data[-1] != 0xBB:
                return False
                
            # Extract data (skip start/end bytes)
            data_bytes = data[1:-1]
            
            if len(data_bytes) != 6:  # throttle(2) + steering(2) + emergency(1) + checksum(1) = 6 bytes
                return False
                
            # Parse data
            throttle, steering, emergency_stop, checksum = struct.unpack('<hhBB', data_bytes)
            
            # Validate checksum
            calculated_checksum = 0
            for i in range(5):  # First 5 bytes
                calculated_checksum ^= data_bytes[i]
            
            if calculated_checksum != checksum:
                return False
            
            # Update control data
            self.throttle = throttle
            self.steering = steering
            self.emergency_stop = emergency_stop
            
            return True
            
        except:
            return False
    
    def read_data_loop(self):
        """Read data from ESP32"""
        buffer = b''
        
        while self.running:
            try:
                if self.esp32_ser and self.esp32_ser.in_waiting > 0:
                    data = self.esp32_ser.read(self.esp32_ser.in_waiting)
                    if data:
                        buffer += data
                        
                        # Look for packets
                        while len(buffer) >= 8:  # Minimum packet size
                            # Find start byte
                            start_idx = buffer.find(b'\xAA')
                            if start_idx == -1:
                                buffer = b''
                                break
                            
                            # Find end byte
                            end_idx = buffer.find(b'\xBB', start_idx)
                            if end_idx == -1:
                                break
                            
                            # Extract packet
                            packet = buffer[start_idx:end_idx + 1]
                            buffer = buffer[end_idx + 1:]
                            
                            # Parse packet
                            if self.parse_data(packet):
                                print(f"✓ Data: Throttle={self.throttle}, Steering={self.steering}, Emergency={self.emergency_stop}")
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"Error reading data: {e}")
                time.sleep(1)
    
    def control_hoverboard(self):
        """Control hoverboard"""
        # Convert joystick to motor speeds
        throttle_norm = self.throttle / 512.0
        steering_norm = self.steering / 512.0
        
        # Tank-style mixing (corrected)
        # For left turn: left motor should slow down/reverse, right motor should speed up/forward
        # For right turn: left motor should speed up/forward, right motor should slow down/reverse
        left_motor = throttle_norm - steering_norm   # Left motor: throttle - steering
        right_motor = throttle_norm + steering_norm  # Right motor: throttle + steering
        
        # Apply deadzone
        deadzone = 0.05
        if abs(left_motor) < deadzone:
            left_motor = 0
        if abs(right_motor) < deadzone:
            right_motor = 0
        
        # Scale to motor range
        max_speed = 500
        left_speed = int(left_motor * max_speed)
        right_speed = int(right_motor * max_speed)
        
        # Emergency stop
        if self.emergency_stop:
            left_speed = 0
            right_speed = 0
        
        # Send command
        if self.hoverboard_connected:
            # Send correct hoverboard protocol: (steer, speed)
            # Based on test script: steer=0 for straight, speed=100 for forward
            # Convert to differential steering protocol (what hoverboard expects)
            # steer = (left - right) / 2 (differential steering)
            # speed = (left + right) / 2 (average speed)
            steer = (left_speed - right_speed) // 2  # Differential steering
            speed = (left_speed + right_speed) // 2  # Average speed
            
            # Add minimum speed for turns (hoverboard might need non-zero speed to accept steering)
            if abs(steer) > 50 and abs(speed) < 50:  # If turning but low speed
                speed = 50 if speed >= 0 else -50  # Add minimum speed in same direction
            
            # Clamp values to reasonable ranges
            steer = max(-500, min(500, steer))  # Allow full motor range
            speed = max(-500, min(500, speed))  # Allow full motor range
            
            # Send in correct order: (steer, speed) as per test script
            self.send_hoverboard_command(steer, speed)
            
            print(f"Motor: Left={left_speed}, Right={right_speed}")
    
    def start(self):
        """Start the controller"""
        if not self.connect_esp32():
            return False
        
        if not self.connect_hoverboard():
            print("Warning: Hoverboard not connected")
        
        self.running = True
        
        # Start data thread
        data_thread = threading.Thread(target=self.read_data_loop, daemon=True)
        data_thread.start()
        
        print("Minimal controller started")
        
        try:
            while self.running:
                self.control_hoverboard()
                time.sleep(0.05)  # 20Hz
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            if self.esp32_ser:
                self.esp32_ser.close()
            if self.hoverboard_ser:
                self.hoverboard_ser.close()

def main():
    print("Minimal Hoverboard Controller")
    print("============================")
    
    controller = MinimalHoverboardController()
    controller.start()

if __name__ == "__main__":
    main()
