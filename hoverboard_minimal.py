#!/usr/bin/env python3
"""
Minimal Hoverboard Controller
Simple, clean implementation to eliminate corruption issues
"""

import serial
import struct
import time
import threading
import os
import sys

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
        self.cruise_control = 0
        self.cruise_speed = 0
        
        # Debug mode - set to False for production (silent operation)
        self.debug_mode = True  # Change to False to disable ALL logging
        
        # Speed change priority flag for immediate motor override
        self.speed_change_priority = False
        self.last_speed_change_time = 0
        
        # Motor command queue for immediate response
        self.motor_command_queue = []
        self.motor_command_lock = threading.Lock()
        
        # Running state
        self.running = False
        
    def debug_print(self, message):
        """Print message only if debug mode is enabled"""
        if self.debug_mode:
            print(message)
    
    def find_esp32_port(self):
        """Find ESP32 port"""
        # First try the persistent symlink
        if os.path.exists('/dev/esp32-receiver'):
            return '/dev/esp32-receiver'
        
        # Fallback to auto-detection
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
            self.debug_print(f"✓ Connected to ESP32 on {self.esp32_port}")
            return True
        except Exception as e:
            self.debug_print(f"✗ Failed to connect to ESP32: {e}")
            return False
    
    def connect_hoverboard(self):
        """Connect to hoverboard"""
        try:
            self.hoverboard_ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
            self.hoverboard_connected = True
            self.debug_print("✓ Connected to hoverboard")
            return True
        except Exception as e:
            self.debug_print(f"✗ Failed to connect to hoverboard: {e}")
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
        """Parse controller data format"""
        try:
            if len(data) < 8:  # Minimum size (start + 6 bytes + end)
                return False
                
            if data[0] != 0xAA or data[-1] != 0xBB:
                return False
                
            # Extract data (skip start/end bytes)
            data_bytes = data[1:-1]
            
            # New structure: throttle(2) + steering(2) + emergency(1) + cruise_control(1) + cruise_speed(2) + checksum(1) = 9 bytes
            if len(data_bytes) != 9:
                return False
                
            # Parse USB data structure (from receiver ESP32)
            throttle, steering, emergency_stop, cruise_control, cruise_speed, checksum = struct.unpack('<hhBBhB', data_bytes)
            
            # Update control data
            self.throttle = throttle
            self.steering = steering
            self.emergency_stop = emergency_stop
            self.cruise_control = cruise_control
            self.cruise_speed = cruise_speed
            
            return True
            
        except:
            return False
    
    def read_data_loop(self):
        """Read data from ESP32 with priority cruise control processing"""
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
                                # Check if cruise control state changed
                                if hasattr(self, 'last_cruise_control') and self.cruise_control != self.last_cruise_control:
                                    self.debug_print(f"🚨 CRUISE CONTROL STATE CHANGE: {self.last_cruise_control} -> {self.cruise_control}")
                                    if not self.cruise_control:
                                        self.debug_print(f"🚨 CRUISE CONTROL DISABLED - Processing immediately for safety!")
                                self.last_cruise_control = self.cruise_control
                                
                                # Check if cruise control speed changed (for instant response)
                                if hasattr(self, 'last_cruise_speed') and self.cruise_speed != self.last_cruise_speed:
                                    self.debug_print(f"🚗 CRUISE SPEED CHANGED: {self.last_cruise_speed} -> {self.cruise_speed}")
                                    
                                    # Set priority flag for immediate motor override
                                    self.speed_change_priority = True
                                    self.last_speed_change_time = time.time()
                                    
                                    # QUEUE MOTOR COMMAND for immediate execution in main thread
                                    if self.hoverboard_connected:
                                        # Calculate motor values directly (same logic as control_hoverboard)
                                        throttle_norm = self.cruise_speed / 512.0
                                        steering_norm = self.steering / 512.0
                                        
                                        # Tank-style mixing
                                        left_motor = throttle_norm - steering_norm
                                        right_motor = throttle_norm + steering_norm
                                        
                                        # Apply deadzone and scaling
                                        deadzone = 0.05
                                        if abs(left_motor) < deadzone: left_motor = 0
                                        if abs(right_motor) < deadzone: right_motor = 0
                                        
                                        max_speed = 300
                                        left_speed = int(left_motor * max_speed)
                                        right_speed = int(right_motor * max_speed)
                                        
                                        # Convert to hoverboard protocol
                                        steer = (left_speed - right_speed) // 2
                                        speed = (left_speed + right_speed) // 2
                                        
                                        # Clamp values
                                        steer = max(-300, min(300, steer))
                                        speed = max(-300, min(300, speed))
                                        
                                        # Queue command for immediate execution
                                        with self.motor_command_lock:
                                            self.motor_command_queue.append((steer, speed))
                                        self.debug_print(f"🚀 MOTOR COMMAND QUEUED: Steer={steer}, Speed={speed}")
                                    
                                    # Also update control loop for consistency
                                    self.control_hoverboard()
                                self.last_cruise_speed = self.cruise_speed
                                
                                # Only print data when not in cruise control to reduce overhead
                                if not self.cruise_control:
                                    self.debug_print(f"✓ Data: Throttle={self.throttle}, Steering={self.steering}, Emergency={self.emergency_stop}, Cruise={self.cruise_control}, CruiseSpeed={self.cruise_speed}")
                            else:
                                # Debug: Show packet info when parsing fails (only when not in cruise control)
                                if not self.cruise_control:
                                    self.debug_print(f"❌ Parse failed: Packet length={len(packet)}, Data length={len(packet)-2}")
                                    self.debug_print(f"   Raw: {[hex(b) for b in packet[:10]]}")
                
                # ULTRA-FAST reading during cruise control for instant response
                if self.cruise_control:
                    time.sleep(0.00001)  # 100000Hz reading rate during cruise control (10x faster!)
                else:
                    time.sleep(0.001)   # 1000Hz normal reading rate
                
            except Exception as e:
                print(f"Error reading data: {e}")
                time.sleep(1)
    
    def control_hoverboard(self):
        """Control hoverboard with immediate cruise control response"""
        # Check if cruise control is enabled
        if self.cruise_control:
            # Use cruise control speed instead of joystick throttle
            throttle_norm = self.cruise_speed / 512.0
            self.debug_print(f"🚗 Cruise control ACTIVE - Speed: {self.cruise_speed}")
        else:
            # Use normal joystick throttle
            throttle_norm = self.throttle / 512.0
            
                    # Track cruise control state changes for debugging
            if hasattr(self, 'last_cruise_state') and self.last_cruise_state != self.cruise_control:
                self.debug_print(f"🚨 CRUISE CONTROL STATE CHANGE: {self.last_cruise_state} -> {self.cruise_control}")
            self.last_cruise_state = self.cruise_control
            
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
        max_speed = 300  # Reduced from 500 for safer operation
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
            steer = max(-300, min(300, steer))  # Reduced from 500 for safer operation
            speed = max(-300, min(300, speed))  # Reduced from 500 for safer operation
            
            # Send in correct order: (steer, speed) as per test script
            self.send_hoverboard_command(steer, speed)
            
            self.debug_print(f"Motor: Left={left_speed}, Right={right_speed}")
    
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
        
        self.debug_print("Minimal controller started")
        self.debug_print("🚀 Performance mode: Auto-enabled during cruise control for instant response")
        
        try:
            while self.running:
                # Check if speed change priority is active
                if self.speed_change_priority:
                    # Clear priority flag after 100ms to allow normal operation
                    if time.time() - self.last_speed_change_time > 0.1:
                        self.speed_change_priority = False
                        if self.debug_mode:
                            self.debug_print("🔄 Speed change priority cleared - returning to normal operation\n")
                
                # EXECUTE QUEUED MOTOR COMMANDS IMMEDIATELY
                with self.motor_command_lock:
                    if self.motor_command_queue:
                        steer, speed = self.motor_command_queue.pop(0)
                        if self.hoverboard_connected:
                            self.send_hoverboard_command(steer, speed)
                            if self.debug_mode:
                                self.debug_print(f"🚀 EXECUTING QUEUED COMMAND: Steer={steer}, Speed={speed}\n")
                
                self.control_hoverboard()
                # ULTRA-FAST control loop during cruise control for instant response
                if self.cruise_control:
                    time.sleep(0.0001)  # 10000Hz during cruise control (100x faster!)
                else:
                    time.sleep(0.01)   # 100Hz normal operation
                
        except KeyboardInterrupt:
            self.debug_print("\nStopping...")
        finally:
            self.running = False
            if self.esp32_ser:
                self.esp32_ser.close()
            if self.hoverboard_ser:
                self.hoverboard_ser.close()

def main():
    print("Minimal Hoverboard Controller")
    print("============================")
    
    # Check for command-line arguments
    debug_mode = True  # Default to debug mode
    if len(sys.argv) > 1:
        if sys.argv[1].lower() in ['--silent', '-s', '--quiet', '-q']:
            debug_mode = False
            print("🔇 SILENT MODE: No logging output")
        elif sys.argv[1].lower() in ['--debug', '-d', '--verbose', '-v']:
            debug_mode = True
            print("🔊 DEBUG MODE: Full logging output")
        else:
            print(f"Usage: {sys.argv[0]} [--silent|--debug]")
            print("  --silent, -s: No logging output (production mode)")
            print("  --debug, -d: Full logging output (debug mode)")
            print("  No argument: Default debug mode")
            print("")
    
    if debug_mode:
        print("🔊 DEBUG MODE: Full logging output")
    print("")
    
    controller = MinimalHoverboardController()
    controller.debug_mode = debug_mode  # Override the default
    controller.start()

if __name__ == "__main__":
    main()
