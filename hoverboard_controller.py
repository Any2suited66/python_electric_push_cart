#!/usr/bin/env python3
"""
Hoverboard Controller - Main control script
Reads USB data from ESP32 receiver and controls hoverboard via UART
"""

import serial
import serial.tools.list_ports
import struct
import time
import threading
from typing import Optional

class HoverboardController:
    def __init__(self, esp32_port=None, esp32_baudrate=115200, 
                 hoverboard_port='/dev/ttyAMA0', hoverboard_baudrate=9600):
        # ESP32 connection parameters - auto-detect if not specified
        self.esp32_port = esp32_port
        self.esp32_baudrate = esp32_baudrate
        self.esp32_ser = None
        
        # Hoverboard UART connection parameters
        self.hoverboard_port = hoverboard_port
        self.hoverboard_baudrate = hoverboard_baudrate
        self.hoverboard_ser = None
        self.hoverboard_connected = False
        
        self.running = False
        self.data_thread = None
        
        # Control data
        self.throttle = 0
        self.steering = 0
        self.mode = 0
        self.emergency_stop = 0  # Start with emergency stop DISABLED for testing
        self.battery_level = 100
        self.cart_battery = 85
        self.follow_me_active = 0
        self.last_data_time = 0
        
        # Data buffering and retry mechanism
        self.last_good_throttle = 0
        self.last_good_steering = 0
        self.last_good_data_time = 0
        self.data_freshness_timeout = 0.3  # Reduced to 300ms for faster recovery
        self.consecutive_bad_packets = 0
        self.max_bad_packets = 5  # Reduced to 5 bad packets for faster fallback
        
        # Fast movement detection and handling
        self.last_throttle_change = 0
        self.last_steering_change = 0
        self.fast_movement_threshold = 100  # Detect fast joystick movements
        self.fast_movement_timeout = 0.1  # 100ms timeout for fast movements
        
        # Hoverboard control parameters - DIRECT CONTROL
        self.max_speed = 500  # Increased for better response and reverse power
        self.max_steering = 600  # Increased for better steering
        
        # Direct control - no smoothing
        self.last_command_time = 0
        self.command_interval = 0.02  # 50Hz command rate for smoother control
        
        # Command tracking
        self.last_throttle_command = 0
        self.last_steering_command = 0
        self.alternate_commands = False  # Use combined commands (worked in test)
    
    def find_esp32_port(self) -> Optional[str]:
        """Find ESP32 USB port automatically with flexible detection"""
        try:
            ports = list(serial.tools.list_ports.comports())
            
            # Priority 1: Look for specific ESP32 identifiers
            for port in ports:
                if any(identifier in port.description.lower() for identifier in ['esp32', 'cp2102', 'cp210x']):
                    print(f"Found ESP32 device: {port.device} - {port.description}")
                    return port.device
            
            # Priority 2: Look for any USB serial device (ttyUSB)
            usb_ports = [port for port in ports if 'ttyUSB' in port.device]
            if usb_ports:
                print(f"Found USB serial device: {usb_ports[0].device} - {usb_ports[0].description}")
                return usb_ports[0].device
            
            # Priority 3: Look for any serial device that might be ESP32
            for port in ports:
                if any(identifier in port.description.lower() for identifier in ['ch340', 'ftdi', 'usb', 'serial']):
                    print(f"Found potential ESP32 device: {port.device} - {port.description}")
                    return port.device
            
            print("No ESP32 device found. Available ports:")
            for port in ports:
                print(f"  {port.device}: {port.description}")
            return None
        except Exception as e:
            print(f"Error detecting ports: {e}")
            return None
    
    def connect_esp32(self) -> bool:
        """Connect to ESP32 via USB with flexible port detection"""
        # If no port specified, use auto-detection
        if not self.esp32_port:
            detected_port = self.find_esp32_port()
            if detected_port:
                self.esp32_port = detected_port
                print(f"Auto-detected ESP32 port: {self.esp32_port}")
            else:
                print("No ESP32 port detected")
                return False
        
        # Try to connect to the specified/detected port
        try:
            self.esp32_ser = serial.Serial(self.esp32_port, self.esp32_baudrate, timeout=1)
            print(f"✓ Connected to ESP32 on {self.esp32_port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to ESP32 on {self.esp32_port}: {e}")
            
            # If connection failed, try auto-detection to find the correct port
            print("Trying auto-detection for different port...")
            detected_port = self.find_esp32_port()
            if detected_port and detected_port != self.esp32_port:
                try:
                    self.esp32_port = detected_port
                    self.esp32_ser = serial.Serial(self.esp32_port, self.esp32_baudrate, timeout=1)
                    print(f"✓ Connected to ESP32 on {self.esp32_port}")
                    return True
                except Exception as e2:
                    print(f"✗ Auto-detection also failed: {e2}")
            
            return False
    
    def connect_hoverboard(self) -> bool:
        """Connect to hoverboard via UART"""
        try:
            self.hoverboard_ser = serial.Serial(self.hoverboard_port, self.hoverboard_baudrate, timeout=1)
            self.hoverboard_connected = True
            print(f"✓ Connected to hoverboard on {self.hoverboard_port} at {self.hoverboard_baudrate} baud")
            
            # Send initial stop command for safety
            self.send_hoverboard_command(0, 0)
            time.sleep(0.5)
            return True
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect to hoverboard on {self.hoverboard_port}: {e}")
            return False
    
    def connect(self) -> bool:
        """Connect to both ESP32 and hoverboard"""
        esp32_ok = self.connect_esp32()
        hoverboard_ok = self.connect_hoverboard()
        
        if not esp32_ok:
            print("Warning: ESP32 connection failed - no control data will be received")
        if not hoverboard_ok:
            print("Warning: Hoverboard connection failed - no motor commands will be sent")
        
        return esp32_ok or hoverboard_ok  # Continue if at least one connection works
    
    def disconnect(self):
        """Disconnect from ESP32 and hoverboard"""
        try:
            if self.esp32_ser:
                self.esp32_ser.close()
                self.esp32_ser = None
            if self.hoverboard_ser and self.hoverboard_connected:
                self.send_hoverboard_command(0, 0)  # Safety stop
                time.sleep(0.5)
                self.hoverboard_ser.close()
                self.hoverboard_ser = None
                self.hoverboard_connected = False
            self.running = False
            print("✓ Disconnected from all devices")
        except Exception as e:
            print(f"Error during disconnect: {e}")
            self.running = False
    
    def calculate_checksum(self, start, steer, speed):
        """Calculate XOR checksum for the hoverboard command (same as golf cart)"""
        # Simple XOR checksum: start_frame ^ speed (same as test script)
        return start ^ speed
    
    def send_hoverboard_command(self, steer, speed):
        """Send a command to the hoverboard using the same protocol as golf cart"""
        if not self.hoverboard_connected or not self.hoverboard_ser:
            return False
            
        start_frame = 0xABCD  # Same start frame as golf cart
        checksum = self.calculate_checksum(start_frame, steer, speed)
        
        # Pack the binary structure: start_frame (H), steer (h), speed (h), checksum (H)
        command = struct.pack('<HhhH', start_frame, steer, speed, checksum)
        
        try:
            self.hoverboard_ser.write(command)
            self.hoverboard_ser.flush()
            return True
        except Exception as e:
            print(f"✗ Hoverboard send failed: {e}")
            self.hoverboard_connected = False
            return False
    
    def parse_data(self, data: bytes) -> bool:
        """Parse USB data from ESP32 with flexible format detection"""
        try:
            # Debug: Always show what we're trying to parse
            print(f"🔍 PARSING: Raw data length={len(data)}, bytes={[hex(b) for b in data[:10]]}...")
            
            if len(data) < 3:  # Need at least start + 1 byte + end
                print(f"❌ Too short: {len(data)} bytes")
                return False
                
            if data[0] != 0xAA:  # Check start byte (changed from 0xFF)
                print(f"❌ Wrong start byte: 0x{data[0]:02x} (expected 0xAA)")
                return False
                
            # Look for end byte (ESP32 sends 0xBB, changed from 0xFE)
            end_pos = -1
            for i in range(1, len(data)):
                if data[i] == 0xBB:
                    end_pos = i
                    break
            
            if end_pos == -1:
                print(f"❌ No end byte (0xBB) found in data")
                return False
                
            # Extract data bytes (everything between start and end)
            data_bytes = data[1:end_pos]
            
            # Debug: Show packet info occasionally
            if not hasattr(self, 'packet_debug_counter'):
                self.packet_debug_counter = 0
            self.packet_debug_counter += 1
            
            # Debug: Show packet info for troubleshooting
            if self.packet_debug_counter % 10 == 0:  # Every 10 packets (more frequent)
                print(f"🔍 DEBUG: Packet size={len(data_bytes)} bytes, data={[hex(b) for b in data_bytes[:10]]}...")
                print(f"🔍 DEBUG: Raw data length={len(data)}, start=0x{data[0]:02x}, end=0x{data[-1] if len(data) > 0 else 0:02x}")
            
            # Try 11-byte format (matches receiver usb_data_t struct)
            if len(data_bytes) >= 11:
                try:
                    # Format: throttle(h), steering(h), mode(B), emergency_stop(B), battery_level(B), cart_battery(B), follow_me_active(B), controller_connected(B), checksum(B)
                    # Total: 2+2+1+1+1+1+1+1+1 = 11 bytes
                    throttle, steering, mode, emergency_stop, battery_level, cart_battery, follow_me_active, controller_connected, checksum = struct.unpack('<hhBBBBBBBB', data_bytes[:11])
                    
                    # Validate data ranges
                    if (abs(throttle) <= 512 and abs(steering) <= 512 and 
                        emergency_stop <= 1 and mode <= 3 and
                        battery_level <= 100 and cart_battery <= 100):
                        
                        return self._update_control_data(throttle, steering, mode, emergency_stop, battery_level, cart_battery, follow_me_active, controller_connected)
                        
                except struct.error:
                    pass
            
            # If we get here, no format worked
            if not hasattr(self, 'format_error_counter'):
                self.format_error_counter = 0
            self.format_error_counter += 1
            
            # Track consecutive bad packets for retry mechanism
            self.consecutive_bad_packets += 1
            
            if self.format_error_counter % 50 == 0:  # Every 50 failed packets
                print(f"⚠ No valid format found for {len(data_bytes)} bytes: {[hex(b) for b in data_bytes[:8]]}...")
                print(f"📊 Bad packet stats: Consecutive={self.consecutive_bad_packets}, Last good data time={time.time() - self.last_good_data_time:.3f}s ago")
            
            return False
            
        except Exception as e:
            print(f"Error parsing data: {e}")
            return False
    
    def _update_control_data(self, throttle, steering, mode, emergency_stop, battery_level, cart_battery, follow_me_active, controller_connected):
        """Update control data after successful parsing"""
        # Check if data actually changed
        data_changed = (throttle != self.throttle or steering != self.steering or 
                       emergency_stop != self.emergency_stop or mode != self.mode)
        
        # Update control data
        self.throttle = throttle
        self.steering = steering
        self.mode = mode
        self.emergency_stop = emergency_stop
        self.battery_level = battery_level
        self.cart_battery = cart_battery
        self.follow_me_active = follow_me_active
        self.controller_connected = controller_connected
        self.last_data_time = time.time()
        
        # Store last good data for retry mechanism
        self.last_good_throttle = throttle
        self.last_good_steering = steering
        self.last_good_data_time = time.time()
        self.consecutive_bad_packets = 0  # Reset bad packet counter on good data
        
        # Debug output when data changes or every 20th packet
        if not hasattr(self, 'debug_counter'):
            self.debug_counter = 0
        self.debug_counter += 1
        if data_changed or self.debug_counter % 20 == 0:
            print(f"✓ Parsed: Throttle={throttle}, Steering={steering}, Emergency={emergency_stop}, Mode={mode}, Battery={battery_level}%, Connected={controller_connected}")
        
        return True
    
    def get_best_available_data(self):
        """Get the best available joystick data with fast movement handling"""
        current_time = time.time()
        
        # Check for fast joystick movements that might cause packet corruption
        throttle_change = abs(self.throttle - self.last_good_throttle)
        steering_change = abs(self.steering - self.last_good_steering)
        
        # Detect fast movements
        fast_movement = (throttle_change > self.fast_movement_threshold or 
                        steering_change > self.fast_movement_threshold)
        
        # Debug: Show current state
        if not hasattr(self, 'data_debug_counter'):
            self.data_debug_counter = 0
        self.data_debug_counter += 1
        if self.data_debug_counter % 50 == 0:  # Every 50 calls
            print(f"🔍 Data Status: Current=({self.throttle},{self.steering}), Last Good=({self.last_good_throttle},{self.last_good_steering}), Bad Packets={self.consecutive_bad_packets}, Time Since Good={current_time - self.last_good_data_time:.3f}s")
            
            # Check if joystick is actually active
            current_active = self.is_joystick_active(self.throttle, self.steering)
            last_good_active = self.is_joystick_active(self.last_good_throttle, self.last_good_steering)
            print(f"🎮 Joystick Activity: Current={current_active}, Last Good={last_good_active}")
            
            # Show fast movement detection
            if fast_movement:
                print(f"⚡ Fast movement detected: Throttle Δ={throttle_change}, Steering Δ={steering_change}")
        
        # If fast movement detected, use last good data for a short period
        if fast_movement and (current_time - self.last_good_data_time < self.fast_movement_timeout):
            # Debug: Show fast movement handling
            if not hasattr(self, 'fast_movement_counter'):
                self.fast_movement_counter = 0
            self.fast_movement_counter += 1
            if self.fast_movement_counter % 10 == 0:  # Every 10 fast movements
                print(f"⚡ Fast movement: Using last good data (throttle Δ={throttle_change}, steering Δ={steering_change})")
            return self.last_good_throttle, self.last_good_steering
        
        # If we have recent good data and no fast movement, use current data
        if (current_time - self.last_good_data_time < self.data_freshness_timeout and 
            self.consecutive_bad_packets < self.max_bad_packets):
            return self.throttle, self.steering
        
        # If we have too many bad packets, use last known good data
        if self.consecutive_bad_packets >= self.max_bad_packets:
            # Debug output for retry mechanism
            if not hasattr(self, 'retry_debug_counter'):
                self.retry_debug_counter = 0
            self.retry_debug_counter += 1
            if self.retry_debug_counter % 20 == 0:  # Every 20 retries
                print(f"🔄 Using last good data: Throttle={self.last_good_throttle}, Steering={self.last_good_steering} (bad packets: {self.consecutive_bad_packets})")
            return self.last_good_throttle, self.last_good_steering
        
        # Fallback to current data
        return self.throttle, self.steering
    
    def read_data_loop(self):
        """Background thread to read data from ESP32"""
        buffer = b''
        packet_count = 0
        last_packet_time = time.time()
        consecutive_errors = 0
        
        print("Data reading thread started")
        
        while self.running:
            try:
                if self.esp32_ser and self.esp32_ser.in_waiting > 0:
                    # Read available data
                    data = self.esp32_ser.read(self.esp32_ser.in_waiting)
                    if data:  # Only process if we actually got data
                        buffer += data
                        consecutive_errors = 0  # Reset error counter on successful read
                        
                        # Debug: Show raw data occasionally
                        if not hasattr(self, 'raw_data_counter'):
                            self.raw_data_counter = 0
                        self.raw_data_counter += 1
                        if self.raw_data_counter % 100 == 0:  # Every 100 reads
                            print(f"📡 Raw data received: {len(data)} bytes, buffer size: {len(buffer)}")
                        
                        # Look for complete packets
                        while len(buffer) >= 12:  # Minimum packet size
                            # Find start byte (changed from 0xFF to 0xAA)
                            start_idx = buffer.find(b'\xAA')
                            if start_idx == -1:
                                buffer = b''
                                break
                            
                            # Look for end byte (changed from 0xFE to 0xBB)
                            end_idx = -1
                            for i in range(start_idx + 1, len(buffer)):
                                if buffer[i] == 0xBB:
                                    end_idx = i
                                    break
                            
                            if end_idx != -1:
                                # Extract complete packet
                                packet = buffer[start_idx:end_idx + 1]
                                buffer = buffer[end_idx + 1:]
                                
                                # Parse the packet
                                if self.parse_data(packet):
                                    packet_count += 1
                                    current_time = time.time()
                                    packet_rate = 1.0 / (current_time - last_packet_time) if current_time > last_packet_time else 0
                                    last_packet_time = current_time
                                    
                                    # Print every packet for debugging (temporarily)
                                    if packet_count % 5 == 0:
                                        print(f"Packet #{packet_count} - Rate: {packet_rate:.1f}Hz - Throttle={self.throttle}, Steering={self.steering}, Emergency={self.emergency_stop}, Battery={self.battery_level}%")
                            else:
                                # No end byte found, keep buffer
                                break
                    else:
                        # No data available, this is normal
                        pass
                
                time.sleep(0.001)  # 1ms delay for higher precision
                
            except serial.SerialException as e:
                consecutive_errors += 1
                print(f"ESP32 serial error ({consecutive_errors}): {e}")
                if consecutive_errors >= 5:
                    print("Too many consecutive errors, attempting to reconnect ESP32...")
                    try:
                        if self.esp32_ser:
                            self.esp32_ser.close()
                        self.connect_esp32()
                        consecutive_errors = 0
                    except Exception as reconnect_error:
                        print(f"Reconnection failed: {reconnect_error}")
                time.sleep(1)
            except Exception as e:
                consecutive_errors += 1
                print(f"Error in data read loop ({consecutive_errors}): {e}")
                if consecutive_errors >= 10:
                    print("Too many errors, pausing for 5 seconds...")
                    time.sleep(5)
                    consecutive_errors = 0
                else:
                    time.sleep(1)
    
    def apply_differential_control(self):
        """Apply differential steering control - joystick directly controls left/right motors"""
        # Get the best available joystick data (fresh or last good)
        throttle, steering = self.get_best_available_data()
        
        # Convert joystick values directly
        # Safety check: clamp values to valid range
        throttle = max(-512, min(512, throttle))  # Y-axis: forward/backward
        steering = max(-512, min(512, steering))  # X-axis: left/right
        
        # DIFFERENTIAL STEERING CALCULATION
        # Convert joystick to motor speeds using tank-style mixing
        # left_motor = throttle + steering
        # right_motor = throttle - steering
        
        # Normalize joystick values to -1.0 to 1.0
        throttle_norm = throttle / 512.0
        steering_norm = steering / 512.0
        
        # Calculate left and right motor speeds
        left_motor = throttle_norm + steering_norm
        right_motor = throttle_norm - steering_norm
        
        # Apply deadzone to prevent motor jitter
        deadzone = 0.05  # 5% deadzone
        if abs(left_motor) < deadzone:
            left_motor = 0
        if abs(right_motor) < deadzone:
            right_motor = 0
        
        # Scale to motor command range
        left_speed = int(left_motor * self.max_speed)
        right_speed = int(right_motor * self.max_speed)
        
        # Clamp to valid range
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
        
        # Emergency stop override
        if self.emergency_stop:
            left_speed = 0
            right_speed = 0
        
        # If we're using last good data and it's been too long, gradually reduce speed
        current_time = time.time()
        if (current_time - self.last_good_data_time > self.data_freshness_timeout * 5 and 
            self.consecutive_bad_packets >= self.max_bad_packets * 2):
            # Only reduce speed after much longer periods (2.5+ seconds)
            left_speed = int(left_speed * 0.8)  # Less aggressive reduction (20% instead of 50%)
            right_speed = int(right_speed * 0.8)
            
            # Debug: Show when safety reduction is applied
            if not hasattr(self, 'safety_reduction_counter'):
                self.safety_reduction_counter = 0
            self.safety_reduction_counter += 1
            if self.safety_reduction_counter % 10 == 0:  # Every 10 reductions
                print(f"⚠️ Safety reduction applied: Left={left_speed}, Right={right_speed} (time since good: {current_time - self.last_good_data_time:.1f}s)")
        
        return left_speed, right_speed
    
    def is_joystick_active(self, throttle, steering):
        """Check if joystick is actually being moved (not just noise)"""
        # Define deadzone for joystick movement
        deadzone = 10  # Minimum movement threshold
        
        # Check if either axis is outside the deadzone
        throttle_active = abs(throttle) > deadzone
        steering_active = abs(steering) > deadzone
        
        return throttle_active or steering_active
    
    def control_hoverboard(self):
        """Send differential control commands to hoverboard - separate left/right motor control"""
        # Apply differential control (left/right motor speeds)
        left_speed, right_speed = self.apply_differential_control()
        
        # Send commands to hoverboard via UART
        current_time = time.time()
        if self.hoverboard_connected and (current_time - self.last_command_time >= self.command_interval):
            
            success = False  # Initialize success variable
            
            # Send differential commands (left and right motor speeds)
            # Note: hoverboard protocol expects (steering, speed) where:
            # speed = (left + right) / 2 (average speed)
            # steering = (left - right) / 2 (differential)
            avg_speed = (left_speed + right_speed) // 2
            differential = (left_speed - right_speed) // 2
            
            success = self.send_hoverboard_command(differential, avg_speed)
            self.last_command_time = current_time
            
            # Debug output (less frequently)
            if not hasattr(self, 'command_counter'):
                self.command_counter = 0
            self.command_counter += 1
            if self.command_counter % 1 == 0:  # Print EVERY command for debugging
                status = "✓" if success else "✗"
                # Show original joystick values and calculated motor commands
                print(f"{status} Hoverboard: Left={left_speed}, Right={right_speed} (Avg={avg_speed}, Diff={differential})")
                print(f"  Joystick: Throttle={self.throttle}, Steering={self.steering}")
                print(f"  Emergency={self.emergency_stop}, Battery={self.battery_level}%")
        elif not self.hoverboard_connected:
            # Debug output when hoverboard not connected
            if not hasattr(self, 'command_counter'):
                self.command_counter = 0
            self.command_counter += 1
            if self.command_counter % 20 == 0:  # Print every 20th command (1 second at 20Hz)
                print(f"⚠ Hoverboard Command (no UART): Left={left_speed}, Right={right_speed}, Emergency={self.emergency_stop}, Battery={self.battery_level}%")
    
    def check_timeout(self):
        """Check for data timeout and handle emergency stop - DISABLED FOR TESTING"""
        # Emergency stop disabled to prevent interference with motor control
        # if time.time() - self.last_data_time > 1.0:  # 1 second timeout
        #     if not self.emergency_stop:
        #         print("SIGNAL LOST - EMERGENCY STOP ACTIVATED")
        #         self.emergency_stop = 1
        #         self.throttle = 0
        #         self.steering = 0
        #         
        #         # Send emergency stop to hoverboard
        #         if self.hoverboard_connected:
        #             self.send_hoverboard_command(0, 0)
        #             print("✓ Emergency stop sent to hoverboard via UART")
        pass
    
    def start(self):
        """Start the controller"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Start data reading thread
        self.data_thread = threading.Thread(target=self.read_data_loop, daemon=True)
        self.data_thread.start()
        
        print("Hoverboard Controller started")
        print("Waiting for ESP32 data...")
        
        try:
            while self.running:
                # Check for timeout
                self.check_timeout()
                
                # Control hoverboard
                self.control_hoverboard()
                
                time.sleep(0.05)  # 20Hz control loop for better responsiveness
                
        except KeyboardInterrupt:
            print("\nStopping controller...")
        finally:
            self.disconnect()
    
    def get_status(self):
        """Get current controller status"""
        return {
            'throttle': self.throttle,
            'steering': self.steering,
            'mode': self.mode,
            'emergency_stop': self.emergency_stop,
            'battery_level': self.battery_level,
            'cart_battery': self.cart_battery,
            'follow_me_active': self.follow_me_active,
            'last_data_time': self.last_data_time
        }

def main():
    print("Hoverboard Controller with UART Communication")
    print("=============================================")
    print("ESP32 USB: Receives control data from wireless controller")
    print("Hoverboard UART: Sends motor commands via GPIO 14/15")
    print()
    
    # Create controller instance with auto-detection
    controller = HoverboardController(esp32_port=None)  # Auto-detect ESP32 port
    controller.start()

if __name__ == "__main__":
    main()
