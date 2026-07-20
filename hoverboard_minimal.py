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
import smbus2
import math

from matrix_lidar import MatrixLidarFollow
from phone_bridge import PhoneBridge, DEFAULT_PORT as PHONE_BRIDGE_PORT
from phone_gps import PhoneGps

class MinimalHoverboardController:
    def __init__(self):
        # ESP32 receiver connection (handheld remote)
        self.esp32_ser = None
        self.esp32_port = None

        # Control data sources (camera_* = phone vision + LiDAR follow targets)
        self.controller_throttle = 0
        self.controller_steering = 0
        self.camera_throttle = 0
        self.camera_steering = 0
        
        # Hoverboard connection
        self.hoverboard_ser = None
        self.hoverboard_connected = False
        
        # Control data (final values used for motor control)
        self.throttle = 0
        self.steering = 0
        self.emergency_stop = 0
        self.cruise_control = 0
        self.cruise_speed = 0
        self.follow_me_mode = 0  # 0 = controller mode, 1 = follow-me mode
        self.turbo_mode = 0
        self.battery_level = 0
        self.button_states = 0
        
        # Debug mode - set to False for production (silent operation)
        self.debug_mode = True  # Change to False to disable ALL logging
        self.phone_bridge = None
        self.phone_gps = None
        self.matrix_lidar = None
        self.lidar_distance_cm = 0
        self.lidar_throttle = 0
        self.distance_source = "none"  # lidar | phone | none
        # When LiDAR is lost, latch phone "too close" so reverse doesn't
        # flicker off every time body-size noise crosses the deadzone.
        self._phone_too_close = False
        self._lidar_thread = None
        self._last_logged_phone_status = None
        self._last_logged_motor_send = None
        self._last_logged_motor_state = None
        
        # Speed change priority flag for immediate motor override
        self.speed_change_priority = False
        self.last_speed_change_time = 0
        
        # Motor command queue for immediate response
        self.motor_command_queue = []
        self.motor_command_lock = threading.Lock()
        
        # MPU6050 tilt protection
        self.mpu6050_bus = None
        self.mpu6050_connected = False
        self.initial_pitch = None  # Initial pitch angle (set as 0 reference)
        self.tilt_threshold = 5.0  # Degrees - tilt protection threshold (REDUCED for safety)
        self.tilt_detected = False
        self.tilt_recovery_time = 0
        self.tilt_cooldown = 3.0  # Seconds to wait after tilt recovery
        self.emergency_stop_active = False
        
        # Smooth acceleration parameters
        self.current_speed = 0.0  # Current smoothed speed
        self.current_steering = 0.0  # Current smoothed steering
        self.accel_rate = 20.0  # Units per second acceleration rate (increased for responsive acceleration)
        self.decel_rate = 0.3  # Units per second deceleration rate (reduced for gentler braking)
        # Follow-me must decelerate near-max or it coasts through the target distance
        self.follow_decel_rate = 350.0  # ~emergency_brake_rate; stop instead of overshoot
        # Follow-me accelerates hard for a clear walk-away command, but tapers
        # near the target so extra responsiveness does not create oscillation.
        self.follow_accel_rate = 110.0
        self.follow_accel_rate_near = 40.0
        self.follow_accel_full_speed = 35.0
        self.steer_rate = 200.0  # Units per second steering rate (INCREASED for responsive turning)
        self.steer_decel_rate = 1000.0  # Units per second steering deceleration (FAST release)
        # Follow-me: slower steer + larger deadzone so standstill pose jitter doesn't weave
        self.follow_steer_rate = 80.0
        self.follow_steering_deadzone = 0.14  # ~72/512 raw steering units
        self.follow_steer_gain = 0.65
        self.follow_pivot_steering_min = 120  # don't in-place pivot for tiny centering error
        self.last_control_time = time.time()
        
        # Firmware mode: 'speed' or 'torque'
        # In speed mode: negative speed = reverse, positive speed = forward
        # In torque mode: negative speed = reverse torque/braking, positive speed = forward torque
        self.firmware_mode = 'speed'  # Set to 'speed' or 'torque' based on firmware configuration
        
        # Speed and steering scaling
        # In speed mode, these are raw values sent directly to firmware (not scaled to 1000)
        # Original used max_speed=80, so 50 is already reduced
        self.max_speed = 80  # Manual/controller maximum speed
        self.follow_max_speed = 110  # Higher ceiling to keep pace while following
        self.max_steering = 120  # Maximum steering (matching original proportion)
        
        # Braking power control (0.0 = no braking, 1.0 = maximum braking)
        # Adjust this value to control braking strength:
        #   - 0.1 = very gentle coasting (minimal braking)
        #   - 0.3 = light braking (recommended for smooth operation)
        #   - 0.5 = moderate braking
        #   - 0.8 = strong braking
        #   - 1.0 = maximum braking (aggressive)
        self.braking_power = 0.15  # Adjust this to control braking strength (reduced from 0.3 for gentler braking)
        
        # Calculate brake parameters from braking_power
        self.brake_gain = 0.01 * self.braking_power  # Proportional braking strength (scales with braking_power)
        self.brake_max_ratio = 0.05 * self.braking_power  # Maximum braking as % of max_speed (scales with braking_power)
        self.brake_release_threshold = 20  # Do not command braking when nearly stopped
        self.direction_change_release_threshold = 3  # Consider near-zero speed when reversing
        self.direction_change_braking = False  # Hard braking when reversing direction
        self.direction_change_target_sign = 0  # Desired direction sign after braking completes
        self.neutral_hold_start = None  # Timestamp when controls entered neutral
        self.neutral_hold_duration = 0.4  # Seconds controls must stay neutral to re-zero smoothing
        
        # Emergency stop state machine
        self.emergency_stop_button_pressed = False
        self.emergency_braking_active = False
        self.emergency_locked = False
        self.emergency_lock_start_time = 0
        self.emergency_lock_duration = 3.0  # Seconds to keep wheels locked
        self.emergency_brake_rate = 400.0  # Fast deceleration without wheel lock
        self.last_emergency_button_state = 0
        
        # Reverse behavior: automatic based on cart motion state
        # No button needed - cart will reverse when joystick pulled back if not moving forward
        # If moving forward, will brake first, then reverse once stopped
        
        # Running state
        self.running = False
        
        # Watchdog for maintaining connection - ensure commands sent at minimum rate
        self.last_command_sent_time = 0
        self.min_command_interval = 0.05  # 20Hz minimum (50ms between commands)

        # Follow-me (phone app calibrates body/color; remote enables motors)
        self.last_follow_me_mode = 0
        self.calibration_beep_active = False
        self._last_follow_block_log = 0.0

        # Follow-me summon / pause (Button 3 — not the FOLLOW mode button):
        # active | summon_approach | summon_rotate | paused | resume_rotate
        self.follow_state = "active"
        self.last_summon_btn_pressed = False
        self.summon_tap_release_times = []
        self.summon_pending_single_tap_at = None
        self.summon_arrive_since = None
        self.summon_rotate_started_at = None
        self.summon_double_tap_window = 0.45
        self.summon_arrive_throttle = 100
        self.summon_arrive_steering = 150
        self.summon_arrive_hold_s = 0.5
        self.summon_approach_throttle = 220
        self.summon_rotate_steering = 380
        self.summon_rotate_duration_s = 2.4
        self.summon_rotate_timeout_s = 7.0
        self._rotate_states = ("summon_rotate", "resume_rotate")
        # Remote Button 3 bit (1<<2). Buttons 1/2 are mode + cruise.
        self.SUMMON_BUTTON_MASK = 0x04
        self.summon_btn_hold_start = None
        self.summon_btn_recalibrate_done = False
        
    def debug_print(self, message):
        """Print message only if debug mode is enabled and write to log file"""
        if self.debug_mode:
            timestamp = time.strftime("%H:%M:%S.%f")[:-3]  # Format: HH:MM:SS.mmm
            log_message = f"[{timestamp}] {message}\n"
            print(log_message.strip())
            
            # Write to log file with rotation
            try:
                log_file_path = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "hoverboard_debug.log"
                )
                max_log_size = 2 * 1024 * 1024  # 2MB max size
                
                # Check file size and rotate if needed
                if os.path.exists(log_file_path):
                    file_size = os.path.getsize(log_file_path)
                    if file_size >= max_log_size:
                        # Rotate: move current log to .old (overwrite if exists)
                        old_log_path = log_file_path + ".old"
                        if os.path.exists(old_log_path):
                            os.remove(old_log_path)
                        os.rename(log_file_path, old_log_path)
                
                with open(log_file_path, "a", encoding="utf-8") as log_file:
                    log_file.write(log_message)
                    log_file.flush()  # Ensure immediate write
            except Exception as e:
                # Don't fail if logging fails
                print(f"[ERROR] Failed to write to log file: {e}")
    
    def _is_excluded_usb_port(self, port) -> bool:
        """True for leftover XIAO/Espressif USB CDC that must not be the handheld receiver."""
        desc = (getattr(port, "description", None) or "").lower()
        hwid = (getattr(port, "hwid", None) or "").upper()
        if any(identifier in desc for identifier in ("xiao", "seeed")):
            return True
        if "303A:1001" in hwid or "VID:PID=303A" in hwid:
            return True
        if "jtag" in desc and "serial" in desc:
            return True
        return False

    def find_esp32_port(self):
        """Find ESP32 receiver port (handheld remote)."""
        # First try the persistent symlink (best practice - set this up!)
        if os.path.exists('/dev/esp32-receiver'):
            return '/dev/esp32-receiver'
        
        # Auto-detect: Find ESP32 receiver (exclude leftover camera boards)
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        receiver_ports = []
        excluded_ports = []
        
        for port in ports:
            if self._is_excluded_usb_port(port):
                excluded_ports.append(port.device)
                continue
            
            # Identify receiver ESP32 (CH340/CP2102 UART adapter)
            desc_lower = port.description.lower()
            if any(identifier in desc_lower for identifier in ['esp32', 'cp2102', 'cp210x', 'ch340', 'usb serial']):
                receiver_ports.append(port.device)
        
        if receiver_ports:
            if len(receiver_ports) > 1:
                self.debug_print(f"Found multiple ESP32 devices: {receiver_ports}")
                self.debug_print(f"Using first receiver: {receiver_ports[0]}")
            return receiver_ports[0]
        
        if excluded_ports:
            self.debug_print(f"WARNING: Skipped non-receiver USB serial on {excluded_ports}")
            self.debug_print("Make sure receiver ESP32 is connected and create symlink:")
            self.debug_print("  sudo ln -s /dev/ttyACM0 /dev/esp32-receiver  # Adjust port as needed")
        
        # Try common USB Serial ports
        for port in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            if os.path.exists(port):
                try:
                    import serial.tools.list_ports
                    all_ports = list(serial.tools.list_ports.comports())
                    for p in all_ports:
                        if p.device == port:
                            if self._is_excluded_usb_port(p):
                                continue
                            return port
                except Exception:
                    pass
                return port
        
        self.debug_print("WARNING: Could not find ESP32 receiver port!")
        self.debug_print("Please create symlink: sudo ln -s /dev/ttyACM0 /dev/esp32-receiver")
        return '/dev/ttyACM0'
    
    def connect_esp32(self):
        """Connect to ESP32 receiver (controller)"""
        try:
            self.esp32_port = self.find_esp32_port()
            # Don't reset ESP32 on connection by disabling DTR/RTS
            self.esp32_ser = serial.Serial(
                self.esp32_port, 
                115200, 
                timeout=1,
                dsrdtr=False,  # Don't use DSR/DTR flow control
                rtscts=False   # Don't use RTS/CTS flow control
            )
            # Explicitly disable DTR and RTS to prevent ESP32 reset
            self.esp32_ser.dtr = False
            self.esp32_ser.rts = False
            self.debug_print(f"✓ Connected to ESP32 receiver (controller) on {self.esp32_port}")
            return True
        except Exception as e:
            self.debug_print(f"✗ Failed to connect to ESP32 receiver: {e}")
            return False
    
    def connect_hoverboard(self):
        """Connect to hoverboard"""
        try:
            self.hoverboard_ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
            self.hoverboard_connected = True
            self.debug_print("✓ Connected to hoverboard")
            return True
        except Exception as e:
            self.debug_print(f"✗ Failed to connect to hoverboard: {e}")
            return False
    
    def connect_mpu6050(self):
        """Connect to MPU6050 via I2C"""
        try:
            # Try I2C bus 1 (default for Raspberry Pi)
            self.mpu6050_bus = smbus2.SMBus(1)
            
            # Test connection by reading WHO_AM_I register (0x75)
            who_am_i = self.mpu6050_bus.read_byte_data(0x68, 0x75)
            if who_am_i == 0x68:  # MPU6050 WHO_AM_I value
                self.mpu6050_connected = True
                self.debug_print("✓ Connected to MPU6050")
                
                # Initialize MPU6050
                self.initialize_mpu6050()
                return True
            else:
                self.debug_print(f"✗ MPU6050 WHO_AM_I mismatch: {hex(who_am_i)}")
                return False
        except Exception as e:
            self.debug_print(f"✗ Failed to connect to MPU6050: {e}")
            return False
    
    def initialize_mpu6050(self):
        """Initialize MPU6050 settings"""
        try:
            # Wake up MPU6050 (clear sleep bit)
            self.mpu6050_bus.write_byte_data(0x68, 0x6B, 0x00)
            time.sleep(0.1)
            
            # Set accelerometer range to ±2g (most sensitive for tilt detection)
            self.mpu6050_bus.write_byte_data(0x68, 0x1C, 0x00)
            time.sleep(0.1)
            
            # Set gyroscope range to ±250°/s
            self.mpu6050_bus.write_byte_data(0x68, 0x1B, 0x00)
            time.sleep(0.1)
            
            # Set DLPF (Digital Low Pass Filter) for smoother readings
            self.mpu6050_bus.write_byte_data(0x68, 0x1A, 0x06)
            time.sleep(0.1)
            
            self.debug_print("✓ MPU6050 initialized")
        except Exception as e:
            self.debug_print(f"✗ Failed to initialize MPU6050: {e}")
    
    def calibrate_mpu6050(self):
        """Calibrate MPU6050 - set current orientation as 0 degrees"""
        if not self.mpu6050_connected:
            return False
        
        try:
            # Take multiple readings for calibration
            readings = []
            for _ in range(50):
                accel_data = self.read_accelerometer()
                if accel_data:
                    readings.append(accel_data)
                time.sleep(0.01)
            
            if len(readings) < 10:
                self.debug_print("✗ Insufficient readings for calibration")
                return False
            
            # Calculate average accelerometer values
            avg_x = sum(r[0] for r in readings) / len(readings)
            avg_y = sum(r[1] for r in readings) / len(readings)
            avg_z = sum(r[2] for r in readings) / len(readings)
            
            # Calculate initial pitch angle
            self.initial_pitch = math.atan2(avg_y, math.sqrt(avg_x**2 + avg_z**2)) * 180.0 / math.pi
            
            self.debug_print(f"✓ MPU6050 calibrated - Initial pitch: {self.initial_pitch:.1f}°")
            return True
            
        except Exception as e:
            self.debug_print(f"✗ Failed to calibrate MPU6050: {e}")
            return False
    
    def read_accelerometer(self):
        """Read accelerometer data from MPU6050"""
        if not self.mpu6050_connected:
            return None
        
        try:
            # Read accelerometer data (6 bytes: X, Y, Z each 2 bytes)
            data = self.mpu6050_bus.read_i2c_block_data(0x68, 0x3B, 6)
            
            # Convert raw data to g-forces
            accel_x = self.convert_accel_data(data[0], data[1])
            accel_y = self.convert_accel_data(data[2], data[3])
            accel_z = self.convert_accel_data(data[4], data[5])
            
            return (accel_x, accel_y, accel_z)
        except Exception as e:
            self.debug_print(f"✗ Failed to read accelerometer: {e}")
            return None
    
    def convert_accel_data(self, high_byte, low_byte):
        """Convert raw accelerometer data to g-forces"""
        # Combine high and low bytes
        raw_value = (high_byte << 8) | low_byte
        
        # Convert to signed 16-bit
        if raw_value > 32767:
            raw_value -= 65536
        
        # Convert to g-forces (±2g range)
        return raw_value / 16384.0

    def get_tilt_angle(self):
        """Get current tilt angle relative to initial position"""
        if not self.mpu6050_connected or self.initial_pitch is None:
            return 0
        
        accel_data = self.read_accelerometer()
        if not accel_data:
            return 0
        
        accel_x, accel_y, accel_z = accel_data
        
        # Calculate current pitch angle
        current_pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180.0 / math.pi
        
        # Return tilt relative to initial position
        tilt_angle = current_pitch - self.initial_pitch
        return tilt_angle
    
    def check_tilt_safety(self):
        """Check if cart is tilting dangerously and handle safety measures"""
        if not self.mpu6050_connected:
            return True  # Allow operation if MPU6050 not connected
        
        tilt_angle = self.get_tilt_angle()
        
        # Check if tilting backward beyond threshold
        if tilt_angle > self.tilt_threshold:
            if not self.tilt_detected:
                self.tilt_detected = True
                self.emergency_stop_active = True
                self.debug_print(f"🚨 TILT DANGER! Angle: {tilt_angle:.1f}° (threshold: {self.tilt_threshold}°)")
                self.debug_print("🚨 EMERGENCY STOP ACTIVATED - Cart tilting backward!")
            
            return False  # Unsafe to operate
        
        # Check if recovered from tilt
        elif self.tilt_detected and tilt_angle < 10:  # 10 degree safety margin
            if not self.tilt_recovery_time:
                self.tilt_recovery_time = time.time()
                self.debug_print(f"✓ Tilt recovered - Angle: {tilt_angle:.1f}°")
                self.debug_print(f"⏳ Safety cooldown: {self.tilt_cooldown}s")
            
            # Check if cooldown period has passed
            if time.time() - self.tilt_recovery_time > self.tilt_cooldown:
                self.tilt_detected = False
                self.emergency_stop_active = False
                self.tilt_recovery_time = 0
                self.debug_print("✓ Safety cooldown complete - Normal operation resumed")
        
        return True  # Safe to operate
    

            
    def send_hoverboard_command(self, steer, speed):
        """Send command to hoverboard"""
        if not self.hoverboard_connected:
            return False
            
        # Clamp to protocol range; ints required for XOR checksum and struct.pack('h')
        steer = int(max(-32767, min(32767, steer)))
        speed = int(max(-32767, min(32767, speed)))
        
        # Use the correct start frame that the hoverboard expects
        start_frame = 0xABCD  # Original hoverboard protocol start frame
        checksum = start_frame ^ steer ^ speed  # Correct checksum from official protocol
        
        # Clamp checksum to valid range for unsigned short
        checksum = checksum & 0xFFFF  # Keep only 16 bits
        
        # Pack: start_frame (H), steer (h), speed (h), checksum (H)
        # Note: 0xABCD (43981) fits in unsigned short range (0-65535)
        command = struct.pack('<HhhH', start_frame, steer, speed, checksum)
        
        # DETAILED LOGGING: log motor commands only when values change
        if self.debug_mode:
            motor_key = (steer, speed)
            if motor_key != self._last_logged_motor_send:
                self._last_logged_motor_send = motor_key
                reverse_info = ""
                if self.firmware_mode == 'speed' and speed < 0:
                    reverse_info = f" [REVERSE, throttle={self.throttle}, current_speed={self.current_speed:.1f}]"
                self.debug_print(
                    f"📤 SEND: Steer={steer}, Speed={speed}, "
                    f"Checksum=0x{checksum:04X}{reverse_info}"
                )
        
        try:
            self.hoverboard_ser.write(command)
            self.hoverboard_ser.flush()
            
            # Try to read any response from hoverboard (if it sends feedback)
            if self.hoverboard_ser.in_waiting > 0:
                response = self.hoverboard_ser.read(self.hoverboard_ser.in_waiting)
                if self.debug_mode and len(response) > 0:
                    self.debug_print(f"📥 RECV: {len(response)} bytes: {response.hex()}")
            
            # Track last sent values for watchdog
            self.last_steer = steer
            self.last_speed = speed
            self.last_command_sent_time = time.time()
            return True
        except (serial.SerialException, OSError, IOError) as e:
            # Serial port error - try to reconnect
            if self.debug_mode:
                self.debug_print(f"❌ SERIAL ERROR: {e} - Attempting to reconnect...")
            self.hoverboard_connected = False
            # Try to reconnect
            if self.connect_hoverboard():
                # Retry sending the command once after reconnection
                try:
                    self.hoverboard_ser.write(command)
                    self.hoverboard_ser.flush()
                    if self.debug_mode:
                        self.debug_print(f"✓ Reconnected and command sent")
                    return True
                except Exception as retry_e:
                    if self.debug_mode:
                        self.debug_print(f"❌ Retry failed: {retry_e}")
                    return False
            else:
                if self.debug_mode:
                    self.debug_print(f"❌ Reconnection failed")
                return False
        except Exception as e:
            if self.debug_mode:
                self.debug_print(f"❌ SEND ERROR: {e}")
            return False
    
    def _extract_packet(self, data):
        """Validate packet framing and return payload bytes."""
        if len(data) < 8 or data[0] != 0xAA or data[-1] != 0xBB:
            return None

        data_bytes = data[1:-1]
        if len(data_bytes) != 13:
            if self.debug_mode:
                self.debug_print(
                    f"⚠️ Packet size mismatch: Expected 13 bytes, got {len(data_bytes)} bytes. "
                    f"Raw data: {[hex(b) for b in data_bytes[:15]]}"
                )
            return None

        return data_bytes

    def _handle_summon_button_long_press(self):
        """Remind user that re-calibration is done in the Cart Follow phone app."""
        pressed = bool(self.button_states & self.SUMMON_BUTTON_MASK)

        if self.follow_me_mode != 1 or self.follow_state in (
            "summon_approach",
            *self._rotate_states,
        ):
            self.summon_btn_hold_start = None
            self.summon_btn_recalibrate_done = False
            return

        if pressed:
            now = time.time()
            if self.summon_btn_hold_start is None:
                self.summon_btn_hold_start = now
            elif (
                now - self.summon_btn_hold_start >= 5.0
                and not self.summon_btn_recalibrate_done
            ):
                self.debug_print("📱 Re-calibrate from the Cart Follow app (not the remote)")
                self.summon_btn_recalibrate_done = True
        else:
            self.summon_btn_hold_start = None
            self.summon_btn_recalibrate_done = False

    def _start_summon(self):
        """Begin summon: approach user, rotate 180°, then pause follow-me."""
        if self.follow_me_mode != 1:
            return
        if self.follow_state in ("summon_approach", *self._rotate_states):
            return

        self.follow_state = "summon_approach"
        self.summon_arrive_since = None
        self.summon_rotate_started_at = None
        self.summon_pending_single_tap_at = None
        self.summon_tap_release_times = []
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.debug_print("📍 Summon started - cart approaching")
        if self.phone_gps and self.phone_gps.is_fresh():
            from pathlib import Path

            round_dir = Path.home() / "golf_rounds"
            log_path = round_dir / f"{time.strftime('%Y-%m-%d')}.jsonl"
            self.phone_gps.record_summon_fix(log_path, event="summon_start")
            self.debug_print(
                f"📍 Summon GPS logged ({self.phone_gps.lat:.6f}, {self.phone_gps.lon:.6f})"
            )

    def _resume_follow_from_pause(self):
        """After summon pause: rotate 180° again so you are in frame, then follow."""
        if self.follow_state != "paused":
            return
        self.follow_state = "resume_rotate"
        self.summon_arrive_since = None
        self.summon_rotate_started_at = time.time()
        self.summon_pending_single_tap_at = None
        self.summon_tap_release_times = []
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.debug_print(
            "🔄 Resume rotate — facing camera toward you, then follow-me"
        )

    def _handle_summon_button_taps(self):
        """
        Button 3 gestures while in follow mode (avoids FOLLOW mode Button 1):
          double tap  -> summon
          single tap  -> resume rotate + follow (when paused)
          long press  -> handled by _handle_summon_button_long_press
        """
        pressed = bool(self.button_states & self.SUMMON_BUTTON_MASK)
        if self.follow_me_mode != 1:
            self.last_summon_btn_pressed = pressed
            self.summon_tap_release_times = []
            self.summon_pending_single_tap_at = None
            return

        now = time.time()

        if not pressed and self.last_summon_btn_pressed:
            self.summon_tap_release_times.append(now)
            self.summon_tap_release_times = [
                t for t in self.summon_tap_release_times
                if now - t <= 1.0
            ]

            if len(self.summon_tap_release_times) >= 2:
                gap = (
                    self.summon_tap_release_times[-1]
                    - self.summon_tap_release_times[-2]
                )
                if gap <= self.summon_double_tap_window:
                    self._start_summon()
                    self.summon_tap_release_times = []
                    self.summon_pending_single_tap_at = None
                elif self.follow_state == "paused":
                    self.summon_pending_single_tap_at = now
            elif self.follow_state == "paused":
                self.summon_pending_single_tap_at = now

        self.last_summon_btn_pressed = pressed

        if (
            self.summon_pending_single_tap_at is not None
            and self.follow_state == "paused"
            and now - self.summon_pending_single_tap_at >= self.summon_double_tap_window
            and len(self.summon_tap_release_times) == 1
        ):
            self._resume_follow_from_pause()
            self.summon_pending_single_tap_at = None
            self.summon_tap_release_times = []

    def _apply_summon_approach(self):
        """Drive toward the user using camera steering; stop when LiDAR says we're close."""
        steering = max(-512, min(512, self.camera_steering))
        throttle = max(self.summon_approach_throttle, self.camera_throttle)
        throttle = max(0, min(512, throttle))

        centered = abs(steering) <= self.summon_arrive_steering
        close_enough = 0 <= self.camera_throttle <= self.summon_arrive_throttle

        if centered and close_enough:
            if self.summon_arrive_since is None:
                self.summon_arrive_since = time.time()
            elif time.time() - self.summon_arrive_since >= self.summon_arrive_hold_s:
                self.follow_state = "summon_rotate"
                self.summon_rotate_started_at = time.time()
                self.debug_print("🔄 Summon rotate - timed 180° turn for club access")
                throttle = 0
                steering = 0
        else:
            self.summon_arrive_since = None

        self.throttle = throttle
        self.steering = steering

    def _apply_summon_rotate(self):
        """In-place 180° turn at medium rate (fixed duration, no IMU)."""
        now = time.time()
        if self.summon_rotate_started_at is None:
            self.summon_rotate_started_at = now

        elapsed = now - self.summon_rotate_started_at
        rotation_done = elapsed >= self.summon_rotate_duration_s
        timed_out = elapsed >= self.summon_rotate_timeout_s

        if rotation_done or timed_out:
            self.throttle = 0
            self.steering = 0
            self.current_speed = 0.0
            self.current_steering = 0.0
            self.summon_rotate_started_at = None
            if self.follow_state == "resume_rotate":
                self.follow_state = "active"
                self.play_calibration_beep()
                self.debug_print("▶️ Resume rotate done — follow-me active")
            else:
                self.follow_state = "paused"
                self.play_calibration_beep()
                self.debug_print(
                    "⏸️ Summon complete - paused. "
                    "Single tap Button 3 to rotate back and resume follow"
                )
            return

        self.throttle = 0
        self.steering = self.summon_rotate_steering

    def _reset_follow_state(self):
        """Clear summon/pause state when leaving follow mode."""
        self.follow_state = "active"
        self.summon_arrive_since = None
        self.summon_rotate_started_at = None
        self.summon_tap_release_times = []
        self.summon_pending_single_tap_at = None

    def _handle_phone_status_line(self, line):
        """ASCII lines from the phone app (calibration, GPS, hole)."""
        if self.phone_gps and self.phone_gps.parse_line(line):
            if line.startswith("HOLE "):
                self.debug_print(f"⛳ Hole {self.phone_gps.hole}")
            elif line.startswith("HEADING "):
                brg = self.phone_gps.course_bearing_deg
                spd = self.phone_gps.speed_mps
                if brg is not None and self.phone_gps.heading_fresh():
                    self.debug_print(f"🧭 Phone heading brg={brg:.0f}° spd={spd:.1f}m/s")
            return
        self._handle_calib_status_line(line)

    def _handle_calib_status_line(self, line):
        """Handle ASCII calibration status lines from the phone app."""
        if line.startswith("CALIB_OK"):
            self.debug_print(f"✓ Body calibration OK ({line[9:].strip() or 'complete'})")
            # Motor chirp can glitch USB tether power — skip it.
            self.debug_print("📱 Calibration complete (motor chirp skipped)")
        elif line == "CALIB_START":
            self.debug_print("🎨 Body calibration starting — stand centered in camera view")
        elif line.startswith("CALIB ") and line[6:].strip().isdigit():
            self.debug_print(f"🎨 Calibration countdown: {line[6:].strip()}")
        elif line == "CALIB_FAIL":
            self.debug_print("✗ Body calibration failed - try again")

    def play_calibration_beep(self):
        """
        Audible confirmation after successful calibration.

        The stock hoverboard firmware does not expose a serial beep command, so this
        uses rapid steer pulses at zero speed to chirp the motor controllers.
        """
        if not self.hoverboard_connected or self.calibration_beep_active:
            return

        def chirp():
            self.calibration_beep_active = True
            try:
                for _ in range(3):
                    self.send_hoverboard_command(220, 0)
                    time.sleep(0.07)
                    self.send_hoverboard_command(-220, 0)
                    time.sleep(0.07)
                self.send_hoverboard_command(0, 0)
                self.debug_print("🔔 Calibration confirmation chirp")
            finally:
                self.calibration_beep_active = False

        threading.Thread(target=chirp, daemon=True).start()

    def _sync_phone_lidar_motion(self):
        """Map phone steering + fused LiDAR/phone distance into camera_*."""
        if not self.phone_bridge:
            self.camera_steering = 0
            self.camera_throttle = 0
            self.distance_source = "none"
            return

        detected = (
            self.phone_bridge.person_detected
            and not self.phone_bridge.is_stale()
            and not self.phone_bridge.calibrating
        )
        if not detected:
            self.camera_steering = 0
            self.camera_throttle = 0
            self.distance_source = "none"
            self._phone_too_close = False
            return

        self.camera_steering = self.phone_bridge.get_steering()
        phone_throttle = self.phone_bridge.get_throttle()

        if self.lidar_distance_cm > 0:
            # LiDAR owns the absolute distance state. Phone body size can refine
            # throttle only after LiDAR has selected catch-up or backup; it must
            # not bypass the requested 2.2/2.35/2.5 m boundaries.
            self._phone_too_close = False
            lidar_throttle = self.lidar_throttle
            fused = self._fuse_lidar_phone_throttle(lidar_throttle, phone_throttle)
            motion_state = getattr(self.matrix_lidar, "motion_state", "lost")
            if motion_state == "hold":
                # LiDAR neutral band — but if phone clearly says too close, reverse
                # anyway (LiDAR empty_band / bias often delays metric backup).
                if phone_throttle <= -25:
                    fused = self._phone_only_throttle(phone_throttle)
                else:
                    fused = 0
            elif motion_state == "catch_up":
                fused = max(0, fused)
            elif motion_state == "backup":
                fused = min(0, fused)
            self.camera_throttle = fused
            self.distance_source = "fused" if phone_throttle else "lidar"
        else:
            # LiDAR lost: phone body-size with a latched "too close" reverse
            # signal (same idea as sustained forward catch-up).
            self.camera_throttle = self._phone_only_throttle(phone_throttle)
            self.distance_source = "phone" if self.camera_throttle != 0 else "none"

    def _phone_only_throttle(self, phone_throttle: int) -> int:
        """Phone body-size throttle with latched reverse while LiDAR is lost.

        Forward already stays on while the torso looks small. Reverse used to
        drop out whenever bodyFrac flickered through the deadzone — latch
        "too close" until the phone clearly reports too-far / catch-up.
        """
        floor = 140  # match LiDAR MIN_THROTTLE so reverse clears motor deadzone
        enter_rev = -25  # latch from a weak "too close" so reverse starts sooner
        exit_rev = 50  # need a clear "too far" before releasing reverse

        if phone_throttle <= enter_rev:
            self._phone_too_close = True
        elif phone_throttle >= exit_rev:
            self._phone_too_close = False

        if self._phone_too_close:
            # Hold decisive reverse even if phoneT is briefly 0 in the deadzone.
            if phone_throttle < 0:
                return max(-512, min(-floor, phone_throttle))
            return -floor

        if phone_throttle == 0:
            return 0
        if phone_throttle > 0 and abs(phone_throttle) < floor and abs(phone_throttle) >= 40:
            return floor
        return max(-512, min(512, phone_throttle))

    @staticmethod
    def _fuse_lidar_phone_throttle(lidar_throttle: int, phone_throttle: int) -> int:
        """Blend range (LiDAR) with body-size (phone) for smoother speed matching."""
        max_t = 512
        if phone_throttle == 0:
            return max(-max_t, min(max_t, lidar_throttle))

        # Vision lead: at target per LiDAR but body shrinking/growing → start moving
        if lidar_throttle == 0 and abs(phone_throttle) >= 40:
            fused = int(phone_throttle * 0.6)
        elif (lidar_throttle > 0) == (phone_throttle > 0) or lidar_throttle == 0:
            # Same direction (or LiDAR neutral): trust blend, lean LiDAR
            fused = int(0.7 * lidar_throttle + 0.3 * phone_throttle)
        else:
            # Conflict: prefer LiDAR (metric), keep a little phone damping
            fused = int(0.85 * lidar_throttle + 0.15 * phone_throttle)

        return max(-max_t, min(max_t, fused))

    def _log_pi_network_addresses(self):
        """Print Pi IPs so USB-tether address is obvious in the log."""
        try:
            import subprocess

            ips = subprocess.check_output(["hostname", "-I"], text=True, timeout=2).strip()
            if ips:
                self.debug_print(f"📡 Pi addresses: {ips}")
        except Exception:
            pass

    def start_phone_lidar_follow(self):
        """Start TCP bridge for phone steering and I2C LiDAR for distance."""
        from pathlib import Path

        course_cache = os.environ.get("GOLF_COURSE_CACHE", "")
        if not course_cache:
            default = Path(__file__).resolve().parent / "golf_courses" / "oak_creek_irvine.json"
            if default.exists():
                course_cache = str(default)
        default_hole = int(os.environ.get("GOLF_HOLE", "1"))
        self.phone_gps = PhoneGps(
            course_cache=course_cache or None,
            default_hole=default_hole,
        )
        if course_cache:
            self.debug_print(f"⛳ Golf yardages: {course_cache} (hole {default_hole})")
        else:
            self.debug_print(
                "⛳ Phone GPS ready — set GOLF_COURSE_CACHE for F/M/B yardages"
            )

        self.phone_bridge = PhoneBridge(
            port=PHONE_BRIDGE_PORT,
            log_cb=lambda msg: self.debug_print(f"📱 {msg}"),
            status_cb=self._handle_phone_status_line,
        )
        self.phone_bridge.start()
        self._log_pi_network_addresses()
        self.debug_print(
            f"✓ Phone bridge started on 0.0.0.0:{PHONE_BRIDGE_PORT} "
            f"(enable USB tethering on phone, then Scan & Connect)"
        )

        # LiDAR init can take 10–30s; never block startup or phone packet handling.
        self.matrix_lidar = None

        def lidar_worker():
            sensor = MatrixLidarFollow()
            self.debug_print("📏 LiDAR: connecting on USB-C (background)…")
            if sensor.connect():
                self.matrix_lidar = sensor
                if sensor.i2c_addr:
                    self.debug_print(
                        f"✓ Matrix LiDAR ready (I2C 0x{sensor.i2c_addr:02X}) "
                        "reverse<220cm forward>250cm max-gap≈350cm"
                    )
                else:
                    self.debug_print(
                        f"✓ Matrix LiDAR ready (USB {sensor.port}) "
                        "reverse<220cm forward>250cm max-gap≈350cm"
                    )
            else:
                self.debug_print(f"✗ Matrix LiDAR: {sensor.last_error}")
                self.debug_print(
                    "   Using phone body-size for distance until LiDAR is fixed."
                )
                self.debug_print(
                    "   Tip: sudo ln -sf /dev/ttyACM# /dev/matrix-lidar  (see ls /dev/ttyACM*)"
                )

        threading.Thread(target=lidar_worker, daemon=True).start()

        self._lidar_thread = threading.Thread(target=self._phone_follow_loop, daemon=True)
        self._lidar_thread.start()
        return True

    def _phone_follow_loop(self):
        """Poll LiDAR + phone bridge and drive follow-me at ~10 Hz."""
        while self.running:
            # Safety: drop follow instantly if phone/calib goes away mid-session.
            if self.follow_me_mode == 1 and not self._follow_calibration_ready():
                self.follow_me_mode = 0
                self.last_follow_me_mode = 0
                self._reset_follow_state()
                self.camera_steering = 0
                self.camera_throttle = 0
                self.debug_print(
                    f"🎮 Follow-me disarmed ({self._follow_block_reason()})"
                )

            if self.matrix_lidar:
                self.lidar_distance_cm = self.matrix_lidar.read_distance_cm()
                self.lidar_throttle = self.matrix_lidar.throttle
            else:
                self.lidar_distance_cm = 0
                self.lidar_throttle = 0

            self._sync_phone_lidar_motion()
            if self.follow_me_mode == 1 and self.follow_state in (
                "active",
                "summon_approach",
                "summon_rotate",
                "resume_rotate",
            ):
                self._apply_active_control()

            if self.debug_mode:
                snap = self.phone_bridge.snapshot() if self.phone_bridge else {}
                status_key = (
                    snap.get("connected"),
                    self.follow_me_mode,
                    self.camera_steering,
                    self.camera_throttle,
                    self.distance_source,
                    self.lidar_distance_cm // 10 if self.lidar_distance_cm else 0,
                    getattr(self.matrix_lidar, "motion_state", "lost"),
                )
                if status_key != self._last_logged_phone_status:
                    self._last_logged_phone_status = status_key
                    lidar_txt = (
                        f"{self.lidar_distance_cm}cm T={self.lidar_throttle}"
                        if self.lidar_distance_cm > 0
                        else "no-reading"
                    )
                    if (
                        self.matrix_lidar
                        and self.lidar_distance_cm > 0
                        and getattr(self.matrix_lidar, "reading_held", False)
                    ):
                        lidar_txt += " sample-held"
                    if self.matrix_lidar and self.lidar_distance_cm <= 0:
                        miss = getattr(self.matrix_lidar, "miss_reason", "") or "?"
                        age = getattr(self.matrix_lidar, "frame_age_s", 0.0)
                        lidar_txt += f" miss={miss} age={age:.2f}s"
                    if self.matrix_lidar:
                        lidar_txt += (
                            f" state={getattr(self.matrix_lidar, 'motion_state', 'lost')}"
                        )
                    follow_txt = (
                        "ON" if self.follow_me_mode == 1 else "OFF (enable on remote)"
                    )
                    self.debug_print(
                        f"📏 follow={follow_txt} steering={self.camera_steering} "
                        f"throttle={self.camera_throttle} dist={self.distance_source} "
                        f"(lidar {lidar_txt}, phoneT={snap.get('phone_throttle', 0)}) "
                        f"| phone {'connected' if snap.get('connected') else 'waiting'} "
                        f"det={snap.get('person_detected', False)}"
                    )
            if self.phone_gps and self.phone_bridge and self.phone_bridge.connected:
                self.phone_gps.maybe_log_yardages(self.debug_print)
            time.sleep(0.1)

    def _apply_active_control(self):
        """Use camera motion when follow-me is enabled, otherwise use the remote."""
        if self.follow_me_mode == 1:
            if self.follow_state == "active":
                self.throttle = self.camera_throttle
                # Soften phone steering; ignore tiny centering noise at standstill
                steer = int(self.camera_steering * self.follow_steer_gain)
                if (
                    abs(self.camera_throttle) <= 30
                    and abs(steer) < self.follow_pivot_steering_min
                ):
                    steer = 0
                self.steering = steer
            elif self.follow_state == "summon_approach":
                self._apply_summon_approach()
            elif self.follow_state in self._rotate_states:
                self._apply_summon_rotate()
            else:
                self.throttle = 0
                self.steering = 0
        else:
            self.throttle = self.controller_throttle
            self.steering = self.controller_steering

    def _follow_calibration_ready(self) -> bool:
        """Follow-me motors require a connected phone with successful calibration."""
        bridge = self.phone_bridge
        if bridge is None or not bridge.connected:
            return False
        if bridge.calibrating:
            return False
        return bool(bridge.calib_ok)

    def _follow_block_reason(self) -> str:
        bridge = self.phone_bridge
        if bridge is None or not bridge.connected:
            return "phone not connected"
        if bridge.calibrating:
            return "calibration in progress"
        if not bridge.calib_ok:
            return "not calibrated — open Cart Follow and tap Calibrate"
        return "unknown"

    def parse_receiver_data(self, data):
        """Parse packets from the ESP-NOW receiver ESP32."""
        try:
            data_bytes = self._extract_packet(data)
            if data_bytes is None:
                return False

            throttle, steering, emergency_stop, cruise_control, cruise_speed, follow_me_mode, turbo_mode, battery_level, button_states = struct.unpack('<hhBBhBBBB', data_bytes[:12])

            self.controller_throttle = throttle
            self.controller_steering = steering
            self.emergency_stop = emergency_stop
            self.cruise_control = cruise_control
            self.cruise_speed = cruise_speed
            self.turbo_mode = turbo_mode
            self.battery_level = battery_level
            self.button_states = button_states

            # Remote FOLLOW request is ignored until phone reports cal=ok.
            requested = 1 if follow_me_mode == 1 else 0
            if requested == 1 and not self._follow_calibration_ready():
                effective = 0
                now = time.time()
                if now - getattr(self, "_last_follow_block_log", 0) >= 3.0:
                    self._last_follow_block_log = now
                    self.debug_print(
                        f"🚫 Follow-me blocked ({self._follow_block_reason()}) "
                        "— remote stays in joystick control"
                    )
            else:
                effective = requested

            prev_follow_me_mode = self.follow_me_mode
            if prev_follow_me_mode != effective:
                if effective == 1:
                    self.debug_print(
                        "🎯 Follow-me ACTIVATED — phone steering + Pi LiDAR distance"
                    )
                else:
                    if requested == 0:
                        self.debug_print(
                            "🎮 Follow-me mode DEACTIVATED - Using controller input"
                        )
                    else:
                        self.debug_print(
                            "🎮 Follow-me disarmed — using controller until calibrated"
                        )
                    self._reset_follow_state()
            self.last_follow_me_mode = effective
            self.follow_me_mode = effective

            self._handle_summon_button_taps()
            self._handle_summon_button_long_press()
            self._apply_active_control()
            return True

        except Exception as e:
            if self.debug_mode:
                self.debug_print(f"❌ Receiver parse exception: {type(e).__name__}: {e}")
            return False

    def parse_data(self, data):
        """Backward-compatible parser entry point."""
        return self.parse_receiver_data(data)
    
    def read_data_loop(self):
        """Read data from ESP32 receiver (handheld remote)."""
        receiver_buffer = b''
        
        while self.running:
            try:
                # Read from receiver ESP32 (controller)
                if self.esp32_ser and self.esp32_ser.in_waiting > 0:
                    data = self.esp32_ser.read(self.esp32_ser.in_waiting)
                    if data:
                        receiver_buffer += data
                        
                        # Look for packets
                        while len(receiver_buffer) >= 8:  # Minimum packet size
                            # Find start byte
                            start_idx = receiver_buffer.find(b'\xAA')
                            if start_idx == -1:
                                receiver_buffer = b''
                                break
                            
                            # Find end byte
                            end_idx = receiver_buffer.find(b'\xBB', start_idx)
                            if end_idx == -1:
                                break
                            
                            # Extract packet
                            packet = receiver_buffer[start_idx:end_idx + 1]
                            receiver_buffer = receiver_buffer[end_idx + 1:]
                            
                            # Parse packet (from receiver ESP32 - controller)
                            if self.parse_receiver_data(packet):
                                if hasattr(self, 'last_cruise_control') and self.cruise_control != self.last_cruise_control:
                                    self.debug_print(f"🚨 CRUISE CONTROL STATE CHANGE: {self.last_cruise_control} -> {self.cruise_control}")
                                    if not self.cruise_control:
                                        self.debug_print("🚨 CRUISE CONTROL DISABLED - Processing immediately for safety!")
                                self.last_cruise_control = self.cruise_control

                                if hasattr(self, 'last_cruise_speed') and self.cruise_speed != self.last_cruise_speed:
                                    self.debug_print(f"🚗 CRUISE SPEED CHANGED: {self.last_cruise_speed} -> {self.cruise_speed}")
                                    self.speed_change_priority = True
                                    self.last_speed_change_time = time.time()

                                    if self.hoverboard_connected:
                                        throttle_norm = self.cruise_speed / 512.0
                                        steering_norm = self.steering / 512.0

                                        deadzone = 0.05
                                        if abs(throttle_norm) < deadzone:
                                            throttle_norm = 0
                                        if abs(steering_norm) < deadzone:
                                            steering_norm = 0

                                        scaled_speed = throttle_norm * self.max_speed
                                        scaled_steering = steering_norm * self.max_steering
                                        left_speed = int(scaled_speed + scaled_steering)
                                        right_speed = int(scaled_speed - scaled_steering)
                                        steer = (left_speed - right_speed) // 2
                                        speed = (left_speed + right_speed) // 2
                                        steer = max(-300, min(300, steer))
                                        speed = max(-150, min(150, speed))

                                        with self.motor_command_lock:
                                            self.motor_command_queue.append((steer, speed))
                                        self.debug_print(f"🚀 MOTOR COMMAND QUEUED: Steer={steer}, Speed={speed}")

                                    self.control_hoverboard()
                                self.last_cruise_speed = self.cruise_speed

                                if not self.cruise_control and not self.follow_me_mode:
                                    self.debug_print(
                                        f"✓ Receiver: Throttle={self.throttle}, Steering={self.steering}, "
                                        f"Emergency={self.emergency_stop}, Cruise={self.cruise_control}, "
                                        f"CruiseSpeed={self.cruise_speed}, FollowMe={self.follow_me_mode}, "
                                        f"Turbo={self.turbo_mode}, Battery={self.battery_level}"
                                    )
                
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
        if self.calibration_beep_active:
            return

        # CRITICAL: Check tilt safety first - override all other controls if unsafe
        if not self.check_tilt_safety():
            # Emergency stop due to tilt danger
            if self.hoverboard_connected:
                self.send_hoverboard_command(0, 0)  # Emergency stop
                if self.debug_mode:
                    self.debug_print("🚨 EMERGENCY STOP: Tilt safety override active")
            return
        
        # Check if emergency stop is active (from tilt recovery cooldown)
        if self.emergency_stop_active:
            if self.hoverboard_connected:
                self.send_hoverboard_command(0, 0)  # Emergency stop
                if self.debug_mode:
                    self.debug_print("🚨 EMERGENCY STOP: Safety cooldown active")
            return
        
        # EMERGENCY STOP BUTTON LOGIC - INSTANT STOP
        # Detect button press (rising edge)
        if self.emergency_stop == 1 and self.last_emergency_button_state == 0:
            # Button just pressed - toggle emergency stop state
            self.emergency_stop_button_pressed = not self.emergency_stop_button_pressed
            
            if self.emergency_stop_button_pressed:
                # INSTANT STOP - immediately zero out speed and lock wheels
                self.emergency_locked = True
                self.emergency_lock_start_time = time.time()
                self.current_speed = 0.0
                self.current_steering = 0.0
                self.debug_print("🚨 EMERGENCY STOP! Wheels locked immediately")
                
                # Send immediate stop command
                if self.hoverboard_connected:
                    self.send_hoverboard_command(0, 0)
            else:
                # Button pressed again - release lock
                self.emergency_locked = False
                self.current_speed = 0.0
                self.current_steering = 0.0
                self.debug_print("✓ Emergency stop released - Normal operation resumed")
        
        self.last_emergency_button_state = self.emergency_stop
        
        # Handle emergency locked state
        if self.emergency_stop_button_pressed and self.emergency_locked:
            # WHEEL LOCK MODE - send continuous zero commands
            if self.hoverboard_connected:
                self.send_hoverboard_command(0, 0)
            
            # Keep locked until button is pressed again
            if self.debug_mode and int(time.time()) % 5 == 0:
                self.debug_print("🔒 LOCKED - Press emergency stop button to release")
            
            # Emergency stop active - don't process normal controls
            return
        
        # REVERSE BEHAVIOR: Automatic based on cart motion
        # No button needed - behavior is automatic:
        #   - If moving forward and joystick pulled back: brake until stopped, then reverse
        #   - If stopped/backward and joystick pulled back: reverse immediately
        
        # Check if cruise control is enabled
        if self.cruise_control:
            # Use cruise control speed instead of joystick throttle
            throttle_norm = self.cruise_speed / 512.0
            self.debug_print(f"🚗 Cruise control ACTIVE - Speed: {self.cruise_speed}, Normalized: {throttle_norm:.3f}")
        else:
            # Use normal joystick throttle
            throttle_norm = self.throttle / 512.0
            
                    # Track cruise control state changes for debugging
            if hasattr(self, 'last_cruise_state') and self.last_cruise_state != self.cruise_control:
                self.debug_print(f"🚨 CRUISE CONTROL STATE CHANGE: {self.last_cruise_state} -> {self.cruise_control}")
            self.last_cruise_state = self.cruise_control
            
        steering_norm = self.steering / 512.0
        speed_limit = (
            self.follow_max_speed
            if self.follow_me_mode == 1
            else self.max_speed
        )
        
        # In speed mode:
        #   - Positive throttle = forward speed
        #   - Negative throttle while moving forward = braking (handled in braking section)
        #   - Negative throttle when stopped/backward = reverse speed
        # Behavior is automatic based on current motion state
        
        # Apply joystick throttle deadzone only in manual mode. Follow-me
        # throttle is deliberate LiDAR/phone controller output and already has
        # its own hold state and minimum-command threshold.
        throttle_deadzone = 0.10
        steering_deadzone = (
            self.follow_steering_deadzone
            if self.follow_me_mode == 1
            else 0.06
        )
        if (
            self.follow_me_mode != 1
            and not self.cruise_control
            and abs(throttle_norm) < throttle_deadzone
        ):
            throttle_norm = 0
        if abs(steering_norm) < steering_deadzone:
            steering_norm = 0
        
        # Scale speed and steering separately for better control
        # Apply turbo mode scaling if enabled
        if self.turbo_mode:
            speed_multiplier = 1.5  # 50% increase in turbo mode
            steering_multiplier = 1.2  # 20% increase in steering responsiveness
        else:
            speed_multiplier = 1.0
            steering_multiplier = 1.0
            
        # Speed mode: Allow negative throttle - behavior depends on current motion state
        # If moving forward, negative throttle will be handled as braking in braking section
        # If stopped/backward, negative throttle will create reverse speed
        # Don't block negative throttle here - let braking section handle it intelligently
        
        scaled_speed = throttle_norm * speed_limit * speed_multiplier
        scaled_steering = steering_norm * self.max_steering * steering_multiplier
        
        # Tank-style mixing with separate scaling
        # For left turn: left motor should slow down/reverse, right motor should speed up/forward
        # For right turn: left motor should speed up/forward, right motor should slow down/reverse
        target_left_speed = scaled_speed - scaled_steering   # Left motor: speed - steering
        target_right_speed = scaled_speed + scaled_steering  # Right motor: speed + steering
        
        # SMOOTH ACCELERATION - gradually ramp to target speed
        current_time = time.time()
        dt = current_time - self.last_control_time
        self.last_control_time = current_time

        # Auto re-zero smoothing if controls stay neutral for a short hold
        neutral_counts = 30
        controls_neutral = (
            not self.cruise_control
            and abs(self.throttle) <= neutral_counts
            and abs(self.steering) <= neutral_counts
        )
        if controls_neutral:
            if self.neutral_hold_start is None:
                self.neutral_hold_start = current_time
            elif current_time - self.neutral_hold_start >= self.neutral_hold_duration:
                if abs(self.current_speed) > 0.5 or abs(self.current_steering) > 0.5:
                    self.current_speed = 0.0
                    self.current_steering = 0.0
                    self.direction_change_braking = False
                    self.direction_change_target_sign = 0
                    if self.debug_mode:
                        self.debug_print("🧘 Neutral hold reset: smoothed speed/steer forced to 0")
        else:
            self.neutral_hold_start = None

        prev_smoothed_speed = self.current_speed
        
        # Calculate target average speed and steering
        target_speed = (target_left_speed + target_right_speed) / 2.0
        target_steer = (target_left_speed - target_right_speed) / 2.0
        
        # DEBUG: Log target_speed when throttle is negative (reverse requested)
        if self.debug_mode and self.firmware_mode == 'speed' and self.throttle < 0:
            self.debug_print(f"🔍 TARGET CALC: throttle={self.throttle}, throttle_norm={throttle_norm:.3f}, scaled_speed={scaled_speed:.1f}, target_speed={target_speed:.1f}, target_left={target_left_speed:.1f}, target_right={target_right_speed:.1f}, current_speed={self.current_speed:.1f}")

        # In speed mode, negative speed = reverse, so direction changes are natural
        # Only enable direction-change braking in speed mode when cart is moving forward
        # (reverse is allowed when stopped/backward, so no direction-change braking needed)
        if self.firmware_mode == 'speed' and self.current_speed > self.brake_release_threshold:
            # Detect direction change requests (stick pushed opposite current motion)
            direction_change_requested = (
                not self.direction_change_braking
                and not self.cruise_control
                and abs(prev_smoothed_speed) > self.direction_change_release_threshold
                and abs(target_speed) > self.direction_change_release_threshold
                and (prev_smoothed_speed * target_speed) < 0
            )
            if direction_change_requested:
                self.direction_change_braking = True
                self.direction_change_target_sign = 1 if target_speed > 0 else -1
                if self.debug_mode:
                    self.debug_print(
                        f"↔️ Direction change requested: Current={prev_smoothed_speed:.1f}, Target={target_speed:.1f}"
                    )
        else:
            self.direction_change_braking = False
        
        # Apply rate limiting for smooth acceleration
        speed_diff = target_speed - self.current_speed
        steer_diff = target_steer - self.current_steering
        
        # Choose acceleration or deceleration rate. A direction reversal must
        # decelerate through zero first; treating it as acceleration can turn
        # small distance corrections into forward/reverse oscillation.
        if abs(speed_diff) > 0.1:
            changing_direction = (
                abs(self.current_speed) > 0.5
                and abs(target_speed) > 0.5
                and self.current_speed * target_speed < 0
            )
            if changing_direction or abs(target_speed) < abs(self.current_speed):
                # Decelerating — follow-me needs real braking, not the gentle manual coast
                decel = (
                    self.follow_decel_rate
                    if self.follow_me_mode == 1
                    else self.decel_rate
                )
                max_speed_change = decel * dt
            else:
                if self.follow_me_mode == 1:
                    # Scale from a gentle near-target ramp to full acceleration.
                    # target_speed already represents the fused LiDAR/phone demand.
                    demand = min(
                        1.0,
                        abs(target_speed) / self.follow_accel_full_speed,
                    )
                    accel = self.follow_accel_rate_near + demand * (
                        self.follow_accel_rate - self.follow_accel_rate_near
                    )
                else:
                    accel = self.accel_rate
                max_speed_change = accel * dt
            
            # Limit speed change
            if abs(speed_diff) > max_speed_change:
                self.current_speed += max_speed_change if speed_diff > 0 else -max_speed_change
            else:
                self.current_speed = target_speed
        else:
            self.current_speed = target_speed
        
        # Apply steering rate limiting with fast deceleration to neutral
        if abs(target_steer) < 1.0 and abs(self.current_steering) > 1.0:
            # Returning to neutral - use fast decel rate
            max_steer_change = self.steer_decel_rate * dt
        else:
            # Applying steering - follow-me uses a gentler rate to avoid weave
            steer_rate = (
                self.follow_steer_rate
                if self.follow_me_mode == 1
                else self.steer_rate
            )
            max_steer_change = steer_rate * dt
        
        if abs(steer_diff) > max_steer_change:
            self.current_steering += max_steer_change if steer_diff > 0 else -max_steer_change
        else:
            self.current_steering = target_steer
        
        # Convert back to left/right motor speeds
        # FIXED: Right steering was inverted - corrected signs to match target calculation above
        # In speed mode: negative values = reverse, positive = forward
        # Use smoothed current_speed for all operations (rate limiting applied equally for forward and reverse)
        speed_for_motors = self.current_speed
        
        left_speed = int(speed_for_motors + self.current_steering)
        right_speed = int(speed_for_motors - self.current_steering)
        
        if self.debug_mode and self.firmware_mode == 'speed' and self.throttle < 0:
            self.debug_print(f"🔄 REVERSE CALC: target_speed={target_speed:.1f}, speed_for_motors={speed_for_motors:.1f}, left_speed={left_speed}, right_speed={right_speed}, throttle={self.throttle}, current_speed={self.current_speed:.1f}")
        
        # Note: Emergency stop is now handled by state machine above
        # This section is kept for backwards compatibility but shouldn't be reached
        
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
            # For steering in place (throttle neutral but steering active), use small minimum speed
            throttle_deadzone_turn = 15
            steering_only = abs(self.throttle) <= throttle_deadzone_turn and abs(self.steering) > throttle_deadzone_turn
            # Follow-me: only pivot in place for a clear off-center error (avoids left/right weave)
            follow_pivot_ok = (
                self.follow_me_mode != 1
                or abs(self.steering) >= self.follow_pivot_steering_min
            )
            
            if abs(steer) > 30 and follow_pivot_ok:  # Significant steering input
                if steering_only:
                    # Steering in place - add very small minimum speed to allow steering
                    speed = 15 if speed >= 0 else -15  # Small speed for steering in place
                elif abs(speed) < 50 and abs(self.throttle) > throttle_deadzone_turn:
                    # Turning with throttle input but low calculated speed - apply proportional floor,
                    # but never exceed the driver's requested speed
                    desired_speed_mag = (abs(self.throttle) / 512.0) * speed_limit
                    steer_ratio = min(1.0, abs(steer) / float(self.max_steering or 1))
                    min_turn_speed = desired_speed_mag * steer_ratio  # proportional to steering effort
                    min_turn_speed = min(min_turn_speed, desired_speed_mag)
                    if min_turn_speed > 0 and abs(speed) < min_turn_speed:
                        direction = 1 if (speed > 0 or (speed == 0 and self.throttle >= 0)) else -1
                        speed = direction * int(round(min_turn_speed))
                        if speed == 0:
                            speed = direction  # keep at least 1 unit when floor is tiny

            throttle_deadzone = 30  # Consider joystick neutral if within ±30 of 0
            steering_active = abs(self.steering) > throttle_deadzone
            throttle_neutral = not self.cruise_control and abs(self.throttle) <= throttle_deadzone

            # Direction-change braking overrides normal commands until cart stops
            if self.direction_change_braking:
                if throttle_neutral:
                    # User released stick to neutral: apply single strong brake pulse then exit
                    brake_strength = min(
                        max(abs(self.current_speed) * (self.brake_gain * 8), speed_limit * 0.15),
                        speed_limit * max(0.3, self.brake_max_ratio * 2)
                    )
                    speed = -int(math.copysign(brake_strength, self.current_speed))
                    self.direction_change_target_sign = 0
                    self.direction_change_braking = False
                    self.current_speed = 0.0
                    if self.debug_mode:
                        self.debug_print(
                            f"⛔ Direction-change brake canceled by neutral: Commanded={speed}"
                        )
                elif abs(self.current_speed) > self.direction_change_release_threshold:
                    # Apply strong braking opposite current motion regardless of joystick command
                    brake_strength = min(
                        max(abs(self.current_speed) * (self.brake_gain * 8), speed_limit * 0.15),
                        speed_limit * max(0.3, self.brake_max_ratio * 2)
                    )
                    speed = -int(math.copysign(brake_strength, self.current_speed))
                    if self.debug_mode:
                        self.debug_print(
                            f"⛔ Direction-change brake: Current={self.current_speed:.1f}, Commanded={speed}"
                        )
                else:
                    # Cart is effectively stopped, allow new direction to engage
                    self.direction_change_braking = False
                    self.current_speed = 0.0
                    desired_sign = self.direction_change_target_sign
                    self.direction_change_target_sign = 0
                    if desired_sign != 0 and abs(speed) < 5:
                        speed = desired_sign * 5
                    if self.debug_mode:
                        self.debug_print("✅ Direction-change brake complete - resuming commanded direction")
            
            # Active braking logic
            if self.firmware_mode == 'speed':
                # Speed mode: Automatic reverse behavior
                # If moving forward and joystick pulled back: brake until stopped, then reverse
                # If stopped/backward and joystick pulled back: reverse immediately
                # Follow-me uses near-max brake so we don't coast through the target distance
                follow_brake = self.follow_me_mode == 1
                if self.throttle < -30 and self.current_speed > self.brake_release_threshold:
                    # Pulling back while moving forward = apply negative speed (braking)
                    if follow_brake:
                        brake_strength = min(
                            max(abs(self.current_speed) * 0.9, speed_limit * 0.35),
                            speed_limit * 0.85,
                        )
                    else:
                        brake_strength = min(
                            abs(self.current_speed) * self.brake_gain,
                            self.max_speed * self.brake_max_ratio * 1.0,
                        )
                    speed = -int(brake_strength)  # Negative speed for braking
                    if self.debug_mode:
                        self.debug_print(f"🛑 Braking: CurrentSpeed={self.current_speed:.1f}, Throttle={self.throttle}, Commanded={speed} (will reverse once stopped)")
                elif self.throttle < -30 and abs(self.current_speed) <= self.brake_release_threshold:
                    # Pulling back at standstill/backward = allow reverse (handled by normal speed calculation above)
                    # Don't override speed here - let the normal calculation handle reverse
                    if self.debug_mode:
                        self.debug_print(f"🔄 Reverse allowed: Standstill/backward, throttle={self.throttle}, current_speed={self.current_speed:.1f}")
                elif throttle_neutral and abs(self.current_speed) > self.brake_release_threshold:
                    # Joystick released / follow deadzone while moving = brake
                    if follow_brake:
                        brake_strength = min(
                            max(abs(self.current_speed) * 0.9, speed_limit * 0.35),
                            speed_limit * 0.85,
                        )
                    else:
                        brake_strength = min(
                            abs(self.current_speed) * self.brake_gain,
                            self.max_speed * self.brake_max_ratio * 1.0,
                        )
                    speed = -int(math.copysign(brake_strength, self.current_speed))
                    if follow_brake:
                        self.current_speed = 0.0  # snap smoothed speed so we don't re-accelerate
                    if self.debug_mode:
                        self.debug_print(f"🛑 Active brake: CurrentSpeed={self.current_speed:.1f}, Commanded={speed}")
            elif self.firmware_mode == 'torque':
                # Torque mode: brake when joystick pulled back while moving forward
                # Negative throttle = braking command (opposite direction of current motion)
                if self.throttle < -30 and self.current_speed > self.brake_release_threshold:
                    # Pulling back while moving forward = apply reverse torque (braking)
                    brake_strength = min(abs(self.current_speed) * self.brake_gain, self.max_speed * self.brake_max_ratio * 1.0)
                    speed = -int(brake_strength)  # Negative torque for braking
                    if self.debug_mode:
                        self.debug_print(f"🛑 Torque mode brake: CurrentSpeed={self.current_speed:.1f}, Throttle={self.throttle}, Commanded={speed}")
                elif self.throttle < -30 and abs(self.current_speed) <= self.brake_release_threshold:
                    # Pulling back at standstill = do nothing (torque mode doesn't support reverse)
                    speed = 0
                    if self.debug_mode:
                        self.debug_print(f"🛑 Torque mode: Standstill, ignoring reverse throttle (torque mode doesn't support reverse)")
            
            # In speed mode: send raw values directly (like original files)
            # In torque mode: scale to -1000 to +1000 range
            if self.firmware_mode == 'speed':
                # Speed mode: clamp to max_speed and send directly (no scaling)
                # This matches the original behavior where max_speed=80 meant 80 units, not 1000
                steer_scaled = int(max(-300, min(300, steer)))  # Match max_steering for responsive turns
                speed_scaled = int(max(-speed_limit, min(speed_limit, speed)))
                
                # Safety: Never send speed=0 unless joystick is truly at neutral
                if abs(speed_scaled) < 5 and abs(self.throttle) > throttle_deadzone and not steering_active:
                    # Joystick is NOT at neutral, but we calculated speed=0 - this would cause braking
                    # Maintain minimum speed in the direction of last known throttle
                    if self.throttle > 0:
                        speed_scaled = max(5, speed_scaled)  # Ensure forward if throttle is positive
                    elif self.throttle < 0:
                        speed_scaled = min(-5, speed_scaled)  # Ensure reverse if throttle is negative
                    if self.debug_mode:
                        self.debug_print(f"⚠️ Prevented accidental brake: ThrottleRaw={self.throttle}, setting min speed={speed_scaled}")
            else:
                # Torque mode: scale to -1000 to +1000 range
                if speed_limit > 0:
                    speed_scaled = int((speed / speed_limit) * 1000)
                else:
                    speed_scaled = 0
                
                if self.max_steering > 0:
                    steer_scaled = int((steer / self.max_steering) * 1000)
                else:
                    steer_scaled = 0
                
                # Clamp to firmware's expected range
                steer_scaled = max(-1000, min(1000, steer_scaled))
                speed_scaled = max(-1000, min(1000, speed_scaled))
            
            # Log motor state only when throttle/steering output changes
            if self.debug_mode:
                motor_key = (self.throttle, self.steering, steer_scaled, speed_scaled)
                if motor_key != self._last_logged_motor_state:
                    self._last_logged_motor_state = motor_key
                    reverse_status = (
                        " [REVERSE]"
                        if (self.firmware_mode == 'speed' and speed < 0)
                        else ""
                    )
                    self.debug_print(
                        f"🔧 MOTOR: steer={steer_scaled} speed={speed_scaled} "
                        f"(throttle={self.throttle}, steering={self.steering})"
                        f"{reverse_status}"
                    )
            
            self.send_hoverboard_command(steer_scaled, speed_scaled)
    
    def start(self):
        """Start the controller (phone-follow + handheld remote + motors)."""
        if not self.connect_esp32():
            return False

        # Set running before starting follow threads so their loops don't exit early.
        self.running = True

        self.debug_print("📱 Phone-follow: phone steering + Pi LiDAR distance")
        self.start_phone_lidar_follow()
        
        if not self.connect_hoverboard():
            print("Warning: Hoverboard not connected")
        
        # Connect and calibrate MPU6050 for tilt protection
        if self.connect_mpu6050():
            self.debug_print("🔄 Calibrating MPU6050 - Please keep cart stable...")
            time.sleep(2)  # Give user time to stabilize cart
            if self.calibrate_mpu6050():
                self.debug_print("✓ Tilt protection enabled")
            else:
                self.debug_print("⚠️ Tilt protection calibration failed - continuing without protection")
        else:
            self.debug_print("⚠️ MPU6050 not connected - tilt protection disabled")

        # Start data thread
        data_thread = threading.Thread(target=self.read_data_loop, daemon=True)
        data_thread.start()
        
        self.debug_print("Minimal controller started")
        self.debug_print("🚀 Performance mode: Auto-enabled during cruise control for instant response")
        
        # Display tilt protection status
        if self.mpu6050_connected:
            self.debug_print(f"🛡️ Tilt protection: ACTIVE (threshold: {self.tilt_threshold}°)")
        else:
            self.debug_print("⚠️ Tilt protection: DISABLED (MPU6050 not connected)")
        
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
                
                # Watchdog: Ensure commands are sent at minimum rate to prevent timeout beeps
                current_time = time.time()
                if (
                    self.hoverboard_connected
                    and not self.calibration_beep_active
                    and (current_time - self.last_command_sent_time) >= self.min_command_interval
                ):
                    # If no command was sent in the last interval, send a keepalive
                    if self.last_command_sent_time > 0:  # Only after first command
                        # Send last known command (or zero if no command sent yet)
                        try:
                            # Get last sent values or send zero
                            if hasattr(self, 'last_steer') and hasattr(self, 'last_speed'):
                                self.send_hoverboard_command(self.last_steer, self.last_speed)
                            else:
                                self.send_hoverboard_command(0, 0)
                            if self.debug_mode:
                                self.debug_print("🔄 Watchdog: Keepalive command sent")
                        except Exception as e:
                            if self.debug_mode:
                                self.debug_print(f"❌ Watchdog error: {e}")
                
                # ULTRA-FAST control loop for instant brake response
                if self.cruise_control:
                    time.sleep(0.0001)  # 10000Hz during cruise control
                else:
                    time.sleep(0.001)   # 1000Hz normal operation (INSTANT brake response!)
                
        except KeyboardInterrupt:
            self.debug_print("\nStopping...")
        finally:
            self.running = False
            if self.phone_bridge:
                self.phone_bridge.stop()
            if self.esp32_ser:
                self.esp32_ser.close()
            if self.hoverboard_ser:
                self.hoverboard_ser.close()

def main():
    print("Minimal Hoverboard Controller")
    print("============================")
    
    debug_mode = True  # Default to debug mode
    args = [a.lower() for a in sys.argv[1:]]

    if '--silent' in args or '-s' in args or '--quiet' in args or '-q' in args:
        debug_mode = False
        print("🔇 SILENT MODE: No logging output")
    if '--debug' in args or '-d' in args or '--verbose' in args or '-v' in args:
        debug_mode = True
        print("🔊 DEBUG MODE: Full logging output")

    known = {
        '--silent', '-s', '--quiet', '-q',
        '--debug', '-d', '--verbose', '-v',
    }
    if args and not any(flag in known for flag in args):
        print(f"Usage: {sys.argv[0]} [options]")
        print("  --silent, -s: No logging output (production mode)")
        print("  --debug, -d: Full logging output (debug mode)")
        print("  No argument: Phone-follow + debug logging")
        print("")

    print("📱 PHONE FOLLOW: phone steering + Pi LiDAR distance")
    if debug_mode:
        print("🔊 DEBUG MODE: Full logging output")
    print("")

    controller = MinimalHoverboardController()
    controller.debug_mode = debug_mode
    controller.start()

if __name__ == "__main__":
    main()
