#!/usr/bin/env python3
"""
Raspberry Pi Zero 2 W Electric Push Cart Controller
Integrates: USB Motor Control + MPU6050 + HOTRC DS600 RC Controller
"""

import odrive
import time
import threading
import math
import RPi.GPIO as GPIO
# from smbus import SMBus  # Commented out for now
import struct
import sys

# class MPU6050:
#     """MPU6050 accelerometer/gyroscope interface"""
#     
#     def __init__(self, i2c_bus=1, address=0x68):
#         self.bus = SMBus(i2c_bus)
#         self.address = address
#         self.setup()
#     
#     def setup(self):
#         """Initialize MPU6050"""
#         # Wake up MPU6050
#         self.bus.write_byte_data(self.address, 0x6B, 0)
#         
#         # Configure accelerometer (+/- 2g)
#         self.bus.write_byte_data(self.address, 0x1C, 0)
#         
#         # Configure gyroscope (+/- 250 deg/s)
#         self.bus.write_byte_data(self.address, 0x1B, 0)
#         
#         print("✅ MPU6050 initialized")
#     
#     def read_raw_data(self, register):
#         """Read raw 16-bit data from register"""
#         high = self.bus.read_byte_data(self.address, register)
#         low = self.bus.read_byte_data(self.address, register + 1)
#         value = (high << 8) | low
#         if value > 32768:
#             value = value - 65536
#         return value
#     
#     def get_accel_data(self):
#         """Get accelerometer data in g"""
#         accel_x = self.read_raw_data(0x3B) / 16384.0
#         accel_y = self.read_raw_data(0x3D) / 16384.0
#         accel_z = self.read_raw_data(0x3F) / 16384.0
#         return accel_x, accel_y, accel_z
#     
#     def get_gyro_data(self):
#         """Get gyroscope data in deg/s"""
#         gyro_x = self.read_raw_data(0x43) / 131.0
#         gyro_y = self.read_raw_data(0x45) / 131.0
#         gyro_z = self.read_raw_data(0x47) / 131.0
#         return gyro_x, gyro_y, gyro_z
#     
#     def get_tilt_angle(self):
#         """Calculate tilt angle from accelerometer"""
#         accel_x, accel_y, accel_z = self.get_accel_data()
#         
#         # Calculate pitch and roll angles
#         pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
#         roll = math.atan2(-accel_x, accel_z) * 180 / math.pi
#         
#         return pitch, roll

class RC_Controller:
    """HOTRC DS600 RC Controller interface - Single thumbstick with 2 axes"""
    
    def __init__(self, throttle_pin=18, steering_pin=20, mixing_mode=False, deadzone=0.05):
        # DS600 has ONE thumbstick that outputs two channels:
        # CH1 (steering) = Left/Right axis of thumbstick → GPIO 20 (changed from 19)
        # CH2 (throttle) = Up/Down axis of thumbstick → GPIO 18
        self.throttle_pin = throttle_pin    # GPIO 18 = CH2 (Up/Down axis)
        self.steering_pin = steering_pin    # GPIO 20 = CH1 (Left/Right axis)
        self.mixing_mode = mixing_mode  # True if DS600 mixing is enabled
        self.deadzone = deadzone  # Deadzone around neutral (0.0 to 1.0)
        
        # DS600 actual measured ranges (from test_pwm_channels.py results)
        # Throttle (CH2/GPIO18): neutral ~1474, forward ~1833, backward ~974-979μs
        self.throttle_neutral = 1474
        self.throttle_min = 974      # Backward (negative)
        self.throttle_max = 1833     # Forward (positive)
        
        # Steering (CH1/GPIO20): neutral ~1559, left ~1059, right ~1665-1881μs  
        self.steering_neutral = 1559
        self.steering_min = 1059     # Left (negative) - reverted to original
        self.steering_max = 1881     # Right (positive) - reverted to original
        
        # Hardware deadzone (microseconds) - increased to eliminate small unwanted inputs
        self.throttle_deadzone_width = 60  # ±60μs around neutral (was 20μs)
        self.steering_deadzone_width = 100  # ±100μs around neutral (was 30μs)
        
        self.throttle_value = self.throttle_neutral  # Start at neutral
        self.steering_value = self.steering_neutral  # Start at neutral
        self.last_update = time.time()
        
        # DS600-specific settings
        self.pwm_timeout = 0.025  # 25ms timeout (DS600 is 20ms)
        self.signal_lost_threshold = 2.0  # 2 seconds
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.throttle_pin, GPIO.IN)
        GPIO.setup(self.steering_pin, GPIO.IN)
        
        mode_str = "DS600 Built-in Mixing" if mixing_mode else "Manual Mixing"
        print(f"✅ RC Controller initialized - {mode_str}")
        print("   📡 HOTRC DS600 - Single thumbstick controller")
        print("   🕹️  Thumbstick: Up/Down=Throttle, Left/Right=Steering")
        print("   📊 Range: 300-500m on water")
        print(f"   🎯 Deadzone: ±{deadzone*100:.0f}% around neutral")
        print(f"   🎯 Hardware Deadzone: ±{self.steering_deadzone_width}μs steering, ±{self.throttle_deadzone_width}μs throttle")
        print(f"   📏 Throttle: {self.throttle_min}-{self.throttle_neutral}-{self.throttle_max}μs")
        print(f"   📏 Steering: {self.steering_min}-{self.steering_neutral}-{self.steering_max}μs")
    
    def read_pwm_pulse(self, pin):
        """Read PWM pulse width optimized for DS600 (20ms PWM)"""
        try:
            # Wait for rising edge with longer timeout (like test script)
            start_time = time.time()
            while GPIO.input(pin) == GPIO.LOW:
                if time.time() - start_time > 0.05:  # 50ms timeout (like test)
                    return None  # Signal lost
            
            # Measure high pulse (DS600: 1000-2000μs)
            pulse_start = time.time()
            while GPIO.input(pin) == GPIO.HIGH:
                if time.time() - pulse_start > 0.003:  # 3ms max (like test)
                    return None
            
            pulse_width = (time.time() - pulse_start) * 1000000  # Convert to microseconds
            
            # Validate DS600 range (1000-2000μs) - same as test script
            if 800 <= pulse_width <= 2200:  # Allow some tolerance
                return int(pulse_width)
            else:
                return None  # Invalid signal
                
        except:
            return None  # Error reading signal
    
    def update(self):
        """Update RC values with DS600 fail-safe handling"""
        throttle_raw = self.read_pwm_pulse(self.throttle_pin)
        steering_raw = self.read_pwm_pulse(self.steering_pin)
        
        # Handle DS600 fail-safe (defaults to neutral on signal loss)
        if throttle_raw is not None:
            self.throttle_value = throttle_raw
            self.last_update = time.time()
        # Keep last valid value if signal temporarily lost
        
        if steering_raw is not None:
            self.steering_value = steering_raw
            self.last_update = time.time()
        # Keep last valid value if signal temporarily lost
    
    def apply_deadzone(self, value):
        """Apply deadzone to normalized value (-1.0 to 1.0)"""
        if abs(value) < self.deadzone:
            return 0.0  # In deadzone - return zero
        else:
            # Scale the remaining range to maintain smooth control
            if value > 0:
                # Positive: scale from deadzone to 1.0
                return (value - self.deadzone) / (1.0 - self.deadzone)
            else:
                # Negative: scale from -deadzone to -1.0  
                return (value + self.deadzone) / (1.0 - self.deadzone)
    
    def get_normalized_values(self):
        """Get normalized values for DS600 control modes with deadzone using actual measured ranges"""
        
        # Apply hardware-level deadzone around neutral positions first
        effective_throttle = self.throttle_value
        effective_steering = self.steering_value
        
        # Throttle hardware deadzone - if close to neutral, snap to neutral
        if abs(self.throttle_value - self.throttle_neutral) <= self.throttle_deadzone_width:
            effective_throttle = self.throttle_neutral
            
        # Steering hardware deadzone - if close to neutral, snap to neutral  
        if abs(self.steering_value - self.steering_neutral) <= self.steering_deadzone_width:
            effective_steering = self.steering_neutral
        
        if self.mixing_mode:
            # DS600 mixing mode: CH1=Left Motor, CH2=Right Motor
            # Convert each channel to motor speed directly using actual ranges
            
            # Throttle (left motor in mixing mode)
            if effective_throttle >= self.throttle_neutral:
                # Forward: scale from neutral to max
                left_motor = (effective_throttle - self.throttle_neutral) / (self.throttle_max - self.throttle_neutral)
            else:
                # Backward: scale from min to neutral  
                left_motor = (effective_throttle - self.throttle_neutral) / (self.throttle_neutral - self.throttle_min)
            
            # Steering (right motor in mixing mode)
            if effective_steering >= self.steering_neutral:
                # Right: scale from neutral to max
                right_motor = (effective_steering - self.steering_neutral) / (self.steering_max - self.steering_neutral)
            else:
                # Left: scale from min to neutral
                right_motor = (effective_steering - self.steering_neutral) / (self.steering_neutral - self.steering_min)
            
            # Clamp values
            left_motor = max(-1.0, min(1.0, left_motor))
            right_motor = max(-1.0, min(1.0, right_motor))
            
            # Apply deadzone
            left_motor = self.apply_deadzone(left_motor)
            right_motor = self.apply_deadzone(right_motor)
            
            return left_motor, right_motor  # Direct motor values
        else:
            # Manual mixing mode: CH1=Steering, CH2=Throttle
            # Use asymmetric normalization for both channels
            
            # Throttle normalization with asymmetric range
            if effective_throttle >= self.throttle_neutral:
                # Forward: scale from neutral to max
                throttle_norm = (effective_throttle - self.throttle_neutral) / (self.throttle_max - self.throttle_neutral)
            else:
                # Backward: scale from min to neutral
                throttle_norm = (effective_throttle - self.throttle_neutral) / (self.throttle_neutral - self.throttle_min)
            
            # Steering normalization with asymmetric range
            if effective_steering >= self.steering_neutral:
                # Right: scale from neutral to max
                steering_norm = (effective_steering - self.steering_neutral) / (self.steering_max - self.steering_neutral)
            else:
                # Left: scale from min to neutral
                steering_norm = (effective_steering - self.steering_neutral) / (self.steering_neutral - self.steering_min)
            
            # Clamp values
            throttle_norm = max(-1.0, min(1.0, throttle_norm))
            steering_norm = max(-1.0, min(1.0, steering_norm))
            
            # Apply deadzone
            throttle_norm = self.apply_deadzone(throttle_norm)
            steering_norm = self.apply_deadzone(steering_norm)
            
            # Apply software deadzone to normalized values 
            software_deadzone = 0.15  # Ignore inputs smaller than 15%
            
            if abs(throttle_norm) < software_deadzone:
                throttle_norm = 0.0
            
            if abs(steering_norm) < software_deadzone:
                steering_norm = 0.0
            
            return throttle_norm, steering_norm  # Throttle/steering values
    
    def is_connected(self):
        """Check DS600 connection (2.4GHz FHSS reliability)"""
        return (time.time() - self.last_update) < self.signal_lost_threshold
    
    def get_signal_strength(self):
        """Estimate signal quality based on update frequency"""
        time_since_update = time.time() - self.last_update
        if time_since_update > self.signal_lost_threshold:
            return "LOST"
        elif time_since_update > 0.5:
            return "WEAK"
        elif time_since_update > 0.1:
            return "GOOD"
        else:
            return "STRONG"

class PushCartController:
    """Main controller integrating all systems"""
    
    def __init__(self, use_ds600_mixing=False):
        # RC Controller
        self.rc_controller = RC_Controller(mixing_mode=use_ds600_mixing)
        
        # Motors
        self.left_motor = None
        self.right_motor = None
        self.max_velocity = 3.0  # Maximum velocity (rotations per second)
        
        # Motor direction flags - set based on physical wiring
        self.left_motor_inverted = True   # Add direction inversion
        self.right_motor_inverted = True  # Add direction inversion
        
        # Serial numbers to identify specific motors
        # These match your actual ODrive serial numbers
        self.left_serial = 0x395833613233   # Left side ODrive
        self.right_serial = 0x3959335e3233  # Right side ODrive
        
        # USB Connection Monitoring
        self.last_left_connection_check = time.time()
        self.last_right_connection_check = time.time()
        self.connection_check_interval = 1.0  # Check connections every 1 second
        self.reconnection_in_progress = False
        self.reconnection_attempts = 0
        self.max_reconnection_attempts = 3
        self.last_reconnection_attempt = 0
        self.reconnection_cooldown = 10.0  # Wait 10 seconds between reconnection attempts
        
        self.mpu6050 = None
        self.emergency_stop = False
        self.running = False
        
        self.max_tilt_angle = 30.0  # Max safe tilt angle
        
    def initialize_sensors(self):
        """Initialize MPU6050 and RC controller"""
        try:
            print("🔧 Initializing sensors...")
            
            # Initialize MPU6050
            # self.mpu6050 = MPU6050()  # Commented out for now
            self.mpu6050 = None  # Set to None for now
            
            # Initialize RC controller with DS600 mixing mode
            self.rc_controller = RC_Controller(mixing_mode=self.use_ds600_mixing)
            
            if self.use_ds600_mixing:
                print("📡 DS600 Setup Instructions:")
                print("   1. Turn off DS600 transmitter")
                print("   2. Hold CH3 switch, then turn on DS600")
                print("   3. Green light should be ON (mixing enabled)")
                print("   4. Single thumbstick controls both motors directly")
            else:
                print("📡 DS600 Manual Mode (Recommended):")
                print("   1. Ensure mixing control is OFF (no green light)")
                print("   2. Single thumbstick: Up/Down=Throttle, Left/Right=Steering")
                print("   3. Wiring: CH1→GPIO20 (steering), CH2→GPIO18 (throttle)")
            
            return True
            
        except Exception as e:
            print(f"❌ Sensor initialization failed: {e}")
            return False
    
    def find_motors(self):
        """Find and connect to both motors"""
        print("🔍 Searching for motors...")
        
        try:
            # Try to find multiple ODrives
            print("   Looking for ODrives...")
            odrives = odrive.find_any(count=2, timeout=10)
            
            # Handle different return types from odrive library
            if odrives is None:
                print("❌ No ODrives found!")
                return False
            
            # Convert to list if needed
            if not isinstance(odrives, (list, tuple)):
                odrives = [odrives]
            elif isinstance(odrives, tuple):
                odrives = list(odrives)
            
            print(f"✅ Found {len(odrives)} ODrive(s)")
            
            # Identify motors by serial number
            for i, odrv in enumerate(odrives):
                try:
                    serial = odrv.serial_number
                    print(f"   📡 ODrive {i+1} serial: {hex(serial)}")
                    
                    if serial == self.left_serial:
                        self.left_motor = odrv
                        print(f"   ✅ Left Motor connected (physical left wheel)")
                    elif serial == self.right_serial:
                        self.right_motor = odrv
                        print(f"   ✅ Right Motor connected (physical right wheel)")
                    else:
                        print(f"   ❓ Unknown motor: {hex(serial)}")
                        
                except Exception as e:
                    print(f"   ❌ Error reading ODrive {i+1}: {e}")
            
            motors_found = sum([self.left_motor is not None, self.right_motor is not None])
            print(f"📊 Motors connected: {motors_found}/2")
            
            if motors_found == 0:
                print("❌ No recognized motors found!")
                print("   Expected serials:")
                print(f"   - Left:  {hex(self.left_serial)}")
                print(f"   - Right: {hex(self.right_serial)}")
            
            return motors_found > 0
            
        except Exception as e:
            print(f"❌ Error finding motors: {e}")
            print("   Trying single ODrive search...")
            
            # Fallback: try to find just one ODrive
            try:
                single_odrv = odrive.find_any(timeout=5)
                if single_odrv:
                    serial = single_odrv.serial_number
                    print(f"   📡 Found single ODrive: {hex(serial)}")
                    
                    if serial == self.left_serial:
                        self.left_motor = single_odrv
                        print(f"   ✅ Left Motor connected")
                        return True
                    elif serial == self.right_serial:
                        self.right_motor = single_odrv
                        print(f"   ✅ Right Motor connected")
                        return True
                    else:
                        print(f"   ❓ Unknown motor: {hex(serial)}")
                        return False
                else:
                    print("   ❌ No ODrives found in single search")
                    return False
                    
            except Exception as e2:
                print(f"   ❌ Single search also failed: {e2}")
                return False
    
    def initialize_motors(self):
        """Initialize motors for control"""
        print("🔧 Initializing motors...")
        
        ready_count = 0
        
        for motor, name in [(self.left_motor, "Left"), (self.right_motor, "Right")]:
            if motor:
                try:
                    state = motor.axis0.current_state
                    if state != 8:  # Not in closed loop
                        print(f"   Setting {name} motor to closed loop...")
                        motor.axis0.requested_state = 8
                        time.sleep(2)
                    
                    if motor.axis0.current_state == 8:
                        print(f"   ✅ {name} motor ready")
                        ready_count += 1
                    else:
                        print(f"   ❌ {name} motor failed to initialize")
                        
                except Exception as e:
                    print(f"   ❌ {name} motor error: {e}")
        
        return ready_count > 0
    
    def set_motor_velocities(self, left_vel, right_vel):
        """Set motor velocities with safety checks, direction correction, and USB error handling"""
        # Safety checks
        if self.emergency_stop:
            left_vel = right_vel = 0.0
        
        # Apply direction inversion if needed
        if self.left_motor_inverted:
            left_vel = -left_vel
        if self.right_motor_inverted:
            right_vel = -right_vel
        
        # Clamp velocities
        left_vel = max(-self.max_velocity, min(self.max_velocity, left_vel))
        right_vel = max(-self.max_velocity, min(self.max_velocity, right_vel))
        
        # Handle left motor
        if self.left_motor:
            try:
                # Check motor state before sending command
                left_state = self.left_motor.axis0.current_state
                if left_state != 8:  # Not in closed loop
                    print(f"⚠️  Left motor not in closed loop (state: {left_state})")
                else:
                    self.left_motor.axis0.controller.input_vel = left_vel
            except Exception as e:
                print(f"❌ Left motor USB error: {e}")
                self.left_motor = None  # Mark as disconnected
                if not self.emergency_stop:
                    print("🚨 Left motor communication failed - entering emergency stop")
                    self.emergency_stop = True
        
        # Handle right motor  
        if self.right_motor:
            try:
                # Check motor state before sending command  
                right_state = self.right_motor.axis0.current_state
                if right_state != 8:  # Not in closed loop
                    print(f"⚠️  Right motor not in closed loop (state: {right_state})")
                else:
                    self.right_motor.axis0.controller.input_vel = right_vel
            except Exception as e:
                print(f"❌ Right motor USB error: {e}")
                self.right_motor = None  # Mark as disconnected
                if not self.emergency_stop:
                    print("🚨 Right motor communication failed - entering emergency stop")
                    self.emergency_stop = True
        
        # If both motors are disconnected, trigger emergency stop
        if self.left_motor is None and self.right_motor is None:
            if not self.emergency_stop:
                print("🚨 All motors disconnected - entering emergency stop")
                self.emergency_stop = True
    
    def calculate_motor_speeds(self, throttle, steering):
        """Convert throttle/steering to left/right motor speeds with proper differential steering"""
        
        # For differential steering, we need opposite adjustments on each motor:
        # - Left turn (negative steering): slow down left motor, speed up right motor  
        # - Right turn (positive steering): speed up left motor, slow down right motor
        
        base_speed = throttle * self.max_velocity
        
        # Handle pure turning (no throttle) - allow opposite motor directions
        if abs(throttle) < 0.05:
            steering_gain = 0.8
            # For pure turning: opposite directions
            left_speed = steering * steering_gain * self.max_velocity    # FIXED: Removed negative sign
            right_speed = -steering * steering_gain * self.max_velocity  # FIXED: Added negative sign
        else:
            # For forward/backward motion with steering, prevent direction reversal
            # Limit steering influence to prevent overwhelming the base speed
            max_steering_ratio = 0.7  # Steering can adjust speed by up to 70% of base speed
            max_steering_adjustment = abs(base_speed) * max_steering_ratio
            steering_adjustment = steering * max_steering_adjustment
            
            # Apply steering with CORRECTED signs
            left_speed = base_speed + steering_adjustment   # FIXED: Changed from - to +
            right_speed = base_speed - steering_adjustment  # FIXED: Changed from + to -
            
            # Prevent direction reversal due to steering (unless pure turning)
            if base_speed > 0:  # Moving forward
                left_speed = max(0.0, left_speed)   # Don't let left motor go backward
                right_speed = max(0.0, right_speed) # Don't let right motor go backward
            elif base_speed < 0:  # Moving backward  
                left_speed = min(0.0, left_speed)   # Don't let left motor go forward
                right_speed = min(0.0, right_speed) # Don't let right motor go forward
        
        # Clamp to maximum velocity limits
        left_speed = max(-self.max_velocity, min(self.max_velocity, left_speed))
        right_speed = max(-self.max_velocity, min(self.max_velocity, right_speed))
        
        return left_speed, right_speed
    
    def safety_monitor(self):
        """Monitor tilt, RC connection, and USB connections"""
        while self.running:
            try:
                # USB Connection monitoring (most important for stability)
                self.monitor_usb_connections()
                
                # MPU6050 tilt monitoring commented out for now
                # if self.mpu6050:
                #     pitch, roll = self.mpu6050.get_tilt_angle()
                #     
                #     # Check for dangerous tilt
                #     if abs(pitch) > self.max_tilt_angle or abs(roll) > self.max_tilt_angle:
                #         print(f"⚠️  TILT WARNING: Pitch={pitch:.1f}°, Roll={roll:.1f}°")
                #         self.emergency_stop = True
                #         self.set_motor_velocities(0, 0)
                
                # Check RC connection
                if self.rc_controller and not self.rc_controller.is_connected():
                    print("⚠️  RC CONNECTION LOST")
                    self.emergency_stop = True
                    self.set_motor_velocities(0, 0)
                
                time.sleep(0.1)  # 10Hz safety monitoring
                
            except Exception as e:
                print(f"❌ Safety monitor error: {e}")
                time.sleep(1)
    
    def check_motor_connection(self, motor, name):
        """Check if a motor is still connected and responsive"""
        if motor is None:
            return False
        
        try:
            # Try to read a simple property to test connection
            _ = motor.serial_number
            _ = motor.axis0.current_state
            return True
        except Exception as e:
            print(f"⚠️  {name} motor connection lost: {e}")
            return False
    
    def attempt_motor_reconnection(self):
        """Attempt to reconnect to disconnected motors"""
        if self.reconnection_in_progress:
            return False
        
        current_time = time.time()
        if current_time - self.last_reconnection_attempt < self.reconnection_cooldown:
            return False
        
        if self.reconnection_attempts >= self.max_reconnection_attempts:
            print(f"❌ Maximum reconnection attempts ({self.max_reconnection_attempts}) reached")
            return False
        
        print("🔄 Attempting to reconnect to ODrive controllers...")
        self.reconnection_in_progress = True
        self.last_reconnection_attempt = current_time
        self.reconnection_attempts += 1
        
        try:
            # Clear existing references
            disconnected_motors = []
            if not self.check_motor_connection(self.left_motor, "Left"):
                self.left_motor = None
                disconnected_motors.append("Left")
            if not self.check_motor_connection(self.right_motor, "Right"):
                self.right_motor = None
                disconnected_motors.append("Right")
            
            if not disconnected_motors:
                print("✅ All motors still connected")
                self.reconnection_in_progress = False
                return True
            
            print(f"🔍 Reconnecting to: {', '.join(disconnected_motors)}")
            
            # Try to find ODrives again
            odrives = odrive.find_any(count=2, timeout=8)
            
            if odrives is None:
                print("❌ No ODrives found during reconnection")
                self.reconnection_in_progress = False
                return False
            
            # Convert to list if needed
            if not isinstance(odrives, (list, tuple)):
                odrives = [odrives]
            elif isinstance(odrives, tuple):
                odrives = list(odrives)
            
            # Re-identify motors by serial number
            reconnected = []
            for odrv in odrives:
                try:
                    serial = odrv.serial_number
                    if serial == self.left_serial and self.left_motor is None:
                        self.left_motor = odrv
                        print(f"🔌 Left motor reconnected")
                        reconnected.append("Left")
                    elif serial == self.right_serial and self.right_motor is None:
                        self.right_motor = odrv
                        print(f"🔌 Right motor reconnected") 
                        reconnected.append("Right")
                except Exception as e:
                    print(f"❌ Error identifying ODrive: {e}")
            
            # Re-initialize reconnected motors
            success = True
            for motor, name in [(self.left_motor, "Left"), (self.right_motor, "Right")]:
                if motor and name in reconnected:
                    try:
                        state = motor.axis0.current_state
                        if state != 8:  # Not in closed loop
                            print(f"   Setting {name} motor to closed loop...")
                            motor.axis0.requested_state = 8
                            time.sleep(2)
                        
                        if motor.axis0.current_state == 8:
                            print(f"   ✅ {name} motor reinitialized")
                        else:
                            print(f"   ❌ {name} motor failed to reinitialize")
                            success = False
                    except Exception as e:
                        print(f"   ❌ {name} motor reinit error: {e}")
                        success = False
            
            if success and reconnected:
                print(f"🎉 Successfully reconnected: {', '.join(reconnected)}")
                self.reconnection_attempts = 0  # Reset counter on success
                # Don't clear emergency_stop here - let safety monitor handle it
            else:
                print("❌ Reconnection failed")
            
            self.reconnection_in_progress = False
            return success
            
        except Exception as e:
            print(f"❌ Reconnection error: {e}")
            self.reconnection_in_progress = False
            return False
    
    def monitor_usb_connections(self):
        """Monitor USB connections and trigger reconnection if needed"""
        current_time = time.time()
        
        # Check connections at specified interval
        if current_time - self.last_left_connection_check >= self.connection_check_interval:
            if not self.check_motor_connection(self.left_motor, "Left"):
                self.left_motor = None
                if not self.emergency_stop:
                    print("🚨 Left motor disconnected - entering emergency stop")
                self.emergency_stop = True
                
        if current_time - self.last_right_connection_check >= self.connection_check_interval:
            if not self.check_motor_connection(self.right_motor, "Right"):
                self.right_motor = None
                if not self.emergency_stop:
                    print("🚨 Right motor disconnected - entering emergency stop")
                self.emergency_stop = True
        
        # Attempt reconnection if in emergency stop due to disconnection
        if self.emergency_stop and (self.left_motor is None or self.right_motor is None):
            if self.attempt_motor_reconnection():
                # Check if we have at least one working motor
                working_motors = sum([self.left_motor is not None, self.right_motor is not None])
                if working_motors > 0:
                    print("🟡 Partial reconnection successful - clearing emergency stop")
                    self.emergency_stop = False
                elif working_motors == 2:
                    print("🟢 Full reconnection successful - clearing emergency stop")
                    self.emergency_stop = False
        
        # Update check timestamps
        self.last_left_connection_check = current_time
        self.last_right_connection_check = current_time
    
    def run(self):
        """Main control loop"""
        print("🚀 Starting push cart controller...")
        
        # Initialize all systems
        if not self.initialize_sensors():
            return False
        
        if not self.find_motors():
            return False
        
        if not self.initialize_motors():
            return False
        
        print("🎉 All systems initialized!")
        
        # Start safety monitor thread
        self.running = True
        safety_thread = threading.Thread(target=self.safety_monitor, daemon=True)
        safety_thread.start()
        
        try:
            print("🎮 Push cart control active...")
            print("Press Ctrl+C to stop")
            
            while True:
                # Update RC inputs
                if self.rc_controller:
                    self.rc_controller.update()
                    
                    if self.use_ds600_mixing:
                        # DS600 handles mixing - get direct motor values
                        left_speed, right_speed = self.rc_controller.get_normalized_values()
                        
                        # Scale to max velocity
                        left_speed *= self.max_velocity
                        right_speed *= self.max_velocity
                        
                    else:
                        # Manual mixing - get throttle/steering
                        throttle, steering = self.rc_controller.get_normalized_values()
                        
                        # Calculate motor speeds using curvature drive mixing
                        left_speed, right_speed = self.calculate_motor_speeds(throttle, steering)
                    
                    # Apply motor speeds (if not in emergency stop)
                    if not self.emergency_stop:
                        self.set_motor_velocities(left_speed, right_speed)
                    
                    # Status display (every 2 seconds)
                    if int(time.time() * 0.5) % 2 == 0:
                        signal_strength = self.rc_controller.get_signal_strength()
                        
                        # MPU6050 tilt display commented out for now
                        # if self.mpu6050:
                        #     pitch, roll = self.mpu6050.get_tilt_angle()
                        #     
                        #     if self.use_ds600_mixing:
                        #         print(f"📊 [DS600-MIX] L:{left_speed:.1f} R:{right_speed:.1f} | "
                        #               f"Signal:{signal_strength} | Tilt: {pitch:.1f}°/{roll:.1f}°")
                        #     else:
                        #         throttle, steering = self.rc_controller.get_normalized_values()
                        #         print(f"📊 [MANUAL] T:{throttle:.2f} S:{steering:.2f} | "
                        #               f"L:{left_speed:.1f} R:{right_speed:.1f} | "
                        #               f"Signal:{signal_strength} | Tilt: {pitch:.1f}°/{roll:.1f}°")
                        
                        # Enhanced status display with raw PWM values for debugging
                        if self.use_ds600_mixing:
                            print(f"📊 [DS600-MIX] L:{left_speed:.1f} R:{right_speed:.1f} | Signal:{signal_strength}")
                        else:
                            throttle, steering = self.rc_controller.get_normalized_values()
                            # Show raw PWM values for debugging
                            raw_throttle = self.rc_controller.throttle_value
                            raw_steering = self.rc_controller.steering_value
                            
                            # Calculate percentages for easier reading using actual ranges
                            if raw_throttle >= self.rc_controller.throttle_neutral:
                                throttle_pct = ((raw_throttle - self.rc_controller.throttle_neutral) / 
                                              (self.rc_controller.throttle_max - self.rc_controller.throttle_neutral)) * 100
                            else:
                                throttle_pct = ((raw_throttle - self.rc_controller.throttle_neutral) / 
                                              (self.rc_controller.throttle_neutral - self.rc_controller.throttle_min)) * -100
                            
                            if raw_steering >= self.rc_controller.steering_neutral:
                                steering_pct = ((raw_steering - self.rc_controller.steering_neutral) / 
                                              (self.rc_controller.steering_max - self.rc_controller.steering_neutral)) * 100
                            else:
                                steering_pct = ((raw_steering - self.rc_controller.steering_neutral) / 
                                              (self.rc_controller.steering_neutral - self.rc_controller.steering_min)) * -100
                            
                            print(f"📊 [MANUAL] T:{throttle:.2f}({throttle_pct:+.0f}%) S:{steering:.2f}({steering_pct:+.0f}%) | "
                                  f"L:{left_speed:.1f} R:{right_speed:.1f} | "
                                  f"PWM: T:{raw_throttle} S:{raw_steering} | Signal:{signal_strength}")
                            
                            # USB Connection Status
                            left_status = "✅" if self.left_motor else "❌"
                            right_status = "✅" if self.right_motor else "❌"
                            emergency_status = "🚨 STOP" if self.emergency_stop else "🟢 OK"
                            
                            connection_info = f"USB: L{left_status} R{right_status} | Status:{emergency_status}"
                            if self.reconnection_attempts > 0:
                                connection_info += f" | Reconnect:{self.reconnection_attempts}/{self.max_reconnection_attempts}"
                            if self.reconnection_in_progress:
                                connection_info += " | 🔄 Reconnecting..."
                            
                            print(f"🔌 {connection_info}")
                            
                            # Motor diagnostics - show actual motor status
                            try:
                                if self.left_motor and abs(left_speed) > 0.1:  # Only show when commanding movement
                                    left_actual_vel = self.left_motor.axis0.encoder.vel_estimate
                                    left_voltage = self.left_motor.axis0.motor.current_control.v_current_control_integral_q
                                    left_errors = self.left_motor.axis0.error
                                    left_direction = "FWD" if left_actual_vel > 0.1 else "REV" if left_actual_vel < -0.1 else "STOP"
                                    print(f"🔍 LEFT: Cmd:{left_speed:.1f} → Actual:{left_actual_vel:.1f} ({left_direction}) | V:{left_voltage:.1f} | Err:{left_errors}")
                                
                                if self.right_motor and abs(right_speed) > 0.1:  # Only show when commanding movement  
                                    right_actual_vel = self.right_motor.axis0.encoder.vel_estimate
                                    right_voltage = self.right_motor.axis0.motor.current_control.v_current_control_integral_q
                                    right_errors = self.right_motor.axis0.error
                                    right_direction = "FWD" if right_actual_vel > 0.1 else "REV" if right_actual_vel < -0.1 else "STOP"
                                    print(f"🔍 RIGHT: Cmd:{right_speed:.1f} → Actual:{right_actual_vel:.1f} ({right_direction}) | V:{right_voltage:.1f} | Err:{right_errors}")
                                    
                                # Check for direction mismatch (commanded forward but going backward)
                                if abs(left_speed) > 0.5 and abs(right_speed) > 0.5:  # Only for significant commands
                                    left_actual_vel = self.left_motor.axis0.encoder.vel_estimate if self.left_motor else 0
                                    right_actual_vel = self.right_motor.axis0.encoder.vel_estimate if self.right_motor else 0
                                    
                                    left_mismatch = (left_speed > 0.5 and left_actual_vel < -0.5) or (left_speed < -0.5 and left_actual_vel > 0.5)
                                    right_mismatch = (right_speed > 0.5 and right_actual_vel < -0.5) or (right_speed < -0.5 and right_actual_vel > 0.5)
                                    
                                    if left_mismatch or right_mismatch:
                                        print("⚠️  DIRECTION MISMATCH DETECTED!")
                                        if left_mismatch:
                                            print(f"   LEFT: Commanded {left_speed:.1f} but spinning {left_actual_vel:.1f}")
                                        if right_mismatch:
                                            print(f"   RIGHT: Commanded {right_speed:.1f} but spinning {right_actual_vel:.1f}")
                                        print("   💡 This suggests motor wiring or encoder issues")
                                    
                            except Exception as e:
                                print(f"🔍 Motor diagnostics error: {e}")
                            
                            # Debug: Show ranges being used
                            print(f"🔍 DEBUG: Steering {raw_steering}μs vs neutral {self.rc_controller.steering_neutral}μs "
                                  f"(range: {self.rc_controller.steering_min}-{self.rc_controller.steering_max}μs)")
                            
                            # Calculate what the steering should be manually for verification
                            if raw_steering >= self.rc_controller.steering_neutral:
                                expected_steering = (raw_steering - self.rc_controller.steering_neutral) / (self.rc_controller.steering_max - self.rc_controller.steering_neutral)
                            else:
                                expected_steering = (raw_steering - self.rc_controller.steering_neutral) / (self.rc_controller.steering_neutral - self.rc_controller.steering_min)
                            expected_steering = max(-1.0, min(1.0, expected_steering))
                            
                            # Determine steering direction for user clarity
                            if abs(steering) > 0.05:  # Only show if significant steering
                                if steering < 0:
                                    steer_direction = "LEFT"
                                    steer_explanation = "L-motor slower, R-motor faster"
                                else:
                                    steer_direction = "RIGHT" 
                                    steer_explanation = "L-motor faster, R-motor slower"
                            else:
                                steer_direction = "STRAIGHT"
                                steer_explanation = "Equal motor speeds"
                            
                            print(f"🔍 Expected steering (before deadzone): {expected_steering:.3f}")
                            print(f"🔍 Deadzone: {self.rc_controller.deadzone} | After deadzone: {steering:.3f}")
                            print(f"🎯 Steering: {steer_direction} ({steer_explanation})")
                            print(f"🔍 Speed Differential: L:{left_speed:.1f} - R:{right_speed:.1f} = {left_speed - right_speed:.1f}")
                            print("---")
                
                time.sleep(0.05)  # 20Hz control loop
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping...")
        finally:
            self.running = False
            self.set_motor_velocities(0, 0)
            GPIO.cleanup()
            print("👋 Push cart controller stopped")

def main():
    print("🚗 Raspberry Pi Zero 2 W Push Cart Controller")
    print("============================================")
    print("Features: USB Motors + MPU6050 + HOTRC DS600")
    
    controller = PushCartController()
    controller.run()

if __name__ == "__main__":
    main() 