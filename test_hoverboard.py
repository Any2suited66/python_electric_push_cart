#!/usr/bin/env python3
"""
Simple Hoverboard UART Test
Tests basic communication with the hoverboard motor controller
"""

import serial
import struct
import time

def test_hoverboard_connection():
    """Test basic hoverboard UART connection"""
    print("Hoverboard UART Connection Test")
    print("=" * 40)
    
    try:
        # Try to connect to hoverboard
        ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
        print(f"✓ Connected to hoverboard on {ser.port} at {ser.baudrate} baud")
        
        # Test 1: Send a simple stop command
        print("\nTest 1: Sending stop command...")
        stop_command = struct.pack('<HhhH', 0xABCD, 0, 0, 0xABCD)
        ser.write(stop_command)
        ser.flush()
        print("✓ Stop command sent")
        
        # Test 2: Send a forward command
        print("\nTest 2: Sending forward command...")
        forward_command = struct.pack('<HhhH', 0xABCD, 0, 100, 0xABCD ^ 100)
        ser.write(forward_command)
        ser.flush()
        print("✓ Forward command sent")
        
        # Test 3: Send a stop command again
        print("\nTest 3: Sending stop command again...")
        ser.write(stop_command)
        ser.flush()
        print("✓ Stop command sent")
        
        ser.close()
        print("\n✓ All tests completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == "__main__":
    test_hoverboard_connection()

