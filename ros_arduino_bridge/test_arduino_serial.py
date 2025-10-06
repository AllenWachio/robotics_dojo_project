#!/usr/bin/env python3
"""
Test script to diagnose Arduino serial communication issues.
This helps identify baud rate mismatches and corrupted data.
"""

import serial
import time
import sys

def test_arduino_serial():
    """Test Arduino serial communication"""
    
    # Configuration
    serial_port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    baud_rates = [9600, 57600, 115200]  # Common baud rates to test
    
    print("=" * 60)
    print("Arduino Serial Communication Test")
    print("=" * 60)
    
    for baud_rate in baud_rates:
        print(f"\n--- Testing Baud Rate: {baud_rate} ---")
        
        try:
            # Open serial connection
            ser = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=2.0,
                write_timeout=2.0,
            )
            
            print(f"✓ Connected to {serial_port} at {baud_rate} baud")
            time.sleep(2)  # Wait for Arduino reset
            
            # Clear buffer
            ser.reset_input_buffer()
            time.sleep(0.5)
            
            # Test 1: Check for any incoming data
            print("\nTest 1: Checking for incoming data (5 seconds)...")
            start = time.time()
            data_received = False
            
            while time.time() - start < 5:
                if ser.in_waiting > 0:
                    try:
                        raw_data = ser.readline()
                        decoded = raw_data.decode('utf-8', errors='replace').strip()
                        print(f"  Raw bytes: {raw_data}")
                        print(f"  Decoded: '{decoded}'")
                        data_received = True
                    except Exception as e:
                        print(f"  Error reading: {e}")
                        print(f"  Raw bytes: {ser.read(ser.in_waiting)}")
                time.sleep(0.1)
            
            if not data_received:
                print("  ✗ No data received in 5 seconds")
            
            # Test 2: Send commands and check responses
            print("\nTest 2: Testing Arduino commands...")
            
            commands = [
                ("r", "Reset encoders"),
                ("e", "Read encoders"),
                ("v", "Read robot state"),
            ]
            
            for cmd, description in commands:
                print(f"\n  Sending '{cmd}' ({description})...")
                ser.reset_input_buffer()
                ser.write((cmd + "\r").encode())
                ser.flush()
                time.sleep(0.2)
                
                if ser.in_waiting > 0:
                    try:
                        response = ser.readline().decode('utf-8', errors='replace').strip()
                        print(f"    Response: '{response}'")
                    except Exception as e:
                        print(f"    Error: {e}")
                else:
                    print(f"    No response")
            
            ser.close()
            print(f"\n✓ Test completed for {baud_rate} baud")
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
    
    print("\n" + "=" * 60)
    print("Test Complete!")
    print("=" * 60)
    print("\nInterpretation:")
    print("  - If you see readable text at a specific baud rate, use that rate")
    print("  - If you see garbage at all rates, check your Arduino code")
    print("  - If 0x80 bytes appear, Arduino might be sending binary data")
    print("  - If no data appears, Arduino might not be running or hung")

if __name__ == "__main__":
    try:
        test_arduino_serial()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
