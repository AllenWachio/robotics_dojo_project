#!/usr/bin/env python3
"""
Arduino Serial Monitor
Continuously listens to Arduino communication and allows sending commands.
Press Ctrl+C to exit.
"""

import serial
import time
import sys
import select

def arduino_monitor():
    try:
        # Connect to Arduino
        ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 57600, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        print("=== Arduino Serial Monitor ===")
        print("Connected to Arduino via device ID at 57600 baud")
        print("Commands you can try:")
        print("  e  - Request encoder values")
        print("  r  - Reset encoders")
        print("  s  - Get robot state")
        print("  i  - Get IMU data")
        print("  m  - Send motor command (format: m left:right:left:right)")
        print("Press Ctrl+C to exit")
        print("=" * 40)
        
        # Clear any startup messages
        time.sleep(1)
        while ser.in_waiting > 0:
            startup_msg = ser.readline()
            print(f"[STARTUP] {startup_msg.decode('utf-8', errors='ignore').strip()}")
        
        print("\n[READY] Arduino initialized. You can type commands or just monitor:")
        
        while True:
            # Check for incoming data from Arduino
            if ser.in_waiting > 0:
                try:
                    data = ser.readline()
                    decoded = data.decode('utf-8', errors='ignore').strip()
                    if decoded:
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"[{timestamp}] Arduino: {decoded}")
                except Exception as e:
                    print(f"[ERROR] Decoding Arduino data: {e}")
            
            # Check for user input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip()
                if user_input:
                    try:
                        # Send command to Arduino
                        command = user_input + '\r'
                        ser.write(command.encode())
                        ser.flush()
                        print(f"[SENT] {user_input}")
                    except Exception as e:
                        print(f"[ERROR] Sending command: {e}")
            
            time.sleep(0.05)  # Small delay to prevent excessive CPU usage
            
    except KeyboardInterrupt:
        print("\n[EXIT] Monitoring stopped by user")
    except serial.SerialException as e:
        print(f"[ERROR] Serial connection error: {e}")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
    finally:
        try:
            ser.close()
            print("[INFO] Serial port closed")
        except:
            pass

if __name__ == "__main__":
    arduino_monitor()