#!/usr/bin/env python3
"""
SLCAN Protocol Test Script for ESP32-C6 USB2CAN Adapter

This script tests the SLCAN protocol implementation by sending
various commands and checking responses.

Usage:
    python3 test_slcan.py /dev/ttyACM0
"""

import sys
import serial
import time

def send_command(ser, cmd):
    """Send SLCAN command and wait for response"""
    print(f">> Sending: {cmd}")
    ser.write(cmd.encode() + b'\r')
    time.sleep(0.1)

    response = b''
    while ser.in_waiting > 0:
        response += ser.read(ser.in_waiting)
        time.sleep(0.01)

    if response:
        # Check for different response types
        if response == b'\r':
            print(f"<< OK (CR)")
            return True
        elif response == b'\x07':
            print(f"<< ERROR (BELL)")
            return False
        else:
            print(f"<< Response: {response}")
            return True
    else:
        print(f"<< No response")
        return False

def test_slcan(port):
    """Test SLCAN protocol on specified port"""
    print(f"Opening {port}...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1,
            write_timeout=1
        )
    except Exception as e:
        print(f"Error opening port: {e}")
        return False

    print(f"Port opened successfully\n")

    # Wait a moment for device to settle
    time.sleep(0.5)

    # Clear any startup messages
    if ser.in_waiting > 0:
        startup_data = ser.read(ser.in_waiting)
        print(f"Startup data received ({len(startup_data)} bytes):")
        try:
            print(startup_data.decode('utf-8', errors='replace'))
        except:
            print(startup_data.hex())
        print()

    # Test 1: Get version
    print("=== Test 1: Get Hardware Version ===")
    send_command(ser, 'V')
    print()

    # Test 2: Get serial number
    print("=== Test 2: Get Serial Number ===")
    send_command(ser, 'N')
    print()

    # Test 3: Close channel (should already be open with auto-start)
    print("=== Test 3: Close Channel ===")
    send_command(ser, 'C')
    print()

    # Test 4: Set bitrate (channel must be closed)
    print("=== Test 4: Set Bitrate to 500kbps (S6) ===")
    send_command(ser, 'S6')
    print()

    # Test 5: Open channel
    print("=== Test 5: Open Channel ===")
    send_command(ser, 'O')
    print()

    # Test 6: Try to set bitrate while open (should fail)
    print("=== Test 6: Try Set Bitrate While Open (Should Fail) ===")
    send_command(ser, 'S6')
    print()

    # Test 7: Send a test CAN frame (ID 0x123, 8 bytes of data)
    print("=== Test 7: Send CAN Frame (ID=0x123, Data=01 02 03 04 05 06 07 08) ===")
    send_command(ser, 't12380102030405060708')
    print()

    # Test 8: Listen for incoming CAN frames
    print("=== Test 8: Listen for Incoming CAN Frames (5 seconds) ===")
    print("Waiting for CAN messages...")
    start_time = time.time()
    msg_count = 0

    while time.time() - start_time < 5.0:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            try:
                msg = data.decode('utf-8', errors='replace')
                print(f"<< CAN Message: {msg.strip()}")
                msg_count += 1
            except:
                print(f"<< Raw data: {data.hex()}")
        time.sleep(0.1)

    print(f"Received {msg_count} CAN messages\n")

    # Test 9: Close channel
    print("=== Test 9: Close Channel ===")
    send_command(ser, 'C')
    print()

    ser.close()
    print("Test completed successfully!")
    return True

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_slcan.py <serial_port>")
        print("Example: python3 test_slcan.py /dev/ttyACM0")
        sys.exit(1)

    port = sys.argv[1]
    test_slcan(port)

if __name__ == "__main__":
    main()
