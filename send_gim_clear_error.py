#!/usr/bin/env python3
"""
Send GIM_CAN 0xAF (Clear Error) command to motor
Based on GIM_CAN Protocol V3.07b0 - Page 15
"""

import serial
import time
import sys

def send_slcan_command(ser, command):
    """Send SLCAN command and wait for response"""
    ser.write((command + '\r').encode())
    time.sleep(0.1)  # Increased for better reliability
    if ser.in_waiting:
        response = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
        return response
    return ""

def parse_slcan_response(slcan_data):
    """
    Parse SLCAN format CAN message
    Format: tIIILDD...
    - t/T = standard/extended frame
    - III = CAN ID (3 hex digits for standard)
    - L = data length
    - DD = data bytes (2 hex digits each)

    Returns: (can_id, data_bytes) or None if invalid
    """
    if not slcan_data or len(slcan_data) < 5:
        return None

    if slcan_data[0] not in ['t', 'T']:
        return None

    try:
        # Parse standard frame (t)
        if slcan_data[0] == 't':
            can_id = int(slcan_data[1:4], 16)
            dlc = int(slcan_data[4], 16)

            # Extract data bytes
            data_bytes = []
            pos = 5
            for _ in range(dlc):
                if pos + 2 <= len(slcan_data):
                    byte_val = int(slcan_data[pos:pos+2], 16)
                    data_bytes.append(byte_val)
                    pos += 2

            return (can_id, data_bytes)
        # Parse extended frame (T)
        elif slcan_data[0] == 'T':
            can_id = int(slcan_data[1:9], 16)
            dlc = int(slcan_data[9], 16)

            # Extract data bytes
            data_bytes = []
            pos = 10
            for _ in range(dlc):
                if pos + 2 <= len(slcan_data):
                    byte_val = int(slcan_data[pos:pos+2], 16)
                    data_bytes.append(byte_val)
                    pos += 2

            return (can_id, data_bytes)
    except (ValueError, IndexError):
        return None

    return None

def read_can_response(ser, timeout=0.5):
    """
    Read CAN response from serial port
    Returns list of parsed CAN messages: [(can_id, [data_bytes]), ...]
    """
    messages = []
    start_time = time.time()
    buffer = ""

    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
            buffer += chunk

            # Process complete messages (terminated by \r)
            while '\r' in buffer:
                line, buffer = buffer.split('\r', 1)
                parsed = parse_slcan_response(line.strip())
                if parsed:
                    messages.append(parsed)

        time.sleep(0.01)

    return messages

def send_gim_clear_error(ser, motor_id):
    """
    Send 0xAF Clear Error command to GIM motor

    According to protocol page 15:
    - Command code: 0xAF
    - DLC: 1 byte
    - Data: [0xAF]

    Response:
    - Command code: 0xAF
    - DLC: 2 bytes
    - Data: [0xAF, fault_status]
      where fault_status bits:
        Bit0: Voltage fault
        Bit1: Current fault
        Bit2: Temperature fault
        Bit3: Encoder fault
        Bit6: Hardware fault
        Bit7: Software fault
    """
    # Build SLCAN command: tIIILDD
    # III = CAN ID in hex (3 digits)
    # L = Data length (1)
    # DD = Data bytes (AF)
    cmd = f"t{motor_id:03X}1AF"

    print(f"\nSending Clear Error to Motor ID {motor_id} (0x{motor_id:03X})")
    print(f"SLCAN command: {cmd}")
    print("  - Command: 0xAF (Clear Fault)")
    print("  - DLC: 1 byte")

    response = send_slcan_command(ser, cmd)

    if 'z' in response or '\r' in response:
        print("✓ Command sent successfully")

        # Wait for motor response
        print("\nWaiting for motor response...")
        can_messages = read_can_response(ser, timeout=1.0)

        if can_messages:
            print(f"✓ Received {len(can_messages)} CAN message(s)")

            for can_id, data in can_messages:
                print(f"\n  CAN ID: 0x{can_id:03X}")
                print(f"  DLC: {len(data)} bytes")
                print(f"  Data: {' '.join(f'{b:02X}' for b in data)}")

                # Check if this is the response from our motor
                if can_id == motor_id and len(data) >= 2 and data[0] == 0xAF:
                    print("\n  ✓ This is the Clear Error response!")
                    fault_status = data[1]
                    print(f"  Fault Status: 0x{fault_status:02X}")

                    fault_description = decode_fault_status(fault_status)
                    if fault_status == 0x00:
                        print(f"  ✓ {fault_description} - Motor is healthy!")
                    else:
                        print(f"  ⚠ Active faults: {fault_description}")

                        # Show individual bits
                        print("\n  Fault breakdown:")
                        if fault_status & 0x01:
                            print("    - Bit0: Voltage fault")
                        if fault_status & 0x02:
                            print("    - Bit1: Current fault")
                        if fault_status & 0x04:
                            print("    - Bit2: Temperature fault")
                        if fault_status & 0x08:
                            print("    - Bit3: Encoder fault")
                        if fault_status & 0x40:
                            print("    - Bit6: Hardware fault")
                        if fault_status & 0x80:
                            print("    - Bit7: Software fault")
        else:
            print("⚠ No CAN response received from motor")
            print("  (Motor may not be connected or powered)")

        return True
    else:
        print("✗ Command send failed")
        return False

def decode_fault_status(fault_byte):
    """Decode fault status byte"""
    faults = []
    if fault_byte & 0x01:
        faults.append("Voltage fault")
    if fault_byte & 0x02:
        faults.append("Current fault")
    if fault_byte & 0x04:
        faults.append("Temperature fault")
    if fault_byte & 0x08:
        faults.append("Encoder fault")
    if fault_byte & 0x40:
        faults.append("Hardware fault")
    if fault_byte & 0x80:
        faults.append("Software fault")

    if not faults:
        return "No faults"
    return ", ".join(faults)

def main():
    # Configuration
    PORT = "/dev/ttyACM1"
    BAUDRATE = 115200
    MOTOR_ID = 0  # Change this if your motor has a different ID

    print("=" * 60)
    print("GIM Motor - Clear Error Command (0xAF)")
    print("GIM_CAN Protocol V3.07b0")
    print("=" * 60)

    try:
        # Open serial port
        print(f"\nOpening serial port {PORT}...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(0.5)
        print("✓ Port opened")

        # Open CAN channel
        print("\nOpening CAN channel...")
        response = send_slcan_command(ser, "O")
        if '\r' in response:
            print("✓ CAN channel opened")
        else:
            print("✗ Failed to open CAN channel")
            return

        # Clear any pending data
        ser.reset_input_buffer()

        # Send Clear Error command
        print("\n" + "=" * 60)
        success = send_gim_clear_error(ser, MOTOR_ID)
        print("=" * 60)

        if not success:
            print("\n⚠ Command failed to send properly")

        # Close CAN channel
        print("\nClosing CAN channel...")
        send_slcan_command(ser, "C")
        print("✓ CAN channel closed")

        # Close serial port
        ser.close()
        print("✓ Serial port closed")

        print("\n" + "=" * 60)
        print("Summary")
        print("=" * 60)
        if success:
            print("✓ Clear Error command completed successfully")
            print("\nNote: Check the CAN response above for fault status")
            print("      ESP32-C6 monitor will also show CAN TX/RX logs")
        else:
            print("✗ Command failed")
        print("=" * 60)

    except serial.SerialException as e:
        print(f"\n✗ Serial port error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        sys.exit(0)

if __name__ == "__main__":
    # Check if user wants to specify a different motor ID
    if len(sys.argv) > 1:
        try:
            MOTOR_ID = int(sys.argv[1])
            print(f"Using motor ID from command line: {MOTOR_ID}")
        except ValueError:
            print(f"Invalid motor ID: {sys.argv[1]}")
            print("Usage: ./send_gim_clear_error.py [motor_id]")
            sys.exit(1)

    main()
