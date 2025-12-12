#!/bin/bash
# Setup script to use ESP32-C6 USB2CAN adapter with Linux can-utils

PORT="/dev/ttyACM1"
INTERFACE="can0"
BITRATE="8"  # S6 = 500 kbps (S0=10k, S1=20k, S2=50k, S3=100k, S4=125k, S5=250k, S6=500k, S7=800k, S8=1M)

echo "ESP32-C6 USB2CAN - can-utils Setup"
echo "==================================="

# Check if can-utils is installed
if ! command -v slcand &> /dev/null; then
    echo "Error: can-utils not installed!"
    echo ""
    echo "Install with:"
    echo "  sudo apt-get install can-utils"
    exit 1
fi

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "Error: Port $PORT not found!"
    echo ""
    echo "Available ports:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  No USB serial ports found"
    exit 1
fi

echo "Using port: $PORT"
echo "Creating interface: $INTERFACE"
echo "Bitrate: S$BITRATE "
echo ""

# Kill existing slcand instances
echo "Stopping any existing slcand processes..."
sudo killall slcand 2>/dev/null

# Bring down interface if it exists
echo "Bringing down $INTERFACE if it exists..."
sudo ip link set down $INTERFACE 2>/dev/null

# Start slcand daemon
echo "Starting slcand daemon..."
sudo slcand -o -c -s$BITRATE $PORT $INTERFACE

if [ $? -ne 0 ]; then
    echo "Error: Failed to start slcand"
    exit 1
fi

sleep 1

# Bring up the interface
echo "Bringing up CAN interface..."
sudo ip link set up $INTERFACE

if [ $? -ne 0 ]; then
    echo "Error: Failed to bring up interface"
    sudo killall slcand
    exit 1
fi

echo ""
echo "✓ CAN interface ready!"
echo ""
echo "==================================="
echo "Usage Examples:"
echo "==================================="
echo ""
echo "1. Send a standard CAN frame:"
echo "   cansend $INTERFACE 123#DEADBEEF"
echo ""
echo "2. Send with 8 data bytes:"
echo "   cansend $INTERFACE 456#CAFEBABE12345678"
echo ""
echo "3. Monitor CAN traffic:"
echo "   candump $INTERFACE"
echo ""
echo "4. Monitor with timestamps:"
echo "   candump -t a $INTERFACE"
echo ""
echo "5. Generate test traffic (1 msg/sec):"
echo "   cangen $INTERFACE -v -v -g 1000"
echo ""
echo "6. Check interface statistics:"
echo "   ip -s -d link show $INTERFACE"
echo ""
echo "==================================="
echo "To stop the interface:"
echo "==================================="
echo "   sudo ip link set down $INTERFACE"
echo "   sudo killall slcand"
echo ""
