#!/bin/bash
# Quick connection script for ESP32-C6 USB-to-Serial adapter

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}ESP32-C6 USB-to-Serial Connection Helper${NC}"
echo "========================================"
echo ""

# Find available devices
DEVICES=$(ls /dev/ttyACM* 2>/dev/null)

if [ -z "$DEVICES" ]; then
    echo -e "${RED}Error: No /dev/ttyACM* devices found!${NC}"
    echo "Make sure your ESP32-C6 is connected via USB."
    exit 1
fi

echo -e "${YELLOW}Available devices:${NC}"
ls -l /dev/ttyACM*
echo ""

# Use first argument or default to first device
DEVICE="${1:-$(echo $DEVICES | awk '{print $1}')}"
BAUD="${2:-115200}"

if [ ! -e "$DEVICE" ]; then
    echo -e "${RED}Error: Device $DEVICE not found${NC}"
    exit 1
fi

echo -e "${GREEN}Connecting to: $DEVICE${NC}"
echo -e "${GREEN}Baud rate: $BAUD${NC}"
echo ""
echo "Choose connection method:"
echo "  1) screen (simple, recommended)"
echo "  2) minicom (full-featured)"
echo "  3) socat (raw passthrough)"
echo "  4) Python miniterm (with color)"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo -e "${YELLOW}Starting screen...${NC}"
        echo "To exit: Press Ctrl+A, then K, then Y"
        sleep 2
        screen "$DEVICE" "$BAUD"
        ;;
    2)
        echo ""
        echo -e "${YELLOW}Starting minicom...${NC}"
        echo "To exit: Press Ctrl+A, then X"
        sleep 2
        minicom -D "$DEVICE" -b "$BAUD"
        ;;
    3)
        echo ""
        echo -e "${YELLOW}Starting socat...${NC}"
        echo "To exit: Press Ctrl+C"
        sleep 2
        socat "$DEVICE",raw,echo=0,b"$BAUD" STDIN
        ;;
    4)
        echo ""
        echo -e "${YELLOW}Starting Python miniterm...${NC}"
        echo "To exit: Press Ctrl+]"
        sleep 2
        python3 -m serial.tools.miniterm "$DEVICE" "$BAUD"
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac
