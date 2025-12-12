#!/bin/bash
# ESP-IDF Environment Setup Script for ESP32-C6 USB2CAN Project

# Set ESP-IDF path (adjust if your installation is different)
export IDF_PATH=~/esp/v5.5/esp-idf

# Source the ESP-IDF export script
if [ -f "$IDF_PATH/export.sh" ]; then
    echo "Setting up ESP-IDF environment from $IDF_PATH"
    . $IDF_PATH/export.sh
    echo "ESP-IDF environment ready!"
    echo "You can now use: idf.py set-target esp32c6"
else
    echo "Error: ESP-IDF not found at $IDF_PATH"
    echo "Please adjust IDF_PATH in this script to match your installation"
    exit 1
fi
