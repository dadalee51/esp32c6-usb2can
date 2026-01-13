# esp32c6-usb2can

USB to CAN bus adapter using ESP32-C6 and SN65HVD230 transceiver

## Hardware

- **MCU**: ESP32-C6 (TWAI controller)
- **Transceiver**: SN65HVD230 CAN transceiver
- **Interface**: USB-C (USB Serial JTAG)
- **LED**: GPIO8 status indicator

### Pinout

```
ESP32-C6    SN65HVD230    Description
GPIO16   →  Pin 1 (D)     CAN TX
GPIO17   →  Pin 4 (R)     CAN RX
3.3V     →  Pin 3 (VCC)   Power
GND      →  Pin 2 (GND)   Ground
         →  Pin 8 (Rs)    10kΩ to GND (high-speed mode)
         →  Pin 7 (CANH)  CAN High (120Ω termination)
         →  Pin 6 (CANL)  CAN Low (120Ω termination)
```

### Required Components

- 100nF ceramic capacitor (VCC to GND, near transceiver)
- 10kΩ resistor (Rs to GND)
- 120Ω termination resistors (both bus ends)

## Features

- **Protocol**: SLCAN (Serial Line CAN)
- **Standards**: CAN 2.0A/2.0B
- **Bitrates**: 125k, 250k, 500k, 1M bps
- **Modes**: Normal, Listen-only
- **USB Device**: `/dev/ttyACM*` (Linux)
- **Frame Types**: Standard (11-bit), Extended (29-bit)

## Build

```bash
# Prerequisites: ESP-IDF installed
idf.py build
idf.py flash
idf.py monitor
```

## Usage

### Linux with can-utils

```bash
# Attach SLCAN device
sudo slcan_attach -f -o /dev/ttyACM0

# Configure CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Monitor CAN bus
candump can0

# Send CAN frame
cansend can0 123#DEADBEEF
```

### SLCAN Commands

| Command | Description |
|---------|-------------|
| `O`     | Open channel (normal mode) |
| `L`     | Open channel (listen-only) |
| `C`     | Close channel |
| `S0-S8` | Set bitrate (S4=125k, S5=250k, S6=500k, S8=1M) |
| `V`     | Get hardware version |
| `N`     | Get serial number |
| `tIIIDLL...LL` | Transmit standard frame |
| `TIIIIIIIIDLL...LL` | Transmit extended frame |

## Firmware

- **Version**: 2.0.0
- **Framework**: ESP-IDF
- **RTOS**: FreeRTOS
- **Tasks**: USB→CAN, CAN→USB, status monitor, LED blink

## Architecture

```
USB Host ↔ USB-C ↔ ESP32-C6 TWAI ↔ GPIO16/17 ↔ SN65HVD230 ↔ CANH/CANL ↔ CAN Bus
```

## Files

```
├── main/
│   └── main.c              # Main firmware (SLCAN, TWAI, USB Serial JTAG)
├── CMakeLists.txt          # ESP-IDF project config
├── sdkconfig               # ESP32-C6 configuration
├── SN65HVD230_wiring.txt   # Hardware wiring guide
└── note.md                 # Usage examples
```

## Troubleshooting

| Issue | Check |
|-------|-------|
| No communication | 120Ω termination resistors |
| Bus errors | Rs pin (10kΩ to GND) |
| Unreliable | 100nF decoupling capacitor |
| Random failures | Common GND between all devices |

## Documentation

**IMPORTANT**: This README.md is the primary project documentation. Any future changes to hardware, pinouts, features, or usage instructions MUST be reflected here immediately. Keep this file synchronized with all other documentation sources.

## License

Hardware design and firmware for ESP32-C6 USB-CAN adapter
