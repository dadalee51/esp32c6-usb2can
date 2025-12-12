On the terminal run:

sudo slcan_attach -f -o /dev/ttyACM1
sudo ip link set can0 up type can bitrate 500000
candump can0

