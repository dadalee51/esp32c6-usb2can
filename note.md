On the terminal run:

sudo slcan_attach -f -o /dev/ttyACM0
sudo ip link set can0 up type can bitrate 1000000
candump can0


#cansend can0 002#AF - doesnt work yet.