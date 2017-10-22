nohup gdbserver :1234 ./gpsctl/gpsctl -v -b 115200 -p /dev/serial0 -s  > ~/gpsctl/gpsctl.out 2>&1 &
