nohup gdbserver :1234 ./gpsctl/gpsctl -p /dev/serial0 -va -b 9600 -a -e 2  > ~/gpsctl/gpsctl.out 2>&1 &
