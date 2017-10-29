nohup gdbserver :1234 ./gpsctl/gpsctl --? -p /dev/serial0 -b 96000 -B 19200  > ~/gpsctl/gpsctl.out 2>&1 &
