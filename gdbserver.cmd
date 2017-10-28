nohup gdbserver :1234 ./gpsctl/gpsctl --? -p /dev/serial0 -b 9600 -B19200 "this is a test" test1 -vvva test2  > ~/gpsctl/gpsctl.out 2>&1 &
