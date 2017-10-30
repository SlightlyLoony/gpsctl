nohup gdbserver :1234 ./gpsctl/gpsctl --version --help -p /dev/serial0 -e  > ~/gpsctl/gpsctl.out 2>&1 &
