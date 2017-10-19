#!/bin/bash
cat <(echo -n "nohup gdbserver :1234 ./gpsctl/gpsctl ") remote_options.txt <(echo "  > ~/gpsctl/gpsctl.out 2>&1 &") > gdbserver.cmd
ssh tom@10.2.5.101 "bash -s" < gdbserver.cmd
