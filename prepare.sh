#!/bin/bash
scp /Users/tom/CLionProjects/gpsctl/*.c tom@10.2.5.101:~/gpsctl
scp /Users/tom/CLionProjects/gpsctl/*.h tom@10.2.5.101:~/gpsctl
ssh tom@10.2.5.101 "bash -s" < build.sh
scp tom@10.2.5.101:~/gpsctl/gpsctl .
