#!/bin/bash
cd ~/gpsctl
gcc -std=c11 -g -fno-omit-frame-pointer -funwind-tables -mapcs -mno-sched-prolog -o gpsctl gpsctl.c slightly_loony.c
