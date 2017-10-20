#!/bin/bash
cat <(echo -n "./gpsctl/gpsctl ") remote_options.txt > run.cmd
ssh tom@10.2.5.101 "bash -s" < run.cmd
