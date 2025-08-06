#!/bin/bash
cd /home/ysc/dros/rl_deploy/bin
sleep 12
chrt 90 taskset -c 4,5,6,7 ./rl_deploy
