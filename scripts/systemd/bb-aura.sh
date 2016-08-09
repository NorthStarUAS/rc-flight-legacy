#!/bin/bash

# set fastest cpu performance possible
cpufreq-set -g performance

HOME=~root
echo "starting aura autopilot in $HOME"
cd $HOME
pwd
nice -n -10 /root/bin/aura --python_path $HOME/src/aura-core/src --remote-link on > /var/log/aura.log
