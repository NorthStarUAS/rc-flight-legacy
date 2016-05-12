#!/bin/bash

# set fastest cpu performance possible
cpufreq-set -g performance

HOME=~root
echo "starting aura autopilot in $HOME"
cd $HOME
pwd
/usr/local/bin/aura --python_path $HOME/Source/aura-core/src --remote-link on > /var/log/aura.log
