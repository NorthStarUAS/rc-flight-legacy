#!/bin/bash

# set fastest cpu performance possible
cpufreq-set -g performance

# nice range is -20 to 20 with -20 being the highest priority.  -10 should be
# a good balanced number.
NICE=-10

HOME=~root

echo "starting aura autopilot in $HOME"
cd $HOME
nice -n $NICE $HOME/bin/aura --python_path $HOME/src/aura-core/src --remote-link on > /var/log/aura.log
