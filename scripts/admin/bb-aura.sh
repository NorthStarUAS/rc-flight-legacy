#!/bin/bash

HOME=~root
echo "starting aura autopilot in $HOME"
cd $HOME
pwd
/usr/local/bin/aura --remote-link on > /var/log/aura.log
