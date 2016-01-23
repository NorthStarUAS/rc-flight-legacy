#!/bin/bash

HOME=~root
echo "starting ugear in $HOME"
cd $HOME
pwd
/usr/local/bin/ugear --remote-link on > /var/log/ugear.log
