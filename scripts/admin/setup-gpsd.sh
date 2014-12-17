#!/bin/sh

#GPS_PORT=/dev/ttyUSB0
GPS_PORT=/dev/ttyO0

LOG_FILE=/var/log/autohome.log

if [ -c $GPS_PORT ]; then
    echo "Starting gpsd..."
    gpsd $GPS_PORT
    sleep 1
    echo "Starting automatic home updater..."
    autohome.py >& $LOG_FILE 2>&1 &
else
    echo "No $GPS_PORT detected..."
fi
