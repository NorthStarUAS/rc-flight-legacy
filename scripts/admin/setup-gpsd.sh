#!/bin/sh

GPS_PORT=/dev/ttyUSB0
LOG_FILE=/var/log/dynamichome.log

if [ -c $GPS_PORT ]; then
    echo "Starting gpsd..."
    gpsd $GPS_PORT
    sleep 1
    echo "Starting dynamic home updater..."
    dynamichome >& $LOG_FILE 2>&1 &
else
    echo "No $GPS_PORT detected..."
fi
