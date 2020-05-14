#!/bin/bash

RUNAS=aura
HOME=`eval echo ~$RUNAS`
SOURCE=$HOME/Source
RUNLOG=`mktemp -u -p $HOME -t auralog.XXXX`

# nice range is -20 to 20 with -20 being the highest priority.  -10 should be
# a good balanced number.  -20 could be a little selfish, but that might be
# exactly what we want.
NICE=-20

# wait for devices to exist (commented out by default)
#
#while [ ! -e /dev/ttyUSB0 ]; do
#	sleep 1
#done
#while [ ! -e /dev/ttyACM0 ]; do
#	sleep 1
#done
#sleep 5

nice -n $NICE sudo -i -u $RUNAS -g dialout bash << EOF

echo "Running aura autopilot as $RUNAS"
echo "Source: $SOURCE"
echo "Runlog: $RUNLOG"

$SOURCE/aura-core/build/src/aura --python_path $SOURCE/aura-core/src --config $SOURCE/aura-config/config > $RUNLOG

EOF
