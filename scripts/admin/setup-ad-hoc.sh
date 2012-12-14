#!/bin/sh

function tryhard {
   COMMAND="$*"
   echo "Running $COMMAND"
   $COMMAND
   while [ $? != 0 ]; do
       sleep 1
       echo "Running $COMMAND"
       $COMMAND
   done
}

tryhard ifconfig wlan0 down

TYPE=`iw wlan0 info | grep type | awk '{print $2}'`
while [ $TYPE != "IBSS" ]; do
    echo "type = $TYPE"
    tryhard iw wlan0 set type ibss
    sleep 1
    TYPE=`iw wlan0 info | grep type | awk '{print $2}'`
done

IP=`ifconfig wlan0 | grep "inet addr"`
while [ "x$IP" = "x" ]; do
    echo "ip = $IP"
    tryhard ifconfig wlan0 192.168.2.2 netmask 255.255.255.0
    sleep 1
    IP=`ifconfig wlan0 | grep "inet addr"`
done

tryhard ifconfig wlan0 up
tryhard iw wlan0 ibss join ATI-adhoc 2412

