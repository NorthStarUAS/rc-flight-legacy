#!/bin/sh

# Goldy2 system:
#   FMU board = 192.168.143.130
#   Flight Computer (FMU sends packets too) = 192.168.143.11
#   Netmask = 255.255.255.0

IPADDR="192.168.143.11"
echo "Setting beaglebone eth0 ip for Goldy2 communication: $IPADDR"
ifconfig eth0 $IPADDR
