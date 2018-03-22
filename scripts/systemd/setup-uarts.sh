#!/bin/sh

# UART1 Beaglebone Black
config-pin p9.24 uart
config-pin p9.26 uart
stty -F /dev/ttyS1 sane

# UART2 Beaglebone Black
config-pin p9.21 uart
config-pin p9.22 uart
stty -F /dev/ttyS1 sane

# UART4 Beaglebone Black
config-pin p9.11 uart
config-pin p9.13 uart
stty -F /dev/ttyS4 sane
