#!/usr/bin/python

import argparse
import serial

parser = argparse.ArgumentParser(description='aura link')
parser.add_argument('--hertz', default=10, type=int, help='specify main loop rate')
parser.add_argument('--flight', help='load specified flight log')
parser.add_argument('--export-text-tab', help='export to tab delimited file')
parser.add_argument('--serial', default='/dev/ttyUSB0', help='input serial port') 
parser.add_argument('--baud', default=115200, type=int, help='serial port baud rate') 
parser.add_argument('--websocket-port', help='websocket port')
parser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')
parser.add_argument('--no-real-time', help='run as fast as possible')

args = parser.parse_args()

dt = 1.0 / float(args.hertz)

try:
    ser = serial.Serial(args.serial, args.baud, timeout=dt)
except:
    print "Cannot open:", args.serial

while True:
    x = ser.read()

