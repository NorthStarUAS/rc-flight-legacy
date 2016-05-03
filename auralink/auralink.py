#!/usr/bin/python

import argparse
import datetime
import serial

from props import root, getNode

import commands
import current
import parser
import telnet
import websocket

argparser = argparse.ArgumentParser(description='aura link')
argparser.add_argument('--hertz', default=10, type=int, help='specify main loop rate')
argparser.add_argument('--serial', help='input serial port') 
argparser.add_argument('--baud', default=115200, type=int, help='serial port baud rate') 
argparser.add_argument('--telnet-port', default=5050, help='telnet port')
argparser.add_argument('--websocket-port', default=8888, help='websocket port')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')
argparser.add_argument('--no-real-time', help='run as fast as possible')

args = argparser.parse_args()

dt = 1.0 / float(args.hertz)

telnet.init(args.telnet_port)
websocket.init(args.websocket_port)

if args.serial:
    try:
        ser = serial.Serial(args.serial, args.baud, timeout=dt)
    except:
        print "Cannot open:", args.serial
        quit()

    d = datetime.datetime.utcnow()
    logfile = 'flight-' + d.strftime("%Y-%m-%d-%H:%M:%S") + '.log'
    try:
        f = open(logfile, 'wb')
    except:
        print "Cannot open:", logfile
        quite()
        
    while True:
        parser.serial_read(ser, f)
        current.compute_derived_data()
        commands.update(ser)
        telnet.update()
        websocket.update()
else:
    print "A serial port must be provided"    
