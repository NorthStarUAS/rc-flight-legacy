#!/usr/bin/python

import argparse
import datetime
import serial

from props import root, getNode

import commands
import current
import auraparser
import httpserver
import telnet

argparser = argparse.ArgumentParser(description='aura link')
argparser.add_argument('--hertz', default=10, type=int, help='specify main loop rate')
argparser.add_argument('--serial', help='input serial port') 
argparser.add_argument('--baud', default=115200, type=int, help='serial port baud rate') 
argparser.add_argument('--telnet-port', default=5050, help='telnet port')
argparser.add_argument('--http-port', default=8888, help='http/ws port')
argparser.add_argument('--html-root', default='.')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')
argparser.add_argument('--no-real-time', help='run as fast as possible')

args = argparser.parse_args()

dt = 1.0 / float(args.hertz)

telnet.init(args.telnet_port)
httpserver.init(args.http_port, args.html_root)

if args.serial:
    try:
        ser = serial.Serial(args.serial, args.baud, timeout=dt)
    except:
        print "Cannot open:", args.serial
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        print "Available ports:"
        for p in ports:
            print p
        quit()

    d = datetime.datetime.utcnow()
    logfile = 'flight-' + d.strftime("%Y-%m-%d-%H:%M:%S") + '.log'
    try:
        f = open(logfile, 'wb')
    except:
        print "Cannot open:", logfile
        quite()
        
    while True:
        auraparser.serial_read(ser, f)
        current.compute_derived_data()
        commands.update(ser)
        telnet.update()
        httpserver.update()
else:
    print "A serial port must be provided"    
