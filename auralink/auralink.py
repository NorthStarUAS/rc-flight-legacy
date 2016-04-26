#!/usr/bin/python

import argparse
import os
import serial
import subprocess
import tempfile

from props import root, getNode

import parser
import telnet
import websocket

argparser = argparse.ArgumentParser(description='aura link')
argparser.add_argument('--hertz', default=10, type=int, help='specify main loop rate')
argparser.add_argument('--flight', help='load specified flight log')
argparser.add_argument('--export-text-tab', help='export to tab delimited file')
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

    while True:
        parser.serial_read(ser)
        telnet.update()
        websocket.update()
elif args.flight:
    (fd, filename) = tempfile.mkstemp()
    command = "zcat " + args.flight + " > " + filename
    print command
    os.system(command)

    try:
        fd = open(filename, 'r')
        full = fd.read()
        os.remove(filename)
    except:
        # eat the expected error
        print "we should be able to ignore the zcat error"

    print "len of decompressed file:", len(full)

    while True:
        parser.file_read(full)
        telnet.update()
        websocket.update()
else:
    print "No input source provided"    
