#!/usr/bin/python3

import argparse
import serial

from props import root, getNode

import commands
import current
import auraparser
import httpserver
import joystick
import requests
import telnet

argparser = argparse.ArgumentParser(description='aura link')
argparser.add_argument('--hertz', default=10, type=int, help='specify main loop rate')
argparser.add_argument('--serial', required=True, help='input serial port') 
argparser.add_argument('--baud', default=115200, type=int, help='serial port baud rate') 
argparser.add_argument('--telnet-port', default=5050, help='telnet port')
argparser.add_argument('--http-port', default=8888, help='http/ws port')
argparser.add_argument('--html-root', default='.')

args = argparser.parse_args()

dt = 1.0 / float(args.hertz)

telnet.init(args.telnet_port)
httpserver.init(args.http_port, args.html_root)

try:
    ser = serial.Serial(args.serial, args.baud, timeout=dt)
except:
    print("Cannot open:", args.serial)
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    print("Available ports:")
    for p in ports:
        print(p)
    quit()
    
auraparser.init()

while True:
    auraparser.update(ser)
    current.compute_derived_data()
    joystick.update()
    requests.gen_requests()
    commands.update(ser)
    telnet.update()
    httpserver.update()
