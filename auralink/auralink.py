#!/usr/bin/python

import argparse
import os
import serial
import subprocess
import tempfile

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

def glean_ascii_message(c):
    pass

START_OF_MSG0 = 147
START_OF_MSG1 = 224

state = 0
pkt_id = 0
pkt_len = 0
counter = 0
cksum_A = 0
cksum_B = 0
cksum_lo = 0
cksum_hi = 0
payload = ''

def serial_update():
    len = 0
    input = ''
    msg_id = -1
    print"enter update(), state:", state
    if state == 0:
        counter = 0
        cksum_A = 0
        cksum_B = 0
        input = serial.read(1)
        while len(input) and input[0] != START_OF_MSG0:
            print " state0 val:", ord(input[0])
            glean_ascii_msgs(input[0])
            input = serial.read(1)
        if len(input) and input[0] == START_OF_MSG0:
            print " read START_OF_MSG0"
            state += 1
    if state == 1:
        input = serial.read(1)
        if len(input):
            if input[0] == START_OF_MSG1:
                print " read START_OF_MSG1"
                state += 1
            elif input[0] == START_OF_MSG0:
                print " read START_OF_MSG0"
            else:
                state = 0
    if state == 2:
        input = serial.read(1)
        if len(input):
            pkt_id = input[0]
            cksum_A = (cksum_A + input[0]) & 0xff
            cksum_B = (cksum_B + cksum_A) & 0xff
            print " pkt_id:", ord(pkt_id)
            state += 1
    if state == 3:
        input = serial.read(1)
        if len(input):
            pkt_len = input[0]
            print " pkt_len:", ord(pkt_len)
            print " payload =",
            cksum_A = (cksum_A + input[0]) & 0xff
            cksum_B = (cksum_B + cksum_A) & 0xff
            state += 1
    if state == 4:
        input = serial.read(1)
        while len(input):
            counter += 1
            payload += input[0]
            print "%02X" % input[0],
            cksum_A = (cksum_A + input[0]) & 0xff
            cksum_B = (cksum_B + cksum_A) & 0xff
            if counter >= pkt_len:
                state += 1
                print ""
                break
            input = serial.read(1)
    if state == 5:
        input = serial.read(1)
        if len(input):
            cksum_lo = input[0]
            print " cksum_lo:", ord(cksum_lo)
            state += 1
    if state == 6:
        input = serial.read(1)
        if len(input):
            cksum_hi = input[0]
            print " cksum_hi:", ord(cksum_hi)
            if ord(cksum_A) == ord(cksum_lo) and ord(cksum_B) == ord(cksum_hi):
                print "checksum passes:", ord(pkt_id)
                parse_msg(pkt_id, payload)
                msg_id = pkt_id
            else:
                print "pkt=%d checksum failed %d %d (computed) != %d %d (message)\n" % (pkt_id, cksum_A, cksum_B, cksum_lo, cksum_hi)
            # this is the end of a record, reset state to 0 to start
            # looking for next record
            state = 0

    print "exit routine, msg_id:", ord(msg_id)
    return msg_id


def validate_cksum(id, size, buf, cksum0, cksum1):
    c0 = 0
    c1 = 0

    c0 = (c0 + id) & 0xff
    c1 = (c1 + c0) & 0xff
    # print "c0 =", c0, "c1 =", c1

    c0 = (c0 + size) & 0xff
    c1 = (c1 + c0) & 0xff
    # print "c0 =", c0, "c1 =", c1

    for i in range(0, size):
        c0 = (c0 + ord(buf[i])) & 0xff
        c1 = (c1 + c0) & 0xff
        # print "c0 =", c0, "c1 =", c1, '[', ord(buf[i]), ']'

    # print "c0 =", c0, "(", cksum0, ")", "c1 =", c1, "(", cksum1, ")"

    if c0 == cksum0 and c1 == cksum1:
        return True
    else:
        return False

def gzip_update(buf):
    global counter
    
    savebuf = ''
    print "gzip_update()"

    myeof = False

    # scan for sync characters
    sync0 = ord(buf[counter]); counter += 1
    sync1 = ord(buf[counter]); counter += 1
    while (sync0 != START_OF_MSG0 or sync1 != START_OF_MSG1) and counter < len(buf):
        sync0 = sync1
        sync1 = ord(buf[counter]); counter += 1
        print "scanning for start of message:", counter, sync0, sync1

    print "found start of message ..."

    # read message id and size
    id = ord(buf[counter]); counter += 1
    size = ord(buf[counter]); counter += 1
    print "message =", id, "size =", size

    # load message
    try:
        savebuf = buf[counter:counter+size]; counter += size
    except:
	print "ERROR: didn't read enough bytes!"

    # read checksum
    cksum0 = ord(buf[counter]); counter += 1
    cksum1 = ord(buf[counter]); counter += 1
    
    if validate_cksum(id, size, savebuf, cksum0, cksum1):
        print "check sum passed"
        #parse_msg( id, savebuf, gpspacket, imupacket, airpacket, filterpacket,
	#	   actpacket, pilotpacket, appacket, healthpacket,
	#	   payloadpacket )
        return id

    print "Check sum failure!"
    return -1

#while True:
#    serial_update()


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
    gzip_update(full)
