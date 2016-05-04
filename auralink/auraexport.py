#!/usr/bin/python

import argparse
import os
import tempfile

from props import root, getNode

import commands
import current
import parser

def generate_record(id, index):
    if id == parser.GPS_PACKET_V1 or id == parser.GPS_PACKET_V2:
        gps_node = getNode('/sensors/gps[%d]' % index, True)
        record = "%.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%f\t%d\t%d\n" % \
		 (gps_node.getFloat('timestamp'),
		  gps_node.getFloat('latitude_deg'),
                  gps_node.getFloat('longitude_deg'),
                  gps_node.getFloat('altitude_m'),
		  gps_node.getFloat('vn_ms'),
                  gps_node.getFloat('ve_ms'),
                  gps_node.getFloat('vd_ms'),
		  gps_node.getFloat('unix_time_sec'),
                  gps_node.getInt('satellites'),
                  gps_node.getInt('status')
                 )
        return record


argparser = argparse.ArgumentParser(description='aura export')
argparser.add_argument('--flight', help='load specified flight log')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')

args = argparser.parse_args()

data = {}

if args.flight:
    filename = args.flight
    if args.flight.endswith('.gz'):
        (fd, filename) = tempfile.mkstemp()
        command = "zcat " + args.flight + " > " + filename
        print command
        os.system(command)
    try:
        fd = open(filename, 'r')
        full = fd.read()
        if args.flight.endswith('.gz'):
            # remove temporary file name
            os.remove(filename)
    except:
        # eat the expected error
        print "we should be able to ignore the zcat error"

    print "len of decompressed file:", len(full)

    while True:
        #try:
            (id, index) = parser.file_read(full)
            key = '%d-%d' % (id, index)
            record = generate_record(id, index)
            if key in data:
                data[key] += 1
            else:
                data[key] = 1
        #except:
        #    print "end of file"
        #    break
else:
    print "A flight log file must be provided"

for key in sorted(data):
    print key, ":", data[key]

