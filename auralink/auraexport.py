#!/usr/bin/python

import argparse
import os
import sys
import tempfile

from props import root, getNode

sys.path.append("../src")
import comms.packer

import commands
import current
import parser


def generate_record(id, index, delim=','):
    if id == parser.GPS_PACKET_V1 or id == parser.GPS_PACKET_V2:
        record = comms.packer.pack_gps_text(index, delim)
        return record
    elif id == parser.IMU_PACKET_V1 or id == parser.IMU_PACKET_V2 or id == parser.IMU_PACKET_V3:
        record = comms.packer.pack_imu_text(index, delim)
        return record
    elif id == parser.AIRDATA_PACKET_V3 or id == parser.AIRDATA_PACKET_V4 or id == parser.AIRDATA_PACKET_V5:
        record = comms.packer.pack_airdata_text(index, delim)
        return record
    elif id == parser.FILTER_PACKET_V1 or id == parser.FILTER_PACKET_V2:
        record = comms.packer.pack_filter_text(index, delim)
        return record
    elif id == parser.ACTUATOR_PACKET_V1 or id == parser.ACTUATOR_PACKET_V2:
        record = comms.packer.pack_act_text(index, delim)
        return record
    elif id == parser.PILOT_INPUT_PACKET_V1 or id == parser.PILOT_INPUT_PACKET_V2:
        record = comms.packer.pack_pilot_text(index, delim)
        return record
    elif id == parser.AP_STATUS_PACKET_V1 or id == parser.AP_STATUS_PACKET_V2 or id == parser.AP_STATUS_PACKET_V3:
        record = comms.packer.pack_ap_status_text(index, delim)
        return record
    elif id == parser.SYSTEM_HEALTH_PACKET_V2 or id == parser.SYSTEM_HEALTH_PACKET_V3 or id == parser.SYSTEM_HEALTH_PACKET_V4:
        record = comms.packer.pack_system_health_text(index, delim)
        return record
    elif id == parser.PAYLOAD_PACKET_V1 or id == parser.PAYLOAD_PACKET_V2:
        record = comms.packer.pack_payload_text(index, delim)
        return record
    # remote_link_node.setInt('sequence_num', result[14]) # include in system health....


argparser = argparse.ArgumentParser(description='aura export')
argparser.add_argument('--flight', help='load specified flight log')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')

args = argparser.parse_args()

data = {}

if args.flight:
    filename = args.flight
    if args.flight.endswith('.gz'):
        (fd, filename) = tempfile.mkstemp()
        command = 'zcat ' + args.flight + ' > ' + filename
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
        print 'we should be able to ignore the zcat error'

    print 'len of decompressed file:', len(full)

    while True:
        try:
            (id, index) = parser.file_read(full)
            key = '%d-%d' % (id, index)
            record = generate_record(id, index)
            if key in data:
                data[key].append(record)
            else:
                data[key] = [ record ]
        except:
            print 'end of file'
            break
else:
    print 'A flight log file must be provided'

for key in sorted(data):
    print key, ':', len(data[key])

