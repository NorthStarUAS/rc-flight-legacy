#!/usr/bin/python

import argparse
import os
import sys
import tempfile
from progress.bar import Bar

from props import root, getNode

sys.path.append("../src")
import comms.packer

import commands
import current
import parser

m2nm   = 0.0005399568034557235 # meters to nautical miles

def logical_category(id):
    if id == parser.GPS_PACKET_V1 or id == parser.GPS_PACKET_V2:
        return 'gps'
    elif id == parser.IMU_PACKET_V1 or id == parser.IMU_PACKET_V2 \
         or id == parser.IMU_PACKET_V3:
        return 'imu'
    elif id == parser.AIRDATA_PACKET_V3 or id == parser.AIRDATA_PACKET_V4 \
         or id == parser.AIRDATA_PACKET_V5:
        return 'air'
    elif id == parser.FILTER_PACKET_V1 or id == parser.FILTER_PACKET_V2:
        return 'filter'
    elif id == parser.ACTUATOR_PACKET_V1 or id == parser.ACTUATOR_PACKET_V2:
        return 'act'
    elif id == parser.PILOT_INPUT_PACKET_V1 \
         or id == parser.PILOT_INPUT_PACKET_V2:
        return 'pilot'
    elif id == parser.AP_STATUS_PACKET_V1 or id == parser.AP_STATUS_PACKET_V2 \
         or id == parser.AP_STATUS_PACKET_V3:
        return 'ap'
    elif id == parser.SYSTEM_HEALTH_PACKET_V2 \
         or id == parser.SYSTEM_HEALTH_PACKET_V3 \
         or id == parser.SYSTEM_HEALTH_PACKET_V4:
        return 'health'
    elif id == parser.PAYLOAD_PACKET_V1 or id == parser.PAYLOAD_PACKET_V2:
        return 'payload'

# When a binary record of some id is read, it gets parsed into the
# property tree structure.  The following code simple calls the
# appropriate text packer function for the given id to extract the
# same data back out of the property tree and format it as a text
# record.
def generate_record(category, index, delim=','):
    if category == 'gps':
        record = comms.packer.pack_gps_text(index, delim)
        return record
    elif category == 'imu':
        record = comms.packer.pack_imu_text(index, delim)
        return record
    elif category == 'air':
        record = comms.packer.pack_airdata_text(index, delim)
        return record
    elif category == 'filter':
        record = comms.packer.pack_filter_text(index, delim)
        return record
    elif category == 'act':
        record = comms.packer.pack_act_text(index, delim)
        return record
    elif category == 'pilot':
        record = comms.packer.pack_pilot_text(index, delim)
        return record
    elif category == 'ap':
        record = comms.packer.pack_ap_status_text(index, delim)
        return record
    elif category == 'health':
        record = comms.packer.pack_system_health_text(index, delim)
        return record
    elif category == 'payload':
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

    divs = 500
    size = len(full)
    chunk_size = size / divs
    threshold = chunk_size
    print 'len of decompressed file:', size

    bar = Bar('Parsing log file:', max = divs, suffix = '%(percent)d%% (%(eta)ds)')

    while True:
        try:
            (id, index, counter) = parser.file_read(full)
            current.compute_derived_data()
            category = logical_category(id)
            record = generate_record(category, index)
            key = '%s-%d' % (category, index)
            if key in data:
                data[key].append(record)
            else:
                data[key] = [ record ]
            if counter > threshold:
                threshold += chunk_size
                bar.next()
        except:
            bar.finish()
            print 'end of file'
            break
else:
    print 'A flight log file must be provided'

# last recorded time stamp
filter_node = getNode('/filters/filter', True)
status_node = getNode('/status', True)
total_time = filter_node.getFloat('timestamp')
apm2_node = getNode("/sensors/APM2", True)

for key in sorted(data):
    size = len(data[key])
    if total_time > 0.01:
        rate = size / total_time
    else:
        rate = 0.0
    print '%-10s %5.1f/sec (%6d records)' % (key, rate, size)
    f = open(key + '.txt', 'w')
    for line in data[key]:
        f.write(line + '\n')

print
print "Total log time: %.2f secs" % total_time
print "On board flight timer: %.1f min" % (status_node.getFloat('flight_timer') / 60.0)
print "Flight timer: %.1f min" % (status_node.getFloat('local_flight_timer') / 60.0)
print "Autopilot time: %.1f min" % (status_node.getFloat('local_autopilot_timer') / 60.0)
print "Distance flown: %.2fnm (%.2fkm)" % (status_node.getFloat('flight_odometer')*m2nm, status_node.getFloat('flight_odometer')*0.001)
print "Battery Usage: %.0f mah" % apm2_node.getInt("extern_current_mah")

