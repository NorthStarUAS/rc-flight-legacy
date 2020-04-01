# logging.py

import gzip
import os
import random
import re
import socket

from props import getNode
import props_json

from comms.packer import packer
import comms.serial_parser

# global variables for data file logging
log_buffer = []
fdata = None

logging_node = getNode('/config/logging')

enable_file = False             # log to file enabled/disabled
enable_udp = False              # log to a udp port enabled/disabled
log_path = ''
flight_dir = ''                 # dir containing all our logged data

# remote logging support
sock = None
udp_port = 6550
udp_host = "127.0.0.1"

START_OF_MSG0 = 147
START_OF_MSG1 = 224

# scan the base path for fltNNNN directories.  Return the biggest
# flight number
def max_flight_num():
    max = -1
    if not os.path.isdir( log_path ):
        print('Flight data directory does not exist:', log_path)
        return max
    p = re.compile('flt(\d+)$')
    for f in os.listdir( log_path ):
        dir = os.path.join(log_path, f)
        if os.path.isdir(dir):
            m = p.search(dir)
            if m:
                val = int(m.group(1))
                if val > max: max = val
    return max

def init_file_logging():
    global enable_file
    global fdata
    global flight_dir
    
    print('Log path:', log_path)

    # find the biggest flight number logged so far
    max = max_flight_num()
    print('Max log dir index:', max)

    # make the new logging directory
    new_dir = 'flt%05d' % (max + 1)
    flight_dir = os.path.join(log_path, new_dir)
    try:
        print('Creating log dir:', flight_dir)
        os.mkdir(flight_dir)
        logging_node.setString('flight_dir', flight_dir)
    except:
        print('Error creating:', flight_dir)
        return False

    # open the logging files
    file = os.path.join(flight_dir, 'flight.dat.gz')
    try:
        fdata = gzip.open(file, 'wb')
    except:
        print('Cannot open:', file)
        return False

    return True

def init_udp_logging():
    global sock
    try:
        # open a UDP socket
        sock = socket.socket(socket.AF_INET,    # Internet
                             socket.SOCK_DGRAM) # UDP
    except:
        print('Error opening logging socket')
        return False
    return True

def init():
    global log_path
    global udp_host
    global udp_port
    global enable_file
    global enable_udp
    
    log_path = logging_node.getString('path')
    udp_host = logging_node.getString('hostname')
    udp_port = logging_node.getInt('port')
    
    if log_path != '':
        if init_file_logging():
            enable_file = True      # success
        # fixme:
        # events->open(flight_dir.c_str())
        # events->log("Log", "Start")
    if udp_host != '' and udp_port > 0:
        if init_udp_logging():
            enable_udp = True
    return True

# write all pending data and flush
def write_messages():
    if len(log_buffer):
        for data in log_buffer:
            fdata.write(data)
        del log_buffer[:]
        fdata.flush()

def close():
    # close files
    fdata.close()
    return True

def log_queue( data ):
    log_buffer.append(data)

def log_message( pkt_id, payload ):
    msg = comms.serial_parser.wrap_packet(pkt_id, payload)
    
    if enable_file:
        log_queue( msg )

    if enable_udp:
        result = sock.sendto(msg, (udp_host, udp_port))
        if result != len(msg):
            print('error transmitting udp log packet')

# build messages and log them as needed

act_skip = logging_node.getInt("actuator_skip")
act_count = random.randint(0, act_skip)
airdata_skip = logging_node.getInt("airdata_skip")
airdata_count = random.randint(0, airdata_skip)
ap_skip = logging_node.getInt("autopilot_skip")
ap_count = random.randint(0, ap_skip)
filter_skip = logging_node.getInt("filter_skip")
filter_count = random.randint(0, filter_skip)
gps_skip = logging_node.getInt("gps_skip")
gps_count = random.randint(0, gps_skip)
health_skip = logging_node.getInt("health_skip")
health_count = random.randint(0, health_skip)
imu_skip = logging_node.getInt("imu_skip")
imu_count = random.randint(0, imu_skip)
pilot_skip = logging_node.getInt("pilot_skip")
pilot_count = random.randint(0, pilot_skip)

def process_messages():
    global act_count
    global airdata_count
    global ap_count
    global filter_count
    global gps_count
    global health_count
    global imu_count
    global pilot_count
    act_count -= 1
    airdata_count -= 1
    ap_count -= 1
    filter_count -= 1
    gps_count -= 1
    health_count -= 1
    imu_count -= 1
    pilot_count -= 1
    if act_count < 0:
        act_count = act_skip
        buf = packer.pack_act_bin()
        if not buf is None and len(buf):
            log_message(packer.act.id, buf)
    if airdata_count < 0:
        airdata_count = airdata_skip
        buf = packer.pack_airdata_bin()
        if not buf is None and len(buf):
            log_message(packer.airdata.id, buf)
    if ap_count < 0:
        ap_count = ap_skip
        buf = packer.pack_ap_status_bin()
        if not buf is None and len(buf):
            log_message(packer.ap.id, buf)
    if filter_count < 0:
        filter_count = filter_skip
        buf = packer.pack_filter_bin()
        if not buf is None and len(buf):
            log_message(packer.filter.id, buf)
    if gps_count < 0:
        gps_count = gps_skip
        buf = packer.pack_gps_bin()
        if not buf is None and len(buf):
            log_message(packer.gps.id, buf)
    if health_count < 0:
        health_count = health_skip
        buf = packer.pack_system_health_bin()
        if not buf is None and len(buf):
            log_message(packer.health.id, buf)
    if imu_count < 0:
        imu_count = imu_skip
        buf = packer.pack_imu_bin()
        if not buf is None and len(buf):
            log_message(packer.imu.id, buf)
    if pilot_count < 0:
        pilot_count = pilot_skip
        buf = packer.pack_pilot_bin()
        if not buf is None and len(buf):
            log_message(packer.pilot.id, buf)
    
def update():
    process_messages()
    write_messages();

# write out the imu calibration parameters associated with this data
# (this allows us to later rederive the original raw sensor values.)
def log_imu_calibration( config ):
    os.path.join(flight_dir, 'imucal.json' )
    props_json.save( config, jsonfile )
    return True

# write several config files to the flight directory so that the data
# can be paired with important configuration settings.
def write_configs():
    config = getNode("/config", True)
    file = os.path.join(flight_dir, 'master-config.json')
    props_json.save(file, config)
    
    config = getNode("/config/autopilot", True)
    file = os.path.join(flight_dir, 'ap-config.json')
    props_json.save(file, config)

    
