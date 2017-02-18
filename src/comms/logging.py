# logging.py

import gzip
import os
import re
import socket

from props import root, getNode

# global variables for data file logging
log_buffer = []
fdata = None

logging_node = None
log_to_file = False  # log to file is enabled/disabled
log_path = ''
flight_dir = ''      # dir containing all our logged data

# remote logging support
#static netSocket sock
#static int port = 0
#static string hostname = ""
#static bool udp_logging_inited = false

# scan the base path for fltNNNN directories.  Return the biggest
# flight number
def max_flight_num():
    max = -1
    if not os.path.isdir( log_path ):
        print 'Flight data directory does not exist:', log_path
        return max
    for f in os.path.listdir( log_path ):
        if os.path.isfile(f):
            print 'file:', f
            p = re.compile('\d+')
            m = p.match(f)
            val = int(m.group())
            if val > max: max = val
    return max

def udp_logging_init():
    port = 6500
    ip = '127.0.0.1'
    # open a UDP socket
    try:
        sock = socket.socket(socket.AF_INET,    # Internet
                             socket.SOCK_DGRAM) # UDP
    except:
	print 'Error opening logging socket'
	return False
    udp_logging_inited = True
    return True


def init():
    logging_node = getNode('/config/logging')
    log_path = logging_node.getString('path')
    
    # find the biggest flight number logged so far
    max = max_flight_num()
    print 'Max log dir index:', max

    # make the new logging directory
    new_dir = 'flt%05d' % (max + 1)
    flight_dir = os.path.join(log_path, new_dir)
    try:
        print 'Creating log dir:', flight_dir
        os.mkdir(flight_dir)
        logging_node.setString('flight_dir', flight_dir)
    except:
        print 'Error creating:', flight_dir

    # open the logging files
    file = os.path.join(flight_dir, 'flight.dat.gz')
    try:
        fdata = gzip.open(file, 'wb')
    except:
        print 'Cannot open:', file
        return False

    # fixme:
    # events->open(flight_dir.c_str())
    # events->log("Log", "Start")
    
    return True

def close():
    # close files
    fdata.close()
    return True

def log_queue( data ):
    log_buffer.append(data)

# simple 2-byte checksum
def compute_cksum(id, size, buf):
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

def log_packet( packet_id, packet_buf, packet_size ):
    buf = ''
    buf += chr(START_OF_MSG0)   # start of message sync bytes
    buf += chr(START_OF_MSG1)   # start of message sync bytes
    buf += chr(packet_id)       # packet id (1 byte)
    buf += chr(packet_size)     # packet size (1 byte)
    buf += data                 # copy packet data

    # check sum (2 bytes)
    (cksum0, cksum1) = compute_cksum( packet_id, packet_buf, packet_size)
    buf += chr(cksum0)
    buf += chr(cksum1)

    log_queue( buf, packet_size + 6 )

    if udp_logging_inited:
        result = sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        if result != packet_size + 6:
            print 'error transmitting udp log packet'
    else:
        udp_logging_init()

def log_gps( buf, size ):
    log_packet( GPS_PACKET_V3, buf, size )

def log_imu( buf, size ):
    log_packet( IMU_PACKET_V3, buf, size )

def log_airdata( buf, size ):
    log_packet( AIRDATA_PACKET_V5, buf, size )

def log_filter( buf, size ):
    log_packet( FILTER_PACKET_V2, buf, size )

def log_actuator( buf, size ):
    log_packet( ACTUATOR_PACKET_V2, buf, size )

def log_pilot( buf, size ):
    log_packet( PILOT_INPUT_PACKET_V2, buf, size )

def log_ap( buf, size ):
    log_packet( AP_STATUS_PACKET_V3, buf, size )

def log_health( buf, size ):
    log_packet( SYSTEM_HEALTH_PACKET_V4, buf, size )

def log_payload( buf, size ):
    log_packet( PAYLOAD_PACKET_V2, buf, size )

def log_raven( buf, size ):
    log_packet( RAVEN_PACKET_V1, buf, size )

# write all pending data and flush
def update():
    if len(log_buffer):
        for data in log_buffer:
            fdata.write(data)
        del log_buffer[:]
        fdata.flush()

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

    
