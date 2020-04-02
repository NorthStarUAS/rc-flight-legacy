import datetime
import sys

sys.path.append("../src")
from comms import aura_messages
from comms.packer import packer
import comms.serial_parser

parser = None
f = None

def init():
    global parser
    parser = comms.serial_parser.serial_parser()
    new_logfile()

def update(ser):
    global parser
    pkt_id = parser.read(ser)
    if pkt_id >= 0:
        parse_msg(pkt_id, parser.payload)
        log_msg(f, pkt_id, parser.pkt_len, parser.payload,
                parser.cksum_lo, parser.cksum_hi)

# simple 2-byte checksum
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
        c0 = (c0 + buf[i]) & 0xff
        c1 = (c1 + c0) & 0xff
        # print "c0 =", c0, "c1 =", c1, '[', ord(buf[i]), ']'
        # print "c0 =", c0, "(", cksum0, ")", "c1 =", c1, "(", cksum1, ")"
    if c0 == cksum0 and c1 == cksum1:
        return True
    else:
        print("c0 =", c0, "(", cksum0, ")", "c1 =", c1, "(", cksum1, ")")
        return False

def new_logfile():
    global f

    d = datetime.datetime.utcnow()
    logfile = 'flight-' + d.strftime("%Y%m%d-%H%M%S") + '.log'
    try:
        f = open(logfile, 'wb')
    except:
        print("Cannot open:", logfile)
        quit()

def parse_msg(id, buf):
    if id == aura_messages.gps_v2_id:
        index = packer.unpack_gps_v2(buf)
    elif id == aura_messages.gps_v3_id:
        index = packer.unpack_gps_v3(buf)
    elif id == aura_messages.gps_v4_id:
        index = packer.unpack_gps_v4(buf)
    elif id == aura_messages.imu_v3_id:
        index = packer.unpack_imu_v3(buf)
    elif id == aura_messages.imu_v4_id:
        index = packer.unpack_imu_v4(buf)
    elif id == aura_messages.imu_v5_id:
        index = packer.unpack_imu_v5(buf)
    elif id == aura_messages.airdata_v5_id:
        index = packer.unpack_airdata_v5(buf)
    elif id == aura_messages.airdata_v6_id:
        index = packer.unpack_airdata_v6(buf)
    elif id == aura_messages.airdata_v7_id:
        index = packer.unpack_airdata_v7(buf)
    elif id == aura_messages.filter_v3_id:
        index = packer.unpack_filter_v3(buf)
    elif id == aura_messages.filter_v4_id:
        index = packer.unpack_filter_v4(buf)
    elif id == aura_messages.filter_v5_id:
        index = packer.unpack_filter_v5(buf)
    elif id == aura_messages.actuator_v2_id:
        index = packer.unpack_act_v2(buf)
    elif id == aura_messages.actuator_v3_id:
        index = packer.unpack_act_v3(buf)
    elif id == aura_messages.pilot_v2_id:
        index = packer.unpack_pilot_v2(buf)
    elif id == aura_messages.pilot_v3_id:
        index = packer.unpack_pilot_v3(buf)
    elif id == aura_messages.ap_status_v4_id:
        index = packer.unpack_ap_status_v4(buf)
    elif id == aura_messages.ap_status_v5_id:
        index = packer.unpack_ap_status_v5(buf)
    elif id == aura_messages.ap_status_v6_id:
        index = packer.unpack_ap_status_v6(buf)
    elif id == aura_messages.ap_status_v7_id:
        index = packer.unpack_ap_status_v7(buf)
    elif id == aura_messages.system_health_v4_id:
        index = packer.unpack_system_health_v4(buf)
    elif id == aura_messages.system_health_v5_id:
        index = packer.unpack_system_health_v5(buf)
    elif id == aura_messages.system_health_v6_id:
        index = packer.unpack_system_health_v6(buf)
    elif id == aura_messages.payload_v2_id:
        index = packer.unpack_payload_v2(buf)
    elif id == aura_messages.payload_v3_id:
        index = packer.unpack_payload_v3(buf)
    elif id == aura_messages.event_v1_id:
        index = packer.unpack_event_v1(buf)
    elif id == aura_messages.event_v2_id:
        index = packer.unpack_event_v2(buf)
    else:
        print("Unknown packet id:", id)
        index = 0
    # except:
    #     print "Error unpacking packet id:", id
    #     index = 0
    return index

def log_msg(f, pkt_id, pkt_len, payload, cksum_lo, cksum_hi):
    f.write(bytes([comms.serial_parser.START_OF_MSG0]))
    f.write(bytes([comms.serial_parser.START_OF_MSG1]))
    f.write(bytes([pkt_id]))
    f.write(bytes([pkt_len]))
    f.write(payload)
    f.write(bytes([cksum_lo]))
    f.write(bytes([cksum_hi]))

counter = 0
def file_read(buf):
    global counter

    savebuf = ''
    myeof = False

    # scan for sync characters
    sync0 = buf[counter]; counter += 1
    sync1 = buf[counter]; counter += 1
    while (sync0 != comms.serial_parser.START_OF_MSG0 or sync1 != comms.serial_parser.START_OF_MSG1) and counter < len(buf):
        sync0 = sync1
        sync1 = buf[counter]; counter += 1
        print("scanning for start of message:", counter, sync0, sync1)

    # print "found start of message ..."

    # read message id and size
    id = buf[counter]; counter += 1
    size = buf[counter]; counter += 1
    # print "message =", id, "size =", size

    # load message
    try:
        savebuf = buf[counter:counter+size]; counter += size
    except:
        print("ERROR: didn't read enough bytes!")

    # read checksum
    cksum0 = buf[counter]; counter += 1
    cksum1 = buf[counter]; counter += 1

    if validate_cksum(id, size, savebuf, cksum0, cksum1):
        # print "check sum passed"
        index = parse_msg(id, savebuf)
        return (id, index, counter)

    print("Check sum failure!")
    return (-1, -1, counter)
