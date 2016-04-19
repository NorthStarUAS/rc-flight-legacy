import sys
sys.path.append("../src")
import comms.packer
import time

# constants

START_OF_MSG0 = 147
START_OF_MSG1 = 224

GPS_PACKET_V1 = 0
IMU_PACKET_V1 = 1
FILTER_PACKET_V1 = 2
ACTUATOR_PACKET_V1 = 3
PILOT_INPUT_PACKET_V1 = 4
AP_STATUS_PACKET_V1 = 5
AIRDATA_PACKET_V3 = 9
AP_STATUS_PACKET_V2 = 10
SYSTEM_HEALTH_PACKET_V2 = 11
PAYLOAD_PACKET_V1 = 12
AIRDATA_PACKET_V4 = 13
SYSTEM_HEALTH_PACKET_V3 = 14
IMU_PACKET_V2 = 15
GPS_PACKET_V2 = 16
IMU_PACKET_V3 = 17
AIRDATA_PACKET_V5 = 18
SYSTEM_HEALTH_PACKET_V4 = 19

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
        c0 = (c0 + ord(buf[i])) & 0xff
        c1 = (c1 + c0) & 0xff
        # print "c0 =", c0, "c1 =", c1, '[', ord(buf[i]), ']'
    # print "c0 =", c0, "(", cksum0, ")", "c1 =", c1, "(", cksum1, ")"
    if c0 == cksum0 and c1 == cksum1:
        return True
    else:
        return False
    
def parse_msg(id, buf):
    if id == GPS_PACKET_V1:
        comms.packer.unpack_gps_v1(buf)
    elif id == GPS_PACKET_V2:
        comms.packer.unpack_gps_v2(buf)
    elif id == IMU_PACKET_V1:
        comms.packer.unpack_imu_v1(buf)
    elif id == IMU_PACKET_V2:
        comms.packer.unpack_imu_v2(buf)
    elif id == IMU_PACKET_V3:
        comms.packer.unpack_imu_v3(buf)
    elif id == AIRDATA_PACKET_V3:
        comms.packer.unpack_airdata_v3(buf)
    elif id == AIRDATA_PACKET_V4:
        comms.packer.unpack_airdata_v4(buf)
    elif id == AIRDATA_PACKET_V5:
        comms.packer.unpack_airdata_v5(buf)
    elif id == FILTER_PACKET_V1:
        comms.packer.unpack_filter_v1(buf)
    elif id == ACTUATOR_PACKET_V1:
        comms.packer.unpack_act_v1(buf)
    elif id == PILOT_INPUT_PACKET_V1:
        comms.packer.unpack_pilot_v1(buf)
    elif id == AP_STATUS_PACKET_V1:
        comms.packer.unpack_ap_status_v1(buf)
    elif id == AP_STATUS_PACKET_V2:
        comms.packer.unpack_ap_status_v2(buf)
    elif id == SYSTEM_HEALTH_PACKET_V2:
        comms.packer.unpack_system_health_v2(buf)
    elif id == SYSTEM_HEALTH_PACKET_V3:
        comms.packer.unpack_system_health_v3(buf)
    elif id == SYSTEM_HEALTH_PACKET_V4:
        comms.packer.unpack_system_health_v4(buf)
    elif id == PAYLOAD_PACKET_V1:
        comms.packer.unpack_payload_v1(buf)
    else:
        print "Unknown packet id:", id
        pass

# 'static' variables for the serial stream and file parsers
state = 0
pkt_id = 0
pkt_len = 0
counter = 0
cksum_A = 0
cksum_B = 0
cksum_lo = 0
cksum_hi = 0
payload = ''

# FIXME: I like this feature which can catch ascii messages injected
# in the output, although with newest code and newest hardware with a
# dedicated uart for messages, this is actually deprecated.
def glean_ascii_msgs(c):
    pass

def serial_read(ser):
    global state
    global counter
    global payload

    start_time = time.time()    # sec
    input = ''
    msg_id = -1
    # print "enter update(), state:", state
    if state == 0:
        counter = 0
        cksum_A = 0
        cksum_B = 0
        input = ser.read(1)
        while len(input) and ord(input[0]) != START_OF_MSG0:
            # print " state0 val:", ord(input[0])
            glean_ascii_msgs(input[0])
            input = ser.read(1)
            cur_time = time.time()
            if cur_time > start_time + 0.1:
                # don't get stuck on a stream that has no parsable data
                return msg_id
        if len(input) and ord(input[0]) == START_OF_MSG0:
            # print " read START_OF_MSG0"
            state += 1
    if state == 1:
        input = ser.read(1)
        if len(input):
            if ord(input[0]) == START_OF_MSG1:
                # print " read START_OF_MSG1"
                state += 1
            elif ord(input[0]) == START_OF_MSG0:
                # print " read START_OF_MSG0"
                pass
            else:
                state = 0
    if state == 2:
        input = ser.read(1)
        if len(input):
            pkt_id = ord(input[0])
            cksum_A = (cksum_A + ord(input[0])) & 0xff
            cksum_B = (cksum_B + cksum_A) & 0xff
            # print " pkt_id:", pkt_id
            state += 1
    if state == 3:
        input = ser.read(1)
        if len(input):
            pkt_len = ord(input[0])
            # print " pkt_len:", pkt_len
            # print " payload =",
            cksum_A = (cksum_A + ord(input[0])) & 0xff
            cksum_B = (cksum_B + cksum_A) & 0xff
            state += 1
    if state == 4:
        input = ser.read(1)
        while len(input):
            counter += 1
            payload += input[0]
            # print "%02X" % ord(input[0]),
            cksum_A = (cksum_A + ord(input[0])) & 0xff
            cksum_B = (cksum_B + cksum_A) & 0xff
            if counter >= pkt_len:
                state += 1
                # print ""
                break
            input = ser.read(1)
    if state == 5:
        input = ser.read(1)
        if len(input):
            cksum_lo = ord(input[0])
            # print " cksum_lo:", cksum_lo
            state += 1
    if state == 6:
        input = ser.read(1)
        if len(input):
            cksum_hi = ord(input[0])
            # print " cksum_hi:", cksum_hi
            if cksum_A == cksum_lo and cksum_B == cksum_hi:
                # print "checksum passes:", pkt_id
                parse_msg(pkt_id, payload)
                msg_id = pkt_id
            else:
                print "pkt=%d checksum failed %d %d (computed) != %d %d (message)\n" % (pkt_id, cksum_A, cksum_B, cksum_lo, cksum_hi)
            # this is the end of a record, reset state to 0 to start
            # looking for next record
            state = 0
            payload = ''

    # print "exit routine, msg_id:", msg_id
    return msg_id

def file_read(buf):
    global counter
    
    savebuf = ''
    print "file_update()"

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
        parse_msg(id, savebuf)
        return id

    print "Check sum failure!"
    return -1
