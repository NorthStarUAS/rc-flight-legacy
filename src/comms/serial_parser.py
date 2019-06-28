# somewhat specific code, but I'd like to share it between both sides
# of the communication link so I have separated it out as it's own
# class.

import string
import time

START_OF_MSG0 = 147
START_OF_MSG1 = 224

# simple 2-byte checksum
def checksum(id, buf, size):
    c0 = 0
    c1 = 0
    c0 = (c0 + id) & 0xff
    c1 = (c1 + c0) & 0xff
    #print("c0 =", c0, "c1 =", c1)
    c0 = (c0 + size) & 0xff
    c1 = (c1 + c0) & 0xff
    #print("c0 =", c0, "c1 =", c1)
    for i in range(0, size):
        c0 = (c0 + buf[i]) & 0xff
        c1 = (c1 + c0) & 0xff
        #print("c0 =", c0, "c1 =", c1, i, '[', buf[i], ']')
    #print("c0 =", c0, "c1 =", c1)
    return (c0, c1)

# wrap payload in header bytes, id, length, payload, and compute checksums
def wrap_packet( packet_id, payload ):
    size = len(payload)
    buf = bytearray()
    buf.append(START_OF_MSG0)   # start of message sync bytes
    buf.append(START_OF_MSG1)   # start of message sync bytes
    buf.append(packet_id)       # packet id (1 byte)
    buf.append(size)            # packet size (1 byte)
    buf.extend(payload)         # copy payload
    (cksum0, cksum1) = checksum( packet_id, payload, size)
    buf.append(cksum0)          # check sum byte 1
    buf.append(cksum1)          # check sum byte 2
    return buf
    
class serial_parser():
    def __init__(self):
        self.state = 0
        self.pkt_id = 0
        self.pkt_len = 0
        self.counter = 0
        self.cksum_lo = 0
        self.cksum_hi = 0
        self.payload = bytearray()

    def read(self, ser):
        start_time = time.time()    # sec
        input = ''
        #print("enter update(), state:", self.state)
        if self.state == 0:
            self.counter = 0
            self.payload = bytearray()
            input = ser.read(1)
            while len(input) and input[0] != START_OF_MSG0:
                # print(" state0 val:", input[0])
                input = ser.read(1)
                cur_time = time.time()
                if cur_time > start_time + 0.1:
                    # don't get stuck on a stream that has no parsable data
                    return -1
            if len(input) and input[0] == START_OF_MSG0:
                # print(" read START_OF_MSG0")
                self.state += 1
        if self.state == 1:
            input = ser.read(1)
            if len(input):
                if input[0] == START_OF_MSG1:
                    # print " read START_OF_MSG1"
                    self.state += 1
                elif input[0] == START_OF_MSG0:
                    # print " read START_OF_MSG0"
                    pass
                else:
                    self.state = 0
        if self.state == 2:
            input = ser.read(1)
            if len(input):
                self.pkt_id = input[0]
                #print(" pkt_id:", self.pkt_id)
                self.state += 1
        if self.state == 3:
            input = ser.read(1)
            if len(input):
                self.pkt_len = input[0]
                # print " pkt_len:", self.pkt_len
                # print " payload =",
                self.state += 1
        if self.state == 4:
            input = ser.read(1)
            while len(input):
                self.counter += 1
                self.payload.append(input[0])
                # print "%02X" % input[0],
                if self.counter >= self.pkt_len:
                    self.state += 1
                    # print ""
                    break
                input = ser.read(1)
        if self.state == 5:
            input = ser.read(1)
            if len(input):
                self.cksum_lo = input[0]
                # print " cksum_lo:", self.cksum_lo
                self.state += 1
        if self.state == 6:
            input = ser.read(1)
            if len(input):
                self.cksum_hi = input[0]
                (cksum0, cksum1) = checksum( self.pkt_id, self.payload, self.pkt_len );
                # print(" cksum_hi:", self.cksum_hi)
                if cksum0 == self.cksum_lo and cksum1 == self.cksum_hi and self.pkt_len > 0:
                    # print("checksum passes:", self.pkt_id, "len:", self.pkt_len)
                    self.state = 0
                    return self.pkt_id
                else:
                    # print "pkt id=%d checksum failed %d %d (computed) != %d %d (message)" % (self.pkt_id, cksum0, cksum1, self.cksum_lo, self.cksum_hi)
                    print("pkt id=%d len=%d checksum failed" % (self.pkt_id, self.pkt_len))
                    self.state = 0

        return -1
    
