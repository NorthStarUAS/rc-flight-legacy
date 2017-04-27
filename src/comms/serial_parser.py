# somewhat specific code, but I'd like to share it between both sides
# of the communication link so I have separated it out as it's own
# class.

import string
import time

START_OF_MSG0 = 147
START_OF_MSG1 = 224

# FIXME: I like this feature which can catch ascii messages injected
# in the output, although with newest code and newest hardware with a
# dedicated uart for messages, this is actually deprecated.
ascii_message = ''
def glean_ascii_msgs(c, display=False):
    global ascii_message
    if c in string.printable:
        ascii_message += str(c)
    elif len(ascii_message) > 4:
        if display: print ascii_message
        ascii_message = ''

class serial_parser():
    def __init__(self):
        self.state = 0
        self.pkt_id = 0
        self.pkt_len = 0
        self.counter = 0
        self.cksum_A = 0
        self.cksum_B = 0
        self.cksum_lo = 0
        self.cksum_hi = 0
        self.payload = ''

    def read(self, ser):
        start_time = time.time()    # sec
        input = ''
        # print "enter update(), state:", self.state
        if self.state == 0:
            self.counter = 0
            self.payload = ''
            self.cksum_A = 0
            self.cksum_B = 0
            input = ser.read(1)
            while len(input) and ord(input[0]) != START_OF_MSG0:
                # print " state0 val:", ord(input[0])
                glean_ascii_msgs(input[0], display=False)
                input = ser.read(1)
                cur_time = time.time()
                if cur_time > start_time + 0.1:
                    # don't get stuck on a stream that has no parsable data
                    return -1
            if len(input) and ord(input[0]) == START_OF_MSG0:
                # print " read START_OF_MSG0"
                self.state += 1
        if self.state == 1:
            input = ser.read(1)
            if len(input):
                if ord(input[0]) == START_OF_MSG1:
                    # print " read START_OF_MSG1"
                    self.state += 1
                elif ord(input[0]) == START_OF_MSG0:
                    # print " read START_OF_MSG0"
                    pass
                else:
                    self.state = 0
        if self.state == 2:
            input = ser.read(1)
            if len(input):
                self.pkt_id = ord(input[0])
                self.cksum_A = (self.cksum_A + ord(input[0])) & 0xff
                self.cksum_B = (self.cksum_B + self.cksum_A) & 0xff
                # print " pkt_id:", self.pkt_id
                self.state += 1
        if self.state == 3:
            input = ser.read(1)
            if len(input):
                self.pkt_len = ord(input[0])
                # print " pkt_len:", self.pkt_len
                # print " payload =",
                self.cksum_A = (self.cksum_A + ord(input[0])) & 0xff
                self.cksum_B = (self.cksum_B + self.cksum_A) & 0xff
                self.state += 1
        if self.state == 4:
            input = ser.read(1)
            while len(input):
                self.counter += 1
                self.payload += input[0]
                # print "%02X" % ord(input[0]),
                self.cksum_A = (self.cksum_A + ord(input[0])) & 0xff
                self.cksum_B = (self.cksum_B + self.cksum_A) & 0xff
                if self.counter >= self.pkt_len:
                    self.state += 1
                    # print ""
                    break
                input = ser.read(1)
        if self.state == 5:
            input = ser.read(1)
            if len(input):
                self.cksum_lo = ord(input[0])
                # print " cksum_lo:", self.cksum_lo
                self.state += 1
        if self.state == 6:
            input = ser.read(1)
            if len(input):
                self.cksum_hi = ord(input[0])
                # print " cksum_hi:", self.cksum_hi
                if self.cksum_A == self.cksum_lo and self.cksum_B == self.cksum_hi and self.pkt_len > 0:
                    #print "checksum passes:", self.pkt_id, "len:", self.pkt_len
                    self.state = 0
                    return self.pkt_id
                else:
                    # print "pkt id=%d checksum failed %d %d (computed) != %d %d (message)" % (self.pkt_id, self.cksum_A, self.cksum_B, self.cksum_lo, self.cksum_hi)
                    print "pkt id=%d len=%d checksum failed" % (self.pkt_id, self.pkt_len)
                    self.state = 0

        return -1
