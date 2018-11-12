#!/usr/bin/python

import gzip
import socket

#UDP_IP = "127.0.0.1"
UDP_IP = "0.0.0.0"
UDP_PORT = 6550

sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

f = gzip.open('flight.dat.gz', 'wb')

print "Aura Logger"
print "listening for", UDP_IP, UDP_PORT

while True:
    data, addr = sock.recvfrom(1024)    # buffer size is 1024 bytes
    print "received message:", len(data)
    f.write(data)
