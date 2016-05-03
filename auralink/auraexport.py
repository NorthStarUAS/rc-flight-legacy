#!/usr/bin/python

import argparse
import os
import tempfile

from props import root, getNode

import commands
import current
import parser

argparser = argparse.ArgumentParser(description='aura export')
argparser.add_argument('--flight', help='load specified flight log')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')

args = argparser.parse_args()

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
        try:
            (id, index) = parser.file_read(full)
        except:
            print "end of file"
            break
else:
    print "A flight log file must be provided"
