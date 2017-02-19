import __builtin__ # open()
from props import root, getNode
import props_json

import logging
import packer

def init():
    return True

def open(path):
    # 'events' are now sent to logging stream
    return True

# pack and send message
def log(header="", message=""):
    event_string = '%s: %s' % (header, message)
    buf = packer.pack_event_v1(event_string)
    logging.log_event(buf, len(buf))
    return True

def close():
    # 'events' are now sent to logging stream
    return True
