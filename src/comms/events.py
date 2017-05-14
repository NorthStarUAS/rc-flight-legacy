import __builtin__ # open()
from props import root, getNode
import props_json

import logging
import packer
import remote_link

def init():
    return True

def update():
    return True

# pack and send message
def log(header="", message=""):
    event_string = '%s: %s' % (header, message)
    buf = packer.pack_event_v1(event_string)
    logging.log_message(buf)
    remote_link.send_message(buf)
    return True
