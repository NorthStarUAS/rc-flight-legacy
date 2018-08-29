from props import getNode
import props_json

import comms.logging as logging
import comms.packer as packer

comms_node = getNode('/comms', True)

def init():
    return True

def update():
    return True

# pack and send message
def log(header="", message=""):
    event_string = '%s: %s' % (header, message)
    if comms_node.getBool('display_on'):
        print(event_string)
    buf = packer.pack_event_bin(event_string)
    logging.log_message(buf)
    return True
