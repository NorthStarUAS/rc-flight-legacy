from props import getNode
import props_json

from comms import aura_messages
import comms.logging as logging

comms_node = getNode('/comms', True)
status_node = getNode( '/status', True)

def init():
    return True

def update():
    return True

# pack and send message
def log(header="", message=""):
    event_string = '%s: %s' % (header, message)
    if comms_node.getBool('display_on'):
        print(event_string)
    event = aura_messages.event_v2()
    event.timestamp_sec = status_node.getFloat('frame_time')
    event.message = event_string
    buf = event.pack()
    logging.log_message(event.id, buf)
    return True
