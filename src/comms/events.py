from PropertyTree import PropertyNode

from comms import rc_messages
import comms.logging as logging

comms_node = PropertyNode("/comms")
status_node = PropertyNode( "/status")

def init():
    return True

def update():
    return True

# pack and send message
def log(header="", message=""):
    event_string = "%s: %s" % (header, message)
    if comms_node.getBool("display_on"):
        print(event_string)
    event = rc_messages.event_v2()
    event.timestamp_sec = status_node.getDouble("frame_time")
    event.message = event_string
    buf = event.pack()
    logging.log_message(event.id, buf)
    return True
