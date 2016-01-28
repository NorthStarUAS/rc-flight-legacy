import __builtin__ # open()
from props import root, getNode

event_log_on = False
fevent = None
logging_node = None
status_node = None

def init():
    global logging_node
    global status_node
    logging_node = getNode("/config/logging", True)
    status_node = getNode("/status", True)
    return True

def open(path):
    global event_log_on
    global fevent
    global logging_node
    global status_node

    if path == "":
        print "ERROR: invalid path in events.py:", path
        return
    event_log_on = logging_node.getBool("events")
    if event_log_on:
        try:
            filename = path + "/events.txt"
            fevent = __builtin__.open(filename, 'w')
            print "opened event log file:", filename
        except:
            print "Warning: unable to open event log file:", filename

def log(header="", message=""):
    global event_log_on
    global fevent
    if event_log_on and fevent:
        line = "%.3f %s: %s\n" % (status_node.getFloat("frame_time"),
                                  header, message)
        fevent.write( line )
        fevent.flush()
        return True
    else:
        return False

def close():
    global event_log_on
    global fevent
    if event_long_on:
        fevent.close()
    return True
