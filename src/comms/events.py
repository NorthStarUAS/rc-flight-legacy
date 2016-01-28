import __builtin__ # open()
from props import root, getNode

event_log_on = False
fevent = None

def init():
    return True

def open(path):
    global event_log_on
    global fevent
    
    if path == "":
        print "ERROR: invalid path in events.py:", path
        return
    if root.config.logging.hasChild("events"):
        event_log_on = root.config.logging.events
    else:
        event_log_on = False
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
        line = "%.3f %s: %s\n" % (root.status.frame_time, header, message)
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
