event_log_on = False
fevent = None

def init(path):
    if path == "":
        print "ERROR: invalid path in events.py:", path
        return
    try:
        filename = path + "/events.txt"
        fevent = open(filename, 'w')
        event_log_on = True
    except:
        print "Warning: unable to open event log file:", filename

def log(header="", message=""):
    if not fevent:
        return False
    line = "%.3f %s %s\n" % (root.system.time, header, message)
    fevent.write( line )
    fevent.flush()
    return True

