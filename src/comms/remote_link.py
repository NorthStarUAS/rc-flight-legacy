import serial

from props import root, getNode
import props_json

import events

status_node = getNode( '/status', True)
route_node = getNode( '/task/route', True )
task_node = getNode( '/task', True )
home_node = getNode( '/task/home', True )
targets_node = getNode( '/autopilot/targets', True )
comms_node = getNode( '/comms', True)

remote_link_config = getNode('/config/remote_link', True)
remote_link_node = getNode('/comms/remote_link', True)

remote_link_on = False    # link to remote operator station
ser = None
serial_buf = ''
max_serial_buffer = 256
link_open = False

# set up the remote link
def init():
    global ser
    global link_open
    
    device = remote_link_config.getString('device')
    try:
        ser = serial.Serial(port=device, baudrate=115200, timeout=0, write_timeout=0)
    except:
        print 'Opening remote link failed:', device
	return False
    
    if comms_node.getBool('display_on'):
	print 'remote link:', device
    link_open = True

    remote_link_node.setInt('sequence_num', 0)
    if not remote_link_config.getInt('write_bytes_per_frame'):
	remote_link_config.setInt('write_bytes_per_frame', 12)

# write as many bytes out of the serial_buf to the uart as the
# driver will accept.
def flush_serial():
    global serial_buf
    if not link_open:
	# device not open
	return

    # write a constrained chunk of available bytes per frame to avoid
    # overflowing the system serial port buffer
    bytes_per_frame = remote_link_config.getInt('write_bytes_per_frame')
    write_len = len(serial_buf)
    if write_len > bytes_per_frame:
        write_len = bytes_per_frame
    if write_len:
        bytes_written = ser.write( serial_buf[:write_len] )
        # print 'avail = %d  written = %d' % (len(serial_buf), bytes_written)
        if bytes_written < 0:
            # perror('serial write')
            pass
        elif bytes_written == 0:
            # nothing was written
            pass
        else:
            # something was written
            serial_buf = serial_buf[bytes_written:]
    # print 'remote link bytes pending:', len(serial_buf)

# append the request data to a fifo buffer if space available.  A
# separate function will flush the data in even chunks to avoid
# saturating the telemetry link.
def send_message( data ):
    global serial_buf
    if ser == None:
        # remote serial link not available
        return False
        
    if len(serial_buf) + len(data) <= max_serial_buffer:
        serial_buf += data
        return True
    elif comms_node.getBool('display_on'):
	print 'remote link serial buffer overflow, size:', len(serial_buf), 'add:', len(data), 'limit:', max_serial_buffer
        return False

route_request = []
def execute_command( command ):
    global route_request
    
    if command == '':
        # no valid tokens
        return

    tokens = command.split(',')
    if tokens[0] == 'hb' and len(tokens) == 1:
        # heart beat, no action needed
        pass
    elif tokens[0] == 'home' and len(tokens) == 5:
        # specify new home location
        lon = float( tokens[1] )
        lat = float( tokens[2] )
        # alt_ft = float( tokens[3] )
        azimuth_deg = float( tokens[4] )
	home_node.setFloat( 'longitude_deg', lon )
	home_node.setFloat( 'latitude_deg', lat )
	home_node.setFloat( 'azimuth_deg', azimuth_deg )
	home_node.setBool( 'valid', True )
    elif tokens[0] == 'route' and len(tokens) >= 5:
	route_request = tokens[1:]
    elif tokens[0] == 'route_cont' and len(tokens) >= 5:
        route_request += tokens[1:]
    elif tokens[0] == 'route_end' and len(tokens) == 1:
	route_node.setString( 'route_request', ','.join(route_request) )
	task_node.setString( 'command_request', 'task,route' )
    elif tokens[0] == 'task':
	task_node.setString( 'command_request', command )
    elif tokens[0] == 'ap' and len(tokens) == 3:
        # specify an autopilot target
        if tokens[1] == 'agl-ft':
            agl_ft = float( tokens[2] )
            targets_node.setFloat( 'altitude_agl_ft', agl_ft )
        elif tokens[1] == 'msl-ft':
            msl_ft = float( tokens[2] )
            targets_node.setFloat( 'target_msl_ft', msl_ft )
        elif tokens[1] == 'speed-kt':
            speed_kt = float( tokens[2] )
            targets_node.setFloat( 'airspeed_kt', speed_kt )
    elif tokens[0] == 'fcs-update':
	decode_fcs_update( command )
    elif tokens[0] == 'set' and len(tokens) >= 3:
        if tokens[1][0] == '/':
            # absolute path
            parts = tokens[1].split('/')
            node_path = '/'.join(parts[0:-1])
            if node_path == '':
                node_path = '/'
            node = getNode(node_path, True)
            name = parts[-1]
            value = ' '.join(tokens[2:])
            node.setString(name, value)
        else:
            # expecting a full path name to set
            pass
    elif tokens[0] == 'la' and len(tokens) == 5:
	if tokens[1] == 'ned':
	    # set ned-vector lookat mode
	    point_node = getNode('/pointing', True)
	    point_node.setString('lookat_mode', 'ned_vector')
	    # specify new lookat ned coordinates
	    vector_node = getNode('/pointing/vector', True)
	    north = float( tokens[2] )
	    east = float( tokens[3] )
	    down = float( tokens[4] )
	    vector_node.setFloat( 'north', north )
	    vector_node.setFloat( 'east', east )
	    vector_node.setFloat( 'down', down )
        elif tokens[1] == 'wgs84':
	    # set wgs84 lookat mode
	    point_node = getNode('/pointing', True)
	    point_node.setString('lookat_mode', 'wgs84')
	    # specify new lookat ned coordinates
	    wgs84_node = getNode('/pointing/wgs84', True)
	    pos_node = getNode('/position', True)
	    lon = float( tokens[2] )
	    lat = float( tokens[3] )
	    wgs84_node.setFloat( 'longitude_deg', lon )
	    wgs84_node.setFloat( 'latitude_deg', lat )
	    ground = pos_node.getFloat('altitude_ground_m')
	    wgs84_node.setFloat( 'altitude_m', ground )

command_buf = ''
def read_link_command():
    global command_buf
    
    if ser == None:
        # remote link open failed
        return ''
    
    # read character by character until we run out of data or find a '\n'
    # if we run out of data, save what we have so far and start with that for
    # the next call.
    input = ser.read(1)
    while len(input) and input[0] != '\n':
        command_buf += input[0]
        input = ser.read(1)

    if len(input) and input[0] == '\n':
        result = command_buf
        command_buf = ''
        return result
    else:
        return ''


# calculate the nmea check sum
def calc_nmea_cksum(sentence):
    sum = 0
    # print 'nmea: sentence'
    sum = ord(sentence[0]) & 0xff
    for c in sentence[1:]:
        sum ^= (ord(c) & 0xff)
    # print 'sum:', sum
    return sum

# read, parse, and execute incomming commands, return True if a valid
# command received, False otherwise.
last_sequence_num = -1
def command():
    global last_sequence_num
    
    result = read_link_command()
    if result == '':
        return False
    
    events.log( 'remote cmd rcvd', result )

    # validate check sum
    if len(result) < 4:
        # bogus command
        return False

    nmea_sum = result[-2:]
    cmd = result[:-3]
    cmd_sum = '%02X' % calc_nmea_cksum(cmd)

    if nmea_sum != cmd_sum:
        # checksum failure
	events.log( 'remote cmd rcvd', 'failed check sum' )
        return False

    # parse the command
    tokens = cmd.split(',')
    if len(tokens) < 2:
        # bogus command
        return False

    # extract command sequence number
    sequence = int( tokens[0] )

    # ignore repeated commands (including roll over logic)
    if sequence != last_sequence_num:
	# remainder
	cmd = ','.join(tokens[1:])

	# execute command
	events.log( 'remote cmd rcvd', 'executed valid command' )
	execute_command( cmd )

	# register that we've received this message correctly
	remote_link_node.setInt( 'sequence_num', sequence )
	last_sequence_num = sequence
        timestamp = status_node.getFloat('frame_time')
	remote_link_node.setFloat( 'last_message_sec', timestamp )

    return True

def decode_fcs_update(command):
    tokens = command.split(',')
    
    # valid sizes will be 7 or 10 at this point
    if tokens[0] == 'fcs-update' and len(tokens) >= 7:
	# remove initial keyword if it exists
	del tokens[0]

    ap_config = getNode( '/config/autopilot', True )
    
    i = int(tokens[0])
    component = ap_config.getChild('component[%d]' % i)
    if not component:
	return False

    config = component.getChild('config')
    if not config:
	return False
	
    if len(tokens) == 7:
	config.setFloat( 'Kp', float(tokens[1]) )
	config.setFloat( 'Ti', float(tokens[2]) )
	config.setFloat( 'Td', float(tokens[3]) )
	config.setFloat( 'u_min', float(tokens[4]) )
	config.setFloat( 'u_max', float(tokens[5]) )
	config.setFloat( 'u_trim', float(tokens[6]) )
    elif len(tokens) == 10:
	config.setFloat( 'Kp', float(tokens[1]) )
	config.setFloat( 'beta', float(tokens[2]) )
	config.setFloat( 'alpha', float(tokens[3]) )
	config.setFloat( 'gamma', float(tokens[4]) )
	config.setFloat( 'Ti', float(tokens[5]) )
	config.setFloat( 'Td', float(tokens[6]) )
	config.setFloat( 'u_min', float(tokens[7]) )
	config.setFloat( 'u_max', float(tokens[8]) )
	config.setFloat( 'u_trim', float(tokens[6]) )
    else:
	return False

    return True
