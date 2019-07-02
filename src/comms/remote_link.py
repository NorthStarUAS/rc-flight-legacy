import re
import serial
from props import getNode
import props_json

import comms.events
import comms.packer
import comms.serial_parser

import survey.survey

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
parser = None
serial_buf = bytearray()
max_serial_buffer = 256
link_open = False

# set up the remote link
def init():
    global ser
    global parser
    global link_open
    
    device = remote_link_config.getString('device')
    try:
        ser = serial.Serial(port=device, baudrate=115200, timeout=0, writeTimeout=0)
        parser = comms.serial_parser.serial_parser()
    except Exception as e:
        print('Opening remote link failed:', device)
        print(e)
        return False

    if comms_node.getBool('display_on'):
        print('remote link:', device)
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

# append the request data to a fifo buffer if space available.  A
# separate function will flush the data in even chunks to avoid
# saturating the telemetry link.
def send_message( pkt_id, payload ):
    global serial_buf
    if ser == None:
        # remote serial link not available
        return False

    msg = comms.serial_parser.wrap_packet(pkt_id, payload)
    if len(serial_buf) + len(msg) <= max_serial_buffer:
        serial_buf.extend(msg)
        return True
    else:
        if comms_node.getBool('display_on'):
            print('remote link serial buffer overflow, size:', len(serial_buf), 'add:', len(msg), 'limit:', max_serial_buffer)
        return False

route_request = []
survey_request = {}
def execute_command( command ):
    global route_request
    global survey_request

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
    elif tokens[0] == 'survey_start' and len(tokens) == 7:
        for i, tok in enumerate(tokens):
            if tokens[i] == 'undefined':
                tokens[i] = 0.0
        survey_request['agl_ft'] = float( tokens[1] )
        survey_request['extend_m'] = float( tokens[2] )
        survey_request['overlap_perc'] = float( tokens[3] )
        survey_request['sidelap_perc'] = float( tokens[4] )
        survey_request['forward_fov'] = float( tokens[5] )
        survey_request['lateral_fov'] = float( tokens[6] )
        survey_request['area'] = []
    elif tokens[0] == 'survey_cont' and len(tokens) > 2:
        for i in range(1, len(tokens), 2):
            wpt = ( float(tokens[i]), float(tokens[i+1]) )
            survey_request['area'].append( wpt )            
    elif tokens[0] == 'survey_end' and len(tokens) == 1:
        survey.survey.do_survey(survey_request)
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
    elif tokens[0] == 'get' and len(tokens) == 2:
        # absolute path
        parts = tokens[1].split('/')
        node_path = '/'.join(parts[0:-1])
        if node_path == '':
            node_path = '/'
        node = getNode(node_path, True)
        name = parts[-1]
        value = node.getString(name)
        if value == '': value = 'undefined'
        # print tokens[0], '=', value
        return_msg = 'get: %s,%s' % (tokens[1], value)
        event = aura_messages.event_v2()
        event.message = return_msg
        buf = event.pack()
        send_message(event.id, buf)
        comms.events.log('get', '%s,%s' % (tokens[1], value))
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
            done = False
            # test for int
            if not done:
                result = re.match('[-+]?\d+', value)
                if result and result.group(0) == value:
                    print('int:', value)
                    node.setInt(name, int(value))
                    done = True
            # test for float
            if not done:
                result = re.match('[-+]?\d*\.\d+', value)
                if result and result.group(0) == value:
                    print('float:', value)
                    node.setFloat(name, float(value))
                    done = True
            # test for bool
            if not done:
                if value == 'True' or value == 'true':
                    print('bool:', True)
                    node.setBool(name, True)
                    done = True
            if not done:
                if value == 'False' or value == 'false':
                    print('bool:', False)
                    node.setBool(name, False)
                    done = True
            # fall back to string
            if not done:
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

def read_link_command():
    global ser
    
    if parser == None:
        # remote link open failed
        return -1, ''
    
    pkt_id = parser.read(ser)
    if pkt_id == comms.packer.COMMAND_PACKET_V1:
        seq, message = comms.packer.unpack_command_v1(parser.payload)
        return seq, message
    else:
        return -1, ''


# read, parse, and execute incomming commands, return True if a valid
# command received, False otherwise.
last_sequence_num = -1
def command():
    global last_sequence_num
    
    sequence, command_bytes = read_link_command()
    if sequence < 0:
        return False
    
    # ignore repeated commands (including roll over logic)
    if sequence != last_sequence_num:
	# execute command
        command = command_bytes.decode()
        comms.events.log( 'remote command',
                          "executed: (%d) %s" % (sequence, command) )
        execute_command( command )

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
