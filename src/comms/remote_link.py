import re

from props import root, getNode
import props_json

remote_link_config = getNode("/config/remote_link", True)
remote_link_node = getNode("/comms/remote_link", True)

remote_link_on = False    # link to remote operator station
serial_fd = None
max_serial_buffer = 128
link_open = False
route_request = ''

# set up the remote link
def init():
    if display_on:
	print 'remote link: direct uart'
    device = remote_link_config.getString('device')
    if not serial_fd.open_port( device ):
	return
    serial_fd.set_baud( 115200 )
    link_open = True

    remote_link_node.setInt("sequence_num", 0)
    if not remote_link_config.getInt("write_bytes_per_frame"):
	remote_link_config.setInt("write_bytes_per_frame", 12)

# write as many bytes out of the serial_buffer to the uart as the
# driver will accept.
def remote_link_flush_serial():
    if not link_open:
	# device not open, or link type is not uart
	return

    # attempt better success by writing multiple small chunks to the
    # serial port (2 * 8 = 16 bytes per call attempted)
    bytes_per_frame = remote_link_config.getInt("write_bytes_per_frame")
    write_len = len(serial_buffer)
    if write_len > bytes_per_frame:
        write_len = bytes_per_frame
    if write_len:
        bytes_written = serial_fd.write_port( serial_buffer[:write_len] )
        print 'avail = %d  written = %d' % (len(serial_buffer), bytes_written)
        if bytes_written < 0:
            # perror("serial write")
            pass
        elif bytes_written == 0:
            # nothing was written
            pass
        else:
            # something was written
            serial_buffer = serial_buff[bytes_written:]
    print 'remote link bytes pending:', len(serial_buffer)

def link_append( data ):
    # stuff the request in a fifo buffer and then work on writing out
    # the front end of the buffer.
    if len(serial_buf) + len(data) <= max_serial_buffer:
        serial_buf += data
        return True
    elif display_on:
	print 'remote link serial buffer overflow'
        return False

def link_read(buf, size ):
    return serial_fd.read_port( buf, size )

def send_message( buf ) {
    return link_append( buf )

def remote_link_execute_command( command ):
    vector <string> token = split( command, "," )

    if ( token.size() < 1 ) {
        # no valid tokens
        return
    }

    if ( token[0] == "hb" && token.size() == 1 ) {
        # heart beat, no action needed
    } else if ( token[0] == "home" && token.size() == 5 ) {
        # specify new home location
        double lon = atof( token[1].c_str() )
        double lat = atof( token[2].c_str() )
        # double alt_ft = atof( token[3].c_str() )
        double azimuth_deg = atof( token[4].c_str() )

	pyPropertyNode home_node = pyGetNode("/task/home", True)
	home_node.setDouble( "longitude_deg", lon )
	home_node.setDouble( "latitude_deg", lat )
	home_node.setDouble( "azimuth_deg", azimuth_deg )
	home_node.setBool( "valid", True )
    } else if ( token[0] == "route" && token.size() >= 5 ) {
	route_request = token[1] # prime the pump
	for ( unsigned int i = 2 i < token.size() i++ ) {
	    route_request += ','
	    route_request += token[i]
	}
    } else if ( token[0] == "route_cont" && token.size() >= 5 ) {
	for ( unsigned int i = 1 i < token.size() i++ ) {
	    route_request += ','
	    route_request += token[i]
	}
    } else if ( token[0] == "route_end" && token.size() == 1 ) {
	pyPropertyNode route_node = pyGetNode("/task/route", True)
	route_node.setString( "route_request", route_request.c_str() )
	pyPropertyNode task_node = pyGetNode("/task", True)
	task_node.setString( "command_request", "task,route" )
    } else if ( token[0] == "task" ) {
	pyPropertyNode task_node = pyGetNode("/task", True)
	task_node.setString( "command_request", command.c_str() )
    } else if ( token[0] == "ap" && token.size() == 3 ) {
        # specify an autopilot target
	pyPropertyNode targets_node = pyGetNode("/autopilot/targets", True)
        if ( token[1] == "agl-ft" ) {
            double agl_ft = atof( token[2].c_str() )
            targets_node.setDouble( "altitude_agl_ft", agl_ft )
        } else if ( token[1] == "msl-ft" ) {
            double msl_ft = atof( token[2].c_str() )
            targets_node.setDouble( "target_msl_ft", msl_ft )
        } else if ( token[1] == "speed-kt" ) {
            double speed_kt = atof( token[2].c_str() )
            targets_node.setDouble( "airspeed_kt", speed_kt )
        }
    } else if ( token[0] == "fcs-update" ) {
	decode_fcs_update(token)
    } else if ( token[0] == "set" && token.size() == 3 ) {
	string prop_name = token[1]
	string value = token[2]
	size_t pos = prop_name.rfind("/")
	if ( pos != string::npos ) {
	    string path = prop_name.substr(0, pos)
	    string attr = prop_name.substr(pos+1)
	    pyPropertyNode node = pyGetNode( path, True )
	    if ( value == "True" or value == "False" ) {
		node.setBool( attr.c_str(), (value != "False") )
	    } else {
		node.setString( attr.c_str(), value )
	    }
	}
    } else if ( token[0] == "wp" && token.size() == 5 ) {
        # specify new waypoint coordinates for a waypoint
        # int index = atoi( token[1].c_str() )
        # double lon = atof( token[2].c_str() )
        # double lat = atof( token[3].c_str() )
        # double alt_ft = atof( token[4].c_str() )
        # SGWayPoint wp( lon, lat, alt_ft * SG_FEET_TO_METER )
        # route_mgr.replace_waypoint( wp, index )
    } else if ( token[0] == "la" && token.size() == 5 ) {
	if ( token[1] == "ned" ) {
	    # set ned-vector lookat mode
	    pyPropertyNode point_node = pyGetNode("/pointing", True)
	    point_node.setString("lookat_mode", "ned_vector")
	    # specify new lookat ned coordinates
	    pyPropertyNode vector_node = pyGetNode("/pointing/vector", True)
	    double north = atof( token[2].c_str() )
	    double east = atof( token[3].c_str() )
	    double down = atof( token[4].c_str() )
	    vector_node.setDouble( "north", north )
	    vector_node.setDouble( "east", east )
	    vector_node.setDouble( "down", down )
	} else if ( token[1] == "wgs84" ) {
	    # set wgs84 lookat mode
	    pyPropertyNode point_node = pyGetNode("/pointing", True)
	    point_node.setString("lookat_mode", "wgs84")
	    # specify new lookat ned coordinates
	    pyPropertyNode wgs84_node = pyGetNode("/pointing/wgs84", True)
	    pyPropertyNode pos_node = pyGetNode("/position", True)
	    double lon = atof( token[2].c_str() )
	    double lat = atof( token[3].c_str() )
	    wgs84_node.setDouble( "longitude_deg", lon )
	    wgs84_node.setDouble( "latitude_deg", lat )
	    double ground = pos_node.getDouble("altitude_ground_m")
	    wgs84_node.setDouble( "altitude_m", ground )
	}
    }
}


#define BUF_SIZE 256
static int read_link_command( char result_buf[BUF_SIZE] ) {
    # read character by character until we run out of data or find a '\n'
    # if we run out of data, save what we have so far and start with that for
    # the next call.

    # persistant data because we may not get the whole command in one call
    static char command_buf[BUF_SIZE]
    static int command_counter = 0

    uint8_t buf[2] buf[0] = 0

    int result = 0
    result = link_read( buf, 1 )
    while ( (result == 1) && (buf[0] != '\n')
	    && (command_counter < BUF_SIZE) ) {
        command_buf[command_counter] = buf[0]
        command_counter++
        result = link_read( buf, 1 )
    }

    if ( command_counter >= BUF_SIZE ) {
	# abort this command and try again
	command_counter = 0
    } else if ( (result == 1) && (buf[0] == '\n') ) {
        command_buf[command_counter] = 0 # terminate string
        int size = command_counter + 1
        strncpy( result_buf, command_buf, size )
        command_counter = 0
        return size
    }

    return 0
}


# calculate the nmea check sum
static char calc_nmea_cksum(const char *sentence) {
    unsigned char sum = 0
    int i, len

    # cout << sentence << endl

    len = strlen(sentence)
    sum = sentence[0]
    for ( i = 1 i < len i++ ) {
        # cout << sentence[i]
        sum ^= sentence[i]
    }
    # cout << endl

    # printf("sum = %02x\n", sum)
    return sum
}


# read, parse, and execute incomming commands, return True if a valid
# command received, False otherwise.
bool remote_link_command() {
    char command_buf[256]
    int result = read_link_command( command_buf )

    if ( result == 0 ) {
        return False
    }
    
    events->log( "remote cmd rcvd", command_buf )

    string cmd = command_buf

    # validate check sum
    if ( cmd.length() < 4 ) {
        # bogus command
        return False
    }
    string nmea_sum = cmd.substr(cmd.length() - 2)
    cmd = cmd.substr(0, cmd.length() - 3)
    char cmd_sum[10]
    snprintf( cmd_sum, 3, "%02X", calc_nmea_cksum(cmd.c_str()) )

    if ( nmea_sum.c_str()[0] != cmd_sum[0]
         || nmea_sum.c_str()[1] != cmd_sum[1])
    {
        # checksum failure
	events->log( "remote cmd rcvd", "failed check sum" )

        return False
    }

    # parse the command
    string::size_type pos = cmd.find_first_of(",")
    if ( pos == string::npos ) {
        # bogus command
        return False
    }

    # extract command sequence number
    string num = cmd.substr(0, pos)
    int sequence = atoi( num.c_str() )
    static int last_sequence_num = -1

    # ignore repeated commands (including roll over logic)
    if ( sequence != last_sequence_num ) {

	# remainder
	cmd = cmd.substr(pos + 1)

	# execute command
	events->log( "remote cmd rcvd", "executed valid command" )
	remote_link_execute_command( cmd )

	# register that we've received this message correctly
	remote_link_node.setInt( "sequence_num", sequence )
	last_sequence_num = sequence
	remote_link_node.setDouble( "last_message_sec", get_Time() )
    }

    return True
}

bool decode_fcs_update(vector <string> tokens) {
    # valid sizes will be 7 or 10 at this point
    if ( tokens.size() >= 7 && tokens[0] == "fcs-update" ) {
	# remove initial keyword if needed
	tokens.erase(tokens.begin())
    }

    pyPropertyNode ap_config = pyGetNode("/config/autopilot", True)
    
    int i = atoi(tokens[0].c_str())
    pyPropertyNode component = ap_config.getChild("component", i)
    if ( component.isNull() ) {
	return False
    }

    pyPropertyNode config = component.getChild("config")
    if ( config.isNull() ) {
	return False
    }
	
    if ( tokens.size() == 6 ) {
	config.setDouble( "Kp", atof(tokens[1].c_str()) )
	config.setDouble( "Ti", atof(tokens[2].c_str()) )
	config.setDouble( "Td", atof(tokens[3].c_str()) )
	config.setDouble( "u_min", atof(tokens[4].c_str()) )
	config.setDouble( "u_max", atof(tokens[5].c_str()) )
    } else if ( tokens.size() == 9 ) {
	config.setDouble( "Kp", atof(tokens[1].c_str()) )
	config.setDouble( "beta", atof(tokens[2].c_str()) )
	config.setDouble( "alpha", atof(tokens[3].c_str()) )
	config.setDouble( "gamma", atof(tokens[4].c_str()) )
	config.setDouble( "Ti", atof(tokens[5].c_str()) )
	config.setDouble( "Td", atof(tokens[6].c_str()) )
	config.setDouble( "u_min", atof(tokens[7].c_str()) )
	config.setDouble( "u_max", atof(tokens[8].c_str()) )
    } else {
	return False
    }

    return True
}
