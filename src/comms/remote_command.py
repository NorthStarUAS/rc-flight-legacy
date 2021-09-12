import random
import re
import time

from PropertyTree import PropertyNode

from comms import rc_messages
import comms.events

import survey.survey

route_node = PropertyNode("/task/route")
task_node = PropertyNode("/task")
home_node = PropertyNode("/task/home")
active_node = PropertyNode("/task/route/active")
targets_node = PropertyNode("/autopilot/targets")
comms_node = PropertyNode("/comms")

# set up the remote command processor
def init():
    pass
    
def update():
    len = comms_node.getLen("commands")
    for i in range(len):
        command = comms_node.getString("commands", i)
        if command != "":
            print("command:", command)
            execute_command(command)
            comms_node.setString("commands", "", i)
    
route_request = []
survey_request = {}
def execute_command( command ):
    global route_request
    global survey_request

    if command == "":
        # no valid tokens
        return

    tokens = command.split(",")
    if tokens[0] == "hb" and len(tokens) == 1:
        # heart beat, no action needed
        pass
    elif tokens[0] == "home" and len(tokens) == 5:
        # specify new home location
        lon = float( tokens[1] )
        lat = float( tokens[2] )
        # alt_ft = float( tokens[3] )
        azimuth_deg = float( tokens[4] )
        home_node.setDouble( "longitude_deg", lon )
        home_node.setDouble( "latitude_deg", lat )
        home_node.setDouble( "azimuth_deg", azimuth_deg )
        home_node.setBool( "valid", True )
    elif tokens[0] == "route" and len(tokens) >= 5:
        route_request = tokens[1:]
    elif tokens[0] == "route_cont" and len(tokens) >= 5:
        route_request += tokens[1:]
    elif tokens[0] == "route_end" and len(tokens) == 1:
        route_node.setString( "route_request", ",".join(route_request) )
        task_node.setString( "command", "route" )
    elif tokens[0] == "survey_start" and len(tokens) == 7:
        for i, tok in enumerate(tokens):
            if tokens[i] == "undefined":
                tokens[i] = 0.0
        survey_request["agl_ft"] = float( tokens[1] )
        survey_request["extend_m"] = float( tokens[2] )
        survey_request["overlap_perc"] = float( tokens[3] )
        survey_request["sidelap_perc"] = float( tokens[4] )
        survey_request["forward_fov"] = float( tokens[5] )
        survey_request["lateral_fov"] = float( tokens[6] )
        survey_request["area"] = []
    elif tokens[0] == "survey_cont" and len(tokens) > 2:
        for i in range(1, len(tokens), 2):
            wpt = ( float(tokens[i]), float(tokens[i+1]) )
            survey_request["area"].append( wpt )            
    elif tokens[0] == "survey_end" and len(tokens) == 1:
        survey.survey.do_survey(survey_request)
    elif tokens[0] == "task" and len(tokens) > 1:
        task_node.setString( "command", ",".join(tokens[1:]) )
    elif tokens[0] == "ap" and len(tokens) == 3:
        # specify an autopilot target
        if tokens[1] == "agl-ft":
            agl_ft = float( tokens[2] )
            targets_node.setDouble( "altitude_agl_ft", agl_ft )
        elif tokens[1] == "msl-ft":
            msl_ft = float( tokens[2] )
            targets_node.setDouble( "target_msl_ft", msl_ft )
        elif tokens[1] == "speed-kt":
            speed_kt = float( tokens[2] )
            targets_node.setDouble( "airspeed_kt", speed_kt )
    elif tokens[0] == "fcs-update":
        decode_fcs_update( command )
    elif tokens[0] == "get" and len(tokens) == 2:
        # absolute path
        parts = tokens[1].split("/")
        node_path = "/".join(parts[0:-1])
        if node_path == "":
            node_path = "/"
        node = PropertyNode(node_path)
        name = parts[-1]
        value = node.getString(name)
        if value == "": value = "undefined"
        # print tokens[0], "=", value
        return_msg = "get: %s,%s" % (tokens[1], value)
        event = rc_messages.event_v2()
        event.message = return_msg
        buf = event.pack()
        send_message(event.id, buf)
        comms.events.log("get", "%s,%s" % (tokens[1], value))
    elif tokens[0] == "set" and len(tokens) >= 3:
        if tokens[1][0] == "/":
            # absolute path
            parts = tokens[1].split("/")
            node_path = "/".join(parts[0:-1])
            if node_path == "":
                node_path = "/"
            node = PropertyNode(node_path)
            name = parts[-1]
            value = " ".join(tokens[2:])
            done = False
            # test for int
            if not done:
                result = re.match("[-+]?\d+", value)
                if result and result.group(0) == value:
                    print("int:", value)
                    node.setInt(name, int(value))
                    done = True
            # test for float
            if not done:
                result = re.match("[-+]?\d*\.\d+", value)
                if result and result.group(0) == value:
                    print("float:", value)
                    node.setDouble(name, float(value))
                    done = True
            # test for bool
            if not done:
                if value == "True" or value == "true":
                    print("bool:", True)
                    node.setBool(name, True)
                    done = True
            if not done:
                if value == "False" or value == "false":
                    print("bool:", False)
                    node.setBool(name, False)
                    done = True
            # fall back to string
            if not done:
                node.setString(name, value)
        else:
            # expecting a full path name to set
            pass
    elif tokens[0] == "la" and len(tokens) == 5:
        if tokens[1] == "ned":
	    # set ned-vector lookat mode
            point_node = PropertyNode("/pointing")
            point_node.setString("lookat_mode", "ned_vector")
	    # specify new lookat ned coordinates
            vector_node = PropertyNode("/pointing/vector")
            north = float( tokens[2] )
            east = float( tokens[3] )
            down = float( tokens[4] )
            vector_node.setDouble( "north", north )
            vector_node.setDouble( "east", east )
            vector_node.setDouble( "down", down )
        elif tokens[1] == "wgs84":
            # set wgs84 lookat mode
            point_node = PropertyNode("/pointing")
            point_node.setString("lookat_mode", "wgs84")
            # specify new lookat ned coordinates
            wgs84_node = PropertyNode("/pointing/wgs84")
            pos_node = PropertyNode("/position")
            lon = float( tokens[2] )
            lat = float( tokens[3] )
            wgs84_node.setDouble( "longitude_deg", lon )
            wgs84_node.setDouble( "latitude_deg", lat )
            ground = pos_node.getDouble("altitude_ground_m")
            wgs84_node.setDouble( "altitude_m", ground )

def decode_fcs_update(command):
    tokens = command.split(",")

    # valid sizes will be 7 or 10 at this point
    if tokens[0] == "fcs-update" and len(tokens) >= 7:
        # remove initial keyword if it exists
        del tokens[0]

    ap_config = PropertyNode("/config/autopilot")

    i = int(tokens[0])
    component = ap_config.getChild("component[%d]" % i)
    if not component:
        return False

    config = component.getChild("config")
    if not config:
        return False

    if len(tokens) == 7:
        config.setDouble( "Kp", float(tokens[1]) )
        config.setDouble( "Ti", float(tokens[2]) )
        config.setDouble( "Td", float(tokens[3]) )
        config.setDouble( "u_min", float(tokens[4]) )
        config.setDouble( "u_max", float(tokens[5]) )
        config.setDouble( "u_trim", float(tokens[6]) )
    elif len(tokens) == 10:
        config.setDouble( "Kp", float(tokens[1]) )
        config.setDouble( "beta", float(tokens[2]) )
        config.setDouble( "alpha", float(tokens[3]) )
        config.setDouble( "gamma", float(tokens[4]) )
        config.setDouble( "Ti", float(tokens[5]) )
        config.setDouble( "Td", float(tokens[6]) )
        config.setDouble( "u_min", float(tokens[7]) )
        config.setDouble( "u_max", float(tokens[8]) )
        config.setDouble( "u_trim", float(tokens[6]) )
    else:
        return False

    return True
