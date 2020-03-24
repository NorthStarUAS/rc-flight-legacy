/**
 * \file: gps_gpsd.cpp
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright (C) 2012 - Curt Olson curtolson@flightgear.org
 *
 */

#include <pyprops.h>

#include <string>
using std::string;

#include "include/globaldefs.h"

#include "comms/display.h"
#include "util/netSocket.h"
#include "util/strutils.h"
#include "util/timing.h"
#include "gps_mgr.h"

#include "gps_gpsd.h"


// property nodes
static pyPropertyNode gps_node;

static int port = 2947;
static string host = "localhost";
static string init_string;
static netSocket gpsd_sock;
static bool socket_connected = false;
static double last_init_time = 0.0;


// initialize gpsd input property nodes
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("port") ) {
	port = config->getLong("port");
    }
    if ( config->hasChild("host") ) {
	host = config->getString("host");
    }
    if ( config->hasChild("init_string") ) {
	init_string = config->getString("init_string");
    }
}


// initialize gpsd output property nodes 
static void bind_output( string output_path ) {
    gps_node = pyGetNode(output_path, true);
}


void gpsd_init( string output_path, pyPropertyNode *config ) {
    bind_input( config );
    bind_output( output_path );
}


// send our configured init strings to configure gpsd the way we prefer
static void gpsd_send_init() {
    if ( !socket_connected ) {
	return;
    }

    if ( init_string != "" ) {
	if ( display_on ) {
	    printf("sending to gpsd: %s\n", init_string.c_str());
	}
	if ( gpsd_sock.send( init_string.c_str(), init_string.length() ) < 0 ) {
	    socket_connected = false;
	}
    }

    last_init_time = get_Time();
}


// attempt to connect to gpsd
static void gpsd_connect() {
    // make sure it's closed
    gpsd_sock.close();

    if ( display_on ) {
	printf("Attempting to connect to gpsd @ %s:%d ... ",
	       host.c_str(), port);
    }

    if ( ! gpsd_sock.open( true ) ) {
	if ( display_on ) {
	    printf("error opening gpsd socket\n");
	}
	return;
    }
    
    if (gpsd_sock.connect( host.c_str(), port ) < 0) {
	if ( display_on ) {
	    printf("error connecting to gpsd\n");
	}
	return;
    }

    gpsd_sock.setBlocking( false );

    socket_connected = true;

    gpsd_send_init();

    if ( display_on ) {
	printf("success!\n");
    }
}


static bool parse_gpsd_sentence( const char *sentence ) {
    static double last_gps_sec = 0.0;
    bool new_position = false;

    // printf("%s", sentence);

    vector <string> token = split( sentence, " " );
    if ( token.size() < 1 ) {
        // no valid tokens
        return false;
    }

    int pos = token[0].find(",");
    string tmp1 = "";
    string gpsd_cmd = "";
    string gpsd_arg = "";
    if ( pos >= 0 ) {
 	tmp1 = token[0].substr(pos + 1);
        pos = tmp1.find("=");
	if ( pos >= 0 ) {
            gpsd_cmd = tmp1.substr(0, pos);
	    gpsd_arg = tmp1.substr(pos + 1);
            // printf("cmd = %s  arg = %s\n", gpsd_cmd.c_str(), gpsd_arg.c_str() ); 
        }
    }

    if ( gpsd_cmd == "F" ) {
	gps_node.setString( "device_name", gpsd_arg.c_str() );
    } else if ( gpsd_cmd == "N" ) {
	if ( gpsd_arg == "0" ) {
	    gps_node.setString( "nmea_mode", "nmea ascii" );
        } else {
	    gps_node.setString( "nmea_mode", "binary" );
        }
    } else if ( gpsd_cmd == "O" && 
                (gpsd_arg == "GGA" || gpsd_arg == "GLL" ||
		 gpsd_arg == "GND" || gpsd_arg == "GSA" ||
		 gpsd_arg == "MID2" || gpsd_arg == "MID4" ||
		 gpsd_arg == "RMC" || gpsd_arg == "0x0106") ) {
	// Output of GPSD "O" (ohhh) command
	//
	// example: GPSD,O=RMC 1232073262.000 0.005 45.138145
	// -93.157083 285.50 3.60 1.80 181.4300 0.046 0.000 ? 7.20 ? 3
	gps_node.setDouble( "unix_time_sec", atof(token[1].c_str()) );
	gps_node.setDouble( "latitude_deg", atof(token[3].c_str()) );
	gps_node.setDouble( "longitude_deg", atof(token[4].c_str()) );
	if ( token[5] != "?" ) {
	    gps_node.setDouble( "altitude_m", atof(token[5].c_str()) );
        }
	if ( token[8] != "?" && token[9] != "?" ) {
	    double course_deg = atof( token[8].c_str() );
	    double speed_mps = atof( token[9].c_str() );
	    double angle_rad = (90.0 - course_deg) * SGD_DEGREES_TO_RADIANS;
	    gps_node.setDouble( "vn_ms", sin(angle_rad) * speed_mps );
	    gps_node.setDouble( "ve_ms", cos(angle_rad) * speed_mps );
	    /* printf("mps=%.1f deg=%.1f rad=%.3f vn=%.1f ve=%.1f\n",
		   speed_mps, course_deg, angle_rad,
		   gps_vn_node->getDouble(),
		   gps_ve_node->getDouble()); */
	}
	// if ( gps_data.date > last_unix_time && last_unix_time > 0.0 ) {
	//   gps_data.vd = (gps_data.alt - last_alt_m) * (gps_data.date - last_unix_time);
	//   last_alt_m = gps_data.alt;
	//    last_unix_time = gps_data.date;
	// }
	if ( token[10] != "?" ) {
	    gps_node.setDouble( "vd_ms", -atof(token[10].c_str()) );
	}
	if ( gps_node.getDouble("unix_time_sec") > last_gps_sec ) {
	    last_gps_sec = gps_node.getDouble("unix_time_sec");
	    gps_node.setDouble( "timestamp", get_Time() );
	    new_position = true;
	}
    } else if ( gpsd_cmd == "Y" && 
		(gpsd_arg == "GSV" || gpsd_arg == "0x0130") ) {
	// Output of GPSD "Y" command 
	// 
	// GPSD,Y=GSV 1232073259.000 11:14 76 358 0 0:22 55 126 41
	// 1:31 41 198 35 1:5 27 82 36 1:30 24 111 32 1:12 22 69 25
	// 1:11 21 302 26 1:18 20 125 38 1:32 17 304 28 1:9 8 41 14
	// 1:51 36 199 37 0:
	//
	// gps_satellites.setString( sentence );
#if 0 // depricated ... could have original been a bug in gpsd for this gps?
    } else if ( gpsd_cmd == "O" &&
                (gpsd_arg == "MID2" || gpsd_arg == "MID4") ) {
	// still in "unreliable" binary mode, resend init sequence
	gpsd_send_init();
#endif
    } else if ( gpsd_cmd == "W" ) {
	// reports if "watcher" mode is set (1) or unset (0).  In watcher mode
	// gpsd streams gps messages to the connected clients as they come in
	// rather than waiting for the client to poll.  This is the default
	// mode of operation.
    } else if ( gpsd_cmd == "X" ) {
	// according to the docs, gpsd_arg will be 0 when the gps is offline
	// or "n" with the timestamp of the most recent message
    } else {
	if ( display_on ) {
	    printf("Unknown GPSD sentence: %s", sentence);
	}
    }

    return new_position;
}


bool gpsd_get_gps() {
    bool gps_data_valid = false;

    if ( !socket_connected ) {
	gpsd_connect();
    }
 
    char gpsd_sentence[256];
    int result;
    while ( (result = gpsd_sock.recv( gpsd_sentence, 256 )) > 0 ) {
	gpsd_sentence[result] = 0;
	if ( parse_gpsd_sentence( gpsd_sentence ) ) {
	    gps_data_valid = true;
	}
    }
    if ( errno != EAGAIN ) {
	if ( display_on ) {
	    perror("gpsd recv");
	}
	socket_connected = false;
    }

    // If more than 5 seconds has elapsed without seeing new data and
    // our last init attempt was more than 5 seconds ago, try
    // resending the init sequence.
    double gps_timestamp = gps_node.getDouble("timestamp");
    if ( get_Time() > gps_timestamp + 5 && get_Time() > last_init_time + 5 ) {
	gpsd_send_init();
    }

    return gps_data_valid;
 }


void gpsd_close() {
}
