/**
 * \file: gpsd.cpp
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.cpp,v 1.7 2009/08/25 15:04:01 curt Exp $
 */

#include <string>

using std::string;

#include "globaldefs.h"

#include "comms/logging.h"
#include "comms/netSocket.h"
#include "props/props.hxx"
#include <util/strutils.hxx>
#include "util/timing.h"
#include "gps_mgr.h"

#include "gps_gpsd.h"


// gpsd property nodes
static SGPropertyNode *configroot = NULL;

static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

static SGPropertyNode *gps_device_name = NULL;
static SGPropertyNode *gps_satellites = NULL;
static SGPropertyNode *gps_nmode = NULL;

static SGPropertyNode *gpsd_port_node = NULL;
static SGPropertyNode *gpsd_host_node = NULL;


static int port = 2947;
static string host = "localhost";
static netSocket gpsd_sock;
static bool socket_connected = false;
static double last_init_time = 0.0;


// initialize gpsd input property nodes
static void bind_input( SGPropertyNode *config ) {
    gpsd_port_node = config->getChild("port");
    if ( gpsd_port_node != NULL ) {
	port = gpsd_port_node->getIntValue();
    }
    gpsd_host_node = config->getChild("host");
    if ( gpsd_host_node != NULL ) {
	host = gpsd_host_node->getStringValue();
    }
    configroot = config;
}


// initialize gpsd output property nodes 
static void bind_output( string rootname ) {
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);

    gps_device_name = outputroot->getChild("device-name", 0, true);
    gps_satellites = outputroot->getChild("satellites", 0, true);
    gps_nmode = outputroot->getChild("nmea-mode", 0, true);
}


void gpsd_init( string rootname, SGPropertyNode *config ) {
    bind_input( config );
    bind_output( rootname );
}


// send our configured init strings to configure gpsd the way we prefer
static void gpsd_send_init() {
    if ( !socket_connected ) {
	return;
    }

    for ( int i = 0; i < configroot->nChildren(); ++i ) {
        SGPropertyNode *child = configroot->getChild(i);
        string cname = child->getName();
        string cval = child->getStringValue();
	if ( cname == "init-string" ) {
	    if ( display_on ) {
		printf("sending to gpsd: %s\n", cval.c_str());
	    }
	    if ( gpsd_sock.send( cval.c_str(), cval.length() ) < 0 ) {
		socket_connected = false;
	    }
	}
    }

    last_init_time = get_Time();
}


// attempt to connect to gpsd
static void gpsd_connect() {
    // make sure it's closed
    gpsd_sock.close();

    if ( display_on ) {
	printf("Attempting to connect to gpsd @ %s:%d\n", host.c_str(), port);
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
	gps_device_name->setStringValue( gpsd_arg.c_str() );
    } else if ( gpsd_cmd == "N" ) {
	if ( gpsd_arg == "0" ) {
	    gps_nmode->setStringValue( "nmea ascii" );
        } else {
	    gps_nmode->setStringValue( "binary" );
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
	gps_unix_sec_node->setDoubleValue( atof(token[1].c_str()) );
	gps_lat_node->setDoubleValue( atof(token[3].c_str()) );
	gps_lon_node->setDoubleValue( atof(token[4].c_str()) );
	if ( token[5] != "?" ) {
	    gps_alt_node->setDoubleValue( atof(token[5].c_str()) );
        }
	if ( token[8] != "?" && token[9] != "?" ) {
	    double course_deg = atof( token[8].c_str() );
	    double speed_mps = atof( token[9].c_str() );
	    double angle_rad = (90.0 - course_deg) * SGD_DEGREES_TO_RADIANS;
	    gps_vn_node->setDoubleValue( sin(angle_rad) * speed_mps );
	    gps_ve_node->setDoubleValue( cos(angle_rad) * speed_mps );
	    /* printf("mps=%.1f deg=%.1f rad=%.3f vn=%.1f ve=%.1f\n",
		   speed_mps, course_deg, angle_rad,
		   gps_vn_node->getDoubleValue(),
		   gps_ve_node->getDoubleValue()); */
	}
	// if ( gps_data.date > last_unix_time && last_unix_time > 0.0 ) {
	//   gps_data.vd = (gps_data.alt - last_alt_m) * (gps_data.date - last_unix_time);
	//   last_alt_m = gps_data.alt;
	//    last_unix_time = gps_data.date;
	// }
	if ( token[10] != "?" ) {
	    gps_vd_node->setDoubleValue( -atof(token[10].c_str()) );
	}
	if ( gps_unix_sec_node->getDoubleValue() > last_gps_sec ) {
	    last_gps_sec = gps_unix_sec_node->getDoubleValue();
	    gps_timestamp_node->setDoubleValue( get_Time() );
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
	// gps_satellites->setStringValue( sentence );
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
	// mode of operation for ugear.
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
    double gps_timestamp = gps_timestamp_node->getDoubleValue();
    if ( get_Time() > gps_timestamp + 5 && get_Time() > last_init_time + 5 ) {
	gpsd_send_init();
    }

    return gps_data_valid;
 }


void gpsd_close() {
}
