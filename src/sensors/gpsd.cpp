/**
 * \file: gpsd.cpp
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.cpp,v 1.5 2009/05/13 22:09:04 curt Exp $
 */

#include <string>

using std::string;

#include "globaldefs.h"

#include "comms/logging.h"
#include "props/props.hxx"
#include "util/netSocket.h"
#include <util/strutils.hxx>
#include "util/timing.h"
#include "gps_mgr.h"

#include "gpsd.h"


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

static SGPropertyNode *gpsd_port_node = NULL;
static SGPropertyNode *gpsd_host_node = NULL;


static int port = 2947;
static string host = "localhost";
static netSocket gpsd_sock;
static bool socket_connected = false;
static struct gps gps_data;
static double last_init_time = 0.0;


void gpsd_init( string rootname, SGPropertyNode *config ) {
    // initialize gpsd property nodes 
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);

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


// send our configured init strings to configure gpsd the way we prefer
static void gpsd_send_init() {
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
        printf("error opening gpsd socket\n");
	return;
    }
    
    if (gpsd_sock.connect( host.c_str(), port ) < 0) {
        printf("error connecting to gpsd\n");
	return;
    }

    gpsd_sock.setBlocking( false );

    socket_connected = true;

    gpsd_send_init();
}


static bool parse_gpsd_sentence( const char *sentence ) {
    static bool new_position = false;

    vector <string> token = split( sentence, " " );
    if ( token.size() < 1 ) {
        // no valid tokens
        return false;
    }

    if ( token[0] == "GPSD,O=GGA" || token[0] == "GPSD,O=GSA"
	 || token[0] == "GPSD,O=RMC" )
    {
	// Output of GPSD "O" (ohhh) command
	//
	// example: GPSD,O=RMC 1232073262.000 0.005 45.138145
	// -93.157083 285.50 3.60 1.80 181.4300 0.046 0.000 ? 7.20 ? 3
	gps_data.date = atof( token[1].c_str() );
	gps_data.lat = atof( token[3].c_str() );
	gps_data.lon = atof( token[4].c_str() );
	gps_data.alt = atof( token[5].c_str() );
	if ( token[8] != "?" && token[9] != "?" ) {
	    double course_deg = atof( token[8].c_str() );
	    double speed_mps = atof( token[9].c_str() );
	    double angle_rad = (90.0 - course_deg) * SGD_DEGREES_TO_RADIANS;
	    gps_data.vn = cos(angle_rad) * speed_mps;
	    gps_data.ve = sin(angle_rad) * speed_mps;
	    // update structure time stamp only after we get ground
	    // track data.  This means the new position report is
	    // delayed until ground track info is sent over, but
	    // because the kalman filter takes all these parameters in
	    // one shot, I feel it's better to delay the position
	    // slightly instead delaying the ground track data by an
	    // entire second.
	    gps_data.time = get_Time();
	    new_position = true;
	}
	// if ( gps_data.date > last_unix_time && last_unix_time > 0.0 ) {
	//   gps_data.vn = (gps_data.alt - last_alt_m) * (gps_data.date - last_unix_time);
	//   last_alt_m = gps_data.alt;
	//    last_unix_time = gps_data.date;
	// }
	gps_data.vn = atof( token[10].c_str() );
	gps_data.status = ValidData;
    } else if ( token[0] == "GPSD,Y=GSV" ) {
	// Output of GPSD "Y" command 
	// 
	// GPSD,Y=GSV 1232073259.000 11:14 76 358 0 0:22 55 126 41
	// 1:31 41 198 35 1:5 27 82 36 1:30 24 111 32 1:12 22 69 25
	// 1:11 21 302 26 1:18 20 125 38 1:32 17 304 28 1:9 8 41 14
	// 1:51 36 199 37 0:
    } else if ( token[0] == "GPSD,O=MID2" || token[0] == "GPSD,O=MID4" ) {
	// still in "unreliable" binary mode, resend init sequence
	gpsd_send_init();
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

    // If more than 5 seconds has elapsed without seeing new data and
    // our last init attempt was more than 5 seconds ago, try
    // resending the init sequence.
    if ( get_Time() > gps_data.time + 5 && get_Time() > last_init_time + 5 ) {
	gpsd_send_init();
    }

    if ( gps_data_valid ) {
	gps_timestamp_node->setIntValue( gps_data.time );
	gps_lat_node->setDoubleValue( gps_data.lat );
	gps_lon_node->setDoubleValue( gps_data.lon );
	gps_alt_node->setDoubleValue( gps_data.alt );
	gps_ve_node->setDoubleValue( gps_data.ve );
	gps_vn_node->setDoubleValue( gps_data.vn );
	gps_vd_node->setDoubleValue( gps_data.vd );
	gps_unix_sec_node->setDoubleValue( gps_data.date );
    }

    return gps_data_valid;
 }


void gpsd_close() {
}
