//
// FILE: act_fgfs.cxx
// DESCRIPTION: send actuator commands to FlightGear
// of Flightgear
//

#include "python/pyprops.hxx"

#include <stdio.h>
#include <string>
#include <string.h>

#include "comms/netSocket.h"
#include "init/globals.hxx"
#include "util/timing.h"

#include "act_fgfs.hxx"

static netSocket sock;
static int port = 0;
static string hostname = "";

// property nodes
static pyPropertyNode act_node;
static pyPropertyNode targets_node;
static pyPropertyNode orient_node;
static pyPropertyNode pos_node;
static pyPropertyNode route_node;

// initialize fgfs_gps input property nodes
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("host") ) {
	hostname = config->getString("host");
    }
    if ( config->hasChild("port") ) {
	port = config->getLong("port");
    }
}


/// initialize actuator property nodes 
static void bind_act_nodes( string output_path ) {
    act_node = pyGetNode(output_path, true);
#define NUM_ACTUATORS 8
    act_node.setLen("channel", NUM_ACTUATORS, 0.0);
    targets_node = pyGetNode("/autopilot/targets", true);
    orient_node = pyGetNode("/orientation", true);
    pos_node = pyGetNode("/position", true);
    route_node = pyGetNode("/task/route", true);    
}


// function prototypes
bool fgfs_act_init( string output_path, pyPropertyNode *config ) {
    printf("actuator_init()\n");

    bind_input( config );
    bind_act_nodes(output_path);

    // open a UDP socket
    if ( ! sock.open( false ) ) {
	printf("Error opening imu input socket\n");
	return false;
    }

    // connect ...
    if ( sock.connect( hostname.c_str(), port ) == -1 ) {
	printf("error connecting to %s:%d\n", hostname.c_str(), port);
	return false;
    }

    // don't block
    sock.setBlocking( false );

    return true;
}


// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count )
{
    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
}


bool fgfs_act_update() {
    // additional autopilot target nodes (note this is a hack, but we
    // are sending data back to FG in this module so it makes some
    // sense to include autopilot targets.)

    const int fgfs_act_size = 76;
    uint8_t packet_buf[fgfs_act_size];
    uint8_t *buf = packet_buf;

    double time = act_node.getDouble("timestamp");
    *(double *)buf = time; buf += 8;

    float ail = act_node.getDouble("channel", 0);
    *(float *)buf = ail; buf += 4;

    float ele = act_node.getDouble("channel", 1);
    *(float *)buf = ele; buf += 4;

    float thr = act_node.getDouble("channel", 2);
    *(float *)buf = thr; buf += 4;

    float rud = act_node.getDouble("channel", 3);
    *(float *)buf = rud; buf += 4;

    float ch5 = act_node.getDouble("channel", 4);
    *(float *)buf = ch5; buf += 4;

    float ch6 = act_node.getDouble("channel", 5);
    *(float *)buf = ch6; buf += 4;

    float ch7 = act_node.getDouble("channel", 6);
    *(float *)buf = ch7; buf += 4;

    float ch8 = act_node.getDouble("channel", 7);
    *(float *)buf = ch8; buf += 4;

    float bank = targets_node.getDouble("roll_deg") * 100 + 18000.0;
    *(float *)buf = bank; buf += 4;

    float pitch = targets_node.getDouble("pitch_deg") * 100 + 9000.0;
    *(float *)buf = pitch; buf += 4;

    float target_track_offset = targets_node.getDouble("groundtrack_deg")
	- orient_node.getDouble("heading_deg");
    if ( target_track_offset < -180 ) { target_track_offset += 360.0; }
    if ( target_track_offset > 180 ) { target_track_offset -= 360.0; }
    float hdg = target_track_offset * 100 + 36000.0;
    *(float *)buf = hdg; buf += 4;

    // FIXME: no longer used so wasted 4 bytes ...
    float climb = targets_node.getDouble("climb_rate_fps") * 1000 + 100000.0;
    *(float *)buf = climb; buf += 4;

    float alt_agl_ft = targets_node.getDouble("altitude_agl_ft");
    float ground_m = pos_node.getDouble("altitude_ground_m");
    float alt_msl_ft = (ground_m * SG_METER_TO_FEET + alt_agl_ft) * 100.0;
    *(float *)buf = alt_msl_ft; buf += 4;

    float speed = targets_node.getDouble("target_speed_kt") * 100;
    *(float *)buf = speed; buf += 4;

    float track_offset = orient_node.getDouble("groundtrack_deg")
	- orient_node.getDouble("heading_deg");
    if ( track_offset < -180 ) { track_offset += 360.0; }
    if ( track_offset > 180 ) { track_offset -= 360.0; }
    float offset = track_offset * 100 + 36000.0;
    *(float *)buf = offset; buf += 4;

    float dist = route_node.getDouble("wp_dist_m") / 10.0;
    *(float *)buf = dist; buf += 4;

    float eta = route_node.getDouble("wp_eta_sec");
    *(float *)buf = eta; buf += 4;

    if ( ulIsLittleEndian ) {
	my_swap( packet_buf, 0, 8 );
	my_swap( packet_buf, 8, 4 );
	my_swap( packet_buf, 12, 4 );
	my_swap( packet_buf, 16, 4 );
	my_swap( packet_buf, 20, 4 );
	my_swap( packet_buf, 24, 4 );
	my_swap( packet_buf, 28, 4 );
	my_swap( packet_buf, 32, 4 );
	my_swap( packet_buf, 36, 4 );
	my_swap( packet_buf, 40, 4 );
	my_swap( packet_buf, 44, 4 );
	my_swap( packet_buf, 48, 4 );
	my_swap( packet_buf, 52, 4 );
	my_swap( packet_buf, 56, 4 );
	my_swap( packet_buf, 60, 4 );
	my_swap( packet_buf, 64, 4 );
	my_swap( packet_buf, 68, 4 );
	my_swap( packet_buf, 72, 4 );
    }

    int result = sock.send( packet_buf, fgfs_act_size, 0 );
    if ( result != fgfs_act_size ) {
	return false;
    }

    return true;
}


void fgfs_act_close() {
    sock.close();
}
