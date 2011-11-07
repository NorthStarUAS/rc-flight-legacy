//
// FILE: act_fgfs.cxx
// DESCRIPTION: send actuator commands to FlightGear
// of Flightgear
//

#include <stdio.h>
#include <string>
#include <string.h>

#include "include/ugear_config.h"

#include "comms/netSocket.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "act_fgfs.hxx"


static netSocket sock;
static int port = 0;
static string hostname = "";

// fgfs_imu property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *act_host_node = NULL;
static SGPropertyNode *act_port_node = NULL;

// actuator property nodes
static SGPropertyNode *act_timestamp_node = NULL;
static SGPropertyNode *act_aileron_node = NULL;
static SGPropertyNode *act_elevator_node = NULL;
static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *act_rudder_node = NULL;
static SGPropertyNode *act_channel5_node = NULL;
static SGPropertyNode *act_channel6_node = NULL;
static SGPropertyNode *act_channel7_node = NULL;
static SGPropertyNode *act_channel8_node = NULL;

// additional autopilot target nodes (note this is a hack, but we are
// sending data back to FG in this module so it makes some sense to
// include autopilot targets.)
static SGPropertyNode *ap_target_bank_deg = NULL;
static SGPropertyNode *ap_target_pitch_deg = NULL;
static SGPropertyNode *ap_target_ground_track_deg = NULL;
static SGPropertyNode *ap_target_climb_fps = NULL;
static SGPropertyNode *filter_ground_alt_m_node = NULL;
static SGPropertyNode *ap_altitude_agl = NULL;
static SGPropertyNode *ap_target_speed_kt = NULL;
static SGPropertyNode *ap_heading_deg = NULL;
static SGPropertyNode *ap_ground_track_deg = NULL;
static SGPropertyNode *ap_dist_m = NULL;
static SGPropertyNode *ap_eta_sec = NULL;

// initialize fgfs_gps input property nodes
static void bind_input( SGPropertyNode *config ) {
    act_host_node = config->getChild("host");
    if ( act_host_node != NULL ) {
	hostname = act_host_node->getStringValue();
    }
    act_port_node = config->getChild("port");
    if ( act_port_node != NULL ) {
	port = act_port_node->getIntValue();
    }
    configroot = config;
}


/// initialize actuator property nodes 
static void bind_act_nodes() {
    act_timestamp_node = fgGetNode("/actuators/actuator/time-stamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);

    ap_target_bank_deg = fgGetNode("/autopilot/settings/target-roll-deg", true);
    ap_target_pitch_deg = fgGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_heading_deg = fgGetNode( "/orientation/heading-deg", true );
    ap_target_ground_track_deg = fgGetNode( "/autopilot/settings/target-groundtrack-deg", true );
    ap_target_climb_fps = fgGetNode("/autopilot/internal/target-climb-rate-fps", true);
    filter_ground_alt_m_node
	= fgGetNode("/position/ground-altitude-filter-m", true);
    ap_altitude_agl = fgGetNode( "/autopilot/settings/target-agl-ft", true );
    ap_target_speed_kt = fgGetNode( "/autopilot/settings/target-speed-kt", true );
    ap_ground_track_deg = fgGetNode( "/orientation/groundtrack-deg", true );
    ap_dist_m = fgGetNode( "/autopilot/route-mgr/wp-dist-m", true );
    ap_eta_sec =  fgGetNode( "/autopilot/route-mgr/wp-eta-m", true );
}


// function prototypes
bool fgfs_act_init( SGPropertyNode *config ) {
    printf("actuator_init()\n");

    bind_input( config );
    bind_act_nodes();

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
    const int fgfs_act_size = 76;
    uint8_t packet_buf[fgfs_act_size];
    uint8_t *buf = packet_buf;

    double time = act_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    float ail = act_aileron_node->getFloatValue();
    *(float *)buf = ail; buf += 4;

    float ele = act_elevator_node->getFloatValue();
    *(float *)buf = ele; buf += 4;

    float thr = act_throttle_node->getFloatValue();
    *(float *)buf = thr; buf += 4;

    float rud = act_rudder_node->getFloatValue();
    *(float *)buf = rud; buf += 4;

    float ch5 = act_channel5_node->getFloatValue();
    *(float *)buf = ch5; buf += 4;

    float ch6 = act_channel6_node->getFloatValue();
    *(float *)buf = ch6; buf += 4;

    float ch7 = act_channel7_node->getFloatValue();
    *(float *)buf = ch7; buf += 4;

    float ch8 = act_channel8_node->getFloatValue();
    *(float *)buf = ch8; buf += 4;

    float bank = ap_target_bank_deg->getFloatValue() * 100 + 18000.0;
    *(float *)buf = bank; buf += 4;

    float pitch = ap_target_pitch_deg->getFloatValue() * 100 + 9000.0;
    *(float *)buf = pitch; buf += 4;

    float target_track_offset = ap_target_ground_track_deg->getFloatValue()
	- ap_heading_deg->getFloatValue();
    if ( target_track_offset < -180 ) { target_track_offset += 360.0; }
    if ( target_track_offset > 180 ) { target_track_offset -= 360.0; }
    float hdg = target_track_offset * 100 + 36000.0;
    *(float *)buf = hdg; buf += 4;

    float climb = ap_target_climb_fps->getFloatValue() * 1000 + 100000.0;
    *(float *)buf = climb; buf += 4;

    float alt_agl_ft = ap_altitude_agl->getFloatValue();
    float ground_m = filter_ground_alt_m_node->getFloatValue();
    float alt_msl_ft = (ground_m * SG_METER_TO_FEET + alt_agl_ft) * 100.0;
    *(float *)buf = alt_msl_ft; buf += 4;

    float speed = ap_target_speed_kt->getFloatValue() * 100;
    *(float *)buf = speed; buf += 4;

    float track_offset = ap_ground_track_deg->getFloatValue()
	- ap_heading_deg->getFloatValue();
    if ( track_offset < -180 ) { track_offset += 360.0; }
    if ( track_offset > 180 ) { track_offset -= 360.0; }
    float offset = track_offset * 100 + 36000.0;
    *(float *)buf = offset; buf += 4;

    float dist = ap_dist_m->getFloatValue() / 10.0;
    *(float *)buf = dist; buf += 4;

    float eta = ap_eta_sec->getFloatValue();
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
