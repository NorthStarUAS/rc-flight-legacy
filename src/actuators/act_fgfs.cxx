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
    const int fgfs_act_size = 40;
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
