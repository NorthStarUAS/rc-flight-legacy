//
// FILE: act_goldy2.cxx
// DESCRIPTION: send actuator commands to Goldy2
//

#include <stdio.h>
#include <string>
#include <string.h>

#include "comms/netSocket.h"
#include "init/globals.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "sensors/util_goldy2.hxx"
#include "act_goldy2.hxx"


static netSocket sock;
static int port = 0;
static string hostname = "";

// goldy2 config property nodes
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


// initialize goldy2 config property nodes
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
bool goldy2_act_init( SGPropertyNode *config ) {
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


bool goldy2_act_update() {
    static uint16_t pos = 1000;
    const int goldy2_act_size = 68;
    const int packet_size = 60;	// 6*n, n=10
    uint8_t packet_buf[goldy2_act_size];
    uint8_t *start_buf = packet_buf;
    uint8_t *buf = packet_buf;

    *(char *)buf = 'U'; buf++;
    *(char *)buf = 'M'; buf++;
    *(char *)buf = 'N'; buf++;
    *(uint8_t *)buf = 0x01; buf++;	  // payload type
    *(uint8_t *)buf = packet_size; buf++; // LSB
    *(uint8_t *)buf = 0; buf++;		  // MSB

    if (pos++ > 2000) {
	pos = 1000;
    }

    for ( uint8_t i = 0; i < 10; i++ ) {
	*(uint8_t *)buf = i; buf++;
	*(uint8_t *)buf = 0; buf++; // 0 = PWM, 1 = angle control
	*(uint16_t *)buf = pos; buf += 2;
	*(int16_t *)buf = 0; buf += 2;
    }

    uint16_t CRC = utilCRC16(start_buf+3, packet_size+3, 0);
    uint16_t CRC_msb = CRC / 256;
    uint16_t CRC_lsb = CRC - (CRC_msb * 256);
    *(uint8_t *)buf = CRC_lsb; buf++;
    *(uint8_t *)buf = CRC_msb; buf++;
    /* printf("actual size = %d (we hoped for %d)\n", (int)(buf - start_buf), goldy2_act_size); */

    int result = sock.send( packet_buf, goldy2_act_size, 0 );
    if ( result != goldy2_act_size ) {
	return false;
    }

    return true;
}


void goldy2_act_close() {
    sock.close();
}
