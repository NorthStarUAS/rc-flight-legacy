//
// FILE: act_goldy2.cxx
// DESCRIPTION: send actuator commands to Goldy2
//

#include "python/pyprops.hxx"

#include <stdio.h>
#include <string>
#include <string.h>

#include "comms/netSocket.h"
#include "init/globals.hxx"
#include "util/timing.h"

#include "sensors/util_goldy2.hxx"
#include "act_goldy2.hxx"


static netSocket sock;
static int port = 0;
static string hostname = "";

// property nodes
static pyPropertyNode act_node;


// initialize goldy2 config property nodes
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
}


// function prototypes
bool goldy2_act_init( string output_path, pyPropertyNode *config ) {
    printf("actuator_init()\n");

    bind_input( config );
    bind_act_nodes( output_path );

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


#if 0
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
#endif


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
