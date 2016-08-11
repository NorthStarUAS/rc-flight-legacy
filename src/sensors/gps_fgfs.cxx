//
// FILE: gps_fgfs.cxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#include "python/pyprops.hxx"

#include <stdio.h>
#include <string>
#include <string.h>

#include "comms/netSocket.h"
#include "util/timing.h"

#include "gps_fgfs.hxx"


static netSocket sock;
static int port = 0;

// property nodes
static pyPropertyNode gps_node;

// initialize fgfs_gps input property nodes
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("port") ) {
	port = config->getLong("port");
    }
}


// initialize gps output property nodes 
static void bind_gps_output( string output_path ) {
    gps_node = pyGetNode(output_path, true);
}


// function prototypes
bool fgfs_gps_init( string output_path, pyPropertyNode *config ) {
    bind_input( config );
    bind_gps_output( output_path );

    // open a UDP socket
    if ( ! sock.open( false ) ) {
	printf("Error opening imu input socket\n");
	return false;
    }

    // bind ...
    if ( sock.bind( "", port ) == -1 ) {
	printf("error binding to port %d\n", port );
	return false;
    }

    // don't block waiting for input
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


bool fgfs_gps_update() {
    const int fgfs_gps_size = 40;
    uint8_t packet_buf[fgfs_gps_size];

    bool fresh_data = false;

    int result;
    while ( (result = sock.recv(packet_buf, fgfs_gps_size, 0))
	    == fgfs_gps_size )
    {
	fresh_data = true;

	if ( ulIsLittleEndian ) {
	    my_swap( packet_buf, 0, 8 );
	    my_swap( packet_buf, 8, 8 );
	    my_swap( packet_buf, 16, 8 );
	    my_swap( packet_buf, 24, 4 );
	    my_swap( packet_buf, 28, 4 );
	    my_swap( packet_buf, 32, 4 );
	    my_swap( packet_buf, 36, 4 );
	}

	uint8_t *buf = packet_buf;
	double time = *(double *)buf; buf += 8;
	double lat = *(double *)buf; buf += 8;
	double lon = *(double *)buf; buf += 8;
	float alt = *(float *)buf; buf += 4;
	float vn = *(float *)buf; buf += 4;
	float ve = *(float *)buf; buf += 4;
	float vd = *(float *)buf; buf += 4;

	gps_node.setDouble( "timestamp", get_Time() );
	gps_node.setDouble( "latitude_deg", lat );
	gps_node.setDouble( "longitude_deg", lon );
	gps_node.setDouble( "altitude_m", alt );
	gps_node.setDouble( "vn_ms", vn );
	gps_node.setDouble( "ve_ms", ve );
	gps_node.setDouble( "vd_ms", vd );
	gps_node.setLong( "satellites", 8 ); // fake a solid number
	gps_node.setDouble( "unix_time_sec", time );
	gps_node.setLong( "status", 2 ); // valid fix
    }

    return fresh_data;
}


void fgfs_gps_close() {
    sock.close();
}
