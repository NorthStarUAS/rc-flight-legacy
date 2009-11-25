//
// FILE: gps_fgfs.cxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#include <stdio.h>
#include <string>
#include <string.h>

#include "include/ugear_config.h"

#include "comms/netSocket.h"
#include "props/props.hxx"
#include "util/timing.h"

#include "gps_fgfs.hxx"


static netSocket sock;
static int port = 0;

// fgfs_imu property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *outputroot = NULL;
static SGPropertyNode *gps_port_node = NULL;

static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;


// initialize fgfs_gps input property nodes
static void bind_input( SGPropertyNode *config ) {
    gps_port_node = config->getChild("port");
    if ( gps_port_node != NULL ) {
	port = gps_port_node->getIntValue();
    }
    configroot = config;
}


/// initialize gps output property nodes 
static void bind_gps_output( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );

    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);
}


// function prototypes
bool fgfs_gps_init( string rootname, SGPropertyNode *config ) {
    bind_input( config );
    bind_gps_output( rootname );

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

	/*
	  my_swap( packet_buf, 0, 8 );
	  my_swap( packet_buf, 8, 4 );
	  my_swap( packet_buf, 12, 4 );
	  my_swap( packet_buf, 16, 4 );
	  my_swap( packet_buf, 20, 4 );
	  my_swap( packet_buf, 24, 4 );
	  my_swap( packet_buf, 28, 4 );
	*/

	uint8_t *buf = packet_buf;
	double time = *(double *)buf; buf += 8;
	double lat = *(double *)buf; buf += 8;
	double lon = *(double *)buf; buf += 8;
	float alt = *(float *)buf; buf += 4;
	float vn = *(float *)buf; buf += 4;
	float ve = *(float *)buf; buf += 4;
	float vd = *(float *)buf; buf += 4;

	gps_timestamp_node->setDoubleValue( get_Time() );
	gps_lat_node->setDoubleValue( lat );
	gps_lon_node->setDoubleValue( lon );
	gps_alt_node->setDoubleValue( alt );
	gps_vn_node->setDoubleValue( vn );
	gps_ve_node->setDoubleValue( ve );
	gps_vd_node->setDoubleValue( vd );
	gps_unix_sec_node->setDoubleValue( time );
    }

    return fresh_data;
}


void fgfs_gps_close() {
    sock.close();
}
