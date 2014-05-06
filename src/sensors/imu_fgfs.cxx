//
// FILE: fgfs_imu.cxx
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

#include "imu_fgfs.hxx"


static netSocket sock;
static int port = 0;

// fgfs_imu property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *outputroot = NULL;
static SGPropertyNode *imu_port_node = NULL;

static SGPropertyNode *imu_timestamp_node = NULL;
static SGPropertyNode *imu_p_node = NULL;
static SGPropertyNode *imu_q_node = NULL;
static SGPropertyNode *imu_r_node = NULL;
static SGPropertyNode *imu_ax_node = NULL;
static SGPropertyNode *imu_ay_node = NULL;
static SGPropertyNode *imu_az_node = NULL;
static SGPropertyNode *imu_hx_node = NULL;
static SGPropertyNode *imu_hy_node = NULL;
static SGPropertyNode *imu_hz_node = NULL;

static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_airspeed_node = NULL;
static SGPropertyNode *airdata_pressure_node = NULL;

static SGPropertyNode *imu_roll_truth_node = NULL;
static SGPropertyNode *imu_pitch_truth_node = NULL;
static SGPropertyNode *imu_yaw_truth_node = NULL;

static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *fake_extern_volts_node = NULL;
static SGPropertyNode *fake_extern_amps_node = NULL;
static SGPropertyNode *fake_extern_current_node = NULL;

static bool airdata_inited = false;


// initialize fgfs_imu input property nodes
static void bind_input( SGPropertyNode *config ) {
    imu_port_node = config->getChild("port");
    if ( imu_port_node != NULL ) {
	port = imu_port_node->getIntValue();
    }
    configroot = config;
}


// initialize imu output property nodes 
static void bind_imu_output( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );

    imu_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    imu_p_node = outputroot->getChild("p-rad_sec", 0, true);
    imu_q_node = outputroot->getChild("q-rad_sec", 0, true);
    imu_r_node = outputroot->getChild("r-rad_sec", 0, true);
    imu_ax_node = outputroot->getChild("ax-mps_sec", 0, true);
    imu_ay_node = outputroot->getChild("ay-mps_sec", 0, true);
    imu_az_node = outputroot->getChild("az-mps_sec", 0, true);
    imu_hx_node = outputroot->getChild("hx", 0, true);
    imu_hy_node = outputroot->getChild("hy", 0, true);
    imu_hz_node = outputroot->getChild("hz", 0, true);

    imu_roll_truth_node = outputroot->getChild("roll-truth-deg", 0, true);
    imu_pitch_truth_node = outputroot->getChild("pitch-truth-deg", 0, true);
    imu_yaw_truth_node = outputroot->getChild("yaw-truth-deg", 0, true);
}


// initialize airdata output property nodes 
static void bind_airdata_output( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );

    airdata_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    airdata_airspeed_node = outputroot->getChild("airspeed-kt", 0, true);
    airdata_pressure_node = outputroot->getChild("pressure-mbar", 0, true);

    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    fake_extern_volts_node = fgGetNode("/sensors/APM2/extern-volt", true);
    fake_extern_amps_node  = fgGetNode("/sensors/APM2/extern-amps", true);
    fake_extern_current_node  = fgGetNode("/sensors/APM2/extern-current-mah", true);

    // set some fake values (write them just once, so if there was an
    // unintended conflict, the actual sensor would overwrite these.)
    SGPropertyNode *tmp_node;
    // note we don't leak here because we are getting a pointer back
    // into the global property structure
    tmp_node = fgGetNode("/sensors/APM2/board-vcc", true);
    tmp_node->setDoubleValue( 5.0 );
    tmp_node = fgGetNode("/sensors/airdata/temp-degC", true);
    tmp_node->setDoubleValue( 15.0 );

    airdata_inited = true;
}


// function prototypes
bool fgfs_imu_init( string rootname, SGPropertyNode *config ) {
    bind_input( config );
    bind_imu_output( rootname );

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


// function prototypes
bool fgfs_airdata_init( string rootname ) {
    bind_airdata_output( rootname );

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


bool fgfs_imu_update() {
    const int fgfs_imu_size = 52;
    uint8_t packet_buf[fgfs_imu_size];

    bool fresh_data = false;

    int result;
    while ( (result = sock.recv(packet_buf, fgfs_imu_size, 0))
	    == fgfs_imu_size )
    {
	fresh_data = true;

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
	}

	uint8_t *buf = packet_buf;
	double time = *(double *)buf; buf += 8;
	float p = *(float *)buf; buf += 4;
	float q = *(float *)buf; buf += 4;
	float r = *(float *)buf; buf += 4;
	float ax = *(float *)buf; buf += 4;
	float ay = *(float *)buf; buf += 4;
	float az = *(float *)buf; buf += 4;
	float airspeed = *(float *)buf; buf += 4;
	float pressure = *(float *)buf; buf += 4;
	float roll_truth = *(float *)buf; buf += 4;
	float pitch_truth = *(float *)buf; buf += 4;
	float yaw_truth = *(float *)buf; buf += 4;

	double cur_time = get_Time();
	imu_timestamp_node->setDoubleValue( cur_time );
	imu_p_node->setDoubleValue( p );
	imu_q_node->setDoubleValue( q );
	imu_r_node->setDoubleValue( r );
	imu_ax_node->setDoubleValue( ax );
	imu_ay_node->setDoubleValue( ay );
	imu_az_node->setDoubleValue( az );
	imu_hx_node->setDoubleValue( 0.0 );
	imu_hy_node->setDoubleValue( 0.0 );
	imu_hz_node->setDoubleValue( 0.0 );
	imu_roll_truth_node->setDoubleValue( roll_truth );
	imu_pitch_truth_node->setDoubleValue( pitch_truth );
	imu_yaw_truth_node->setDoubleValue( yaw_truth );

	if ( airdata_inited ) {
	    airdata_timestamp_node->setDoubleValue( cur_time );
	    airdata_airspeed_node->setDoubleValue( airspeed );
	    const double inhg2mbar = 33.8638866667;
	    airdata_pressure_node->setDoubleValue( pressure * inhg2mbar );

	    // fake volt/amp values here for no better place to do it
	    static double last_time = cur_time;
	    static double mah = 0.0;
	    double thr = act_throttle_node->getDoubleValue();
	    fake_extern_volts_node->setDoubleValue(16.0 - thr);
	    fake_extern_amps_node->setDoubleValue(thr * 12.0);
	    double dt = cur_time - last_time;
	    mah += thr*12.0 * (1000.0/3600.0) * dt;
	    last_time = cur_time;
	    fake_extern_current_node->setDoubleValue( mah );
	}
    }

    return fresh_data;
}


bool fgfs_airdata_update() {
    bool fresh_data = false;

    static double last_time = 0.0;
    double cur_time = airdata_timestamp_node->getDoubleValue();

    if ( cur_time > last_time ) {
	fresh_data = true;
    }

    last_time = cur_time;

    return fresh_data;
}


void fgfs_imu_close() {
    sock.close();
}
