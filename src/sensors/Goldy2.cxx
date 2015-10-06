//
// FILE: goldy2_imu.cxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#include <stdio.h>
#include <string>
#include <string.h>

#include "comms/netSocket.h"
#include "props/props.hxx"
#include "util/timing.h"

#include "Goldy2.hxx"


static netSocket sock;
static int port = 0;

// goldy2_imu property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *outputroot = NULL;
static SGPropertyNode *goldy2_port_node = NULL;

// imu property nodes
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

// airdata property nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_pressure_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_day_secs_node = NULL;
static SGPropertyNode *gps_date_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;
static SGPropertyNode *gps_satellites_node = NULL;
static SGPropertyNode *gps_pdop_node = NULL;
static SGPropertyNode *gps_status_node = NULL;

// pilot input property nodes
static SGPropertyNode *pilot_timestamp_node = NULL;
static SGPropertyNode *pilot_aileron_node = NULL;
static SGPropertyNode *pilot_elevator_node = NULL;
static SGPropertyNode *pilot_throttle_node = NULL;
static SGPropertyNode *pilot_rudder_node = NULL;
static SGPropertyNode *pilot_channel5_node = NULL;
static SGPropertyNode *pilot_channel6_node = NULL;
static SGPropertyNode *pilot_channel7_node = NULL;
static SGPropertyNode *pilot_channel8_node = NULL;
static SGPropertyNode *pilot_manual_node = NULL;
static SGPropertyNode *pilot_status_node = NULL;

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
static SGPropertyNode *act_status_node = NULL;

static SGPropertyNode *imu_roll_truth_node = NULL;
static SGPropertyNode *imu_pitch_truth_node = NULL;
static SGPropertyNode *imu_yaw_truth_node = NULL;

static bool master_init = false;
static bool imu_inited = false;
static bool airdata_inited = false;
static bool gps_inited = false;
static bool pilot_input_inited = false;
static bool actuator_inited = false;


// initialize goldy2_imu input property nodes
static void bind_input( SGPropertyNode *config ) {
    goldy2_port_node = fgGetNode("/config/sensors/Goldy2/port");
    if ( goldy2_port_node != NULL ) {
	port = goldy2_port_node->getIntValue();
    }
    printf("Goldy2 port = %d\n", port);

    configroot = config;
}


// initialize imu output property nodes 
static void bind_imu_output( string rootname ) {
    if ( imu_inited ) {
        return;
    }

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

    imu_inited = true;
}


// initialize airdata output property nodes 
static void bind_airdata_output( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );

    airdata_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    airdata_pressure_node = outputroot->getChild("pressure-mbar", 0, true);

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


// initialize gps output property nodes
static void bind_gps_output( string rootname ) {
    if ( gps_inited ) {
        return;
    }

    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_day_secs_node = outputroot->getChild("day-seconds", 0, true);
    gps_date_node = outputroot->getChild("date", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_satellites_node = outputroot->getChild("satellites", 0, true);
    gps_status_node = outputroot->getChild("status", 0, true);
    gps_pdop_node = outputroot->getChild("pdop", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);

    gps_inited = true;
}

// initialize pilot output property nodes
static void bind_pilot_controls( string rootname ) {
    if ( pilot_input_inited ) {
        return;
    }

    pilot_timestamp_node = fgGetNode("/sensors/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = fgGetNode("/sensors/pilot/channel", 4, true);
    pilot_channel6_node = fgGetNode("/sensors/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/sensors/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/sensors/pilot/channel", 7, true);
    pilot_manual_node = fgGetNode("/sensors/pilot/manual", true);
    pilot_status_node = fgGetNode("/sensors/pilot/status", true);

    pilot_input_inited = true;
}


// initialize actuator property nodes
static void bind_act_nodes() {
    if ( actuator_inited ) {
        return;
    }

    act_timestamp_node = fgGetNode("/actuators/actuator/time-stamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);
    act_status_node = fgGetNode("/actuators/actuator/status", true);

    actuator_inited = true;
}


bool goldy2_init( SGPropertyNode *config ) {
    if ( master_init ) {
        return true;
    }

    bind_input( config );

    // open a UDP socket
    if ( ! sock.open( false ) ) {
	printf("Error opening goldy2 input socket\n");
	return false;
    }

    // bind ...
    if ( sock.bind( "", port ) == -1 ) {
	printf("error binding to port %d\n", port );
	return false;
    }

    // don't block waiting for input
    sock.setBlocking( false );

    master_init = true;

    return true;
}


// function prototypes
bool goldy2_imu_init( string rootname, SGPropertyNode *config ) {
    if ( ! goldy2_init(config) ) {
        return false;
    }

    bind_imu_output( rootname );

    return true;
}


// function prototypes
bool goldy2_airdata_init( string rootname, SGPropertyNode *config ) {
    if ( ! goldy2_init(config) ) {
        return false;
    }

    bind_airdata_output( rootname );

    return true;
}

bool goldy2_gps_init( string rootname, SGPropertyNode *config  ) {
    if ( ! goldy2_init(config) ) {
        return false;
    }

    bind_gps_output( rootname );

    return true;
}

bool goldy2_pilot_init( string rootname, SGPropertyNode *config ) {
    if ( ! goldy2_init(config) ) {
        return false;
    }

    bind_pilot_controls( rootname );

    return true;
}

bool goldy2_act_init( SGPropertyNode *config ) {
    bind_act_nodes();

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


uint16_t utilCRC16( const void* data_p, uint16_t data_len, uint16_t crc_start )
{
    uint8_t* data_u8_p;
    uint16_t data_idx;
    uint16_t crc;
        
    // Typecast input for processing.  Note: typecase to 8-bit type does not
    // yield data alignment issues.
    data_u8_p = (uint8_t*) data_p;
        
    // Start CRC calculation value as that supplied.
    crc = crc_start;
        
    for( data_idx = 0; data_idx < data_len; data_idx++ ) {
	crc = (uint8_t)(crc >> 8) | (crc << 8);
	crc ^= data_u8_p[ data_idx ];
	crc ^= (uint8_t)(crc & 0xff) >> 4;
	crc ^= crc << 12;
	crc ^= (crc & 0x00ff) << 5;
    } 
        
    return crc;
}


// parse packets
bool goldy2_parse( uint8_t *buf, int size ) {
    if ( size < 8 ) {
        printf("goldy packet corruption!\n");
	return false;
    }
    printf("  header = %c %c %c\n", buf[0], buf[1], buf[2]);
    printf("  type id = 0x%02x\n", buf[3]);
    int len = buf[4] + 256*buf[5];
    printf("  package len = %d\n", len);
    printf("  CRC = %d\n", buf[6+len] + 256*buf[7+len]);
    printf("  Computed CRC = %d\n", utilCRC16(buf+3, len+3, 0));
}


// main input parsing function
bool goldy2_update() {
    const int goldy2_max_size = 2048;
    uint8_t packet_buf[goldy2_max_size];

    printf("checking for packet ...\n");

    int result;
    while ( (result = sock.recv(packet_buf, goldy2_max_size, 0)) >= 0 ) {
        printf("Read a packet, len = %d\n", result);
	goldy2_parse(packet_buf, result);
    }

    return true;
}


bool goldy2_imu_update() {
    goldy2_update();

    if ( imu_inited ) {
        // do some stuff
    }

    bool fresh_data = false;

#if 0
    // this should all move/change below...


    int result;
    while ( (result = sock.recv(packet_buf, goldy2_imu_size, 0))
	    == goldy2_imu_size )
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
	    const double inhg2mbar = 33.8638866667;
	    airdata_pressure_node->setDoubleValue( pressure * inhg2mbar );
	}
    }
#endif

    return fresh_data;
}


bool goldy2_airdata_update() {
    bool fresh_data = false;

    static double last_time = 0.0;
    double cur_time = airdata_timestamp_node->getDoubleValue();

    if ( cur_time > last_time ) {
	fresh_data = true;
    }

    last_time = cur_time;

    return fresh_data;
}


bool goldy2_gps_update() {
    return false;
}

bool goldy2_pilot_update() {
    return false;
}

bool goldy2_act_update() {
    return false;
}

void goldy2_imu_close() {
    sock.close();
}

void goldy2_airdata_close() {
}

void goldy2_gps_close() {
}

void goldy2_pilot_close() {
}

void goldy2_act_close() {
}
