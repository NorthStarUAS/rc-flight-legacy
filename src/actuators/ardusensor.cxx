//
// FILE: ardusensor.hxx
// DESCRIPTION: interact with ardupilot based servo subsystem board
//

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf) et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()

#include "include/ugear_config.h"

#include "comms/logging.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "ardusensor.hxx"

#define START_OF_MSG0 147
#define START_OF_MSG1 224
#define SENSOR_PACKET_ID 10
#define COMMAND_PACKET_ID 11
#define MAX_PILOT_INPUTS 4
#define MAX_ACTUATORS 4
#define MAX_ANALOG_INPUTS 4

// fgfs_imu property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *act_device_node = NULL;
// static SGPropertyNode *act_baud_node = NULL;

// pilot input property nodes
static SGPropertyNode *pilot_timestamp_node = NULL;
static SGPropertyNode *pilot_aileron_node = NULL;
static SGPropertyNode *pilot_elevator_node = NULL;
static SGPropertyNode *pilot_throttle_node = NULL;
static SGPropertyNode *pilot_rudder_node = NULL;
static SGPropertyNode *pilot_manual_node = NULL;
static SGPropertyNode *pilot_channel6_node = NULL;
static SGPropertyNode *pilot_channel7_node = NULL;
static SGPropertyNode *pilot_channel8_node = NULL;
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

// air data nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_analog0_node = NULL;
static SGPropertyNode *airdata_analog1_node = NULL;
static SGPropertyNode *airdata_airspeed_node = NULL;
static SGPropertyNode *airdata_altitude_node = NULL;

static bool airdata_inited = false;
static bool pilot_controls_inited = false;
static bool fresh_pilot_data = false;

static int fd = -1;
static string device_name = "/dev/ttyS0";
// static int baud = 115200;

static double data_in_timestamp = 0.0;
static uint16_t analog[MAX_ANALOG_INPUTS];     // internal stash
static bool manual_mode = true;
static uint16_t pilot_input[MAX_PILOT_INPUTS]; // internal stash


// initialize fgfs_gps input property nodes
static void bind_input( SGPropertyNode *config ) {
    act_device_node = config->getChild("device");
    if ( act_device_node != NULL ) {
	device_name = act_device_node->getStringValue();
    }
    // act_baud_node = config->getChild("baud");
    // if ( act_baud_node != NULL ) {
    // 	baud = act_baud_node->getIntValue();
    // }
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
    act_status_node = fgGetNode("/actuators/actuator/status", true);
}


// initialize airdata output property nodes 
static void bind_airdata_output( string rootname ) {
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );

    airdata_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    airdata_analog0_node = outputroot->getChild("analog0", 0, true);
    airdata_analog1_node = outputroot->getChild("analog1", 0, true);
    airdata_airspeed_node = outputroot->getChild("airspeed-kt", 0, true);
    airdata_altitude_node = outputroot->getChild("altitude-m", 0, true);

    airdata_inited = true;
}


// initialize airdata output property nodes 
static void bind_pilot_controls( string rootname ) {
    // SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );

    pilot_timestamp_node = fgGetNode("/sensors/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", 0, true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", 1, true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", 2, true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", 3, true);
    pilot_manual_node = fgGetNode("/sensors/pilot/manual", 4, true);
    pilot_channel6_node = fgGetNode("/sensors/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/sensors/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/sensors/pilot/channel", 7, true);
    pilot_status_node = fgGetNode("/sensors/pilot/status", true);

    pilot_controls_inited = true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool ardusensor_open() {
    if ( display_on ) {
	printf("Ardu servo subsystem on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios oldTio;	// Old Serial Port Settings
    struct termios newTio; 	// New Serial Port Settings
    memset(&oldTio, 0, sizeof(oldTio));
    memset(&newTio, 0, sizeof(newTio));

    // Save Current Serial Port Settings
    tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    newTio.c_cflag     = B115200 | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    newTio.c_iflag     = IGNPAR;   // ignore parity bits
    newTio.c_oflag     = 0;
    newTio.c_lflag     = 0;
    newTio.c_cc[VTIME] = 0;
    newTio.c_cc[VMIN]  = 1;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &newTio );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    return true;
}


// function prototypes
bool ardusensor_init( SGPropertyNode *config ) {
    printf("ardusensor_init()\n");

    bind_input( config );
    bind_act_nodes();

    bool result = ardusensor_open();

    return result;
}


bool ardusensor_airdata_init( string rootname ) {
    bind_airdata_output( rootname );

    return true;
}


bool ardusensor_pilot_init( string rootname ) {
    bind_pilot_controls( rootname );

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


// convert a pwm pulse length to a normalize [-1 to 1] or [0 to 1] range
static float normalize_pulse( int pulse, bool symmetrical ) {
    float result = 0.0;

    if ( symmetrical ) {
	// i.e. aileron, rudder, elevator
	result = (pulse - 1500.0) / 400.0;
	if ( result < -1.0 ) { result = -1.0; }
	if ( result > 1.0 ) { result = 1.0; }
    } else {
	// i.e. throttle
	result = (pulse - 1100.0) / 800.0;
	if ( result < 0.0 ) { result = 0.0; }
	if ( result > 1.0 ) { result = 1.0; }
    }

    return result;
}

static bool ardusensor_parse( uint8_t pkt_id, uint8_t pkt_len,
			     uint8_t *payload )
{
    bool new_data = false;

    if ( pkt_id == SENSOR_PACKET_ID ) {
	if ( pkt_len == MAX_PILOT_INPUTS * 2 + 1 + MAX_ANALOG_INPUTS * 2 ) {
	    uint8_t lo, hi;

	    data_in_timestamp = get_Time();

	    for ( int i = 0; i < MAX_PILOT_INPUTS; i++ ) {
		lo = payload[0 + 2*i]; hi = payload[1 + 2*i];
		pilot_input[i] = hi*256 + lo;
	    }

	    manual_mode = !payload[8];

	    for ( int i = 0; i < MAX_ANALOG_INPUTS; i++ ) {
		lo = payload[9 + 2*i]; hi = payload[10 + 2*i];
		analog[i] = hi*256 + lo;
	    }

#if 0
	    if ( display_on ) {
		printf("%5.2f %5.2f %4.2f %5.2f %d\n",
		       pilot_aileron_node->getDoubleValue(),
		       pilot_elevator_node->getDoubleValue(),
		       pilot_throttle_node->getDoubleValue(),
		       pilot_rudder_node->getDoubleValue(),
		       pilot_manual_node->getIntValue());
	    }
#endif
		      
	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("ardusensor: packet size mismatch in pilot input\n");
	    }
	}
    }

    return new_data;
}


#if 0
static void ardusensor_read_tmp() {
    int len;
    uint8_t input[16];
    len = read( fd, input, 1 );
    while ( len > 0 ) {
	printf("%c", input[0]);
	len = read( fd, input, 1 );
    }
}
#endif


static bool ardusensor_read() {
    static int state = 0;
    static int pkt_id = 0;
    static int pkt_len = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // if ( display_on ) {
    //   printf("read ardusensor, entry state = %d\n", state);
    // }

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != START_OF_MSG0 ) {
	    // fprintf( stderr, "state0: len = %d val = %2X\n", len, input[0] );
	    len = read( fd, input, 1 );
	}
	if ( len > 0 && input[0] == START_OF_MSG0 ) {
	    // fprintf( stderr, "read START_OF_MSG0\n");
	    state++;
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    if ( input[0] == START_OF_MSG1 ) {
		// fprintf( stderr, "read START_OF_MSG1\n");
		state++;
	    } else if ( input[0] == START_OF_MSG0 ) {
		// fprintf( stderr, "read START_OF_MSG0\n");
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    pkt_id = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    // fprintf( stderr, "pkt_id = %d\n", pkt_id );
	    state++;
	}
    }
    if ( state == 3 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    pkt_len = input[0];
	    if ( pkt_len < 256 ) {
		// fprintf( stderr, "pkt_len = %d\n", pkt_len );
		cksum_A += input[0];
		cksum_B += cksum_A;
		state++;
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 4 ) {
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    payload[counter++] = input[0];
	    // fprintf( stderr, "%02X ", input[0] );
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    if ( counter >= pkt_len ) {
		break;
	    }
	    len = read( fd, input, 1 );
	}

	if ( counter >= pkt_len ) {
	    state++;
	    // fprintf( stderr, "\n" );
	}
    }
    if ( state == 5 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_lo = input[0];
	    state++;
	}
    }
    if ( state == 6 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_hi = input[0];
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
		// fprintf( stderr, "checksum passes (%d)!\n", pkt_id );
		new_data = ardusensor_parse( pkt_id, pkt_len, payload );
		if ( new_data ) {
		    fresh_pilot_data = true;
		}
	    } else {
		if ( display_on ) {
		    // printf("checksum failed %d %d (computed) != %d %d (message)\n",
		    //	   cksum_A, cksum_B, cksum_lo, cksum_hi );
		}
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    return new_data;
}


// generate a pwm pulse length from a normalized [-1 to 1] or [0 to 1] range
static int gen_pulse( double val, bool symmetrical ) {
    int pulse = 0;

    if ( symmetrical ) {
	// i.e. aileron, rudder, elevator
	if ( val < -1.0 ) { val = -1.0; }
	if ( val > 1.0 ) { val = 1.0; }
	pulse = 1500 + (int)(400 * val);
    } else {
	// i.e. throttle
	if ( val < 0.0 ) { val = 0.0; }
	if ( val > 1.0 ) { val = 1.0; }
	pulse = 1100 + (int)(800 * val);
    }

    return pulse;
}


static void ardu_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += hdr1;
    c1 += c0;

    c0 += hdr2;
    c1 += c0;

    for ( uint8_t i = 0; i < size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}


static bool ardusensor_write() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    int len;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    len = write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = COMMAND_PACKET_ID; buf[1] = 0;
    // packet length (1 byte)
    buf[1] = 2 * MAX_ACTUATORS;
    len = write( fd, buf, 2 );

#if 0  
    // generate some test data
    static double t = 0.0;
    t += 0.02;
    double dummy = sin(t);
    act_aileron_node->setFloatValue(dummy);
    act_elevator_node->setFloatValue(dummy);
    act_throttle_node->setFloatValue((dummy/2)+0.5);
    act_rudder_node->setFloatValue(dummy);
#endif

    // servo data
    if ( MAX_ACTUATORS == 4 ) {
	int val;
	uint8_t hi, lo;

	val = gen_pulse( act_aileron_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_elevator_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_throttle_node->getFloatValue(), false );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_rudder_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;
    }
  
    // write packet
    len = write( fd, buf, size );
  
    // check sum (2 bytes)
    ardu_cksum( COMMAND_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    len = write( fd, buf, 2 );

    return true;
}


bool ardusensor_update() {
    // read receiver values from ardu servo subsystem
    while ( ardusensor_read() );

    // send actuator commands to ardu servo subsystem
    ardusensor_write();

    return true;
}


bool ardusensor_airdata_update() {
    bool fresh_data = false;
    static double last_time = 0.0;

    if ( airdata_inited ) {
	double cur_time = pilot_timestamp_node->getDoubleValue();

	// basic formula: v = sqrt((2/p) * q) where v = velocity, q =
	// dynamic pressure (pitot tube sensor value), and p = air
	// density.

	// if p is specified in kg/m^3 (value = 1.225) and if q is
	// specified in Pa (N/m^2) where 1 psi == 6900 Pa, then the
	// velocity will be in meters per second.

	// The MPXV5004DP has a full scale span of 3.9V, Maximum
	// pressure reading is 0.57psi (4000Pa)

	// With a 10bit ADC (ardupilot) we record a value of 226
	// (0-1024) at zero velocity.  Assuming a 5V input source,
	// this corresponds to about 1.1V ... via another reference we
	// predict the device saturates at about 3.9V.  In other words 1.1V
	// (226ADC) == 0Pa and 3.9V(799ADC) == 4000Pa.  Thus:

	// Pa = (ADC - 226) * 147
	// Airspeed(mps) = sqrt( (2/1.225) * Pa )

	// This yields a theoretical maximum speed sensor reading of
	// about 81mps (156 kts)

	// hard coded (probably should use constants from the config file)
	float Pa = (analog[0] - 226) * 7;
	if ( Pa < 0.0 ) { Pa = 0.0; } // avoid sqrt(neg_number) situation
	float airspeed_mps = sqrt( 2*Pa / 1.225 );
	float airspeed_kts = airspeed_mps * SG_MPS_TO_KT;

	if ( cur_time > last_time ) {
	    airdata_timestamp_node->setDoubleValue( cur_time );
	    airdata_analog0_node->setIntValue( analog[0] );
	    airdata_analog1_node->setIntValue( analog[1] );
	    airdata_airspeed_node->setDoubleValue( airspeed_kts );
	    airdata_altitude_node->setDoubleValue( analog[1] );

	    fresh_data = true;
	}

	last_time = cur_time;
    }

    return fresh_data;
}


bool ardusensor_pilot_update() {
    // basically a no-op other than managing the fresh_data flag correctly
    bool fresh_data = fresh_pilot_data;

    if ( fresh_data && pilot_controls_inited ) {
	float val;

	pilot_timestamp_node->setDoubleValue( data_in_timestamp );

	val = normalize_pulse( pilot_input[0], true );
	pilot_aileron_node->setDoubleValue( val );

	val = normalize_pulse( pilot_input[1], true );
	pilot_elevator_node->setDoubleValue( val );

	val = normalize_pulse( pilot_input[2], false );
	pilot_throttle_node->setDoubleValue( val );

	val = normalize_pulse( pilot_input[3], true );
	pilot_rudder_node->setDoubleValue( val );

	pilot_manual_node->setIntValue( manual_mode );
    }

    fresh_pilot_data = false;
    return fresh_data;
}


void ardusensor_close() {
    close(fd);
}
