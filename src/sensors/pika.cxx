/**
*  \file: airdata_uart.cxx
*
* Driver for the Bolder Flight Systems airdata module (build on AMSYS pressure
* sensors.)
*
* Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
*
*/

#include "python/pyprops.hxx"

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <math.h>		// fabs()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()
#include <sys/ioctl.h>

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "include/globaldefs.h"

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "sensors/cal_temp.hxx"
#include "util/strutils.hxx"
#include "util/timing.h"

#include "pika.hxx"
#include "pika_sensorData.h"

// payload struct and size
static struct sensors payload;
const int payloadSize = sizeof(payload);

// property nodes
static pyPropertyNode airdata_node;
static pyPropertyNode act_node;
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode pilot_node;

static int fd = -1;
static string device_name = "/dev/ttyO4";

static bool master_opened = false;
static bool imu_inited = false;
static bool gps_inited = false;
static bool airdata_inited = false;
static bool pilot_input_inited = false;

static bool reverse_imu_mount = false;

static bool airspeed_inited = false;
static double airspeed_zero_start_time = 0.0;

//static AuraCalTemp p_cal;
//static AuraCalTemp q_cal;
//static AuraCalTemp r_cal;
static AuraCalTemp ax_cal;
static AuraCalTemp ay_cal;
static AuraCalTemp az_cal;
static Matrix4d mag_cal;

#define START_OF_MSG0 0x42
#define START_OF_MSG1 0x46

#define PIKA_NUM_ACTUATORS 14	// must match the pika firmware

#define FLIGHT_COMMAND_PACKET_ID 23

#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)


// initialize imu output property nodes 
static void bind_airdata_output( string output_path ) {
    if ( airdata_inited ) {
	return;
    }
    airdata_node = pyGetNode(output_path, true);
    airdata_node.setString("module", "pika");
    airdata_inited = true;
}


// initialize actuator property nodes 
static void bind_act_nodes() {
    act_node = pyGetNode("/actuators", true);
}


// open the uart
static bool pika_open() {
    if ( master_opened ) {
	return true;
    }

    if ( display_on ) {
	printf("pika on %s @ 921,600\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
	fprintf( stderr, "open serial: unable to open %s - %s\n",
		 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// New Serial Port Settings

    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    config.c_cflag     = B921600 | // bps rate
	CS8	 | // 8n1
	CLOCAL	 | // local connection, no modem
	CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
    // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
	fprintf( stderr, "error configuring device: %s - %s\n",
		 device_name.c_str(), strerror(errno) );
	return false;
    }

    // Enable non-blocking IO (one more time for good measure)
    fcntl(fd, F_SETFL, O_NONBLOCK);

    master_opened = true;

    return true;
}


void pika_close() {
    close(fd);

    master_opened = false;
}


// initialize imu output property nodes 
static void bind_imu_output( string output_node ) {
    if ( imu_inited ) {
	return;
    }
    imu_node = pyGetNode(output_node, true);
    imu_inited = true;
}

// initialize gps output property nodes 
static void bind_gps_output( string output_path ) {
    if ( gps_inited ) {
	return;
    }
    gps_node = pyGetNode(output_path, true);
    gps_inited = true;
}

// initialize pilot output property nodes 
static void bind_pilot_controls( string output_path ) {
    if ( pilot_input_inited ) {
	return;
    }
    pilot_node = pyGetNode(output_path, true);
    pilot_node.setLen("channel", PIKA_NUM_ACTUATORS, 0.0);
    pilot_input_inited = true;
}


bool pika_imu_init( string output_path, pyPropertyNode *config ) {
    if ( ! pika_open() ) {
	return false;
    }

    bind_imu_output( output_path );

    if ( config->hasChild("reverse_imu_mount") ) {
	reverse_imu_mount = config->getBool("reverse_imu_mount");
    }

    if ( config->hasChild("calibration") ) {
	pyPropertyNode cal = config->getChild("calibration");
	double min_temp = 27.0;
	double max_temp = 27.0;
	if ( cal.hasChild("min_temp_C") ) {
	    min_temp = cal.getDouble("min_temp_C");
	}
	if ( cal.hasChild("max_temp_C") ) {
	    max_temp = cal.getDouble("max_temp_C");
	}

	//p_cal.init( cal->getChild("p"), min_temp, max_temp );
	//q_cal.init( cal->getChild("q"), min_temp, max_temp );
	//r_cal.init( cal->getChild("r"), min_temp, max_temp );
	pyPropertyNode ax_node = cal.getChild("ax");
	ax_cal.init( &ax_node, min_temp, max_temp );
	pyPropertyNode ay_node = cal.getChild("ay");
	ay_cal.init( &ay_node, min_temp, max_temp );
	pyPropertyNode az_node = cal.getChild("az");
	az_cal.init( &az_node, min_temp, max_temp );

	if ( cal.hasChild("mag_affine") ) {
	    string tokens_str = cal.getString("mag_affine");
	    vector<string> tokens = split(tokens_str);
	    if ( tokens.size() == 16 ) {
		int r = 0, c = 0;
		for ( unsigned int i = 0; i < 16; i++ ) {
		    mag_cal(r,c) = atof(tokens[i].c_str());
		    c++;
		    if ( c > 3 ) {
			c = 0;
			r++;
		    }
		}
	    } else {
		printf("ERROR: wrong number of elements for mag_cal affine matrix!\n");
		mag_cal.setIdentity();
	    }
	} else {
	    mag_cal.setIdentity();
	}

	// save the imu calibration parameters with the data file so that
	// later the original raw sensor values can be derived.
	if ( log_to_file ) {
	    log_imu_calibration( &cal );
	}
    }

    return true;
}


bool pika_gps_init( string output_path, pyPropertyNode *config ) {
    if ( ! pika_open() ) {
	return false;
    }

    bind_gps_output( output_path );

    return true;
}

bool pika_pilot_init( string output_path, pyPropertyNode *config ) {
    if ( ! pika_open() ) {
	return false;
    }

    bind_pilot_controls( output_path );

    return true;
}


// this keeps the imu_mgr happy, but the real work to update the
// property tree is performed right away when we receive and parse the
// packet.
bool pika_imu_update() {
    imu_node.setDouble( "timestamp", get_Time() );
    double temp_C = payload.imu_temp_c;
	
    imu_node.setDouble( "p_rad_sec", payload.imu_gyro_rads[0] );
    imu_node.setDouble( "q_rad_sec", payload.imu_gyro_rads[1] );
    imu_node.setDouble( "r_rad_sec", payload.imu_gyro_rads[2] );
    imu_node.setDouble( "ax_mps_sec", ax_cal.calibrate(payload.imu_accel_mss[0], temp_C) );
    imu_node.setDouble( "ay_mps_sec", ay_cal.calibrate(payload.imu_accel_mss[1], temp_C) );
    imu_node.setDouble( "az_mps_sec", az_cal.calibrate(payload.imu_accel_mss[2], temp_C) );

    imu_node.setDouble( "hx_raw", payload.imu_mag_uTesla[0] );
    imu_node.setDouble( "hy_raw", payload.imu_mag_uTesla[1] );
    imu_node.setDouble( "hz_raw", payload.imu_mag_uTesla[2] );
	
    Vector4d hs((double)payload.imu_mag_uTesla[0], (double)payload.imu_mag_uTesla[1], (double)payload.imu_mag_uTesla[2], 1.0);
    Vector4d hc = mag_cal * hs;
    imu_node.setDouble( "hx", hc(0) );
    imu_node.setDouble( "hy", hc(1) );
    imu_node.setDouble( "hz", hc(2) );

    imu_node.setDouble( "temp_C", temp_C );

    return true;
}

void pika_imu_close() {
    pika_close();
}

void pika_gps_close() {
    // noop
}

void pika_pilot_close() {
    // noop
}

void pika_airdata_init( string output_path, pyPropertyNode *config ) {
    bind_airdata_output( output_path );
}


static bool pika_read() {
    static int state = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[512];
    static uint8_t *payload_ptr;

    //if ( display_on ) {
    //    printf("read pika, entry state = %d\n", state);
    //}

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	payload_ptr = (uint8_t *)(&payload);
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	//printf( "state0: len = %d val = %2X (%c)\n", len, input[0] , input[0]);
	
	while ( len > 0 && input[0] != START_OF_MSG0 ) {
	    //printf( "state0: len = %d val = %2X (%c)\n", len, input[0] , input[0]);
	    len = read( fd, input, 1 );
	}
	if ( len > 0 && input[0] == START_OF_MSG0 ) {
	    //printf( "read START_OF_MSG0\n");
	    state++;
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    if ( input[0] == START_OF_MSG1 ) {
		//printf( "read START_OF_MSG1\n");
		state++;
	    } else if ( input[0] == START_OF_MSG0 ) {
		//fprintf( stderr, "read START_OF_MSG0\n");
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    payload_ptr[counter++] = input[0];
	    //printf( "%02X ", input[0] );
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    if ( counter >= payloadSize ) {
		break;
	    }
	    len = read( fd, input, 1 );
	}

	if ( counter >= payloadSize ) {
	    state++;
	    //printf( "\n" );
	}
    }
    if ( state == 3 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_lo = input[0];
	    state++;
	}
    }
    if ( state == 4 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_hi = input[0];
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
		//printf( "checksum passes\n" );
		new_data = true;
	    } else {
		printf("checksum failed %d %d (computed) != %d %d (message)\n",
		       cksum_A, cksum_B, cksum_lo, cksum_hi );
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    if ( new_data ) {
	return true;
    } else {
	return false;
    }
}


// Read pika packets.
// Returns the dt from the IMU perspective, not the localhost
// perspective.  This should generally be far more accurate and
// consistent.
double pika_update() {
    // read packets until the uart buffer is mostly empty. 
    double last_time = imu_node.getDouble( "timestamp" );
    int bytes_available = 0;
    while ( true ) {
        bool result = pika_read();
        ioctl(fd, FIONREAD, &bytes_available);
	if ( bytes_available < 64 ) {
	    break;
        }
    }
    double cur_time = imu_node.getDouble( "timestamp" );

    return cur_time - last_time;
}


bool pika_gps_update() {
    static int gps_fix_value = 0;
    static double last_timestamp = 0.0;
    double current_timestamp = 0.0;
    if ( gps_node.hasChild("timestamp") ) {
	current_timestamp = gps_node.getDouble("timestamp");
    }

    gps_fix_value = payload.gps_fixType;
    if ( gps_fix_value == 0 ) {
	gps_node.setLong( "status", 0 );
    } else if ( gps_fix_value == 1 || gps_fix_value == 2 ) {
	gps_node.setLong( "status", 1 );
    } else if ( gps_fix_value == 3 ) {
	gps_node.setLong( "status", 2 );
    }
    // printf("fix: %d lon: %.8f lat: %.8f\n", payload.gps_fixType, payload.gps_lon_rad, payload.gps_lat_rad);

    gps_node.setDouble( "timestamp", get_Time() );

    struct tm gps_time;
    gps_time.tm_sec = payload.gps_utcSec;
    gps_time.tm_min = payload.gps_utcMin;
    gps_time.tm_hour = payload.gps_utcHour;
    gps_time.tm_mday = payload.gps_utcDay;
    gps_time.tm_mon = payload.gps_utcMonth - 1;
    gps_time.tm_year = payload.gps_utcYear - 1900;
    double unix_sec = (double)mktime( &gps_time );
    unix_sec += payload.gps_utcNano / 1000000000.0;
    gps_node.setDouble( "unix_time_sec", unix_sec );
    gps_node.setDouble( "time_accuracy_ns", payload.gps_tAcc );
	    
    gps_node.setLong( "satellites", payload.gps_numSV );
	    
    gps_node.setDouble( "latitude_deg", payload.gps_lat_rad * SGD_RADIANS_TO_DEGREES );
    gps_node.setDouble( "longitude_deg", payload.gps_lon_rad * SGD_RADIANS_TO_DEGREES );
    gps_node.setDouble( "altitude_m", payload.gps_hMSL_m );
    gps_node.setDouble( "vn_ms", payload.gps_velN_mps );
    gps_node.setDouble( "ve_ms", payload.gps_velE_mps );
    gps_node.setDouble( "vd_ms", payload.gps_velD_mps );
    gps_node.setDouble( "horiz_accuracy_m", payload.gps_hAcc_m );
    gps_node.setDouble( "vert_accuracy_m", payload.gps_vAcc_m );
    gps_node.setDouble( "pdop", payload.gps_pDOP );
    gps_node.setLong( "fixType", payload.gps_fixType);

    if ( current_timestamp > last_timestamp ) {
        last_timestamp = current_timestamp;
        return true;
    } else {
        return false;
    }
}


bool pika_pilot_update() {
    if ( !pilot_input_inited ) {
	return false;
    }

    pilot_node.setDouble( "timestamp", imu_node.getDouble("timestamp") );

    for ( int i = 0; i < PIKA_NUM_ACTUATORS; i++ ) {
	pilot_node.setDouble( "channel", i, payload.sbus_channels[i] );
    }

    return true;
}



bool pika_airdata_update() {
    static double diff_sum = 0.0;
    static int diff_count = 0;
    static float diff_offset = 0.0;

    airdata_node.setDouble( "timestamp", get_Time() );
    airdata_node.setDouble( "pressure_mbar", (payload.airdata_staticPress_pa / 100.0) );
    airdata_node.setDouble( "diff_pa", payload.airdata_diffPress_pa);

    if ( ! airspeed_inited ) {
	if ( airspeed_zero_start_time > 0 ) {
	    diff_sum += payload.airdata_diffPress_pa;
	    diff_count++;
	    diff_offset = diff_sum / diff_count;
	} else {
	    airspeed_zero_start_time = get_Time();
	    diff_sum = 0.0;
	    diff_count = 0;
	}
	if ( get_Time() > airspeed_zero_start_time + 10.0 ) {
	    //printf("diff_offset = %.2f\n", diff_offset);
	    airspeed_inited = true;
	}
    }

    double pitot_calibrate = 1.0; // make configurable in the future?
    double diff_pa = payload.airdata_diffPress_pa - diff_offset;
    if ( diff_pa < 0.0 ) { diff_pa = 0.0; } // avoid sqrt(neg_number) situation
    float airspeed_mps = sqrt( 2*diff_pa / 1.225 ) * pitot_calibrate;
    float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
    airdata_node.setDouble( "airspeed_mps", airspeed_mps );
    airdata_node.setDouble( "airspeed_kt", airspeed_kt );

    return true;
}


void pika_airdata_close() {
    pika_close();
}


// generate a pwm pulse length from a normalized [-1 to 1] or [0 to 1] range
static int gen_pulse( double val, bool symmetrical ) {
    int pulse = 0;

    if ( symmetrical ) {
	// i.e. aileron, rudder, elevator
	if ( val < -1.5 ) { val = -1.5; }
	if ( val > 1.5 ) { val = 1.5; }
	pulse = PWM_CENTER + (int)(PWM_HALF_RANGE * val);
    } else {
	// i.e. throttle, flaps
	if ( val < 0.0 ) { val = 0.0; }
	if ( val > 1.0 ) { val = 1.0; }
	pulse = PWM_MIN + (int)(PWM_RANGE * val);
    }

    return pulse;
}


static void pika_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
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


#if 0
static bool pika_act_write() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 2 * RAVEN_NUM_ACTS;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = FLIGHT_COMMAND_PACKET_ID;
    // packet length (1 byte)
    buf[1] = size;
    /* len = */ write( fd, buf, 2 );

    uint8_t *packet = buf;
    int val;

    for ( int i = 0; i < RAVEN_NUM_ACTS; i++ ) {
	val = gen_pulse( act_node.getDouble(act_input[i].c_str())
			 * act_gain[i], true );
	*(uint16_t *)packet = val; packet += 2;
    }
    
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    pika_cksum( FLIGHT_COMMAND_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}
#endif


bool pika_act_init( pyPropertyNode *section ) {
    bind_act_nodes();

    int inputs = section->getLen("channel");
    printf("Found %d channel definitions\n", inputs);
    for ( int i = 0; i < inputs; i++ ) {
	pyPropertyNode channel = section->getChild("channel", i);
	if ( !channel.isNull() ) {
	    //printf(" channel: %d = %s (%.2f)\n", i, act_input[i].c_str(), act_gain[i]);
	}
    }

    if ( ! pika_open() ) {
	return false;
    }

    return true;
}


bool pika_act_update() {
    // send actuator commands to APM2 servo subsystem
    // pika_act_write();

    return true;
}

void pika_act_close() {
    // no-op
}
