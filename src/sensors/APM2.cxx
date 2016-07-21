//
// FILE: APM2.cxx
// DESCRIPTION: interact with APM2 with "sensor head" firmware
//

#include "python/pyprops.hxx"

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()
#include <math.h>		// M_PI

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

// #include <iostream>
// using std::cout;
// using std::endl;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "init/globals.hxx"
#include "sensors/cal_temp.hxx"
#include "util/poly1d.hxx"
#include "util/timing.h"

#include "APM2.hxx"

#define START_OF_MSG0 147
#define START_OF_MSG1 224

#define ACK_PACKET_ID 20

#define PWM_RATE_PACKET_ID 21
#define BAUD_PACKET_ID 22
#define FLIGHT_COMMAND_PACKET_ID 23
#define ACT_GAIN_PACKET_ID 24
#define MIX_MODE_PACKET_ID 25
#define SAS_MODE_PACKET_ID 26
#define SERIAL_NUMBER_PACKET_ID 27
#define WRITE_EEPROM_PACKET_ID 28

#define PILOT_PACKET_ID 50
#define IMU_PACKET_ID 51
#define GPS_PACKET_ID 52
#define BARO_PACKET_ID 53
#define ANALOG_PACKET_ID 54
#define CONFIG_INFO_PACKET_ID 55

#define NUM_PILOT_INPUTS 8
#define NUM_ACTUATORS 8
#define NUM_IMU_SENSORS 10
#define NUM_ANALOG_INPUTS 6

#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)

// Actuator gain (reversing) commands, format is cmd(byte) ch(byte) gain(float)
#define ACT_GAIN_DEFAULTS 0
#define ACT_GAIN_SET 1

// Mix mode commands (format is cmd(byte), gain 1 (float), gain 2 (float)
#define MIX_DEFAULTS 0
#define MIX_AUTOCOORDINATE 1
#define MIX_THROTTLE_TRIM 2
#define MIX_FLAP_TRIM 3
#define MIX_ELEVONS 4
#define MIX_FLAPERONS 5
#define MIX_VTAIL 6
#define MIX_DIFF_THRUST 7

// SAS mode commands (format is cmd(byte), gain)
#define SAS_DEFAULTS 0
#define SAS_ROLLAXIS 1
#define SAS_PITCHAXIS 2
#define SAS_YAWAXIS 3
#define SAS_CH7_TUNE 10

static pyPropertyNode apm2_node;
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode pilot_node;
static pyPropertyNode act_node;
static pyPropertyNode airdata_node;
static pyPropertyNode analog_node;

static bool master_opened = false;
static bool imu_inited = false;
static bool gps_inited = false;
static bool airdata_inited = false;
static bool pilot_input_inited = false;
static bool actuator_inited = false;

static int fd = -1;
static string device_name = "/dev/ttyS0";
static int baud = 230400;
static float volt_div_ratio = 100; // a nonsense value
static int battery_cells = 4;
static float extern_amp_offset = 0.0;
static float extern_amp_ratio = 0.1; // a nonsense value
static float extern_amp_sum = 0.0;
static float pitot_calibrate = 1.0;
static bool reverse_imu_mount = false;

static int last_ack_id = 0;
static int last_ack_subid = 0;

static uint16_t act_rates[NUM_ACTUATORS] = { 50, 50, 50, 50, 50, 50, 50, 50 };

static double pilot_in_timestamp = 0.0;
static uint16_t pilot_input[NUM_PILOT_INPUTS]; // internal stash
static string pilot_mapping[NUM_PILOT_INPUTS]; // channel->name mapping
static bool pilot_symmetric[NUM_PILOT_INPUTS]; // normalization symmetry flag
static bool pilot_invert[NUM_PILOT_INPUTS];    // invert input flag

static double imu_timestamp = 0.0;
static int16_t imu_sensors[NUM_IMU_SENSORS];

struct gps_sensors_t {
    double timestamp;
    uint32_t time;
    uint32_t date;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int16_t vel_north;
    int16_t vel_east;
    int16_t vel_down;
    int16_t pdop;
    uint8_t num_sats;
    uint8_t status;
} gps_sensors;

struct air_data_t {
    double timestamp;
    float pressure;
    float temp;
    float climb_rate;
    float airspeed;
} airdata;

static float analog[NUM_ANALOG_INPUTS];     // internal stash

static bool airspeed_inited = false;
static double airspeed_zero_start_time = 0.0;

//static AuraCalTemp p_cal;
//static AuraCalTemp q_cal;
//static AuraCalTemp r_cal;
static AuraCalTemp ax_cal;
static AuraCalTemp ay_cal;
static AuraCalTemp az_cal;
static Matrix<double,4,4> mag_cal;

static uint32_t pilot_packet_counter = 0;
static uint32_t imu_packet_counter = 0;
static uint32_t gps_packet_counter = 0;
static uint32_t baro_packet_counter = 0;
static uint32_t analog_packet_counter = 0;

// pulled from apm2-sensors.ino
static const float d2r = M_PI / 180.0;
static const float g = 9.81;

static const float _gyro_lsb_per_dps = 65536 / 1000; // +/- 500dps
static const float _accel_lsb_per_dps = 65536 / 8;   // +/- 4g

static const float MPU6000_gyro_scale = d2r / _gyro_lsb_per_dps;
static const float MPU6000_accel_scale = g / _accel_lsb_per_dps;
static const float MPU6000_temp_scale = 0.02;


static void APM2_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
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
bool APM2_request_baud( uint32_t baud ) {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 4;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = BAUD_PACKET_ID;
    // packet length (1 byte)
    buf[1] = size;
    /* len = */ write( fd, buf, 2 );

    // actuator data
    *(uint32_t *)buf = baud;
  
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( BAUD_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}
#endif

static bool APM2_act_write_eeprom() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = WRITE_EEPROM_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 0;
    /* len = */ write( fd, buf, 2 );

    // check sum (2 bytes)
    APM2_cksum( WRITE_EEPROM_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


static bool APM2_act_set_serial_number( uint16_t serial_number ) {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len */
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = SERIAL_NUMBER_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 2;
    /* len = */ write( fd, buf, 2 );

    // actuator data
    uint8_t hi = serial_number / 256;
    uint8_t lo = serial_number - (hi * 256);
    buf[size++] = lo;
    buf[size++] = hi;
  
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( SERIAL_NUMBER_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


static bool APM2_act_set_pwm_rates( uint16_t rates[NUM_ACTUATORS] ) {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = PWM_RATE_PACKET_ID;
    // packet length (1 byte)
    buf[1] = NUM_ACTUATORS * 2;
    /* len = */ write( fd, buf, 2 );

    // actuator data
    for ( int i = 0; i < NUM_ACTUATORS; i++ ) {
	uint16_t val = rates[i];
	uint8_t hi = val / 256;
	uint8_t lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;
    }
  
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( PWM_RATE_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


static bool APM2_act_gain_mode( int channel, float gain)
{
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = ACT_GAIN_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 3;
    /* len = */ write( fd, buf, 2 );

    buf[size++] = (uint8_t)channel;

    uint16_t val;
    uint8_t hi, lo;
    
    // gain
    val = 32767 + gain * 10000;
    hi = val / 256;
    lo = val - (hi * 256);
    buf[size++] = lo;
    buf[size++] = hi;
    
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( ACT_GAIN_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


static bool APM2_act_mix_mode( int mode_id, bool enable,
			       float gain1, float gain2)
{
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = MIX_MODE_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 6;
    /* len = */ write( fd, buf, 2 );

    buf[size++] = mode_id;
    buf[size++] = enable;

    uint16_t val;
    uint8_t hi, lo;
    
    // gain1
    val = 32767 + gain1 * 10000;
    hi = val / 256;
    lo = val - (hi * 256);
    buf[size++] = lo;
    buf[size++] = hi;
    
    // gain2
    val = 32767 + gain2 * 10000;
    hi = val / 256;
    lo = val - (hi * 256);
    buf[size++] = lo;
    buf[size++] = hi;
    
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( MIX_MODE_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


static bool APM2_act_sas_mode( int mode_id, bool enable, float gain)
{
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = SAS_MODE_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 4;
    /* len = */ write( fd, buf, 2 );

    buf[size++] = mode_id;
    buf[size++] = enable;

    uint16_t val;
    uint8_t hi, lo;
    
    // gain
    val = 32767 + gain * 10000;
    hi = val / 256;
    lo = val - (hi * 256);
    buf[size++] = lo;
    buf[size++] = hi;
    
    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( SAS_MODE_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
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


// initialize actuator property nodes 
static void bind_act_nodes( string output_path ) {
    if ( actuator_inited ) {
	return;
    }
    act_node = pyGetNode(output_path, true);
    act_node.setLen("channel", NUM_ACTUATORS, 0.0);
    actuator_inited = true;
}

// initialize airdata output property nodes 
static void bind_airdata_output( string output_path ) {
    if ( airdata_inited ) {
	return;
    }
    airdata_node = pyGetNode(output_path, true);
    airdata_inited = true;
}


// initialize pilot output property nodes 
static void bind_pilot_controls( string output_path ) {
    if ( pilot_input_inited ) {
	return;
    }
    pilot_node = pyGetNode(output_path, true);
    pilot_node.setLen("channel", NUM_ACTUATORS, 0.0);
    pilot_input_inited = true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool APM2_open_device( int baud_bits ) {
    if ( display_on ) {
	printf("APM2 Sensor Head on %s @ %d(code) baud\n", device_name.c_str(),
	       baud_bits);
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config;	// Old Serial Port Settings

    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    config.c_cflag     = baud_bits | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 1;	   // block 'read' from returning until at
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

    // bind main apm2 property nodes here for lack of a better place..
    apm2_node = pyGetNode("/sensors/APM2", true);
    analog_node = pyGetNode("/sensors/APM2/raw_analog", true);
    analog_node.setLen("channel", NUM_ANALOG_INPUTS, 0.0);
    
    return true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool APM2_open() {
    if ( master_opened ) {
	return true;
    }

    pyPropertyNode apm2_config = pyGetNode("/config/sensors/APM2", true);

    if ( apm2_config.hasChild("device") ) {
	device_name = apm2_config.getString("device");
    }
    if ( apm2_config.hasChild("baud") ) {
       baud = apm2_config.getLong("baud");
    }
    if ( apm2_config.hasChild("volt_divider_ratio") ) {
	volt_div_ratio = apm2_config.getDouble("volt_divider_ratio");
    }
    if ( apm2_config.hasChild("battery_cells") ) {
	battery_cells = apm2_config.getDouble("battery_cells");
    }
    if ( battery_cells < 1 ) { battery_cells = 1; }
    if ( apm2_config.hasChild("external_amp_offset") ) {
	extern_amp_offset = apm2_config.getDouble("external_amp_offset");
    }
    if ( apm2_config.hasChild("external_amp_ratio") ) {
	extern_amp_ratio = apm2_config.getDouble("external_amp_ratio");
    }

    if ( apm2_config.hasChild("pitot_calibrate_factor") ) {
	pitot_calibrate = apm2_config.getDouble("pitot_calibrate_factor");
    }

    int baud_bits = B115200;
    if ( baud == 115200 ) {
	baud_bits = B115200;
    } else if ( baud == 230400 ) {
	baud_bits = B230400;
    } else if ( baud == 500000 ) {
	baud_bits = B500000;
     } else {
	printf("unsupported baud rate = %d\n", baud);
    }

    if ( ! APM2_open_device( baud_bits ) ) {
	printf("device open failed ...\n");
	return false;
    }

    sleep(1);
    
    master_opened = true;

    return true;
}


#if 0
bool APM2_init( SGPropertyNode *config ) {
    printf("APM2_init()\n");

    bind_input( config );

    bool result = APM2_open();

    return result;
}
#endif


bool APM2_imu_init( string output_path, pyPropertyNode *config ) {
    if ( ! APM2_open() ) {
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


bool APM2_gps_init( string output_path, pyPropertyNode *config ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_gps_output( output_path );

    return true;
}


bool APM2_airdata_init( string output_path ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_airdata_output( output_path );

    return true;
}


bool APM2_pilot_init( string output_path, pyPropertyNode *config ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_pilot_controls( output_path );

    if ( config->hasChild("channel") ) {
	for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
	    pilot_mapping[i] = config->getString("channel", i);
	    printf("pilot input: channel %d maps to %s\n", i, pilot_mapping[i].c_str());
	}
    }
    if ( config->hasChild("symmetric") ) {
	for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
	    pilot_symmetric[i] = config->getBool("symmetric", i);
	    printf("pilot input: channel %d symmetry %d\n", i, pilot_symmetric[i]);
	}
    }
    if ( config->hasChild("invert") ) {
	for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
	    pilot_invert[i] = config->getBool("invert", i);
	    printf("pilot input: channel %d invert %d\n", i, pilot_invert[i]);
	}
    }

    return true;
}


bool APM2_act_init( string output_path, pyPropertyNode *section ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_act_nodes( output_path );

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
	result = (pulse - PWM_CENTER) / (float)PWM_HALF_RANGE;
	if ( result < -1.0 ) { result = -1.0; }
	if ( result > 1.0 ) { result = 1.0; }
    } else {
	// i.e. throttle
	result = (pulse - PWM_MIN) / (float)PWM_RANGE;
	if ( result < 0.0 ) { result = 0.0; }
	if ( result > 1.0 ) { result = 1.0; }
    }

    return result;
}

static bool APM2_parse( uint8_t pkt_id, uint8_t pkt_len,
			uint8_t *payload )
{
    bool new_data = false;
    static float extern_volt_filt = 0.0;
    static float extern_amp_filt = 0.0;

    if ( pkt_id == ACK_PACKET_ID ) {
	if ( display_on ) {
	    printf("Received ACK = %d %d\n", payload[0], payload[1]);
	}
	if ( pkt_len == 2 ) {
	    last_ack_id = payload[0];
	    last_ack_subid = payload[1];
	} else {
	    printf("APM2: packet size mismatch in ACK\n");
	}
    } else if ( pkt_id == PILOT_PACKET_ID ) {
	if ( pkt_len == NUM_PILOT_INPUTS * 2 ) {
	    uint8_t lo, hi;

	    pilot_in_timestamp = get_Time();

	    for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
		lo = payload[0 + 2*i]; hi = payload[1 + 2*i];
		pilot_input[i] = hi*256 + lo;
	    }

#if 0
	    if ( display_on ) {
		printf("%5.2f %5.2f %4.2f %5.2f %d\n",
		       pilot_aileron_node.getDouble(),
		       pilot_elevator_node.getDouble(),
		       pilot_throttle_node.getDouble(),
		       pilot_rudder_node.getDouble(),
		       pilot_manual_node->getLong());
	    }
#endif

	    pilot_packet_counter++;
	    apm2_node.setLong( "pilot_packet_count", pilot_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in pilot input\n");
	    }
	}
    } else if ( pkt_id == IMU_PACKET_ID ) {
	if ( pkt_len == NUM_IMU_SENSORS * 2 ) {
	    uint8_t lo, hi;

	    imu_timestamp = get_Time();

	    for ( int i = 0; i < NUM_IMU_SENSORS; i++ ) {
		lo = payload[0 + 2*i]; hi = payload[1 + 2*i];
		imu_sensors[i] = hi*256 + lo;
	    }

#if 0
	    if ( display_on ) {
		for ( int i = 0; i < NUM_IMU_SENSORS; i++ ) {
		    printf("%d ", imu_sensors[i]);
		}
		printf("\n");
	    }
#endif
		      
	    imu_packet_counter++;
	    apm2_node.setLong( "imu_packet_count", imu_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in imu input\n");
	    }
	}
    } else if ( pkt_id == GPS_PACKET_ID ) {
	if ( pkt_len == 30 ) {
	    gps_sensors.timestamp = get_Time();
	    gps_sensors.time = *(uint32_t *)payload; payload += 4;
	    gps_sensors.date = *(uint32_t *)payload; payload += 4;
	    gps_sensors.latitude = *(int32_t *)payload; payload += 4;
	    gps_sensors.longitude = *(int32_t *)payload; payload += 4;
	    gps_sensors.altitude = *(int32_t *)payload; payload += 4;
	    gps_sensors.vel_north = *(int16_t *)payload; payload += 2;
	    gps_sensors.vel_east = *(int16_t *)payload; payload += 2;
	    gps_sensors.vel_down = *(int16_t *)payload; payload += 2;
	    gps_sensors.pdop = *(int16_t *)payload; payload += 2;
	    gps_sensors.num_sats = *(uint8_t *)payload; payload += 1;
	    gps_sensors.status = *(uint8_t *)payload; payload += 1;

	    gps_packet_counter++;
	    apm2_node.setLong( "gps_packet_count", gps_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in gps input\n");
	    }
	}
    } else if ( pkt_id == BARO_PACKET_ID ) {
	if ( pkt_len == 12 ) {
	    airdata.timestamp = get_Time();
	    airdata.pressure = *(float *)payload; payload += 4;
	    airdata.temp = *(float *)payload; payload += 4;
	    airdata.climb_rate = *(float *)payload; payload += 4;

	    // if ( display_on ) {
	    // 	printf("baro %.3f %.1f %.1f %.1f\n", airdata.timestamp,
	    // 		airdata.pressure, airdata.temp, airdata.climb_rate);
	    // }
		      
	    baro_packet_counter++;
	    apm2_node.setLong( "baro_packet_count", baro_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in barometer input\n");
	    }
	}
    } else if ( pkt_id == ANALOG_PACKET_ID ) {
	if ( pkt_len == 2 * NUM_ANALOG_INPUTS ) {
	    uint8_t lo, hi;
	    for ( int i = 0; i < NUM_ANALOG_INPUTS; i++ ) {
		lo = payload[0 + 2*i]; hi = payload[1 + 2*i];
		float val = (float)(hi*256 + lo);
		if ( i != 5 ) {
		    // tranmitted value is left shifted 6 bits (*64)
		    analog[i] = val / 64.0;
		} else {
		    // special case APM2 specific sensor values, write to
		    // property tree here
		    analog[i] = val / 1000.0;
		}
		bool result = analog_node.setDouble( "channel", i, analog[i] );
		if ( ! result ) {
		    printf("channel write failed %d\n", i);
		}
	    }

	    // fill in property values that don't belong to some other
	    // sub system right now.
	    double analog_timestamp = get_Time();
	    static double last_analog_timestamp = analog_timestamp;
	    double dt = analog_timestamp - last_analog_timestamp;
	    last_analog_timestamp = analog_timestamp;

	    static float filter_vcc = analog[5];
	    filter_vcc = 0.9999 * filter_vcc + 0.0001 * analog[5];
	    apm2_node.setDouble( "board_vcc", filter_vcc );

	    float extern_volts = analog[1] * (filter_vcc/1024.0) * volt_div_ratio;
	    extern_volt_filt = 0.995 * extern_volt_filt + 0.005 * extern_volts;
	    float cell_volt = extern_volt_filt / (float)battery_cells;
	    float extern_amps = ((analog[2] * (filter_vcc/1024.0)) - extern_amp_offset) * extern_amp_ratio;
	    extern_amp_filt = 0.99 * extern_amp_filt + 0.01 * extern_amps;
	    /*printf("a[2]=%.1f vcc=%.2f ratio=%.2f amps=%.2f\n",
		analog[2], filter_vcc, extern_amp_ratio, extern_amps); */
	    extern_amp_sum += extern_amps * dt * 0.277777778; // 0.2777... is 1000/3600 (conversion to milli-amp hours)

	    apm2_node.setDouble( "extern_volt", extern_volt_filt );
	    apm2_node.setDouble( "extern_cell_volt", cell_volt );
	    apm2_node.setDouble( "extern_amps", extern_amp_filt );
	    apm2_node.setDouble( "extern_current_mah", extern_amp_sum );

#if 0
	    if ( display_on ) {
		for ( int i = 0; i < NUM_ANALOG_INPUTS; i++ ) {
		    printf("%.2f ", (float)analog[i] / 64.0);
		}
		printf("\n");
	    }
#endif
		      
	    analog_packet_counter++;
	    apm2_node.setLong( "analog_packet_count", analog_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in analog input\n");
	    }
	}
    } else if ( pkt_id == CONFIG_INFO_PACKET_ID ) {
	static bool first_time = true;
	if ( pkt_len == 12 ) {
	    uint16_t serial_num = *(uint16_t *)payload; payload += 2;
	    uint16_t firmware_rev = *(uint16_t *)payload; payload += 2;
	    uint16_t master_hz = *(uint16_t *)payload; payload += 2;
	    uint32_t baud_rate = *(uint32_t *)payload; payload += 4;
	    uint16_t byte_rate = *(uint16_t *)payload; payload += 2;

#if 0
	    if ( display_on ) {
		printf("info %d %d %d %d\n", serial_num, firmware_rev,
		       master_hz, baud_rate);
	    }
#endif
		      
	    apm2_node.setLong( "serial_number", serial_num );
	    apm2_node.setLong( "firmware_rev", firmware_rev );
	    apm2_node.setLong( "master_hz", master_hz );
	    apm2_node.setLong( "baud_rate", baud_rate );
	    apm2_node.setLong( "byte_rate_sec", byte_rate );

	    if ( first_time ) {
		// log the data to events.txt
		first_time = false;
		char buf[128];
		snprintf( buf, 32, "Serial Number = %d", serial_num );
		events->log("APM2", buf );
		snprintf( buf, 32, "Firmware Revision = %d", firmware_rev );
		events->log("APM2", buf );
		snprintf( buf, 32, "Master Hz = %d", master_hz );
		events->log("APM2", buf );
		snprintf( buf, 32, "Baud Rate = %d", baud_rate );
		events->log("APM2", buf );
	    }
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in config info\n");
	    }
	}
    } else {
	if ( display_on ) {
	    printf("APM2: unknown packet id = %d\n", pkt_id);
	}
    }

    return new_data;
}


#if 0
static void APM2_read_tmp() {
    int len;
    uint8_t input[16];
    len = read( fd, input, 1 );
    while ( len > 0 ) {
	printf("%c", input[0]);
	len = read( fd, input, 1 );
    }
}
#endif


static int APM2_read() {
    static int state = 0;
    static int pkt_id = 0;
    static int pkt_len = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // if ( display_on ) {
    //    printf("read APM2, entry state = %d\n", state);
    // }

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != START_OF_MSG0 ) {
	    //fprintf( stderr, "state0: len = %d val = %2X (%c)\n", len, input[0] , input[0]);
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
		// printf( "checksum passes (%d)!\n", pkt_id );
		new_data = APM2_parse( pkt_id, pkt_len, payload );
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

    if ( new_data ) {
	return pkt_id;
    } else {
	return 0;
    }
}


// send a full configuration to APM2 and return true only when all
// parameters are acknowledged.
static bool APM2_send_config() {
    if ( display_on ) {
	printf("APM2_send_config(): begin\n");
    }

    double start_time = 0.0;
    double timeout = 0.5;
    vector<string> children;

    pyPropertyNode apm2_config = pyGetNode("/config/sensors/APM2", true);
    if ( apm2_config.hasChild("setup_serial_number") ) {
	uint16_t serial_number = apm2_config.getLong("setup_serial_number");
	start_time = get_Time();    
	APM2_act_set_serial_number( serial_number );
	last_ack_id = 0;
	while ( (last_ack_id != SERIAL_NUMBER_PACKET_ID) ) {
	    APM2_read();
	    if ( get_Time() > start_time + timeout ) {
		if ( display_on ) {
		    printf("Timeout waiting for set serial_number ack...\n");
		}
		return false;
	    }
	}
    }

    int count;
    
    if ( display_on ) {
	printf("APM2_send_config(): pwm_rates\n");
    }

    // init all channels to default
    for ( int i = 0; i < NUM_ACTUATORS; i++ ) {
	act_rates[i] = 0; /* no change from default */
    }
    pyPropertyNode pwm_node
	= pyGetNode("/config/actuators/actuator/pwm_rates", true);
    count = pwm_node.getLen("channel");
    if ( count > 0 ) {
	for ( int i = 0; i < NUM_ACTUATORS; i++ ) {
	    act_rates[i] = 0; /* no change from default */
	}
	printf("pwm_rates: ");
	for ( int i = 0; i < count; i++ ) {
	    act_rates[i] = pwm_node.getLong("channel", i);
	    printf("%d ", act_rates[i]);
	}
	printf("\n");
	start_time = get_Time();    
	APM2_act_set_pwm_rates( act_rates );
	last_ack_id = 0;
	while ( (last_ack_id != PWM_RATE_PACKET_ID) ) {
	    APM2_read();
	    if ( get_Time() > start_time + timeout ) {
		if ( display_on ) {
		    printf("Timeout waiting for pwm_rate ack...\n");
		}
		return false;
	    }
	}
    }

    if ( display_on ) {
	printf("APM2_send_config(): gains\n");
    }

    pyPropertyNode gain_node
	= pyGetNode("/config/actuators/actuator/gains", true);
    count = gain_node.getLen("channel");
    if ( count ) {
	for ( int i = 0; i < count; i++ ) {
	    float gain = gain_node.getDouble("channel", i);
	    if ( display_on ) {
		printf("gain: %d %.2f\n", i, gain);
	    }
	    start_time = get_Time();    
	    APM2_act_gain_mode( i, gain );
	    last_ack_id = 0;
	    last_ack_subid = 0;
	    while ( (last_ack_id != ACT_GAIN_PACKET_ID)
		    || (last_ack_subid != i) )
	    {
		APM2_read();
		if ( get_Time() > start_time + timeout ) {
		    printf("Timeout waiting for gain %d ACK\n", i);
		    return false;
		}
	    }
	}
    }

    if ( display_on ) {
	printf("APM2_send_config(): mixing\n");
    }

    pyPropertyNode mixing_node
	= pyGetNode("/config/actuators/actuator/mixing", true);
    count = mixing_node.getLen("mix");
    if ( count ) {
	for ( int i = 0; i < count; i++ ) {
	    string mode = "";
	    int mode_id = 0;
	    bool enable = false;
	    float gain1 = 0.0;
	    float gain2 = 0.0;
	    pyPropertyNode mix_node = mixing_node.getChild("mix", i, true);
	    if ( mix_node.hasChild("mode") ) {
		mode = mix_node.getString("mode");
		if ( mode == "auto_coordination" ) {
		    mode_id = MIX_AUTOCOORDINATE;
		} else if ( mode == "throttle_trim" ) {
		    mode_id = MIX_THROTTLE_TRIM;
		} else if ( mode == "flap_trim" ) {
		    mode_id = MIX_FLAP_TRIM;
		} else if ( mode == "elevon" ) {
		    mode_id = MIX_ELEVONS;
		} else if ( mode == "flaperon" ) {
		    mode_id = MIX_FLAPERONS;
		} else if ( mode == "vtail" ) {
		    mode_id = MIX_VTAIL;
		} else if ( mode == "diff_thrust" ) {
		    mode_id = MIX_DIFF_THRUST;
		}
	    }
	    if ( mix_node.hasChild("enable") ) {
		enable = mix_node.getBool("enable");
	    }
	    if ( mix_node.hasChild("gain1") ) {
		gain1 = mix_node.getDouble("gain1");
	    }
	    if ( mix_node.hasChild("gain2") ) {
		gain2 = mix_node.getDouble("gain2");
	    }
	    if ( display_on ) {
		printf("mix: %s %d %.2f %.2f\n", mode.c_str(), enable,
		       gain1, gain2);
	    }
	    start_time = get_Time();    
	    APM2_act_mix_mode( mode_id, enable, gain1, gain2);
	    last_ack_id = 0;
	    last_ack_subid = 0;
	    while ( (last_ack_id != MIX_MODE_PACKET_ID)
		    || (last_ack_subid != mode_id) )
	     {
		APM2_read();
		if ( get_Time() > start_time + timeout ) {
		    printf("Timeout waiting for %s ACK\n", mode.c_str());
		    return false;
		}
	    }
	}
    }

    if ( display_on ) {
	printf("APM2_send_config(): sas\n");
    }

    pyPropertyNode sas_node = pyGetNode("/config/actuators/actuator/sas", true);
    children = sas_node.getChildren(false);
    count = (int)children.size();
    for ( int i = 0; i < count; ++i ) {
	string mode = "";
	int mode_id = 0;
	bool enable = false;
	float gain = 0.0;
	if ( children[i] == "axis" ) {
	    for ( int j = 0; j < sas_node.getLen("axis"); j++ ) {
		pyPropertyNode sas_section = sas_node.getChild("axis", j);
		if ( sas_section.hasChild("mode") ) {
		    mode = sas_section.getString("mode");
		    if ( mode == "roll" ) {
			mode_id = SAS_ROLLAXIS;
		    } else if ( mode == "pitch" ) {
			mode_id = SAS_PITCHAXIS;
		    } else if ( mode == "yaw" ) {
			mode_id = SAS_YAWAXIS;
		    }
		}
		if ( sas_section.hasChild("enable") ) {
		    enable = sas_section.getBool("enable");
		}
		if ( sas_section.hasChild("gain") ) {
		    gain = sas_section.getDouble("gain");
		}
		if ( display_on ) {
		    printf("sas: %s %d %.2f\n", mode.c_str(), enable, gain);
		}
		start_time = get_Time();    
		APM2_act_sas_mode( mode_id, enable, gain );
		last_ack_id = 0;
		last_ack_subid = 0;
		while ( (last_ack_id != SAS_MODE_PACKET_ID)
			|| (last_ack_subid != mode_id) )
		{
		    APM2_read();
		    if ( get_Time() > start_time + timeout ) {
			printf("Timeout waiting for %s ACK\n", mode.c_str());
			return false;
		    }
		}
	    }
	} else if ( children[i] == "pilot_tune" ) {
	    pyPropertyNode sas_section = sas_node.getChild(children[i].c_str());
	    mode_id = SAS_CH7_TUNE;
	    mode = "ch7_tune";
	    if ( sas_section.hasChild("enable") ) {
		enable = sas_section.getBool("enable");
	    }
	    gain = 0.0; // not used
	    if ( display_on ) {
		printf("sas: %s %d %.2f\n", mode.c_str(), enable, gain);
	    }
	    start_time = get_Time();    
	    APM2_act_sas_mode( mode_id, enable, gain );
	    last_ack_id = 0;
	    last_ack_subid = 0;
	    while ( (last_ack_id != SAS_MODE_PACKET_ID)
		    || (last_ack_subid != mode_id) )
	    {
		APM2_read();
		if ( get_Time() > start_time + timeout ) {
		    printf("Timeout waiting for %s ACK\n", mode.c_str());
		    return false;
		}
	    }
	}
    }

    if ( display_on ) {
	printf("APM2_send_config(): eeprom\n");
    }

    start_time = get_Time();    
    APM2_act_write_eeprom();
    last_ack_id = 0;
    while ( (last_ack_id != WRITE_EEPROM_PACKET_ID) ) {
	APM2_read();
	if ( get_Time() > start_time + timeout ) {
	    if ( display_on ) {
		printf("Timeout waiting for write EEPROM ack...\n");
	    }
	    return false;
	}
    }

    if ( display_on ) {
	printf("APM2_send_config(): end\n");
    }

    return true;
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


static bool APM2_act_write() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = FLIGHT_COMMAND_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 2 * NUM_ACTUATORS;
    /* len = */ write( fd, buf, 2 );

#if 0
    // generate some test data
    static double t = 0.0;
    t += 0.02;
    double dummy = sin(t);
    act_aileron_node.setDouble(dummy);
    act_elevator_node.setDouble(dummy);
    act_throttle_node.setDouble((dummy/2)+0.5);
    act_rudder_node.setDouble(dummy);
    act_channel5_node.setDouble(dummy);
    act_channel6_node.setDouble(dummy);
    act_channel7_node.setDouble(dummy);
    act_channel8_node.setDouble(dummy);
#endif

    // actuator data
    if ( NUM_ACTUATORS == 8 ) {
	int val;
	uint8_t hi, lo;

	val = gen_pulse( act_node.getDouble("channel", 0), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 1), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 2), false );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 3), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 4), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 5), false );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 6), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 7), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;
    }

    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( FLIGHT_COMMAND_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


bool APM2_update() {
    // read any pending APM2 data (and parse any completed messages)
    while ( APM2_read() > 0 );
    // APM2_read_tmp();

    return true;
}


bool APM2_imu_update() {
    static double last_imu_timestamp = 0.0;

    APM2_update();

    if ( imu_inited ) {
	double p_raw = (double)imu_sensors[0] * MPU6000_gyro_scale;
	double q_raw = (double)imu_sensors[1] * MPU6000_gyro_scale;
	double r_raw = (double)imu_sensors[2] * MPU6000_gyro_scale;
	double ax_raw = (double)imu_sensors[3] * MPU6000_accel_scale;
	double ay_raw = (double)imu_sensors[4] * MPU6000_accel_scale;
	double az_raw = (double)imu_sensors[5] * MPU6000_accel_scale;
	int16_t hx = (int16_t)imu_sensors[6];
	int16_t hy = (int16_t)imu_sensors[7];
	int16_t hz = (int16_t)imu_sensors[8];
	double temp_C = (double)imu_sensors[9] * MPU6000_temp_scale;

	if ( reverse_imu_mount ) {
	    // reverse roll/pitch gyros, and x/y accelerometers (and mags).
	    p_raw = -p_raw;
	    q_raw = -q_raw;
	    ax_raw = -ax_raw;
	    ay_raw = -ay_raw;
	    hx = -hx;
	    hy = -hy;
	}

	if ( imu_timestamp > last_imu_timestamp + 5.0 ) {
	    //imu_p_bias_node.setDouble( p_cal.get_bias( temp_C ) );
	    //imu_q_bias_node.setDouble( q_cal.get_bias( temp_C ) );
	    //imu_r_bias_node.setDouble( r_cal.eval_bias( temp_C ) );
	    imu_node.setDouble( "ax_bias", ax_cal.get_bias( temp_C ) );
	    imu_node.setDouble( "ay_bias", ay_cal.get_bias( temp_C ) );
	    imu_node.setDouble( "az_bias", az_cal.get_bias( temp_C ) );
	    last_imu_timestamp = imu_timestamp;
	}

	imu_node.setDouble( "timestamp", imu_timestamp );
	imu_node.setDouble( "p_rad_sec", p_raw );
	imu_node.setDouble( "q_rad_sec", q_raw );
	imu_node.setDouble( "r_rad_sec", r_raw );
	imu_node.setDouble( "ax_mps_sec", ax_cal.calibrate(ax_raw, temp_C) );
	imu_node.setDouble( "ay_mps_sec", ay_cal.calibrate(ay_raw, temp_C) );
	imu_node.setDouble( "az_mps_sec", az_cal.calibrate(az_raw, temp_C) );
	imu_node.setLong( "hx_raw", hx );
	imu_node.setLong( "hy_raw", hy );
	imu_node.setLong( "hz_raw", hz );
	Matrix<double,4,1> hs = {(double)hx, (double)hy, (double)hz, 1.0};
	Matrix<double,4,1> hc = mag_cal * hs;
	imu_node.setDouble( "hx", hc(0) );
	imu_node.setDouble( "hy", hc(1) );
	imu_node.setDouble( "hz", hc(2) );
	imu_node.setDouble( "temp_C", temp_C );
    }

    return true;
}


// This function works ONLY with the UBLOX date format (the ublox reports
// weeks since the GPS epoch.)
static double ublox_date_time_to_unix_sec( int week, float gtime ) {
    double julianDate = (week * 7.0) + 
	(0.001 * gtime) / 86400.0 +  // 86400 = seconds in 1 day
	2444244.5; // 2444244.5 Julian date of GPS epoch (Jan 5 1980 at midnight)
    julianDate = julianDate - 2440587.5; // Subtract Julian Date of Unix Epoch (Jan 1 1970)

    double unixSecs = julianDate * 86400.0;

    // hardcoded handling of leap seconds
    unixSecs -= 17.0;

    /* printf("unix time from gps = %.0f system = %.0f\n", unixSecs,
       system("date +%s")); */

    return unixSecs;
}

#if 0
// This function works ONLY with the MTK16 date format (the ublox reports
// weeks since the GPS epoch.)
static double MTK16_date_time_to_unix_sec( int gdate, float gtime ) {
    gtime /= 1000.0;
    int hour = (int)(gtime / 3600); gtime -= hour * 3600;
    int min = (int)(gtime / 60); gtime -= min * 60;
    int isec = (int)gtime; gtime -= isec;
    float fsec = gtime;

    int day = gdate / 10000; gdate -= day * 10000;
    int mon = gdate / 100; gdate -= mon * 100;
    int year = gdate;

    // printf("%02d:%02d:%02d + %.3f  %02d / %02d / %02d\n", hour, min,
    //        isec, fsec, day, mon,
    //        year );

    struct tm t;
    t.tm_sec = isec;
    t.tm_min = min;
    t.tm_hour = hour;
    t.tm_mday = day;
    t.tm_mon = mon - 1;
    t.tm_year = year + 100;
    t.tm_gmtoff = 0;

    // force timezone to GMT/UTC so mktime() does the proper conversion
    tzname[0] = tzname[1] = (char *)"GMT";
    timezone = 0;
    daylight = 0;
    setenv("TZ", "UTC", 1);
    
    // printf("%d\n", mktime(&t));
    // printf("tzname[0]=%s, tzname[1]=%s, timezone=%d, daylight=%d\n",
    //        tzname[0], tzname[1], timezone, daylight);

    double result = (double)mktime(&t);
    result += fsec;

    // printf("unix time = %.0f\n", result);

    return result;
}
#endif

bool APM2_gps_update() {
    static double last_timestamp = 0.0;
    
    APM2_update();

    if ( !gps_inited ) {
	return false;
    }

    if ( gps_sensors.timestamp > last_timestamp ) {
	gps_node.setDouble( "timestamp", gps_sensors.timestamp );
	gps_node.setDouble( "day_seconds", gps_sensors.time / 1000.0 );
	gps_node.setDouble( "date", gps_sensors.date );
	gps_node.setDouble( "latitude_deg", gps_sensors.latitude / 10000000.0 );
	gps_node.setDouble( "longitude_deg", gps_sensors.longitude / 10000000.0 );
	double alt_m = gps_sensors.altitude / 100.0;
	gps_node.setDouble( "altitude_m", alt_m );
	gps_node.setDouble( "vn_ms", gps_sensors.vel_north * 0.01 );
	gps_node.setDouble( "ve_ms", gps_sensors.vel_east * 0.01 );
	gps_node.setDouble( "vd_ms", gps_sensors.vel_down * 0.01 );
	gps_node.setLong( "satellites", gps_sensors.num_sats);
	gps_node.setDouble( "pdop", gps_sensors.pdop * 0.01);
	gps_node.setLong( "status", gps_sensors.status );
	double unix_secs = ublox_date_time_to_unix_sec( gps_sensors.date,
							gps_sensors.time );
	gps_node.setDouble( "unix_time_sec", unix_secs );
	last_timestamp = gps_sensors.timestamp;
	return true;
    } else {
	return false;
    }
}


bool APM2_airdata_update() {
    APM2_update();

    bool fresh_data = false;
    static double analog0_sum = 0.0;
    static int analog0_count = 0;
    static float analog0_offset = 0.0;
    static float analog0_filter = 0.0;

    if ( airdata_inited ) {
	double cur_time = airdata.timestamp;

	if ( ! airspeed_inited ) {
	    if ( airspeed_zero_start_time > 0 ) {
		analog0_sum += analog[0];
		analog0_count++;
		analog0_offset = analog0_sum / analog0_count;
		//printf("a0 off=%.1f a0 sum=%.1f a0 count=%d\n",
		//	analog0_offset, analog0_sum, analog0_count);
	    } else {
		airspeed_zero_start_time = get_Time();
		analog0_sum = 0.0;
		analog0_count = 0;
		analog0_filter = analog[0];
	    }
	    if ( cur_time > airspeed_zero_start_time + 10.0 ) {
		//printf("analog0_offset = %.2f\n", analog0_offset);
		airspeed_inited = true;
	    }
	}

	airdata_node.setDouble( "timestamp", cur_time );

	// basic pressure to airspeed formula: v = sqrt((2/p) * q)
	// where v = velocity, q = dynamic pressure (pitot tube sensor
	// value), and p = air density.

	// if p is specified in kg/m^3 (value = 1.225) and if q is
	// specified in Pa (N/m^2) where 1 psi == 6900 Pa, then the
	// velocity will be in meters per second.

	// The MPXV5004DP has a full scale span of 3.9V, Maximum
	// pressure reading is 0.57psi (4000Pa)

	// Example (APM2): With a 10bit ADC (APM2) we record a value
	// of 230 (0-1024) at zero velocity.  The sensor saturates at
	// a value of about 1017 (4000psi).  Thus:

	// Pa = (ADC - 230) * 5.083
	// Airspeed(mps) = sqrt( (2/1.225) * Pa )

	// This yields a theoretical maximum speed sensor reading of
	// about 81mps (156 kts)

	// hard coded filter (probably should use constants from the
	// config file, or zero itself out on init.)
	analog0_filter = 0.95 * analog0_filter + 0.05 * analog[0];

	// choose between using raw pitot value or filtered pitot value
	float analog0 = analog[0];
	// float analog0 = analog0_filt;
	
	float Pa = (analog0 - analog0_offset) * 5.083;
	if ( Pa < 0.0 ) { Pa = 0.0; } // avoid sqrt(neg_number) situation
	float airspeed_mps = sqrt( 2*Pa / 1.225 ) * pitot_calibrate;
	float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
	airdata_node.setDouble( "airspeed_mps", airspeed_mps );
	airdata_node.setDouble( "airspeed_kt", airspeed_kt );

	// publish sensor values
	airdata_node.setDouble( "pressure_mbar", airdata.pressure / 100.0 );
	airdata_node.setDouble( "temp_degC", airdata.temp / 10.0 );
	airdata_node.setDouble( "vertical_speed_mps", airdata.climb_rate );
	airdata_node.setDouble( "vertical_speed_fps", airdata.climb_rate * SG_METER_TO_FEET );

	fresh_data = true;
    }

    return fresh_data;
}


// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void APM2_airdata_zero_airspeed() {
    airspeed_inited = false;
    airspeed_zero_start_time = 0.0;
}


bool APM2_pilot_update() {
    APM2_update();

    if ( !pilot_input_inited ) {
	return false;
    }

    float val;

    pilot_node.setDouble( "timestamp", pilot_in_timestamp );

    for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
	val = normalize_pulse( pilot_input[i], pilot_symmetric[i] );
	if ( pilot_invert[i] ) {
	    if ( pilot_symmetric[i] ) {
		val *= -1.0;
	    } else {
		val = 1.0 - val;
	    }
	}
	pilot_node.setDouble( pilot_mapping[i].c_str(), val );
	pilot_node.setDouble( "channel", i, val );
    }

    return true;
}


bool APM2_act_update() {
    static bool actuator_configured = false;
    
    if ( !actuator_inited ) {
	return false;
    }

    if ( !actuator_configured ) {
	actuator_configured = APM2_send_config();
    }
    
    // send actuator commands to APM2 servo subsystem
    APM2_act_write();

    return true;
}


void APM2_close() {
    close(fd);

    master_opened = false;
}


void APM2_imu_close() {
    APM2_close();
}


void APM2_gps_close() {
    APM2_close();
}


void APM2_airdata_close() {
    APM2_close();
}


void APM2_pilot_close() {
    APM2_close();
}


void APM2_act_close() {
    APM2_close();
}
