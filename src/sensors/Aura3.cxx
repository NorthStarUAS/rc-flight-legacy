//
// FILE: Aura3.cxx
// DESCRIPTION: interact with Aura3 (Teensy/Pika) sensor head
//

#include "python/pyprops.hxx"

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()
#include <math.h>		// M_PI
#include <sys/ioctl.h>

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "init/globals.hxx"
#include "sensors/cal_temp.hxx"
#include "util/linearfit.hxx"
#include "util/lowpass.hxx"
//#include "util/poly1d.hxx"
#include "util/timing.h"

#include "Aura3.hxx"

#define START_OF_MSG0 147
#define START_OF_MSG1 224

#define ACK_PACKET_ID 20

#define CONFIG_PACKET_ID 21
#define FLIGHT_COMMAND_PACKET_ID 22
#define WRITE_EEPROM_PACKET_ID 23

#define PILOT_PACKET_ID 50
#define IMU_PACKET_ID 51
#define GPS_PACKET_ID 52
#define AIRDATA_PACKET_ID 53
#define ANALOG_PACKET_ID 54
#define STATUS_INFO_PACKET_ID 55

#define NUM_PILOT_INPUTS 18
#define NUM_IMU_SENSORS 10
#define NUM_ANALOG_INPUTS 6
#define PWM_CHANNELS 5
#define AP_CHANNELS 6

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

static pyPropertyNode aura3_node;
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
bool Aura3_actuator_configured = false; // externally visible

static int fd = -1;
static string device_name = "/dev/ttyS0";
static int baud = 500000;
static float volt_div_ratio = 100; // a nonsense value
static int battery_cells = 4;
static float extern_amp_offset = 0.0;
static float extern_amp_ratio = 0.1; // a nonsense value
static float extern_amp_sum = 0.0;
static float pitot_calibrate = 1.0;
static bool reverse_imu_mount = false;

static int last_ack_id = 0;
static int last_ack_subid = 0;

static double pilot_in_timestamp = 0.0;
static float pilot_input[NUM_PILOT_INPUTS]; // internal stash
static uint8_t pilot_flags = 0x00;
static string pilot_mapping[NUM_PILOT_INPUTS]; // channel->name mapping

static double imu_timestamp = 0.0;
static uint32_t imu_micros = 0;
static int16_t imu_sensors[NUM_IMU_SENSORS];

static LinearFitFilter imu_offset(200.0);

#pragma pack(push, 1)           // set alignment to 1 byte boundary

// configuration structure
typedef struct {
    int version;
    
    /* hz for pwm output signal, 50hz default for analog servos, maximum rate is servo dependent:
       digital servos can usually do 200-250hz
       analog servos and ESC's typically require 50hz */
    uint16_t pwm_hz[PWM_CHANNELS];
    
    /* actuator gain (reversing/scaling) */
    float act_gain[PWM_CHANNELS];
    
    /* mixing modes */
    bool mix_autocoord;
    bool mix_throttle_trim;
    bool mix_flap_trim;
    bool mix_elevon;
    bool mix_flaperon;
    bool mix_vtail;
    bool mix_diff_thrust;

    /* mixing gains */
    float mix_Gac; // aileron gain for autocoordination
    float mix_Get; // elevator trim w/ throttle gain
    float mix_Gef; // elevator trim w/ flap gain
    float mix_Gea; // aileron gain for elevons
    float mix_Gee; // elevator gain for elevons
    float mix_Gfa; // aileron gain for flaperons
    float mix_Gff; // flaps gain for flaperons
    float mix_Gve; // elevator gain for vtail
    float mix_Gvr; // rudder gain for vtail
    float mix_Gtt; // throttle gain for diff thrust
    float mix_Gtr; // rudder gain for diff thrust
    
    /* sas modes */
    bool sas_rollaxis;
    bool sas_pitchaxis;
    bool sas_yawaxis;
    bool sas_ch7tune;

    /* sas gains */
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_ch7gain;
} config_t;

// gps structure
static struct nav_pvt_t {
    uint32_t iTOW;
    int16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
} nav_pvt;
#pragma pack(pop)              // restore original alignment

config_t config;
static double nav_pvt_timestamp = 0;

static struct air_data_t {
    double timestamp;
    float static_pres_pa;
    float diff_pres_pa;
} airdata;

static LowPassFilter analog_filt[NUM_ANALOG_INPUTS];
static float analog[NUM_ANALOG_INPUTS];

static bool airspeed_inited = false;
static double airspeed_zero_start_time = 0.0;

//static AuraCalTemp p_cal;
//static AuraCalTemp q_cal;
//static AuraCalTemp r_cal;
static AuraCalTemp ax_cal;
static AuraCalTemp ay_cal;
static AuraCalTemp az_cal;
static Matrix4d mag_cal;

static uint32_t pilot_packet_counter = 0;
static uint32_t imu_packet_counter = 0;
static uint32_t gps_packet_counter = 0;
static uint32_t airdata_packet_counter = 0;
static uint32_t analog_packet_counter = 0;

// pulled from apm2-sensors.ino
const float _pi = 3.14159265358979323846;
const float _g = 9.807;
const float _d2r = _pi / 180.0;

const float _gyro_lsb_per_dps = 32767.5 / 500;  // -500 to +500 spread across 65535
const float gyroScale = _d2r / _gyro_lsb_per_dps;

const float _accel_lsb_per_dps = 32767.5 / 8;   // -4g to +4g spread across 65535
const float accelScale = _g / _accel_lsb_per_dps;

const float magScale = 0.01;
const float tempScale = 0.01;


static void Aura3_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
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
bool Aura3_request_baud( uint32_t baud ) {
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
    Aura3_cksum( BAUD_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}
#endif

static bool Aura3_act_write_eeprom() {
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
    Aura3_cksum( WRITE_EEPROM_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


static bool Aura3_write_config() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = sizeof(config);
    // uint8_t len;
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = CONFIG_PACKET_ID;
    // packet length (1 byte)
    buf[1] = size;
    /* len = */ write( fd, buf, 2 );

    // write packet
    /* len = */ write( fd, &config, size );
  
    // check sum (2 bytes)
    Aura3_cksum( CONFIG_PACKET_ID, size, (uint8_t *)&config, size, &cksum0, &cksum1 );
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
static void bind_act_nodes() {
    if ( actuator_inited ) {
	return;
    }
    act_node = pyGetNode("/actuators", true);
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
    pilot_node.setLen("channel", NUM_PILOT_INPUTS, 0.0);
    pilot_input_inited = true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool Aura3_open_device( int baud_bits ) {
    if ( display_on ) {
	printf("Aura3 Sensor Head on %s @ %d(code) baud\n", device_name.c_str(),
	       baud_bits);
    }

    // fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY );
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
    // fcntl(fd, F_SETFL, O_NONBLOCK);

    // bind main apm2 property nodes here for lack of a better place..
    aura3_node = pyGetNode("/sensors/Aura3", true);
    analog_node = pyGetNode("/sensors/Aura3/raw_analog", true);
    analog_node.setLen("channel", NUM_ANALOG_INPUTS, 0.0);
    
    return true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool Aura3_open() {
    if ( master_opened ) {
	return true;
    }

    pyPropertyNode apm2_config = pyGetNode("/config/sensors/Aura3", true);

    for ( int i = 0; i < NUM_ANALOG_INPUTS; i++ ) {
	analog_filt[i].set_time_factor(0.5);
    }
    
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

    int baud_bits = B500000;
    if ( baud == 115200 ) {
	baud_bits = B115200;
    } else if ( baud == 500000 ) {
	baud_bits = B500000;
     } else {
	printf("unsupported baud rate = %d\n", baud);
    }

    if ( ! Aura3_open_device( baud_bits ) ) {
	printf("device open failed ...\n");
	return false;
    }

    sleep(1);
    
    master_opened = true;

    return true;
}


#if 0
bool Aura3_init( SGPropertyNode *config ) {
    printf("Aura3_init()\n");

    bind_input( config );

    bool result = Aura3_open();

    return result;
}
#endif


bool Aura3_imu_init( string output_path, pyPropertyNode *config ) {
    if ( ! Aura3_open() ) {
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


bool Aura3_gps_init( string output_path, pyPropertyNode *config ) {
    if ( ! Aura3_open() ) {
	return false;
    }

    bind_gps_output( output_path );

    return true;
}


bool Aura3_airdata_init( string output_path ) {
    if ( ! Aura3_open() ) {
	return false;
    }

    bind_airdata_output( output_path );

    return true;
}


bool Aura3_pilot_init( string output_path, pyPropertyNode *config ) {
    if ( ! Aura3_open() ) {
	return false;
    }

    bind_pilot_controls( output_path );

    if ( config->hasChild("channel") ) {
	for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
	    pilot_mapping[i] = config->getString("channel", i);
	    printf("pilot input: channel %d maps to %s\n", i, pilot_mapping[i].c_str());
	}
    }

    return true;
}


bool Aura3_act_init( pyPropertyNode *section ) {
    if ( ! Aura3_open() ) {
	return false;
    }

    bind_act_nodes();

    return true;
}


static bool Aura3_imu_update_internal() {
    static double last_bias_update = 0.0;
    
    if ( imu_inited ) {
	float ax_raw = (float)imu_sensors[0] * accelScale;
	float ay_raw = (float)imu_sensors[1] * accelScale;
	float az_raw = (float)imu_sensors[2] * accelScale;
	float p_raw = (float)imu_sensors[3] * gyroScale;
	float q_raw = (float)imu_sensors[4] * gyroScale;
	float r_raw = (float)imu_sensors[5] * gyroScale;
	float hx = (float)imu_sensors[6] * magScale;
	float hy = (float)imu_sensors[7] * magScale;
	float hz = (float)imu_sensors[8] * magScale;
	float temp_C = (float)imu_sensors[9] * tempScale;

	if ( reverse_imu_mount ) {
	    // reverse roll/pitch gyros, and x/y accelerometers (and mags).
	    p_raw = -p_raw;
	    q_raw = -q_raw;
	    ax_raw = -ax_raw;
	    ay_raw = -ay_raw;
	    hx = -hx;
	    hy = -hy;
	}

	if ( imu_timestamp > last_bias_update + 5.0 ) {
	    //imu_p_bias_node.setDouble( p_cal.get_bias( temp_C ) );
	    //imu_q_bias_node.setDouble( q_cal.get_bias( temp_C ) );
	    //imu_r_bias_node.setDouble( r_cal.eval_bias( temp_C ) );
	    imu_node.setDouble( "ax_bias", ax_cal.get_bias( temp_C ) );
	    imu_node.setDouble( "ay_bias", ay_cal.get_bias( temp_C ) );
	    imu_node.setDouble( "az_bias", az_cal.get_bias( temp_C ) );
	    last_bias_update = imu_timestamp;
	}

	// timestamp dance: this is a little jig that I do to make a
	// more consistent time stamp that still is in the host
	// reference frame.  Assumes the Aura3 clock drifts relative to
	// host clock.  Assumes the Aura3 imu stamp dt is very stable.
	// Assumes the host system is not-real time and there may be
	// momentary external disruptions to execution. The code
	// estimates the error (difference) between Aura3 clock and
	// host clock.  Then builds a real time linear fit of Aura3
	// clock versus difference with the host.  This linear fit is
	// used to estimate the current error (smoothly), add that to
	// the Aura3 clock and derive a more regular/stable IMU time
	// stamp (versus just sampling current host time.)
	
	// imu_micros &= 0xffffff; // 24 bits = 16.7 microseconds roll over
	
	static uint32_t last_imu_micros = 0;
	double imu_remote_sec = (double)imu_micros / 1000000.0;
	double diff = imu_timestamp - imu_remote_sec;
	if ( last_imu_micros > imu_micros ) {
	    events->log("Aura3", "micros() rolled over\n");
	    imu_offset.reset();
	}
	imu_offset.update(imu_remote_sec, diff, 0.01);
	double fit_diff = imu_offset.get_value(imu_remote_sec);
	// printf("fit_diff = %.6f  diff = %.6f  ts = %.6f\n",
	//        fit_diff, diff, imu_remote_sec + fit_diff );

	last_imu_micros = imu_micros;
	
	imu_node.setDouble( "timestamp", imu_remote_sec + fit_diff );
	imu_node.setLong( "imu_micros", imu_micros );
	imu_node.setDouble( "imu_sec", (double)imu_micros / 1000000.0 );
	imu_node.setDouble( "p_rad_sec", p_raw );
	imu_node.setDouble( "q_rad_sec", q_raw );
	imu_node.setDouble( "r_rad_sec", r_raw );
	imu_node.setDouble( "ax_mps_sec", ax_cal.calibrate(ax_raw, temp_C) );
	imu_node.setDouble( "ay_mps_sec", ay_cal.calibrate(ay_raw, temp_C) );
	imu_node.setDouble( "az_mps_sec", az_cal.calibrate(az_raw, temp_C) );
	imu_node.setDouble( "hx_raw", hx );
	imu_node.setDouble( "hy_raw", hy );
	imu_node.setDouble( "hz_raw", hz );
	Vector4d hs((double)hx, (double)hy, (double)hz, 1.0);
	Vector4d hc = mag_cal * hs;
	imu_node.setDouble( "hx", hc(0) );
	imu_node.setDouble( "hy", hc(1) );
	imu_node.setDouble( "hz", hc(2) );
	imu_node.setDouble( "temp_C", temp_C );
    }

    return true;
}

static bool Aura3_parse( uint8_t pkt_id, uint8_t pkt_len,
			uint8_t *payload )
{
    bool new_data = false;

    if ( pkt_id == ACK_PACKET_ID ) {
	if ( display_on ) {
	    printf("Received ACK = %d %d\n", payload[0], payload[1]);
	}
	if ( pkt_len == 2 ) {
	    last_ack_id = payload[0];
	    last_ack_subid = payload[1];
	} else {
	    printf("Aura3: packet size mismatch in ACK\n");
	}
    } else if ( pkt_id == PILOT_PACKET_ID ) {
	if ( pkt_len == (NUM_PILOT_INPUTS-2) * 2 + 1 ) {
	    pilot_in_timestamp = get_Time();
	    for ( int i = 0; i < (NUM_PILOT_INPUTS-2); i++ ) {
                int16_t val = *(int16_t *)payload; payload += 2;
		pilot_input[i] = (float)val / 16384.0;
	    }
            pilot_flags = *(uint8_t *)payload; payload += 1;

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
	    aura3_node.setLong( "pilot_packet_count", pilot_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in pilot input\n");
	    }
	}
    } else if ( pkt_id == IMU_PACKET_ID ) {
	if ( pkt_len == 4 + NUM_IMU_SENSORS * 2 ) {
	    imu_timestamp = get_Time();
	    imu_micros = *(uint32_t *)payload; payload += 4;
	    //printf("%d\n", imu_micros);
	    
	    for ( int i = 0; i < NUM_IMU_SENSORS; i++ ) {
		imu_sensors[i] = *(int16_t *)payload; payload += 2;
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
	    aura3_node.setLong( "imu_packet_count", imu_packet_counter );

	    // update the propery tree and timestamps
	    Aura3_imu_update_internal();

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in imu input\n");
	    }
	}
    } else if ( pkt_id == GPS_PACKET_ID ) {
	if ( pkt_len == sizeof(nav_pvt) ) {
	    nav_pvt_timestamp = get_Time();
            nav_pvt = *(nav_pvt_t *)(payload);
	    gps_packet_counter++;
	    aura3_node.setLong( "gps_packet_count", gps_packet_counter );
	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in gps input\n");
                printf("got %d, expected %d\n", pkt_len, (int)sizeof(nav_pvt));
	    }
	}
    } else if ( pkt_id == AIRDATA_PACKET_ID ) {
	if ( pkt_len == 8 ) {
	    airdata.timestamp = get_Time();
	    airdata.static_pres_pa = *(float *)payload; payload += 4;
	    airdata.diff_pres_pa = *(float *)payload; payload += 4;

	    // if ( display_on ) {
	    // 	printf("airdata %.3f %.2f %.2f\n", airdata.timestamp,
	    // 		airdata.static_pres_pa, airdata.diff_pres_pa);
	    // }
		      
	    airdata_packet_counter++;
	    aura3_node.setLong( "airdata_packet_count", airdata_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in airdata input\n");
	    }
	}
    } else if ( pkt_id == ANALOG_PACKET_ID ) {
	if ( pkt_len == 2 * NUM_ANALOG_INPUTS ) {
	    for ( int i = 0; i < NUM_ANALOG_INPUTS; i++ ) {
		analog_filt[i].update(*(uint16_t *)payload, 0.01);
		payload += 2;
		analog[i] = analog_filt[i].get_value() ;
		if ( i == 5 ) {
		    analog[i] /= 1000.0;
		} else {
		    analog[i] /= 64.0;
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

	    static LowPassFilter vcc_filt(10.0);
	    vcc_filt.update(analog[5], dt);
	    aura3_node.setDouble( "board_vcc", vcc_filt.get_value() );

	    float extern_volts = analog[1] * (vcc_filt.get_value()/1024.0) * volt_div_ratio;
	    static LowPassFilter extern_volt_filt(2.0);
	    extern_volt_filt.update(extern_volts, dt);
	    float cell_volt = extern_volt_filt.get_value() / (float)battery_cells;
	    float extern_amps = ((analog[2] * (vcc_filt.get_value()/1024.0)) - extern_amp_offset) * extern_amp_ratio;
	    static LowPassFilter extern_amp_filt(1.0);
	    extern_amp_filt.update(extern_amps, dt);
	    /*printf("a[2]=%.1f vcc=%.2f ratio=%.2f amps=%.2f\n",
		analog[2], vcc_filt, extern_amp_ratio, extern_amps); */
	    extern_amp_sum += extern_amp_filt.get_value() * dt * 0.277777778; // 0.2777... is 1000/3600 (conversion to milli-amp hours)

	    aura3_node.setDouble( "extern_volt", extern_volt_filt.get_value() );
	    aura3_node.setDouble( "extern_cell_volt", cell_volt );
	    aura3_node.setDouble( "extern_amps", extern_amp_filt.get_value() );
	    aura3_node.setDouble( "extern_current_mah", extern_amp_sum );

#if 0
	    if ( display_on ) {
		for ( int i = 0; i < NUM_ANALOG_INPUTS; i++ ) {
		    printf("%.2f ", (float)analog[i] / 64.0);
		}
		printf("\n");
	    }
#endif
		      
	    analog_packet_counter++;
	    aura3_node.setLong( "analog_packet_count", analog_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in analog input\n");
	    }
	}
    } else if ( pkt_id == STATUS_INFO_PACKET_ID ) {
	static bool first_time = true;
	if ( pkt_len == 14 ) {
	    uint16_t serial_num = *(uint16_t *)payload; payload += 2;
	    uint16_t firmware_rev = *(uint16_t *)payload; payload += 2;
	    uint16_t master_hz = *(uint16_t *)payload; payload += 2;
	    uint32_t baud_rate = *(uint32_t *)payload; payload += 4;
	    uint16_t byte_rate = *(uint16_t *)payload; payload += 2;
            uint16_t pwr_v = *(uint16_t *)payload; payload += 2;

#if 0
	    if ( display_on ) {
		printf("info %d %d %d %d\n", serial_num, firmware_rev,
		       master_hz, baud_rate);
	    }
#endif
		      
	    aura3_node.setLong( "serial_number", serial_num );
	    aura3_node.setLong( "firmware_rev", firmware_rev );
	    aura3_node.setLong( "master_hz", master_hz );
	    aura3_node.setLong( "baud_rate", baud_rate );
	    aura3_node.setLong( "byte_rate_sec", byte_rate );
            aura3_node.setDouble( "avionics_vcc", (float)pwr_v / 100.0);

	    if ( first_time ) {
		// log the data to events.txt
		first_time = false;
		char buf[128];
		snprintf( buf, 32, "Serial Number = %d", serial_num );
		events->log("Aura3", buf );
		snprintf( buf, 32, "Firmware Revision = %d", firmware_rev );
		events->log("Aura3", buf );
		snprintf( buf, 32, "Master Hz = %d", master_hz );
		events->log("Aura3", buf );
		snprintf( buf, 32, "Baud Rate = %d", baud_rate );
		events->log("Aura3", buf );
	    }
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in config info\n");
	    }
	}
    } else {
	if ( display_on ) {
	    printf("Aura3: unknown packet id = %d\n", pkt_id);
	}
    }

    return new_data;
}


#if 0
static void Aura3_read_tmp() {
    int len;
    uint8_t input[16];
    len = read( fd, input, 1 );
    while ( len > 0 ) {
	printf("%c", input[0]);
	len = read( fd, input, 1 );
    }
}
#endif


static int Aura3_read() {
    static int state = 0;
    static int pkt_id = 0;
    static int pkt_len = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];
    int giveup_counter = 0;

    // if ( display_on ) {
    //    printf("read Aura3, entry state = %d\n", state);
    // }

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	giveup_counter = 0;
	while ( len > 0 && input[0] != START_OF_MSG0 && giveup_counter < 100 ) {
	    // printf("state0: len = %d val = %2X (%c)\n", len, input[0] , input[0]);
	    len = read( fd, input, 1 );
	    giveup_counter++;
	    // fprintf( stderr, "giveup_counter = %d\n", giveup_counter);
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
		//fprintf( stderr, "read START_OF_MSG1\n");
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
	if ( len > 0 ) {
	    pkt_id = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    //fprintf( stderr, "pkt_id = %d\n", pkt_id );
	    state++;
	}
    }
    if ( state == 3 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    pkt_len = input[0];
	    if ( pkt_len < 256 ) {
		//fprintf( stderr, "pkt_len = %d\n", pkt_len );
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
		// printf( "checksum passes (%d)\n", pkt_id );
		new_data = Aura3_parse( pkt_id, pkt_len, payload );
	    } else {
		if ( display_on ) {
		    // printf("checksum failed %d %d (computed) != %d %d (message)\n",
		    //        cksum_A, cksum_B, cksum_lo, cksum_hi );
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


// reset pwm output rates to safe startup defaults
static void pwm_rate_defaults() {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
         config.pwm_hz[i] = 50;    
    }
}

// reset actuator gains (reversing) to startup defaults
static void act_gain_defaults() {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        config.act_gain[i] = 1.0;
    }
}

// reset sas parameters to startup defaults
static void sas_defaults() {
    config.sas_rollaxis = false;
    config.sas_pitchaxis = false;
    config.sas_yawaxis = false;
    config.sas_ch7tune = false;

    config.sas_rollgain = 0.0;
    config.sas_pitchgain = 0.0;
    config.sas_yawgain = 0.0;
    config.sas_ch7gain = 2.0;
};


// reset mixing parameters to startup defaults
static void mixing_defaults() {
    config.mix_autocoord = false;
    config.mix_throttle_trim = false;
    config.mix_flap_trim = false;
    config.mix_elevon = false;
    config.mix_flaperon = false;
    config.mix_vtail = false;
    config.mix_diff_thrust = false;

    config.mix_Gac = 0.5;       // aileron gain for autocoordination
    config.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config.mix_Gea = 1.0;       // aileron gain for elevons
    config.mix_Gee = 1.0;       // elevator gain for elevons
    config.mix_Gfa = 1.0;       // aileron gain for flaperons
    config.mix_Gff = 1.0;       // flaps gain for flaperons
    config.mix_Gve = 1.0;       // elevator gain for vtail
    config.mix_Gvr = 1.0;       // rudder gain for vtail
    config.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config.mix_Gtr = 0.1;       // rudder gain for diff thrust
};


// send a full configuration to Aura3 and return true only when all
// parameters are acknowledged.
static bool Aura3_send_config() {
    if ( display_on ) {
	printf("Aura3: building config structure.\n");
    }

    double start_time = 0.0;
    double timeout = 0.5;
    vector<string> children;

    pyPropertyNode apm2_config = pyGetNode("/config/sensors/Aura3", true);

    // set all parameters to defaults
    pwm_rate_defaults();
    act_gain_defaults();
    mixing_defaults();
    sas_defaults();

    int count;
    
    pyPropertyNode pwm_node
	= pyGetNode("/config/actuators/actuator/pwm_rates", true);
    count = pwm_node.getLen("channel");
    for ( int i = 0; i < count; i++ ) {
        config.pwm_hz[i] = pwm_node.getLong("channel", i);
        if ( display_on ) {
            printf("pwm_hz[%d] = %d\n", i, config.pwm_hz[i]);
        }
    }

    pyPropertyNode gain_node
	= pyGetNode("/config/actuators/actuator/gains", true);
    count = gain_node.getLen("channel");
    for ( int i = 0; i < count; i++ ) {
        config.act_gain[i] = gain_node.getDouble("channel", i);
        if ( display_on ) {
            printf("act_gain[%d] = %.2f\n", i, config.act_gain[i]);
        }
    }

    pyPropertyNode mixing_node
	= pyGetNode("/config/actuators/actuator/mixing", true);
    count = mixing_node.getLen("mix");
    if ( count ) {
	for ( int i = 0; i < count; i++ ) {
	    string mode = "";
	    bool enable = false;
	    float gain1 = 0.0;
	    float gain2 = 0.0;
	    pyPropertyNode mix_node = mixing_node.getChild("mix", i, true);
	    if ( mix_node.hasChild("enable") ) {
		enable = mix_node.getBool("enable");
	    }
	    if ( mix_node.hasChild("gain1") ) {
		gain1 = mix_node.getDouble("gain1");
	    }
	    if ( mix_node.hasChild("gain2") ) {
		gain2 = mix_node.getDouble("gain2");
	    }
	    if ( mix_node.hasChild("mode") ) {
		mode = mix_node.getString("mode");
		if ( mode == "auto_coordination" ) {
                    config.mix_autocoord = enable;
                    config.mix_Gac = gain1;
		} else if ( mode == "throttle_trim" ) {
                    config.mix_throttle_trim = enable;
                    config.mix_Get = gain1;
		} else if ( mode == "flap_trim" ) {
                    config.mix_flap_trim = enable;
                    config.mix_Gef = gain1;
		} else if ( mode == "elevon" ) {
                    config.mix_elevon = enable;
                    config.mix_Gea = gain1;
                    config.mix_Gee = gain2;
		} else if ( mode == "flaperon" ) {
                    config.mix_flaperon = enable;
                    config.mix_Gfa = gain1;
                    config.mix_Gff = gain2;
		} else if ( mode == "vtail" ) {
                    config.mix_vtail = enable;
                    config.mix_Gve = gain1;
                    config.mix_Gvr = gain2;
		} else if ( mode == "diff_thrust" ) {
                    config.mix_diff_thrust = enable;
                    config.mix_Gtt = gain1;
                    config.mix_Gtr = gain2;
		}
	    }
	    if ( display_on ) {
		printf("mix: %s %d %.2f %.2f\n", mode.c_str(), enable,
		       gain1, gain2);
	    }
	}
    }

    pyPropertyNode sas_node = pyGetNode("/config/actuators/actuator/sas", true);
    children = sas_node.getChildren(false);
    count = (int)children.size();
    for ( int i = 0; i < count; ++i ) {
	string mode = "";
	bool enable = false;
	float gain = 0.0;
	if ( children[i] == "axis" ) {
	    for ( int j = 0; j < sas_node.getLen("axis"); j++ ) {
		pyPropertyNode sas_section = sas_node.getChild("axis", j);
		if ( sas_section.hasChild("enable") ) {
		    enable = sas_section.getBool("enable");
		}
		if ( sas_section.hasChild("gain") ) {
		    gain = sas_section.getDouble("gain");
		}
		if ( sas_section.hasChild("mode") ) {
		    mode = sas_section.getString("mode");
		    if ( mode == "roll" ) {
                        config.sas_rollaxis = enable;
                        config.sas_rollgain = gain;
		    } else if ( mode == "pitch" ) {
                        config.sas_pitchaxis = enable;
                        config.sas_pitchgain = gain;
		    } else if ( mode == "yaw" ) {
                        config.sas_yawaxis = enable;
                        config.sas_yawgain = gain;
		    }
		}
		if ( display_on ) {
		    printf("sas: %s %d %.2f\n", mode.c_str(), enable, gain);
		}
	    }
	} else if ( children[i] == "pilot_tune" ) {
	    pyPropertyNode sas_section = sas_node.getChild("pilot_tune");
	    mode = "ch7_tune";
	    if ( sas_section.hasChild("enable") ) {
		config.sas_ch7tune = sas_section.getBool("enable");
	    }
	    if ( display_on ) {
		printf("sas: %s %d\n", mode.c_str(), enable);
	    }
	}
    }

    if ( display_on ) {
	printf("Aura3: transmitting config ...\n");
    }
    start_time = get_Time();    
    Aura3_write_config();
    last_ack_id = 0;
    while ( (last_ack_id != CONFIG_PACKET_ID) ) {
	Aura3_read();
	if ( get_Time() > start_time + timeout ) {
	    if ( display_on ) {
		printf("Timeout waiting for write config ack...\n");
	    }
	    return false;
	}
    }

    if ( display_on ) {
	printf("Aura3: requesting save config ...\n");
    }
    start_time = get_Time();    
    Aura3_act_write_eeprom();
    last_ack_id = 0;
    while ( (last_ack_id != WRITE_EEPROM_PACKET_ID) ) {
	Aura3_read();
	if ( get_Time() > start_time + timeout ) {
	    if ( display_on ) {
		printf("Timeout waiting for write EEPROM ack...\n");
	    }
	    return false;
	}
    }

    if ( display_on ) {
	printf("Aura3_send_config(): end\n");
    }

    return true;
}


static bool Aura3_act_write() {
    uint8_t buf[256];
    uint8_t *p = buf;
    uint8_t cksum0, cksum1;
    uint8_t size = 2 * AP_CHANNELS;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = FLIGHT_COMMAND_PACKET_ID;
    // packet length (1 byte)
    buf[1] = size;
    /* len = */ write( fd, buf, 2 );

    // actuator data
    if ( AP_CHANNELS == 6 ) {
	int val;

	val = act_node.getDouble("throttle") * 16384.0;
        *(int16_t *)p = val; p += 2;

	val = act_node.getDouble("aileron") * 16384.0;
        *(int16_t *)p = val; p += 2;

	val = act_node.getDouble("elevator") * 16384.0;
        *(int16_t *)p = val; p += 2;

	val = act_node.getDouble("rudder") * 16384.0;
        *(int16_t *)p = val; p += 2;

	val = act_node.getDouble("flaps") * 16384.0;
        *(int16_t *)p = val; p += 2;

	val = act_node.getDouble("gear") * 16384.0;
        *(int16_t *)p = val; p += 2;
    }

    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    Aura3_cksum( FLIGHT_COMMAND_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


// Read Aura3 packets using IMU packet as the main timing reference.
// Returns the dt from the IMU perspective, not the localhost
// perspective.  This should generally be far more accurate and
// consistent.
double Aura3_update() {
    // read packets until we receive an IMU packet and the uart buffer
    // is mostly empty.  The IMU packet (combined with being caught up
    // reading the uart buffer is our signal to run an interation of
    // the main loop.
    double last_time = imu_node.getDouble( "timestamp" );
    int bytes_available = 0;
    while ( true ) {
        int pkt_id = Aura3_read();
        if ( pkt_id == IMU_PACKET_ID ) {
            ioctl(fd, FIONREAD, &bytes_available);
	    if ( bytes_available < 64 ) {
		break;
            }
        }
    }
    double cur_time = imu_node.getDouble( "timestamp" );

    return cur_time - last_time;
}


// this keeps the imu_mgr happy, but the real work to update the
// property tree is performed right away when we receive and parse the
// packet.
bool Aura3_imu_update() {
    return true;
}


bool Aura3_gps_update() {
    static double last_timestamp = 0.0;
    
    if ( !gps_inited ) {
	return false;
    }

    if ( nav_pvt_timestamp > last_timestamp ) {
	gps_node.setDouble( "timestamp", nav_pvt_timestamp );
	gps_node.setLong( "year", nav_pvt.year );
	gps_node.setLong( "month", nav_pvt.month );
	gps_node.setLong( "day", nav_pvt.day );
	gps_node.setLong( "hour", nav_pvt.hour );
	gps_node.setLong( "min", nav_pvt.min );
	gps_node.setLong( "sec", nav_pvt.sec );
	gps_node.setDouble( "latitude_deg", nav_pvt.lat / 10000000.0 );
	gps_node.setDouble( "longitude_deg", nav_pvt.lon / 10000000.0 );
	gps_node.setDouble( "altitude_m", nav_pvt.hMSL / 1000.0 );
	gps_node.setDouble( "vn_ms", nav_pvt.velN / 1000.0 );
	gps_node.setDouble( "ve_ms", nav_pvt.velE / 1000.0 );
	gps_node.setDouble( "vd_ms", nav_pvt.velD / 1000.0 );
	gps_node.setLong( "satellites", nav_pvt.numSV);
	gps_node.setDouble( "pdop", nav_pvt.pDOP / 100.0 );
        gps_node.setLong( "fixType", nav_pvt.fixType );
        // backwards compatibility
	if ( nav_pvt.fixType == 0 ) {
	    gps_node.setLong( "status", 0 );
	} else if ( nav_pvt.fixType == 1 || nav_pvt.fixType == 2 ) {
	    gps_node.setLong( "status", 1 );
	} else if ( nav_pvt.fixType == 3 ) {
	    gps_node.setLong( "status", 2 );
	}
        struct tm gps_time;
	gps_time.tm_sec = nav_pvt.sec;
	gps_time.tm_min = nav_pvt.min;
	gps_time.tm_hour = nav_pvt.hour;
	gps_time.tm_mday = nav_pvt.day;
	gps_time.tm_mon = nav_pvt.month - 1;
	gps_time.tm_year = nav_pvt.year - 1900;
	double unix_sec = (double)mktime( &gps_time ) - timezone;
	unix_sec += nav_pvt.nano / 1000000000.0;
	gps_node.setDouble( "unix_time_sec", unix_sec );
	last_timestamp = nav_pvt_timestamp;
	return true;
    } else {
	return false;
    }
}


bool Aura3_airdata_update() {
    bool fresh_data = false;
    static double pitot_sum = 0.0;
    static int pitot_count = 0;
    static float pitot_offset = 0.0;
    static LowPassFilter pitot_filt(0.2);

    if ( airdata_inited ) {
	double cur_time = airdata.timestamp;

	pitot_filt.update(airdata.diff_pres_pa, 0.01);
	if ( ! airspeed_inited ) {
	    if ( airspeed_zero_start_time > 0.0 ) {
		pitot_sum += airdata.diff_pres_pa;
		pitot_count++;
		pitot_offset = pitot_sum / (double)pitot_count;
		/* printf("a1 raw=%.1f filt=%.1f a1 off=%.1f a1 sum=%.1f a1 count=%d\n",
		   analog[0], pitot_filt.get_value(), pitot_offset, pitot_sum,
		   pitot_count); */
	    } else {
		airspeed_zero_start_time = get_Time();
		pitot_sum = 0.0;
		pitot_count = 0;
		pitot_filt.init(airdata.diff_pres_pa);
	    }
	    if ( cur_time > airspeed_zero_start_time + 10.0 ) {
		//printf("pitot_offset = %.2f\n", pitot_offset);
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

	// Example (Aura3): With a 10bit ADC (Aura3) we record a value
	// of 230 (0-1024) at zero velocity.  The sensor saturates at
	// a value of about 1017 (4000psi).  Thus:

	// Pa = (ADC - 230) * 5.083
	// Airspeed(mps) = sqrt( (2/1.225) * Pa )

	// This yields a theoretical maximum speed sensor reading of
	// about 81mps (156 kts)

	// choose between using raw pitot value or filtered pitot value
	float pitot = airdata.diff_pres_pa;
	// float pitot = pitot_filt.get_value();
	
	float Pa = (pitot - pitot_offset);
	if ( Pa < 0.0 ) { Pa = 0.0; } // avoid sqrt(neg_number) situation
	float airspeed_mps = sqrt( 2*Pa / 1.225 ) * pitot_calibrate;
	float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
	airdata_node.setDouble( "airspeed_mps", airspeed_mps );
	airdata_node.setDouble( "airspeed_kt", airspeed_kt );

	// publish sensor values
	airdata_node.setDouble( "pressure_mbar", airdata.static_pres_pa / 100.0 );
	airdata_node.setDouble( "diff_pressure_pa", airdata.diff_pres_pa );

	fresh_data = true;
    }

    return fresh_data;
}


// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void Aura3_airdata_zero_airspeed() {
    airspeed_inited = false;
    airspeed_zero_start_time = 0.0;
}


bool Aura3_pilot_update() {
    if ( !pilot_input_inited ) {
	return false;
    }

    float val;

    pilot_node.setDouble( "timestamp", pilot_in_timestamp );

    for ( int i = 0; i < NUM_PILOT_INPUTS; i++ ) {
	val = pilot_input[i];
	pilot_node.setDouble( pilot_mapping[i].c_str(), val );
	pilot_node.setDouble( "channel", i, val );
    }

    // sbus ch17 (channel[16])
    if ( pilot_flags & 0x01 ) {
        pilot_node.setDouble( "channel", 16, 1.0 );
    } else {
        pilot_node.setDouble( "channel", 16, 0.0 );
    }
    // sbus ch18 (channel[17])
    if ( pilot_flags & (1 << 1) ) {
        pilot_node.setDouble( "channel", 17, 1.0 );
    } else {
        pilot_node.setDouble( "channel", 17, 0.0 );
    }
    if ( pilot_flags & (1 << 2) ) {
        pilot_node.setBool( "frame_lost", true );
    } else {
        pilot_node.setBool( "frame_lost", false );
    }
    if ( pilot_flags & (1 << 3) ) {
        pilot_node.setBool( "fail_safe", true );
    } else {
        pilot_node.setBool( "fail_safe", false );
    }

    return true;
}


bool Aura3_act_update() {
    if ( !actuator_inited ) {
	return false;
    }

    if ( !Aura3_actuator_configured ) {
	Aura3_actuator_configured = Aura3_send_config();
    }
    
    // send actuator commands to Aura3 servo subsystem
    Aura3_act_write();

    return true;
}


void Aura3_close() {
    close(fd);

    master_opened = false;
}


void Aura3_imu_close() {
    Aura3_close();
}


void Aura3_gps_close() {
    Aura3_close();
}


void Aura3_airdata_close() {
    Aura3_close();
}


void Aura3_pilot_close() {
    Aura3_close();
}


void Aura3_act_close() {
    Aura3_close();
}
