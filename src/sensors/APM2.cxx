//
// FILE: APM2.cxx
// DESCRIPTION: interact with APM2 with "sensor head" firmware
//

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf) et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()

#include "include/ugear_config.h"

#include "comms/display.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "APM2.hxx"

#define START_OF_MSG0 147
#define START_OF_MSG1 224

#define ACK_PACKET_ID 10
#define ACT_COMMAND_PACKET_ID 20
#define PWM_RATE_PACKET_ID 21
#define BAUD_PACKET_ID 22
#define AUX2_RELAY_PACKET_ID 23

#define PILOT_PACKET_ID 30
#define IMU_PACKET_ID 31
#define GPS_PACKET_ID 32
#define BARO_PACKET_ID 33
#define ANALOG_PACKET_ID 34

#define MAX_PILOT_INPUTS 8
#define MAX_ACTUATORS 8
#define MAX_IMU_SENSORS 7
#define MAX_ANALOG_INPUTS 6

#define PWM_CENTER 1525.0
#define PWM_HALF_RANGE 414.0
#define PWM_RANGE (PWM_HALF_RANGE * 2.0)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)

// APM2 interface and config property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *APM2_device_node = NULL;
static SGPropertyNode *APM2_baud_node = NULL;
static SGPropertyNode *APM2_pwm_rate_node = NULL;
static SGPropertyNode *APM2_volt_ratio_node = NULL;
static SGPropertyNode *APM2_amp_offset_node = NULL;
static SGPropertyNode *APM2_amp_ratio_node = NULL;
static SGPropertyNode *APM2_analog_nodes[MAX_ANALOG_INPUTS];
static SGPropertyNode *APM2_extern_volt_node = NULL;
static SGPropertyNode *APM2_extern_amp_node = NULL;
static SGPropertyNode *APM2_extern_amp_sum_node = NULL;
static SGPropertyNode *APM2_board_vcc_node = NULL;
static SGPropertyNode *APM2_pilot_packet_count_node = NULL;
static SGPropertyNode *APM2_imu_packet_count_node = NULL;
static SGPropertyNode *APM2_gps_packet_count_node = NULL;
static SGPropertyNode *APM2_baro_packet_count_node = NULL;
static SGPropertyNode *APM2_analog_packet_count_node = NULL;

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
static SGPropertyNode *imu_temp_node = NULL;
static SGPropertyNode *imu_p_bias_node = NULL;
static SGPropertyNode *imu_q_bias_node = NULL;
static SGPropertyNode *imu_r_bias_node = NULL;

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

// air data nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_pressure_node = NULL;
static SGPropertyNode *airdata_temperature_node = NULL;
static SGPropertyNode *airdata_altitude_node = NULL;
static SGPropertyNode *airdata_climb_rate_mps_node = NULL;
static SGPropertyNode *airdata_climb_rate_fps_node = NULL;
static SGPropertyNode *airdata_airspeed_mps_node = NULL;
static SGPropertyNode *airdata_airspeed_kt_node = NULL;

static bool master_opened = false;
static bool imu_inited = false;
static bool gps_inited = false;
static bool airdata_inited = false;
static bool pilot_input_inited = false;
static bool actuator_inited = false;

static int fd = -1;
static string device_name = "/dev/ttyS0";
static int baud = 230400;
static int act_pwm_rate_hz = 50;
static float volt_div_ratio = 100; // a nonsense value
static float extern_amp_offset = 0.0;
static float extern_amp_ratio = 0.1; // a nonsense value
static float extern_amp_sum = 0.0;

static bool act_pwm_rate_ack = false;
//static bool baud_rate_ack = false;

static double pilot_in_timestamp = 0.0;
static uint16_t pilot_input[MAX_PILOT_INPUTS]; // internal stash
static bool pilot_input_rev[MAX_PILOT_INPUTS];

static double imu_timestamp = 0.0;
static int16_t imu_sensors[MAX_IMU_SENSORS];

struct gps_sensors_t {
    double timestamp;
    uint32_t time;
    uint32_t date;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    uint16_t ground_speed;
    uint16_t ground_course;
    // int32_t speed_3d;
    int16_t hdop;
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

static float analog[MAX_ANALOG_INPUTS];     // internal stash

static bool airspeed_inited = false;
static double airspeed_zero_start_time = 0.0;

static float ax_bias = 0.0;
static float ay_bias = 0.0;
static float az_bias = 0.0;
static double ax_scale = 1.0;
static double ay_scale = 1.0;
static double az_scale = 1.0;

static uint32_t pilot_packet_counter = 0;
static uint32_t imu_packet_counter = 0;
static uint32_t gps_packet_counter = 0;
static uint32_t baro_packet_counter = 0;
static uint32_t analog_packet_counter = 0;


// initialize fgfs_gps input property nodes
static void bind_input( SGPropertyNode *config ) {
    for ( int i = 0; i < MAX_PILOT_INPUTS; i++ ) {
	pilot_input_rev[i] = false;
    }
    SGPropertyNode *node, *child;
    node = config->getChild("aileron");
    if ( node != NULL ) {
	child = node->getChild("reverse");
	if ( child != NULL ) {
	    pilot_input_rev[0] = child->getBoolValue();
	}
    }
    node = config->getChild("elevator");
    if ( node != NULL ) {
	child = node->getChild("reverse");
	if ( child != NULL ) {
	    pilot_input_rev[1] = child->getBoolValue();
	}
    }
    node = config->getChild("throttle");
    if ( node != NULL ) {
	child = node->getChild("reverse");
	if ( child != NULL ) {
	    pilot_input_rev[2] = child->getBoolValue();
	}
    }
    node = config->getChild("rudder");
    if ( node != NULL ) {
	child = node->getChild("reverse");
	if ( child != NULL ) {
	    pilot_input_rev[3] = child->getBoolValue();
	}
    }

    configroot = config;
}

// initialize imu output property nodes 
static void bind_imu_output( string rootname ) {
    if ( imu_inited ) {
	return;
    }

    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );

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
    imu_temp_node = outputroot->getChild("temp_C", 0, true);
    imu_p_bias_node = outputroot->getChild("p-bias", 0, true);
    imu_q_bias_node = outputroot->getChild("q-bias", 0, true);
    imu_r_bias_node = outputroot->getChild("r-bias", 0, true);

    imu_inited = true;
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
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);

    gps_inited = true;
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

// initialize airdata output property nodes 
static void bind_airdata_output( string rootname ) {
    if ( airdata_inited ) {
	return;
    }

    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );

    airdata_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    airdata_pressure_node = outputroot->getChild("pressure-mbar", 0, true);
    airdata_temperature_node = outputroot->getChild("temp-degC", 0, true);
    airdata_altitude_node = outputroot->getChild("altitude-m", 0, true);
    airdata_climb_rate_mps_node
	= outputroot->getChild("vertical-speed-mps", 0, true);
    airdata_climb_rate_fps_node
	= outputroot->getChild("vertical-speed-fps", 0, true);
    airdata_airspeed_mps_node = outputroot->getChild("airspeed-mps", 0, true);
    airdata_airspeed_kt_node = outputroot->getChild("airspeed-kt", 0, true);

    airdata_inited = true;
}


// initialize airdata output property nodes 
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

    return true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool APM2_open() {
    if ( master_opened ) {
	return true;
    }

    APM2_device_node = fgGetNode("/config/sensors/APM2/device");
    if ( APM2_device_node != NULL ) {
	device_name = APM2_device_node->getStringValue();
    }
    APM2_baud_node = fgGetNode("/config/sensors/APM2/baud");
    if ( APM2_baud_node != NULL ) {
       baud = APM2_baud_node->getIntValue();
    }
    APM2_pwm_rate_node = fgGetNode("/config/sensors/APM2/pwm-hz");
    if ( APM2_pwm_rate_node != NULL ) {
	act_pwm_rate_hz = APM2_pwm_rate_node->getIntValue();
    }
    APM2_volt_ratio_node = fgGetNode("/config/sensors/APM2/volt-divider-ratio");
    if ( APM2_volt_ratio_node != NULL ) {
	volt_div_ratio = APM2_volt_ratio_node->getFloatValue();
    }
    APM2_amp_offset_node = fgGetNode("/config/sensors/APM2/external-amp-offset");
    if ( APM2_amp_offset_node != NULL ) {
	extern_amp_offset = APM2_amp_offset_node->getFloatValue();
    }
    APM2_amp_ratio_node = fgGetNode("/config/sensors/APM2/external-amp-ratio");
    if ( APM2_amp_ratio_node != NULL ) {
	extern_amp_ratio = APM2_amp_ratio_node->getFloatValue();
    }

    for ( int i = 0; i < MAX_ANALOG_INPUTS; i++ ) {
	APM2_analog_nodes[i]
	    = fgGetNode("/sensors/APM2/raw-analog/channel", i, true);
    }
    APM2_extern_volt_node = fgGetNode("/sensors/APM2/extern-volt", true);
    APM2_extern_amp_node = fgGetNode("/sensors/APM2/extern-amps", true);
    APM2_extern_amp_sum_node = fgGetNode("/sensors/APM2/extern-current-mah", true);
    APM2_board_vcc_node = fgGetNode("/sensors/APM2/board-vcc", true);
    APM2_pilot_packet_count_node
	= fgGetNode("/sensors/APM2/pilot-packet-count", true);
    APM2_imu_packet_count_node
	= fgGetNode("/sensors/APM2/imu-packet-count", true);
    APM2_gps_packet_count_node
	= fgGetNode("/sensors/APM2/gps-packet-count", true);
    APM2_baro_packet_count_node
	= fgGetNode("/sensors/APM2/baro-packet-count", true);
    APM2_analog_packet_count_node
	= fgGetNode("/sensors/APM2/analog-packet-count", true);

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
	printf("device open failed ...");
	return false;
    }

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


bool APM2_imu_init( string rootname, SGPropertyNode *config ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_imu_output( rootname );

    SGPropertyNode *node = NULL;
    node = config->getChild("ax-bias");
    if ( node != NULL ) {
	ax_bias = node->getDoubleValue();
    }
    node = config->getChild("ay-bias");
    if ( node != NULL ) {
	ay_bias = node->getDoubleValue();
    }
    node = config->getChild("az-bias");
    if ( node != NULL ) {
	az_bias = node->getDoubleValue();
    }
    node = config->getChild("ax-scale");
    if ( node != NULL ) {
	ax_scale = node->getDoubleValue();
    }
    node = config->getChild("ay-scale");
    if ( node != NULL ) {
	ay_scale = node->getDoubleValue();
    }
    node = config->getChild("az-scale");
    if ( node != NULL ) {
	az_scale = node->getDoubleValue();
    }
 
    return true;
}


bool APM2_gps_init( string rootname, SGPropertyNode *config ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_gps_output( rootname );

    return true;
}


bool APM2_airdata_init( string rootname ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_airdata_output( rootname );

    return true;
}


bool APM2_pilot_init( string rootname ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_pilot_controls( rootname );

    return true;
}


bool APM2_act_init( SGPropertyNode *config ) {
    if ( ! APM2_open() ) {
	return false;
    }

    bind_act_nodes();

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
	result = (pulse - PWM_CENTER) / PWM_HALF_RANGE;
	if ( result < -1.0 ) { result = -1.0; }
	if ( result > 1.0 ) { result = 1.0; }
    } else {
	// i.e. throttle
	result = (pulse - PWM_MIN) / PWM_RANGE;
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
	// printf("Received ACK = %d\n", payload[0]);
	if ( pkt_len == 1 ) {
	    if ( payload[0] == PWM_RATE_PACKET_ID ) {
		act_pwm_rate_ack = true;
	    // } else if ( payload[0] == BAUD_PACKET_ID ) {
	    //   baud_rate_ack = true;
	    }
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in ack\n");
	    }
	}
    } else if ( pkt_id == PILOT_PACKET_ID ) {
	if ( pkt_len == MAX_PILOT_INPUTS * 2 ) {
	    uint8_t lo, hi;

	    pilot_in_timestamp = get_Time();

	    for ( int i = 0; i < MAX_PILOT_INPUTS; i++ ) {
		lo = payload[0 + 2*i]; hi = payload[1 + 2*i];
		pilot_input[i] = hi*256 + lo;
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

	    pilot_packet_counter++;
	    APM2_pilot_packet_count_node->setIntValue( pilot_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in pilot input\n");
	    }
	}
    } else if ( pkt_id == IMU_PACKET_ID ) {
	if ( pkt_len == MAX_IMU_SENSORS * 2 ) {
	    uint8_t lo, hi;

	    imu_timestamp = get_Time();

	    for ( int i = 0; i < MAX_IMU_SENSORS; i++ ) {
		lo = payload[0 + 2*i]; hi = payload[1 + 2*i];
		imu_sensors[i] = hi*256 + lo;
	    }

#if 0
	    if ( display_on ) {
		for ( int i = 0; i < MAX_IMU_SENSORS; i++ ) {
		    printf("%d ", imu_sensors[i]);
		}
		printf("\n");
	    }
#endif
		      
	    imu_packet_counter++;
	    APM2_imu_packet_count_node->setIntValue( imu_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in imu input\n");
	    }
	}
    } else if ( pkt_id == GPS_PACKET_ID ) {
	if ( pkt_len == 28 ) {
	    gps_sensors.timestamp = get_Time();
	    gps_sensors.time = *(uint32_t *)payload; payload += 4;
	    gps_sensors.date = *(uint32_t *)payload; payload += 4;
	    gps_sensors.latitude = *(int32_t *)payload; payload += 4;
	    gps_sensors.longitude = *(int32_t *)payload; payload += 4;
	    gps_sensors.altitude = *(int32_t *)payload; payload += 4;
	    gps_sensors.ground_speed = *(uint16_t *)payload; payload += 2;
	    gps_sensors.ground_course = *(uint16_t *)payload; payload += 2;
	    // gps_sensors.speed_3d = *(int32_t *)payload; payload += 4;
	    gps_sensors.hdop = *(int16_t *)payload; payload += 2;
	    gps_sensors.num_sats = *(uint8_t *)payload; payload += 1;
	    gps_sensors.status = *(uint8_t *)payload; payload += 1;

#if 0
	    if ( display_on ) {
		for ( int i = 0; i < MAX_IMU_SENSORS; i++ ) {
		    printf("%d ", imu_sensors[i]);
		}
		printf("\n");
	    }
#endif
		      
	    gps_packet_counter++;
	    APM2_gps_packet_count_node->setIntValue( gps_packet_counter );

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

#if 0
	    if ( display_on ) {
		printf("baro %.3f %.1f %.1f %.1f\n", airdata.timestamp,
			airdata.pressure, airdata.temp, airdata.climb_rate);
	    }
#endif
		      
	    baro_packet_counter++;
	    APM2_baro_packet_count_node->setIntValue( baro_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in barometer input\n");
	    }
	}
    } else if ( pkt_id == ANALOG_PACKET_ID ) {
	if ( pkt_len == 2 * MAX_ANALOG_INPUTS ) {
	    uint8_t lo, hi;
	    for ( int i = 0; i < MAX_ANALOG_INPUTS; i++ ) {
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
		APM2_analog_nodes[i]->setFloatValue( analog[i] );
	    }

	    // fill in property values that don't belong to some other
	    // sub system right now.
	    double analog_timestamp = get_Time();
	    static double last_analog_timestamp = analog_timestamp;
	    double dt = analog_timestamp - last_analog_timestamp;
	    last_analog_timestamp = analog_timestamp;

	    static float filter_vcc = analog[5];
	    filter_vcc = 0.9999 * filter_vcc + 0.0001 * analog[5];
	    APM2_board_vcc_node->setDoubleValue( filter_vcc );

	    float extern_volts = analog[1] * (filter_vcc/1024.0) * volt_div_ratio;
	    extern_volt_filt = 0.995 * extern_volt_filt + 0.005 * extern_volts;
	    float extern_amps = ((analog[2] * (filter_vcc/1024.0)) - extern_amp_offset) * extern_amp_ratio;
	    extern_amp_filt = 0.99 * extern_amp_filt + 0.01 * extern_amps;
	    /*printf("a[2]=%.1f vcc=%.2f ratio=%.2f amps=%.2f\n",
		analog[2], filter_vcc, extern_amp_ratio, extern_amps); */
	    extern_amp_sum += extern_amps * dt * 0.277777778; // 0.2777... is 1000/3600 (conversion to milli-amp hours)

	    APM2_extern_volt_node->setFloatValue( extern_volt_filt );
	    APM2_extern_amp_node->setFloatValue( extern_amp_filt );
	    APM2_extern_amp_sum_node->setFloatValue( extern_amp_sum );

#if 0
	    if ( display_on ) {
		for ( int i = 0; i < MAX_ANALOG_INPUTS; i++ ) {
		    printf("%.2f ", (float)analog[i] / 64.0);
		}
		printf("\n");
	    }
#endif
		      
	    analog_packet_counter++;
	    APM2_analog_packet_count_node->setIntValue( analog_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("APM2: packet size mismatch in analog input\n");
	    }
	}
    } else if ( pkt_id == AUX2_RELAY_PACKET_ID ) {
	if ( display_on ) {
	    payload[pkt_len] = 0;
	    printf("aux2: %s\n", payload);
	}
		      
	baro_packet_counter++;
	APM2_baro_packet_count_node->setIntValue( baro_packet_counter );
	
	new_data = true;
    }

    return new_data;
}


#if 1
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


static bool APM2_read() {
    static int state = 0;
    static int pkt_id = 0;
    static int pkt_len = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // if ( display_on ) {
    //   printf("read APM2, entry state = %d\n", state);
    // }

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != START_OF_MSG0 ) {
	    // fprintf( stderr, "state0: len = %d val = %2X (%c)\n", len, input[0] , input[0]);
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

    return new_data;
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
	// i.e. throttle
	if ( val < 0.0 ) { val = 0.0; }
	if ( val > 1.0 ) { val = 1.0; }
	pulse = PWM_MIN + (int)(PWM_RANGE * val);
    }

    return pulse;
}


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
    int len;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    len = write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = BAUD_PACKET_ID;
    // packet length (1 byte)
    buf[1] = size;
    len = write( fd, buf, 2 );

    // actuator data
    *(uint32_t *)buf = baud;
  
    // write packet
    len = write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( BAUD_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    len = write( fd, buf, 2 );

    return true;
}
#endif

static bool APM2_act_set_pwm_rates( uint16_t rates[MAX_ACTUATORS] ) {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    int len;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    len = write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = PWM_RATE_PACKET_ID;
    // packet length (1 byte)
    buf[1] = MAX_ACTUATORS * 2;
    len = write( fd, buf, 2 );

    // actuator data
    for ( int i = 0; i < MAX_ACTUATORS; i++ ) {
	uint16_t val = rates[i];
	uint8_t hi = val / 256;
	uint8_t lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;
    }
  
    // write packet
    len = write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( PWM_RATE_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    len = write( fd, buf, 2 );

    return true;
}

static bool APM2_act_write() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    int len;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    len = write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = ACT_COMMAND_PACKET_ID;
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
    act_channel5_node->setFloatValue(dummy);
    act_channel6_node->setFloatValue(dummy);
    act_channel7_node->setFloatValue(dummy);
    act_channel8_node->setFloatValue(dummy);
#endif

    // actuator data
    if ( MAX_ACTUATORS == 8 ) {
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

	val = gen_pulse( act_channel5_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_channel6_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_channel7_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_channel8_node->getFloatValue(), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;
    }
  
    // write packet
    len = write( fd, buf, size );
  
    // check sum (2 bytes)
    APM2_cksum( ACT_COMMAND_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    len = write( fd, buf, 2 );

    return true;
}


bool APM2_update() {
    // read any pending APM2 data (and parse any completed messages)
    while ( APM2_read() );
    // APM2_read_tmp();

    return true;
}


bool APM2_imu_update() {
    APM2_update();

    if ( imu_inited ) {
	const float gyro_scale = 0.0174532 / 16.4;
	const float accel_scale = 9.81 / 4096.0;
	const float temp_scale = 0.02;

	imu_timestamp_node->setDoubleValue( imu_timestamp );
	imu_p_node->setDoubleValue( imu_sensors[0] * gyro_scale );
	imu_q_node->setDoubleValue( imu_sensors[1] * gyro_scale );
	imu_r_node->setDoubleValue( imu_sensors[2] * gyro_scale );
	double ax = (imu_sensors[3]*accel_scale - ax_bias) * ax_scale;
	double ay = (imu_sensors[4]*accel_scale - ay_bias) * ay_scale;
	double az = (imu_sensors[5]*accel_scale - az_bias) * az_scale;
	imu_ax_node->setDoubleValue( ax );
	imu_ay_node->setDoubleValue( ay );
	imu_az_node->setDoubleValue( az );
	imu_temp_node->setDoubleValue( imu_sensors[6] * temp_scale );
    }

    return true;
}


static double date_time_to_unix_sec( int gdate, float gtime ) {
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

    return result;
}


bool APM2_gps_update() {
    static double last_timestamp = 0.0;
    static double last_alt_m = 0.0;

    APM2_update();

    if ( !gps_inited ) {
	return false;
    }

    double dt = gps_sensors.timestamp - last_timestamp;
    if ( dt < 0.001 ) {
	return false;
    }

    gps_timestamp_node->setDoubleValue(gps_sensors.timestamp);
    gps_day_secs_node->setDoubleValue(gps_sensors.time / 1000.0);
    gps_date_node->setDoubleValue(gps_sensors.date);
    gps_lat_node->setDoubleValue(gps_sensors.latitude / 10000000.0);
    gps_lon_node->setDoubleValue(gps_sensors.longitude / 10000000.0);
    double alt_m = gps_sensors.altitude / 100.0;
    gps_alt_node->setDoubleValue( alt_m );

    // compute horizontal speed components
    double speed_mps = gps_sensors.ground_speed * 0.01;
    double angle_rad = (90.0 - gps_sensors.ground_course*0.01)
	* SGD_DEGREES_TO_RADIANS;
    gps_vn_node->setDoubleValue( sin(angle_rad) * speed_mps );
    gps_ve_node->setDoubleValue( cos(angle_rad) * speed_mps );

    // compute vertical speed
    double vspeed_mps = 0.0;
    double da = alt_m - last_alt_m;
    // dt should be safely non zero for a divide or we wouldn't be here
    vspeed_mps = da / dt;
    gps_vd_node->setDoubleValue( -vspeed_mps );
    last_alt_m = alt_m;

    //gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_satellites_node->setIntValue(gps_sensors.num_sats);
    gps_status_node->setIntValue( gps_sensors.status );
    double unix_secs = date_time_to_unix_sec( gps_sensors.date,
					      gps_sensors.time / 1000.0 );
    gps_unix_sec_node->setDoubleValue( unix_secs );

    last_timestamp = gps_sensors.timestamp;

    return true;
}


bool APM2_airdata_update() {
    APM2_update();

    bool fresh_data = false;
    static double last_time = 0.0;
    static double analog0_sum = 0.0;
    static int analog0_count = 0;
    static float analog0_offset = 0.0;
    static float analog0_filter = 0.0;

    if ( airdata_inited ) {
	double cur_time = airdata.timestamp;

	if ( cur_time <= last_time ) {
	    return false;
	}

	if ( ! airspeed_inited ) {
	    if ( airspeed_zero_start_time > 0 ) {
		analog0_sum += analog[0];
		analog0_count++;
		analog0_offset = analog0_sum / analog0_count;
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

	airdata_timestamp_node->setDoubleValue( cur_time );

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

	// hard coded (probably should use constants from the config file,
	// or zero itself out on init.)
	analog0_filter = 0.95 * analog0_filter + 0.05 * analog[0];
	//printf("analog0 = %.2f (analog[0] = %.2f)\n", analog0_filter, analog[0]);
	float Pa = (analog0_filter - analog0_offset) * 5.083;
	if ( Pa < 0.0 ) { Pa = 0.0; } // avoid sqrt(neg_number) situation
	float airspeed_mps = sqrt( 2*Pa / 1.225 );
	float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
	airdata_airspeed_mps_node->setDoubleValue( airspeed_mps );
	airdata_airspeed_kt_node->setDoubleValue( airspeed_kt );

	// Altitude next
	airdata_pressure_node->setDoubleValue( airdata.pressure / 100.0 );
	airdata_temperature_node->setDoubleValue( airdata.temp / 10.0 );
	airdata_climb_rate_mps_node->setDoubleValue( airdata.climb_rate );
	airdata_climb_rate_fps_node->setDoubleValue( airdata.climb_rate * SG_METER_TO_FEET );

	// from here: http://keisan.casio.com/has10/SpecExec.cgi?path=06000000%2eScience%2f02100100%2eEarth%20science%2f12000300%2eAltitude%20from%20atmospheric%20pressure%2fdefault%2exml&charset=utf-8
	const float sea_press = 1013.25;
	float alt_m = ((pow((sea_press / (airdata.pressure/100.0)), 1.0/5.257) - 1.0) * ((airdata.temp/10.0) + 273.15)) / 0.0065;
	airdata_altitude_node->setDoubleValue( alt_m );

	fresh_data = true;

	last_time = cur_time;
    }

    return fresh_data;
}


bool APM2_pilot_update() {
    APM2_update();

    if ( !pilot_input_inited ) {
	return false;
    }

    float val;

    pilot_timestamp_node->setDoubleValue( pilot_in_timestamp );

    val = normalize_pulse( pilot_input[0], true );
    if ( pilot_input_rev[0] ) { val *= -1.0; }
    pilot_aileron_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[1], true );
    if ( pilot_input_rev[1] ) { val *= -1.0; }
    pilot_elevator_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[2], false );
    if ( pilot_input_rev[2] ) { val *= -1.0; }
    pilot_throttle_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[3], true );
    if ( pilot_input_rev[3] ) { val *= -1.0; }
    pilot_rudder_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[4], true );
    if ( pilot_input_rev[4] ) { val *= -1.0; }
    pilot_channel5_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[5], true );
    if ( pilot_input_rev[5] ) { val *= -1.0; }
    pilot_channel6_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[6], true );
    if ( pilot_input_rev[6] ) { val *= -1.0; }
    pilot_channel7_node->setDoubleValue( val );

    val = normalize_pulse( pilot_input[7], true );
    if ( pilot_input_rev[7] ) { val *= -1.0; }
    pilot_channel8_node->setDoubleValue( val );

    pilot_manual_node->setIntValue( pilot_channel8_node->getDoubleValue() > 0 );

    return true;
}


bool APM2_act_update() {
    if ( actuator_inited ) {
	if ( ! act_pwm_rate_ack ) {
	    uint16_t rates[MAX_ACTUATORS] = { 50, 50, 50, 50, 50, 50, 50, 50 };
	    // uint16_t rates[] = { 100, 100, 100, 100, 100, 100, 100, 100 };
	    // uint16_t rates[] = { 200, 200, 200, 200, 200, 200, 200, 200 };
	    // uint16_t rates[] = { 400, 400, 400, 400, 400, 400, 400, 400 };
	    for ( int i = 0; i < MAX_ACTUATORS; i++ ) {
		rates[i] = act_pwm_rate_hz;
	    }
	    APM2_act_set_pwm_rates( rates );
	}

	// send actuator commands to APM2 servo subsystem
	APM2_act_write();
    }

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
