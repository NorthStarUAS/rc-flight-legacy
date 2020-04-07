//
// FILE: Aura3.cpp
// DESCRIPTION: interact with Aura3 (Teensy/Pika) sensor head
//

#include <pyprops.h>

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "comms/display.h"
#include "comms/serial_link.h"
#include "init/globals.h"
#include "util/butter.h"
#include "util/cal_temp.h"
#include "util/linearfit.h"
#include "util/lowpass.h"
#include "util/strutils.h"
#include "util/timing.h"

#include "Aura3.h"
#include "aura3_messages.h"

// FIXME: could be good to be able to define constants in the message.json
#define NUM_IMU_SENSORS 10
#define NUM_ANALOG_INPUTS 6

static pyPropertyNode aura3_node;
static pyPropertyNode power_node;
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode pilot_node;
static pyPropertyNode act_node;
static pyPropertyNode airdata_node;
//static pyPropertyNode analog_node;
static pyPropertyNode aura3_config;
static pyPropertyNode config_specs_node;

static bool master_opened = false;
static bool imu_inited = false;
static bool gps_inited = false;
static bool airdata_inited = false;
static bool pilot_input_inited = false;
static bool actuator_inited = false;
bool Aura3_actuator_configured = false; // externally visible

static SerialLink serial;
static string device_name = "/dev/ttyS4";
static int baud = 500000;

static float volt_div_ratio = 100; // a nonsense value
static int battery_cells = 4;
static float pitot_calibrate = 1.0;

static int last_ack_id = 0;
static int last_ack_subid = 0;

static double pilot_in_timestamp = 0.0;
static float pilot_input[message::sbus_channels]; // internal stash
static uint8_t pilot_flags = 0x00;
static string pilot_mapping[message::sbus_channels]; // channel->name mapping

static double imu_timestamp = 0.0;
static uint32_t imu_micros = 0;
static int16_t imu_sensors[NUM_IMU_SENSORS];

static LinearFitFilter imu_offset(200.0, 0.01);

// 2nd order filter, 100hz sample rate expected, 3rd field is cutoff freq.
// higher freq value == noisier, a value near 1 hz should work well
// for airspeed.
static ButterworthFilter pitot_filter(2, 100, 0.8);

static uint32_t parse_errors = 0;
static uint32_t skipped_frames = 0;

static message::aura_nav_pvt_t nav_pvt;
static message::airdata_t airdata;

static message::config_master_t config_master;
static message::config_imu_t config_imu;
static message::config_actuators_t config_actuators;
static message::config_airdata_t config_airdata;
static message::config_power_t config_power;
static message::config_led_t config_led;

static double nav_pvt_timestamp = 0;

//static LowPassFilter analog_filt[NUM_ANALOG_INPUTS];
//static float analog[NUM_ANALOG_INPUTS];

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
//static uint32_t analog_packet_counter = 0;

// pulled from aura-sensors.ino
const float _pi = 3.14159265358979323846;
const float _g = 9.807;
const float _d2r = _pi / 180.0;

const float _gyro_lsb_per_dps = 32767.5 / 500;  // -500 to +500 spread across 65535
const float gyroScale = _d2r / _gyro_lsb_per_dps;

const float _accel_lsb_per_dps = 32767.5 / 8;   // -4g to +4g spread across 65535
const float accelScale = _g / _accel_lsb_per_dps;

const float magScale = 0.01;
const float tempScale = 0.01;


static bool Aura3_imu_update_internal() {
    static double last_bias_update = 0.0;
    
    if ( imu_inited ) {
        float p_raw = 0.0, q_raw = 0.0, r_raw = 0.0;
        float ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0;
        float hx_raw = 0.0, hy_raw = 0.0, hz_raw = 0.0;
        ax_raw = (float)imu_sensors[0] * accelScale;
        ay_raw = (float)imu_sensors[1] * accelScale;
        az_raw = (float)imu_sensors[2] * accelScale;
        p_raw = (float)imu_sensors[3] * gyroScale;
        q_raw = (float)imu_sensors[4] * gyroScale;
        r_raw = (float)imu_sensors[5] * gyroScale;
        hx_raw = (float)imu_sensors[6] * magScale;
        hy_raw = (float)imu_sensors[7] * magScale;
        hz_raw = (float)imu_sensors[8] * magScale;

        float temp_C = (float)imu_sensors[9] * tempScale;

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
	imu_offset.update(imu_remote_sec, diff);
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
	imu_node.setDouble( "hx_raw", hx_raw );
	imu_node.setDouble( "hy_raw", hy_raw );
	imu_node.setDouble( "hz_raw", hz_raw );
	Vector4d hs((double)hx_raw, (double)hy_raw, (double)hz_raw, 1.0);
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

    static LowPassFilter avionics_vcc_filt(2.0);
    static LowPassFilter int_main_vcc_filt(2.0);
    static LowPassFilter ext_main_vcc_filt(2.0);

    if ( pkt_id == message::command_ack_id ) {
        message::command_ack_t ack;
        ack.unpack(payload, pkt_len);
	if ( pkt_len == ack.len ) {
            last_ack_id = ack.command_id;
	    last_ack_subid = ack.subcommand_id;
            if ( display_on ) {
                printf("Received ACK = %d %d\n", ack.command_id,
                       ack.subcommand_id);
            }
	} else {
	    printf("Aura3: packet size mismatch in ACK\n");
	}
    } else if ( pkt_id == message::pilot_id ) {
        message::pilot_t pilot;
        pilot.unpack(payload, pkt_len);
	if ( pkt_len == pilot.len ) {
	    pilot_in_timestamp = get_Time();
	    for ( int i = 0; i < message::sbus_channels; i++ ) {
		pilot_input[i] = pilot.channel[i];
	    }
            pilot_flags = pilot.flags;

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
		printf("Aura3: packet size mismatch in pilot input packet\n");
	    }
	}
    } else if ( pkt_id == message::imu_raw_id ) {
        message::imu_raw_t imu;
        imu.unpack(payload, pkt_len);
	if ( pkt_len == imu.len ) {
	    imu_timestamp = get_Time();
	    imu_micros = imu.micros;
	    //printf("%d\n", imu_micros);
	    for ( int i = 0; i < NUM_IMU_SENSORS; i++ ) {
		imu_sensors[i] = imu.channel[i];
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
		printf("Aura3: packet size mismatch in imu packet\n");
	    }
	}
    } else if ( pkt_id == message::aura_nav_pvt_id ) {
        nav_pvt.unpack(payload, pkt_len);
	if ( pkt_len == nav_pvt.len ) {
	    nav_pvt_timestamp = get_Time();
	    gps_packet_counter++;
	    aura3_node.setLong( "gps_packet_count", gps_packet_counter );
	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in gps packet\n");
                printf("got %d, expected %d\n", pkt_len, nav_pvt.len);
	    }
	}
    } else if ( pkt_id == message::airdata_id ) {
        airdata.unpack(payload, pkt_len);
	if ( pkt_len == airdata.len ) {
	    // if ( display_on ) {
	    // 	printf("airdata %.3f %.2f %.2f\n", airdata.timestamp,
	    // 		airdata.temp_C, airdata.diff_pres_pa);
	    // }
		      
	    airdata_packet_counter++;
	    aura3_node.setLong( "airdata_packet_count", airdata_packet_counter );

	    new_data = true;
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in airdata packet\n");
	    }
	}
    } else if ( pkt_id == message::power_id ) {
        message::power_t power;
        power.unpack(payload, pkt_len);
	if ( pkt_len == power.len ) {

            // we anticipate a 0.01 sec dt value
            int_main_vcc_filt.update((float)power.int_main_v, 0.01);
            ext_main_vcc_filt.update((float)power.ext_main_v, 0.01);
            avionics_vcc_filt.update((float)power.avionics_v, 0.01);

            power_node.setDouble( "main_vcc", int_main_vcc_filt.get_value() );
            power_node.setDouble( "ext_main_vcc", ext_main_vcc_filt.get_value() );
            power_node.setDouble( "avionics_vcc", avionics_vcc_filt.get_value() );

            float cell_volt = int_main_vcc_filt.get_value() / (float)battery_cells;
            float ext_cell_volt = ext_main_vcc_filt.get_value() / (float)battery_cells;
            power_node.setDouble( "cell_vcc", cell_volt );
            power_node.setDouble( "ext_cell_vcc", ext_cell_volt );
            power_node.setDouble( "main_amps", (float)power.ext_main_amp);
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in power packet\n");
	    }
	}
    } else if ( pkt_id == message::status_id ) {
	static bool first_time = true;
        message::status_t msg;
        msg.unpack(payload, pkt_len);
	if ( pkt_len == msg.len ) {
	    // if ( display_on ) {
            //  printf("info %d %d %d %d\n", serial_num, firmware_rev,
            //         master_hz, baud_rate);
	    // }
	    aura3_node.setLong( "serial_number", msg.serial_number );
	    aura3_node.setLong( "firmware_rev", msg.firmware_rev );
	    aura3_node.setLong( "master_hz", msg.master_hz );
	    aura3_node.setLong( "baud_rate", msg.baud );
	    aura3_node.setLong( "byte_rate_sec", msg.byte_rate );
 
	    if ( first_time ) {
		// log the data to events.txt
		first_time = false;
		char buf[128];
		snprintf( buf, 32, "Serial Number = %d", msg.serial_number );
		events->log("Aura3", buf );
		snprintf( buf, 32, "Firmware Revision = %d", msg.firmware_rev );
		events->log("Aura3", buf );
		snprintf( buf, 32, "Master Hz = %d", msg.master_hz );
		events->log("Aura3", buf );
		snprintf( buf, 32, "Baud Rate = %d", msg.baud );
		events->log("Aura3", buf );
	    }
	} else {
	    if ( display_on ) {
		printf("Aura3: packet size mismatch in status packet\n");
	    }
	}
    } else {
	if ( display_on ) {
	    printf("Aura3: unknown packet id = %d\n", pkt_id);
	}
    }

    return new_data;
}


static bool wait_for_ack(uint8_t id) {
    double timeout = 0.5;
    double start_time = get_Time();
    last_ack_id = 0;
    while ( (last_ack_id != id) ) {
	if ( serial.update() ) {
            Aura3_parse( serial.pkt_id, serial.pkt_len, serial.payload );
        }
	if ( get_Time() > start_time + timeout ) {
	    if ( display_on ) {
		printf("Timeout waiting for ack...\n");
	    }
	    return false;
	}
    }
    return true;
}


static bool write_config_master() {
    config_master.pack();
    serial.write_packet( config_master.id, config_master.payload, config_master.len );
    return wait_for_ack(config_master.id);
}

static bool write_config_imu() {
    config_imu.pack();
    serial.write_packet( config_imu.id, config_imu.payload, config_imu.len );
    return wait_for_ack(config_imu.id);
}

static bool write_config_actuators() {
    config_actuators.pack();
    serial.write_packet( config_actuators.id, config_actuators.payload,
                  config_actuators.len );
    return wait_for_ack(config_actuators.id);
}

static bool write_config_airdata() {
    config_airdata.pack();
    serial.write_packet( config_airdata.id, config_airdata.payload,
                  config_airdata.len );
    return wait_for_ack(config_airdata.id);
}

static bool write_config_led() {
    config_led.pack();
    serial.write_packet( config_led.id, config_led.payload, config_led.len );
    return wait_for_ack(config_led.id);
}

static bool write_config_power() {
    config_power.pack();
    serial.write_packet( config_power.id, config_power.payload,
                  config_power.len );
    return wait_for_ack(config_power.id);
}

static bool write_command_zero_gyros() {
    message::command_zero_gyros_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

static bool write_command_cycle_inceptors() {
    message::command_cycle_inceptors_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
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
    pilot_node.setLen("channel", message::sbus_channels, 0.0);
    pilot_input_inited = true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool Aura3_open_device( int baud ) {
    if ( display_on ) {
	printf("Aura3 Sensor Head on %s @ %d baud\n", device_name.c_str(),
	       baud);
    }

    bool result = serial.open( baud, device_name.c_str() );
    if ( !result ) {
        fprintf( stderr, "Error opening serial link to Aura3 device, cannot continue.\n" );
	return false;
    }

    // bind main property nodes here for lack of a better place..
    aura3_node = pyGetNode("/sensors/Aura3", true);
    power_node = pyGetNode("/sensors/power", true);
    
    return true;
}


// send our configured init strings to configure gpsd the way we prefer
static bool Aura3_open() {
    if ( master_opened ) {
	return true;
    }

    aura3_config = pyGetNode("/config/sensors/Aura3", true);
    config_specs_node = pyGetNode("/config/specs", true);

    //for ( int i = 0; i < NUM_ANALOG_INPUTS; i++ ) {
    //  analog_filt[i].set_time_factor(0.5);
    //}
    
    if ( aura3_config.hasChild("device") ) {
	device_name = aura3_config.getString("device");
    }
    if ( aura3_config.hasChild("baud") ) {
       baud = aura3_config.getLong("baud");
    }
    if ( aura3_config.hasChild("volt_divider_ratio") ) {
	volt_div_ratio = aura3_config.getDouble("volt_divider_ratio");
    }

    if ( aura3_config.hasChild("pitot_calibrate_factor") ) {
	pitot_calibrate = aura3_config.getDouble("pitot_calibrate_factor");
    }
    
    if ( config_specs_node.hasChild("battery_cells") ) {
	battery_cells = config_specs_node.getLong("battery_cells");
    }
    if ( battery_cells < 1 ) { battery_cells = 1; }

    if ( ! Aura3_open_device( baud ) ) {
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

        printf("\nFIXME!!!!!!!!!!!!!!!!!!!!!\n\n");
        #if 0
        //p_cal.init( cal->getChild("p"), min_temp, max_temp );
        //q_cal.init( cal->getChild("q"), min_temp, max_temp );
        //r_cal.init( cal->getChild("r"), min_temp, max_temp );
        pyPropertyNode ax_node = cal.getChild("ax");
        ax_cal.init( &ax_node, min_temp, max_temp );
        pyPropertyNode ay_node = cal.getChild("ay");
        ay_cal.init( &ay_node, min_temp, max_temp );
        pyPropertyNode az_node = cal.getChild("az");
        az_cal.init( &az_node, min_temp, max_temp );
        #endif
        
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
        write_imu_calibration( &cal );
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
	for ( int i = 0; i < message::sbus_channels; i++ ) {
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


// master board selector defaults
static void master_defaults() {
    config_master.board = 0;
}

// Setup imu defaults:
// Marmot v1 has mpu9250 on SPI CS line 24
// Aura v2 has mpu9250 on I2C Addr 0x68
static void imu_setup_defaults() {
    config_imu.interface = 0;       // SPI
    config_imu.pin_or_address = 24; // CS pin
    float ident[] = { 1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    for ( int i = 0; i < 9; i++ ) {
        config_imu.orientation[i] = ident[i];
    }
}

// reset pwm output rates to safe startup defaults
static void pwm_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
         config_actuators.pwm_hz[i] = 50;    
    }
}

// reset actuator gains (reversing) to startup defaults
static void act_gain_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
        config_actuators.act_gain[i] = 1.0;
    }
}

// reset airdata to startup defaults
static void airdata_defaults() {
    config_airdata.barometer = 0;
    config_airdata.pitot = 0;
    config_airdata.swift_baro_addr = 0;
    config_airdata.swift_pitot_addr = 0;
}

// reset sas parameters to startup defaults
static void sas_defaults() {
    config_actuators.sas_rollaxis = false;
    config_actuators.sas_pitchaxis = false;
    config_actuators.sas_yawaxis = false;
    config_actuators.sas_tune = false;

    config_actuators.sas_rollgain = 0.0;
    config_actuators.sas_pitchgain = 0.0;
    config_actuators.sas_yawgain = 0.0;
    config_actuators.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
static void mixing_defaults() {
    config_actuators.mix_autocoord = false;
    config_actuators.mix_throttle_trim = false;
    config_actuators.mix_flap_trim = false;
    config_actuators.mix_elevon = false;
    config_actuators.mix_flaperon = false;
    config_actuators.mix_vtail = false;
    config_actuators.mix_diff_thrust = false;

    config_actuators.mix_Gac = 0.5;       // aileron gain for autocoordination
    config_actuators.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config_actuators.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config_actuators.mix_Gea = 1.0;       // aileron gain for elevons
    config_actuators.mix_Gee = 1.0;       // elevator gain for elevons
    config_actuators.mix_Gfa = 1.0;       // aileron gain for flaperons
    config_actuators.mix_Gff = 1.0;       // flaps gain for flaperons
    config_actuators.mix_Gve = 1.0;       // elevator gain for vtail
    config_actuators.mix_Gvr = 1.0;       // rudder gain for vtail
    config_actuators.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config_actuators.mix_Gtr = 0.1;       // rudder gain for diff thrust
};

static void power_defaults() {
    config_power.have_attopilot = false;
}

static void led_defaults() {
    config_led.pin = 0;
}

// send a full configuration to Aura3 and return true only when all
// parameters are acknowledged.
static bool Aura3_send_config() {
    if ( display_on ) {
	printf("Aura3: building config structure.\n");
    }

    vector<string> children;

    // set all parameters to defaults
    master_defaults();
    imu_setup_defaults();
    pwm_defaults();
    act_gain_defaults();
    airdata_defaults();
    mixing_defaults();
    sas_defaults();
    power_defaults();
    led_defaults();

    int count;

    if ( aura3_config.hasChild("board") ) {
        string board = aura3_config.getString("board");
        if ( board == "marmot_v1" ) {
            config_master.board = 0;
        } else if ( board == "aura_v2" ) {
            config_master.board = 1;
        } else {
            printf("Warning: no valid PWM pin layout defined.\n");
        }
    }

    if ( aura3_config.hasChild("have_attopilot") ) {
        config_power.have_attopilot = aura3_config.getBool("have_attopilot");
    }

    pyPropertyNode imu_node
        = pyGetNode("/config/sensors/imu_group/imu", true);
    if ( imu_node.hasChild("orientation") ) {
        int len = imu_node.getLen("orientation");
        if ( len == 9 ) {
            for ( int i = 0; i < len; i++ ) {
                config_imu.orientation[i] = imu_node.getDouble("orientation", i);
            }
        } else {
            printf("WARNING: imu orienation improper matrix size\n");
        }
    } else {
        printf("Note: no imu orientation defined, default is identity matrix\n");
    }
    if ( imu_node.hasChild("interface") ) {
        string interface = imu_node.getString("interface");
        if ( interface == "spi" ) {
            config_imu.interface = 0;
            config_imu.pin_or_address = imu_node.getLong("cs_pin");
        } else if ( interface == "i2c" ) {
            config_imu.interface = 1;
        } else {
            printf("Warning: unknown IMU interface = %s\n", interface.c_str());
        }
    }
            
    pyPropertyNode pwm_node
	= pyGetNode("/config/actuators/actuator/pwm_rates", true);
    count = pwm_node.getLen("channel");
    for ( int i = 0; i < count; i++ ) {
        config_actuators.pwm_hz[i] = pwm_node.getLong("channel", i);
        if ( display_on ) {
            printf("pwm_hz[%d] = %d\n", i, config_actuators.pwm_hz[i]);
        }
    }

    pyPropertyNode gain_node
	= pyGetNode("/config/actuators/actuator/gains", true);
    count = gain_node.getLen("channel");
    for ( int i = 0; i < count; i++ ) {
        config_actuators.act_gain[i] = gain_node.getDouble("channel", i);
        if ( display_on ) {
            printf("act_gain[%d] = %.2f\n", i, config_actuators.act_gain[i]);
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
                    config_actuators.mix_autocoord = enable;
                    config_actuators.mix_Gac = gain1;
		} else if ( mode == "throttle_trim" ) {
                    config_actuators.mix_throttle_trim = enable;
                    config_actuators.mix_Get = gain1;
		} else if ( mode == "flap_trim" ) {
                    config_actuators.mix_flap_trim = enable;
                    config_actuators.mix_Gef = gain1;
		} else if ( mode == "elevon" ) {
                    config_actuators.mix_elevon = enable;
                    config_actuators.mix_Gea = gain1;
                    config_actuators.mix_Gee = gain2;
		} else if ( mode == "flaperon" ) {
                    config_actuators.mix_flaperon = enable;
                    config_actuators.mix_Gfa = gain1;
                    config_actuators.mix_Gff = gain2;
		} else if ( mode == "vtail" ) {
                    config_actuators.mix_vtail = enable;
                    config_actuators.mix_Gve = gain1;
                    config_actuators.mix_Gvr = gain2;
		} else if ( mode == "diff_thrust" ) {
                    config_actuators.mix_diff_thrust = enable;
                    config_actuators.mix_Gtt = gain1;
                    config_actuators.mix_Gtr = gain2;
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
                        config_actuators.sas_rollaxis = enable;
                        config_actuators.sas_rollgain = gain;
		    } else if ( mode == "pitch" ) {
                        config_actuators.sas_pitchaxis = enable;
                        config_actuators.sas_pitchgain = gain;
		    } else if ( mode == "yaw" ) {
                        config_actuators.sas_yawaxis = enable;
                        config_actuators.sas_yawgain = gain;
		    }
		}
		if ( display_on ) {
		    printf("sas: %s %d %.2f\n", mode.c_str(), enable, gain);
		}
	    }
	} else if ( children[i] == "pilot_tune" ) {
	    pyPropertyNode sas_section = sas_node.getChild("pilot_tune");
	    if ( sas_section.hasChild("enable") ) {
		config_actuators.sas_tune = sas_section.getBool("enable");
	    }
	    if ( display_on ) {
		printf("sas: global tune %d\n", config_actuators.sas_tune);
	    }
	}
    }

    pyPropertyNode airdata_node
        = pyGetNode("/config/sensors/airdata_group/airdata", true);
    if ( airdata_node.hasChild("barometer") ) {
        string baro = airdata_node.getString("barometer");
        if ( baro == "bme280" ) {
            config_airdata.barometer = 0;
        } else if ( baro == "bmp280" ) {
            config_airdata.barometer = 1;
        } else if ( baro == "swift" ) {
            config_airdata.barometer = 2;
            config_airdata.swift_baro_addr = airdata_node.getLong("swift_baro_addr");
        }
    }
    if ( airdata_node.hasChild("pitot") ) {
        string pitot = airdata_node.getString("pitot");
        if ( pitot == "ms4525" ) {
            config_airdata.pitot = 0;
        } else if ( pitot == "ms5525" ) {
            config_airdata.pitot = 1;
        } else if ( pitot == "swift" ) {
            config_airdata.pitot = 2;
            config_airdata.swift_pitot_addr = airdata_node.getLong("swift_pitot_addr");
        }
    }
    
    if ( aura3_config.hasChild("led_pin") ) {
        config_led.pin = aura3_config.getLong("led_pin");
    }

    if ( display_on ) {
	printf("Aura3: transmitting master config ...\n");
    }
    if ( !write_config_master() ) {
        return false;
    }

    if ( display_on ) {
	printf("Aura3: transmitting imu config ...\n");
    }
    if ( !write_config_imu() ) {
        return false;
    }

    if ( display_on ) {
	printf("Aura3: transmitting actuator config ...\n");
    }
    if ( !write_config_actuators() ) {
        return false;
    }

    if ( display_on ) {
	printf("Aura3: transmitting airdata config ...\n");
    }
    if ( !write_config_airdata() ) {
        return false;
    }

    if ( display_on ) {
	printf("Aura3: transmitting power config ...\n");
    }
    if ( !write_config_power() ) {
        return false;
    }

    if ( display_on ) {
	printf("Aura3: transmitting led config ...\n");
    }
    if ( !write_config_led() ) {
        return false;
    }

    if ( display_on ) {
	printf("Aura3_send_config(): end\n");
    }

    return true;
}


static bool Aura3_act_write() {
    // actuator data
    if ( message::ap_channels = 6 ) {
        message::command_inceptors_t act;
	act.channel[0] = act_node.getDouble("throttle");
	act.channel[1] = act_node.getDouble("aileron");
	act.channel[2] = act_node.getDouble("elevator");
	act.channel[3] = act_node.getDouble("rudder");
	act.channel[4] = act_node.getDouble("flaps");
	act.channel[5] = act_node.getDouble("gear");
        act.pack();
        serial.write_packet( act.id, act.payload, act.len );
        return true;
    } else {
        return false;
    }
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

    while ( true ) {
        if ( serial.update() ) {
            Aura3_parse( serial.pkt_id, serial.pkt_len, serial.payload );
            if ( serial.pkt_id == message::imu_raw_id ) {
                if ( serial.bytes_available() < 256 ) {
                    // a smaller value here means more skipping ahead and
                    // less catching up.
                    break;
                } else {
                    skipped_frames++;
                }
            }
        }
    }

    // track communication errors from FMU
    aura3_node.setLong("parse_errors", parse_errors);
    aura3_node.setLong("skipped_frames", skipped_frames);

    // experimental: write optional zero gyros command back to FMU upon request
    string command = aura3_node.getString( "command" );
    if ( command.length() ) {
        if ( command == "zero_gyros" ) {
            if ( write_command_zero_gyros() ) {
                aura3_node.setString( "command", "" );
                aura3_node.setString( "command_result",
                                      "success: " + command );
            }
        } else {
            // unknown command
            aura3_node.setString( "command", "" );
            aura3_node.setString( "command_result",
                                  "unknown command: " + command );
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
	gps_node.setDouble( "horiz_accuracy_m", nav_pvt.hAcc / 1000.0 );
	gps_node.setDouble( "vert_accuracy_m", nav_pvt.vAcc / 1000.0 );
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
	double cur_time = imu_timestamp;

	float pitot_butter = pitot_filter.update(airdata.ext_diff_press_pa);
        
	if ( ! airspeed_inited ) {
	    if ( airspeed_zero_start_time > 0.0 ) {
		pitot_sum += airdata.ext_diff_press_pa;
		pitot_count++;
		pitot_offset = pitot_sum / (double)pitot_count;
		/* printf("a1 raw=%.1f filt=%.1f a1 off=%.1f a1 sum=%.1f a1 count=%d\n",
		   analog[0], pitot_filt.get_value(), pitot_offset, pitot_sum,
		   pitot_count); */
	    } else {
		airspeed_zero_start_time = get_Time();
		pitot_sum = 0.0;
		pitot_count = 0;
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
	// float pitot = airdata.diff_pres_pa;
	float pitot = pitot_butter;
	
	float Pa = (pitot - pitot_offset);
	if ( Pa < 0.0 ) { Pa = 0.0; } // avoid sqrt(neg_number) situation
	float airspeed_mps = sqrt( 2*Pa / 1.225 ) * pitot_calibrate;
	float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
	airdata_node.setDouble( "airspeed_mps", airspeed_mps );
	airdata_node.setDouble( "airspeed_kt", airspeed_kt );
	airdata_node.setDouble( "temp_C", airdata.ext_temp_C );

	// publish sensor values
	airdata_node.setDouble( "pressure_mbar", airdata.baro_press_pa / 100.0 );
	airdata_node.setDouble( "bme_temp_C", airdata.baro_temp_C );
	airdata_node.setDouble( "humidity", airdata.baro_hum );
	airdata_node.setDouble( "diff_pressure_pa", airdata.ext_diff_press_pa );
	airdata_node.setDouble( "ext_static_press_pa", airdata.ext_static_press_pa );
        airdata_node.setLong( "error_count", airdata.error_count );

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

    for ( int i = 0; i < message::sbus_channels; i++ ) {
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
    serial.close();

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
