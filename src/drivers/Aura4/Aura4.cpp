 //
// FILE: Aura4.cpp
// DESCRIPTION: interact with Aura4 FMU
//

// TODO:
// - straighten out what I'm doing with imu timestamp dance
// - straighten out what I'm doing with skipped frames
// - gps age?
// - parse ekf packet
// - write actuators
// - deal with how to choose output paths in property tree

#include <pyprops.h>

#include <stdarg.h>
#include <stdlib.h>             // exit()

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "comms/display.h"
#include "comms/logging.h"
#include "init/globals.h"
#include "util/timing.h"

#include "Aura4.h"

// FIXME: could be good to be able to define constants in the message.json
const int NUM_IMU_SENSORS = 10;

// pulled from aura-sensors/src/imu.cpp
const float _pi = 3.14159265358979323846;
const float _g = 9.807;
const float _d2r = _pi / 180.0;

const float _gyro_lsb_per_dps = 32767.5 / 500;  // -500 to +500 spread across 65535
const float gyroScale = _d2r / _gyro_lsb_per_dps;

const float _accel_lsb_per_dps = 32767.5 / 8;   // -4g to +4g spread across 65535
const float accelScale = _g / _accel_lsb_per_dps;

const float magScale = 0.01;
const float tempScale = 0.01;


// fixme: this could go in a central location for general use
static string get_next_path( const char *path, const char *base,
                             bool primary )
{
    pyPropertyNode node = pyGetNode(path, true);
    int len = node.getLen(base);
    if ( primary and len > 0 ) {
        // nop
    } else {
        node.setLen(base, len + 1);
    }
    ostringstream output_path;
    if ( primary ) {
        output_path << path << "/" << base << "[0]";
    } else {
        output_path << path << "/" << base << '[' << len << ']';
    }
    printf("  new path: %s\n", output_path.str().c_str());
    return output_path.str();
}

void Aura4_t::info( const char *format, ... ) {
    if ( display_on ) {
        printf("Aura4: ");
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\n");
    }
}

void Aura4_t::hard_error( const char *format, ... ) {
    printf("Aura4 hard error: ");
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
    printf("Cannot continue.");
    exit(-1);
}

void Aura4_t::init( pyPropertyNode *config ) {
    // bind main property nodes
    aura4_node = pyGetNode("/sensors/Aura4", true);
    power_node = pyGetNode("/sensors/power", true);

    if ( true ) {               // fixme: move or delete
        // initialize property nodes and configurable values
        aura4_config = pyGetNode("/config/sensors/Aura4", true);

        if ( aura4_config.hasChild("pitot_calibrate_factor") ) {
            pitot_calibrate = aura4_config.getDouble("pitot_calibrate_factor");
        }

        pyPropertyNode specs_node = pyGetNode("/config/specs", true);
        if ( specs_node.hasChild("battery_cells") ) {
            battery_cells = specs_node.getLong("battery_cells");
        }
        if ( battery_cells < 1 ) { battery_cells = 1; }
    }
    
    if ( config->hasChild("board") ) {
        pyPropertyNode board_config = config->getChild("board");
        open( &board_config );
    } else {
        hard_error("no board defined\n");
    }

    if ( config->hasChild("airdata") ) {
        pyPropertyNode airdata_config = config->getChild("airdata");
        init_airdata( &airdata_config );
    } else {
        hard_error("no airdata configuration\n");
    }
    
    if ( config->hasChild("gps") ) {
        pyPropertyNode gps_config = config->getChild("gps");
        init_gps( &gps_config );
    } else {
        hard_error("no gps configuration\n");
    }
    
    if ( config->hasChild("imu") ) {
        pyPropertyNode imu_config = config->getChild("imu");
        init_imu( &imu_config );
    } else {
        hard_error("no imu configuration\n");
    }

    if ( config->hasChild("pilot_input") ) {
        pyPropertyNode pilot_config = config->getChild("pilot");
        init_pilot( &pilot_config );
    } else {
        hard_error("no pilot configuration\n");
    }

    // fixme: stuff should go here?
    // if ( config->hasChild("actuators") ) {
    //     pyPropertyNode act_config = config->getChild("actuators");
    //     init_actuators( &act_config );
    // } else {
    //     hard_error("no actuator configuration\n");
    // }

    sleep(1);
}

bool Aura4_t::open( pyPropertyNode *config ) {
    if ( config->hasChild("device") ) {
	device_name = config->getString("device");
    }
    if ( config->hasChild("baud") ) {
       baud = config->getLong("baud");
    }
    
    if ( serial.is_open() ) {
        info("device already open");
        return true;
    } else {
        info("device on %s @ %d baud", device_name.c_str(), baud);
    }

    bool result = serial.open( baud, device_name.c_str() );
    if ( !result ) {
        hard_error("Error opening serial link to Aura4 device");
    }

    return true;
}

void Aura4_t::init_airdata( pyPropertyNode *config ) {
    string output_path = get_next_path("/sensors", "airdata", true);
    airdata_node = pyGetNode(output_path.c_str(), true);
}

void Aura4_t::init_gps( pyPropertyNode *config ) {
    string output_path = get_next_path("/sensors", "gps", true);
    gps_node = pyGetNode(output_path.c_str(), true);
}

void Aura4_t::init_imu( pyPropertyNode *config ) {
    string output_path = get_next_path("/sensors", "imu", true);
    imu_node = pyGetNode(output_path.c_str(), true);

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
        write_imu_calibration( &cal );
    }
}

void Aura4_t::init_pilot( pyPropertyNode *config ) {
    string output_path = get_next_path("/sensors", "pilot", true);
    pilot_node = pyGetNode(output_path.c_str(), true);
    if ( config->hasChild("channel") ) {
	for ( int i = 0; i < message::sbus_channels; i++ ) {
	    pilot_mapping[i] = config->getString("channel", i);
	    printf("pilot input: channel %d maps to %s\n", i, pilot_mapping[i].c_str());
	}
    }
    pilot_node.setLen("channel", message::sbus_channels, 0.0);
}

void Aura4_t::init_actuators( pyPropertyNode *config ) {
    act_node = pyGetNode("/actuators", true);
}

bool Aura4_t::update_imu( message::imu_raw_t *imu ) {
    imu_timestamp = get_Time();

    float p_raw = 0.0, q_raw = 0.0, r_raw = 0.0;
    float ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0;
    float hx_raw = 0.0, hy_raw = 0.0, hz_raw = 0.0;
    ax_raw = (float)imu->channel[0] * accelScale;
    ay_raw = (float)imu->channel[1] * accelScale;
    az_raw = (float)imu->channel[2] * accelScale;
    p_raw = (float)imu->channel[3] * gyroScale;
    q_raw = (float)imu->channel[4] * gyroScale;
    r_raw = (float)imu->channel[5] * gyroScale;
    hx_raw = (float)imu->channel[6] * magScale;
    hy_raw = (float)imu->channel[7] * magScale;
    hz_raw = (float)imu->channel[8] * magScale;

    float temp_C = (float)imu->channel[9] * tempScale;

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
    // reference frame.  Assumes the Aura4 clock drifts relative to
    // host clock.  Assumes the Aura4 imu stamp dt is very stable.
    // Assumes the host system is not-real time and there may be
    // momentary external disruptions to execution. The code
    // estimates the error (difference) between Aura4 clock and
    // host clock.  Then builds a real time linear fit of Aura4
    // clock versus difference with the host.  This linear fit is
    // used to estimate the current error (smoothly), add that to
    // the Aura4 clock and derive a more regular/stable IMU time
    // stamp (versus just sampling current host time.)
	
    // imu->micros &= 0xffffff; // 24 bits = 16.7 microseconds roll over
	
    double imu_remote_sec = (double)imu->micros / 1000000.0;
    double diff = imu_timestamp - imu_remote_sec;
    if ( last_imu_micros > imu->micros ) {
        events->log("Aura4", "micros() rolled over\n");
        imu_offset.reset();
    }
    imu_offset.update(imu_remote_sec, diff);
    double fit_diff = imu_offset.get_value(imu_remote_sec);
    // printf("fit_diff = %.6f  diff = %.6f  ts = %.6f\n",
    //        fit_diff, diff, imu_remote_sec + fit_diff );

    last_imu_micros = imu->micros;
	
    imu_node.setDouble( "timestamp", imu_remote_sec + fit_diff );
    imu_node.setLong( "imu->micros", imu->micros );
    imu_node.setDouble( "imu_sec", (double)imu->micros / 1000000.0 );
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

    return true;
}


bool Aura4_t::parse( uint8_t pkt_id, uint8_t pkt_len, uint8_t *payload ) {
    bool new_data = false;

    if ( pkt_id == message::command_ack_id ) {
        message::command_ack_t ack;
        ack.unpack(payload, pkt_len);
	if ( pkt_len == ack.len ) {
            last_ack_id = ack.command_id;
	    last_ack_subid = ack.subcommand_id;
            info("Received ACK = %d %d", ack.command_id, ack.subcommand_id);
	} else {
	    printf("Aura4: packet size mismatch in ACK\n");
	}
    } else if ( pkt_id == message::pilot_id ) {
        message::pilot_t pilot;
        pilot.unpack(payload, pkt_len);
	if ( pkt_len == pilot.len ) {
            update_pilot( &pilot );
	    pilot_packet_counter++;
	    aura4_node.setLong( "pilot_packet_count", pilot_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in pilot input packet");
	}
    } else if ( pkt_id == message::imu_raw_id ) {
        message::imu_raw_t imu;
        imu.unpack(payload, pkt_len);
	if ( pkt_len == imu.len ) {
            update_imu(&imu);
	    imu_packet_counter++;
	    aura4_node.setLong( "imu_packet_count", imu_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in imu packet");
	}
    } else if ( pkt_id == message::aura_nav_pvt_id ) {
        message::aura_nav_pvt_t nav_pvt;
        nav_pvt.unpack(payload, pkt_len);
	if ( pkt_len == nav_pvt.len ) {
            update_gps(&nav_pvt);
	    gps_packet_counter++;
	    aura4_node.setLong( "gps_packet_count", gps_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in gps packet");
            info("got %d, expected %d", pkt_len, nav_pvt.len);
	}
    } else if ( pkt_id == message::ekf_id ) {
        // ekf message, just toss silently for the moment
    } else if ( pkt_id == message::airdata_id ) {
        message::airdata_t airdata;
        airdata.unpack(payload, pkt_len);
	if ( pkt_len == airdata.len ) {
            update_airdata(&airdata);
	    airdata_packet_counter++;
	    aura4_node.setLong( "airdata_packet_count", airdata_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in airdata packet");
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
            info("packet size mismatch in power packet");
	}
    } else if ( pkt_id == message::status_id ) {
        message::status_t msg;
        msg.unpack(payload, pkt_len);
	if ( pkt_len == msg.len ) {
	    aura4_node.setLong( "serial_number", msg.serial_number );
	    aura4_node.setLong( "firmware_rev", msg.firmware_rev );
	    aura4_node.setLong( "master_hz", msg.master_hz );
	    aura4_node.setLong( "baud_rate", msg.baud );
	    aura4_node.setLong( "byte_rate_sec", msg.byte_rate );
 
	    if ( first_status_message ) {
		// log the data to events.txt
		first_status_message = false;
		char buf[128];
		snprintf( buf, 32, "Serial Number = %d", msg.serial_number );
		events->log("Aura4", buf );
		snprintf( buf, 32, "Firmware Revision = %d", msg.firmware_rev );
		events->log("Aura4", buf );
		snprintf( buf, 32, "Master Hz = %d", msg.master_hz );
		events->log("Aura4", buf );
		snprintf( buf, 32, "Baud Rate = %d", msg.baud );
		events->log("Aura4", buf );
	    }
	} else {
            info("packet size mismatch in status packet");
	}
    } else {
        info("unknown packet id = %d", pkt_id);
    }

    return new_data;
}


bool Aura4_t::wait_for_ack(uint8_t id) {
    double timeout = 0.5;
    double start_time = get_Time();
    last_ack_id = 0;
    while ( (last_ack_id != id) ) {
	if ( serial.update() ) {
            parse( serial.pkt_id, serial.pkt_len, serial.payload );
        }
	if ( get_Time() > start_time + timeout ) {
            info("timeout waiting for ack...");
	    return false;
	}
    }
    return true;
}


bool Aura4_t::write_config_master() {
    config_master.pack();
    serial.write_packet( config_master.id, config_master.payload, config_master.len );
    return wait_for_ack(config_master.id);
}

bool Aura4_t::write_config_imu() {
    config_imu.pack();
    serial.write_packet( config_imu.id, config_imu.payload, config_imu.len );
    return wait_for_ack(config_imu.id);
}

bool Aura4_t::write_config_mixer() {
    config_mixer.pack();
    serial.write_packet( config_mixer.id, config_mixer.payload,
                         config_mixer.len );
    return wait_for_ack(config_mixer.id);
}

bool Aura4_t::write_config_pwm() {
    config_pwm.pack();
    serial.write_packet( config_pwm.id, config_pwm.payload,
                  config_pwm.len );
    return wait_for_ack(config_pwm.id);
}

bool Aura4_t::write_config_airdata() {
    config_airdata.pack();
    serial.write_packet( config_airdata.id, config_airdata.payload,
                  config_airdata.len );
    return wait_for_ack(config_airdata.id);
}

bool Aura4_t::write_config_led() {
    config_led.pack();
    serial.write_packet( config_led.id, config_led.payload, config_led.len );
    return wait_for_ack(config_led.id);
}

bool Aura4_t::write_config_power() {
    config_power.pack();
    serial.write_packet( config_power.id, config_power.payload,
                  config_power.len );
    return wait_for_ack(config_power.id);
}

bool Aura4_t::write_config_stab() {
    config_stab.pack();
    serial.write_packet( config_stab.id, config_stab.payload,
                  config_stab.len );
    return wait_for_ack(config_stab.id);
}

bool Aura4_t::write_command_zero_gyros() {
    message::command_zero_gyros_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

bool Aura4_t::write_command_reset_ekf() {
    message::command_reset_ekf_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

bool Aura4_t::write_command_cycle_inceptors() {
    message::command_cycle_inceptors_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

// master board selector defaults
void Aura4_t::master_defaults() {
    config_master.board = 0;
}

// Setup imu defaults:
// Marmot v1 has mpu9250 on SPI CS line 24
// Aura v2 has mpu9250 on I2C Addr 0x68
void Aura4_t::imu_setup_defaults() {
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
void Aura4_t::pwm_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
         config_pwm.pwm_hz[i] = 50;    
         config_pwm.act_gain[i] = 1.0;
    }
}

// reset airdata to startup defaults
void Aura4_t::airdata_defaults() {
    config_airdata.barometer = 0;
    config_airdata.pitot = 0;
    config_airdata.swift_baro_addr = 0;
    config_airdata.swift_pitot_addr = 0;
}

// reset sas parameters to startup defaults
void Aura4_t::stability_defaults() {
    config_stab.sas_rollaxis = false;
    config_stab.sas_pitchaxis = false;
    config_stab.sas_yawaxis = false;
    config_stab.sas_tune = false;

    config_stab.sas_rollgain = 0.0;
    config_stab.sas_pitchgain = 0.0;
    config_stab.sas_yawgain = 0.0;
    config_stab.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
void Aura4_t::mixer_defaults() {
    config_mixer.mix_autocoord = false;
    config_mixer.mix_throttle_trim = false;
    config_mixer.mix_flap_trim = false;
    config_mixer.mix_elevon = false;
    config_mixer.mix_flaperon = false;
    config_mixer.mix_vtail = false;
    config_mixer.mix_diff_thrust = false;

    config_mixer.mix_Gac = 0.5;       // aileron gain for autocoordination
    config_mixer.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config_mixer.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config_mixer.mix_Gea = 1.0;       // aileron gain for elevons
    config_mixer.mix_Gee = 1.0;       // elevator gain for elevons
    config_mixer.mix_Gfa = 1.0;       // aileron gain for flaperons
    config_mixer.mix_Gff = 1.0;       // flaps gain for flaperons
    config_mixer.mix_Gve = 1.0;       // elevator gain for vtail
    config_mixer.mix_Gvr = 1.0;       // rudder gain for vtail
    config_mixer.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config_mixer.mix_Gtr = 0.1;       // rudder gain for diff thrust
};

void Aura4_t::power_defaults() {
    config_power.have_attopilot = false;
}

void Aura4_t::led_defaults() {
    config_led.pin = 0;
}

// send a full configuration to Aura4 and return true only when all
// parameters are acknowledged.
bool Aura4_t::send_config() {
    info("building config structure.");

    vector<string> children;

    // set all parameters to defaults
    master_defaults();
    imu_setup_defaults();
    pwm_defaults();
    airdata_defaults();
    mixer_defaults();
    power_defaults();
    led_defaults();
    stability_defaults();

    int count;

    pyPropertyNode board_node = aura4_config.getChild("board", true);
    string board = aura4_config.getString("board");
    if ( board == "marmot_v1" ) {
        config_master.board = 0;
    } else if ( board == "aura_v2" ) {
        config_master.board = 1;
    } else {
        printf("Warning: no valid PWM pin layout defined.\n");
    }

    pyPropertyNode power_node = aura4_config.getChild("power", true);
    if ( power_node.hasChild("have_attopilot") ) {
        config_power.have_attopilot = power_node.getBool("have_attopilot");
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
        config_pwm.pwm_hz[i] = pwm_node.getLong("channel", i);
        info("pwm_hz[%d] = %d", i, config_pwm.pwm_hz[i]);
    }
    pyPropertyNode gain_node
	= pyGetNode("/config/actuators/actuator/pwm_rates", true);
    count = gain_node.getLen("channel");
    for ( int i = 0; i < count; i++ ) {
        config_pwm.act_gain[i] = gain_node.getDouble("channel", i);
        info("act_gain[%d] = %.2f", i, config_pwm.act_gain[i]);
    }

    pyPropertyNode mixer_node
	= pyGetNode("/config/actuators/actuator/mixing", true);
    count = mixer_node.getLen("mix");
    if ( count ) {
	for ( int i = 0; i < count; i++ ) {
	    string mode = "";
	    bool enable = false;
	    float gain1 = 0.0;
	    float gain2 = 0.0;
	    pyPropertyNode mix_node = mixer_node.getChild("mix", i, true);
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
                    config_mixer.mix_autocoord = enable;
                    config_mixer.mix_Gac = gain1;
		} else if ( mode == "throttle_trim" ) {
                    config_mixer.mix_throttle_trim = enable;
                    config_mixer.mix_Get = gain1;
		} else if ( mode == "flap_trim" ) {
                    config_mixer.mix_flap_trim = enable;
                    config_mixer.mix_Gef = gain1;
		} else if ( mode == "elevon" ) {
                    config_mixer.mix_elevon = enable;
                    config_mixer.mix_Gea = gain1;
                    config_mixer.mix_Gee = gain2;
		} else if ( mode == "flaperon" ) {
                    config_mixer.mix_flaperon = enable;
                    config_mixer.mix_Gfa = gain1;
                    config_mixer.mix_Gff = gain2;
		} else if ( mode == "vtail" ) {
                    config_mixer.mix_vtail = enable;
                    config_mixer.mix_Gve = gain1;
                    config_mixer.mix_Gvr = gain2;
		} else if ( mode == "diff_thrust" ) {
                    config_mixer.mix_diff_thrust = enable;
                    config_mixer.mix_Gtt = gain1;
                    config_mixer.mix_Gtr = gain2;
		}
	    }
            info("mix: %s %d %.2f %.2f", mode.c_str(), enable, gain1, gain2);
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
                        config_stab.sas_rollaxis = enable;
                        config_stab.sas_rollgain = gain;
		    } else if ( mode == "pitch" ) {
                        config_stab.sas_pitchaxis = enable;
                        config_stab.sas_pitchgain = gain;
		    } else if ( mode == "yaw" ) {
                        config_stab.sas_yawaxis = enable;
                        config_stab.sas_yawgain = gain;
		    }
		}
                info("sas: %s %d %.2f", mode.c_str(), enable, gain);
	    }
	} else if ( children[i] == "pilot_tune" ) {
	    pyPropertyNode sas_section = sas_node.getChild("pilot_tune");
	    if ( sas_section.hasChild("enable") ) {
		config_stab.sas_tune = sas_section.getBool("enable");
	    }
            info("sas global tune %d", config_stab.sas_tune);
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
    
    if ( aura4_config.hasChild("led_pin") ) {
        config_led.pin = aura4_config.getLong("led_pin");
    }

    info("transmitting master config ...");
    if ( !write_config_master() ) {
        return false;
    }

    info("transmitting imu config ...");
    if ( !write_config_imu() ) {
        return false;
    }

    info("transmitting pwm config ...");
    if ( !write_config_pwm() ) {
        return false;
    }

    printf("transmitting mixer config ...");
    if ( !write_config_mixer() ) {
        return false;
    }

    info("transmitting airdata config ...");
    if ( !write_config_airdata() ) {
        return false;
    }

    info("transmitting power config ...");
    if ( !write_config_power() ) {
        return false;
    }

    info("transmitting led config ...");
    if ( !write_config_led() ) {
        return false;
    }

    info("transmitting stability damping config ...");
    if ( !write_config_stab() ) {
        return false;
    }

    info("send_config() finished");

    return true;
}

// Read Aura4 packets using IMU packet as the main timing reference.
// Returns the dt from the IMU perspective, not the localhost
// perspective.  This should generally be far more accurate and
// consistent.
float Aura4_t::read() {
    // read packets until we receive an IMU packet and the uart buffer
    // is mostly empty.  The IMU packet (combined with being caught up
    // reading the uart buffer is our signal to run an interation of
    // the main loop.
    double last_time = imu_node.getDouble( "timestamp" );

    // try sending the configuration if not yet successful
    if ( !configuration_sent ) {
	configuration_sent = send_config();
    }
    
    while ( true ) {
        if ( serial.update() ) {
            parse( serial.pkt_id, serial.pkt_len, serial.payload );
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
    aura4_node.setLong("parse_errors", parse_errors);
    aura4_node.setLong("skipped_frames", skipped_frames);

    // relay optional zero gyros command back to FMU upon request
    string command = aura4_node.getString( "command" );
    if ( command.length() ) {
        if ( command == "zero_gyros" ) {
            if ( write_command_zero_gyros() ) {
                aura4_node.setString( "command", "" );
                aura4_node.setString( "command_result",
                                      "success: " + command );
            }
        } else {
            // unknown command
            aura4_node.setString( "command", "" );
            aura4_node.setString( "command_result",
                                  "unknown command: " + command );
        }
    }
    
    double cur_time = imu_node.getDouble( "timestamp" );
    float dt = cur_time - last_time;
    return dt;
}


bool Aura4_t::update_gps( message::aura_nav_pvt_t *nav_pvt ) {
    gps_node.setDouble( "timestamp", get_Time() );
    gps_node.setLong( "year", nav_pvt->year );
    gps_node.setLong( "month", nav_pvt->month );
    gps_node.setLong( "day", nav_pvt->day );
    gps_node.setLong( "hour", nav_pvt->hour );
    gps_node.setLong( "min", nav_pvt->min );
    gps_node.setLong( "sec", nav_pvt->sec );
    gps_node.setDouble( "latitude_deg", nav_pvt->lat / 10000000.0 );
    gps_node.setDouble( "longitude_deg", nav_pvt->lon / 10000000.0 );
    gps_node.setDouble( "altitude_m", nav_pvt->hMSL / 1000.0 );
    gps_node.setDouble( "horiz_accuracy_m", nav_pvt->hAcc / 1000.0 );
    gps_node.setDouble( "vert_accuracy_m", nav_pvt->vAcc / 1000.0 );
    gps_node.setDouble( "vn_ms", nav_pvt->velN / 1000.0 );
    gps_node.setDouble( "ve_ms", nav_pvt->velE / 1000.0 );
    gps_node.setDouble( "vd_ms", nav_pvt->velD / 1000.0 );
    gps_node.setLong( "satellites", nav_pvt->numSV);
    gps_node.setDouble( "pdop", nav_pvt->pDOP / 100.0 );
    gps_node.setLong( "fixType", nav_pvt->fixType );
    // backwards compatibility
    if ( nav_pvt->fixType == 0 ) {
        gps_node.setLong( "status", 0 );
    } else if ( nav_pvt->fixType == 1 || nav_pvt->fixType == 2 ) {
        gps_node.setLong( "status", 1 );
    } else if ( nav_pvt->fixType == 3 ) {
        gps_node.setLong( "status", 2 );
    }
    struct tm gps_time;
    gps_time.tm_sec = nav_pvt->sec;
    gps_time.tm_min = nav_pvt->min;
    gps_time.tm_hour = nav_pvt->hour;
    gps_time.tm_mday = nav_pvt->day;
    gps_time.tm_mon = nav_pvt->month - 1;
    gps_time.tm_year = nav_pvt->year - 1900;
    double unix_sec = (double)mktime( &gps_time ) - timezone;
    unix_sec += nav_pvt->nano / 1000000000.0;
    gps_node.setDouble( "unix_time_sec", unix_sec );
    return true;
}


bool Aura4_t::update_airdata( message::airdata_t *airdata ) {
    bool fresh_data = false;

    float pitot_butter = pitot_filter.update(airdata->ext_diff_press_pa);
        
    if ( ! airspeed_inited ) {
        if ( airspeed_zero_start_time > 0.0 ) {
            pitot_sum += airdata->ext_diff_press_pa;
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
        if ( imu_timestamp > airspeed_zero_start_time + 10.0 ) {
            //printf("pitot_offset = %.2f\n", pitot_offset);
            airspeed_inited = true;
        }
    }

    airdata_node.setDouble( "timestamp", imu_timestamp );

    // basic pressure to airspeed formula: v = sqrt((2/p) * q)
    // where v = velocity, q = dynamic pressure (pitot tube sensor
    // value), and p = air density.

    // if p is specified in kg/m^3 (value = 1.225) and if q is
    // specified in Pa (N/m^2) where 1 psi == 6900 Pa, then the
    // velocity will be in meters per second.

    // The MPXV5004DP has a full scale span of 3.9V, Maximum
    // pressure reading is 0.57psi (4000Pa)

    // Example (Aura4): With a 10bit ADC (Aura4) we record a value
    // of 230 (0-1024) at zero velocity.  The sensor saturates at
    // a value of about 1017 (4000psi).  Thus:

    // Pa = (ADC - 230) * 5.083
    // Airspeed(mps) = sqrt( (2/1.225) * Pa )

    // This yields a theoretical maximum speed sensor reading of
    // about 81mps (156 kts)

    // choose between using raw pitot value or filtered pitot value
    // float pitot = airdata->diff_pres_pa;
    float pitot = pitot_butter;
	
    float Pa = (pitot - pitot_offset);
    if ( Pa < 0.0 ) { Pa = 0.0; } // avoid sqrt(neg_number) situation
    float airspeed_mps = sqrt( 2*Pa / 1.225 ) * pitot_calibrate;
    float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
    airdata_node.setDouble( "airspeed_mps", airspeed_mps );
    airdata_node.setDouble( "airspeed_kt", airspeed_kt );
    airdata_node.setDouble( "temp_C", airdata->ext_temp_C );

    // publish sensor values
    airdata_node.setDouble( "pressure_mbar", airdata->baro_press_pa / 100.0 );
    airdata_node.setDouble( "bme_temp_C", airdata->baro_temp_C );
    airdata_node.setDouble( "humidity", airdata->baro_hum );
    airdata_node.setDouble( "diff_pressure_pa", airdata->ext_diff_press_pa );
    airdata_node.setDouble( "ext_static_press_pa", airdata->ext_static_press_pa );
    airdata_node.setLong( "error_count", airdata->error_count );

    fresh_data = true;

    return fresh_data;
}


// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void Aura4_t::airdata_zero_airspeed() {
    airspeed_inited = false;
    airspeed_zero_start_time = 0.0;
}


bool Aura4_t::update_pilot( message::pilot_t *pilot ) {
    float val;

    pilot_node.setDouble( "timestamp", get_Time() );

    for ( int i = 0; i < message::sbus_channels; i++ ) {
	val = pilot->channel[i];
	pilot_node.setDouble( pilot_mapping[i].c_str(), val );
	pilot_node.setDouble( "channel", i, val );
    }

    // sbus ch17 (channel[16])
    if ( pilot->flags & 0x01 ) {
        pilot_node.setDouble( "channel", 16, 1.0 );
    } else {
        pilot_node.setDouble( "channel", 16, 0.0 );
    }
    // sbus ch18 (channel[17])
    if ( pilot->flags & (1 << 1) ) {
        pilot_node.setDouble( "channel", 17, 1.0 );
    } else {
        pilot_node.setDouble( "channel", 17, 0.0 );
    }
    if ( pilot->flags & (1 << 2) ) {
        pilot_node.setBool( "frame_lost", true );
    } else {
        pilot_node.setBool( "frame_lost", false );
    }
    if ( pilot->flags & (1 << 3) ) {
        pilot_node.setBool( "fail_safe", true );
    } else {
        pilot_node.setBool( "fail_safe", false );
    }

    return true;
}


bool Aura4_t::update_actuators() {
    if ( !configuration_sent ) {
        // send configuration
	configuration_sent = send_config();
    } else {
        // send actuator commands to Aura4 servo subsystem
        if ( message::ap_channels == 6 ) {
            message::command_inceptors_t act;
            act.channel[0] = act_node.getDouble("throttle");
            act.channel[1] = act_node.getDouble("aileron");
            act.channel[2] = act_node.getDouble("elevator");
            act.channel[3] = act_node.getDouble("rudder");
            act.channel[4] = act_node.getDouble("flaps");
            act.channel[5] = act_node.getDouble("gear");
            act.pack();
            serial.write_packet( act.id, act.payload, act.len );
        }
    }
    return true;
}


void Aura4_t::close() {
    serial.close();
}
