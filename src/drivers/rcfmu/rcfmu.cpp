//
// FILE: rcfmu.cpp
// DESCRIPTION: interact with rcfmu FMU
//

// TODO:
// - (for now ok) straighten out what I'm doing with imu timestamp dance
// - (for now ok) straighten out what I'm doing with skipped frames
// - (for now ok) gps age?
// - (ok) send ekf config
// - (ok) figure out how to deal with accel/mag calibration if ekf is remote 
// - (ok) parse ekf packet
// - (ok) write actuators
// - (ok--for now) deal with how to arbitrate output path enumeration in property tree
// - (ok) need to log cal and nocal imu values

#include <props2.h>

#include <stdarg.h>
#include <stdlib.h>             // exit()
#include <time.h>               // gmtime()
#include <unistd.h>             // sleep()

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

//#include "init/globals.h"
#include "util/props_helper.h"
#include "util/timing.h"

#include "rcfmu.h"

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

void rcfmu_t::info( const char *format, ... ) {
    if ( verbose ) {
        printf("rcfmu: ");
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\n");
    }
}

void rcfmu_t::hard_fail( const char *format, ... ) {
    printf("rcfmu hard error: ");
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
    printf("Cannot continue.");
    exit(-1);
}

void rcfmu_t::init( PropertyNode *config ) {
    PropertyNode comms_node("/comms");
    verbose = comms_node.getBool("display_on");
    
    // bind main property nodes
    rcfmu_node = PropertyNode( "/sensors/rcfmu" );
    active_node = PropertyNode("/task/route/active");

    ap_node = PropertyNode("/autopilot");
    circle_node = PropertyNode("/task/circle/active");
    home_node = PropertyNode("/task/home");
    pos_node = PropertyNode("/position");
    power_node = PropertyNode( "/sensors/power" );
    remote_link_node = PropertyNode("/comms/remote_link");
    route_node = PropertyNode("/task/route");

    status_node = PropertyNode( "/status" );
    switches_node = PropertyNode( "/switches" );
    targets_node = PropertyNode("/autopilot/targets");
    task_node = PropertyNode("/task");

    rcfmu_config = *config;

    printf("rcfmu driver init(): event logging broken!\n");
    printf("rcfmu driver init(): write imu calibration broken!\n");
    
    if ( true ) {               // fixme: move or delete
        PropertyNode specs_node( "/config/specs" );
        if ( specs_node.hasChild("battery_cells") ) {
            battery_cells = specs_node.getInt("battery_cells");
        }
        if ( battery_cells < 1 ) { battery_cells = 1; }
    }

    if ( config->hasChild("board") ) {
        PropertyNode board_config = config->getChild( "board" );
        open( &board_config );
    } else {
        hard_fail("no board defined\n");
    }
    
    if ( config->hasChild("airdata") ) {
        PropertyNode airdata_config = config->getChild( "airdata" );
        init_airdata( &airdata_config );
    } else {
        hard_fail("no airdata configuration\n");
    }
    
    if ( config->hasChild("ekf") ) {
        PropertyNode ekf_config = config->getChild( "ekf" );
        init_ekf( &ekf_config );
    } else {
        hard_fail("no ekf configuration\n");
    }
    
    if ( config->hasChild("gps") ) {
        PropertyNode gps_config = config->getChild( "gps" );
        init_gps( &gps_config );
    } else {
        hard_fail("no gps configuration\n");
    }
    
    if ( config->hasChild("imu") ) {
        PropertyNode imu_config = config->getChild( "imu" );
        init_imu( &imu_config );
    } else {
        hard_fail("no imu configuration\n");
    }

    if ( config->hasChild("pilot_input") ) {
        PropertyNode pilot_config = config->getChild( "pilot_input" );
        init_pilot( &pilot_config );
    } else {
        hard_fail("no pilot configuration\n");
    }

    // fixme: should we have a config section to trigger this
    init_actuators(NULL);
    
    sleep(1);
}

bool rcfmu_t::open( PropertyNode *config ) {
    if ( config->hasChild("device") ) {
	device_name = config->getString("device");
    }
    if ( config->hasChild("baud") ) {
       baud = config->getInt("baud");
    }
    
    if ( serial.is_open() ) {
        info("device already open");
        return true;
    } else {
        info("device on %s @ %d baud", device_name.c_str(), baud);
    }

    bool result = serial.open( baud, device_name.c_str() );
    if ( !result ) {
        hard_fail("Error opening serial link to rcfmu device");
    }

    return true;
}

void rcfmu_t::init_airdata( PropertyNode *config ) {
    string output_path = get_next_path("/sensors", "airdata", true);
    airdata_node = PropertyNode( output_path.c_str() );
    if ( config->hasChild("pitot_calibrate_factor") ) {
        pitot_calibrate = config->getDouble("pitot_calibrate_factor");
    }
}

void rcfmu_t::init_ekf( PropertyNode *config ) {
    // fixme: how do we configure parameters?  saved onboard the
    // rc-fmu-ap for now?
    if ( config->hasChild("select") ) {
        string val = config->getString("select");
        if ( val == "nav15" or val == "nav15_mag" ) {
            string output_path = get_next_path("/filters", "filter", true);
            ekf_node = PropertyNode( output_path.c_str() );
        } else if ( val == "none" ) {
            // FIXME
            ekf_node = rcfmu_node.getChild( "aura4_ekf_disabled" );
        } else {
            hard_fail("bad nav/ekf selection: %s", val.c_str());
        }
    } else {
        // FIXME
        ekf_node = rcfmu_node.getChild( "aura4_ekf_disabled" );
    }
}

void rcfmu_t::init_gps( PropertyNode *config ) {
    string output_path = get_next_path("/sensors", "gps", true);
    gps_node = PropertyNode( output_path.c_str() );
}

void rcfmu_t::init_imu( PropertyNode *config ) {
    string output_path = get_next_path("/sensors", "imu", true);
    imu_node = PropertyNode( output_path.c_str() );
}

void rcfmu_t::init_pilot( PropertyNode *config ) {
    pilot_node = PropertyNode("/sensors/pilot_input");
    if ( config->hasChild("channel") ) {
	for ( int i = 0; i < rcfmu_message::sbus_channels; i++ ) {
	    pilot_mapping[i] = config->getString("channel", i);
            if ( pilot_mapping[i] != "" ) {
                printf("pilot input: channel %d maps to %s\n", i, pilot_mapping[i].c_str());
            }
	}
    }
}

void rcfmu_t::init_actuators( PropertyNode *config ) {
    act_node = PropertyNode( "/actuators" );
}

bool rcfmu_t::update_imu( rc_message::imu_v6_t *imu ) {
    imu_timestamp = get_Time();
    imu->msg2props(imu_node);

    // timestamp dance: this is a little jig that I do to make a
    // more consistent time stamp that still is in the host
    // reference frame.  Assumes the rcfmu clock drifts relative to
    // host clock.  Assumes the rcfmu imu stamp dt is very stable.
    // Assumes the host system is not-real time and there may be
    // momentary external disruptions to execution. The code
    // estimates the error (difference) between rcfmu clock and
    // host clock.  Then builds a real time linear fit of rcfmu
    // clock versus difference with the host.  This linear fit is
    // used to estimate the current error (smoothly), add that to
    // the rcfmu clock and derive a more regular/stable IMU time
    // stamp (versus just sampling current host time.)
	
    // imu->micros &= 0xffffff; // 24 bits = 16.7 microseconds roll over
    // imu->micros &= 0xffffff; // 24 bits = 16.7 microseconds roll over
	
    double imu_remote_sec = (double)imu->millis / 1000.0;
    double diff = imu_timestamp - imu_remote_sec;
    if ( last_imu_millis > imu->millis ) {
        // FIXME: events->log("rcfmu", "millis() rolled over\n");
        imu_offset.reset();
    }
    imu_offset.update(imu_remote_sec, diff);
    double fit_diff = imu_offset.get_value(imu_remote_sec);
    // printf("fit_diff = %.6f  diff = %.6f  ts = %.6f\n",
    //        fit_diff, diff, imu_remote_sec + fit_diff );

    last_imu_millis = imu->millis;
	
    imu_node.setDouble( "timestamp", imu_remote_sec + fit_diff );
    imu_node.setDouble( "imu_sec", (double)imu->millis / 1000.0 );

    return true;
}


bool rcfmu_t::parse( uint8_t pkt_id, uint16_t pkt_len, uint8_t *payload ) {
    bool new_data = false;
    if ( pkt_id == rc_message::ack_v1_id ) {
        rc_message::ack_v1_t ack;
        ack.unpack(payload, pkt_len);
	if ( pkt_len == ack.len ) {
            last_ack_sequence = ack.sequence_num;
	    last_ack_result = ack.result;
            info("Received ACK = sequence: %d  result: %d", ack.sequence_num, ack.result);
	} else {
	    printf("rcfmu: packet size mismatch in ACK\n");
	}
    } else if ( pkt_id == rc_message::airdata_v8_id ) {
        rc_message::airdata_v8_t airdata;
        airdata.unpack(payload, pkt_len);
	if ( pkt_len == airdata.len ) {
            update_airdata(&airdata);
	    airdata_packet_counter++;
	    rcfmu_node.setInt( "airdata_packet_count", airdata_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in airdata packet");
	}
    } else if ( pkt_id == rc_message::nav_v6_id ) {
        rc_message::nav_v6_t nav_msg;
        nav_msg.unpack(payload, pkt_len);
        if ( pkt_len == nav_msg.len ) {
            update_nav(&nav_msg);
            nav_packet_counter++;
            rcfmu_node.setInt( "nav_packet_count", nav_packet_counter );
            new_data = true;
        } else {
            info("packet size mismatch in nav packet");
        }
    } else if ( pkt_id == rc_message::nav_metrics_v6_id ) {
        rc_message::nav_metrics_v6_t metrics_msg;
        metrics_msg.unpack(payload, pkt_len);
        if ( pkt_len == metrics_msg.len ) {
            metrics_msg.msg2props(ekf_node);
            nav_metrics_packet_counter++;
            rcfmu_node.setInt( "nav_metrics_packet_count",
                               nav_metrics_packet_counter );
            new_data = true;
        } else {
            info("packet size mismatch in nav packet");
        }
    } else if ( pkt_id == rc_message::gps_v5_id ) {
        rc_message::gps_v5_t gps_msg;
        gps_msg.unpack(payload, pkt_len);
	if ( pkt_len == gps_msg.len ) {
            update_gps(&gps_msg);
	    gps_packet_counter++;
            // fixme: node name
	    rcfmu_node.setInt( "gps_packet_count", gps_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in gps packet");
            info("got %d, expected %d", pkt_len, gps_msg.len);
	}
    } else if ( pkt_id == rc_message::imu_v6_id ) {
        rc_message::imu_v6_t imu_msg;
        imu_msg.unpack(payload, pkt_len);
	if ( pkt_len == imu_msg.len ) {
            update_imu(&imu_msg);
	    imu_packet_counter++;
	    rcfmu_node.setInt( "imu_packet_count",
                                imu_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in imu packet");
	}
    } else if ( pkt_id == rc_message::pilot_v4_id ) {
        rc_message::pilot_v4_t pilot_msg;
        pilot_msg.unpack(payload, pkt_len);
	if ( pkt_len == pilot_msg.len ) {
            pilot_msg.msg2props(pilot_node);
            pilot_node.setDouble("timestamp", pilot_msg.millis / 1000.0);
            switches_node.setBool("master-switch", pilot_msg.master_switch);
            switches_node.setBool("throttle-safety", pilot_msg.throttle_safety);
	    pilot_packet_counter++;
	    rcfmu_node.setInt( "pilot_packet_count", pilot_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in pilot input packet");
	}
    } else if ( pkt_id == rc_message::power_v1_id ) {
        rc_message::power_v1_t power;
        power.unpack(payload, pkt_len);
	if ( pkt_len == power.len ) {
            power.msg2props(power_node);

            // FIXME: filter on sending side!?!
            
            // we anticipate a 0.01 sec dt value
            //int_main_vcc_filt.update((float)power.int_main_v, 0.01);
            //ext_main_vcc_filt.update((float)power.ext_main_v, 0.01);
            //avionics_vcc_filt.update((float)power.avionics_v, 0.01);

            //power_node.setDouble( "main_vcc", int_main_vcc_filt.get_value() );
            //power_node.setDouble( "ext_main_vcc", ext_main_vcc_filt.get_value() );
            //power_node.setDouble( "avionics_vcc", avionics_vcc_filt.get_value() );

            // FIXME: cell volts computed on sending side
            // float cell_volt = power.int_main_vcc_filt.get_value() / (float)battery_cells;
            // float ext_cell_volt = ext_main_vcc_filt.get_value() / (float)battery_cells;
            //power_node.setDouble( "cell_vcc", cell_volt );
            //power_node.setDouble( "ext_cell_vcc", ext_cell_volt );
            //power_node.setDouble( "main_amps", (float)power.ext_main_amp);
	} else {
            info("packet size mismatch in power packet");
	}
    } else if ( pkt_id == rc_message::status_v7_id ) {
        rc_message::status_v7_t msg;
        msg.unpack(payload, pkt_len);
        if ( pkt_len == msg.len ) {
            msg.msg2props(rcfmu_node);
	} else {
            info("packet size mismatch in status packet");
	}
    } else {
        info("unknown packet id = %d", pkt_id);
    }

    return new_data;
}


bool rcfmu_t::wait_for_ack(uint16_t sequence_num) {
    double timeout = 0.5;
    double start_time = get_Time();
    while ( (last_ack_sequence != sequence_num) ) {
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

bool rcfmu_t::write_config_message(int id, uint8_t *payload, int len) {
    serial.write_packet( id, payload, len );
    return wait_for_ack(id);
}

bool rcfmu_t::write_command(string message) {
    rc_message::command_v1_t cmd;
    cmd.sequence_num = command_seq_num;
    cmd.message = message;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    command_seq_num++;
    last_ack_sequence = 0;
    last_ack_result = 0;
    return wait_for_ack(cmd.sequence_num);
}

// Read rcfmu packets using IMU packet as the main timing reference.
// Returns the dt from the IMU perspective, not the localhost
// perspective.  This should generally be far more accurate and
// consistent.
float rcfmu_t::read() {
    // read packets until we receive an IMU packet and the uart buffer
    // is mostly empty.  The IMU packet (combined with being caught up
    // reading the uart buffer is our signal to run an interation of
    // the main loop.
    double last_time = imu_node.getDouble( "timestamp" );

    while ( true ) {
        if ( serial.update() ) {
            parse( serial.pkt_id, serial.pkt_len, serial.payload );
            if ( serial.pkt_id == rc_message::imu_v6_id ) {
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
    rcfmu_node.setInt("parse_errors", serial.parse_errors);
    rcfmu_node.setInt("skipped_frames", skipped_frames);

    // relay optional commands to FMU upon request
    string command = rcfmu_node.getString( "command" );
    if ( command.length() ) {
        if ( write_command(command) ) {
            rcfmu_node.setString( "command_ack", command + ": true" );
            rcfmu_node.setInt( "command_result", last_ack_result );
        } else {
            rcfmu_node.setString( "command_ack", command + ": false" );
            rcfmu_node.setInt( "command_result", last_ack_result );
        }
        rcfmu_node.setString( "command", "" );
    }
    
    double cur_time = imu_node.getDouble( "timestamp" );
    float dt = cur_time - last_time;
    return dt;
}


bool rcfmu_t::update_nav( rc_message::nav_v6_t *nav ) {
    const double R2D = 180.0 / M_PI;
    const double F2M = 0.3048;
    const double M2F = 1 / F2M;

    nav->msg2props(ekf_node);
    ekf_node.setDouble("timestamp", nav->millis / 1000.0);
    ekf_node.setDouble("latitude_deg", nav->latitude_raw / 10000000.0 );
    ekf_node.setDouble("longitude_deg", nav->longitude_raw / 10000000.0);

    // FIXME: move the following to filter_mgr?
    ekf_node.setDouble( "altitude_ft", nav->altitude_m * M2F );
    ekf_node.setDouble( "groundtrack_deg",
                        90 - atan2(nav->vn_mps, nav->ve_mps) * R2D );
    double gs_mps = sqrt(nav->vn_mps * nav->vn_mps + nav->ve_mps * nav->ve_mps);
    ekf_node.setDouble( "groundspeed_ms", gs_mps );
    ekf_node.setDouble( "groundspeed_kt", gs_mps * SG_MPS_TO_KT );
    ekf_node.setDouble( "vertical_speed_fps", -nav->vd_mps * M2F );
    return true;
}

bool rcfmu_t::update_gps( rc_message::gps_v5_t *gps ) {
    gps->msg2props(gps_node);
    gps_node.setDouble("timestamp", gps->millis / 1000.0);
    gps_node.setDouble("unix_time_sec", gps->unix_usec / 1000000.0);
    gps_node.setDouble("latitude_deg", gps->latitude_raw / 10000000.0 );
    gps_node.setDouble("longitude_deg", gps->longitude_raw / 10000000.0);
 
    // generate broken-down time
    struct tm *tm;
    time_t time_sec = gps->unix_usec / 1000000U;
    tm = gmtime(&time_sec);
    gps_node.setInt("year", tm->tm_year + 1900);
    gps_node.setInt("month", tm->tm_mon + 1);
    gps_node.setInt("day", tm->tm_mday);
    gps_node.setInt("hour", tm->tm_hour);
    gps_node.setInt("min", tm->tm_min);
    gps_node.setInt("sec", tm->tm_sec);

    return true;
}


bool rcfmu_t::update_airdata( rc_message::airdata_v8_t *airdata ) {
    bool fresh_data = false;
    airdata->msg2props(airdata_node);
    
    float pitot_butter = pitot_filter.update(airdata->diff_press_pa);
        
    if ( ! airspeed_inited ) {
        if ( airspeed_zero_start_time > 0.0 ) {
            pitot_sum += airdata->diff_press_pa;
            pitot_count++;
            pitot_offset = pitot_sum / (double)pitot_count;
            /* printf("a1 raw=%.1f filt=%.1f a1 off=%.1f a1 sum=%.1f a1 count=%d\n",
               analog[0], pitot_filt.get_value(), pitot_offset, pitot_sum,
               pitot_count); */
        } else {
            airspeed_zero_start_time = airdata->millis;
            pitot_sum = 0.0;
            pitot_count = 0;
        }
        if ( airdata->millis > airspeed_zero_start_time + 10.0 * 1000 ) {
            //printf("pitot_offset = %.2f\n", pitot_offset);
            airspeed_inited = true;
        }
    }

    airdata_node.setDouble( "timestamp", airdata->millis / 1000.0 );

    // basic pressure to airspeed formula: v = sqrt((2/p) * q)
    // where v = velocity, q = dynamic pressure (pitot tube sensor
    // value), and p = air density.

    // if p is specified in kg/m^3 (value = 1.225) and if q is
    // specified in Pa (N/m^2) where 1 psi == 6900 Pa, then the
    // velocity will be in meters per second.

    // The MPXV5004DP has a full scale span of 3.9V, Maximum
    // pressure reading is 0.57 psi (4000 Pa)

    // Example (rcfmu): With a 10bit ADC (rcfmu) we record a value
    // of 230 (0-1024) at zero velocity.  The sensor saturates at
    // a value of about 1017 (4000 pa).  Thus:

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

    // publish sensor values
    airdata_node.setDouble( "pressure_mbar", airdata->baro_press_pa / 100.0 );

    fresh_data = true;

    return fresh_data;
}

// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void rcfmu_t::airdata_zero_airspeed() {
    airspeed_inited = false;
    airspeed_zero_start_time = 0.0;
}

void rcfmu_t::write() {
    // send actuator commands to fmu
    rc_message::inceptors_v4_t inceptors;
    inceptors.index = 0;
    inceptors.millis = imu_node.getUInt("millis");
    inceptors.channel[0] = act_node.getDouble("throttle");
    inceptors.channel[1] = act_node.getDouble("aileron");
    inceptors.channel[2] = act_node.getDouble("elevator");
    inceptors.channel[3] = act_node.getDouble("rudder");
    inceptors.channel[4] = act_node.getDouble("flaps");
    inceptors.channel[5] = act_node.getDouble("gear");
    inceptors.pack();
    serial.write_packet( inceptors.id, inceptors.payload, inceptors.len );

    // send autopilot targets to fmu
    rc_message::ap_targets_v1_t ap_msg;
    ap_msg.props2msg(targets_node);
    ap_msg.millis = imu_node.getUInt("millis");
    ap_msg.pack();
    serial.write_packet( ap_msg.id, ap_msg.payload, ap_msg.len );
    
    rc_message::mission_v1_t mission;
    mission.millis = imu_node.getUInt("millis");
    mission.flight_timer = task_node.getDouble("flight_timer");

    mission.task_name = task_node.getString("current_task");
    mission.target_waypoint_idx = route_node.getInt("target_waypoint_idx");

    // Note: task_attribute is an overloaded (uint16_t) field!  There
    // will be a better way figured out sometime in the future.
    mission.task_attribute = 0;
            
    // wp_counter will get incremented externally in the remote_link
    // message sender because each time we send a serial message to
    // the remote ground station is when we want to advance to the
    // next waypoint.
    int counter = remote_link_node.getInt("wp_counter");
    mission.wp_longitude_raw = 0;
    mission.wp_latitude_raw = 0;
    mission.wp_index = 0;
    mission.route_size = active_node.getInt("route_size");
    if ( mission.route_size > 0 and counter < mission.route_size ) {
        mission.wp_index = counter;
        string wp_path = "wpt/" + std::to_string(mission.wp_index);
        PropertyNode wp_node = active_node.getChild(wp_path.c_str());
        mission.wp_longitude_raw = intround(wp_node.getDouble("longitude_deg") * 10000000);
        mission.wp_latitude_raw = intround(wp_node.getDouble("latitude_deg") * 10000000);
    } else if ( counter == mission.route_size ) {
        mission.wp_longitude_raw = intround(circle_node.getDouble("longitude_deg") * 10000000);
        mission.wp_latitude_raw = intround(circle_node.getDouble("latitude_deg") * 10000000);
        mission.wp_index = 65534;
        mission.task_attribute = int(round(circle_node.getDouble("radius_m") * 10));
        if ( mission.task_attribute > 32767 ) {
            mission.task_attribute = 32767;
        }
    } else if ( counter == mission.route_size + 1 ) {
        mission.wp_longitude_raw = intround(home_node.getDouble("longitude_deg") * 10000000);
        mission.wp_latitude_raw = intround(home_node.getDouble("latitude_deg") * 10000000);
        mission.wp_index = 65535;
    }

    mission.pack();
    serial.write_packet( mission.id, mission.payload, mission.len );
}

void rcfmu_t::close() {
    serial.close();
}

void rcfmu_t::command( const char *cmd ) {
    if ( (string)cmd == "airdata_calibrate" ) {
        airdata_zero_airspeed();
    }
}
