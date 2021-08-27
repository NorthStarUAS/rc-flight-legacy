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
    // bind main property nodes
    rcfmu_node = PropertyNode( "/sensors/rcfmu" );
    power_node = PropertyNode( "/sensors/power" );
    status_node = PropertyNode( "/status" );
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
    // pilot_node.setLen("channel", rcfmu_message::sbus_channels, 0.0);
}

void rcfmu_t::init_actuators( PropertyNode *config ) {
    act_node = PropertyNode( "/actuators" );
}

bool rcfmu_t::update_imu( rcfmu_message::imu_t *imu ) {
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

    if ( pkt_id == rcfmu_message::command_ack_id ) {
        rcfmu_message::command_ack_t ack;
        ack.unpack(payload, pkt_len);
	if ( pkt_len == ack.len ) {
            last_ack_id = ack.command_id;
	    last_ack_subid = ack.subcommand_id;
            info("Received ACK = %d %d", ack.command_id, ack.subcommand_id);
	} else {
	    printf("rcfmu: packet size mismatch in ACK\n");
	}
    } else if ( pkt_id == rcfmu_message::airdata_id ) {
        rcfmu_message::airdata_t airdata;
        airdata.unpack(payload, pkt_len);
	if ( pkt_len == airdata.len ) {
            update_airdata(&airdata);
	    airdata_packet_counter++;
	    rcfmu_node.setInt( "airdata_packet_count", airdata_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in airdata packet");
	}
    } else if ( pkt_id == rcfmu_message::ekf_id ) {
        rcfmu_message::ekf_t ekf;
        ekf.unpack(payload, pkt_len);
        if ( pkt_len == ekf.len ) {
            update_ekf(&ekf);
            ekf_packet_counter++;
            rcfmu_node.setInt( "ekf_packet_count", ekf_packet_counter );
            new_data = true;
        } else {
            info("packet size mismatch in ekf packet");
        }
    } else if ( pkt_id == rcfmu_message::gps_id ) {
        rcfmu_message::gps_t gps;
        gps.unpack(payload, pkt_len);
	if ( pkt_len == gps.len ) {
            update_gps(&gps);
	    gps_packet_counter++;
            // fixme: node name
	    rcfmu_node.setInt( "gps_packet_count", gps_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in gps packet");
            info("got %d, expected %d", pkt_len, gps.len);
	}
    } else if ( pkt_id == rcfmu_message::imu_id ) {
        rcfmu_message::imu_t imu;
        imu.unpack(payload, pkt_len);
	if ( pkt_len == imu.len ) {
            update_imu(&imu);
	    imu_packet_counter++;
	    rcfmu_node.setInt( "imu_packet_count",
                                imu_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in imu packet");
	}
    } else if ( pkt_id == rcfmu_message::pilot_id ) {
        rcfmu_message::pilot_t pilot;
        pilot.unpack(payload, pkt_len);
	if ( pkt_len == pilot.len ) {
            update_pilot( &pilot );
	    pilot_packet_counter++;
	    rcfmu_node.setInt( "pilot_packet_count", pilot_packet_counter );
	    new_data = true;
	} else {
            info("packet size mismatch in pilot input packet");
	}
    } else if ( pkt_id == rcfmu_message::power_id ) {
        rcfmu_message::power_t power;
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
            if ( mah_last_time > 0 ) {
                float cur_time = get_Time();
                float dt = cur_time - mah_last_time;
                if ( dt > 0.0 ) {
                    // 0.2777 is 1000/3600 (conversion to milli-amp hours)
                    total_mah += (float)power.ext_main_amp * dt * 0.277777778;
                    power_node.setDouble( "total_mah", total_mah);
                }
                mah_last_time = cur_time;
            } else {
                mah_last_time = get_Time();
            }
	} else {
            info("packet size mismatch in power packet");
	}
    } else if ( pkt_id == rcfmu_message::status_id ) {
        rcfmu_message::status_t msg;
        msg.unpack(payload, pkt_len);
	if ( pkt_len == msg.len ) {
	    rcfmu_node.setInt( "serial_number", msg.serial_number );
	    rcfmu_node.setInt( "firmware_rev", msg.firmware_rev );
	    rcfmu_node.setInt( "master_hz", msg.master_hz );
	    rcfmu_node.setInt( "baud_rate", msg.baud );
	    rcfmu_node.setInt( "byte_rate_sec", msg.byte_rate );
            status_node.setInt( "fmu_timer_misses", msg.timer_misses );

            // FIXME:
	    // if ( first_status_message ) {
	    //     // log the data to events.txt
	    //     first_status_message = false;
	    //     char buf[128];
	    //     snprintf( buf, 32, "Serial Number = %d", msg.serial_number );
	    //     events->log("rcfmu", buf );
	    //     snprintf( buf, 32, "Firmware Revision = %d", msg.firmware_rev );
	    //     events->log("rcfmu", buf );
	    //     snprintf( buf, 32, "Master Hz = %d", msg.master_hz );
	    //     events->log("rcfmu", buf );
	    //     snprintf( buf, 32, "Baud Rate = %d", msg.baud );
	    //     events->log("rcfmu", buf );
	    // }
	} else {
            info("packet size mismatch in status packet");
	}
    } else {
        info("unknown packet id = %d", pkt_id);
    }

    return new_data;
}


bool rcfmu_t::wait_for_ack(uint8_t id) {
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

bool rcfmu_t::write_config_message(int id, uint8_t *payload, int len) {
    serial.write_packet( id, payload, len );
    return wait_for_ack(id);
}

bool rcfmu_t::write_command_zero_gyros() {
    rcfmu_message::command_zero_gyros_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

bool rcfmu_t::write_command_reset_ekf() {
    rcfmu_message::command_reset_ekf_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

bool rcfmu_t::write_command_cycle_inceptors() {
    rcfmu_message::command_cycle_inceptors_t cmd;
    cmd.pack();
    serial.write_packet( cmd.id, cmd.payload, cmd.len );
    return wait_for_ack(cmd.id);
}

// send a full configuration to rcfmu and return true only when all
// parameters are acknowledged.
bool rcfmu_t::send_config() {
    return true;
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

    // try sending the configuration if not yet successful
    if ( !configuration_sent ) {
	configuration_sent = send_config();
    }
    
    while ( true ) {
        if ( serial.update() ) {
            parse( serial.pkt_id, serial.pkt_len, serial.payload );
            if ( serial.pkt_id == rcfmu_message::imu_id ) {
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

    // relay optional zero gyros command back to FMU upon request
    string command = rcfmu_node.getString( "command" );
    if ( command.length() ) {
        if ( command == "zero_gyros" ) {
            if ( write_command_zero_gyros() ) {
                rcfmu_node.setString( "command", "" );
                rcfmu_node.setString( "command_result",
                                      "success: " + command );
            }
        } else if ( command == "reset_ekf" ) {
            if ( write_command_reset_ekf() ) {
                rcfmu_node.setString( "command", "" );
                rcfmu_node.setString( "command_result",
                                      "success: " + command );
            }
        } else {
            // unknown command
            rcfmu_node.setString( "command", "" );
            rcfmu_node.setString( "command_result",
                                  "unknown command: " + command );
        }
    }
    
    double cur_time = imu_node.getDouble( "timestamp" );
    float dt = cur_time - last_time;
    return dt;
}


bool rcfmu_t::update_ekf( rcfmu_message::ekf_t *ekf ) {
    const double R2D = 180 / M_PI;
    const double F2M = 0.3048;
    const double M2F = 1 / F2M;
    // do a little dance to estimate the ekf timestamp in seconds
    int imu_millis = imu_node.getInt("millis");
    int diff_millis = ekf->millis - imu_millis;
    if ( diff_millis < 0 ) { diff_millis = 0; } // don't puke on wraparound
    double timestamp = imu_node.getDouble("timestamp")
        + (float)diff_millis / 1000.0;
    ekf_node.setDouble( "timestamp", timestamp );
    ekf_node.setInt( "ekf_millis", ekf->millis );
    ekf_node.setDouble( "latitude_deg", ekf->lat_rad * R2D );
    ekf_node.setDouble( "longitude_deg", ekf->lon_rad * R2D );
    ekf_node.setDouble( "altitude_m", ekf->altitude_m );
    ekf_node.setDouble( "vn_ms", ekf->vn_ms );
    ekf_node.setDouble( "ve_ms", ekf->ve_ms );
    ekf_node.setDouble( "vd_ms", ekf->vd_ms );
    ekf_node.setDouble( "phi_rad", ekf->phi_rad );
    ekf_node.setDouble( "the_rad", ekf->the_rad );
    ekf_node.setDouble( "psi_rad", ekf->psi_rad );
    ekf_node.setDouble( "roll_deg", ekf->phi_rad * R2D );
    ekf_node.setDouble( "pitch_deg", ekf->the_rad * R2D );
    ekf_node.setDouble( "heading_deg", ekf->psi_rad * R2D );
    ekf_node.setDouble( "p_bias", ekf->p_bias );
    ekf_node.setDouble( "q_bias", ekf->q_bias );
    ekf_node.setDouble( "r_bias", ekf->r_bias );
    ekf_node.setDouble( "ax_bias", ekf->ax_bias );
    ekf_node.setDouble( "ay_bias", ekf->ay_bias );
    ekf_node.setDouble( "az_bias", ekf->az_bias );
    ekf_node.setDouble( "max_pos_cov", ekf->max_pos_cov );
    ekf_node.setDouble( "max_vel_cov", ekf->max_vel_cov );
    ekf_node.setDouble( "max_att_cov", ekf->max_att_cov );
    ekf_node.setInt("status", ekf->status );
    
    /*FIXME:move the following to filter_mgr?*/
    ekf_node.setDouble( "altitude_ft", ekf->altitude_m * M2F );
    ekf_node.setDouble( "groundtrack_deg",
                        90 - atan2(ekf->vn_ms, ekf->ve_ms) * R2D );
    double gs_ms = sqrt(ekf->vn_ms * ekf->vn_ms + ekf->ve_ms * ekf->ve_ms);
    ekf_node.setDouble( "groundspeed_ms", gs_ms );
    ekf_node.setDouble( "groundspeed_kt", gs_ms * SG_MPS_TO_KT );
    ekf_node.setDouble( "vertical_speed_fps", -ekf->vd_ms * M2F );
    return true;
}

bool rcfmu_t::update_gps( rcfmu_message::gps_t *gps ) {
    gps_node.setDouble( "timestamp", get_Time() );
    gps_node.setDouble( "unix_time_usec", gps->unix_usec );
    gps_node.setDouble( "unix_time_sec", (double)gps->unix_usec / 1000000.0 );
    // for ( int i = 0; i < 8; i++ ) {
    //     printf("%02X ", *(uint8_t *)(&(gps->unix_usec) + i));
    // }
    // printf(" %ldf\n", gps->unix_usec);
    gps_node.setInt( "satellites", gps->num_sats );
    gps_node.setInt( "fixType", gps->status );
    gps_node.setDouble( "latitude_deg", gps->latitude_raw / 10000000.0 );
    gps_node.setDouble( "longitude_deg", gps->longitude_raw / 10000000.0 );
    gps_node.setDouble( "altitude_m", gps->altitude_m );
    // fixme: node property units name?
    gps_node.setDouble( "vn_ms", gps->vn_mps );
    gps_node.setDouble( "ve_ms", gps->ve_mps );
    gps_node.setDouble( "vd_ms", gps->vd_mps );
    gps_node.setDouble( "horiz_accuracy_m", gps->hAcc );
    gps_node.setDouble( "vert_accuracy_m", gps->vAcc );
    gps_node.setDouble( "hdop", gps->hdop );
    gps_node.setDouble( "vdop", gps->vdop );
    // backwards compatibility
    if ( gps->status == 0 ) {
        gps_node.setInt( "status", 0 );
    } else if ( gps->status == 1 || gps->status == 2 ) {
        gps_node.setInt( "status", 1 );
    } else if ( gps->status == 3 ) {
        gps_node.setInt( "status", 2 );
    }
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


bool rcfmu_t::update_airdata( rcfmu_message::airdata_t *airdata ) {
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

    // Example (rcfmu): With a 10bit ADC (rcfmu) we record a value
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
    airdata_node.setInt( "error_count", airdata->error_count );

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


bool rcfmu_t::update_pilot( rcfmu_message::pilot_t *pilot ) {
    float val;

    pilot_node.setDouble( "timestamp", get_Time() );

    for ( int i = 0; i < rcfmu_message::sbus_channels; i++ ) {
	val = pilot->channel[i];
        pilot_node.setDouble( "channel", val, i );
        if ( pilot_mapping[i] != "" ) {
            pilot_node.setDouble( pilot_mapping[i].c_str(), val );
        }
    }

    // sbus ch17 (channel[16])
    if ( pilot->flags & 0x01 ) {
        pilot_node.setDouble( "channel", 1.0, 16 );
    } else {
        pilot_node.setDouble( "channel", 0.0, 16 );
    }
    // sbus ch18 (channel[17])
    if ( pilot->flags & (1 << 1) ) {
        pilot_node.setDouble( "channel", 1.0, 17 );
    } else {
        pilot_node.setDouble( "channel", 0.0, 17 );
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


void rcfmu_t::write() {
    // send actuator commands to rcfmu servo subsystem
    if ( rcfmu_message::ap_channels == 6 ) {
        rcfmu_message::command_inceptors_t act;
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


void rcfmu_t::close() {
    serial.close();
}

void rcfmu_t::command( const char *cmd ) {
    if ( (string)cmd == "airdata_calibrate" ) {
        airdata_zero_airspeed();
    }
}
