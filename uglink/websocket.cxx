// \file websocket.cxx
// WebSocket server class.
//
// Copyright (C) 2012  Curtis L. Olson - colson@atiak.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#include <openssl/sha.h>
#include <openssl/md5.h>
#include <sstream>

#include "comms/netBuffer.h"

#include <include/globaldefs.h>
//#include <simgear/structure/commands.hxx>
#include <props/props.hxx>
#include <util/strutils.hxx>

#include "base64.hxx"
#include "command.hxx"
#include "globals.hxx"
#include "websocket.hxx"

using std::stringstream;
using std::ends;

/**
 * Props connection class.
 * This class represents a connection to props client.
 */
class WSChannel : public netBufferChannel
{
    /* websocket hand shaking values */
    bool handshake;
    string ws_root;
    string http_version;
    string connection_mode;
    string host;
    string origin;
    string ws_key;
    string ws_key1; // safari websockets
    string ws_key2; // safari websockets
    char l8b[8]; // safari websockets
    int ws_version;
    string ws_upgrade;

    /**
     * Property Nodes
     */
    SGPropertyNode *gps_timestamp_node;
    SGPropertyNode *gps_lat_node;
    SGPropertyNode *gps_lon_node;
    SGPropertyNode *gps_alt_node;
    SGPropertyNode *gps_vn_node;
    SGPropertyNode *gps_ve_node;
    SGPropertyNode *gps_vd_node;
    SGPropertyNode *gps_unix_sec_node;
    SGPropertyNode *gps_satellites_node;
    SGPropertyNode *gps_status_node;

    // imu property nodes
    SGPropertyNode *imu_timestamp_node;
    SGPropertyNode *imu_p_node;
    SGPropertyNode *imu_q_node;
    SGPropertyNode *imu_r_node;
    SGPropertyNode *imu_ax_node;
    SGPropertyNode *imu_ay_node;
    SGPropertyNode *imu_az_node;
    SGPropertyNode *imu_hx_node;
    SGPropertyNode *imu_hy_node;
    SGPropertyNode *imu_hz_node;
    SGPropertyNode *imu_status_node;

    // air data property nodes
    SGPropertyNode *airdata_timestamp_node;
    SGPropertyNode *airdata_pressure_node;
    SGPropertyNode *airdata_temperature_node;
    SGPropertyNode *airdata_altitude_node;
    SGPropertyNode *airdata_altitude_true_node;
    SGPropertyNode *airdata_airspeed_node;
    SGPropertyNode *airdata_climb_fpm_node;
    SGPropertyNode *airdata_accel_ktps_node;
    SGPropertyNode *airdata_status_node;
    SGPropertyNode *airdata_wind_dir_node;
    SGPropertyNode *airdata_wind_speed_node;
    SGPropertyNode *airdata_pitot_scale_node;

    // filter property nodes
    SGPropertyNode *filter_timestamp_node;
    SGPropertyNode *filter_theta_node;
    SGPropertyNode *filter_phi_node;
    SGPropertyNode *filter_psi_node;
    SGPropertyNode *filter_lat_node;
    SGPropertyNode *filter_lon_node;
    SGPropertyNode *filter_alt_node;
    SGPropertyNode *filter_vn_node;
    SGPropertyNode *filter_ve_node;
    SGPropertyNode *filter_vd_node;
    SGPropertyNode *filter_status_node;

    // actuator property nodes
    SGPropertyNode *act_timestamp_node;
    SGPropertyNode *act_aileron_node;
    SGPropertyNode *act_elevator_node;
    SGPropertyNode *act_throttle_node;
    SGPropertyNode *act_rudder_node;
    SGPropertyNode *act_channel5_node;
    SGPropertyNode *act_channel6_node;
    SGPropertyNode *act_channel7_node;
    SGPropertyNode *act_channel8_node;
    SGPropertyNode *act_status_node;

    // pilot input property nodes
    SGPropertyNode *pilot_timestamp_node;
    SGPropertyNode *pilot_aileron_node;
    SGPropertyNode *pilot_elevator_node;
    SGPropertyNode *pilot_throttle_node;
    SGPropertyNode *pilot_rudder_node;
    SGPropertyNode *pilot_channel5_node;
    SGPropertyNode *pilot_channel6_node;
    SGPropertyNode *pilot_channel7_node;
    SGPropertyNode *pilot_channel8_node;
    SGPropertyNode *pilot_status_node;

    // autopilot status nodes
    SGPropertyNode *ap_timestamp_node;
    SGPropertyNode *ap_hdg_node;
    SGPropertyNode *ap_roll_node;
    SGPropertyNode *ap_altitude_node;
    SGPropertyNode *ap_climb_node;
    SGPropertyNode *ap_pitch_node;
    SGPropertyNode *ap_speed_node;
    SGPropertyNode *ap_waypoint_target_node;
    SGPropertyNode *ap_route_size_node;

    // system health nodes
    SGPropertyNode *health_avionics_vcc_node;
    SGPropertyNode *health_extern_volts_node;
    SGPropertyNode *health_extern_amps_node;
    SGPropertyNode *health_extern_mah_node;
    SGPropertyNode *health_load_avg_node;

    // payload nodes
    SGPropertyNode *payload_trigger_num_node;
    SGPropertyNode *payload_lookat_lon_node;
    SGPropertyNode *payload_lookat_lat_node;
    SGPropertyNode *payload_ll_lon_node;
    SGPropertyNode *payload_ll_lat_node;
    SGPropertyNode *payload_lr_lon_node;
    SGPropertyNode *payload_lr_lat_node;
    SGPropertyNode *payload_ul_lon_node;
    SGPropertyNode *payload_ul_lat_node;
    SGPropertyNode *payload_ur_lon_node;
    SGPropertyNode *payload_ur_lat_node;

    SGPropertyNode *filter_track_node;
    SGPropertyNode *filter_speed_node;
    SGPropertyNode *wind_deg_node;
    SGPropertyNode *wind_speed_node;
    SGPropertyNode *pitot_scale_node;
    SGPropertyNode *filter_climb_node;
    SGPropertyNode *flight_flying_status;
    SGPropertyNode *flight_total_timer;
    SGPropertyNode *flight_auto_timer;

public:

    /**
     * Constructor.
     */
    WSChannel();
    
    /**
     * Process a complete request from the props client.
     */
    void handleBufferRead( netBuffer &in_buffer );
    int unmask( const char *payload, int length, char *ubuf );
    void process_line( string line );
    unsigned int get_key_sec_safari( string key );

private:
    /**
     * Bind property nodes
     */
    void bind();

    /**
     * Return a "Node no found" error message to the client.
     */
    void node_not_found_error( const string& node_name );

    /**
     * encode/send the message for the websocket channel
     */
    void encode_send( const string& message );
};

/**
 * 
 */
WSChannel::WSChannel()
    : handshake(false),
      ws_key(""),
      ws_key1(""),
      ws_key2(""),
      ws_version(0),
      ws_upgrade("")
{
    bind();
    l8b[0] = 0;
}

/**
 *
 */
void WSChannel::bind()
{
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);
    gps_satellites_node = fgGetNode("/sensors/gps/satellites", true);

    imu_timestamp_node = fgGetNode("/sensors/imu/time-stamp", true);
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);
    imu_status_node = fgGetNode("/sensors/imu/status", true);

    airdata_timestamp_node = fgGetNode("/sensors/air-data/time-stamp", true);
    airdata_pressure_node = fgGetNode("/sensors/air-data/pressure-mbar", true);
    airdata_temperature_node = fgGetNode("/sensors/air-data/temperature-degC", true);
    airdata_altitude_node = fgGetNode("/sensors/air-data/altitude-pressure-m", true);
    airdata_altitude_true_node = fgGetNode("/position/altitude-true-combined-m", true);
    airdata_airspeed_node = fgGetNode("/sensors/air-data/airspeed-kt", true);
    airdata_climb_fpm_node
	= fgGetNode("/sensors/air-data/vertical-speed-fpm", true);
    airdata_accel_ktps_node
	= fgGetNode("/sensors/air-data/acceleration-ktps", true);
    airdata_status_node = fgGetNode("/sensors/air-data/status", true);

    filter_timestamp_node = fgGetNode("/filters/filter/time-stamp", true);
    filter_theta_node = fgGetNode("/filters/filter/pitch-deg", true);
    filter_phi_node = fgGetNode("/filters/filter/roll-deg", true);
    filter_psi_node = fgGetNode("/filters/filter/heading-deg", true);
    filter_lat_node = fgGetNode("/filters/filter/latitude-deg", true);
    filter_lon_node = fgGetNode("/filters/filter/longitude-deg", true);
    filter_alt_node = fgGetNode("/filters/filter/altitude-m", true);
    filter_vn_node = fgGetNode("/filters/filter/vn-ms", true);
    filter_ve_node = fgGetNode("/filters/filter/ve-ms", true);
    filter_vd_node = fgGetNode("/filters/filter/vd-ms", true);
    filter_status_node = fgGetNode("/filters/filter/status", true);

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

    pilot_timestamp_node = fgGetNode("/sensors/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = fgGetNode("/sensors/pilot/manual", true);
    pilot_channel6_node = fgGetNode("/sensors/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/sensors/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/sensors/pilot/channel", 7, true);
    pilot_status_node = fgGetNode("/sensors/pilot/status", true);

    ap_timestamp_node = fgGetNode("/autopilot/time-stamp", true);
    ap_hdg_node = fgGetNode( "/autopilot/settings/target-heading-deg",
			     true );
    ap_roll_node = fgGetNode("/autopilot/settings/target-roll-deg", true);
    ap_altitude_node = fgGetNode( "/autopilot/settings/target-msl-ft", true );
    ap_climb_node = fgGetNode("/autopilot/internal/target-climb-rate-fps",
			      true);
    ap_pitch_node = fgGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_speed_node = fgGetNode( "/autopilot/settings/target-speed-kt", true );
    ap_waypoint_target_node
	= fgGetNode( "/autopilot/route/target-waypoint-index", true );
    ap_route_size_node = fgGetNode( "/autopilot/route/size", true );

    health_avionics_vcc_node = fgGetNode( "/status/input-vcc", true );
    health_extern_volts_node = fgGetNode( "/status/extern-volts", true );
    health_extern_amps_node = fgGetNode( "/status/extern-amps", true );
    health_extern_mah_node = fgGetNode( "/status/extern-mah", true );
    health_load_avg_node = fgGetNode( "/status/system-load-avg", true );

    payload_trigger_num_node = fgGetNode("/payload/camera/trigger-num", true);
    payload_lookat_lon_node = fgGetNode("/payload/camera/lookat-lon-deg", true);
    payload_lookat_lat_node = fgGetNode("/payload/camera/lookat-lat-deg", true);
    payload_ll_lon_node = fgGetNode("/payload/camera/lower-left-lon-deg", true);
    payload_ll_lat_node = fgGetNode("/payload/camera/lower-left-lat-deg", true);
    payload_lr_lon_node = fgGetNode("/payload/camera/lower-right-lon-deg", true);
    payload_lr_lat_node = fgGetNode("/payload/camera/lower-right-lat-deg", true);
    payload_ul_lon_node = fgGetNode("/payload/camera/upper-left-lon-deg", true);
    payload_ul_lat_node = fgGetNode("/payload/camera/upper-left-lat-deg", true);
    payload_ur_lon_node = fgGetNode("/payload/camera/upper-right-lon-deg", true);
    payload_ur_lat_node = fgGetNode("/payload/camera/upper-right-lat-deg", true);
    filter_track_node = fgGetNode("/filters/filter/track-deg", true);
    filter_speed_node = fgGetNode("/filters/filter/speed-kt", true);
    wind_deg_node = fgGetNode("/filters/wind-deg", true);
    wind_speed_node = fgGetNode("/filters/wind-speed-kt", true);
    pitot_scale_node = fgGetNode("/filters/pitot-scale-factor", true);
    filter_climb_node = fgGetNode("/filters/climb-rate-fps", true);

    flight_flying_status = fgGetNode("/status/in-flight", true);
    flight_total_timer = fgGetNode("/status/flight-timer-secs", true);
    flight_auto_timer = fgGetNode("/status/autopilot-timer-secs", true);
}


/**
 * 
 */
void
WSChannel::node_not_found_error( const string& node_name )
{
    string error = "-ERR Node \"";
    error += node_name;
    error += "\" not found\r\n";
    bufferSend( error.c_str(), error.length() );
}

/**
 * 
 */
void
WSChannel::encode_send( const string& message )
{
    unsigned int length = message.length();
    if ( ws_version >= 13 ) {
	// 0x1 text frame (FIN + opcode)
	unsigned char b1 = 0x80 | (0x1 & 0x0f);
	
	unsigned char header[6];
	unsigned int header_length = 0;
	if ( length <= 125 ) {
	    //header = pack('CC', $b1, $length);
	    header[0] = b1;
	    header[1] = length;
	    header_length = 2;
	} else if ( length > 125 && length < 65536 ) {
	    // header = pack('CCS', $b1, 126, $length);
	    header[0] = b1;
	    header[1] = 126;
	    unsigned char *ptr = (unsigned char *)(&length);
	    header[2] = ptr[1];
	    header[3] = ptr[0];
	    header_length = 4;
	} else if ( length >= 65536 ) {
	    // header = pack('CCN', $b1, 127, $length);
	    header[0] = b1;
	    header[1] = 127;
	    unsigned char *ptr = (unsigned char *)(&length);
	    header[2] = ptr[3];
	    header[3] = ptr[2];
	    header[4] = ptr[0];
	    header[5] = ptr[1];
	    header_length = 6;
	}
	bufferSend((const char *)header, header_length);
	bufferSend( message.c_str(), length );
    } else {
	char wrap[1];
	wrap[0] = 0;
	bufferSend(wrap, 1);
	bufferSend( message.c_str(), length );
	wrap[0] = 255;
	bufferSend(wrap, 1);
    }
}

static void json_add( string *reply, string fmt, double value, bool comma = true ) {
    const int maxbuf = 64;
    char buf[maxbuf];
    snprintf(buf, maxbuf, fmt.c_str(), value);
    if ( comma ) {
	*reply += ",";
    }
    *reply += buf;
}

static void json_add( string *reply, string fmt, int value, bool comma = true ) {
    const int maxbuf = 64;
    char buf[maxbuf];
    snprintf(buf, maxbuf, fmt.c_str(), value);
    if ( comma ) {
	*reply += ",";
    }
    *reply += buf;
}

/**
 * We have a command.
 * 
 */
void
WSChannel::process_line( string line )
{
    cout << "processing line = \"" << line << "\"" << endl;
    // printf("processing line = '%s' (%d)\n", line.c_str(), (int)line.length());

    vector<string> tokens = split( line );
    unsigned int count = tokens.size();

    if (!tokens.empty()) {
	string command = tokens[0];

	if ( command == "GET" && count == 3 && !handshake ) {
	    ws_root = tokens[1];
	    http_version = tokens[2];
	} else if ( command == "Upgrade:" && count == 2 && !handshake ) {
	    ws_upgrade = tokens[1];
	} else if ( command == "Connection:" && count == 2 && !handshake ) {
	    connection_mode = tokens[1];
	} else if ( command == "Host:" && count == 2 && !handshake ) {
	    host = tokens[1];
	} else if ( command == "Origin:" && count == 2 && !handshake ) {
	    origin = tokens[1];
	} else if ( command == "Sec-WebSocket-Key:" && count == 2
		    && !handshake ) {
	    ws_key = tokens[1];
	} else if ( command == "Sec-WebSocket-Version:" && count == 2
		    && !handshake ) {
	    ws_version = atoi(tokens[1].c_str());
	} else if ( command == "Sec-WebSocket-Key1:" && !handshake ) {
	    ws_key1 = line.substr(19);
	    l8b[0] = 0;
	} else if ( command == "Sec-WebSocket-Key2:" && !handshake ) {
	    ws_key2 = line.substr(19);
	    l8b[0] = 0;
        } else if ( command == "send" ) {
            command_mgr.add( tokens[1] );
        } else if ( command == "quit" ) {
	    close();
	    shouldDelete();
	    return;
	} else if ( command == "request" ) {
	    encode_send("reply to your request!\r\n");
	} else if ( command == "get" ) {
	    if ( tokens[1] == "locatt" ) {
		char reply[256];
		double airspeed = airdata_airspeed_node->getDoubleValue()
		    * pitot_scale_node->getDoubleValue();
		if ( airspeed < 0 ) { airspeed = 0.0; }
		snprintf( reply, 256,
			  "locatt %.8f %.8f %.1f %.1f %.1f %.1f\r\n",
			  filter_lon_node->getDoubleValue(),
			  filter_lat_node->getDoubleValue(),
			  filter_alt_node->getDoubleValue(),
			  airspeed,
			  filter_psi_node->getDoubleValue(),
			  filter_track_node->getDoubleValue() );
		encode_send(reply);
	    } else if ( tokens[1] == "posatt1" ) {
		char reply[256];
		double airspeed = airdata_airspeed_node->getDoubleValue()
		    * pitot_scale_node->getDoubleValue();
		if ( airspeed < 0 ) { airspeed = 0.0; }
		snprintf( reply, 256,
			  "posatt1 %.8f %.8f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\r\n",
			  filter_lon_node->getDoubleValue(),
			  filter_lat_node->getDoubleValue(),
			  filter_alt_node->getDoubleValue(),
			  airspeed,
			  filter_psi_node->getDoubleValue(),
			  filter_track_node->getDoubleValue(),
			  filter_speed_node->getDoubleValue(),
			  wind_deg_node->getDoubleValue(),
			  wind_speed_node->getDoubleValue());
		encode_send(reply);
	    } else if ( tokens[1] == "update_json" ) {
		string reply = "update_json {";

		json_add(&reply, "\"lon\":\"%.8f\"",
			 filter_lon_node->getDoubleValue(), false );

		json_add(&reply, "\"lat\":\"%.8f\"",
			 filter_lat_node->getDoubleValue() );

		json_add(&reply, "\"alt_true\":\"%.1f\"",
			 airdata_altitude_true_node->getDoubleValue() );

		json_add(&reply, "\"airspeed\":\"%.1f\"",
			 airdata_airspeed_node->getDoubleValue() );

		json_add(&reply, "\"filter_psi\":\"%.1f\"",
			 filter_psi_node->getDoubleValue() );

		json_add(&reply, "\"filter_track\":\"%.1f\"",
			 filter_track_node->getDoubleValue() );

		json_add(&reply, "\"filter_speed\":\"%.1f\"",
			 filter_speed_node->getDoubleValue() );

		json_add(&reply, "\"wind_deg\":\"%.1f\"",
			 wind_deg_node->getDoubleValue() );

		json_add(&reply, "\"wind_kts\":\"%.1f\"",
			 wind_speed_node->getDoubleValue() );

		json_add(&reply, "\"gps_sats\":\"%d\"",
			 gps_satellites_node->getIntValue() );

		json_add(&reply, "\"lost_link\":\"%d\"",
			 command_mgr.remote_lost_link_predict() );

		json_add(&reply, "\"control_mode\":\"%.0f\"",
			 pilot_channel5_node->getDoubleValue() );

		json_add(&reply, "\"ap_hdg\":\"%.1f\"",
			 ap_hdg_node->getDoubleValue() );

		json_add(&reply, "\"airdata_climb\":\"%.2f\"",
			 airdata_climb_fpm_node->getDoubleValue() );

		json_add(&reply, "\"ap_climb\":\"%.2f\"",
			 ap_climb_node->getDoubleValue() );

		json_add(&reply, "\"imu_ay\":\"%.2f\"",
			 imu_ay_node->getDoubleValue() );

		json_add(&reply, "\"imu_az\":\"%.2f\"",
			 imu_az_node->getDoubleValue() );

		json_add(&reply, "\"imu_r\":\"%.2f\"",
			 imu_r_node->getDoubleValue()*SG_RADIANS_TO_DEGREES );

		json_add(&reply, "\"filter_phi\":\"%.2f\"",
			 filter_phi_node->getDoubleValue() );

		json_add(&reply, "\"filter_theta\":\"%.2f\"",
			 filter_theta_node->getDoubleValue() );

		json_add(&reply, "\"ap_altitude\":\"%.2f\"",
			 ap_altitude_node->getDoubleValue() );

		json_add(&reply, "\"ap_speed\":\"%.1f\"",
			 ap_speed_node->getDoubleValue() );

		json_add(&reply, "\"pitot_scale\":\"%.3f\"",
			 pitot_scale_node->getDoubleValue() );

		json_add(&reply, "\"avionics_vcc\":\"%.2f\"",
			 health_avionics_vcc_node->getDoubleValue() );

		json_add(&reply, "\"main_volts\":\"%.2f\"",
			 health_extern_volts_node->getDoubleValue() );

		json_add(&reply, "\"main_amps\":\"%.2f\"",
			 health_extern_amps_node->getDoubleValue() );

		json_add(&reply, "\"main_mah\":\"%.0f\"",
			 health_extern_mah_node->getDoubleValue() );

		json_add(&reply, "\"flight_timer\":\"%.1f\"",
			 flight_total_timer->getDoubleValue() );

		json_add(&reply, "\"airdata_temp\":\"%.1f\"",
			 airdata_temperature_node->getDoubleValue() );

		json_add(&reply, "\"camera_trigger\":\"%d\"",
			 payload_trigger_num_node->getIntValue() );

		json_add(&reply, "\"camera_lookat_lon\":\"%.8f\"",
			 payload_lookat_lon_node->getDoubleValue() );

		json_add(&reply, "\"camera_lookat_lat\":\"%.8f\"",
			 payload_lookat_lat_node->getDoubleValue() );

		json_add(&reply, "\"camera_ll_lon\":\"%.8f\"",
			 payload_ll_lon_node->getDoubleValue() );

		json_add(&reply, "\"camera_ll_lat\":\"%.8f\"",
			 payload_ll_lat_node->getDoubleValue() );

		json_add(&reply, "\"camera_lr_lon\":\"%.8f\"",
			 payload_lr_lon_node->getDoubleValue() );

		json_add(&reply, "\"camera_lr_lat\":\"%.8f\"",
			 payload_lr_lat_node->getDoubleValue() );

		json_add(&reply, "\"camera_ul_lon\":\"%.8f\"",
			 payload_ul_lon_node->getDoubleValue() );

		json_add(&reply, "\"camera_ul_lat\":\"%.8f\"",
			 payload_ul_lat_node->getDoubleValue() );

		json_add(&reply, "\"camera_ur_lon\":\"%.8f\"",
			 payload_ur_lon_node->getDoubleValue() );

		json_add(&reply, "\"camera_ur_lat\":\"%.8f\"",
			 payload_ur_lat_node->getDoubleValue() );

		reply += "}\r\n",
		encode_send(reply.c_str());
	    } else if ( tokens[1] == "update1" ) {
		char reply[1024];
		snprintf( reply, 1024,
			  "update1 %.8f %.8f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %d %d %.0f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.1f %.3f %.2f %.2f %.0f %.1f %.1f\r\n",
			  filter_lon_node->getDoubleValue(),
			  filter_lat_node->getDoubleValue(),
			  airdata_altitude_true_node->getDoubleValue(),
			  airdata_airspeed_node->getDoubleValue(),
			  filter_psi_node->getDoubleValue(),
			  filter_track_node->getDoubleValue(),
			  filter_speed_node->getDoubleValue(),
			  wind_deg_node->getDoubleValue(),
			  wind_speed_node->getDoubleValue(),
			  gps_satellites_node->getIntValue(),
			  command_mgr.remote_lost_link_predict(),
			  pilot_channel5_node->getDoubleValue(),
			  ap_hdg_node->getDoubleValue(),
			  airdata_climb_fpm_node->getDoubleValue(),
			  ap_climb_node->getDoubleValue(),
			  imu_ay_node->getDoubleValue(),
			  imu_az_node->getDoubleValue(),
			  imu_r_node->getDoubleValue()*SG_RADIANS_TO_DEGREES,
			  filter_phi_node->getDoubleValue(),
			  filter_theta_node->getDoubleValue(),
			  ap_altitude_node->getDoubleValue(),
			  ap_speed_node->getDoubleValue(),
			  pitot_scale_node->getDoubleValue(),
			  health_extern_volts_node->getDoubleValue(),
			  health_extern_amps_node->getDoubleValue(),
			  health_extern_mah_node->getDoubleValue(),
			  flight_total_timer->getDoubleValue(),
			  airdata_temperature_node->getDoubleValue()
			  );
		encode_send(reply);
	    } else if ( tokens[1] == "route" ) {
		SGPropertyNode *route_node
		    = fgGetNode("/autopilot/route", true);
		SGPropertyNode *size_node
		    = route_node->getChild("size", 0, true);
		SGPropertyNode *current_node
		    = route_node->getChild("target-waypoint-index", 0, true);
		SGPropertyNode *home_node
		    = route_node->getChild("home", 0, true);
		SGPropertyNode *lon_node
		    = home_node->getChild("lon-deg", 0, true);
		SGPropertyNode *lat_node
		    = home_node->getChild("lat-deg", 0, true);
		string reply = "";
		char buf[256];
		snprintf(buf, 256, "route %.8f %.8f %d %d",
			 lon_node->getDoubleValue(),
			 lat_node->getDoubleValue(),
			 size_node->getIntValue(),
			 current_node->getIntValue() );
		reply += buf;
		for ( int i = 0; i < size_node->getIntValue(); i++ ) {
		    SGPropertyNode *wpt_node
			= route_node->getChild("wpt", i, true);
		    lon_node = wpt_node->getChild("lon-deg", 0, true);
		    lat_node = wpt_node->getChild("lat-deg", 0, true);
		    snprintf(buf, 256, " %.8f %.8f",
			     lon_node->getDoubleValue(),
			     lat_node->getDoubleValue());
		    reply += buf;
		}
		reply += "\r\n";
		encode_send(reply.c_str());
	    }
	} else if ( line.length() >= 8 ) {
	    int len = line.length();
	    strncpy(l8b, line.substr(len-8, 8).c_str(), 8);
	} else {
	    printf("remote request not understood = %s\n", command.c_str());
	}
    }

    // Chrome/Firefox v13 handshaking
    if ( !handshake && (ws_version >= 13) && ws_key.length()
	 && (ws_upgrade == "websocket") ) 
    {
	// do handshaking
	printf("Client headers are:\n");
	printf("\t- Root: %s\n", ws_root.c_str());
	printf("\t- Host: %s\n", host.c_str());
	printf("\t- Origin: %s\n", origin.c_str());
	printf("\t- Sec-WebSocket-Key: %s\n", ws_key.c_str());
	printf("\t- Sec-WebSocket-Version: %d\n", ws_version);
	printf("\n");
	printf("Generating Sec-WebSocket-Accept key...\n");
	string accept_key = ws_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
	unsigned char obuf[20];
	SHA1( (const unsigned char *)accept_key.c_str(), accept_key.length(),
	      obuf );
	string b64_key = base64_encode(obuf,20);

	string upgrade = "HTTP/1.1 101 Switching Protocols\r\n";
	upgrade += "Upgrade: websocket\r\n";
	upgrade += "Connection: Upgrade\r\n";
	upgrade += "Sec-WebSocket-Accept: ";
	upgrade += b64_key;
	upgrade += "\r\n";
	upgrade += "\r\n";
                        
	printf("Sending this response to the client:\n");
	printf("len=%ld %s", upgrade.length(), upgrade.c_str());
	bufferSend( upgrade.c_str(), upgrade.length() );
	handshake = true;
	printf("Handshake is successfully done!\n");

	// encode_send("Hello client!\r\n");
    }

    // Safari handshaking
    // printf("hs = %d key1 = %s  key2 = %s upgrade = %s\n",
    //   handshake, ws_key1.c_str(), ws_key2.c_str(), ws_upgrade.c_str());
    if ( !handshake && ws_key1.length() && ws_key2.length()
	 && (ws_upgrade == "WebSocket") && l8b[0] ) 
    {
	/*
	ws_root = "/";
	ws_upgrade = "WebSocket";
        host = "example.com";
	origin = "http://example.com";
	ws_key1 = "18x 6]8vM;54 *(5:  {   U1]8  z [  8";
	ws_key2 = "1_ tx7X d  <  nw  334J702) 7]o}` 0";
	string tmp = "Tm[K T2u";
	strncpy(l8b, tmp.c_str(), 8);
	*/

	/*
	ws_root = "/phpwebsocket/server.php";
	ws_upgrade = "WebSocket";
        host = "192.168.1.62:3000";
	origin = "http://192.168.1.62";
	ws_key1 = "E26% K7196D  W   R 8 3\".? 9- ` 8";
	ws_key2 = "{V yJ%l 1 4  $ @8198$0N1 64";
l8b[0] = 88;
l8b[1] = 68;
l8b[2] = 45;
l8b[3] = 198;
l8b[4] = 200;
l8b[5] = 62;
l8b[6] = 183;
l8b[7] = 88;
	*/

	// do handshaking
	printf("Client headers are:\n");
	printf("\t- Root: %s\n", ws_root.c_str());
	printf("\t- Host: %s\n", host.c_str());
	printf("\t- Origin: %s\n", origin.c_str());
	printf("\t- Sec-WebSocket-Key1: %s\n", ws_key1.c_str());
	printf("\t- Sec-WebSocket-Key2: %s\n", ws_key2.c_str());
	printf("\n");

	string upgrade = "HTTP/1.1 101 WebSocket Protocol Handshake\r\n";
	upgrade += "Upgrade: WebSocket\r\n";
	upgrade += "Connection: Upgrade\r\n";
	upgrade += "Sec-WebSocket-Origin: " + origin + "\r\n";
	upgrade += "Sec-WebSocket-Location: ws://" + host + ws_root + "\r\n";
	upgrade += "\r\n";
	unsigned int1 = get_key_sec_safari( ws_key1 );
	unsigned int2 = get_key_sec_safari( ws_key2 );
	unsigned char cres[16];
	cres[0] = int1 >> 24;
	cres[1] = int1 >> 16;
	cres[2] = int1 >> 8;
	cres[3] = int1;
	cres[4] = int2 >> 24;
	cres[5] = int2 >> 16;
	cres[6] = int2 >> 8;
	cres[7] = int2;
	for ( int i = 0; i < 8; i++ ) {
	    cres[8+i] = l8b[i];
	    printf("%c ", cres[8+i]);
	}
	printf("\n");
	char md[MD5_DIGEST_LENGTH+1];
	MD5(cres, 16, (unsigned char *)md);
	md[MD5_DIGEST_LENGTH] = 0;
	upgrade += md;
	upgrade += "\r\n";

	for ( int i = 0; i < MD5_DIGEST_LENGTH; i++ ) {
	    printf("%d ", (unsigned char)md[i]);
	}
	printf("\n");

	printf("Sending this response to the client (%ld):\n", upgrade.length());
	printf("%s", upgrade.c_str());
	bufferSend( upgrade.c_str(), upgrade.length() );
	char term[1]; term[0] = 0;
	bufferSend( term, 1 );
	handshake = true;
	printf("Handshake is successfully done (null terminated)!\n");

	//encode_send("Hello client!\r\n");
    }
}

unsigned int WSChannel::get_key_sec_safari( string key ) {
    string key_num = "";
    int key_spaces = 0;
    unsigned int result;

    printf("key = '%s'\n", key.c_str());
    
    for ( unsigned int i = 0; i < key.length(); i++ ) {
	char c = key.c_str()[i];
	if ( isdigit(c) ) {
	    key_num += c;
	} else if ( c == 32 ) {
	    key_spaces++;
	}
    }
    printf("key_num = %s  spaces = %d\n", key_num.c_str(), key_spaces);
    result = atof(key_num.c_str()) / key_spaces;
    printf("result = %ld\n", (long int)result);

    return result;
}


/**
 * Unmask a received payload
 * returns total bytes unmasked including headers (not just the payload len)
 * 
 */
int
WSChannel::unmask( const char *payload, int length, char *ubuf ) {
    // printf("unmasking ...\n");
    unsigned char x = payload[1] & 0x7f;
    const char *masks = NULL;
    const char *data = NULL;
    int data_len = 0;
    unsigned int payload_len = 0;
    if ( x == 126) {
	masks = payload + 4;
	data = payload + 8;
	uint16_t payload_cast = *(uint16_t *)(payload + 2);
	ulEndianSwap( &payload_cast );
	payload_len = payload_cast;
	data_len = payload_len + 8;
	// printf("umask data_len = %d %d\n", payload_cast, data_len);
    } else if ( x == 127 ) {
	masks = payload + 10;
	data = payload + 14;
	data_len = length - 14;
	unsigned char *ptr = (unsigned char *)(&payload_len);
	ptr[3] = payload[2];
	ptr[2] = payload[3];
	ptr[0] = payload[4];
	ptr[1] = payload[5];
	//payload_len = 
	//fix me -- is this 4 bytes (int) or 8 bytes (long???)
    } else {
	masks = payload + 2;
	data = payload + 6;
	payload_len = payload[1] & 0x7f;	
	data_len = payload_len + 6;
	// printf("umask data_len = %d %d\n", payload_len, data_len);
    }

    for ( unsigned int i = 0; i < payload_len; i++ ) {
	ubuf[i] = data[i] ^ masks[i%4];
    }
    ubuf[payload_len] = 0;

    return data_len;
}

void
WSChannel::handleBufferRead( netBuffer &in_buffer ) {
    //printf("Read %d bytes\n", in_buffer.getLength());

    char ubuf[in_buffer.getLength()];

    while ( in_buffer.getLength() ) {
	//printf("original = %s\n", in_buffer.getData());
	if ( handshake ) {
	    int len = unmask( in_buffer.getData(), in_buffer.getLength(),
			      ubuf );
	    // printf("unmasked %d bytes (%d orig) = '%s'\n",
	    //	   len, in_buffer.getLength(), ubuf);
	    in_buffer.remove(0, len);
	} else {
	    strncpy(ubuf, in_buffer.getData(), in_buffer.getLength());
	    in_buffer.remove();
	}

	vector<string> lines;
	lines = split( ubuf, "\r\n" );

	unsigned int count = lines.size();
	for ( unsigned int i = 0; i < count; ++i ) {
	    if ( lines[i].length() ) {
		process_line( lines[i] );
	    }
	}
    }
}

/**
 * 
 */
UGWebSocket::UGWebSocket( const int port_num ):
    enabled(false)
{
    port = port_num;
}

/**
 * 
 */
UGWebSocket::~UGWebSocket()
{
}

/**
 * 
 */
bool
UGWebSocket::open()
{
    if (enabled ) {
	printf("This shouldn't happen, but the telnet channel is already in use, ignoring\n" );
	return false;
    }

    netChannel::open();
    netChannel::bind( "", port );
    netChannel::listen( 5 );
    printf("WebSocket server started on port %d\n", port );

    enabled = true;

    return true;
}

/**
 * 
 */
bool
UGWebSocket::close()
{
    cout << "closing UGWebSocket" << endl;
    return true;
}

/**
 * 
 */
bool
UGWebSocket::process()
{
    netChannel::poll();
    return true;
}

/**
 * 
 */
void
UGWebSocket::handleAccept()
{
    netAddress addr;
    int handle = netChannel::accept( &addr );
    printf("WebSocket server accepted connection from %s:%d\n",
           addr.getHost(), addr.getPort() );
    WSChannel* channel = new WSChannel();
    channel->setHandle( handle );
}


