//
// FILE: FGFS.cpp
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#include <pyprops.h>

#include <stdlib.h>		// drand48()
#include <sys/ioctl.h>

#include <iostream>
using std::cout;
using std::endl;

#include "comms/display.h"
#include "filters/nav_common/coremag.h"
#include "filters/nav_common/nav_functions.h"
#include "util/props_helper.h"
#include "util/timing.h"

#include "fgfs.h"

static const float D2R = M_PI / 180.0;

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

void fgfs_t::info( const char *format, ... ) {
    if ( display_on ) {
        printf("fgfs: ");
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\n");
    }
}

void fgfs_t::hard_error( const char *format, ... ) {
    printf("fgfs hard error: ");
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
    printf("Cannot continue.");
    exit(-1);
}

void fgfs_t::init_airdata( pyPropertyNode *config ) {
    string output_path = get_next_path("/sensors", "airdata", true);
    airdata_node = pyGetNode(output_path.c_str(), true);
}

void fgfs_t::init_act( pyPropertyNode *config ) {
    string hostname = "";
    if ( config->hasChild("host") ) {
        hostname = config->getString("host");
    } else {
        hard_error("no actuator hostname specified in driver config.");
    }
    int port = -1;
    if ( config->hasChild("port") ) {
        port = config->getLong("port");
    } else {
        hard_error("no actuator port specified in driver config.");
    }
    
    string output_path = get_next_path("/sensors", "gps", true);
    gps_node = pyGetNode(output_path.c_str(), true);

    // open a UDP socket
    if ( ! sock_act.open( false ) ) {
	hard_error("Error opening actuator output socket");
    }

    // connect
    if ( sock_act.connect( hostname.c_str(), port ) == -1 ) {
	hard_error("error connecting to %s:%d\n", hostname.c_str(), port);
    }
    
    // don't block waiting for input
    sock_act.setBlocking( false );
}

void fgfs_t::init_gps( pyPropertyNode *config ) {
    int port = -1;
    if ( config->hasChild("port") ) {
        port = config->getLong("port");
    } else {
        hard_error("no gps port specified in driver config.");
    }
    
    // open a UDP socket
    if ( ! sock_gps.open( false ) ) {
	hard_error("Error opening gps input socket");
    }

    // bind ...
    if ( sock_gps.bind( "", port ) == -1 ) {
	hard_error("error binding to gps port %d\n", port );
    }

    // don't block waiting for input
    sock_gps.setBlocking( false );
}

void fgfs_t::init_imu( pyPropertyNode *config ) {
    int port = -1;
    if ( config->hasChild("port") ) {
        port = config->getLong("port");
    } else {
        hard_error("no imu port specified in driver config.");
    }
        
    string output_path = get_next_path("/sensors", "imu", true);
    imu_node = pyGetNode(output_path.c_str(), true);
    
    // open a UDP socket
    if ( ! sock_imu.open( false ) ) {
	hard_error("Error opening imu input socket");
    }

    // bind ...
    if ( sock_imu.bind( "", port ) == -1 ) {
	hard_error("error binding to imu port %d", port );
    }

#if 0 // we are using blocking for main loop sync now
    // don't block waiting for input
    sock_imu.setBlocking( false );
#endif
}

void fgfs_t::init( pyPropertyNode *config ) {
    act_node = pyGetNode("/actuators", true);
    orient_node = pyGetNode("/orientation", true);
    pos_node = pyGetNode("/position", true);
    power_node = pyGetNode("/sensors/power", true);
    power_node.setDouble( "avionics_vcc", 5.05 ); // set initial fake value
    route_node = pyGetNode("/task/route", true);    
    targets_node = pyGetNode("/autopilot/targets", true);
    
    if ( config->hasChild("actuators") ) {
        pyPropertyNode act_config = config->getChild("actuators");
        init_act(&act_config);
    }        
    if ( config->hasChild("gps") ) {
        pyPropertyNode gps_config = config->getChild("gps");
        init_gps(&gps_config);
    }        
    if ( config->hasChild("imu") ) {
        pyPropertyNode imu_config = config->getChild("imu");
        init_imu(&imu_config);
        pyPropertyNode airdata_config = config->getChild("airdata");
        init_airdata(&airdata_config);
    }
    
    airdata_node.setDouble( "temp_degC", 15.0 ); // set initial fake value
    pyPropertyNode specs_node = pyGetNode("/config/specs", true);
    if ( specs_node.hasChild("battery_cells") ) {
        battery_cells = specs_node.getLong("battery_cells");
        if ( battery_cells < 1 ) { battery_cells = 4; }
    }
}

bool fgfs_t::update_gps() {
    const int fgfs_gps_size = 40;
    uint8_t packet_buf[fgfs_gps_size];

    bool fresh_data = false;

    int result;
    while ( (result = sock_gps.recv(packet_buf, fgfs_gps_size, 0))
	    == fgfs_gps_size )
    {
	fresh_data = true;

	if ( ulIsLittleEndian ) {
	    my_swap( packet_buf, 0, 8 );
	    my_swap( packet_buf, 8, 8 );
	    my_swap( packet_buf, 16, 8 );
	    my_swap( packet_buf, 24, 4 );
	    my_swap( packet_buf, 28, 4 );
	    my_swap( packet_buf, 32, 4 );
	    my_swap( packet_buf, 36, 4 );
	}

	uint8_t *buf = packet_buf;
	double time = *(double *)buf; buf += 8;
	double lat = *(double *)buf; buf += 8;
	double lon = *(double *)buf; buf += 8;
	float alt = *(float *)buf; buf += 4;
	float vn = *(float *)buf; buf += 4;
	float ve = *(float *)buf; buf += 4;
	float vd = *(float *)buf; buf += 4;

        if ( false ) {
            // add some random white noise
            double vel_noise = 0.1;
            double vel_offset = vel_noise * 0.5;
            vn += drand48()*vel_noise - vel_offset;
            ve += drand48()*vel_noise - vel_offset;
            vd += drand48()*vel_noise - vel_offset;
        }
        
        // compute ideal magnetic vector in ned frame
        long int jd = now_to_julian_days();
        double field[6];
        calc_magvar( lat*D2R, lon*D2R, alt / 1000.0, jd, field );
        mag_ned(0) = field[3];
        mag_ned(1) = field[4];
        mag_ned(2) = field[5];
        mag_ned.normalize();
        // cout << "mag vector (ned): " << mag_ned(0) << " " << mag_ned(1) << " " << mag_ned(2) << endl;
        
	gps_node.setDouble( "timestamp", get_Time() );
	gps_node.setDouble( "latitude_deg", lat );
	gps_node.setDouble( "longitude_deg", lon );
	gps_node.setDouble( "altitude_m", alt );
	gps_node.setDouble( "vn_ms", vn );
	gps_node.setDouble( "ve_ms", ve );
	gps_node.setDouble( "vd_ms", vd );
	gps_node.setLong( "satellites", 8 ); // fake a solid number
	gps_node.setDouble( "unix_time_sec", time );
	gps_node.setLong( "status", 2 ); // valid fix
    }

    return fresh_data;
}

bool fgfs_t::update_imu() {
    const int fgfs_imu_size = 52;
    uint8_t packet_buf[fgfs_imu_size];

    bool fresh_data = false;

    int result;
    if ( (result = sock_imu.recv(packet_buf, fgfs_imu_size, 0))
	    == fgfs_imu_size )
    {
	fresh_data = true;

	if ( ulIsLittleEndian ) {
	    my_swap( packet_buf, 0, 8 );
	    my_swap( packet_buf, 8, 4 );
	    my_swap( packet_buf, 12, 4 );
	    my_swap( packet_buf, 16, 4 );
	    my_swap( packet_buf, 20, 4 );
	    my_swap( packet_buf, 24, 4 );
	    my_swap( packet_buf, 28, 4 );
	    my_swap( packet_buf, 32, 4 );
	    my_swap( packet_buf, 36, 4 );
	    my_swap( packet_buf, 40, 4 );
	    my_swap( packet_buf, 44, 4 );
	    my_swap( packet_buf, 48, 4 );
	}

	uint8_t *buf = packet_buf;
	/*double time = *(double *)buf;*/ buf += 8;
	float p = *(float *)buf; buf += 4;
	float q = *(float *)buf; buf += 4;
	float r = *(float *)buf; buf += 4;
	float ax = *(float *)buf; buf += 4;
	float ay = *(float *)buf; buf += 4;
	float az = *(float *)buf; buf += 4;
	float airspeed = *(float *)buf; buf += 4;
	float pressure = *(float *)buf; buf += 4;
	float roll_truth = *(float *)buf; buf += 4;
	float pitch_truth = *(float *)buf; buf += 4;
	float yaw_truth = *(float *)buf; buf += 4;

        // simulate an off kilter imu mounting
        Vector3f gv = Vector3f(p, q, r);
        Vector3f av = Vector3f(ax, ay, az);
        float a_deg = imu_node.getDouble("bank_bias_deg");
        float a_rad = a_deg * D2R;
        float sina = sin(a_rad);
        float cosa = cos(a_rad);
        Matrix3f R;
        R << 1.0,   0.0,  0.0,
             0.0, cosa,  sina,
             0.0, -sina, cosa;
        Vector3f ngv = R * gv;
        Vector3f nav = R * av;
        //cout << av << endl << nav << endl << endl;

        // generate fake magnetometer readings
        q_N2B = eul2quat(roll_truth * D2R, pitch_truth * D2R, yaw_truth * D2R);
        // rotate ideal mag vector into body frame (then normalized)
        Vector3f mag_body = q_N2B.inverse() * mag_ned;
        mag_body.normalize();
        // cout << "mag vector (body): " << mag_body(0) << " " << mag_body(1) << " " << mag_body(2) << endl;

	double cur_time = get_Time();
	imu_node.setDouble( "timestamp", cur_time );
	imu_node.setDouble( "p_rad_sec", ngv(0) );
	imu_node.setDouble( "q_rad_sec", ngv(1) );
	imu_node.setDouble( "r_rad_sec", ngv(2) );
	imu_node.setDouble( "ax_mps_sec", nav(0) );
	imu_node.setDouble( "ay_mps_sec", nav(1) );
	imu_node.setDouble( "az_mps_sec", nav(2) );
	imu_node.setDouble( "ax_nocal", nav(0) );
	imu_node.setDouble( "ay_nocal", nav(1) );
	imu_node.setDouble( "az_nocal", nav(2) );
	imu_node.setDouble( "hx", mag_body(0) );
	imu_node.setDouble( "hy", mag_body(1) );
	imu_node.setDouble( "hz", mag_body(2) );
	imu_node.setDouble( "hx_nocal", mag_body(0) );
	imu_node.setDouble( "hy_nocal", mag_body(1) );
	imu_node.setDouble( "hz_nocal", mag_body(2) );
	imu_node.setDouble( "roll_truth", roll_truth );
	imu_node.setDouble( "pitch_truth", pitch_truth );
	imu_node.setDouble( "yaw_truth", yaw_truth );

        airdata_node.setDouble( "timestamp", cur_time );
        airdata_node.setDouble( "airspeed_kt", airspeed );
        const double inhg2mbar = 33.8638866667;
        airdata_node.setDouble( "pressure_mbar", pressure * inhg2mbar );

        // fake volt/amp values here for no better place to do it
        static double last_time = cur_time;
        static double mah = 0.0;
        double thr = act_node.getDouble("throttle");
        power_node.setDouble("main_vcc", 16.0 - thr);
        power_node.setDouble("cell_vcc", (16.0 - thr) / battery_cells);
        power_node.setDouble("main_amps", thr * 12.0);
        double dt = cur_time - last_time;
        mah += thr*75.0 * (1000.0/3600.0) * dt;
        last_time = cur_time;
        power_node.setDouble( "total_mah", mah );
    }

    return fresh_data;
}

// Read fgfs packets using IMU packet as the main timing reference.
// Returns the dt from the IMU perspective, not the localhost
// perspective.  This should generally be far more accurate and
// consistent.
float fgfs_t::read() {
    // read packets until we receive an IMU packet and the socket
    // buffer is empty.  The IMU packet (combined with being caught up
    // reading the buffer is our signal to run an interation of the
    // main loop.
    double last_time = imu_node.getDouble( "timestamp" );
    int bytes_available = 0;
    update_gps();
    while ( true ) {
        update_imu();
	ioctl(sock_imu.getHandle(), FIONREAD, &bytes_available);
	if ( !bytes_available ) {
	    break;
        }
	// printf("looping: %d bytes available in imu sock buffer\n", bytes_available);
    }

    double cur_time = imu_node.getDouble( "timestamp" );

    return cur_time - last_time;
}

void fgfs_t::write() {
    const double F2M = 0.3048;
    const double M2F = 1 / F2M;
    
    // additional autopilot target nodes (note this is a hack, but we
    // are sending data back to FG in this module so it makes some
    // sense to include autopilot targets.)

    const int fgfs_act_size = 76;
    uint8_t packet_buf[fgfs_act_size];
    uint8_t *buf = packet_buf;

    double time = act_node.getDouble("timestamp");
    *(double *)buf = time; buf += 8;

    float ail = act_node.getDouble("aileron");
    *(float *)buf = ail; buf += 4;

    float ele = act_node.getDouble("elevator");
    *(float *)buf = ele; buf += 4;

    float thr = act_node.getDouble("throttle");
    *(float *)buf = thr; buf += 4;

    float rud = act_node.getDouble("rudder");
    *(float *)buf = rud; buf += 4;

    float ch5 = act_node.getDouble("channel5");
    *(float *)buf = ch5; buf += 4;

    float ch6 = act_node.getDouble("channel6");
    *(float *)buf = ch6; buf += 4;

    float ch7 = act_node.getDouble("channel7");
    *(float *)buf = ch7; buf += 4;

    float ch8 = act_node.getDouble("channel8");
    *(float *)buf = ch8; buf += 4;

    float bank = targets_node.getDouble("roll_deg") * 100 + 18000.0;
    *(float *)buf = bank; buf += 4;

    float pitch = targets_node.getDouble("pitch_deg") * 100 + 9000.0;
    *(float *)buf = pitch; buf += 4;

    float target_track_offset = targets_node.getDouble("groundtrack_deg")
	- orient_node.getDouble("heading_deg");
    if ( target_track_offset < -180 ) { target_track_offset += 360.0; }
    if ( target_track_offset > 180 ) { target_track_offset -= 360.0; }
    float hdg = target_track_offset * 100 + 36000.0;
    *(float *)buf = hdg; buf += 4;

    // FIXME: no longer used so wasted 4 bytes ...
    float climb = targets_node.getDouble("climb_rate_fps") * 1000 + 100000.0;
    *(float *)buf = climb; buf += 4;

    float alt_agl_ft = targets_node.getDouble("altitude_agl_ft");
    float ground_m = pos_node.getDouble("altitude_ground_m");
    float alt_msl_ft = (ground_m * M2F + alt_agl_ft) * 100.0;
    *(float *)buf = alt_msl_ft; buf += 4;

    float speed = targets_node.getDouble("target_speed_kt") * 100;
    *(float *)buf = speed; buf += 4;

    float track_offset = orient_node.getDouble("groundtrack_deg")
	- orient_node.getDouble("heading_deg");
    if ( track_offset < -180 ) { track_offset += 360.0; }
    if ( track_offset > 180 ) { track_offset -= 360.0; }
    float offset = track_offset * 100 + 36000.0;
    *(float *)buf = offset; buf += 4;

    float dist = route_node.getDouble("wp_dist_m") / 10.0;
    *(float *)buf = dist; buf += 4;

    float eta = route_node.getDouble("wp_eta_sec");
    *(float *)buf = eta; buf += 4;

    if ( ulIsLittleEndian ) {
	my_swap( packet_buf, 0, 8 );
	my_swap( packet_buf, 8, 4 );
	my_swap( packet_buf, 12, 4 );
	my_swap( packet_buf, 16, 4 );
	my_swap( packet_buf, 20, 4 );
	my_swap( packet_buf, 24, 4 );
	my_swap( packet_buf, 28, 4 );
	my_swap( packet_buf, 32, 4 );
	my_swap( packet_buf, 36, 4 );
	my_swap( packet_buf, 40, 4 );
	my_swap( packet_buf, 44, 4 );
	my_swap( packet_buf, 48, 4 );
	my_swap( packet_buf, 52, 4 );
	my_swap( packet_buf, 56, 4 );
	my_swap( packet_buf, 60, 4 );
	my_swap( packet_buf, 64, 4 );
	my_swap( packet_buf, 68, 4 );
	my_swap( packet_buf, 72, 4 );
    }

    int result = sock_act.send( packet_buf, fgfs_act_size, 0 );
    if ( result != fgfs_act_size ) {
	info("unable to write full actuator packet.");
    }
}


void fgfs_t::close() {
    sock_act.close();
    sock_gps.close();
    sock_imu.close();
}
