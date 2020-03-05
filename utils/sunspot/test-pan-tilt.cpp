#include <string.h>
#include <string>
#include <termios.h>            // tcgetattr() et. al.
#
#include <gps.h>

#include <plib/netSocket.h>

#include <simgear/ephemeris/moonpos.hxx>
#include <simgear/ephemeris/star.hxx>
#include <simgear/math/SGMath.hxx>
#include <simgear/timing/sg_time.hxx>

using std::string;

static string track_mode = "sun";
//static string track_mode = "moon";
//static string track_mode = "wgs84";

static Star our_sun;
static MoonPos moon;

static int fd = -1;
static string device_name = "/dev/ttyUSB0";

static string gpsd_host = "localhost";
//static string gpsd_host = "192.168.1.64";
static string gpsd_port = "2947";

/* given a particular time expressed in side real time at prime
 * meridian (GST), compute position on the earth (lat, lon) such that
 * sun is directly overhead.  (lat, lon are reported in radians */
void fgSunPositionGST(SGTime t, double *lon_deg, double *lat_deg) {
    /* SGTime t;               current time             */
    /* double *lat;            (return) latitude        */
    /* double *lon;            (return) longitude       */

    our_sun.updatePosition( t.getMjd() );

    printf("Sun -> ra = %.8f dec=%.8f\n",
	   our_sun.getRightAscension() * SG_RADIANS_TO_DEGREES,
	   our_sun.getDeclination() * SG_RADIANS_TO_DEGREES);
    printf("Sun -> Gst = %.2f\n", t.getGst());
    double gst_deg = t.getGst() * 360.0 /*degrees*/ / 24.0 /*hours*/;
    *lon_deg = our_sun.getRightAscension() * SG_RADIANS_TO_DEGREES - gst_deg;
    if ( *lon_deg < -180.0 ) { *lon_deg += 360.0; }
    if ( *lon_deg > 180.0 ) { *lon_deg -= 360.0; }
    *lat_deg = our_sun.getDeclination() * SG_RADIANS_TO_DEGREES;	
    printf("Sun -> lon = %.8f lat=%.8f\n", *lon_deg, *lat_deg);
 }


void fgMoonPositionGST(SGTime t, double *lon_deg, double *lat_deg) {
    /* SGTime t;               current time             */
    /* double *lat;            (return) latitude        */
    /* double *lon;            (return) longitude       */

    moon.updatePosition( t.getMjd(), t.getLst(), our_sun.getLat(), &our_sun );

    printf("Moon -> ra = %.8f dec=%.8f\n",
	   moon.getRightAscension() * SG_RADIANS_TO_DEGREES,
	   moon.getDeclination() * SG_RADIANS_TO_DEGREES);
    printf("Moon -> Gst = %.2f\n", t.getGst());
    double gst_deg = t.getGst() * 360.0 /*degrees*/ / 24.0 /*hours*/;
    *lon_deg = moon.getRightAscension() * SG_RADIANS_TO_DEGREES - gst_deg;
    if ( *lon_deg < -180.0 ) { *lon_deg += 360.0; }
    if ( *lon_deg > 180.0 ) { *lon_deg -= 360.0; }
    *lat_deg = moon.getDeclination() * SG_RADIANS_TO_DEGREES;	
    printf("Moon -> lon = %.8f lat=%.8f\n", *lon_deg, *lat_deg);
}


static SGVec3d compute_sun_ecef( double lon_deg, double lat_deg ) {
    SGGeod pos_geod = SGGeod::fromDegM( lon_deg, lat_deg, 0 );

    SGTime t = SGTime();
    time_t cur_time = time(NULL);
    t.update( pos_geod, cur_time, 0 );

    double sun_lon, sun_gd_lat;
    fgSunPositionGST( t, &sun_lon, &sun_gd_lat );
    printf("Sun is straight over lon=%.8f lat=%.8f\n",
           sun_lon, sun_gd_lat);
    SGVec3d sun_ecef = SGVec3d::fromGeod(SGGeod::fromDeg(sun_lon, sun_gd_lat));
    //printf("Sun ecef=%.3f %.3f %.3f\n",
    //       sun_ecef[0], sun_ecef[1], sun_ecef[2]);

    return sun_ecef;
}


static SGVec3d compute_moon_ecef( double lon_deg, double lat_deg ) {
    SGGeod pos_geod = SGGeod::fromDegM( lon_deg, lat_deg, 0 );

    SGTime t = SGTime();
    time_t cur_time = time(NULL);
    t.update( pos_geod, cur_time, 0 );

    double moon_lon, moon_gd_lat;
    fgMoonPositionGST( t, &moon_lon, &moon_gd_lat );
    SGVec3d moon_ecef
	= SGVec3d::fromGeod(SGGeod::fromDeg(moon_lon, moon_gd_lat));

    return moon_ecef;
}


// send our configured init strings to configure gpsd the way we prefer
static bool uart_open(tcflag_t baud) {
    fd = open( device_name.c_str(), O_RDWR /* | O_NOCTTY */ );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// Serial port settings
    // memset(&config, 0, sizeof(config));

    // Fetch current serial port settings
    tcgetattr(fd, &config); 
    cfsetspeed( &config, baud );
    // cfmakeraw( &config );

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    return true;
}


bool uart_send( const char *payload ) {
    unsigned int size = strlen( payload );
    unsigned int len = write( fd, payload, size );
    if ( len != size ) {
	printf("wrote %d of %d bytes -- try again\n", len, size);
	return false;
    } else {
	printf("wrote %d bytes: %s\n", len, payload);
	fsync(fd);
	return true;
    }

}


int main() {
    struct gps_data_t gps_data;

    SGGeod loc;

    netInit(NULL, NULL);

    netSocket uglink_sock;

    int ret = gps_open(gpsd_host.c_str(), gpsd_port.c_str(), &gps_data);
    if ( ret < 0 ) {
	printf("Error connecting to gpsd.  Is it running?\n");
	exit(-1);
    } else {
        printf("connected to gpsd = %d\n", ret);
    }

    gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    double base_lon_deg = -93.15705;
    double base_lat_deg = 45.13800;
    double base_alt_m = 278.0;
    double track_deg = 0.0;

    double body_heading_deg = 134.0; // positioned to look south at zero angle
    double body_roll_deg = 0.0;
    double body_pitch_deg = 0.0;
    
    int last_pan_pos = INT_MAX;
    int last_tilt_pos = INT_MAX;

    if ( ! uart_open( B9600 ) ) {
	printf("Cannot open pan/tilt uart\n");
	exit(-1);
    }
    uart_send("\r\n\r\n");
    
    while ( true ) {
	bool gps_valid = true;
	if ( ! gps_waiting (&gps_data, 500000) ) {
	    gps_valid = false;
	}
	if ( gps_read (&gps_data) <= 0 ) {
	    gps_valid = false;
	} else {
	    printf("read something from gps\n");
	}
	if ( ! gps_data.set ) {
	    gps_valid = false;
	} else {
	    printf("sats used=%d fix=%d\n", 
		   gps_data.satellites_used, gps_data.fix.mode);
	}
	if ( gps_data.fix.mode < 2 ) {
	    gps_valid = false;
	}
	if ( isnan(gps_data.fix.longitude) ) {
	    gps_valid = false;
	}
	if ( isnan(gps_data.fix.latitude) ) {
	    gps_valid = false;
	}
	if ( isnan(gps_data.fix.track) ) {
	    gps_valid = false;
	}

	if ( gps_valid ) {
	    base_lon_deg = gps_data.fix.longitude;
	    base_lat_deg = gps_data.fix.latitude;
	    base_alt_m = gps_data.fix.altitude;
	    double track_deg = gps_data.fix.track;
	}

	printf(" pos=%.8f,%.8f trk=%.1f\n", base_lon_deg, base_lat_deg, track_deg );

	SGGeod pos_geod = SGGeod::fromDegM( base_lon_deg, base_lat_deg, 0 );
	SGVec3d pos_ecef = SGVec3d::fromGeod(pos_geod);
	SGQuatd ecef2ned = SGQuatd::fromLonLat(pos_geod);

	SGVec3d dir_ecef;
	if ( track_mode == "sun" ) {
	    dir_ecef = compute_sun_ecef(base_lon_deg, base_lat_deg);
	} else if ( track_mode == "moon" ) {
	    dir_ecef = compute_moon_ecef(base_lon_deg, base_lat_deg);
	} else if ( track_mode == "wgs84" ) {
	    double target_lon_deg = base_lon_deg + 1.0;
	    double target_lat_deg = base_lat_deg;
	    double target_alt_m = base_alt_m + 40000.0;

	    SGGeod target_geod = SGGeod::fromDegM( target_lon_deg,
						   target_lat_deg,
						   target_alt_m );
	    printf("target_geod = %.3f %.3f %.3f\n",
		   target_geod.getLongitudeDeg(),
		   target_geod.getLatitudeDeg(),
		   target_geod.getElevationM());
	    SGVec3d target_ecef = SGVec3d::fromGeod( target_geod );
	    printf("target_ecef = %.3f %.3f %.3f\n",
		   target_ecef[0], target_ecef[1], target_ecef[2]);
	    dir_ecef = target_ecef - pos_ecef;
	    printf("dir_ecef = %.3f %.3f %.3f\n",
		   dir_ecef[0], dir_ecef[1], dir_ecef[2]);
	}
	
	// Body orientation (the "body" frame of reference which the
	// pan/tilt is mounted to that could potential move around or
	// be positioned different ways.)
	double heading_rad = body_heading_deg * SGD_DEGREES_TO_RADIANS;
	double pitch_rad = body_pitch_deg * SGD_DEGREES_TO_RADIANS;
	double roll_rad = body_roll_deg * SGD_DEGREES_TO_RADIANS;
	SGQuatd ned2body
	    = SGQuatd::fromYawPitchRoll( heading_rad, pitch_rad, roll_rad );

	// ECEF to aircraft body (eye) coordinate system
	// transformation
	SGQuatd ecef2body = ecef2ned * ned2body;

	// compute pointing direction in the ned frame of reference
	SGVec3d dir_ned = normalize( ecef2ned.transform(dir_ecef) );
	printf("dir_ned = %.3f %.3f %.3f\n",
	       dir_ned[0], dir_ned[1], dir_ned[2]);

	// compute pointing direction in the body frame of reference
	SGVec3d dir_body = normalize( ecef2body.transform(dir_ecef) );
	printf("dir_body = %.3f %.3f %.3f\n",
	       dir_body[0], dir_body[1], dir_body[2]);

	SGVec3d inverse_ned = dir_ned * -1.0;
	printf("shadow_ned = %.3f %.3f %.3f\n",
	       inverse_ned[0], inverse_ned[1], inverse_ned[2]);

	double target_bearing_deg = atan2(dir_ned[1], dir_ned[0])
	    * SG_RADIANS_TO_DEGREES;
	double horiz_range = sqrt(dir_ned[0]*dir_ned[0]
			    + dir_ned[1]*dir_ned[1]);
	double target_azimuth_deg = atan2(-dir_ned[2], horiz_range)
	    * SG_RADIANS_TO_DEGREES;
	printf("target bearing=%.2f azimuth=%.2f\n",
	       target_bearing_deg, target_azimuth_deg);

	double pan_deg = body_heading_deg - target_bearing_deg;
	if ( pan_deg < -180.0 ) {
	    pan_deg += 360.0;
	}
	if ( pan_deg > 180.0 ) {
	    pan_deg -= 360.0;
	}
	double tilt_deg = target_azimuth_deg;
	printf("pan = %.2f  tilt = %.2f\n", pan_deg, tilt_deg);

	double pan_res = 185.1428; // seconds/arc
	double tilt_res = 46.2857; // seconds/arc
	int pan_min = -3087;
	int pan_max = 3088;
	int tilt_min = -3510;
	int tilt_max = 2314;
	
	int pan_pos = round(pan_deg / (pan_res/3600.0));
	if ( pan_pos < pan_min ) { pan_pos = pan_min; }
	if ( pan_pos > pan_max ) { pan_pos = pan_max; }
	int tilt_pos = round(tilt_deg / (tilt_res/3600.0));
	if ( tilt_pos < tilt_min ) { tilt_pos = tilt_min; }
	if ( tilt_pos > tilt_max ) { tilt_pos = tilt_max; }

	const int maxlen = 256;
	char command[maxlen];
	if ( pan_pos != last_pan_pos ) {
	    last_pan_pos = pan_pos;
	    snprintf( command, maxlen, "PP%d\r\n", pan_pos );
	    printf(command);
	    uart_send(command);
	}
	if ( tilt_pos != last_tilt_pos ) {
	    last_tilt_pos = tilt_pos;
	    snprintf( command, maxlen, "TP%d\r\n", tilt_pos );
	    printf(command);
	    uart_send(command);
	}
    }
}

