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

static Star our_sun;
static MoonPos moon;

static int fd = -1;
static string device_name = "/dev/ttyUSB0";

/* given a particular time expressed in side real time at prime
 * meridian (GST), compute position on the earth (lat, lon) such that
 * sun is directly overhead.  (lat, lon are reported in radians */

void fgSunPositionGST(SGTime t, double *lon, double *lat) {
    /* time_t  ssue;           seconds since unix epoch */
    /* double *lat;            (return) latitude        */
    /* double *lon;            (return) longitude       */

    our_sun.updatePosition( t.getMjd() );

    double alpha, delta;
    double tmp;

    double beta = our_sun.getLat();
    double xs = our_sun.getxs();
    double ys = our_sun.getys();
    double ye = our_sun.getye();
    double ze = our_sun.getze();
    alpha = atan2(ys - tan(beta)*ze/ys, xs);
    delta = asin(sin(beta)*ye/ys + cos(beta)*ze);
 
    tmp = alpha - (SGD_2PI/24)*t.getGst();
    if (tmp < -SGD_PI) {
        do tmp += SGD_2PI;
        while (tmp < -SGD_PI);
    } else if (tmp > SGD_PI) {
        do tmp -= SGD_2PI;
        while (tmp < -SGD_PI);
    }

    *lon = tmp;
    *lat = delta;
}


#if 0
void fgMoonPositionGST(SGTime t, double *lon, double *lat) {
    /* time_t  ssue;           seconds since unix epoch */
    /* double *lat;            (return) latitude        */
    /* double *lon;            (return) longitude       */

    moon.updatePosition( t.getMjd(), t.getLst(), our_sun.getLat(), &our_sun );

    double alpha, delta;
    double tmp;

    double beta = moon.getLat();
    double xs = moon.getxs();
    double ys = moon.getys();
    double ye = moon.getye();
    double ze = moon.getze();
    alpha = atan2(ys - tan(beta)*ze/ys, xs);
    delta = asin(sin(beta)*ye/ys + cos(beta)*ze);
 
    tmp = alpha - (SGD_2PI/24)*t.getGst();
    if (tmp < -SGD_PI) {
        do tmp += SGD_2PI;
        while (tmp < -SGD_PI);
    } else if (tmp > SGD_PI) {
        do tmp -= SGD_2PI;
        while (tmp < -SGD_PI);
    }

    *lon = tmp;
    *lat = delta;
}
#endif


static SGVec3d compute_sun_ecef( double lon_deg, double lat_deg ) {
    SGGeod pos_geod = SGGeod::fromDegM( lon_deg, lat_deg, 0 );

    SGTime t = SGTime();
    time_t cur_time = time(NULL);
    t.update( pos_geod, cur_time, 0 );

    double sun_lon, sun_gd_lat;
    fgSunPositionGST( t, &sun_lon, &sun_gd_lat );
    //printf("Sun is straight over lon=%.8f lat=%.8f\n",
    //       sun_lon*SG_RADIANS_TO_DEGREES, sun_gd_lat*SG_RADIANS_TO_DEGREES);
    SGVec3d sun_ecef = SGVec3d::fromGeod(SGGeod::fromRad(sun_lon, sun_gd_lat));
    //printf("Sun ecef=%.3f %.3f %.3f\n",
    //       sun_ecef[0], sun_ecef[1], sun_ecef[2]);

    return sun_ecef;
}


#if 0
static SGVec3d compute_moon_ecef( double lon_deg, double lat_deg ) {
    double lon_rad = lon_deg * SG_DEGREES_TO_RADIANS;
    double lat_rad = lat_deg * SG_DEGREES_TO_RADIANS;

    SGTime t = SGTime( lon_rad, lat_rad, "", 0 );
    time_t cur_time = time(NULL);
    t.update( lon_rad, lat_rad, cur_time, 0 );

    double moon_lon, moon_gd_lat;
    fgMoonPositionGST( t, &moon_lon, &moon_gd_lat );
    SGVec3d moon_ecef
	= SGVec3d::fromGeod(SGGeod::fromRad(moon_lon, moon_gd_lat));

    return moon_ecef;
}
#endif


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

    int ret = gps_open("localhost", "2947", &gps_data);
    if ( ret < 0 ) {
	printf("Error connecting to gpsd.  Is it running?\n");
	exit(-1);
    } else {
        printf("connected to gpsd = %d\n", ret);
    }

    gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    double lon_deg = -93.15705;
    double lat_deg = 45.13800;
    double track_deg = 0.0;
    double pan_tilt_zero_hdg = 180.0; // positioned to look south at zero angle
    int last_pan_pos = INT_MAX;
    int last_tilt_pos = INT_MAX;

    if ( ! uart_open( B9600 ) ) {
	printf("Cannot open pan/tilt uart\n");
	exit(-1);
    }
    
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
	    lon_deg = gps_data.fix.longitude;
	    lat_deg = gps_data.fix.latitude;
	    double track_deg = gps_data.fix.track;
	}

	printf(" pos=%.8f,%.8f trk=%.1f\n", lon_deg, lat_deg, track_deg );

	SGGeod pos_geod = SGGeod::fromDegM( lon_deg, lat_deg, 0 );
	// SGVec3d pos_ecef = SGVec3d::fromGeod(pos_geod);
	SGQuatd ecef2ned = SGQuatd::fromLonLat(pos_geod);

	SGVec3d target_ecef = compute_sun_ecef( lon_deg, lat_deg );
	// SGVec3d target_ecef = compute_moon_ecef( lon_deg, lat_deg );

	SGVec3d target_ned = normalize( ecef2ned.transform(target_ecef) );
	SGVec3d inverse_ned = target_ned * -1.0;
	printf("target_ned = %.3f %.3f %.3f\n",
	       target_ned[0], target_ned[1], target_ned[2]);
	printf("shadow_ned = %.3f %.3f %.3f\n",
	       inverse_ned[0], inverse_ned[1], inverse_ned[2]);

	double target_bearing_deg = atan2(target_ned[1], target_ned[0])
	    * SG_RADIANS_TO_DEGREES;
	double horiz = sqrt(target_ned[0]*target_ned[0]
			    + target_ned[1]*target_ned[1]);
	double target_azimuth_deg = atan2(-target_ned[2], horiz)
	    * SG_RADIANS_TO_DEGREES;
	printf("target bearing=%.2f azimuth=%.2f\n",
	       target_bearing_deg, target_azimuth_deg);

	double pan_deg = pan_tilt_zero_hdg - target_bearing_deg;
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
	int pan_min = -3088;
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

