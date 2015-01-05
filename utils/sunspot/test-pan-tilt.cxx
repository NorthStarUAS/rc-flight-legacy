#include <string.h>
#include <string>

#include <gps.h>

#include <plib/netSocket.h>

#include <simgear/ephemeris/moonpos.hxx>
#include <simgear/ephemeris/star.hxx>
#include <simgear/math/SGMath.hxx>
#include <simgear/timing/sg_time.hxx>

using std::string;


const int update_interval = 10;                       // seconds

static Star our_sun;
static MoonPos moon;


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
    double lon_rad = lon_deg * SG_DEGREES_TO_RADIANS;
    double lat_rad = lat_deg * SG_DEGREES_TO_RADIANS;

    SGTime t = SGTime( lon_rad, lat_rad, "", 0 );
    time_t cur_time = time(NULL);
    t.update( lon_rad, lat_rad, cur_time, 0 );

    double sun_lon, sun_gd_lat;
    fgSunPositionGST( t, &sun_lon, &sun_gd_lat );
    printf("Sun is straight over lon=%.8f lat=%.8f\n",
	   sun_lon*SG_RADIANS_TO_DEGREES, sun_gd_lat*SG_RADIANS_TO_DEGREES);
    SGVec3d sun_ecef = SGVec3d::fromGeod(SGGeod::fromRad(sun_lon, sun_gd_lat));
    printf("Sun ecef=%.3f %.3f %.3f\n", sun_ecef[0], sun_ecef[1], sun_ecef[2]);

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


int main() {

    struct gps_data_t gps_data;

    SGGeod loc;
    int count = update_interval;

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
    
    while ( true ) {
	bool gps_valid = true;
	if ( ! gps_waiting (&gps_data, 500000) ) {
	    gps_valid = false;
	}
	if ( gps_read (&gps_data) <= 0 ) {
	    gps_valid = false;
	}
	/* Display data from the GPS receiver. */
	printf("read something from gps\n");
	if ( ! gps_data.set ) {
	    gps_valid = false;
	}
	printf("sats used=%d fix=%d\n", 
	       gps_data.satellites_used, gps_data.fix.mode);
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
	double tilt_deg = target_azimuth_deg;
	printf("pan = %.2f  tilt = %.2f\n", pan_deg, tilt_deg);
       
    }
}

