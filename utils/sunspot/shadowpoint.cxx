#include <string.h>
#include <string>

#include <plib/netSocket.h>

#include <simgear/ephemeris/moonpos.hxx>
#include <simgear/ephemeris/star.hxx>
#include <simgear/math/SGMath.hxx>
#include <simgear/timing/sg_time.hxx>

using std::string;


const int update_interval = 10;                       // seconds

static Star our_sun;
static MoonPos moon;


bool parse_pos(const string text, double *lat_deg, double *lon_deg ) {
    string::size_type pos = text.find_first_of("=");
    if ( pos == string::npos ) {
        // bogus command
        return false;
    }

    if ( text.substr(pos+1)[0] == '?' ) {
        // no gps solution
        return false;
    }

    string gpstext = text.substr(pos + 1);

    pos = gpstext.find_first_of(" ");
    if ( pos == string::npos ) {
        // bogus command
        return false;
    }

    *lat_deg = atof( gpstext.substr(0, pos).c_str() );
    *lon_deg = atof( gpstext.substr(pos + 1).c_str() );

    return true;
}


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
    SGVec3d sun_ecef = SGVec3d::fromGeod(SGGeod::fromRad(sun_lon, sun_gd_lat));

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

    SGGeod loc, last_loc;
    double track_filt_deg = 0.0;
    int count = update_interval;

    netInit(NULL, NULL);

    netSocket gpsd_sock;
    netSocket uglink_sock;

    if ( ! gpsd_sock.open( true ) ) {
        printf("error opening gpsd socket\n");
    }

    if (gpsd_sock.connect( "localhost", 2947 ) < 0) {
        printf("error connecting to gpsd\n");
    }

    while ( true ) {
        const char *cmdpos = "p";

        int result;
        char pos_str[256];

        gpsd_sock.send( cmdpos, strlen(cmdpos) );
        result = gpsd_sock.recv( pos_str, 256 );
	pos_str[result] = 0;
        printf("pos = %d: %s", result, pos_str);

        double lat_deg, lon_deg;

        if ( parse_pos( pos_str, &lat_deg, &lon_deg ) ) {
            printf( "position = %.6f %.6f\n", lat_deg, lon_deg );

            // compute track based on current position and last position.
            loc = SGGeod::fromDeg( lon_deg, lat_deg );
            double course1, course2, distance;
            SGGeodesy::inverse( last_loc, loc, course1, course2, distance );
            // printf(" my course = %.1f\n", course1);

            last_loc = loc;
        } else {
            printf( "no position\n" );
        }

        count--;
        printf("count = %d\n", count);
        if ( count < 0 ) {
	    lon_deg = -93.210;
	    lat_deg = 45.145;
	    SGGeod pos_geod = SGGeod::fromDegM( lon_deg, lat_deg, 0 );
	    // SGVec3d pos_ecef = SGVec3d::fromGeod(pos_geod);
	    SGQuatd ecef2ned = SGQuatd::fromLonLat(pos_geod);

	    SGVec3d sun_ecef = compute_sun_ecef( lon_deg, lat_deg );
	    SGVec3d sun_ned = normalize( ecef2ned.transform(sun_ecef) );
	    SGVec3d shadow_ned = sun_ned * -1.0;
	    printf("sun_ned = %.3f %.3f %.3f\n", sun_ned[0], sun_ned[1], sun_ned[2]);
	    printf("shadow_ned = %.3f %.3f %.3f\n",
		   shadow_ned[0], shadow_ned[1], shadow_ned[2]);

	    // SGVec3d moon_ecef = compute_moon_ecef( lon_deg, lat_deg );

            // send out home position update to UAS
            char uglink_cmd[256];
            snprintf(uglink_cmd, 256, "send laned,%.4f,%.4f,%.4f\r\n",
                     shadow_ned[0], shadow_ned[1], shadow_ned[2]);
            printf("%s", uglink_cmd);

            if ( uglink_sock.open( true ) ) {
                if (uglink_sock.connect( "localhost", 5402 ) >= 0) {
                    uglink_sock.send( uglink_cmd, strlen(uglink_cmd) );
                } else {
                    printf("error connecting to uglink\n");
                }
                uglink_sock.close();
            } else {
                printf("error opening uglink socket\n");
            }

            count = update_interval;
        }

        sleep(1);
    }

    gpsd_sock.close();
}

