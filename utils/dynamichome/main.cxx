#include <string.h>
#include <string>

#include "comms/netSocket.h"
#include "math/SGMath.hxx"

using std::string;


const int update_interval = 10;                       // seconds

bool parse_track(const string text, double *track_deg /* true */ ) {
    string::size_type pos = text.find_first_of("=");
    if ( pos == string::npos ) {
        // bogus command
        return false;
    }

    if ( text.substr(pos+1)[0] == '?' ) {
        // no gps solution
        return false;
    }

    *track_deg = atof( text.substr(pos + 1).c_str() );
    return true;
}


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


int main() {

    SGGeod loc, last_loc;
    double filt_x = 0.0;
    double filt_y = 0.0;
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
        const char *cmdtrack = "t";
        const char *cmdpos = "p";

        int result;
        char track_str[256];
        char pos_str[256];

        gpsd_sock.send( cmdtrack, strlen(cmdtrack) );
        result = gpsd_sock.recv( track_str, 256 );
        // printf("track = %d '%s'\n", result, track_str);

        gpsd_sock.send( cmdpos, strlen(cmdpos) );
        result = gpsd_sock.recv( pos_str, 256 );
        // printf("pos = %d '%s'\n", result, pos_str);

        double track_deg, lat_deg, lon_deg;

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

        if ( parse_track( track_str, &track_deg ) ) {
            double rad = SGD_PI_2 - track_deg * SGD_DEGREES_TO_RADIANS;
            double x = cos(rad);
            double y = sin(rad);
            printf("hdg = %0f  rad = %.2f  x = %.2f  y = %.2f\n",
                   track_deg, rad, x, y);
            filt_x = 0.9 * filt_x + 0.1 * x;
            filt_y = 0.9 * filt_y + 0.1 * y;

            track_filt_deg
                = 90 - atan2( filt_y, filt_x ) * SGD_RADIANS_TO_DEGREES;
            if ( track_filt_deg < 0 ) { track_filt_deg += 360.0; }
            printf( "current track = %.1f filtered track = %.1f\n",
                    track_deg, track_filt_deg );
        } else {
            printf( "no track\n" );
        }

        count--;
        printf("count = %d\n", count);
        if ( count < 0 ) {
            // send out home position update to UAS
            char uglink_cmd[256];
            snprintf(uglink_cmd, 256, "send home,%.8f,%.8f,0,%.1f\r\n",
                     lon_deg, lat_deg, track_filt_deg);
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

