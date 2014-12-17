#include <string.h>
#include <string>

#include <gps.h>

#include "comms/netSocket.h"
#include "math/SGMath.hxx"

using std::string;


const int update_interval = 30;                       // seconds

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

    struct gps_data_t gps_data;

    SGGeod loc, last_loc;
    double filt_x = 0.0;
    double filt_y = 0.0;
    double track_filt_deg = 0.0;
    int count = update_interval;

    bool good_pos = false;

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

    double track_deg = 0.0, lat_deg = 0.0, lon_deg = 0.0;

    while ( true ) {
        if ( gps_waiting (&gps_data, 500000) ) {
            if (gps_read (&gps_data) >= 0) {
                /* Display data from the GPS receiver. */
	        printf("read something from gps\n");
                if (gps_data.set) {
		    printf("sats used=%d fix=%d\n", 
			   gps_data.satellites_used, gps_data.fix.mode);
		    if ( gps_data.fix.mode > 1 ) {
			good_pos = true;
			if ( !isnan(gps_data.fix.longitude) ) {
			    lon_deg = gps_data.fix.longitude;
			} else {
			    good_pos = false;
			}
			if ( !isnan(gps_data.fix.latitude) ) {
			    lat_deg = gps_data.fix.latitude;
			} else {
			    good_pos = false;
			}
			if ( !isnan(gps_data.fix.track) ) {
			    track_deg = gps_data.fix.track;
			} else {
			    good_pos = false;
			}
			if ( good_pos ) {
		            printf(" pos=%.8f,%.8f trk=%.1f\n",
			           lon_deg, lat_deg, track_deg );
            		    double rad = SGD_PI_2 - 
			    	    track_deg * SGD_DEGREES_TO_RADIANS;
            		    double x = cos(rad);
            		    double y = sin(rad);
            		    printf("hdg = %0f  rad = %.2f  x = %.2f  y = %.2f\n",
                   		    track_deg, rad, x, y);
            		    filt_x = 0.9 * filt_x + 0.1 * x;
            		    filt_y = 0.9 * filt_y + 0.1 * y;

            		    track_filt_deg = 
				    90 - atan2( filt_y, filt_x ) * 
				    SGD_RADIANS_TO_DEGREES;
	                    if ( track_filt_deg < 0 ) { track_filt_deg += 360.0; }
            		    printf( "current track = %.1f filtered track = %.1f\n",
                                    track_deg, track_filt_deg );

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
		                	count = update_interval;
                		    } else {
                    			printf("error connecting to uglink\n");
                		    }
                		    uglink_sock.close();
            			} else {
                		    printf("error opening uglink socket\n");
            			}

        		    }
			}
		    }
                }
            }
        }

	// sleep(1);

    }
}

