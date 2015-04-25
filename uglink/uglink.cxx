#ifndef _MSC_VER
#  include <strings.h>		// for bzero()
#else
#  define bzero(a,b) memset(a,0,b)
#endif
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <string>

//#include <plib/net.h>
//#include <plib/sg.h>

#include <include/globaldefs.h>
#include <util/timing.h>
//#include <simgear/io/lowlevel.hxx> // endian tests
#include "sg_file.hxx"
//#include <simgear/math/sg_geodesy.hxx>
//#include <simgear/props/props.hxx>
//#include <simgear/timing/timestamp.hxx>

//#include <include/net_ctrls.hxx>
//#include <include/net_fdm.hxx>
//#include <include/net_gui.hxx>

#include "serial.hxx"
#include "messages.hxx"
#include "command.hxx"
#include "current.hxx"
#include "push_udp.hxx"
#include "telnet.hxx"
#include "websocket.hxx"

using std::cout;
using std::endl;
using std::string;


// ugear data
UGTrack track;

// Default path
static string infile = "";
static string flight_dir = "";
static string serialdev = "";
static string outfile = "";
static string fcs_debug_path = "";

// Master time counter
float sim_time = 0.0f;
double frame_us = 0.0f;

// sim control
double last_time_stamp;
double current_time_stamp;

// altitude offset
static SGPropertyNode *alt_offset_node = NULL;
static SGPropertyNode *est_controls_node = NULL;
static SGPropertyNode *use_groundtrack_hdg_node = NULL;
static SGPropertyNode *use_ground_speed_node = NULL;
static SGPropertyNode *flying_wing_node = NULL;
static SGPropertyNode *flight_total_timer = NULL;
static SGPropertyNode *flight_auto_timer = NULL;
static SGPropertyNode *flight_odometer = NULL;
static SGPropertyNode *extern_mah_node = NULL;

// skip initial seconds
double skip = 0.0;

// for gpx export
double last_lat = 0.0, last_lon = 0.0;

// command line options
bool run_real_time = true;

bool ignore_checksum = false;

bool sg_swap = false;

bool export_raw_umn = false;
string path_raw_umn = ".";

bool export_text_tab = false;
string path_text_tab = ".";

// estimate gps status
float gps_status = -1.0;

// dump a line of all the data (via printf) so we can cross correlate
// everything against everything.
static void dump_data_point( struct gps *gpspacket,
			     struct imu *imupacket,
			     struct airdata *airpacket,
			     struct filter *filterpacket,
			     struct actuator *actpacket,
			     struct pilot *pilotpacket,
			     struct apstatus *appacket,
			     struct health *healthpacket )
{
    printf( "[data] %.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%f\t%d\t%d",
	    gpspacket->timestamp /* 2 */,
	    gpspacket->lat, gpspacket->lon, gpspacket->alt,
	    gpspacket->vn, gpspacket->ve, gpspacket->vd,
	    gpspacket->gps_time, gpspacket->satellites, gpspacket->status );
    printf( "\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.3f\t%.3f\t%.3f\t%.1f\t%d",
	    imupacket->p /* 12 */, imupacket->q, imupacket->r,
	    imupacket->ax, imupacket->ay, imupacket->az,
	    imupacket->hx, imupacket->hy, imupacket->hz,
	    imupacket->temp, imupacket->status
	    );
    printf( "\t%.1f\t%.2f\t%.2f\t%.2f\t%d",
	    airpacket->airspeed /* 22 */, airpacket->altitude, airpacket->climb_fpm,
	    airpacket->acceleration, airpacket->status
	    );
    printf( "\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\t%d",
	    filterpacket->lat /* 27 */, filterpacket->lon, filterpacket->alt,
	    filterpacket->vn, filterpacket->ve, filterpacket->vd,
	    filterpacket->phi, filterpacket->theta, filterpacket->psi,
	    filterpacket->status
	    );
    printf( "\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d",
	    actpacket->ail /* 37 */, actpacket->ele, actpacket->thr, actpacket->rud,
	    actpacket->ch5, actpacket->ch6, actpacket->ch7, actpacket->ch8,
	    actpacket->status
	    );
    printf( "\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d",
	    pilotpacket->ail, pilotpacket->ele,
	    pilotpacket->thr, pilotpacket->rud,
	    pilotpacket->ch5, pilotpacket->ch6,
	    pilotpacket->ch7, pilotpacket->ch8,
	    pilotpacket->status
	    );
    printf( "\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\t%.10f\t%.10f\t%d\t%d",
	    appacket->target_heading_deg, appacket->target_roll_deg,
	    appacket->target_altitude_msl_ft, appacket->target_climb_fps,
	    appacket->target_pitch_deg, appacket->target_speed_kt,
	    appacket->target_wp, appacket->wp_lon, appacket->wp_lat,
	    appacket->wp_index, appacket->route_size
	    );
    printf("\n");
}

/*
 * Generate a date stamp from the current date/time
 */

string gen_date_stamp() {
    time_t t = time(NULL);
    struct tm *lt = localtime( &t );
    char stamp[32];
    snprintf(stamp, 32, "%04d-%02d-%02d--%02d:%02d:%02d",
	   lt->tm_year+1900, lt->tm_mon+1, lt->tm_mday,
	   lt->tm_hour, lt->tm_min, lt->tm_sec);
    return (string)stamp;
}


void dump_fcs_debug_record( SGFile &fcs_nav, SGFile &fcs_speed,
			    SGFile &fcs_alt, double current,
			    struct imu *imupacket,
			    struct filter *filterpacket,
			    struct airdata *airpacket,
			    struct apstatus *appacket,
			    struct actuator *actpacket )
{
    static double last_fcs_time = 0.0;
    static int max_buf = 256;
    char buf[max_buf];
    if ( current >= last_fcs_time + 0.1 ) {
	// nav
	double filter_hdg = (SGD_PI * 0.5 - atan2(filterpacket->vn, filterpacket->ve)) * SG_RADIANS_TO_DEGREES;
	snprintf(buf, max_buf, "%.2f\t%.1f\t%.1f\t%.1f\t%.1f\t%.2f\n",
		 imupacket->timestamp,
		 appacket->target_heading_deg,
		 appacket->target_roll_deg,
		 filter_hdg,
		 filterpacket->phi,
		 actpacket->ail);
	fcs_nav.writestring(buf);
	// speed
	snprintf(buf, max_buf, "%.2f\t%.1f\t%.1f\t%.1f\t%.1f\t%.2f\n",
		 imupacket->timestamp,
		 appacket->target_speed_kt,
		 appacket->target_pitch_deg,
		 airpacket->airspeed,
		 filterpacket->theta,
		 actpacket->ele);
	fcs_speed.writestring(buf);
	// alt
	snprintf(buf, max_buf, "%.2f\t%.1f\t%.1f\t%.2f\n",
		 imupacket->timestamp,
		 appacket->target_altitude_msl_ft,
		 filterpacket->alt,
		 actpacket->thr);
	fcs_alt.writestring(buf);

	last_fcs_time = current;
    }
}

void usage( const string &argv0 ) {
    cout << "Usage: " << argv0 << endl;
    cout << "\t[ --help ]" << endl;
    cout << endl;
    cout << "\t[ --infile <infile_name>" << endl;
    cout << "\t[ --flight <flight_dir>" << endl;
    cout << "\t[ --serial <dev_name>" << endl;
    cout << "\t[ --outfile <outfile_name> (capture the data to a file)" << endl;
    cout << "\t[ --fcs-debug <path>" << endl;
    cout << "\t[ --export-raw-umn <base_path> ]" << endl;
    cout << "\t[ --export-text-tab <base_path> ]" << endl;
    cout << endl;
    cout << "\t[ --hertz <hertz> ]" << endl;
    cout << "\t[ --host <hostname> ]" << endl;
    cout << "\t[ --broadcast ]" << endl;
    cout << "\t[ --ctrls-port <ctrls output port #> ]" << endl;
    cout << "\t[ --fdm-port <fdm output port #> ]" << endl;
    cout << "\t[ --gui-port <gui output port #> ]" << endl;
    cout << "\t[ --opengc-port <opengc output port #> ]" << endl;
    cout << "\t[ --telnet-port <telnet server port #> ]" << endl;
    cout << "\t[ --websocket-port <ws:// server port #> ]" << endl;
    cout << endl;
    cout << "\t[ --groundtrack-heading ]" << endl;
    cout << "\t[ --ground-speed ]" << endl;
    cout << "\t[ --estimate-control-deflections ]" << endl;
    cout << "\t[ --altitude-offset <meters> ]" << endl;
    cout << "\t[ --skip-seconds <seconds> ]" << endl;
    cout << "\t[ --no-real-time ]" << endl;
    cout << "\t[ --ignore-checksum ]" << endl;
    cout << "\t[ --sg-swap ]" << endl;
}


int main( int argc, char **argv ) {
    // initialize property system
    props = new SGPropertyNode;
    alt_offset_node = fgGetNode("/config/alt-offset-m", true);
    est_controls_node = fgGetNode("/config/estimate-controls", true);
    use_groundtrack_hdg_node = fgGetNode("/config/use-groundtrack-heading",
					 true);
    use_ground_speed_node = fgGetNode("/config/use-ground-speed", true);
    flying_wing_node = fgGetNode("/config/flying-wing-mode", true);

    // timers/counters for end of run reporting
    flight_total_timer = fgGetNode("/status/flight-timer-secs", true);
    flight_auto_timer = fgGetNode("/status/autopilot-timer-secs", true);
    flight_odometer = fgGetNode("/status/flight-odometer", true);
    extern_mah_node = fgGetNode("/status/extern-mah", true);

    // set some default values
    use_groundtrack_hdg_node->setBoolValue( false );
    flying_wing_node->setBoolValue( false );

    double hertz = 60.0;

    // Default ports
    out_host = "localhost";
    do_broadcast = false;
    ctrls_port = 5506;
    fdm_port = 5505;
    gui_port = 5504;
    openiris_port = 6001;

    int telnet_port = 5402;
    int ws_port = 31655;

    // process command line arguments
    for ( int i = 1; i < argc; ++i ) {
        if ( strcmp( argv[i], "--help" ) == 0 ) {
            usage( argv[0] );
            exit( 0 );
        } else if ( strcmp( argv[i], "--hertz" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                hertz = atof( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--infile" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                infile = argv[i];
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--flight" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                flight_dir = argv[i];
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--outfile" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                outfile = argv[i];
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--fcs-debug" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                fcs_debug_path = argv[i];
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--export-text-tab" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                path_text_tab = argv[i];
		export_text_tab = true;
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--serial" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                serialdev = argv[i];
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--host" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                out_host = argv[i];
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--broadcast" ) == 0 ) {
            do_broadcast = true;
        } else if ( strcmp( argv[i], "--ctrls-port" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                ctrls_port = atoi( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--fdm-port" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                fdm_port = atoi( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--gui-port" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                gui_port = atoi( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--openiris-port" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                openiris_port = atoi( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
	} else if ( strcmp( argv[i], "--telnet-port" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                telnet_port = atoi( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
	} else if ( strcmp( argv[i], "--websocket-port" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                ws_port = atoi( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp (argv[i], "--groundtrack-heading" ) == 0 ) {
            use_groundtrack_hdg_node->setBoolValue( true );
        } else if ( strcmp (argv[i], "--ground-speed" ) == 0 ) {
            use_ground_speed_node->setBoolValue( true );
        } else if (strcmp (argv[i], "--estimate-control-deflections" ) == 0) {
            est_controls_node->setBoolValue( true );
        } else if ( strcmp (argv[i], "--wing-controls" ) == 0) {
            flying_wing_node->setBoolValue( true );
        } else if ( strcmp ( argv[i], "--conventional-controls" ) == 0) {
            flying_wing_node->setBoolValue( false );
        } else if ( strcmp( argv[i], "--altitude-offset" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                alt_offset_node->setDoubleValue( atof( argv[i] ) );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
        } else if ( strcmp( argv[i], "--skip-seconds" ) == 0 ) {
            ++i;
            if ( i < argc ) {
                skip = atof( argv[i] );
            } else {
                usage( argv[0] );
                exit( -1 );
            }
	} else if ( strcmp( argv[i], "--no-real-time" ) == 0 ) {
            run_real_time = false;
	} else if ( strcmp( argv[i], "--ignore-checksum" ) == 0 ) {
            ignore_checksum = true;
	} else if ( strcmp( argv[i], "--sg-swap" ) == 0 ) {
            sg_swap = true;
        } else {
            usage( argv[0] );
            exit( -1 );
        }
    }

    // Setup up outgoing network connections

    netInit( &argc,argv ); // We must call this before any other net stuff

    udp_open_sockets();

    if ( sg_swap ) {
        track.set_stargate_swap_mode();
    }
    
    // open up the fcs debug files if requested
    string fcs_nav_path = "";
    string fcs_speed_path = "";
    string fcs_alt_path = "";
    if ( fcs_debug_path.length() ) {
	fcs_nav_path = fcs_debug_path + "/fcs-debug-nav.txt";
	fcs_speed_path = fcs_debug_path + "/fcs-debug-speed.txt";
	fcs_alt_path = fcs_debug_path + "/fcs-debug-alt.txt";
    }
    SGFile fcs_nav( fcs_nav_path );
    if ( fcs_nav_path.length() ) {
	if ( !fcs_nav.open( SG_IO_OUT ) ) {
	    cout << "Cannot open: " << fcs_nav_path << endl;
	    return false;
	}
    }
    SGFile fcs_speed( fcs_speed_path );
    if ( fcs_speed_path.length() ) {
	if ( !fcs_speed.open( SG_IO_OUT ) ) {
	    cout << "Cannot open: " << fcs_speed_path << endl;
	    return false;
	    }
    }
    SGFile fcs_alt( fcs_alt_path );
    if ( fcs_alt_path.length() ) {
	if ( !fcs_alt.open( SG_IO_OUT ) ) {
	    cout << "Cannot open: " << fcs_alt_path << endl;
	    return false;
	}
    }

    UGTelnet telnet( telnet_port );
    telnet.open();

    UGWebSocket ws( ws_port );
    ws.open();

    if ( infile.length() || flight_dir.length() ) {
        if ( infile.length() ) {
            // Load data from a stream log data file
            track.load_stream( infile, ignore_checksum );
        } else if ( flight_dir.length() ) {
            // Load data from a flight directory
            track.load_flight( flight_dir );
        }
        cout << "Loaded " << track.gps_size() << " gps records." << endl;
        cout << "Loaded " << track.imu_size() << " imu records." << endl;
	cout << "Loaded " << track.airdata_size() << " air data records." << endl;
        cout << "Loaded " << track.filter_size() << " filter records." << endl;
        cout << "Loaded " << track.act_size() << " actuator records." << endl;
        cout << "Loaded " << track.pilot_size() << " pilot records." << endl;
        cout << "Loaded " << track.ap_size() << " ap status records." << endl;
        cout << "Loaded " << track.health_size() << " health status records." << endl;
        cout << "Loaded " << track.payload_size() << " payload records." << endl;

	if ( export_raw_umn ) {
	    track.export_raw_umn( path_raw_umn );
	}

	if ( export_text_tab ) {
	    track.export_text_tab( path_raw_umn );
	}

        int size = track.filter_size();

        double current_time = track.get_filterpt(0).timestamp;
        cout << "Track begin time is " << current_time << endl;
        double end_time = track.get_filterpt(size-1).timestamp;
        cout << "Track end time is " << end_time << endl;
        cout << "Duration = " << end_time - current_time << endl;

        if ( track.gps_size() > 0 ) {
            double tmp = track.get_gpspt(track.gps_size()-1).gps_time;
            int days = (int)(tmp / (24 * 60 * 60));
            tmp -= days * 24 * 60 * 60;
            int hours = (int)(tmp / (60 * 60));
            tmp -= hours * 60 * 60;
            int min = (int)(tmp / 60);
            tmp -= min * 60;
            double sec = tmp;
            printf("*WRONG* [GPS  ]:ITOW= %.3f[sec]  %dd %02d:%02d:%06.3f\n",
                   tmp, days, hours, min, sec);
        }

        // advance skip seconds forward
        current_time += skip;

        frame_us = 1000000.0 / hertz;
        if ( frame_us < 0.0 ) {
            frame_us = 0.0;
        }

        double start_time = get_Time();
        int gps_count = 0;
        int imu_count = 0;
	int airdata_count = 0;
        int filter_count = 0;
        int act_count = 0;
        int pilot_count = 0;
        int ap_count = 0;
        int health_count = 0;
        int payload_count = 0;

        gps gps0, gps1;
        gps0 = gps1 = track.get_gpspt( 0 );
    
        imu imu0, imu1;
        imu0 = imu1 = track.get_imupt( 0 );
    
        airdata air0, air1;
        air0 = air1 = track.get_airdatapt( 0 );
    
        filter filter0, filter1;
        filter0 = filter1 = track.get_filterpt( 0 );
    
        actuator act0, act1;
        act0 = act1 = track.get_actpt( 0 );
    
        pilot pilot0, pilot1;
        pilot0 = pilot1 = track.get_pilotpt( 0 );
    
        apstatus ap0, ap1;
        ap0 = ap1 = track.get_appt( 0 );

        health health0, health1;
        health0 = health1 = track.get_healthpt( 0 );

	payload payload0, payload1;
        payload0 = payload1 = track.get_payloadpt( 0 );

	double last_lat = -999.9, last_lon = -999.9;

        printf("<gpx>\n");
        printf(" <trk>\n");
        printf("  <trkseg>\n");
        while ( current_time < end_time ) {
            // cout << "current_time = " << current_time << " end_time = "
            //      << end_time << endl;

            // Advance gps pointer
            while ( current_time > gps1.timestamp
                    && gps_count < track.gps_size() - 1 )
            {
                gps0 = gps1;
                ++gps_count;
                // cout << "count = " << count << endl;
                gps1 = track.get_gpspt( gps_count );
            }
            // cout << "p0 = " << p0.get_time() << " p1 = " << p1.get_time()
            //      << endl;

            // Advance imu pointer
            while ( current_time > imu1.timestamp
                    && imu_count < track.imu_size() - 1 )
            {
                imu0 = imu1;
                ++imu_count;
                // cout << "count = " << count << endl;
                imu1 = track.get_imupt( imu_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

            // Advance airdata pointer
            while ( current_time > air1.timestamp
                    && airdata_count < track.airdata_size() - 1 )
            {
                air0 = air1;
                ++airdata_count;
                // cout << "count = " << count << endl;
                air1 = track.get_airdatapt( airdata_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

            // Advance filter pointer
            while ( current_time > filter1.timestamp
                    && filter_count < track.filter_size() - 1 )
            {
                filter0 = filter1;
                ++filter_count;
                // cout << "filter count = " << filter_count << endl;
                filter1 = track.get_filterpt( filter_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

            // Advance actuator pointer
            while ( current_time > act1.timestamp
                    && act_count < track.act_size() - 1 )
            {
                act0 = act1;
                ++act_count;
                // cout << "count = " << count << endl;
                act1 = track.get_actpt( act_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

            // Advance pilot pointer
            while ( current_time > pilot1.timestamp
                    && pilot_count < track.pilot_size() - 1 )
            {
                pilot0 = pilot1;
                ++pilot_count;
                // cout << "count = " << count << endl;
                pilot1 = track.get_pilotpt( pilot_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

            // Advance ap status pointer
            while ( current_time > ap1.timestamp
                    && ap_count < track.ap_size() - 1 )
            {
                ap0 = ap1;
                ++ap_count;
                // cout << "count = " << count << endl;
                ap1 = track.get_appt( ap_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

            // Advance health status pointer
            while ( current_time > health1.timestamp
                    && health_count < track.health_size() - 1 )
            {
                health0 = health1;
                ++health_count;
                // cout << "count = " << count << endl;
                health1 = track.get_healthpt( health_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

             // Advance payload status pointer
            while ( current_time > payload1.timestamp
                    && payload_count < track.payload_size() - 1 )
            {
                payload0 = payload1;
                ++payload_count;
                // cout << "count = " << count << endl;
                payload1 = track.get_payloadpt( payload_count );
            }
            //  cout << "pos0 = " << pos0.get_seconds()
            // << " pos1 = " << pos1.get_seconds() << endl;

           double gps_percent;
            if ( fabs(gps1.timestamp - gps0.timestamp) < 0.00001 ) {
                gps_percent = 0.0;
            } else {
                gps_percent =
                    (current_time - gps0.timestamp) /
                    (gps1.timestamp - gps0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

            double imu_percent;
            if ( fabs(imu1.timestamp - imu0.timestamp) < 0.00001 ) {
                imu_percent = 0.0;
            } else {
                imu_percent =
                    (current_time - imu0.timestamp) /
                    (imu1.timestamp - imu0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

	    double air_percent;
            if ( fabs(air1.timestamp - air0.timestamp) < 0.00001 ) {
                air_percent = 0.0;
            } else {
                air_percent =
                    (current_time - air0.timestamp) /
                    (air1.timestamp - air0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

	    double filter_percent;
            if ( fabs(filter1.timestamp - filter0.timestamp) < 0.00001 ) {
                filter_percent = 0.0;
            } else {
                filter_percent =
                    (current_time - filter0.timestamp) /
                    (filter1.timestamp - filter0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

            double act_percent;
            if ( fabs(act1.timestamp - act0.timestamp) < 0.00001 ) {
                act_percent = 0.0;
            } else {
                act_percent =
                    (current_time - act0.timestamp) /
                    (act1.timestamp - act0.timestamp);
            }
            // cout << "actuator percent = " << act_percent << endl;

            double pilot_percent;
            if ( fabs(pilot1.timestamp - pilot0.timestamp) < 0.00001 ) {
                pilot_percent = 0.0;
            } else {
                pilot_percent =
                    (current_time - pilot0.timestamp) /
                    (pilot1.timestamp - pilot0.timestamp);
            }
            // cout << "pilot percent = " << pilot_percent << endl;

            double ap_percent;
            if ( fabs(ap1.timestamp - ap0.timestamp) < 0.00001 ) {
                ap_percent = 0.0;
            } else {
                ap_percent =
                    (current_time - ap0.timestamp) /
                    (ap1.timestamp - ap0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

            double health_percent;
            if ( fabs(health1.timestamp - health0.timestamp) < 0.00001 ) {
                health_percent = 0.0;
            } else {
                health_percent =
                    (current_time - health0.timestamp) /
                    (health1.timestamp - health0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

            double payload_percent;
            if ( fabs(payload1.timestamp - payload0.timestamp) < 0.00001 ) {
                payload_percent = 0.0;
            } else {
                payload_percent =
                    (current_time - payload0.timestamp) /
                    (payload1.timestamp - payload0.timestamp);
            }
            // cout << "Percent = " << percent << endl;

	    gps gpspacket = UGEARInterpGPS( gps0, gps1, gps_percent );
            imu imupacket = UGEARInterpIMU( imu0, imu1, imu_percent );
            airdata airpacket = UGEARInterpAIR( air0, air1, air_percent );
            filter filterpacket = UGEARInterpFILTER( filter0, filter1,
						     filter_percent );
            actuator actpacket = UGEARInterpACT( act0, act1, act_percent );
            pilot pilotpacket = UGEARInterpPILOT( pilot0, pilot1,
						  pilot_percent );
            apstatus appacket = UGEARInterpAP( ap0, ap1, ap_percent );
            health healthpacket = UGEARInterpHEALTH( health0, health1, health_percent );
            payload payloadpacket = UGEARInterpPAYLOAD( payload0, payload1, payload_percent );

            // cout << current_time << " " << p0.lat_deg << ", " << p0.lon_deg
            //      << endl;
            // cout << current_time << " " << p1.lat_deg << ", " << p1.lon_deg
            //      << endl;
            // cout << (double)current_time << " " << pos.lat_deg << ", "
            //      << pos.lon_deg << " " << att.yaw_deg << endl;

            if ( gpspacket.lat > -500 ) {
		double ground_speed_kts = sqrt(gpspacket.ve*gpspacket.ve + gpspacket.vn*gpspacket.vn) * SG_MPS_TO_KT;
#if 0
                printf( "%.3f %.6f %.6f %.1f %.6f %.6f %.1f %.2f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f\n",
                        current_time,
                        filterpacket.lat, filterpacket.lon, filterpacket.alt,
                        gpspacket.lat, gpspacket.lon, gpspacket.alt,
                        filterpacket.psi, filterpacket.theta, filterpacket.phi,
                        /* imupacket.Ps, */
			actpacket.thr,
			/* imupacket.Pt, */
			ground_speed_kts,
			filterpacket.ve, filterpacket.vn, filterpacket.vd,
			gpspacket.ve, gpspacket.vn, gpspacket.vd,
			derivedpacket.wind_deg,
			derivedpacket.wind_speed_kt);
#endif
		
#if 0
		double ail = pilotpacket.ail;
		double ele = pilotpacket.ele;
		if ( flying_wing_node->getBoolValue() ) {
		    double ch1 = pilotpacket.ail;
		    double ch2 = pilotpacket.ele;
		    double e = (ch1 - ch2) / 2.0;
		    double a = ch1 - e;
		    ele = e;
		    ail = a;
		}
		printf("TREND %.3f %.3f %.3f %.3f\n",
		       imupacket.p, imupacket.q, ail, ele);
#endif
                double dlat = last_lat - filterpacket.lat;
                double dlon = last_lon - filterpacket.lon;
                double dist = sqrt( dlat*dlat + dlon*dlon );
                if ( dist > 0.01 ) {
                    printf("   <trkpt lat=\"%.8f\" lon=\"%.8f\"></trkpt>\n",
                           filterpacket.lat, filterpacket.lon );
                    // printf(" </wpt>\n");
                    last_lat = filterpacket.lat;
                    last_lon = filterpacket.lon;
                }
	    }

            if ( (fabs(gpspacket.lat) < 0.0001 &&
                  fabs(gpspacket.lon) < 0.0001 &&
                  fabs(gpspacket.alt) < 0.0001) )
            {
                printf("WARNING: LOST GPS!!!\n");
                gps_status = -1.0;
            } else {
                gps_status = 1.0;
            }

	    command_mgr.update_cmd_sequence( filterpacket.command_seq,
					     filterpacket.timestamp );

	    compute_derived_data( &gpspacket, &imupacket, &airpacket,
				  &filterpacket, &actpacket, &pilotpacket,
				  &appacket, &healthpacket, &payloadpacket );

	    update_props( &gpspacket, &imupacket, &airpacket,
			  &filterpacket, &actpacket, &pilotpacket,
			  &appacket, &healthpacket, &payloadpacket );

            udp_send_data( &gpspacket, &imupacket, &airpacket, &filterpacket,
			   &actpacket, &pilotpacket, &appacket, &healthpacket );

            // dump_data_point( &gpspacket, &imupacket, &airpacket,
	    //                  &filterpacket, &actpacket, &pilotpacket,
	    //                  &appacket, &healthpacket );

	    // fcs debug data output @ 10hz if requested
	    if ( fcs_debug_path.length() ) {
		dump_fcs_debug_record( fcs_nav, fcs_speed, fcs_alt,
				       current_time_stamp,
				       &imupacket, &filterpacket, &airpacket,
				       &appacket, &actpacket );
	    }

            telnet.process();
            ws.process();

            if ( run_real_time ) {
                // Update the elapsed time.
                static bool first_time = true;
                if ( first_time ) {
                    last_time_stamp = get_Time();
                    first_time = false;
                }

                current_time_stamp = get_Time();
		double result = current_time_stamp - last_time_stamp;
		double elapsed_us = result * 1000000;
                if ( elapsed_us < (frame_us - 2000) ) {
                    double requested_us = (frame_us - elapsed_us) - 2000 ;
                    ulMilliSecondSleep ( (int)(requested_us / 1000.0) ) ;
                }
                current_time_stamp = get_Time();
		result = current_time_stamp - last_time_stamp;
		elapsed_us = result * 1000000;
		while ( elapsed_us < frame_us ) {
                    current_time_stamp = get_Time();
		    result = current_time_stamp - last_time_stamp;
		    elapsed_us = result * 1000000;
                }
            }

            current_time += (frame_us / 1000000.0);
            last_time_stamp = current_time_stamp;
        }

        printf("   <trkpt lat=\"%.8f\" lon=\"%.8f\"></trkpt>\n",
               filter1.lat, filter1.lon );

        printf("  </trkseg>\n");
        printf(" </trk>\n");
        filter0 = track.get_filterpt( 0 );
        filter1 = track.get_filterpt( track.filter_size() - 1 );
        printf(" <wpt lat=\"%.8f\" lon=\"%.8f\"></wpt>\n",
               filter0.lat, filter0.lon );
        printf(" <wpt lat=\"%.8f\" lon=\"%.8f\"></wpt>\n",
               filter1.lat, filter1.lon );
        printf("<gpx>\n");

	double total = current_time_stamp - start_time;
        cout << "Processed " << imu_count << " entries in "
             << total << " seconds." << endl;

	printf("Total Flight Time = %.1f min\n", flight_total_timer->getDoubleValue() / 60.0);
	printf("Total Autopilot Time = %.1f min\n", flight_auto_timer->getDoubleValue() / 60.0);
	printf("Estimated Battery Usage = %.0f Mah\n", extern_mah_node->getDoubleValue());
	printf("Estimated Distance Traveled = %.0f m %.2f nm\n", flight_odometer->getDoubleValue(), flight_odometer->getDoubleValue() * SG_METER_TO_NM );
    } else if ( serialdev.length() ) {
        // process incoming data from the serial port

        int count = 0;
        double packet_time = 0.0;
        double last_time = 0.0;

        gps gpspacket; bzero( &gpspacket, sizeof(gpspacket) );
	imu imupacket; bzero( &imupacket, sizeof(imupacket) );
	airdata airpacket; bzero( &airpacket, sizeof(airpacket) );
	filter filterpacket; bzero( &filterpacket, sizeof(filterpacket) );
	actuator actpacket; bzero( &actpacket, sizeof(actpacket) );
	pilot pilotpacket; bzero( &pilotpacket, sizeof(pilotpacket) );
	apstatus appacket; bzero( &appacket, sizeof(appacket) );
	health healthpacket; bzero( &healthpacket, sizeof(healthpacket) );
	payload payloadpacket; bzero( &payloadpacket, sizeof(payloadpacket) );

        double gps_time = 0.0;
        double imu_time = 0.0;
        double air_time = 0.0;
        double filter_time = 0.0;
        double act_time = 0.0;
        double pilot_time = 0.0;
        double ap_time = 0.0;
        double health_time = 0.0;
        // double command_time = 0.0;
        // double command_heartbeat = 0.0;
	double hb_time_stamp = 0.0;
	double command_time_stamp = 0.0;

        // open the serial port device
        SGSerialPort uavcom( serialdev, 115200, true );
        if ( !uavcom.is_enabled() ) {
            cout << "Cannot open: " << serialdev << endl;
            return false;
        }

        // open up the data log file if requested
        if ( !outfile.length() ) {
	    outfile = "flight-";
	    outfile += gen_date_stamp();
	    outfile += ".ugl";
            cout << "Using flight log file name = " << outfile << endl;
        }
        SGFile log( outfile );
        if ( !log.open( SG_IO_OUT ) ) {
            cout << "Cannot open: " << outfile << endl;
            return false;
        }

        // add some test commands
        //command_mgr.add("ap,alt,1000");
        //command_mgr.add("home,158.0,32.5");
        //command_mgr.add("go,home");
        //command_mgr.add("go,route");

	int id = -1;
        while ( uavcom.is_enabled() ) {

	    // cout << "looking for next message ..." << endl;
	    do {
		id = track.next_message( &uavcom, &log, &gpspacket,
					 &imupacket, &airpacket, &filterpacket,
					 &actpacket, &pilotpacket,
					 &appacket, &healthpacket,
					 &payloadpacket, ignore_checksum );
		// cout << "message id = " << id << endl;
		count++;

		if ( id == GPS_PACKET_V1 ) {
		    if ( gpspacket.timestamp > gps_time ) {
			gps_time = gpspacket.timestamp;
			packet_time = gps_time;
		    } else {
			cout << "oops gps back in time: " << gpspacket.gps_time << " " << gps_time << endl;
		    }
		} else if ( id == IMU_PACKET_V2 ) {
		    if ( imupacket.timestamp > imu_time ) {
			imu_time = imupacket.timestamp;
			packet_time = imu_time;
		    } else {
			cout << "oops imu back in time: " << imupacket.timestamp << " " << imu_time << endl;
		    }
		} else if ( id == AIR_DATA_PACKET_V4 ) {
		    if ( airpacket.timestamp > air_time ) {
			air_time = airpacket.timestamp;
			packet_time = air_time;
		    } else {
			cout << "oops airdata back in time: " << airpacket.timestamp << " " << air_time << endl;
		    }
		} else if ( id == FILTER_PACKET_V1 ) {
		    if ( filterpacket.timestamp > filter_time ) {
			filter_time = filterpacket.timestamp;
			packet_time = filter_time;
		    } else {
			cout << "oops filter back in time: " << filterpacket.timestamp << " " << filter_time << endl;
		    }
		} else if ( id == ACTUATOR_PACKET_V1 ) {
		    if ( actpacket.timestamp > act_time ) {
			act_time = actpacket.timestamp;
			packet_time = act_time;
		    } else {
			cout << "oops actuator back in time: " << actpacket.timestamp << " " << act_time << endl;
		    }
		} else if ( id == PILOT_INPUT_PACKET_V1 ) {
		    if ( pilotpacket.timestamp > pilot_time ) {
			pilot_time = pilotpacket.timestamp;
			packet_time = pilot_time;
		    } else {
			cout << "oops pilot back in time: " << pilotpacket.timestamp << " " << pilot_time << endl;
		    }
		} else if ( id == AP_STATUS_PACKET_V2) {
		    if ( appacket.timestamp > ap_time ) {
			ap_time = appacket.timestamp;
			packet_time = ap_time;
			/* printf("Received an ap status packet -- sequence: %d\n",
			   command_mgr.get_cmd_recv_index()); */
			
			/* printf("time=%.2f r=%.1f h=%.1f p=%.1f c=%.1f alt=%.1f\n",
			       appacket.timestamp,
			       appacket.target_roll_deg,
			       appacket.target_heading_deg,
			       appacket.target_pitch_deg,
			       appacket.target_climb_fps,
			       appacket.target_altitude_ft); */

			/* printf("wpt %d of %d: %.8f %.8f\n",
			       appacket.wp_index, appacket.route_size,
			       appacket.wp_lon, appacket.wp_lat);
			printf("target wpt = %d\n",
                               (int)appacket.target_wp); */
		    } else {
			cout << "oops ap back in time: " << appacket.timestamp << " " << ap_time << endl;
		    }
		} else if ( id == SYSTEM_HEALTH_PACKET_V3 ) {
		    if ( healthpacket.timestamp > health_time ) {
			health_time = healthpacket.timestamp;
			packet_time = health_time;
		    } else {
			cout << "oops ap back in time: " << healthpacket.timestamp << " " << health_time << endl;
		    }
		}
	    } while ( id >= 0 );

            if ( (packet_time > gps_time + 10) ||
                 (fabs(gpspacket.lat) < 0.0001 &&
                  fabs(gpspacket.lon) < 0.0001 &&
                  fabs(gpspacket.alt) < 0.0001) )
            {
		static double last_gps_warn = 0.0;
		if ( packet_time > last_gps_warn + 0.25 ) {
		    printf("WARNING: LOST GPS (Satellites=%d)!!!\n",
			   gpspacket.satellites);
                    printf("cur=%.3f gps=%.3f lat=%.8f lon=%.8f alt=%.1f\n",
                           packet_time, gps_time, gpspacket.lat, gpspacket.lon,
                           gpspacket.alt);
		    last_gps_warn = packet_time;
		}
                gps_status = -1.0;
            } else {
                gps_status = 1.0;
            }

            // Generate a ground station heart beat every 10 seconds (if no
	    // other message in the send queue)
	    double current = get_Time();
	    double result = current - hb_time_stamp;
	    double elapsed_sec = result;
            if ( (command_mgr.cmd_queue_empty())
		 && (elapsed_sec > 10.0) )
	    {
                command_mgr.add("hb");
                hb_time_stamp = get_Time();
            }
                
            // Command update @ 1hz
	    result = current - command_time_stamp;
	    elapsed_sec = result;
            if ( elapsed_sec > 1.0 ) {
                command_mgr.update(&uavcom);
                command_time_stamp = get_Time();
            }

            // Relay data on to FlightGear and LFSTech Glass
	    /* printf("current = %.6f  last_time + 1/hertz = %.6f\n",
	       current.toSecs(), last_time + (1/hertz)); */
            if ( current >= last_time + (1/hertz) ) {
                // if ( gpspacket.lat > -500 ) {
                int londeg = (int)filterpacket.lon;
                // double lonmin = fabs(filterpacket.lon - londeg);
                int latdeg = (int)filterpacket.lat;
                // double latmin = fabs(filterpacket.lat - latdeg);
                char londir = 'E'; if ( londeg < 0 ) londir = 'W';
                char latdir = 'N'; if ( latdeg < 0 ) latdir = 'S';
                londeg = abs(londeg);
                latdeg = abs(latdeg);
                /*printf( "%.2f  %c%02d:%.4f %c%03d:%.4f %.1f  %.2f %.2f %.2f\n",
                        current_time,
                        latdir, latdeg, latmin, londir, londeg, lonmin,
                        filterpacket.alt,
                        imupacket.phi*SGD_RADIANS_TO_DEGREES,
                        imupacket.the*SGD_RADIANS_TO_DEGREES,
                        imupacket.psi*SGD_RADIANS_TO_DEGREES ); */
                // }

		// printf("command_mgr.update_cmd_sequence = %.4f\n", current.toSecs());
	        command_mgr.update_cmd_sequence( filterpacket.command_seq,
					         current );

		compute_derived_data( &gpspacket, &imupacket, &airpacket,
				      &filterpacket, &actpacket, &pilotpacket,
				      &appacket, &healthpacket,
				      &payloadpacket );

		update_props( &gpspacket, &imupacket, &airpacket,
			      &filterpacket, &actpacket, &pilotpacket,
			      &appacket, &healthpacket, &payloadpacket );

                udp_send_data( &gpspacket, &imupacket, &airpacket,
			       &filterpacket, &actpacket, &pilotpacket,
			       &appacket, &healthpacket );

                last_time = current;
            }

	    // fcs debug data output @ 10hz if requested
	    if ( fcs_debug_path.length() ) {
		dump_fcs_debug_record( fcs_nav, fcs_speed, fcs_alt, current,
				       &imupacket, &filterpacket, &airpacket,
				       &appacket, &actpacket );
	    }
	    
            telnet.process();
            ws.process();
        }
    } else {
        cout << "No input source specified, nothing to do, exiting now."
             << endl;
    }

    return 0;
}
