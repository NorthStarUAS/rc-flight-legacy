/*******************************************************************************
 * FILE: ugear.cpp
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * REVISED: 9/02/05 Jung Soon Jang
 * REVISED: 4/07/06 Jung Soon Jang
 *******************************************************************************/

#include <stdio.h>
#include <sys/types.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/resource.h>
#include <unistd.h>

#include <string>

#include "comms/console_link.h"
#include "comms/groundstation.h"
#include "comms/logging.h"
#include "comms/uplink.h"
#include "control/control.h"
#include "control/route_mgr.hxx"
#include "health/health.h"
#include "include/globaldefs.h"
#include "navigation/ahrs.h"
#include "navigation/mnav.h"
#include "navigation/nav.h"
#include "props/props.hxx"
#include "props/props_io.hxx"
#include "util/exception.hxx"
#include "util/myprof.h"
#include "util/sg_path.hxx"
#include "util/timing.h"

using std::string;


//
// #defines
//
#define NETWORK_PORT      9001		 // network port number
#define UPDATE_USECS	  200000         // downlink at 5 Hz


//
// global variables & objects
//

FGRouteMgr route_mgr;           // route manager object


//
// usage message
//
void usage()
{
    printf("\n./ugear --option1 on/off --option2 on/off --option3 ... \n");
    printf("--log-dir path       : enable onboard data logging to path\n");
    printf("--log-servo in/out   : specify which servo data to log (out=default)\n");
    printf("--mnav <device>      : specify mnav communication device\n");
    printf("--console <dev>      : specify console device and enable link\n");
    printf("--display on/off     : dump periodic data to display\n");	
    printf("--wifi on/off        : enable or disable WiFi communication with GS \n");
    printf("--ip xxx.xxx.xxx.xxx : set GS i.p. address for WiFi comm\n");
    printf("--help               : display this help messages\n\n");
    
    _exit(0);	
}	


//
// main ...
//
int main( int argc, char **argv )
{
    int iarg;
    static short	attempt = 0;
    bool log_servo_out  = true;    // log outgoing servo commands by default
    bool enable_control = false;   // autopilot control module enabled/disabled
    bool enable_nav     = false;   // nav filter enabled/disabled
    bool enable_route   = false;   // route module enabled/disabled
    bool wifi           = false;   // wifi connection enabled/disabled

    // initialize properties
    props = new SGPropertyNode;

    string root = ".";
    SGPropertyNode *root_node = fgGetNode("/config/root-path", true);
    root_node->setStringValue( root.c_str() );

    // load master config file
    SGPath master( root );
    master.append( "config.xml" );
    try {
        readProperties( master.c_str(), props);
        printf("Loaded configuration from %s\n", master.c_str());
    } catch (const sg_exception &exc) {
        printf("\n");
        printf("*** Cannot load master config file: %s\n", master.c_str());
        printf("\n");
        sleep(1);
    }

    // extract configuration values from the property tree (which is
    // now populated with the master config.xml data.  Do this before
    // the command line processing so that any options specified on
    // the command line will override what is in the config.xml file.

    SGPropertyNode *p;

    p = fgGetNode("/config/data/log-path", true);
    log_path.set( p->getStringValue() );
    p = fgGetNode("/config/data/enable", true);
    log_to_file = p->getBoolValue();
    printf("log path = %s enabled = %d\n", log_path.c_str(), log_to_file);

    p = fgGetNode("/config/mnav/device", true);
    strncpy( mnav_dev, p->getStringValue(), MAX_MNAV_DEV );

    p = fgGetNode("/config/console/device", true);
    strncpy( console_dev, p->getStringValue(), MAX_CONSOLE_DEV );

    p = fgGetNode("/config/nav-filter/enable", true);
    enable_nav = p->getBoolValue();

    p = fgGetNode("/config/autopilot/enable", true);
    enable_control = p->getBoolValue();

    p = fgGetNode("/config/route/enable", true);
    enable_route = p->getBoolValue();

    // Parse the command line
    for ( iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--log-dir" )  ) {
            ++iarg;
            log_path.set( argv[iarg] );
            log_to_file = true;
        } else if ( !strcmp(argv[iarg],"--log-servo") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "out") ) log_servo_out = true;
            if ( !strcmp(argv[iarg], "in") ) log_servo_out = false;
        } else if ( !strcmp(argv[iarg], "--mnav" )  ) {
            ++iarg;
            strncpy( mnav_dev, argv[iarg], MAX_MNAV_DEV );
        } else if ( !strcmp(argv[iarg], "--console" )  ) {
            ++iarg;
            strncpy( console_dev, argv[iarg], MAX_CONSOLE_DEV );
	    console_link_on = true;
        } else if ( !strcmp(argv[iarg],"--display") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) display_on = true;
            if ( !strcmp(argv[iarg], "off") ) display_on = false;
        } else if ( !strcmp(argv[iarg], "--wifi") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) wifi = true;
            if ( !strcmp(argv[iarg], "off") ) wifi = false;
        } else if ( !strcmp(argv[iarg], "--ip") ) {
            ++iarg;
            HOST_IP_ADDR = argv[iarg];
        } else if ( !strcmp(argv[iarg], "--help") ) {
            usage();
        } else {
            printf("Unknown option \"%s\"\n", argv[iarg]);
            usage();
        }
    }

    // open console link if requested
    if ( console_link_on ) {
        console_link_init();
    }

    // open logging files if requested
    if ( log_to_file ) {
        bool result = logging_init();
        if ( !result ) {
            printf("Warning: error opening one or more data files, logging disabled\n");
            log_to_file = false;
        }
    }

    // Initialize AHRS code.  Must be called before ahrs_update() or
    // ahrs_close()
    ahrs_init();

    if ( enable_nav ) {
        // Initialize the NAV code.  Must be called before nav_update() or
        // nav_close()
        nav_init();
    }

    // Initialize the communcation channel with the MNAV
    mnav_init();

    // init system health and status monitor
    health_init();

    // open networked ground station client
    if ( wifi ) retvalsock = open_client();

    if ( enable_control ) {
        // initialize the autopilot
        control_init();
    }

    if ( enable_route ) {
        // initialize the route manager
        route_mgr.init();
    }

    //
    // Main loop.  The mnav_update() command blocks on MNAV sensor
    // data which is spit out at precisely 50hz.  So this loop will
    // run at 50 hz and we can time all the other functions based off
    // that update rate.
    //

    int nav_counter = 0;
    int health_counter = 0;
    int display_counter = 0;
    int wifi_counter = 0;
    int ap_counter = 0;
    int route_counter = 0;
    int flush_counter = 0;

    printf("Everything inited ... ready to run\n");

    while ( true ) {
        // upate timing counters
        health_counter++;
        display_counter++;
        wifi_counter++;
	ap_counter++;
        route_counter++;
        if ( enable_nav ) {
            nav_counter++;
	}
        flush_counter++;

        // fetch the next data packet from the MNAV sensor.  This
        // function will then call the ahrs_update() function as
        // appropriate to compute the attitude estimate.
	mnav_prof.start();
        mnav_update();
	mnav_prof.stop();

	if ( enable_nav ) {
	  // navigation (update at 10hz.)  compute a location estimate
	  // based on gps and accelerometer data.
	  if ( nav_counter >= 5 && gpspacket.err_type != no_gps_update ) {
	    nav_counter = 0;
	    nav_prof.start();
	    nav_update();
	    nav_prof.stop();
	  }
	}

        if ( enable_route ) {
            // route updates at 5 hz
            if ( route_counter >= 10 ) {
                route_counter = 0;
                route_mgr_prof.start();
                route_mgr.update();
                route_mgr_prof.stop();
            }
        }

	if ( enable_control ) {
            // autopilot update at 25 hz
            if ( ap_counter >= 2 ) { 
                ap_counter = 0;
                control_prof.start();
                control_update(0);
                control_prof.stop();
            }
	}

        if ( console_link_on ) {
            if ( log_servo_out ) {
                console_link_servo( &servo_out );
            } else {
                console_link_servo( &servo_in );
            }
        }

        if ( log_to_file ) {
            if ( log_servo_out ) {
                log_servo( &servo_out );
            } else {
                log_servo( &servo_in );
            }
        }

        // health status (update at 0.1hz)
        if ( health_counter >= 500 ) {
            health_counter = 0;
	    health_prof.start();
            health_update();
            if ( log_to_file ) {
                log_health( &healthpacket );
            }
	    if ( console_link_on ) {
	      console_link_health( &healthpacket );
	    }
	    health_prof.stop();
        }

        // telemetry (update at 5hz)
        if ( wifi && wifi_counter >= 10 ) {
            wifi_counter = 0;
            if ( retvalsock ) {
                send_client();
                if ( display_on ) snap_time_interval("TCP",  5, 2);
            } else {
                //attempt connection every 2.0 sec
                if ( attempt++ == 10 ) { 
                    close_client(); 
                    retvalsock = open_client();
                    attempt = 0;
                }
            }        
        }

        // sensor summary dispay (update at 0.5hz)
        if ( display_on && display_counter >= 100 ) {
            display_counter = 0;
            display_message( &imupacket, &gpspacket, &navpacket,
                             &servo_in, &healthpacket );
	    mnav_prof.stats   ( "MNAV" );
	    ahrs_prof.stats   ( "AHRS" );
	    if ( enable_nav ) {
	      nav_prof.stats    ( "NAV " );
	      nav_alg_prof.stats    ( "NAVA" );
	    }
	    if ( enable_control ) {
	      control_prof.stats( "CTRL" );
	    }
	    health_prof.stats ( "HLTH" );
        }

        // round robin flushing of logging streams (update at 0.5hz)
        if ( flush_counter >= 100 ) {
            flush_counter = 0;
            static int flush_state = 0;
            if ( log_to_file ) {
                switch ( flush_state ) {
                case 0:
                    flush_gps();
                    break;
                case 1:
                    flush_imu();
                    break;
                case 2:
                    flush_nav();
                    break;
                case 3:
                    flush_servo();
                    break;
                case 4:
                    flush_health();
                    break;
                default:
                    flush_state = 0;
                }
                flush_state++;
            }
        }

    } // end main loop

    // close and exit
    ahrs_close();
    mnav_close();
    if ( enable_nav ) {
      nav_close();
    }
    if ( enable_control ) {
      control_close();
    }
}


