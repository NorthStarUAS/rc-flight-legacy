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

#include "actuators/act_mgr.h"
#include "adns/mnav/ahrs.h"
#include "adns/mnav/nav.h"
#include "comms/console_link.h"
#include "comms/groundstation.h"
#include "comms/logging.h"
#include "comms/uplink.h"
#include "control/control.h"
#include "control/route_mgr.hxx"
#include "health/health.h"
#include "include/globaldefs.h"
#include "props/props.hxx"
#include "props/props_io.hxx"
#include "sensors/gps_mgr.h"
#include "sensors/imu_mgr.h"
#include "sensors/mnav.h"
#include "util/exception.hxx"
#include "util/myprof.h"
#include "util/netSocket.h"
#include "util/sg_path.hxx"
#include "util/timing.h"

using std::string;


//
// #defines
//
#define NETWORK_PORT      9001		 // network port number
#define UPDATE_USECS	  200000         // downlink at 5 Hz

//
// Configuration settings


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
    bool log_servo_out  = true;    // log outgoing servo commands by default
    bool enable_control = false;   // autopilot control module enabled/disabled
    bool enable_nav     = false;   // nav filter enabled/disabled
    bool enable_route   = false;   // route module enabled/disabled
    bool wifi           = false;   // wifi connection enabled/disabled
    bool initial_home   = false;   // initial home position determined
    double gps_timeout_sec = 9.0; // nav algorithm gps timeout
    double lost_link_sec = 59.0;   // lost link timeout

    // initialize network library
    netInit( NULL, NULL );

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

    p = fgGetNode("/config/nav-filter/enable", true);
    enable_nav = p->getBoolValue();

    p = fgGetNode("/config/adns/gps-timeout-sec");
    if ( p != NULL && p->getDoubleValue() > 0.0001 ) {
	// stick with the default if nothing valid specified
	gps_timeout_sec = p->getDoubleValue();
    }
    printf("navigation filter enabled = %d  gps timeout = %.1f\n",
	   enable_nav, gps_timeout_sec);

    p = fgGetNode("/config/remote-link/lost-link-timeout-sec");
    if ( p != NULL && p->getDoubleValue() > 0.0001 ) {
	// stick with the default if nothing valid specified
	lost_link_sec = p->getDoubleValue();
    }
    printf("lost link timeout = %.1f\n", lost_link_sec);
    
    p = fgGetNode("/config/fcs/enable", true);
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
	    p = fgGetNode("/config/sensors/mnav/device", true);
	    p->setStringValue( argv[iarg] );
        } else if ( !strcmp(argv[iarg], "--console" )  ) {
            ++iarg;
	    p = fgGetNode("/config/remote-link/device", true);
	    p->setStringValue( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--display") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) display_on = true;
            if ( !strcmp(argv[iarg], "off") ) display_on = false;
        } else if ( !strcmp(argv[iarg], "--wifi") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) wifi = true;
            if ( !strcmp(argv[iarg], "off") ) wifi = false;
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
    mnav_ahrs_init( NULL );

    if ( enable_nav ) {
        // Initialize the NAV code.  Must be called before nav_update() or
        // nav_close()
        mnav_nav_init();
    }

    // Initialize communication with the selected IMU
    IMU_init();

    // Initialize communication with the selected GPS
    GPS_init();

    // init system health and status monitor
    health_init();

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
    int command_counter = 0;
    int flush_counter = 0;
    bool read_command = false;
    double last_command_time = 0.0;
    double current_time = 0.0;

    printf("Everything inited ... ready to run\n");

    // Notice: the main loop runs at 50hz synced to the IMU data input.
    while ( true ) {
        current_time = get_Time();

        // upate timing counters
        if ( enable_nav ) {
            nav_counter++;
	}
        health_counter++;
        display_counter++;
        wifi_counter++;
	ap_counter++;
        route_counter++;
        command_counter++;
        flush_counter++;

        // Fetch the next data packet from the IMU.
	//
	// This is a blocking routine.  It waits for the next output
        // of the IMU.  The assumption is that the IMU will spit out
        // data at a regular interval and thus control the
        // timing/syncronization of this entire application.
	IMU_update();

	// Run the AHRS algorithm.
	mnav_ahrs_update();

	mnav_imu_update();

	// Fetch GPS data if available.
	GPS_update();

	mnav_manual_override_check();

	if ( enable_nav && nav_counter >= 5 ) {
            // navigation (update at 10hz.)  compute a location estimate
            // based on gps and accelerometer data.
	    nav_counter = 0;

	    mnav_nav_update();

	    // check gps data age.  The nav filter continues to run,
	    // but the results are marked as NotValid if the most
	    // recent gps data becomes too old.
	    if ( GPS_age() > gps_timeout_sec ) {
		SGPropertyNode *nav_status_node
		    = fgGetNode("/status/navigation", true);
		nav_status_node->setStringValue("invalid");
	    }

	    // initial home is set to the location where the gps first
	    // comes alive
	    if ( !initial_home && GPS_age() < 2.0 ) {
		SGPropertyNode *gps_lat_node
		    = fgGetNode("/sensors/gps/latitude-deg", true);
		SGPropertyNode *gps_lon_node
		    = fgGetNode("/sensors/gps/longitude-deg", true);
		SGWayPoint wp( gps_lon_node->getDoubleValue(),
			       gps_lat_node->getDoubleValue(),
			       -9999.9 );
		if ( route_mgr.update_home(wp, 0.0, true /* force update */) ) {
		    initial_home = true;
		}
            }
	}

        if ( console_link_on ) {
            // check for incoming command data (5hz)
            if ( command_counter > 10 ) {
                command_counter = 0;
                if ( console_link_command() ) {
                    read_command = true;
                    last_command_time = current_time;
                    // FIXME: we shouldn't necessarily assume route
                    // mode just because we read a command from the
                    // ground station
                    route_mgr.set_route_mode();
                }
            }
            if ( read_command
                 && current_time > last_command_time + lost_link_sec
                 && route_mgr.get_route_mode() != FGRouteMgr::GoHome )
            {
                // We have previously established a positive link with
                // the groundstation, but it's been lost_link seconds
                // since the last command received and we aren't
                // already in GoHome mode.  Console link is assumed to
                // be down or we've flown out of radio modem range.
                // Switch to fly home mode.  Ground station operator
                // will need to send a resume route command to resume
                // the route.
                route_mgr.set_home_mode();
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

        // health status (update at 1hz)
        if ( health_counter >= 50 ) {
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

        // sensor summary dispay (update at 0.5hz)
        if ( display_on && display_counter >= 100 ) {
            display_counter = 0;
            display_message( &imupacket, &servo_in, &healthpacket );
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
    mnav_ahrs_close();
    mnav_close();
    GPS_close();
    if ( enable_nav ) {
	mnav_nav_close();
    }
    if ( enable_control ) {
	control_close();
    }
}


