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

#include "comms/console_link.h"
#include "comms/groundstation.h"
#include "comms/logging.h"
#include "comms/uplink.h"
#include "health/sgbatmon.h"
#include "include/globaldefs.h"
#include "navigation/ahrs.h"
#include "navigation/mnav.h"
#include "navigation/nav.h"
#include "util/timing.h"

#ifdef NCURSE_DISPLAY_OPTION
#include <ncurses/ncurses.h>
#endif   


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined defintions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NETWORK_PORT      9001		 // network port number
#define UPDATE_USECS	  200000         // downlink at 5 Hz

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool wifi        = false;       // wifi connection enabled/disabled

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//prototypes
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void	    help_message();


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//main here...
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
    int tnum, iarg;
    static short	attempt = 0;
   
    // Parse the command line
    for ( iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--log-file" )  ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) log_to_file = true;
            if ( !strcmp(argv[iarg], "off") ) log_to_file = false;
        } else if ( !strcmp(argv[iarg], "--console-link" )  ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) console_link_on = true;
            if ( !strcmp(argv[iarg], "off") ) console_link_on = false;
        } else if ( !strcmp(argv[iarg], "--wifi") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) wifi = true;
            if ( !strcmp(argv[iarg], "off") ) wifi = false;
        } else if ( !strcmp(argv[iarg],"--display") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) display_on = true;
            if ( !strcmp(argv[iarg], "off") ) display_on = false;
        } else if ( !strcmp(argv[iarg], "--ip") ) {
            ++iarg;
            HOST_IP_ADDR = argv[iarg];
        } else if ( !strcmp(argv[iarg], "--help") ) {
            help_message();
        } else {
            printf("Unknown option \"%s\"\n", argv[iarg]);
            help_message();
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

    // Initialize the NAV code.  Must be called before nav_update() or
    // nav_close()
    nav_init();

    // Initialize the communcation channel with the MNAV
    mnav_init();

    // open networked ground station client
    if ( wifi ) retvalsock = open_client();

    // init battery status monitor
    sgbatmon_init();

    //
    // main loop
    //

    while ( true ) {
        mnav_update();

        // telemetry
        if ( wifi ) {
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

        // health status
        sgbatmon_update();
	if ( log_to_file ) {
            log_health( &healthpacket );
	}

        if ( display_on ) {
            display_message( &imupacket, &gpspacket, &navpacket,
                             &servopacket, &healthpacket, 5 );
        }
    } // end main loop

    // close and exit
    ahrs_close();
    nav_close();
    mnav_close();
}


//
// help message
//
void help_message()
{
    printf("\n./ugear --option1 on/off --option2 on/off --option3 ... \n");
    printf("--wifi on/off        : enable or disable WiFi communication with GS \n");
    printf("--log-file on/off    : enable or disable datalogging in /mnt/cf1/ \n");	
    printf("--console-link on/off: enable or disable serial/console link\n");	
    printf("--display on/off     : enable or disable dumping data to display \n");	
    printf("--ip xxx.xxx.xxx.xxx : set GS i.p. address for WiFi comm. \n");
    printf("--help               : display the help messages \n\n");
    
    _exit(0);	
}	
