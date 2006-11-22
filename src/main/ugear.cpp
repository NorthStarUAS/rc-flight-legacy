/*******************************************************************************
 * FILE: avionics.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * REVISED: 9/02/05 Jung Soon Jang
 * REVISED: 4/07/06 Jung Soon Jang
 *******************************************************************************/

#include <stdio.h>
#include <pthread.h>
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
#include "navigation/imugps.h"
#include "navigation/nav.h"
#include "util/timing.h"

#ifdef NCURSE_DISPLAY_OPTION
#include <ncurses/ncurses.h>
#endif   


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined defintions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NUM_THREADS	  4		 // number of thread to spawn by main
#define NETWORK_PORT      9001		 // network port number
#define UPDATE_USECS	  200000         // downlink at 5 Hz

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool wifi        = false;       // wifi connection enabled/disabled

//mutex and conditional variables
pthread_mutex_t	mutex_imu;
pthread_mutex_t	mutex_gps;
pthread_mutex_t mutex_nav;
pthread_cond_t  trigger_ahrs;
pthread_cond_t  trigger_nav;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//thread prototypes
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void	    timer_intr1(int sig);
void	    help_message();


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//main here...
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
    pthread_t 		threads[NUM_THREADS];
    pthread_attr_t	attr;
    struct sched_param	param;
    struct timespec	timeout;
    int 		rc[NUM_THREADS], tnum, iarg;
    static short	attempt = 0;
    struct itimerval    it;
    struct sigaction    sa;
    sigset_t            allsigs;
   
    /*********************************************************************
     *Parse the command line
     *********************************************************************/
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

#ifdef NCURSE_DISPLAY_OPTION    
    /*********************************************************************
     *ncurses setting for display
     *********************************************************************/
    int width,height;
 
    initscr();
    if (has_colors())
   	start_color();
    cbreak();
    curs_set(0);
    width = 70;
    height= 23;
    // Create a drawing window 
    win = newwin(height, width, (LINES - height) /2, (COLS - width) /2);
    if (win == NULL) {
   	endwin();
   	printf("ncurses window creation failed!...\n");
  	_exit(-1);
    }		
#endif

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

    // initialize mutex and condition variables
    pthread_mutex_init(&mutex_imu,NULL);
    pthread_mutex_init(&mutex_gps,NULL);
    pthread_mutex_init(&mutex_nav,NULL);
    pthread_cond_init(&trigger_ahrs,NULL);
    pthread_cond_init(&trigger_nav,NULL);

    //
    // create multiple threads
    //

    // setup the nice value for higher process priority
    // setpriority(PRIO_PROCESS, getpid(), -10);

    // initialize and set thread detached attribute
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    // set scheduling policy
    pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    sleep(2);

    // top priority
    param.sched_priority = sched_get_priority_max(SCHED_RR); 
    pthread_attr_setschedparam(&attr, &param);
    rc[0] = pthread_create(&threads[0], &attr, imugps_acq,  (void *)0);

    // top priority
    param.sched_priority -=  0;
    pthread_attr_setschedparam(&attr, &param);
    rc[1] = pthread_create(&threads[1], &attr, ahrs_thread, (void *)1);

    // priority - 5
    param.sched_priority -=  5;
    pthread_attr_setschedparam(&attr, &param);
    rc[2] = pthread_create(&threads[2], &attr, navigation, (void *)2);

    // priority - 10
    param.sched_priority -=  5;
    pthread_attr_setschedparam(&attr, &param);
    rc[3] = pthread_create(&threads[3], &attr, uplink_acq, (void *)3);

    for ( tnum = 0; tnum < NUM_THREADS; tnum++ ) {
        if ( rc[tnum] ) {
            printf("ERROR: return code from pthread_create() is %d\n",
                   rc[tnum]);
            _exit(-1);
        }
    }

    // time interval setting
    timeout.tv_sec = 0;
    timeout.tv_nsec = NSECS_PER_SEC / 10;

    it.it_interval.tv_sec  = 0;
    it.it_interval.tv_usec = UPDATE_USECS;
    it.it_value            = it.it_interval;

    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sa.sa_handler = timer_intr1;

    sigaction(SIGALRM, &sa, NULL);
    setitimer(ITIMER_REAL, &it, NULL);
    sigemptyset(&allsigs);

    // open networked ground station client
    if ( wifi ) retvalsock = open_client();

    // init battery status monitor
    sgbatmon_init();

    //
    // main loop
    //

    while (1) {
        sigsuspend(&allsigs);
  
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
        
        if ( display_on ) {
            display_message(&imupacket, &gpspacket, &navpacket, 5);
            printf("Battery voltage = %.2f  Estimated remaining = %d sec\n",
                   batmonpacket.volts, batmonpacket.est_seconds);
        }
    } // end main loop

    //
    // close
    //

#ifdef NCURSE_DISPLAY_OPTION   
    endwin();
#endif
   
    pthread_mutex_destroy(&mutex_imu);
    pthread_mutex_destroy(&mutex_gps);
    pthread_mutex_destroy(&mutex_nav);
   
    pthread_attr_destroy(&attr);
    pthread_cond_destroy(&trigger_ahrs);
    pthread_exit(NULL);
}


void timer_intr1(int sig)
{
    return;
}


//
// help message
//
void help_message()
{
    printf("\n./avionics -option1 -option2 ... \n");
    printf("--wifi on/off        : enable or disable WiFi communication with GS \n");
    printf("--log-file on/off    : enable or disable datalogging in /mnt/cf1/ \n");	
    printf("--console-link on/off: enable or disable serial/console link\n");	
    printf("--display on/off     : enable or disable dumping data to display \n");	
    printf("--ip xxx.xxx.xxx.xxx : set GS i.p. address for WiFi comm. \n");
    printf("--help               : display the help messages \n\n");
    
    _exit(0);	
}	
