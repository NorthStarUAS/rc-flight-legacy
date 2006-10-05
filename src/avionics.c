/*****************************************************************************************
* FILE: avionics.c
* DESCRIPTION:
*   
*   
*
* SOURCE: 
* REVISED: 9/02/05 Jung Soon Jang
* REVISED: 4/07/06 Jung Soon Jang
******************************************************************************************/
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/resource.h>
#include "globaldefs.h"

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
int     sock_fd;
short   wifi          = 1;		  //wifi is enabled
short	retvalsock    = 0;
short	screen_on     = 0;	          //data logging is disabled	
char    *HOST_IP_ADDR = "192.168.11.101"; //default ground station IP address
char    buf_err[50];

#ifdef NCURSE_DISPLAY_OPTION
WINDOW  *win;
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//thread prototypes
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern void *ahrs_main(void *thread_id);
extern void *imugps_acq(void *thread_id);
extern void *navigation(void *thread_id);
extern void *uplink_acq(void *thread_id);

extern void display_message(struct imu *data, struct gps *gdata, struct nav *ndata, int id);
short	    open_client();
void        send_client();
void	    timer_intr1(int sig);
void	    help_message();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//main here...
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
main(int argc, char **argv)
{
   pthread_t 		threads[NUM_THREADS];
   pthread_attr_t	attr;
   struct sched_param	param;
   struct timespec	timeout;
   int 			rc[NUM_THREADS],tnum,rout,iarg;
   static short		try = 0;
   struct itimerval     it;
   struct sigaction     sa;
   sigset_t             allsigs;
   short		disp_on=1;
   
   /*********************************************************************
    *Parse the command line
    *********************************************************************/
   for(iarg=1;iarg<argc;iarg++)
   {
	if(!strcmp(argv[iarg],"-save")  ) {
	   if(!strcmp(argv[iarg+1], "on")) screen_on  = 1;
	   if(!strcmp(argv[iarg+1],"off")) screen_on  = 0;
	}
	if(!strcmp(argv[iarg],"-wifi")  ) {
	   if(!strcmp(argv[iarg+1], "on")) wifi  = 1;
	   if(!strcmp(argv[iarg+1],"off")) wifi  = 0;
	}
	if(!strcmp(argv[iarg],"-screen")  ) {
	   if(!strcmp(argv[iarg+1], "on")) disp_on  = 1;
	   if(!strcmp(argv[iarg+1],"off")) disp_on  = 0;
	}
        if(!strcmp(argv[iarg],"-ip"    ))  HOST_IP_ADDR = argv[iarg+1];
        if(!strcmp(argv[iarg],"-help")) help_message();
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
       
   /*********************************************************************
    *initialize mutex and conditional variables
    *********************************************************************/
   pthread_mutex_init(&mutex_imu,NULL);
   pthread_mutex_init(&mutex_gps,NULL);
   pthread_mutex_init(&mutex_nav,NULL);
   pthread_cond_init(&trigger_ahrs,NULL);
   pthread_cond_init(&trigger_nav,NULL);

   /*********************************************************************
    *create multiple threads
    *********************************************************************/
   /*setup the nice value for higher process priority */
   //setpriority(PRIO_PROCESS, getpid(), -10);
   /*initialize and set thread detached attribute */
   pthread_attr_init(&attr);
   pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
   /*set scheduling policy */
   pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&attr, SCHED_RR);
   /*set thread priority */
   sleep(2);
   printf("Creating threads...\n");
   param.sched_priority = sched_get_priority_max(SCHED_RR); 
   pthread_attr_setschedparam(&attr, &param);
   rc[0] = pthread_create(&threads[0], &attr, ahrs_main,  (void *)0);
   
   param.sched_priority -=  0;
   pthread_attr_setschedparam(&attr, &param);
   rc[1] = pthread_create(&threads[1], &attr, imugps_acq, (void *)1);
      
   param.sched_priority -=  5; pthread_attr_setschedparam(&attr, &param);
   rc[2] = pthread_create(&threads[2], &attr, navigation, (void *)2);

   param.sched_priority -=  5; pthread_attr_setschedparam(&attr, &param);
   rc[3] = pthread_create(&threads[3], &attr, uplink_acq, (void *)3);

   for(tnum=0;tnum<NUM_THREADS;tnum++) {
      if (rc[tnum]){
            printf("ERROR: return code from pthread_create() is %d\n", rc[tnum]);
            _exit(-1);
      }
   }

   
   /*time interval setting */
   timeout.tv_sec = 0;
   timeout.tv_nsec= NSECS_PER_SEC/10;

   it.it_interval.tv_sec = 0;
   it.it_interval.tv_usec= UPDATE_USECS;
   it.it_value           = it.it_interval;

   sigemptyset(&sa.sa_mask);
   sa.sa_flags = 0;
   sa.sa_handler = timer_intr1;

   sigaction(SIGALRM, &sa, NULL);
   setitimer(ITIMER_REAL, &it, NULL);
   sigemptyset(&allsigs);

   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //open client
   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if (wifi == 1) retvalsock = open_client();

   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //main-loop
   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   while (1)
   {
      //rout = nanosleep(&timeout,NULL);
      sigsuspend(&allsigs); rout = 0;
  
      if (rout == 0) //timout is expired ...
      {
          //telemetry
          if (wifi == 1)
          {
             if (retvalsock) { send_client(); snap_time_interval("TCP",  5, 2); }
             else {
                //try connection every 2.0 sec
                if (try++ == 10) { 
                   close(sock_fd); 
                   retvalsock = open_client(); try = 0;
                }
             }        
          }
          if (disp_on) display_message(&imupacket, &gpspacket, &navpacket, 5);
          
      }
      else { }

   } //end while

  /**********************************************************************
   * close
   **********************************************************************/
#ifdef NCURSE_DISPLAY_OPTION   
   endwin();
#endif
   
   pthread_mutex_destory(&mutex_imu);
   pthread_mutex_destory(&mutex_gps);
   pthread_mutex_destory(&mutex_nav);
   
   pthread_attr_destory(&attr);
   pthread_cond_destory(&trigger_ahrs);
   pthread_exit(NULL);
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * open client to transmit/receive the packet to/from ground station
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
short open_client ()
{
  struct sockaddr_in serv_addr;
  struct timeval     tval;
  short  ret;
  int    flags;
  fd_set rset,wset;

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family      = AF_INET; 
  serv_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
  serv_addr.sin_port        = htons(NETWORK_PORT); 
  sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  
  //make a nonblocking connection
  flags = fcntl(sock_fd,F_GETFL,0);
  fcntl(sock_fd,F_SETFL,flags | O_NONBLOCK);
  
  //printf("uNAV CLIENT: Starting to connect to server.\n");
  if (connect(sock_fd,(void *) &serv_addr,sizeof(serv_addr)) < 0) {
      
     FD_ZERO(&rset);
     FD_SET(sock_fd,&rset); wset = rset;
     //timeout
     tval.tv_sec = 0;         
     tval.tv_usec= 1e2;       

     if(select(sock_fd+1,&rset,&wset,NULL, &tval) < 0) {
#ifdef NCURSE_DISPLAY_OPTION
        sprintf(buf_err,"Connection::Failed!   ");
#else        	
        printf("uNAV CLIENT: Connect Failed.\n");
#endif        
        close(sock_fd);
        ret = 0;
     }
     else 
     {
#ifdef NCURSE_DISPLAY_OPTION  
        sprintf(buf_err,"Connection::Try!      ");
#else     	
        printf("uNAV CLIENT: Connected to server.\n");
#endif        
        //restore
        fcntl(sock_fd,F_SETFL,flags);
        ret = 1;
     }

  }
  else 
  {
#ifdef NCURSE_DISPLAY_OPTION
        sprintf(buf_err,"Connection::Try!      ");
#else     	
        printf("uNAV CLIENT: Connected to server.\n");
#endif    	
     
     //restore
     fcntl(sock_fd,F_SETFL,flags);
     ret = 1;
  }
  return ret;
}

void send_client (void)
{
  char buf[200]={0,};
  short i = 0;
  unsigned long  sum = 0;

  sprintf(buf,"%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %f %f %6.2f %d %d %d  end",
            imupacket.p,  imupacket.q,  imupacket.r,
            imupacket.ax, imupacket.ay, imupacket.az,
            imupacket.phi,imupacket.the,imupacket.psi,
            imupacket.hx, imupacket.hy, imupacket.hz,
            imupacket.Ps, imupacket.Pt, 
            gpspacket.lat,gpspacket.lon,gpspacket.alt,gpspacket.err_type,imupacket.err_type,navpacket.err_type);

  for(i=0;i<199;i++) sum += buf[i];
  buf[199] = (char)(sum%256);
  
  sprintf(buf_err,"Sending Packet::OK!    ");
     
  if (send(sock_fd, buf, 200, 0) == -1) {
#ifdef NCURSE_DISPLAY_OPTION
     sprintf(buf_err,"Sending Packet::Failed!   ");
#else  	
     printf("uNAV CLIENT: Sending Packet Failed.\n");
#endif     
     close(sock_fd);
     retvalsock = open_client();
  }
    
}

void timer_intr1(int sig)
{
   return;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// help message
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void help_message()
{
   printf("\n./avionics -option1 -option2 ... \n");
   printf("-wifi on/off        : enable or disable WiFi communication with GS \n");
   printf("-save on/off        : enable or disable datalogging in /mnt/cf1/ \n");	
   printf("-screen on/off      : enable or disable dumping data to screen \n");	
   printf("-ip xxx.xxx.xxx.xxx : set GS i.p. address for WiFi comm. \n");
   printf("-help               : display the help messages \n\n");
    
   _exit(0);	
}	
