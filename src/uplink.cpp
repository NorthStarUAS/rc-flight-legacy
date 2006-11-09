/******************************************************************************
 * FILE: uplink.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 10/11/05 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>

#include "globaldefs.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined constant
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define	numofuplink	180
#define maxwaypoints    8

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//globals
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern short   open_client();
extern int     gs_sock_fd;		   //socket 
extern short   retvalsock;         //socket status
extern struct  sockaddr_in serv_addr;

char	       bufs[numofuplink];  //data buffer
int	       numofwaypoints;     //number of waypoints from ground
double 	       waypoints[maxwaypoints][2];
float	       pid_gain[3];
short          manual=1,pid_mode,altholdc,turnc,waypointc;
//initial values for control gains
double         pitch_gain[3]  ={0.256, 0.010, 0.011}, roll_gain[3]={0.326,  0.04, 0.023};
double         heading_gain[3]={0.714, 0.203, 0.000}, alt_gain[3] ={0.080, 0.097, 0.024};
double         pos_gain[3]={0.23,0.0,0.0};
short          whichmode[6]={-1,-1,-1, 1, 1,-1};
char           uplinkstr[80];

void *uplink_acq(void *thread_id)
{
    int ret,errFlag,i;
    char temp[20],tempr[20];
    unsigned long sum=0;
    socklen_t  serv_addrlen = sizeof(serv_addr);

#ifndef NCURSE_DISPLAY_OPTION
    printf("[uplink_acq]::thread[%x] initiated...\n", (int)thread_id);  
#else
    sprintf(uplinkstr,"[UPLINK  ]:Uplink Data has not been received!");   
#endif

    //wait until wifi is connected
    for(;;){ sleep(3); if(retvalsock) break; };
    //wait until the uplink data arrives
   
    while (1) {
        if(retvalsock) {
            if( (ret=recvfrom(gs_sock_fd,bufs,numofuplink,0,(struct sockaddr *) &serv_addr,&serv_addrlen)) < 0) {
   		errFlag = 1;
            } else {
                if (ret == 0) {
                    //the connection is gracefully closed
                    continue;
                }
                //check checksum
                sum = 0; for(i=0;i<numofuplink-1;i++) sum += bufs[i];
                if(bufs[numofuplink-1] != sum%256) continue;
                switch (bufs[0]) {
                case 'W':
                    sscanf(bufs+2,"%d",&numofwaypoints);
                    for(i=0;i<numofwaypoints;i++) {
                        sscanf(bufs+3+i*23,"%s %s",&temp,&tempr);
                        waypoints[i][0] = atof(temp);    
                        waypoints[i][1] = atof(tempr);
                    }
#ifndef NCURSE_DISPLAY_OPTION
                    //print the results
                    for(i=0;i<numofwaypoints;i++) {
                        printf("[uplink]:waypts = %d=>Lat=%f, Lon=%f \n",
                               numofwaypoints,waypoints[i][0],waypoints[i][1]);
                    }
#else
                    sprintf(uplinkstr,"[UPLINK  ]:%d WayPts, Last>> Lat=%f, Lon=%f",
                            numofwaypoints,waypoints[numofwaypoints-1][0],waypoints[numofwaypoints-1][1]);
#endif
                    break;
                case 'G':
                    sscanf(bufs+2,"%hd %f %f %f",&pid_mode, &pid_gain[0],&pid_gain[1],&pid_gain[2]);
#ifndef NCURSE_DISPLAY_OPTION
                    printf("[uplink]:[mode=%d]P=%4.2f I=%4.2f D=%4.2f\n",pid_mode,pid_gain[0],pid_gain[1],pid_gain[2]);
#else
                    sprintf(uplinkstr,"[UPLINK  ]:Gain [mode=%d]>> P=%4.2f I=%4.2f D=%4.2f\n",pid_mode,pid_gain[0],pid_gain[1],pid_gain[2]);
#endif	
                    //gain tuning
                    switch (pid_mode) {
                    case 0:  //pitch_mode:
                        for(i=0;i<3;i++) pitch_gain[i]  = pid_gain[i];
                        whichmode[0] = 1;
                        whichmode[3] =-1;
                        whichmode[4] =-1;
                        break;
                    case 1:  //roll_mode:
                        for(i=0;i<3;i++) roll_gain[i]   = pid_gain[i];
                        whichmode[1] = 1;
                        whichmode[2] =-1;
                        break;
                    case 2:  //heading_mode:
                        for(i=0;i<3;i++) heading_gain[i]= pid_gain[i];
                        whichmode[2] = 1;
                        break;
                    case 3:  //altitude_mode:
                        for(i=0;i<3;i++) alt_gain[i]    = pid_gain[i];
                        whichmode[3] = 1;
                        break;
                    case 4:  //pos_mode:
                        for(i=0;i<3;i++) pos_gain[i]  = pid_gain[i];
                        whichmode[4] = 1;
                        break;
                    default:
                        printf("[control.c]:unrecognized control gain setting mode!...\n");
                    }
                    break;	
                case 'C':
                    //currently not available
                    sscanf(bufs+2,"%hd %hd %hd %hd",&manual,&altholdc,&turnc,&waypointc);
                    printf("\n[uplink]:Manual=%d,AltHold=%d,Turn=%d,WayPoint=%d\n\n",manual,altholdc,turnc,waypointc);
                    break;
                default:
                    printf("[uplink]:Invalid Uplink...!\n");
                } //end switch
            } //end else
        }
        sleep(1);
    } //end while
}
