/*******************************************************************************
 * FILE: ground_station.c
 * DESCRIPTION: routines to do ethernet based communication with the ground
 *              station.
 *
 * SOURCE: 
 * REVISED: 9/02/05 Jung Soon Jang
 * REVISED: 4/07/06 Jung Soon Jang
 *******************************************************************************/

#include <fcntl.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "globaldefs.h"
#include "groundstation.h"

#ifdef NCURSE_DISPLAY_OPTION
#include <ncurses/ncurses.h>
#endif   


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined defintions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NETWORK_PORT      9001		 // network port number

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int gs_sock_fd;
extern char *HOST_IP_ADDR;
char buf_err[50];

#ifdef NCURSE_DISPLAY_OPTION
WINDOW  *win;
#endif


//
// open client to transmit/receive the packet to/from ground station
//
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
    gs_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  
    //make a nonblocking connection
    flags = fcntl(gs_sock_fd,F_GETFL,0);
    fcntl( gs_sock_fd, F_SETFL, flags | O_NONBLOCK);
  
    //printf("uNAV CLIENT: Starting to connect to server.\n");
    if (connect(gs_sock_fd,(void *) &serv_addr,sizeof(serv_addr)) < 0) {
      
        FD_ZERO(&rset);
        FD_SET(gs_sock_fd,&rset); wset = rset;
        //timeout
        tval.tv_sec = 0;         
        tval.tv_usec= 1e2;       

        if(select(gs_sock_fd+1,&rset,&wset,NULL, &tval) < 0) {
#ifdef NCURSE_DISPLAY_OPTION
            sprintf(buf_err,"Connection::Failed!   ");
#else        	
            printf("uNAV CLIENT: Connect Failed.\n");
#endif        
            close(gs_sock_fd);
            ret = 0;
        } else {
#ifdef NCURSE_DISPLAY_OPTION  
            sprintf(buf_err,"Connection::Try!      ");
#else     	
            printf("uNAV CLIENT: Connected to server.\n");
#endif        
            //restore
            fcntl(gs_sock_fd,F_SETFL,flags);
            ret = 1;
        }
    } else {
#ifdef NCURSE_DISPLAY_OPTION
        sprintf(buf_err,"Connection::Try!      ");
#else     	
        printf("uNAV CLIENT: Connected to server.\n");
#endif    	
     
        //restore
        fcntl(gs_sock_fd, F_SETFL, flags);
        ret = 1;
    }

    return ret;
}


short send_client (void)
{
    char buf[200]={0,};
    short i = 0;
    unsigned long  sum = 0;
    short ret = 0;

    sprintf(buf,"%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %f %f %6.2f %d %d %d  end",
            imupacket.p,  imupacket.q,  imupacket.r,
            imupacket.ax, imupacket.ay, imupacket.az,
            imupacket.phi,imupacket.the,imupacket.psi,
            imupacket.hx, imupacket.hy, imupacket.hz,
            imupacket.Ps, imupacket.Pt, 
            gpspacket.lat,gpspacket.lon,gpspacket.alt,gpspacket.err_type,imupacket.err_type,navpacket.err_type);

    for ( i = 0; i < 199; i++ ) sum += buf[i];
    buf[199] = (char)(sum%256);
  
    sprintf(buf_err,"Sending Packet::OK!    ");
     
    if (send(gs_sock_fd, buf, 200, 0) == -1) {
#ifdef NCURSE_DISPLAY_OPTION
        sprintf(buf_err,"Sending Packet::Failed!   ");
#else  	
        printf("uNAV CLIENT: Sending Packet Failed.\n");
#endif     
        ret = 0;
    } else {
        ret = 1;
    }

    return ret;
}


void close_client(void) {
    close(gs_sock_fd);
}
