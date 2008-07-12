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
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "globaldefs.h"
#include "groundstation.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined defintions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NETWORK_PORT      9001		 // network port number

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int gs_sock_fd;
char buf_err[50];
struct  sockaddr_in serv_addr;
char *HOST_IP_ADDR = "192.168.11.101"; //default ground station IP address

//
// open client to transmit/receive the packet to/from ground station
//
bool open_client ()
{
    //struct sockaddr_in serv_addr;
    bool result;
  
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family      = AF_INET; 
    serv_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    serv_addr.sin_port        = htons(NETWORK_PORT); 
    gs_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  
  
    if (connect(gs_sock_fd,(sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        printf("uNAV CLIENT: Connect Failed.\n");
        close_client();
        result = false;
    }
    else {
        printf("uNAV CLIENT: Connected to server.\n");
	result = true;
    }
  
    return result;
}

void send_client ( )
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
            gpspacket.lat,gpspacket.lon,gpspacket.alt,
            (int)gpspacket.err_type,
            (int)imupacket.err_type,
            (int)navpacket.err_type);

    for(i=0;i<199;i++) sum += buf[i];
    buf[199] = (char)(sum%256);
  
  
     
    if (sendto(gs_sock_fd, buf, 200, 0,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1) {
        printf("uNAV CLIENT: Sending Packet Failed.\n");
    } else {
        sprintf(buf_err,"Sending Packet::OK!    ");
    }
    
}

void close_client() {
    close(gs_sock_fd);
}
