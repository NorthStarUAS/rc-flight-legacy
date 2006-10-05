/******************************************************************************
* FILE: imugps.c
* DESCRIPTION:
*   
*   
*
* SOURCE: 
* LAST REVISED: 4/05/06 Jung Soon Jang
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>


#include "serial.h"
#include "globaldefs.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//uNAV packet length definition
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define FULL_PACKET_LENGTH	38
#define SENSOR_PACKET_LENGTH    51   
#define GPS_PACKET_LENGTH	35
#define FULL_PACKET_SIZE        86   // scaled mode with sampling less than 100Hz
#define fullspeed		0

#define D2R			0.017453292519940
#define R2D			57.29577951308232
#define g			9.81

//temperature compensation for accel.
//temperature compensation for mag.

//bias of acclerometers
#define		bias_ax	0.10791				//(0.0110*g)
#define		bias_ay 0.37965				//(0.0387*g)
#define		bias_az 0.13734				//(0.0140*g)



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//prototype definition
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int  checksum(byte* buffer, int packet_len);
void decode_imupacket(struct imu *data, byte* buffer);
void decode_gpspacket(struct gps *data, byte* buffer);
extern void snap_time_interval(char *threadname,int displaytime,short id);
extern double get_Time();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern short screen_on;
extern char  buf_err[50];
int   	     sPort0;


void *imugps_acq(void *thread_id)
{
  int		count=0,nbytes=0,headerOK=0;
  short		i=0;
  static int    GPS_INIT=FALSE,GPS_FULL=0,err_cnt=0;
  byte  	input_buffer[FULL_PACKET_SIZE]={0,};
  byte  	SCALED_MODE[11] ={0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00, 'S',0x00,0xF0};
  byte          CH_BAUD[11]     ={0x55,0x55,0x57,0x46,0x01,0x00,0x02,0x00,0x03,0x00,0xA3};
  byte		CH_SAMP[11]     ={0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x02,0x00,0x9D};
  byte          CH_SERVO[7]     ={0x55,0x55,0x53,0x50,0x08,0x00,0xAB};
  byte		temp;
  FILE   	*fimu,*fgps;
  
  /*********************************************************************
   *Open Files
   *********************************************************************/
  if (screen_on) {
         if((fimu = fopen("/mnt/cf1/imu.dat","w+b"))==NULL) {
            printf("imu.dat cannot be created in /mnt/cf1 directory...error!\n");
            _exit(-1);
         }
	 if((fgps = fopen("/mnt/cf1/gps.dat","w+b"))==NULL) {
            printf("gps.dat cannot be created in /mnt/cf1 directory...error!\n");
            _exit(-1);
         }
  }

  printf("[imugps_acq]::thread[%d] initiated...\n",thread_id);
  /*********************************************************************
   *Open and configure Serial Port2 (com2)
   *********************************************************************/
 #ifdef SG2  
    sPort0 = open_serial("/dev/ttyS0",BAUDRATE_38400);                  nbytes = 0; 
    while (nbytes != 11) nbytes = write(sPort0,(char*)CH_BAUD, 11);     nbytes = 0;  
    close(sPort0);
    sPort0 = open_serial("/dev/ttyS0",BAUDRATE_57600); 
 #else   
    //SG1
    sPort0 = open_serial(SERIAL_PORT2,BAUDRATE_38400); 
  
    while (nbytes != 11) nbytes = write(sPort0,(char*)CH_BAUD, 11);     nbytes = 0;  
    close(sPort0);
    sPort0 = open_serial(SERIAL_PORT2,BAUDRATE_57600); 
 #endif 
  
  while (nbytes != 11) nbytes = write(sPort0,(char*)CH_SAMP, 11);     nbytes = 0;  
  
  while (nbytes != 11) nbytes = write(sPort0,(char*)SCALED_MODE, 11); nbytes = 0;
  while (nbytes !=  7) nbytes = write(sPort0,(char*)CH_SERVO, 7);     nbytes = 0;  
  
  

  while (1) {
  /*********************************************************************
   *Find start of packet: the heade r (2 bytes) starts with 0x5555
   *********************************************************************/
  
  while (headerOK !=2)
  {
     while(1!=read(sPort0,input_buffer,1));
     if (input_buffer[0] == 0x55) headerOK++;
     else		 	  headerOK = 0;
  }
     	
  headerOK = 0; while(1!=read(sPort0,&input_buffer[2],1));
  nbytes = 3; 
  
  
  /*********************************************************************
   *Read packet contents
   *********************************************************************/
  switch (input_buffer[2])
  {
	case 'S': /* IMU packet without GPS */
		  while(nbytes < SENSOR_PACKET_LENGTH)
		  {
		    nbytes += read(sPort0, input_buffer+nbytes, SENSOR_PACKET_LENGTH-nbytes); 
		  }

		  /*************************
                   *check checksum
                   *************************/
                  if(checksum(input_buffer,SENSOR_PACKET_LENGTH)==TRUE)
		  {

		     pthread_mutex_lock(&mutex_imu);
		       decode_imupacket(&imupacket, input_buffer);
		       if(screen_on) fwrite(&imupacket, sizeof(struct imu),1,fimu);
		       pthread_cond_signal(&trigger_ahrs);
		     pthread_mutex_unlock(&mutex_imu);

		  }
                  else {
#ifndef NCURSE_DISPLAY_OPTION 
                  	printf("[imu]:checksum error...!\n"); 
#endif                  	
                  	imupacket.err_type = checksum_err; 
                  };
       		  
		  
		  break;
        case 'N': /* IMU packet with    GPS */
                  while(nbytes < FULL_PACKET_SIZE)
		  {
		    nbytes += read(sPort0, input_buffer+nbytes, FULL_PACKET_SIZE-nbytes); 
		  }

  		  /*************************
                   *check checksum
                   *************************/
                  if(checksum(input_buffer,FULL_PACKET_SIZE)==TRUE)
		  {
	             pthread_mutex_lock(&mutex_imu);
		       decode_imupacket(&imupacket, input_buffer);
		       if(screen_on) fwrite(&imupacket, sizeof(struct imu),1,fimu);
		       pthread_cond_signal(&trigger_ahrs);
		     pthread_mutex_unlock(&mutex_imu);


		     /******************************************
                      *check GPS data packet
                      ******************************************/
		     if(input_buffer[31]==(byte)0x55 && input_buffer[32]==(byte)0x55 && input_buffer[33]=='G') 
		     {
	  	        pthread_mutex_lock(&mutex_gps);
 			  decode_gpspacket(&gpspacket, input_buffer);
			  if(screen_on) fwrite(&gpspacket, sizeof(struct gps),1,fgps);
		        pthread_mutex_unlock(&mutex_gps);
		     }	
                     else
		     {
#ifndef NCURSE_DISPLAY_OPTION		     	
		         printf("[gps]:data error...!\n");
#endif		         
			 gpspacket.err_type = got_invalid;
		     }	
		  } /* end if(checksum(input_buffer... */
		  else
		  { 
#ifndef NCURSE_DISPLAY_OPTION		  	
		     printf("[imu]:checksum error(gps)...!\n");
#endif		     
                     gpspacket.err_type = checksum_err;
		     imupacket.err_type = checksum_err; 
		  }
		  
                  
		  break;
        default : 
#ifdef NCURSE_DISPLAY_OPTION        
		     sprintf(buf_err,"Invalid [imu] data packet (%d)",++err_cnt);
#else
		     printf("[imu]:invalid data packet...!\n");
#endif		  
		  
		  
  } /* end case  */

  } /* end while */

  //close the serial port
  close(sPort0);
  //close files
  fclose(fimu);
  fclose(fgps);
  //exit the thread
  pthread_exit(NULL);

} /* end void *imugps_acq() */

/***************************************************************************************
 *check the checksum of the data packet
 ***************************************************************************************/
int checksum(byte* buffer, int packet_len)
{
   word     	 i=0,rcvchecksum=0;
   //unsigned long sum=0;
   word          sum=0;

   for(i=2;i<packet_len-2;i++) sum = sum + buffer[i];
   rcvchecksum = ((rcvchecksum = buffer[packet_len-2]) << 8) | buffer[packet_len-1];

// if (rcvchecksum == sum%0x10000)
   if (rcvchecksum == sum) //&0xFFFF)
	return    TRUE;
   else
 	return    FALSE;



}


/***************************************************************************************
 *decode the gps data packet
 ***************************************************************************************/
void decode_gpspacket(struct gps *data, byte* buffer)
{
   signed long tmp=0;

   /* gps velocity in m/s */ 
   data->vn =(double)((((((tmp = (signed char)buffer[37]<<8)|buffer[36])<<8)|buffer[35])<<8)|buffer[34])*1.0e-2; tmp=0;
   data->ve =(double)((((((tmp = (signed char)buffer[41]<<8)|buffer[40])<<8)|buffer[39])<<8)|buffer[38])*1.0e-2; tmp=0;
   data->vd =(double)((((((tmp = (signed char)buffer[45]<<8)|buffer[44])<<8)|buffer[43])<<8)|buffer[42])*1.0e-2; tmp=0;

   /* gps position */
   data->lon=(double)((((((tmp = (signed char)buffer[49]<<8)|buffer[48])<<8)|buffer[47])<<8)|buffer[46]);  	 tmp=0;
   data->lat=(double)((((((tmp = (signed char)buffer[53]<<8)|buffer[52])<<8)|buffer[51])<<8)|buffer[50]);  	 tmp=0;
   data->alt=(double)((((((tmp = (signed char)buffer[57]<<8)|buffer[56])<<8)|buffer[55])<<8)|buffer[54])*1.0e-3; tmp=0;
   
   data->lon=(data->lon)*1.0e-7;
   data->lat=(data->lat)*1.0e-7;
   

   /* gps time */
   data->ITOW = ((data->ITOW = buffer[59]) << 8)|buffer[58];
   data->err_type = TRUE;
   data->time = get_Time();

}

/***************************************************************************************
 *decode the imu data packet
 ***************************************************************************************/
void decode_imupacket(struct imu *data, byte* buffer)
{
   signed short tmp=0;
   unsigned short tmpr=0;
  

   /* acceleration in m/s^2 */
   data->ax = (double)(((tmp = (signed char)buffer[ 3])<<8)|buffer[ 4])*5.98754883e-04; tmp=0;
   data->ay = (double)(((tmp = (signed char)buffer[ 5])<<8)|buffer[ 6])*5.98754883e-04; tmp=0;
   data->az = (double)(((tmp = (signed char)buffer[ 7])<<8)|buffer[ 8])*5.98754883e-04; tmp=0;
   
  
   /* angular rate in rad/s */
   data->p  = (double)(((tmp = (signed char)buffer[ 9])<<8)|buffer[10])*1.065264436e-04; tmp=0;
   data->q  = (double)(((tmp = (signed char)buffer[11])<<8)|buffer[12])*1.065264436e-04; tmp=0;
   data->r  = (double)(((tmp = (signed char)buffer[13])<<8)|buffer[14])*1.065264436e-04; tmp=0;
   
   /* magnetic field in Gauss */
   data->hx = (double)(((tmp = (signed char)buffer[15])<<8)|buffer[16])*6.103515625e-05; tmp=0;
   data->hy = (double)(((tmp = (signed char)buffer[17])<<8)|buffer[18])*6.103515625e-05; tmp=0;
   data->hz = (double)(((tmp = (signed char)buffer[19])<<8)|buffer[20])*6.103515625e-05; tmp=0;

   /* temperature in Celcius */
   /*
   data->Tx = (double)(((tmp = (signed char)buffer[21])<<8)|buffer[22])*6.103515625e-03; tmp=0;
   data->Ty = (double)(((tmp = (signed char)buffer[23])<<8)|buffer[24])*6.103515625e-03; tmp=0;
   data->Tz = (double)(((tmp = (signed char)buffer[25])<<8)|buffer[26])*6.103515625e-03; tmp=0;
   */
   
   /* pressure in m and m/s */
   data->Ps = (double)(((tmp = (signed char)buffer[27])<<8)|buffer[28])*3.0517578125e-01; tmp=0;
   data->Pt = (double)(((tmp = (signed char)buffer[29])<<8)|buffer[30])*2.4414062500e-03; tmp=0;

   
   
   /* servo packet */
   switch (buffer[2]) {
      case 'S' :   servopacket.status = buffer[32];
   		   servopacket.chn[0] = ((tmpr = buffer[33]) << 8)|buffer[34]; tmpr = 0;
	           servopacket.chn[1] = ((tmpr = buffer[35]) << 8)|buffer[36]; tmpr = 0;
		   servopacket.chn[2] = ((tmpr = buffer[37]) << 8)|buffer[38]; tmpr = 0;
		   servopacket.chn[3] = ((tmpr = buffer[39]) << 8)|buffer[40]; tmpr = 0;
		   servopacket.chn[4] = ((tmpr = buffer[41]) << 8)|buffer[42]; tmpr = 0;
		   servopacket.chn[5] = ((tmpr = buffer[43]) << 8)|buffer[44]; tmpr = 0;
		   servopacket.chn[6] = ((tmpr = buffer[45]) << 8)|buffer[46]; tmpr = 0;
		   servopacket.chn[7] = ((tmpr = buffer[47]) << 8)|buffer[48]; tmpr = 0; 
		   break;
      case 'N' :   servopacket.status = buffer[67];
   		   servopacket.chn[0] = ((tmpr = buffer[68]) << 8)|buffer[69]; tmpr = 0;
	           servopacket.chn[1] = ((tmpr = buffer[70]) << 8)|buffer[71]; tmpr = 0;
		   servopacket.chn[2] = ((tmpr = buffer[72]) << 8)|buffer[73]; tmpr = 0;
		   servopacket.chn[3] = ((tmpr = buffer[74]) << 8)|buffer[75]; tmpr = 0;
		   servopacket.chn[4] = ((tmpr = buffer[76]) << 8)|buffer[77]; tmpr = 0;
		   servopacket.chn[5] = ((tmpr = buffer[78]) << 8)|buffer[79]; tmpr = 0;
		   servopacket.chn[6] = ((tmpr = buffer[80]) << 8)|buffer[81]; tmpr = 0;
		   servopacket.chn[7] = ((tmpr = buffer[82]) << 8)|buffer[83]; tmpr = 0;
                   break;
      default  :
                   printf("[imu]:fail to decode servo packet..!\n");
   }

   printf("servo: %d %d %d %d %d\n", servopacket.chn[0], servopacket.chn[1], servopacket.chn[2], servopacket.chn[3], servopacket.chn[4] );
  
   data->time = get_Time();
   data->err_type = no_error;
     
}
