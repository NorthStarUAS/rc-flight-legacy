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
#include <math.h>
#include <pthread.h>

#include "globaldefs.h"
#include "imugps.h"
#include "logging.h"
#include "serial.h"
#include "timing.h"

//
// uNAV packet length definition
//
#define FULL_PACKET_LENGTH	38
#define SENSOR_PACKET_LENGTH    51   
#define GPS_PACKET_LENGTH	35
#define FULL_PACKET_SIZE        86  // scaled mode with sampling less than 100Hz
#define fullspeed		0

#define D2R			0.017453292519940
#define R2D			57.29577951308232
#define g			9.81

//temperature compensation for accel.
//temperature compensation for mag.

//
// prototype definition
//
bool checksum(uint8_t* buffer, int packet_len);
void decode_imupacket(struct imu *data, uint8_t* buffer);
void decode_gpspacket(struct gps *data, uint8_t* buffer);

//
// global variables
//
static int sPort2;

struct servo servopacket;
bool         autopilot_enable = false;
bool         control_init = false;
char         *cnt_status;

struct imu imupacket;
struct gps gpspacket;
struct nav navpacket;

// Main IMU/GPS data aquisition thread.
//
// Note this thread runs as fast as IMU/GPS data is available.  It
// blocks on the serial port read, otherwise it runs full bore,
// pulling in data as fast as it's available.
//
void *imugps_acq(void *thread_id)
{
    int		count=0,nbytes=0,headerOK=0;
    uint8_t  	input_buffer[FULL_PACKET_SIZE]={0,};
    uint8_t  	SCALED_MODE[11] ={0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00, 'S',0x00,0xF0};
    uint8_t          CH_BAUD[11]     ={0x55,0x55,0x57,0x46,0x01,0x00,0x02,0x00,0x03,0x00,0xA3};
    uint8_t		CH_SAMP[11]     ={0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x02,0x00,0x9D};
    uint8_t          CH_SERVO[7]     ={0x55,0x55,0x53,0x50,0x08,0x00,0xAB};
  
#ifndef NCURSE_DISPLAY_OPTION
    if ( display_on ) {
        printf("[imugps_acq]::thread[%d] initiated...\n", thread_id);
    }
#endif
  
    /*********************************************************************
     *Open and configure Serial Port2 (com2)
     *********************************************************************/
    sPort2 = open_serial(SERIAL_PORT2,BAUDRATE_38400); 
      
    while (nbytes != 11) nbytes = write(sPort2,(char*)CH_BAUD, 11);     nbytes = 0;  
    close(sPort2);
    sPort2 = open_serial(SERIAL_PORT2,BAUDRATE_57600); 
  
    
    while (nbytes != 11) nbytes = write(sPort2,(char*)CH_SAMP, 11);     nbytes = 0;  
    while (nbytes != 11) nbytes = write(sPort2,(char*)SCALED_MODE, 11); nbytes = 0;
    while (nbytes !=  7) nbytes = write(sPort2,(char*)CH_SERVO, 7);     nbytes = 0;  
  
    while (1) {
        /*********************************************************************
         *Find start of packet: the heade r (2 bytes) starts with 0x5555
         *********************************************************************/
  
        while ( headerOK != 2 ) {
            while(1!=read(sPort2,input_buffer,1));
            if (input_buffer[0] == 0x55)
                headerOK++;
            else
                headerOK = 0;
        }
     	
        headerOK = 0; while(1!=read(sPort2,&input_buffer[2],1));
        nbytes = 3; 
  
        // Read packet contents
        switch (input_buffer[2]) {
        case 'S':               // IMU packet without GPS
            while ( nbytes < SENSOR_PACKET_LENGTH ) {
                nbytes += read(sPort2, input_buffer+nbytes,
                               SENSOR_PACKET_LENGTH-nbytes); 
            }

            // check checksum
            if ( checksum(input_buffer,SENSOR_PACKET_LENGTH) ) {
                pthread_mutex_lock(&mutex_imu);
                decode_imupacket(&imupacket, input_buffer);
                pthread_cond_signal(&trigger_ahrs);
                if ( log_to_file ) {
                    log_imu( &imupacket );
                    log_servo( &servopacket );
                }
                pthread_mutex_unlock(&mutex_imu);  
            } else {
#ifndef NCURSE_DISPLAY_OPTION 
                printf("[imu]:checksum error...!\n"); 
#endif                  	
                imupacket.err_type = checksum_err; 
            };
       		  
		  
            break;
        case 'N':               // IMU packet with GPS
            while ( nbytes < FULL_PACKET_SIZE ) {
                nbytes += read(sPort2, input_buffer+nbytes,
                               FULL_PACKET_SIZE-nbytes); 
            }

            // check checksum
            if ( checksum(input_buffer,FULL_PACKET_SIZE) ) {
                pthread_mutex_lock(&mutex_imu);
                decode_imupacket(&imupacket, input_buffer);
                pthread_cond_signal(&trigger_ahrs);
                if ( log_to_file ) {
                    log_imu( &imupacket );
                    log_servo( &servopacket );
                }
                pthread_mutex_unlock(&mutex_imu);  
		     
                // check GPS data packet
                if(input_buffer[33]=='G') {
                    pthread_mutex_lock(&mutex_gps);
                    decode_gpspacket(&gpspacket, input_buffer);
                    if ( log_to_file ) log_gps( &gpspacket );
                    pthread_mutex_unlock(&mutex_gps);
                } else {
#ifndef NCURSE_DISPLAY_OPTION		     	
                    printf("[gps]:data error...!\n");
#endif		         
                    gpspacket.err_type = got_invalid;
                } // end if(checksum(input_buffer...
            } else { 
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
		  
        } // end case


        //////////////////////////////////////////////////////////////
        // NOTICE: MANUAL OVERRIDE
        //////////////////////////////////////////////////////////////

        // Manual override functionality for the uNAV is hardwired
        // into the unit.  It is hardwired to the gear channel (CH5
        // when counting from 1.)  This is not optional.  When you
        // enter manual override mode, the servo commands you send to
        // the uNAV are ignored and the unit passes through the
        // transmited values directly.
        //
        // So all the code needs to do is monitor the manual override
        // switch and not send autopilot commands in manual override
        // mode (they would be ignored anyway.)  There is no need to
        // copy the data through in software, it should all happen
        // automatically.
        //
        // Also note that I call this a "manual override" and not a
        // "fail safe".  If the uNAV fails for any reason, you will be
        // picking up toothpicks.
        //

        if ( servopacket.chn[4] <= 12000 ) {
            // if the autopilot is enabled, or signal is lost
            if ( !autopilot_enable && display_on ) {
                printf("[CONTROL]: switching to autopilot\n");
            }
            autopilot_enable = true;  
            count  = 15;
            cnt_status = "MNAV in AutoPilot Mode";
        } else if ( servopacket.chn[4] > 12000
                    && servopacket.chn[4] < 60000 )
        {
            // add delay on control trigger to minimize mode confusion
            // caused by the transmitter power off
            if ( count < 0 ) {
                if ( autopilot_enable && display_on ) {
                    printf("[CONTROL]: switching to manual pass through\n");
                }
                autopilot_enable = false;
                control_init = false;
                cnt_status = "MNAV in Manual Mode";
            } else {
                count--;
            }

        }
		
    } // end while

    //close the serial port
    close(sPort2);

    //close files
    logging_close();

    //exit the thread
    pthread_exit(NULL);

} // end void *imugps_acq()


//
// check the checksum of the data packet
//
bool checksum( uint8_t* buffer, int packet_len ) {
    uint16_t i = 0, rcvchecksum = 0;
    uint16_t sum = 0;

    for ( i = 2; i < packet_len - 2; i++ ) sum = sum + buffer[i];
    rcvchecksum = ((rcvchecksum = buffer[packet_len-2]) << 8) | buffer[packet_len-1];

    // if (rcvchecksum == sum%0x10000)
    if ( rcvchecksum == sum ) //&0xFFFF)
	return true;
    else
 	return false;
}


//
// decode the gps data packet
//
void decode_gpspacket( struct gps *data, uint8_t* buffer )
{
    signed long tmp = 0;

    // gps velocity in m/s
    data->vn =(double)((((((tmp = (signed char)buffer[37]<<8)|buffer[36])<<8)|buffer[35])<<8)|buffer[34])*1.0e-2; tmp=0;
    data->ve =(double)((((((tmp = (signed char)buffer[41]<<8)|buffer[40])<<8)|buffer[39])<<8)|buffer[38])*1.0e-2; tmp=0;
    data->vd =(double)((((((tmp = (signed char)buffer[45]<<8)|buffer[44])<<8)|buffer[43])<<8)|buffer[42])*1.0e-2; tmp=0;

    // gps position
    data->lon=(double)((((((tmp = (signed char)buffer[49]<<8)|buffer[48])<<8)|buffer[47])<<8)|buffer[46])*1.0e-7; tmp=0;
    data->lat=(double)((((((tmp = (signed char)buffer[53]<<8)|buffer[52])<<8)|buffer[51])<<8)|buffer[50])*1.0e-7; tmp=0;
    data->alt=(double)((((((tmp = (signed char)buffer[57]<<8)|buffer[56])<<8)|buffer[55])<<8)|buffer[54])*1.0e-3; tmp=0;
   
   
    // gps time
    data->ITOW = ((data->ITOW = buffer[59]) << 8)|buffer[58];
    data->err_type = no_error;
    data->time = get_Time();
}


//
// decode the imu data packet
//
void decode_imupacket( struct imu *data, uint8_t* buffer )
{
    signed short tmp = 0;
    unsigned short tmpr = 0;

    /* acceleration in m/s^2 */
    data->ax = (double)(((tmp = (signed char)buffer[ 3])<<8)|buffer[ 4])*5.98755e-04; tmp=0;
    data->ay = (double)(((tmp = (signed char)buffer[ 5])<<8)|buffer[ 6])*5.98755e-04; tmp=0;
    data->az = (double)(((tmp = (signed char)buffer[ 7])<<8)|buffer[ 8])*5.98755e-04; tmp=0;
   
  
    /* angular rate in rad/s */
    data->p  = (double)(((tmp = (signed char)buffer[ 9])<<8)|buffer[10])*1.06526e-04; tmp=0;
    data->q  = (double)(((tmp = (signed char)buffer[11])<<8)|buffer[12])*1.06526e-04; tmp=0;
    data->r  = (double)(((tmp = (signed char)buffer[13])<<8)|buffer[14])*1.06526e-04; tmp=0;
   
    /* magnetic field in Gauss */
    data->hx = (double)(((tmp = (signed char)buffer[15])<<8)|buffer[16])*6.10352e-05; tmp=0;
    data->hy = (double)(((tmp = (signed char)buffer[17])<<8)|buffer[18])*6.10352e-05; tmp=0;
    data->hz = (double)(((tmp = (signed char)buffer[19])<<8)|buffer[20])*6.10352e-05; tmp=0;

    /* temperature in Celcius */
    /*
      data->Tx = (double)(((tmp = (signed char)buffer[21])<<8)|buffer[22])*6.10352e-03; tmp=0;
      data->Ty = (double)(((tmp = (signed char)buffer[23])<<8)|buffer[24])*6.10352e-03; tmp=0;
      data->Tz = (double)(((tmp = (signed char)buffer[25])<<8)|buffer[26])*6.10352e-03; tmp=0;
    */
   
    /* pressure in m and m/s */
    data->Ps = (double)(((tmp = (signed char)buffer[27])<<8)|buffer[28])*3.05176e-01; tmp=0;
    data->Pt = (double)(((tmp = (signed char)buffer[29])<<8)|buffer[30])*2.44141e-03; tmp=0;

    // servo packet
    switch (buffer[2]) {
    case 'S' :   servopacket.status = buffer[32];
        servopacket.chn[0] = ((tmpr = buffer[33]) << 8)|buffer[34]; tmpr = 0;
        servopacket.chn[1] = ((tmpr = buffer[35]) << 8)|buffer[36]; tmpr = 0;
        servopacket.chn[2] = ((tmpr = buffer[37]) << 8)|buffer[38]; tmpr = 0;
        servopacket.chn[3] = ((tmpr = buffer[39]) << 8)|buffer[40]; tmpr = 0;
        servopacket.chn[4] = ((tmpr = buffer[41]) << 8)|buffer[42]; tmpr = 0;
        servopacket.chn[5] = ((tmpr = buffer[43]) << 8)|buffer[44]; tmpr = 0;
        servopacket.chn[6] = ((tmpr = buffer[45]) << 8)|buffer[46]; tmpr = 0;
        servopacket.chn[7] = ((tmpr = buffer[47]) << 8)|buffer[48]; 
        break;
    case 'N' :   servopacket.status = buffer[67];
        servopacket.chn[0] = ((tmpr = buffer[68]) << 8)|buffer[69]; tmpr = 0;
        servopacket.chn[1] = ((tmpr = buffer[70]) << 8)|buffer[71]; tmpr = 0;
        servopacket.chn[2] = ((tmpr = buffer[72]) << 8)|buffer[73]; tmpr = 0;
        servopacket.chn[3] = ((tmpr = buffer[74]) << 8)|buffer[75]; tmpr = 0;
        servopacket.chn[4] = ((tmpr = buffer[76]) << 8)|buffer[77]; tmpr = 0;
        servopacket.chn[5] = ((tmpr = buffer[78]) << 8)|buffer[79]; tmpr = 0;
        servopacket.chn[6] = ((tmpr = buffer[80]) << 8)|buffer[81]; tmpr = 0;
        servopacket.chn[7] = ((tmpr = buffer[82]) << 8)|buffer[83]; 
        break;
    default  :
        printf("[imu]:fail to decode servo packet..!\n");
    }

    data->time = get_Time();
    servopacket.time = data->time;
    data->err_type = no_error;
}


void send_servo_cmd(uint16_t cnt_cmd[9])
{
    // cnt_cmd[1] = ch1:elevator
    // cnt_cmd[0] = ch0:aileron
    // cnt_cmd[2] = ch2:throttle

    uint8_t data[24];
    short i = 0, nbytes = 0;
    uint16_t sum = 0;

    // printf("sending servo data ");
    // for ( i = 0; i < 9; ++i ) printf("%d ", cnt_cmd[i]);
    // printf("\n");

    data[0] = 0x55; 
    data[1] = 0x55;
    data[2] = 0x53;
    data[3] = 0x53;

    for ( i = 0; i < 9; ++i ) {
        data[4+2*i] = (uint8_t)(cnt_cmd[i] >> 8); 
        data[5+2*i] = (uint8_t)cnt_cmd[i];
    }

    // aileron
    // data[4] = (uint8_t)(cnt_cmd[0] >> 8); 
    // data[5] = (uint8_t)cnt_cmd[0];

    // elevator
    // data[6] = (uint8_t)(cnt_cmd[1] >> 8);
    // data[7] = (uint8_t)cnt_cmd[1];

    // throttle
    // data[8] = (uint8_t)(cnt_cmd[2] >> 8);
    // data[9] = (uint8_t)cnt_cmd[2];

    //checksum: need to be verified
    sum = 0xa6;
    for (i = 4; i < 22; i++) sum += data[i];
  
    data[22] = (uint8_t)(sum >> 8);
    data[23] = (uint8_t)sum;

    //sendout the command packet
    while (nbytes != 24) {
        // printf("  writing servos ...\n");
        nbytes = write(sPort2,(char*)data, 24);
    }
}
