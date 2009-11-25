/******************************************************************************
* FILE: mnav.c
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

#include "comms/console_link.h"
#include "comms/logging.h"
#include "comms/serial.hxx"
#include "include/globaldefs.h"
#include "filters/mnav/ahrs.h"
#include "filters/mnav/nav.h"
#include "props/props.hxx"
#include "util/myprof.h"
#include "util/timing.h"

#include "gps_mgr.h"

#include "mnav.h"

//
// uNAV packet length definition
//
#define FULL_PACKET_LENGTH	38
#define SENSOR_PACKET_LENGTH    51   
#define GPS_PACKET_LENGTH	35
#define FULL_PACKET_SIZE        86 // scaled mode with sampling less than 100Hz
#define SERVO_PACKET_LENGTH     24
#define SHORT_SERVO_PACKET_LENGTH 14

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
static SGSerialPort sPort2;

bool autopilot_active = false;
bool autopilot_reinit = false;
int  autopilot_count = 0;
char *cnt_status;

static struct imu imu_data;
static struct gps gps_data;
static struct servo servo_in;
static struct servo servo_out;
static bool imu_data_valid = false;
static bool gps_data_valid = false;
static double start_time;

// mnav property nodes
static SGPropertyNode *mnav_dev = NULL;
static SGPropertyNode *outputroot = NULL;

// imu property nodes
static SGPropertyNode *imu_timestamp_node = NULL;
static SGPropertyNode *imu_p_node = NULL;
static SGPropertyNode *imu_q_node = NULL;
static SGPropertyNode *imu_r_node = NULL;
static SGPropertyNode *imu_ax_node = NULL;
static SGPropertyNode *imu_ay_node = NULL;
static SGPropertyNode *imu_az_node = NULL;
static SGPropertyNode *imu_hx_node = NULL;
static SGPropertyNode *imu_hy_node = NULL;
static SGPropertyNode *imu_hz_node = NULL;

// air data property nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_Ps_node = NULL;
static SGPropertyNode *airdata_Pt_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

// actuator property nodes
static SGPropertyNode *act_timestamp_node = NULL;
static SGPropertyNode *act_aileron_node = NULL;
static SGPropertyNode *act_elevator_node = NULL;
static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *act_rudder_node = NULL;
static SGPropertyNode *act_channel5_node = NULL;
static SGPropertyNode *act_channel6_node = NULL;
static SGPropertyNode *act_channel7_node = NULL;
static SGPropertyNode *act_channel8_node = NULL;

// initialize mnav input property nodes
static void bind_input( SGPropertyNode *config ) {
    mnav_dev = config->getChild("device", 0, true);
}


// initialize mnav imu output property nodes 
static void bind_imu_output( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );
    imu_timestamp_node = outputroot->getChild("timestamp", 0, true);
    imu_p_node = outputroot->getChild("p-rad_sec", 0, true);
    imu_q_node = outputroot->getChild("q-rad_sec", 0, true);
    imu_r_node = outputroot->getChild("r-rad_sec", 0, true);
    imu_ax_node = outputroot->getChild("ax-mps_sec", 0, true);
    imu_ay_node = outputroot->getChild("ay-mps_sec", 0, true);
    imu_az_node = outputroot->getChild("az-mps_sec", 0, true);
    imu_hx_node = outputroot->getChild("hx", 0, true);
    imu_hy_node = outputroot->getChild("hy", 0, true);
    imu_hz_node = outputroot->getChild("hz", 0, true);
}


// initialize mnav gps output property nodes 
static void bind_airdata_output( string rootname ) {
    // "/sensors/air-data"
    outputroot = fgGetNode( rootname.c_str(), true );
    airdata_timestamp_node = outputroot->getChild("timestamp", 0, true);
    airdata_Ps_node = outputroot->getChild("Ps-m", 0, true);
    airdata_Pt_node = outputroot->getChild("Pt-ms", 0, true);
}


// initialize mnav gps output property nodes 
static void bind_gps_output( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);
}

// initialize actuator property nodes 
static void bind_act_nodes() {
    act_timestamp_node = fgGetNode("/actuators/actuator/timestamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);
}


// open and intialize the MNAV communication channel
void mnav_imu_init( string rootname, SGPropertyNode *config ) {
    if ( outputroot != NULL ) {
	// init has already been run
	return;
    }

    bind_input( config );
    bind_imu_output( rootname );

    int		nbytes = 0;
    uint8_t  	SCALED_MODE[11] ={0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00, 'S',0x00,0xF0};
    uint8_t          CH_BAUD[11]     ={0x55,0x55,0x57,0x46,0x01,0x00,0x02,0x00,0x03,0x00,0xA3};
    uint8_t		CH_SAMP[11]     ={0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x02,0x00,0x9D};
    uint8_t          CH_SERVO[7]     ={0x55,0x55,0x53,0x50,0x00,0x00,0xA3};

    printf("[mnav (%s)] ...\n", mnav_dev->getStringValue());

    //
    // Open and configure Serial Port2 (com2)
    //

    // Note: the MNAV serial input routine depends on only a single
    // command being in it's input buffer at a time.  It doesn't look
    // at the actual data values initially, just reads to the end of
    // the buffer.  If we write our commands too quickly and stack up
    // more than one message in the MNAV input buffer, all but the
    // first message will be lost.  Thus we need to sleep for at least
    // 100-200ms between message.  For now it's easier to just sleep 1
    // second.

    sPort2.open_port( mnav_dev->getStringValue(), false );
    sPort2.set_baud( 38400 );
    // printf("Opened serial port at 38400.\n");
      
    while (nbytes != 11) {
        printf("  writing CH_BAUD\n");
        nbytes = sPort2.write_port((char*)CH_BAUD, 11);
        sleep(1);
    }
    nbytes = 0;  
    sPort2.close_port();

    sPort2.open_port( mnav_dev->getStringValue(), false );
    sPort2.set_baud( 57600 );
    // printf("Opened serial port at 57600.\n");
  
    
    while (nbytes != 11) {
        printf("  writing CH_SAMP\n");
        nbytes = sPort2.write_port((char*)CH_SAMP, 11);
        sleep(1);
    }
    nbytes = 0;

    while (nbytes != 11) {
        printf("  writing SCALED_MODE\n");
        nbytes = sPort2.write_port((char*)SCALED_MODE, 11);
        sleep(1);
    }
    nbytes = 0;

    while (nbytes !=  7) {
        printf("  writing CH_SERVO\n");
        nbytes = sPort2.write_port((char*)CH_SERVO, 7);
        sleep(1);
    }
    nbytes = 0;  

    if ( display_on ) {
        printf(" initialized.\n");
    }
}


void mnav_airdata_init( string rootname ) {
    bind_airdata_output( rootname );
}


void mnav_gps_init( string rootname ) {
    bind_gps_output( rootname );
}

void mnav_act_init() {
    bind_act_nodes();
}


// Main MNAV data aquisition routine
//
// Note this blocks until new IMU/GPS data is available.  The rate at
// which the MNAV sends data dictates the timing and rate of the
// entire ugear program.  This routine fills in both the imu and gps
// structure if data is available from the MNAV.

bool mnav_read()
{
    imu_data_valid = false;
    gps_data_valid = false;

    int headerOK = 0;
    int nbytes = 0;
    uint8_t input_buffer[FULL_PACKET_SIZE]={0,};

    int trouble_count = 1;

    // Find start of packet: the header (2 bytes) starts with 0x5555
    while ( headerOK != 2 ) {
        while ( 1 != sPort2.read_port((char *)input_buffer,1) );
        if ( input_buffer[0] == 0x55 ) {
            headerOK++;
            trouble_count = 1;
        } else {
            headerOK = 0;
            trouble_count++;
        }
        if ( trouble_count % 10000 == 0 ) {
            printf("Having trouble finding a valid packet header.\n");
        }
    }
     	
    headerOK = 0; while ( 1 != sPort2.read_port((char *)&input_buffer[2],1) );
    nbytes = 3; 
  
    // Read packet contents
    switch (input_buffer[2]) {
    case 'S':               // IMU packet without GPS (< 100hz)
    case 's':               // IMU packet without GPS (100hz)
        // printf("no gps data being sent ... :-(\n");
        while ( nbytes < SENSOR_PACKET_LENGTH ) {
            nbytes += sPort2.read_port((char *)input_buffer+nbytes,
				       SENSOR_PACKET_LENGTH-nbytes); 
        }

        // check checksum
        if ( checksum(input_buffer,SENSOR_PACKET_LENGTH) ) {
            decode_imupacket(&imu_data, input_buffer);
            imu_data_valid = true;
        } else {
            if ( display_on ) {
                printf("[imu]:checksum error...!\n"); 
            }
            imu_data.status = ChecksumError; 
        };
        break;

    case 'N':               // IMU packet with GPS (< 100hz)
    case 'n':               // IMU packet with GPS (100hz)
        while ( nbytes < FULL_PACKET_SIZE ) {
            nbytes += sPort2.read_port((char *)input_buffer+nbytes,
				       FULL_PACKET_SIZE-nbytes); 
        }

        // printf("G P S   D A T A   A V A I L A B L E\n");

        // check checksum
        if ( checksum(input_buffer,FULL_PACKET_SIZE) ) {
            decode_imupacket(&imu_data, input_buffer);
            imu_data_valid = true;
		     
            // check GPS data packet
            if(input_buffer[33]=='G') {
                decode_gpspacket(&gps_data, input_buffer);
		gps_data_valid = true;
            } else {
		printf("[gps]:data error...!\n");
		gps_data.status = NotValid;
            } // end if(checksum(input_buffer...
        } else { 
            if ( display_on ) {
                printf("[imu]:checksum error(gps)...!\n");
            }
            gps_data.status = ChecksumError;
            imu_data.status = ChecksumError; 
        }
        break;

    default : 
        if ( display_on ) {
            printf("[imu] invalid data packet ... !\n");
        }

    } // end case

    return imu_data_valid;
}


void mnav_start_nonblock_read() {
    imu_data_valid = false;
    gps_data_valid = false;
    start_time = get_Time();
}


bool mnav_read_nonblock()
{
    const double time_out = 0.0025; // 1/4 of 100hz interval

    static int nbytes = 0;
    static uint8_t input_buffer[FULL_PACKET_SIZE]={0,};

    // set mode as nonblocking
    sPort2.set_nonblocking();

    if ( nbytes < 3 ) {
	// Find start of packet: the header (2 bytes) starts with 0x55 0x55
	// bail if we find nothing in the input buffer
	int headerOK = 0;

	while ( (headerOK != 2) && (get_Time() - start_time < time_out) ) {
	    if ( sPort2.read_port((char *)input_buffer,1) == 1 ) {
		if ( input_buffer[0] == 0x55 ) {
		    headerOK++;
		} else {
		    headerOK = 0;
		}
	    } else {
		// exit immediately (not waiting for timeout) if
		// nothing in the input buffer.
		return false;
	    }
	}
	if ( headerOK != 2 ) {
	    // printf("Timeout (1) looking for header.\n");
	    return false;
	}
     	
	headerOK = 0;
	while ( (headerOK != 1) && (get_Time() - start_time < time_out) ) {
	    if ( sPort2.read_port((char *)&input_buffer[2],1) == 1 ) {
		headerOK = 1;
	    }
	}
	if ( headerOK != 1 ) {
	    // printf("Timeout (2) looking for header.\n");
	    return false;
	}
	
	nbytes = 3; 
    } else {
	// printf("Restarting packet read\n");
    }
  
    // Read packet contents
    switch (input_buffer[2]) {
    case 'S':               // IMU packet without GPS (< 100hz)
    case 's':               // IMU packet without GPS (100hz)
        // printf("no gps data being sent ... :-(\n");
        while ( nbytes < SENSOR_PACKET_LENGTH
		&& (get_Time() - start_time < time_out) )
	{
	    int result = sPort2.read_port((char *)input_buffer+nbytes,
					  SENSOR_PACKET_LENGTH-nbytes);
	    if ( result > 0 ) {
		nbytes += result;
	    }
        }
	if ( nbytes < SENSOR_PACKET_LENGTH ) {
	    // printf("Timeout reading IMU packet\n");
	    return false;
	}

        // check checksum
        if ( checksum(input_buffer,SENSOR_PACKET_LENGTH) ) {
            decode_imupacket(&imu_data, input_buffer);
            imu_data_valid = true;
        } else {
            if ( display_on ) {
                printf("[imu]:checksum error...!\n"); 
            }
            imu_data.status = ChecksumError; 
        };
        break;

    case 'N':               // IMU packet with GPS (< 100hz)
    case 'n':               // IMU packet with GPS (100hz)
        while ( nbytes < FULL_PACKET_SIZE
		&& (get_Time() - start_time < time_out) )
	{
            int result = sPort2.read_port((char *)input_buffer+nbytes,
					  FULL_PACKET_SIZE-nbytes); 
	    if ( result > 0 ) {
		nbytes += result;
	    }
        }
	if ( nbytes < FULL_PACKET_SIZE ) {
	    if ( display_on ) {
		printf("Timeout reading IMU+GPS packet, but will resume next frame...\n");
	    }

	    return false;
	}

        // printf("G P S   D A T A   A V A I L A B L E\n");

        // check checksum
        if ( checksum(input_buffer,FULL_PACKET_SIZE) ) {
            decode_imupacket(&imu_data, input_buffer);
            imu_data_valid = true;
		     
            // check GPS data packet
            if(input_buffer[33]=='G') {
                decode_gpspacket(&gps_data, input_buffer);
		gps_data_valid = true;
            } else {
		printf("[gps]:data error...!\n");
		gps_data.status = NotValid;
            } // end if(checksum(input_buffer...
        } else { 
            if ( display_on ) {
                printf("[imu]:checksum error(gps)...!\n");
            }
            gps_data.status = ChecksumError;
            imu_data.status = ChecksumError; 
        }
        break;

    default : 
        if ( display_on ) {
            printf("[imu] invalid data packet ... !\n");
        }

    } // end case

    nbytes = 0;

    return true;
}


void mnav_imu_update() {
}


void mnav_gps_update() {
    // noop for now
}


void mnav_airdata_update() {
    // noop for now
}


void mnav_manual_override_check() {
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

    int ch5 = 32768 + act_channel5_node->getFloatValue() * 32768;

    if ( ch5 <= 12000 ) {
        // MNAV is in AutoPilot Mode
        // if the autopilot is enabled, or signal is lost
        if ( !autopilot_active && display_on ) {
            printf("[CONTROL]: switching to autopilot\n");
	    autopilot_reinit = true;
        }
        autopilot_active = true;
        autopilot_count  = 15;
    } else if ( ch5 > 12000 && ch5 < 60000 ) {
        // add delay on control trigger to minimize mode confusion
        // caused by the transmitter power off
        if ( autopilot_count < 0 ) {
            // MNAV is in Manual Mode
            if ( autopilot_active && display_on ) {
                printf("[CONTROL]: switching to manual pass through\n");
            }
            autopilot_active = false;
        } else {
            autopilot_count--;
        }
    }
}


void mnav_close()
{
    //close the serial port
    sPort2.close_port();

    //close files
    logging_close();
}


//
// check the checksum of the data packet
//
bool checksum( uint8_t* buffer, int packet_len ) {
    uint16_t i = 0, rcvchecksum = 0;
    uint16_t sum = 0;

    for ( i = 2; i < packet_len - 2; i++ ) sum = sum + buffer[i];
    // original code generates a warning:
    //   "operation on 'rcvchecksum' may be undefined"
    // rcvchecksum = ((rcvchecksum = buffer[packet_len-2]) << 8) | buffer[packet_len-1];
    rcvchecksum = (buffer[packet_len-2] << 8) | buffer[packet_len-1];

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
    // data->ITOW = ((data->ITOW = buffer[59]) << 8)|buffer[58];
    data->date = (buffer[61] << 24) | (buffer[60] << 16) | (buffer[59] << 8)
        | buffer[58];
    data->date /= 1000.0;

    // uint16_t msb;
    // msb = ((msb = buffer[61]) << 8) | buffer[60];
    // printf("gps itow = (%d) (%d) (%d) (%d) %.3f\n", buffer[61], buffer[60], buffer[59], buffer[58], data->ITOW);
    data->status = ValidData;
    data->time = get_Time();

    // printf("sizeof gps = %d  time = %.3f\n", sizeof(struct gps), data->time);
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
    case 'S' :   servo_in.status = buffer[32];
        servo_in.channel[0] = ((tmpr = buffer[33]) << 8)|buffer[34]; tmpr = 0;
        servo_in.channel[1] = ((tmpr = buffer[35]) << 8)|buffer[36]; tmpr = 0;
        servo_in.channel[2] = ((tmpr = buffer[37]) << 8)|buffer[38]; tmpr = 0;
        servo_in.channel[3] = ((tmpr = buffer[39]) << 8)|buffer[40]; tmpr = 0;
        servo_in.channel[4] = ((tmpr = buffer[41]) << 8)|buffer[42]; tmpr = 0;
        servo_in.channel[5] = ((tmpr = buffer[43]) << 8)|buffer[44]; tmpr = 0;
        servo_in.channel[6] = ((tmpr = buffer[45]) << 8)|buffer[46]; tmpr = 0;
        servo_in.channel[7] = ((tmpr = buffer[47]) << 8)|buffer[48]; 
        break;
    case 'N' :   servo_in.status = buffer[67];
        servo_in.channel[0] = ((tmpr = buffer[68]) << 8)|buffer[69]; tmpr = 0;
        servo_in.channel[1] = ((tmpr = buffer[70]) << 8)|buffer[71]; tmpr = 0;
        servo_in.channel[2] = ((tmpr = buffer[72]) << 8)|buffer[73]; tmpr = 0;
        servo_in.channel[3] = ((tmpr = buffer[74]) << 8)|buffer[75]; tmpr = 0;
        servo_in.channel[4] = ((tmpr = buffer[76]) << 8)|buffer[77]; tmpr = 0;
        servo_in.channel[5] = ((tmpr = buffer[78]) << 8)|buffer[79]; tmpr = 0;
        servo_in.channel[6] = ((tmpr = buffer[80]) << 8)|buffer[81]; tmpr = 0;
        servo_in.channel[7] = ((tmpr = buffer[82]) << 8)|buffer[83]; 
        break;
    default  :
        printf("[imu]:fail to decode servo packet..!\n");
    }

    data->time = get_Time();
    data->status = ValidData;

    servo_in.time = data->time;
}


bool mnav_get_imu() {
    if ( imu_data_valid ) {
	imu_timestamp_node->setDoubleValue( imu_data.time );
	imu_p_node->setDoubleValue( imu_data.p );
	imu_q_node->setDoubleValue( imu_data.q );
	imu_r_node->setDoubleValue( imu_data.r );
	imu_ax_node->setDoubleValue( imu_data.ax );
	imu_ay_node->setDoubleValue( imu_data.ay );
	imu_az_node->setDoubleValue( imu_data.az );
	imu_hx_node->setDoubleValue( imu_data.hx );
	imu_hy_node->setDoubleValue( imu_data.hy );
	imu_hz_node->setDoubleValue( imu_data.hz );
    }

    return imu_data_valid;
}


bool mnav_get_gps() {
    // mnav conveys little useful date information from the gps,
    // fake it with a recent date that is close enough to compute
    // a reasonable magnetic variation, this should be updated
    // every year or two.
    gps_data.date = 1240238933; /* Apr 20, 2009 */

    if ( gps_data_valid ) {
	gps_timestamp_node->setDoubleValue( gps_data.time );
	gps_lat_node->setDoubleValue( gps_data.lat );
	gps_lon_node->setDoubleValue( gps_data.lon );
	gps_alt_node->setDoubleValue( gps_data.alt );
	gps_ve_node->setDoubleValue( gps_data.ve );
	gps_vn_node->setDoubleValue( gps_data.vn );
	gps_vd_node->setDoubleValue( gps_data.vd );
	gps_unix_sec_node->setDoubleValue( gps_data.date );
    }

    return gps_data_valid;
}


bool mnav_get_airdata() {
    // this is a bit of a hack ... just fill in the air data entries
    // in the "imu" structure which is badly named and return
    if ( imu_data_valid ) {
	airdata_timestamp_node->setDoubleValue( imu_data.time );
	airdata_Ps_node->setDoubleValue( imu_data.Ps );
	airdata_Pt_node->setDoubleValue( imu_data.Pt );
    }

    return imu_data_valid;
}


static void mnav_gen_servo() {
    servo_out.channel[0]
	= 32768 + (int16_t)(act_aileron_node->getFloatValue() * 32768);
    servo_out.channel[1]
	= 32768 + (int16_t)(act_elevator_node->getFloatValue() * 32768);
    servo_out.channel[2]
	= 32768 + (int16_t)((act_throttle_node->getFloatValue()*2.0 - 1.0) * 32768);
    servo_out.channel[3]
	= 32768 + (int16_t)(act_rudder_node->getFloatValue() * 32768);
    servo_out.channel[0]
	= 32768 + (int16_t)(act_channel5_node->getFloatValue() * 32768);
    servo_out.channel[0]
	= 32768 + (int16_t)(act_channel6_node->getFloatValue() * 32768);
    servo_out.channel[0]
	= 32768 + (int16_t)(act_channel7_node->getFloatValue() * 32768);
    servo_out.channel[0]
	= 32768 + (int16_t)(act_channel8_node->getFloatValue() * 32768);
}


void mnav_send_servo_cmd( /* struct servo *servo_out */ )
{
    // ch0: aileron, ch1: elevator, ch2: throttle, ch3: rudder

    // populate servo packet
    mnav_gen_servo();

    uint8_t data[SERVO_PACKET_LENGTH];
    short i = 0;
    uint16_t sum = 0;

    // printf("sending servo data ");
    // for ( i = 0; i < 9; ++i ) printf("%d ", cnt_cmd[i]);
    // printf("\n");

    data[0] = 0x55; 
    data[1] = 0x55;
    data[2] = 0x53;
    data[3] = 0x53;

    for ( i = 0; i < 8; ++i ) {
        data[4+2*i] = (uint8_t)(servo_out.channel[i] >> 8); 
        data[5+2*i] = (uint8_t)servo_out.channel[i];
    }

    // pad last unused channel
    data[4+2*8] = 0x80;
    data[5+2*8] = 0x00;

    //checksum: need to be verified
    sum = 0xa6;
    for (i = 4; i < 22; i++) sum += data[i];
  
    data[22] = (uint8_t)(sum >> 8);
    data[23] = (uint8_t)sum;

    // don't attempt any manner of retry if write fails (it shouldn't
    // ever fail) :-)
    sPort2.write_port((char *)data, SERVO_PACKET_LENGTH);

    // printf("%d %d\n", cnt_cmd[0], cnt_cmd[1]);

}


// identical to full servo command, but only sends first 4 channels of
// data, saving 10 bytes per message.
void mnav_send_short_servo_cmd( /* struct servo *servo_out */ )
{
    // populate servo packet
    mnav_gen_servo();

    uint8_t data[SHORT_SERVO_PACKET_LENGTH];
    short i = 0;
    uint16_t sum = 0;

    // printf("sending servo data ");
    // for ( i = 0; i < 4; ++i ) printf("%d ", servo_out->channel[i]);
    // printf("\n");

    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = 0x53;
    data[3] = 0x54;

    for ( i = 0; i < 4; ++i ) {
        data[4+2*i] = (uint8_t)(servo_out.channel[i] >> 8); 
        data[5+2*i] = (uint8_t)servo_out.channel[i];
    }

    //checksum: need to be verified
    sum = 0xa6;
    for (i = 4; i < 12; i++) sum += data[i];
  
    data[12] = (uint8_t)(sum >> 8);
    data[13] = (uint8_t)sum;

    // don't attempt any manner of retry if write fails
    sPort2.write_port((char *)data, SHORT_SERVO_PACKET_LENGTH);
}
