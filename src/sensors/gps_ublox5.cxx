/**
 * \file: gps_ublox5.cpp
 *
 * u-blox 5 protocol driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.cpp,v 1.7 2009/08/25 15:04:01 curt Exp $
 */

#include <errno.h>		// errno
#include <math.h>		// sin() cos()
#include <sys/types.h>		// open()
#include <sys/stat.h>		// open()
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset()
#include <sys/time.h>		// gettimeofday()
#include <time.h>
#include <string>

using std::string;

#include "globaldefs.h"

#include "comms/logging.h"
#include "math/SGMath.hxx"
#include "math/SGGeodesy.hxx"
#include "props/props.hxx"
#include "util/strutils.hxx"
#include "util/timing.h"
#include "gps_mgr.h"

#include "gps_ublox5.h"


#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

static struct timeval timestamp; // Unix Time Stamp Structure

// gpsd property nodes
static SGPropertyNode *configroot = NULL;

static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

static SGPropertyNode *gps_satellites = NULL;
static SGPropertyNode *gps_nmode = NULL;

static SGPropertyNode *gps_device_name_node = NULL;

static int fd = -1;
static string device_name = "/dev/ttyS0";


// initialize gpsd input property nodes
static void bind_input( SGPropertyNode *config ) {
    gps_device_name_node = config->getChild("device");
    if ( gps_device_name_node != NULL ) {
	device_name = gps_device_name_node->getStringValue();
    }
    configroot = config;
}


// initialize gpsd output property nodes 
static void bind_output( string rootname ) {
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);

    gps_satellites = outputroot->getChild("satellites", 0, true);
    gps_nmode = outputroot->getChild("nmea-mode", 0, true);
}


// send our configured init strings to configure gpsd the way we prefer
static bool gps_ublox5_open() {
    if ( display_on ) {
	printf("ublox5 on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios oldTio;	// Old Serial Port Settings
    struct termios newTio; 	// New Serial Port Settings
    memset(&oldTio, 0, sizeof(oldTio));
    memset(&newTio, 0, sizeof(newTio));

    // Save Current Serial Port Settings
    tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    newTio.c_cflag     = B115200 | // bps rate
                         CRTSCTS | // output flow ctnl
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    newTio.c_iflag     = IGNPAR;   // ignore parity bits
    newTio.c_oflag     = 0;
    newTio.c_lflag     = 0;
    newTio.c_cc[VTIME] = 0;
    newTio.c_cc[VMIN]  = 1;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &newTio );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    return true;
}


void gps_ublox5_init( string rootname, SGPropertyNode *config ) {
    bind_input( config );
    bind_output( rootname );
    gps_ublox5_open();
}


// swap big/little endian bytes
void my_swap( uint8_t *buf, int index, int count ) {
    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
}


static bool parse_ublox5_msg( uint8_t msg_class, uint8_t msg_id,
			      uint16_t payload_length, uint8_t *payload )
{
    bool new_position = false;
    static bool set_system_time = false;

    if ( msg_class == 1 && msg_id == 2 ) {
	// NAV-POSLLH
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 20, 4);
	my_swap( payload, 24, 4);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	int32_t lon = *((int32_t *)(p+4));
	int32_t lat = *((int32_t *)(p+8));
	int32_t height = *((int32_t *)(p+12));
	int32_t hMSL = *((int32_t *)(p+16));
	uint32_t hAcc = *((uint32_t *)(p+20));
	uint32_t vAcc = *((uint32_t *)(p+24));
	// printf("nav-posllh (%d) %d %d %d %d\n", iTOW, lon, lat, height, hMSL);
    } else if ( msg_class == 1 && msg_id == 6 ) {
	// NAV-SOL
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 2);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 20, 4);
	my_swap( payload, 24, 4);
	my_swap( payload, 28, 4);
	my_swap( payload, 32, 4);
	my_swap( payload, 36, 4);
	my_swap( payload, 40, 4);
	my_swap( payload, 44, 2);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	int32_t fTOW = *((int32_t *)(p+4));
	int16_t week = *((int16_t *)(p+8));
	uint8_t gpsFix = p[10];
	uint8_t flags = p[11];
	int32_t ecefX = *((int32_t *)(p+12));
	int32_t ecefY = *((int32_t *)(p+16));
	int32_t ecefZ = *((int32_t *)(p+20));
	uint32_t pAcc = *((uint32_t *)(p+24));
	int32_t ecefVX = *((int32_t *)(p+28));
	int32_t ecefVY = *((int32_t *)(p+32));
	int32_t ecefVZ = *((int32_t *)(p+36));
	uint32_t sAcc = *((uint32_t *)(p+40));
	uint16_t pDOP = *((uint16_t *)(p+44));
	uint8_t numSV = p[47];
	// printf("nav-sol (%d) %d %d %d %d %d [ %d %d %d ]\n",
	//        gpsFix, iTOW, fTOW, ecefX, ecefY, ecefZ, ecefVX, ecefVY, ecefVZ);
	SGVec3d ecef( ecefX / 100.0, ecefY / 100.0, ecefZ / 100.0 );
	SGGeod wgs84;
	SGGeodesy::SGCartToGeod( ecef, wgs84 );
	SGQuatd ecef2ned = SGQuatd::fromLonLat(wgs84);
	SGVec3d vel_ecef( ecefVX / 100.0, ecefVY / 100.0, ecefVZ / 100.0 );
	// SGVec3d vel_ecef = ecef2ned.backTransform(vel_ned);
	SGVec3d vel_ned = ecef2ned.transform( vel_ecef );
	// printf("my vel ned = %.2f %.2f %.2f\n", vel_ned.x(), vel_ned.y(), vel_ned.z());
	if ( gpsFix == 3 ) {
	    new_position = true;
	    gps_timestamp_node->setDoubleValue( get_Time() );
	    gps_lat_node->setDoubleValue( wgs84.getLatitudeDeg() );
	    gps_lon_node->setDoubleValue( wgs84.getLongitudeDeg() );
	    gps_alt_node->setDoubleValue( wgs84.getElevationM() );
	    gps_vn_node->setDoubleValue( vel_ned.x() );
	    gps_ve_node->setDoubleValue( vel_ned.y() );
	    gps_vd_node->setDoubleValue( vel_ned.z() );
	    // printf("        %.10f %.10f %.2f - %.2f %.2f %.2f\n",
	    //     wgs84.getLatitudeDeg(),
	    //     wgs84.getLongitudeDeg(),
	    //     wgs84.getElevationM(),
	    //     vel_ned.x(), vel_ned.y(), vel_ned.z() );

	    double julianDate = (week * 7.0) + 
		(0.001 * iTOW) / 86400.0 +  //86400 = seconds in 1 day
		2444244.5; // 2444244.5 Julian date of GPS epoch (Jan 5 1980 at midnight)
	    julianDate = julianDate - 2440587.5; // Subtract Julian Date of Unix Epoch (Jan 1 1970)

	    double unixSecs = julianDate * 86400.0;
	    double unixFract = unixSecs - floor(unixSecs);
	    struct timeval time;
	    gps_unix_sec_node->setDoubleValue( unixSecs );
	    if ( unixSecs > 1263154775 && !set_system_time) {
		printf("Setting system time to %.3f\n", unixSecs);
		set_system_time = true;
		time.tv_sec = floor(unixSecs);
		time.tv_usec = floor(unixFract * 1000000.);
		settimeofday(&time, NULL);
	    }
	}
   } else if ( msg_class == 1 && msg_id == 18 ) {
	// NAV-VELNED
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 20, 4);
	my_swap( payload, 24, 4);
	my_swap( payload, 28, 4);
	my_swap( payload, 32, 4);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	int32_t velN = *((int32_t *)(p+4));
	int32_t velE = *((int32_t *)(p+8));
	int32_t velD = *((int32_t *)(p+12));
	uint32_t speed = *((uint32_t *)(p+16));
	uint32_t gspeed = *((uint32_t *)(p+20));
	int32_t heading = *((int32_t *)(p+24));
	uint32_t sAcc = *((uint32_t *)(p+28));
	uint32_t cAcc = *((uint32_t *)(p+32));
	// printf("nav-velned (%d) %.2f %.2f %.2f s = %.2f h = %.2f\n", iTOW, velN / 100.0, velE / 100.0, velD / 100.0, speed / 100.0, heading / 100000.0);
    } else if ( msg_class == 1 && msg_id == 33 ) {
	// NAV-TIMEUTC
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 2);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	uint32_t tAcc = *((uint32_t *)p+4);
	int32_t nano = *((int32_t *)(p+8));
	int16_t year = *((int16_t *)(p+12));
	uint8_t month = p[14];
	uint8_t day = p[15];
	uint8_t hour = p[16];
	uint8_t min = p[17];
	uint8_t sec = p[18];
	uint8_t valid = p[19];
#if 0
	if ( !set_system_time && year > 2009 ) {
	    set_system_time = true;
	    printf("nav-timeutc (%d) %02x %04d/%02d/%02d %02d:%02d:%02d\n",
		   iTOW, valid, year, month, day, hour, min, sec);
	    struct tm gps_time;
	    gps_time.tm_sec = sec;
	    gps_time.tm_min = min;
	    gps_time.tm_hour = hour;
	    gps_time.tm_mday = day;
	    gps_time.tm_mon = month - 1;
	    gps_time.tm_year = year - 1900;
	    time_t unix_sec = mktime( &gps_time );
	    printf("gps->unix time = %d\n", (int)unix_sec);
	    struct timeval fulltime;
	    fulltime.tv_sec = unix_sec;
	    fulltime.tv_usec = nano / 1000;
	    settimeofday( &fulltime, NULL );
	}
#endif
    }

    return new_position;
}

static bool read_ublox5() {
    static int state = 0;
    static int msg_class = 0, msg_id = 0;
    static int length_lo = 0, length_hi = 0, payload_length = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    uint8_t payload[500];

    // printf("read ublox5, entry state = %d\n", state);

    bool new_position = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != 0xB5 ) {
	    // fprintf( stderr, "state0: len = %d val = %2X\n", len, input[0] );
	    len = read( fd, input, 1 );
	}
	if ( len > 0 && input[0] == 0xB5 ) {
	    // fprintf( stderr, "read 0xB5\n");
	    state++;
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    if ( input[0] == 0x62 ) {
		// fprintf( stderr, "read 0x62\n");
		state++;
	    } else if ( input[0] == 0xB5 ) {
		// fprintf( stderr, "read 0xB5\n");
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    msg_class = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    // fprintf( stderr, "msg class = %d\n", msg_class );
	    state++;
	}
    }
    if ( state == 3 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    msg_id = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    // fprintf( stderr, "msg id = %d\n", msg_id );
	    state++;
	}
    }
    if ( state == 4 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    length_lo = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    state++;
	}
    }
    if ( state == 5 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    length_hi = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    payload_length = length_hi*256 + length_lo;
	    // fprintf( stderr, "payload len = %d\n", payload_length );
	    if ( payload_length > 400 ) {
		state = 0;
	    } else {
		state++;
	    }
	}
    }
    if ( state == 6 ) {
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    payload[counter++] = input[0];
	    //fprintf( stderr, "%02X ", input[0] );
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    if ( counter >= payload_length ) {
		break;
	    }
	    len = read( fd, input, 1 );
	}

	if ( counter >= payload_length ) {
	    state++;
	    //fprintf( stderr, "\n" );
	}
    }
    if ( state == 7 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_lo = input[0];
	    state++;
	}
    }
    if ( state == 8 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_hi = input[0];
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
		// fprintf( stderr, "checksum passes (%d)!\n", msg_id );
		new_position = parse_ublox5_msg( msg_class, msg_id,
						 payload_length, payload );
		state++;
	    } else {
		if ( display_on ) {
		    printf("checksum failed %d %d (computed) != %d %d (message)\n",
			   cksum_A, cksum_B, cksum_lo, cksum_hi );
		}
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    return new_position;
}


bool gps_ublox5_update() {
    // run an iteration of the ublox5 scanner/parser
    bool gps_data_valid = read_ublox5();

    return gps_data_valid;
 }


void gps_ublox5_close() {
}
