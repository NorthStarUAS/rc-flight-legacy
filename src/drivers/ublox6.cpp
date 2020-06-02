/**
 * \file: gps_ublox6.cpp
 *
 * u-blox 6 protocol driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#include <pyprops.h>

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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "include/globaldefs.h"

#include "comms/display.h"
#include "init/globals.h"
#include "util/geodesy.h"
#include "util/strutils.h"
#include "util/timing.h"

#include "ublox6.h"


// property nodes
static pyPropertyNode gps_node;

static int fd = -1;
static string device_name = "/dev/ttyS0";
static int baud = 57600;
static int gps_fix_value = 0;

// initialize gpsd input property nodes
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("device") ) {
	device_name = config->getString("device");
    }
    if ( config->hasChild("baud") ) {
	baud = config->getLong("baud");
    }
}


// initialize gpsd output property nodes 
static void bind_output( string output_node ) {
    gps_node = pyGetNode(output_node, true);
}


// send our configured init strings to configure gpsd the way we prefer
static bool gps_ublox6_open() {
    if ( display_on ) {
	printf("ublox6 on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// Serial port settings
    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(fd,&oldTio); 

    int config_baud = B115200;
    if ( baud == 115200 ) {
	config_baud = B115200;
    } else if ( baud == 57600 ) {
	config_baud = B57600;
    } else if ( baud == 9600 ) {
	config_baud = B9600;
    } else {
	fprintf( stderr, "ublox6 baud rate (%d) unsupported by driver, using back to 115200.\n", baud);
    }

    // Configure New Serial Port Settings
    config.c_cflag     = config_baud | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    // Enable non-blocking IO (one more time for good measure)
    fcntl(fd, F_SETFL, O_NONBLOCK);

    return true;
}


void gps_ublox6_init( string output_node, pyPropertyNode *config ) {
    bind_input( config );
    bind_output( output_node );
    gps_ublox6_open();
}


// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count ) {
#if defined( __powerpc__ )
    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
#endif
}


static bool parse_ublox6_msg( uint8_t msg_class, uint8_t msg_id,
			      uint16_t payload_length, uint8_t *payload )
{
    bool new_position = false;
    static bool set_system_time = false;

    if ( msg_class == 0x01 && msg_id == 0x02 ) {
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
	// uint32_t hAcc = *((uint32_t *)(p+20));
	// uint32_t vAcc = *((uint32_t *)(p+24));
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-posllh (%d) %d %d %d %d\n",
		       iTOW, lon, lat, height, hMSL);
	    }
	}
    } else if ( msg_class == 0x01 && msg_id == 0x06 ) {
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
	uint32_t iTOW = *((uint32_t *)(p+0));
	int32_t fTOW = *((int32_t *)(p+4));
	int16_t week = *((int16_t *)(p+8));
	uint8_t gpsFix = p[10];
	// uint8_t flags = p[11];
	int32_t ecefX = *((int32_t *)(p+12));
	int32_t ecefY = *((int32_t *)(p+16));
	int32_t ecefZ = *((int32_t *)(p+20));
	// uint32_t pAcc = *((uint32_t *)(p+24));
	int32_t ecefVX = *((int32_t *)(p+28));
	int32_t ecefVY = *((int32_t *)(p+32));
	int32_t ecefVZ = *((int32_t *)(p+36));
	// uint32_t sAcc = *((uint32_t *)(p+40));
	// uint16_t pDOP = *((uint16_t *)(p+44));
	uint8_t numSV = p[47];
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-sol (%d) %d %d %d %d %d [ %d %d %d ]\n",
		       gpsFix, iTOW, fTOW, ecefX, ecefY, ecefZ,
		       ecefVX, ecefVY, ecefVZ);
	    }
	}
	Vector3d ecef( ecefX / 100.0, ecefY / 100.0, ecefZ / 100.0 );
	Vector3d wgs84 = ecef2lla_for_ublox6(ecef);
	Quaterniond ecef2ned = fromLonLatRad(wgs84[1], wgs84[0]);
	Vector3d vel_ecef( ecefVX / 100.0, ecefVY / 100.0, ecefVZ / 100.0 );
	Vector3d vel_ned = quat_backtransform(ecef2ned, vel_ecef);
	// printf("my vel ned = %.2f %.2f %.2f\n", vel_ned.x(), vel_ned.y(), vel_ned.z());

 	gps_node.setLong( "satellites", numSV );
 	gps_fix_value = gpsFix;
	if ( gps_fix_value == 0 ) {
	    gps_node.setLong( "status", 0 );
	} else if ( gps_fix_value == 1 || gps_fix_value == 2 ) {
	    gps_node.setLong( "status", 1 );
	} else if ( gps_fix_value == 3 ) {
	    gps_node.setLong( "status", 2 );
	}

	if ( fabs(ecefX) > 650000000
	     || fabs(ecefY) > 650000000
	     || fabs(ecefZ) > 650000000 ) {
	    // earth radius is about 6371km (637,100,000 cm).  If one
	    // of the ecef coordinates is beyond this radius we know
	    // we have bad data.  This means we won't toss data until
	    // above about 423,000' MSL
	    events->log( "ublox6", "received bogus ecef data" );
	} else if ( wgs84[2] > 60000 ) {
	    // sanity check: assume altitude > 60k meters (200k feet) is bad
	} else if ( wgs84[2] < -1000 ) {
	    // sanity check: assume altitude < -1000 meters (-3000 feet) is bad
	} else if ( gpsFix == 3 ) {
	    // passed basic sanity checks and gps is reporting a 3d fix
	    new_position = true;
	    gps_node.setDouble( "timestamp", get_Time() );
	    gps_node.setDouble( "latitude_deg", wgs84[0] * 180.0 / M_PI );
	    gps_node.setDouble( "longitude_deg", wgs84[1] * 180.0 / M_PI );
	    gps_node.setDouble( "altitude_m", wgs84[2] );
	    gps_node.setDouble( "vn_ms", vel_ned.x() );
	    gps_node.setDouble( "ve_ms", vel_ned.y() );
	    gps_node.setDouble( "vd_ms", vel_ned.z() );
	    // printf("        %.10f %.10f %.2f - %.2f %.2f %.2f\n",
	    //        wgs84.getLatitudeDeg(),
	    //        wgs84.getLongitudeDeg(),
	    //        wgs84.getElevationM(),
	    //        vel_ned.x(), vel_ned.y(), vel_ned.z() );

	    double julianDate = (week * 7.0) + 
		(0.001 * iTOW) / 86400.0 +  //86400 = seconds in 1 day
		2444244.5; // 2444244.5 Julian date of GPS epoch (Jan 5 1980 at midnight)
	    julianDate = julianDate - 2440587.5; // Subtract Julian Date of Unix Epoch (Jan 1 1970)

	    double unixSecs = julianDate * 86400.0;
	    //double unixFract = unixSecs - floor(unixSecs);
	    //struct timeval time;
	    gps_node.setDouble( "unix_time_sec", unixSecs );
#if 0
	    if ( unixSecs > 1263154775 && !set_system_time) {
		printf("Setting system time to %.3f\n", unixSecs);
		set_system_time = true;
		time.tv_sec = floor(unixSecs);
		time.tv_usec = floor(unixFract * 1000000.);
		settimeofday(&time, NULL);
	    }
#endif
	}
   } else if ( msg_class == 0x01 && msg_id == 0x12 ) {
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
	// uint32_t gspeed = *((uint32_t *)(p+20));
	int32_t heading = *((int32_t *)(p+24));
	// uint32_t sAcc = *((uint32_t *)(p+28));
	// uint32_t cAcc = *((uint32_t *)(p+32));
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-velned (%d) %.2f %.2f %.2f s = %.2f h = %.2f\n",
		       iTOW, velN / 100.0, velE / 100.0, velD / 100.0,
		       speed / 100.0, heading / 100000.0);
	    }
	}
    } else if ( msg_class == 0x01 && msg_id == 0x21 ) {
	// NAV-TIMEUTC
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 2);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)(p+0));
	// uint32_t tAcc = *((uint32_t *)(p+4));
	int32_t nano = *((int32_t *)(p+8));
	int16_t year = *((int16_t *)(p+12));
	uint8_t month = p[14];
	uint8_t day = p[15];
	uint8_t hour = p[16];
	uint8_t min = p[17];
	uint8_t sec = p[18];
	uint8_t valid = p[19];
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-timeutc (%d) %02x %04d/%02d/%02d %02d:%02d:%02d\n",
		       iTOW, valid, year, month, day, hour, min, sec);
	    }
	}
	if ( !set_system_time && year > 2009 ) {
	    set_system_time = true;
	    printf("set system clock: nav-timeutc (%d) %02x %04d/%02d/%02d %02d:%02d:%02d\n",
		   iTOW, valid, year, month, day, hour, min, sec);
	    struct tm gps_time;
	    gps_time.tm_sec = sec;
	    gps_time.tm_min = min;
	    gps_time.tm_hour = hour;
	    gps_time.tm_mday = day;
	    gps_time.tm_mon = month - 1;
	    gps_time.tm_year = year - 1900;
	    time_t unix_sec = mktime( &gps_time ) - timezone;
	    printf("gps->unix time = %d\n", (int)unix_sec);
	    struct timeval fulltime;
	    fulltime.tv_sec = unix_sec;
	    fulltime.tv_usec = nano / 1000;
	    settimeofday( &fulltime, NULL );
	}
    } else if ( msg_class == 0x01 && msg_id == 0x30 ) {
	// NAV-SVINFO (partial parse)
	my_swap( payload, 0, 4);

	uint8_t *p = payload;
	// uint32_t iTOW = *((uint32_t *)(p+0));
	uint8_t numCh = p[4];
	// uint8_t globalFlags = p[5];
	int satUsed = 0;
	for ( int i = 0; i < numCh; i++ ) {
	    // uint8_t satid = p[9 + 12*i];
	    // uint8_t flags = p[10 + 12*i];
	    uint8_t quality = p[11 + 12*i];
	    // printf(" chn=%d satid=%d flags=%d quality=%d\n", i, satid, flags, quality);
	    if ( quality > 3 ) {
		satUsed++;
	    }
	}
 	// gps_satellites_node.setLong( satUsed );
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("Satellite count = %d/%d\n", satUsed, numCh);
	    }
	}
    } else {
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("ublox6 msg class = %d  msg id = %d\n",
		       msg_class, msg_id);
	    }
	}
    }

    return new_position;
}

static bool read_ublox6() {
    static int state = 0;
    static int msg_class = 0, msg_id = 0;
    static int length_lo = 0, length_hi = 0, payload_length = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // printf("read ublox6, entry state = %d\n", state);

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
		new_position = parse_ublox6_msg( msg_class, msg_id,
						 payload_length, payload );
		state++;
	    } else {
		if ( display_on && 0 ) {
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


bool gps_ublox6_update() {
    // run an iteration of the ublox scanner/parser
    bool gps_data_valid = read_ublox6();

    return gps_data_valid;
 }


void gps_ublox6_close() {
}
