/**
 * \file: gps_ublox_config.cxx
 *
 * u-blox configuration utility - attempts to robustly update the configuration
 * of a ublox gps receiver based on the a config.txt file saved by u-center.
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
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

#include "include/globaldefs.h"

#include "math/SGMath.hxx"
#include "math/SGGeodesy.hxx"
#include "util/strutils.hxx"
#include "util/timing.h"


static int fd = -1;
static string device_name = "/dev/ttyS0";

// message acknowledgement
unsigned char ack_msg_class = 0;
unsigned char ack_msg_id = 0;
bool ack_msg = false;
 

// satisfy library linkage
int display_on = 0;

// send our configured init strings to configure gpsd the way we prefer
static bool gps_ublox_open(tcflag_t baud) {
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

    // Configure New Serial Port Settings
    config.c_cflag     = baud | // bps rate
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


// NOTICE: this routine looks for things in the form of nmea messages
// but doesn't checksum, validate, or parse them.
static bool read_ublox_nmea() {
    static int state = 0;
    static int counter = 0;
    int len;
    char input[8];
    static char payload[257];	// 256+1 for a null string terminator

    // printf("read ublox, entry state = %d\n", state);

    bool new_message = false;

    if ( state == 0 ) {
	counter = 0;
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    //printf("ublox read: %d\n", (unsigned int)input[0]);
	    if ( input[0] == '$' ) {
		state = 1;
		break;
	    }
	    len = read( fd, input, 1 );
	    //printf("ublox read: %d\n", (unsigned int)input[0]);
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	//printf("ublox read: %d (%d)\n", (unsigned int)input[0], counter);
	while ( len > 0 && counter < 256 ) {
	    if ( input[0] == '\r' ) {
		payload[counter] = 0;
		state = 2;
		break;
	    } else {
		payload[counter++] = input[0];
	    }
	    len = read( fd, input, 1 );
	    //printf("ublox read: %d (%d)\n", (unsigned int)input[0], counter);
	}
	if ( counter >= 256 ) {
	    state = 0;
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	//printf("ublox read: %d\n", (unsigned int)input[0]);
	if ( len > 0 ) {
	    if ( input[0] == '\n' ) {
		new_message = true;
		// printf("ublox: calling parser with '%s'\n", payload);
		// parse_nmea_msg( payload, counter );
	    }
	    state = 0;
	}
	if ( counter >= 256 ) {
	    state = 0;
	}
    }

    return new_message;
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


static bool parse_ublox_msg( uint8_t msg_class, uint8_t msg_id,
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
	uint32_t hAcc = *((uint32_t *)(p+20));
	uint32_t vAcc = *((uint32_t *)(p+24));
	if ( 0 ) {
	    printf("nav-posllh (%d) %d %d %d %d\n",
		   iTOW, lon, lat, height, hMSL);
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
	if ( 0 ) {
	    printf("nav-sol (%d) %d %d %d %d %d [ %d %d %d ]\n",
		   gpsFix, iTOW, fTOW, ecefX, ecefY, ecefZ,
		   ecefVX, ecefVY, ecefVZ);
	}
	SGVec3d ecef( ecefX / 100.0, ecefY / 100.0, ecefZ / 100.0 );
	SGGeod wgs84;
	SGGeodesy::SGCartToGeod( ecef, wgs84 );
	SGQuatd ecef2ned = SGQuatd::fromLonLat(wgs84);
	SGVec3d vel_ecef( ecefVX / 100.0, ecefVY / 100.0, ecefVZ / 100.0 );
	// SGVec3d vel_ecef = ecef2ned.backTransform(vel_ned);
	SGVec3d vel_ned = ecef2ned.transform( vel_ecef );
	// printf("my vel ned = %.2f %.2f %.2f\n", vel_ned.x(), vel_ned.y(), vel_ned.z());

	if ( fabs(ecefX) > 650000000
	     || fabs(ecefY) > 650000000
	     || fabs(ecefZ) > 650000000 ) {
	    // earth radius is about 6371km (637,100,000 cm).  If one
	    // of the ecef coordinates is beyond this radius we know
	    // we have bad data.  This means we won't toss data until
	    // above about 423,000' MSL
	    printf( "ublox: received bogus ecef data\n" );
	} else if ( wgs84.getElevationM() > 60000 ) {
	    // sanity check: assume altitude > 60k meters (200k feet) is bad
	} else if ( wgs84.getElevationM() < -1000 ) {
	    // sanity check: assume altitude < -1000 meters (-3000 feet) is bad
	} else if ( gpsFix == 3 ) {
	    // passed basic sanity checks and gps is reporting a 3d fix
	    new_position = true;
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
	    double unixFract = unixSecs - floor(unixSecs);
	    struct timeval time;
#if 0
	    if ( unixSecs > 1263154775 && !set_system_time) {
		printf("Setting system time to %.3f\n", unixSecs);
		set_system_time = true;
		time.tv_sec = floor(unixSecs);
		time.tv_usec = floor(unixFract * 1000000.);
		settimeofday(&time, NULL);
		// first time through, restamp this packet with the
		// "reset" system time to avoid a huge dt
		gps_timestamp_node->setDouble( get_Time() );
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
	uint32_t gspeed = *((uint32_t *)(p+20));
	int32_t heading = *((int32_t *)(p+24));
	uint32_t sAcc = *((uint32_t *)(p+28));
	uint32_t cAcc = *((uint32_t *)(p+32));
	if ( 0 ) {
	    printf("nav-velned (%d) %.2f %.2f %.2f s = %.2f h = %.2f\n",
		   iTOW, velN / 100.0, velE / 100.0, velD / 100.0,
		   speed / 100.0, heading / 100000.0);
	}
    } else if ( msg_class == 0x01 && msg_id == 0x21 ) {
	// NAV-TIMEUTC
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 2);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)(p+0));
	uint32_t tAcc = *((uint32_t *)(p+4));
	int32_t nano = *((int32_t *)(p+8));
	int16_t year = *((int16_t *)(p+12));
	uint8_t month = p[14];
	uint8_t day = p[15];
	uint8_t hour = p[16];
	uint8_t min = p[17];
	uint8_t sec = p[18];
	uint8_t valid = p[19];
	if ( 0 ) {
	    printf("nav-timeutc (%d) %02x %04d/%02d/%02d %02d:%02d:%02d\n",
		   iTOW, valid, year, month, day, hour, min, sec);
	}
#if 0
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
	    time_t unix_sec = mktime( &gps_time );
	    printf("gps->unix time = %d\n", (int)unix_sec);
	    struct timeval fulltime;
	    fulltime.tv_sec = unix_sec;
	    fulltime.tv_usec = nano / 1000;
	    settimeofday( &fulltime, NULL );
	    get_Time(); 	// reset precise clock timer to zero
	}
#endif
    } else if ( msg_class == 0x01 && msg_id == 0x30 ) {
	// NAV-SVINFO (partial parse)
	my_swap( payload, 0, 4);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)(p+0));
	uint8_t numCh = p[4];
	uint8_t globalFlags = p[5];
	int satUsed = 0;
	for ( int i = 0; i < numCh; i++ ) {
	    uint8_t satid = p[9 + 12*i];
	    uint8_t flags = p[10 + 12*i];
	    uint8_t quality = p[11 + 12*i];
	    // printf(" chn=%d satid=%d flags=%d quality=%d\n", i, satid, flags, quality);
	    if ( quality > 3 ) {
		satUsed++;
	    }
	}
 	// gps_satellites_node->setLong( satUsed );
	if ( 0 ) {
	    printf("Satellite count = %d/%d\n", satUsed, numCh);
	}
    } else if ( msg_class == 0x05 && msg_id == 0x01 ) {
	// ACK-ACK
	ack_msg_class = payload[0];
	ack_msg_id = payload[1];
	/* printf("received an ACK-ACK for %02X %02X\n",
	   ack_msg_class, ack_msg_id); */
	ack_msg = true;
    } else if ( msg_class == 0x05 && msg_id == 0x00 ) {
	// ACK-NAK
	printf("Received ACK-NAK\n");
	ack_msg_class = payload[0];
	ack_msg_id = payload[1];
	ack_msg = false;
    } else {
	printf("UBLOX msg class = %02X  msg id = %02X\n",
	       msg_class, msg_id);
    }

    return new_position;
}

static bool read_ublox_ubx() {
    static int state = 0;
    static int msg_class = 0, msg_id = 0;
    static int length_lo = 0, length_hi = 0, payload_length = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // fprintf("read ublox, entry state = %d\n", state);

    bool new_message = false;

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
	    // fprintf( stderr, "%02X ", input[0] );
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    if ( counter >= payload_length ) {
		break;
	    }
	    len = read( fd, input, 1 );
	}

	if ( counter >= payload_length ) {
	    state++;
	    // fprintf( stderr, "\n" );
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
		new_message = true;
		parse_ublox_msg( msg_class, msg_id, payload_length, payload );
		state++;
	    } else {
		if ( 0 ) {
		    printf("checksum failed %d %d (computed) != %d %d (message)\n",
			   cksum_A, cksum_B, cksum_lo, cksum_hi );
		}
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    return new_message;
}


int gps_ublox_make_package( string line, unsigned char *payload ) {
    unsigned int counter = 0;
    unsigned char cksum_A = 0, cksum_B = 0;

    vector <string> token = split( line, " " );
    if ( token.size() < 6 ) {
        // no valid message
        return false;
    }

    // ubx start of msg
    payload[counter++] = 0xb5;
    payload[counter++] = 0x62;

    // assemble config message
    for ( unsigned int i = 2; i < token.size(); i++ ) {
	/* printf("token = %s - %c %c ", token[i].c_str(),
	       token[i].c_str()[0],
	       token[i].c_str()[1]); */
	if ( token[i].length() != 2 ) {
	    printf("invalid message token: '%s' (aborting!)\n",
		   token[i].c_str() );
	    exit(-1);
	}
	unsigned int byte = 0;
	sscanf(token[i].c_str(), "%x", &byte);
	// printf("message byte = %02X\n", byte);
	payload[counter++] = byte;
	cksum_A += byte;
	cksum_B += cksum_A;
    }
    //printf("\n");

    payload[counter++] = cksum_A;
    payload[counter++] = cksum_B;

    return counter;
}

int gps_ublox_getack( int target_class, int target_id, double listen_time ) {
    // look for ack
    ack_msg = false;
    double start_time = get_Time();
    while ( get_Time() < start_time + listen_time ) {
	/*unsigned char input[256];
	int len = read( fd, input, 1 );
	if ( len > 0 ) {
	    printf("%02X\n", input[0]);
	    }*/
	if ( read_ublox_ubx() ) {
	    if ( ack_msg ) {
	    }
	}
    }

    if ( ack_msg && ack_msg_class == target_class && ack_msg_id== target_id ) {
	// received proper ack
	printf("Received proper ack\n");
	return true;
    }

    return false;
}


bool gps_ublox_send( unsigned char *payload, unsigned int size ) {
    unsigned int len = write( fd, payload, size );
    len = write( fd, payload, size );
    if ( len != size ) {
	printf("wrote %d of %d bytes -- try again\n", len, size);
	return false;
    } else {
	// printf("wrote %d bytes\n", len);
	return true;
    }

}


bool gps_ublox_send_with_ack( string line, int num_tries, double listen_time )
{
    bool result = false;
    unsigned char payload[256];
    unsigned int size = 0;

    size = gps_ublox_make_package( line, payload );
    if ( payload[2] != 0x06 ) {
	printf("Not a CFG message class (%d), skipping\n", payload[2]);
	return false;
    }
    /*printf("message size = %d\n", size);
    for ( int i = 0; i < size; i++ ) {
	printf("%02X ", payload[i]);
    }
    printf("\n");*/

    for ( int i = 0; i < num_tries; i++ ) {
	gps_ublox_send( payload, size );

	if ( gps_ublox_getack( payload[2], payload[3], listen_time ) ) {
	    result = true;
	    break;
	} else {
	    printf("No message ACK (OK if changing baud rate)\n");
	}
    }

    return result;
}


void gps_ublox_close() {
    close( fd );
}


void usage( char *prog ) {
    printf("Usage: %s <device> <config.txt>\n", prog );
    exit(-1);
}


tcflag_t bauds[] = { B9600,  B57600,  B115200,  B38400,  B4800,  B19200,  B0 };
string sbauds[] =  { "9600", "57600", "115200", "38400", "4800", "19200", "0" };

int main( int argc, char **argv ) {

    if ( argc != 3 ) {
	usage( argv[0] );
    }

    device_name = argv[1];
    string config_file = argv[2];

    // determine current baud by opening the gps at various bauds and
    // listening for valid messages until we find something.

    int bindex = 0;

    while ( bauds[bindex] != B0 ) {
	// open at the next baud rate
	printf("ublox: trying on %s at %s baud\n",
	       device_name.c_str(), sbauds[bindex].c_str() );
	gps_ublox_open( bauds[bindex] );

	double start_time = 0.0;
	double listen_time = 2.0;
	int messages = 0;

	// listen for NMEA message
	start_time = get_Time();
	messages = 0;
	while ( get_Time() < start_time + listen_time ) {
	    if ( read_ublox_nmea() ) {
		messages++;
	    }
	}
	printf("  ublox: read %d NMEA messages at %s baud\n",
	       messages, sbauds[bindex].c_str() );
	if ( messages > 0 ) {
	    gps_ublox_close();
	    break;
	}

	// listen for UBX message
	start_time = get_Time();
	messages = 0;
	while ( get_Time() < start_time + listen_time ) {
	    if ( read_ublox_ubx() ) {
		messages++;
	    }
	}
	printf("  ublox: read %d UBX messages at %s baud\n",
	       messages, sbauds[bindex].c_str() );
	if ( messages > 0 ) {
	    gps_ublox_close();
	    break;
	}

	gps_ublox_close();

	bindex++;
    }

    printf( "ublox: detected receiver on %s at %s baud\n",
	    device_name.c_str(), sbauds[bindex].c_str() );

    // abort if no receiver detected
    if ( bauds[bindex] == B0 ) {
	printf("ublox: no receiver detected, exiting.\n");
	exit(-1);
    }

    int num_tries = 5;
    unsigned char payload[256];
    unsigned int size = 0;

    // Note: there is some hard coded assumptions right here:
    // - gps will be talking on it's uart1 port
    // - gps will be configured to talk at 57600
    // - the cfg-msg string we send is actually acommplishing this and not something lese.

    // Send the pre-generated message that will configure uart1 to
    // 57600 baud on the gps

    gps_ublox_open( bauds[bindex] ); // detected baud rate

    string msg = "CFG-PRT - 06 00 14 00 01 00 00 00 C0 08 00 00 00 E1 00 00 07 00 07 00 00 00 00 00";
    printf("\n");
    printf("Sending %s\n", msg.c_str());
    gps_ublox_send_with_ack( msg.c_str(), 5, 0.5 );

    gps_ublox_close();

    // Proceeding under the assumption that the baud rate change was
    // successful.

    // Now load the specified config file and send the data strings
    // one by one (waiting for the proper ACK-ACK before going on to
    // the next.)

    gps_ublox_open( B57600 );	// newly configured baud rate

    FILE *cfg = fopen( config_file.c_str(), "r" );
    if ( cfg == NULL ) {
	printf("Error opening %s (aborted)\n", config_file.c_str());
    }

    while ( !feof(cfg) ) {
	char buffer[4096];
	char *result = fgets( buffer, 4096, cfg );
	if ( result != NULL ) {
	    // trim the trailing nl/cr
	    int size = strlen(buffer);
	    if ( size > 1 ) {
		buffer[size-2] = 0;
	    }

	    printf("\n");
	    printf("Sending %s\n", buffer);
	    gps_ublox_send_with_ack( buffer, 5, 0.5 );
	}
    }

    // now send the message to save all these settings to nvram
    msg = "CFG-CFG - 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 07";
    printf("\n");
    printf("Sending %s\n", msg.c_str());
    gps_ublox_send_with_ack( msg.c_str(), 5, 0.5 );

    gps_ublox_close();
}
