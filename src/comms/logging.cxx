#include "python/pyprops.hxx"

#include <sys/types.h>		// opendir() mkdir()
#include <dirent.h>		// opendir()
#include <stdio.h>		// sscanf()
#include <stdlib.h>		// random()
#include <string.h>		// strncmp()
#include <sys/stat.h>		// mkdir()
#include <sys/time.h>
#include <zlib.h>

#include "init/globals.hxx"
#include "util/timing.h"

#include "checksum.hxx"
#include "display.hxx"
#include "netBuffer.h"		// for netBuffer structure

#include "logging.hxx"

// global variables for data file logging
static netBuffer log_buffer(1024);
static gzFile fdata = NULL;

bool log_to_file = false;  // log to file is enabled/disabled
SGPath log_path;	   // base log path
static SGPath flight_dir;  // dir containing all our logged data

// scan the base path for fltNNNN directories.  Return the biggest
// flight number
int max_flight_num() {
    int max = -1;

    DIR *d = opendir( log_path.c_str() );
    if ( d == NULL ) {
        printf( "Cannot open %s for log writing\n", log_path.c_str() );
        return max;
    }

    struct dirent *file;
    while ( ( file = readdir(d) ) != NULL  ) {
        if ( strncmp( file->d_name, "flt", 3 ) == 0 ) {
            int num;
            sscanf( file->d_name, "flt%d", &num );
            if ( num > max ) {
                max = num;
            }
            printf("file = %s  num = %d  max = %d\n", file->d_name, num, max);
        }
    }

    if ( closedir( d ) != 0 ) {
        printf("Error: cannot close log directory\n");
        return -1;
    }

    return max;
}


bool logging_init() {
    // find the biggest flight number logged so far
    int max = max_flight_num();
    printf("Max log dir index is %05d\n", max);

    // make the new logging directory
    char new_dir[256];
    snprintf( new_dir, 256, "flt%05d", max+1 );
    flight_dir = log_path;
    flight_dir.append(new_dir);
    printf("Creating log dir: %s\n", flight_dir.c_str());
    int result = mkdir( flight_dir.c_str(), 00777 );
    if ( result != 0 ) {
        printf("Error: creating %s\n", flight_dir.c_str());
    }

    // open the logging files

    SGPath file;

    file = flight_dir; file.append( "flight.dat.gz" );
    if ( (fdata = gzopen( file.c_str(), "wb" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    events->open(flight_dir.c_str());
    events->log("Log", "Start");
    
    return true;
}


bool logging_close() {
    // close files

    gzclose(fdata);

    return true;
}


static int log_queue( const uint8_t *buf, const short size ) {
    bool result = log_buffer.append((char *)buf, size);
    if ( ! result and display_on ) {
	printf("log buffer overflow\n");
    }
    return result;
}


static void log_packet( const uint8_t packet_id,
			const uint8_t *packet_buf,
			const int packet_size )
{
    const int MAX_PACKET_SIZE = 256;

    // printf(" begin log_packet()\n");
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t *ptr = buf;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    ptr[0] = START_OF_MSG0; ptr[1] = START_OF_MSG1;
    ptr += 2;

    // packet id (1 byte)
    ptr[0] = packet_id;
    ptr += 1;

    // packet size (1 byte)
    ptr[0] = packet_size;
    ptr += 1;

    // copy packet data
    memmove( ptr, packet_buf, packet_size );
    ptr += packet_size;

    // check sum (2 bytes)
    aura_cksum( packet_id, packet_size, packet_buf, packet_size,
		&cksum0, &cksum1 );
    ptr[0] = cksum0; ptr[1] = cksum1;
    /*if ( packet_id == 2 ) {
	printf("cksum = %d %d\n", cksum0, cksum1);
    }*/

    log_queue( buf, packet_size + 6 );
    // printf(" end log_packet()\n");
}


// return a random integer between 0 and max - 1
static int my_random( int max ) {
    int result = (int)(((double)random() / RAND_MAX) * max);
    // printf("log rand(%d) = %d\n", max, result);
    return result;
}


void log_gps( uint8_t *buf, int size ) {
    log_packet( GPS_PACKET_V2, buf, size );
}


void log_imu( uint8_t *buf, int size ) {
    log_packet( IMU_PACKET_V3, buf, size );
}


void log_airdata( uint8_t *buf, int size ) {
    log_packet( AIRDATA_PACKET_V5, buf, size );
}


void log_filter( uint8_t *buf, int size ) {
    log_packet( FILTER_PACKET_V2, buf, size );
}


void log_actuator( uint8_t *buf, int size ) {
    log_packet( ACTUATOR_PACKET_V2, buf, size );
}


void log_pilot( uint8_t *buf, int size ) {
    log_packet( PILOT_INPUT_PACKET_V2, buf, size );
}


void log_ap( uint8_t *buf, int size ) {
    log_packet( AP_STATUS_PACKET_V3, buf, size );
}


void log_health( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    log_packet( SYSTEM_HEALTH_PACKET_V4, buf, size );
}

void log_payload( uint8_t *buf, int size ) {
    log_packet( PAYLOAD_PACKET_V2, buf, size );
}

void log_raven( uint8_t *buf, int size ) {
    log_packet( RAVEN_PACKET_V1, buf, size );
}


void log_flush() {
    int size = log_buffer.getLength();
    if ( size > 0 ) {
	// printf("flush log data (bytes=%d)\n", size);
	int result = gzwrite( fdata, log_buffer.getData(), size );
	log_buffer.remove(0, result);
    }
    gzflush( fdata, Z_SYNC_FLUSH );
}


// write out the imu calibration parameters associated with this data
// (this allows us to later rederive the original raw sensor values.)
bool log_imu_calibration( pyPropertyNode *config ) {
    SGPath file = flight_dir;
    file.append( "imucal.xml" );
    return writeXML( file.str(), config );
}


// write out the autopilot configuration
bool log_ap_config() {
    pyPropertyNode config_props = pyGetNode( "/config/autopilot", true );
    SGPath file = flight_dir;
    file.append( "ap-config.xml" );
    return writeXML( file.str(), &config_props );
}

// write out the master configuration tree
bool log_master_config() {
    pyPropertyNode root = pyGetNode( "/config", true );
    SGPath file = flight_dir;
    file.append( "master-config.xml" );
    return writeXML( file.str(), &root );
}
