#include <sys/types.h>		// opendir() mkdir()
#include <dirent.h>		// opendir()
#include <stdio.h>		// sscanf()
#include <stdlib.h>		// random()
#include <string.h>		// strncmp()
#include <sys/stat.h>		// mkdir()
#include <sys/time.h>
#include <zlib.h>

#include "filters/mnav/ahrs.h"	// FIXME: need to remove
#include "filters/mnav/nav.h"	// FIXME: need to remove
#include "props/props.hxx"
#include "sensors/gps_mgr.hxx"
#include "util/timing.h"

#include "logging.h"

// global variables for data file logging
static gzFile fgps = NULL;
static gzFile fimu = NULL;
static gzFile fair = NULL;
static gzFile ffilter = NULL;
static gzFile fact = NULL;
static gzFile fpilot = NULL;
static gzFile fap = NULL;
static gzFile fhealth = NULL;
static FILE *fevent = NULL;

bool log_to_file = false;  // log to file is enabled/disabled
SGPath log_path;	   // base log path
bool event_log_on = false; // events log written to events.txt

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
    printf("Max log dir is flt%05d\n", max);

    // make the new logging directory
    char new_dir[256];
    snprintf( new_dir, 256, "%s/flt%05d", log_path.c_str(), max+1 );
    printf("Creating log dir: %s\n", new_dir);
    int result = mkdir( new_dir, 01777 );
    if ( result != 0 ) {
        printf("Error: creating %s\n", new_dir);
    }

    // open all the logging files

    SGPath file;

    file = new_dir; file.append( "imu.dat.gz" );
    if ( (fimu = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "gps.dat.gz" );
    if ( (fgps = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "air.dat.gz" );
    if ( (fair = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "filter.dat.gz" );
    if ( (ffilter = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "actuator.dat.gz" );
    if ( (fact = gzopen( file.c_str(),"w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "pilot.dat.gz" );
    if ( (fpilot = gzopen( file.c_str(),"w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "health.dat.gz" );
    if ( (fhealth = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "ap.dat.gz" );
    if ( (fap = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "events.dat" );
    if ( (fevent = fopen( file.c_str(), "w" )) == NULL ) {
	printf("Cannot open %s\n", file.c_str());
	return false;
    }

    return true;
}


bool logging_close() {
    // close files

    gzclose(fimu);
    gzclose(fgps);
    gzclose(ffilter);
    gzclose(fact);
    gzclose(fpilot);
    gzclose(fap);
    gzclose(fhealth);

    return true;
}


// return a random integer between 0 and max - 1
static int my_random( int max ) {
    int result = (int)(((double)random() / RAND_MAX) * max);
    // printf("log rand(%d) = %d\n", max, result);
    return result;
}

void log_gps( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fgps, buf, size );
}


void log_imu( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fimu, buf, size );
}


void log_airdata( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fair, buf, size );
}


void log_filter( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( ffilter, buf, size );
}


void log_actuator( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fact, buf, size );
}


void log_pilot( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fpilot, buf, size );
}


void log_ap( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = my_random(skip_count);

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fap, buf, size );
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

    gzwrite( fhealth, buf, size );
}


void flush_gps() {
    // printf("flush gps\n");
    gzflush( fgps, Z_SYNC_FLUSH );
}


void flush_imu() {
    // printf("flush imu\n");
    gzflush( fimu, Z_SYNC_FLUSH );
}


void flush_airdata() {
    // printf("flush airdata\n");
    gzflush( fair, Z_SYNC_FLUSH );
}


void flush_filter() {
    // printf("flush filter\n");
    gzflush( ffilter, Z_SYNC_FLUSH );
}


void flush_actuator() {
    // printf("flush actuator\n");
    gzflush( fact, Z_SYNC_FLUSH );
}


void flush_pilot() {
    // printf("flush pilot\n");
    gzflush( fpilot, Z_SYNC_FLUSH );
}


void flush_ap() {
    // printf("flush ap\n");
    gzflush( fap, Z_SYNC_FLUSH );
}


void flush_health() {
    // printf("flush ap\n");
    gzflush( fhealth, Z_SYNC_FLUSH );
}


bool event_log( const char *hdr, const char *msg ) {
    if ( fevent == NULL ) {
	return false;
    }

    fprintf( fevent, "%.3f %s %s\n", get_Time(), hdr, msg );
    fflush( fevent );

    return true;
}
