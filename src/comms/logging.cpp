#include <sys/types.h>		// opendir() mkdir()
#include <dirent.h>		// opendir()
#include <stdio.h>		// sscanf()
#include <string.h>		// strncmp()
#include <sys/stat.h>		// mkdir()
#include <sys/time.h>
#include <zlib.h>

#include "filters/mnav/ahrs.h"	// FIXME: need to remove
#include "filters/mnav/nav.h"	// FIXME: need to remove
#include "props/props.hxx"
#include "sensors/gps_mgr.h"
#include "util/timing.h"

#include "logging.h"

// global variables for data file logging

static FILE *fnavstate = NULL;

static gzFile fgps = NULL;
static gzFile fimu = NULL;
static gzFile fair = NULL;
static gzFile ffilter = NULL;
static gzFile fact = NULL;
static gzFile fpilot = NULL;
static gzFile fap = NULL;

bool log_to_file = false;  // log to file is enabled/disabled
SGPath log_path;	   // base log path
bool display_on = false;   // dump summary to display periodically
bool debug_on = false;	   // extra debugging log written to debug.txt

// imu property nodes
static bool props_inited = false;

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

// air data nodes
static SGPropertyNode *airdata_altitude_node = NULL;
static SGPropertyNode *airdata_airspeed_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_track_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

// filter output nodes
static SGPropertyNode *filter_theta_node = NULL;
static SGPropertyNode *filter_phi_node = NULL;
static SGPropertyNode *filter_psi_node = NULL;
static SGPropertyNode *filter_status_node = NULL;
static SGPropertyNode *filter_lat_node = NULL;
static SGPropertyNode *filter_lon_node = NULL;
static SGPropertyNode *filter_alt_node = NULL;
static SGPropertyNode *filter_vn_node = NULL;
static SGPropertyNode *filter_ve_node = NULL;
static SGPropertyNode *filter_vd_node = NULL;

// actuator property nodes
static SGPropertyNode *act_aileron_node = NULL;
static SGPropertyNode *act_elevator_node = NULL;
static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *act_rudder_node = NULL;
static SGPropertyNode *act_channel5_node = NULL;

// pilot property nodes
static SGPropertyNode *pilot_aileron_node = NULL;
static SGPropertyNode *pilot_elevator_node = NULL;
static SGPropertyNode *pilot_throttle_node = NULL;
static SGPropertyNode *pilot_rudder_node = NULL;
static SGPropertyNode *pilot_channel5_node = NULL;

// health/status nodes
static SGPropertyNode *console_seq_num = NULL;
static SGPropertyNode *target_waypoint = NULL;
static SGPropertyNode *system_load_avg = NULL;


static void init_props() {
    props_inited = true;

    // initialize imu property nodes
    imu_timestamp_node = fgGetNode("/sensors/imu/time-stamp");
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);

    // initialize air data nodes
    airdata_altitude_node = fgGetNode("/sensors/air-data/altitude-m", true);
    airdata_airspeed_node = fgGetNode("/sensors/air-data/airspeed-kt", true);

    // initialize gps property nodes
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_track_node = fgGetNode("/sensors/gps/groundtrack-deg", true);
    gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);

    // initialize filter property nodes 
    filter_theta_node = fgGetNode("/orientation/pitch-deg", true);
    filter_phi_node = fgGetNode("/orientation/roll-deg", true);
    filter_psi_node = fgGetNode("/orientation/heading-deg", true);
    filter_lat_node = fgGetNode("/position/latitude-deg", true);
    filter_lon_node = fgGetNode("/position/longitude-deg", true);
    filter_alt_node = fgGetNode("/position/altitude-m", true);
    filter_vn_node = fgGetNode("/velocity/vn-ms", true);
    filter_ve_node = fgGetNode("/velocity/ve-ms", true);
    filter_vd_node = fgGetNode("/velocity/vd-ms", true);
    filter_status_node = fgGetNode("/health/navigation", true);

    // initialize actuator property nodes
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);

    // initialize pilot property nodes
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = fgGetNode("/sensors/pilot/manual", true);

    // initialize health/status property nodes
    console_seq_num = fgGetNode("/status/console-link-sequence-num", true);
    target_waypoint = fgGetNode( "/autopilot/route-mgr/target-waypoint-idx",
				 true );
    system_load_avg = fgGetNode("/status/system-load-avg", true);
}


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

    file = new_dir; file.append( "ap.dat.gz" );
    if ( (fap = gzopen( file.c_str(), "w+b" )) == NULL ) {
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

    return true;
}


void log_gps( uint8_t *buf, int size, int skip_count ) {
    if ( skip_count < 0 ) { skip_count = 0; }
    static uint8_t skip = skip_count;

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
    static uint8_t skip = skip_count;

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
    static uint8_t skip = skip_count;

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
    static uint8_t skip = skip_count;

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
    static uint8_t skip = skip_count;

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
    static uint8_t skip = skip_count;

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
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    gzwrite( fap, buf, size );
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


// periodic console summary of attitude/location estimate
void display_message()
{
    // double current_time = get_Time();

    if ( !props_inited ) {
	init_props();
    }

    printf("[m/s^2]:ax  = %6.3f ay  = %6.3f az  = %6.3f \n",
	   imu_ax_node->getDoubleValue(),
	   imu_ay_node->getDoubleValue(),
	   imu_az_node->getDoubleValue());
    printf("[deg/s]:p   = %6.3f q   = %6.3f r   = %6.3f \n",
	   imu_p_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES,
	   imu_q_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES,
	   imu_r_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES);
    printf("[Gauss]:hx  = %6.3f hy  = %6.3f hz  = %6.3f \n",
	   imu_hx_node->getDoubleValue(),
	   imu_hy_node->getDoubleValue(),
	   imu_hz_node->getDoubleValue());
    printf("[deg  ]:phi = %6.2f the = %6.2f psi = %6.2f \n",
	   filter_phi_node->getDoubleValue(),
	   filter_theta_node->getDoubleValue(),
	   filter_psi_node->getDoubleValue());
    printf("[     ]:Palt  = %6.3f Pspd  = %6.3f             \n",
	   airdata_altitude_node->getDoubleValue(),
	   airdata_airspeed_node->getDoubleValue());
#if 0
    // gyro bias from mnav filter
    printf("[deg/s]:bp  = %6.3f,bq  = %6.3f,br  = %6.3f \n",
	   xs[4] * SGD_RADIANS_TO_DEGREES,
	   xs[5] * SGD_RADIANS_TO_DEGREES,
	   xs[6] * SGD_RADIANS_TO_DEGREES);
#endif

    if ( GPS_age() < 10.0 ) {
	time_t current_time = gps_unix_sec_node->getIntValue();
	double remainder = gps_unix_sec_node->getDoubleValue() - current_time;
	struct tm *date = gmtime(&current_time);
        printf("[GPS  ]:date = %04d/%02d/%02d %02d:%02d:%05.2f\n",
	       date->tm_year + 1900, date->tm_mon + 1, date->tm_mday,
	       date->tm_hour, date->tm_min, date->tm_sec + remainder);
        printf("[GPS  ]:lon = %f[deg], lat = %f[deg], alt = %f[m], age = %.2f\n",
	       gps_lon_node->getDoubleValue(), gps_lat_node->getDoubleValue(),
	       gps_alt_node->getDoubleValue(), GPS_age());
    } else {
	printf("[GPS  ]:[%0f seconds old]\n", GPS_age());
    }

    if ( strcmp( filter_status_node->getStringValue(), "valid" ) == 0 ) {
        printf("[filter]:lon = %f[deg], lat = %f[deg], alt = %f[m]\n",
	       filter_lon_node->getDoubleValue(),
	       filter_lat_node->getDoubleValue(),
	       filter_alt_node->getDoubleValue());	
    } else {
	printf("[filter]:[No Valid Data]\n");
    }

    printf("[act  ]: %.2f %.2f %.2f %.2f %.2f\n",
	   act_aileron_node->getDoubleValue(),
	   act_elevator_node->getDoubleValue(),
	   act_throttle_node->getDoubleValue(),
	   act_rudder_node->getDoubleValue(),
	   act_channel5_node->getDoubleValue());
    printf("[health]: cmdseq = %d  tgtwp = %d  loadavg = %.2f\n",
           console_seq_num->getIntValue(), target_waypoint->getIntValue(),
           system_load_avg->getFloatValue());
    printf("\n");

    // printf("imu size = %d\n", sizeof( struct imu ) );
}


bool debug_log( const char *hdr, const char *msg ) {
    static FILE *log = NULL;

    if ( log == NULL ) {
	log = fopen( "debug.txt", "a" );
    }
    if ( log == NULL ) {
	return false;
    }

    fprintf( log, "%.3f %s %s\n", get_Time(), hdr, msg );
    fflush( log );

    return true;
}


bool logging_navstate_init() {
    printf("Opening navstate file 'ugear.navstate' in current directory\n");

    fnavstate = fopen("ugear.navstate", "w");

    if ( fnavstate == NULL ) {
	printf("Error opening ugear.navstate\n");
	return false;
    }

    return true;
}


void logging_navstate()
{
    // return;

    if ( !props_inited ) {
	init_props();
    }

    if ( fnavstate == NULL ) {
	return;
    }

    double pretty_yaw
	= filter_psi_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
    if ( pretty_yaw < 0 ) {
        pretty_yaw += 2 * 3.14159265358979323846;
    }

    //struct timeval tv;
    //gettimeofday( &tv, NULL );
    //double unixSec = tv.tv_sec + (tv.tv_usec / 1000000.0);
    fprintf( fnavstate,
             "%.3f %.12f %.12f %.3f %.4f %.4f %.4f %.4f %.4f %.4f\n",
             imu_timestamp_node->getDoubleValue() /* unixSec */,
	     filter_lat_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
	     filter_lon_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
	     -filter_alt_node->getDoubleValue(),
	     filter_vn_node->getDoubleValue(),
	     filter_ve_node->getDoubleValue(),
	     filter_vd_node->getDoubleValue(),
	     pretty_yaw,
	     filter_theta_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
	     filter_phi_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS
	     );
}


void logging_navstate_close() {
    fclose( fnavstate );
}


