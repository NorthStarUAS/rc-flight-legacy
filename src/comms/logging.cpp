#include <sys/types.h>		// opendir() mkdir()
#include <dirent.h>		// opendir()
#include <stdio.h>		// sscanf()
#include <string.h>		// strncmp()
#include <sys/stat.h>		// mkdir()
#include <zlib.h>

#include "adns/mnav/ahrs.h"
#include "adns/mnav/nav.h"
#include "props/props.hxx"
#include "sensors/gps_mgr.h"
#include "util/timing.h"

#include "logging.h"

// global variables for data file logging

static gzFile fimu = NULL;
static gzFile fgps = NULL;
static gzFile fnav = NULL;
static gzFile fservo = NULL;
static gzFile fhealth = NULL;

bool log_to_file = false;       // log to file is enabled/disabled
SGPath log_path;                // base log path
bool display_on = false;        // dump summary to display periodically

// imu property nodes
static SGPropertyNode *p_node = NULL;
static SGPropertyNode *q_node = NULL;
static SGPropertyNode *r_node = NULL;
static SGPropertyNode *ax_node = NULL;
static SGPropertyNode *ay_node = NULL;
static SGPropertyNode *az_node = NULL;
static SGPropertyNode *hx_node = NULL;
static SGPropertyNode *hy_node = NULL;
static SGPropertyNode *hz_node = NULL;

// air data nodes
static SGPropertyNode *Ps_node = NULL;
static SGPropertyNode *Pt_node = NULL;

// gps property nodes
static SGPropertyNode *gps_time_stamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_track_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

// imu output nodes
static SGPropertyNode *theta_node = NULL;
static SGPropertyNode *phi_node = NULL;
static SGPropertyNode *psi_node = NULL;

// "nav" property nodes
static SGPropertyNode *nav_status_node = NULL;
static SGPropertyNode *nav_lat_node = NULL;
static SGPropertyNode *nav_lon_node = NULL;
static SGPropertyNode *nav_alt_node = NULL;


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
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "gps.dat.gz" );
    if ( (fgps = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "nav.dat.gz" );
    if ( (fnav = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "servo.dat.gz" );
    if ( (fservo = gzopen( file.c_str(),"w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "health.dat.gz" );
    if ( (fhealth = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    return true;
}


bool logging_close() {
    // close files

    gzclose(fimu);
    gzclose(fgps);
    gzclose(fnav);
    gzclose(fservo);
    gzclose(fhealth);

    return true;
}


void log_gps( struct gps *gpspacket ) {
    gzwrite( fgps, gpspacket, sizeof(struct gps) );
}


void log_imu( struct imu *imupacket ) {
    gzwrite( fimu, imupacket, sizeof(struct imu) );
}


void log_nav( struct nav *navpacket ) {
    gzwrite( fnav, navpacket, sizeof(struct nav) );
}


void log_servo( struct servo *servopacket ) {
    gzwrite( fservo, servopacket, sizeof(struct servo) );
}


void log_health( struct health *healthpacket ) {
    gzwrite( fhealth, healthpacket, sizeof(struct health) );
}


void flush_gps() {
    // printf("flush gps\n");
    gzflush( fgps, Z_SYNC_FLUSH );
}


void flush_imu() {
    // printf("flush imu\n");
    gzflush( fimu, Z_SYNC_FLUSH );
}


void flush_nav() {
    // printf("flush nav\n");
    gzflush( fnav, Z_SYNC_FLUSH );
}


void flush_servo() {
    // printf("flush servo\n");
    gzflush( fservo, Z_SYNC_FLUSH );
}


void flush_health() {
    // printf("flush health\n");
    gzflush( fhealth, Z_SYNC_FLUSH );
}


// periodic console summary of attitude/location estimate
void display_message( struct servo *sdata, struct health *hdata )
{
    static bool props_inited = false;
    if ( !props_inited ) {
	props_inited = true;

	// initialize imu property nodes
	p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
	q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
	r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
	ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
	ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
	az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
	hx_node = fgGetNode("/sensors/imu/hx", true);
	hy_node = fgGetNode("/sensors/imu/hy", true);
	hz_node = fgGetNode("/sensors/imu/hz", true);

	// initialize air data nodes
	Ps_node = fgGetNode("/sensors/air-data/Ps-m", true);
	Pt_node = fgGetNode("/sensors/air-data/Pt-ms", true);

	// initialize gps property nodes
	gps_time_stamp_node = fgGetNode("/sensors/gps/time-stamp", true);
	gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
	gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
	gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
	gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
	gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
	gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
	gps_track_node = fgGetNode("/sensors/gps/groundtrack-deg", true);
	gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);

	// initialize ahrs property nodes 
	theta_node = fgGetNode("/orientation/pitch-deg", true);
	phi_node = fgGetNode("/orientation/roll-deg", true);
	psi_node = fgGetNode("/orientation/heading-deg", true);

	// initialize nav property nodes
	nav_status_node = fgGetNode("/health/navigation", true);
	nav_lat_node = fgGetNode("/position/latitude-deg", true);
	nav_lon_node = fgGetNode("/position/longitude-deg", true);
	nav_alt_node = fgGetNode("/position/altitude-m", true);
    }

    // double current_time = get_Time();

    printf("[m/s^2]:ax  = %6.3f ay  = %6.3f az  = %6.3f \n",
	   ax_node->getDoubleValue(),
	   ay_node->getDoubleValue(),
	   az_node->getDoubleValue());
    printf("[deg/s]:p   = %6.3f q   = %6.3f r   = %6.3f \n",
	   p_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES,
	   q_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES,
	   r_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES);
    printf("[Gauss]:hx  = %6.3f hy  = %6.3f hz  = %6.3f \n",
	   hx_node->getDoubleValue(),
	   hy_node->getDoubleValue(),
	   hz_node->getDoubleValue());
    printf("[deg  ]:phi = %6.2f the = %6.2f psi = %6.2f \n",
	   phi_node->getDoubleValue(),
	   theta_node->getDoubleValue(),
	   psi_node->getDoubleValue());
    printf("[     ]:Ps  = %6.3f Pt  = %6.3f             \n",
	   Ps_node->getDoubleValue(),
	   Pt_node->getDoubleValue());
    printf("[deg/s]:bp  = %6.3f,bq  = %6.3f,br  = %6.3f \n",
	   xs[4] * SGD_RADIANS_TO_DEGREES,
	   xs[5] * SGD_RADIANS_TO_DEGREES,
	   xs[6] * SGD_RADIANS_TO_DEGREES);

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
	printf("[GPS  ]:[No Recent Data]\n");
    }

    if ( strcmp( nav_status_node->getStringValue(), "valid" ) == 0 ) {
        printf("[nav  ]:lon = %f[deg], lat = %f[deg], alt = %f[m]\n",
	       nav_lon_node->getDoubleValue(),
	       nav_lat_node->getDoubleValue(),
	       nav_alt_node->getDoubleValue());	
    } else {
	printf("[nav  ]:[No Valid Data]\n");
    }

    printf("[Servo]: %d %d %d %d %d %d\n", sdata->chn[0], sdata->chn[1],
           sdata->chn[2], sdata->chn[3], sdata->chn[4], sdata->chn[5]);
    printf("[health]: cmdseq = %d  tgtwp = %d  loadavg = %.2f\n",
           (int)hdata->command_sequence, (int)hdata->target_waypoint,
           (float)hdata->loadavg / 100.0);
    printf("\n");

    // printf("imu size = %d\n", sizeof( struct imu ) );
}
