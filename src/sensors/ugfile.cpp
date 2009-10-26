//
// FILE: ugfile.cpp
// DESCRIPTION: aquire saved sensor data from a set of files (rather
// than from a live sensor)
//

#include <stdio.h>
#include <string>
#include <string.h>

#include "adns/mnav/ahrs.h"
#include "props/props.hxx"
#include "util/timing.h"

#include "gps_mgr.h"

#include "ugfile.h"


typedef struct
{
    double pos[3];
    double vel[3];
    double eul[3];

    double gyrobias[3];
    double accelbias[3];
} NavState;

	
static FILE *imufile = NULL;
static FILE *istatefile = NULL;
static FILE *gpsfile = NULL;

static int imucount = 0;
static int gpscount = 0;

static struct imu imu_data, next_imu_data;
static struct gps gps_data, next_gps_data;
static bool imu_data_valid = false;
static bool gps_data_valid = false;

string file_base_name = "";

static double real_time_offset = 0.0;

// fixme: this should move over to the UMN INS/GNS module
static NavState state;

// ugfile property nodes
static SGPropertyNode *configroot = NULL;
static SGPropertyNode *outputroot = NULL;

static SGPropertyNode *file_base_node = NULL;

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

static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;


static bool read_imu() {
    static bool first_time = true;
    static bool next_valid = false;

    if( feof(imufile) ) {
	return false;
    }

// #define FORCE_REAL_TIME
    if ( !first_time ) {
#ifdef FORCE_REAL_TIME
	if ( get_Time() - real_time_offset < next_imu_data.time ) {
	    return false;
	}
#endif

	if ( next_valid ) {
	    memcpy( &imu_data, &next_imu_data, sizeof(struct imu) );
	}
    }

#ifdef FORCE_REAL_TIME
    do {
#endif
	int result = fscanf( imufile,
			     "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			     &next_imu_data.time,
			     &next_imu_data.p,
			     &next_imu_data.q,
			     &next_imu_data.r,
			     &next_imu_data.ax,
			     &next_imu_data.ay,
			     &next_imu_data.az,
			     &next_imu_data.hx,
			     &next_imu_data.hy,
			     &next_imu_data.hz );

	if ( result != 10 ) {
	    next_valid = false;
	} else {
	    next_valid = true;
	    imucount++;
	}
#ifdef FORCE_REAL_TIME
    } while ( get_Time() - real_time_offset > next_imu_data.time );
#endif

    if ( first_time && next_valid ) {
	first_time = false;
	memcpy( &imu_data, &next_imu_data, sizeof(struct imu) );
    }

    /* printf("read timestamp = %.3f\n", next_imu_data.time); */

    return true;
}


static bool read_gps() {
    static bool first_time = true;
    static bool next_valid = false;

    if( feof(gpsfile) ) {
	return false;
    }

    if ( !first_time ) {
#ifdef FORCE_REAL_TIME
 	if ( get_Time() - real_time_offset < next_gps_data.time ) {
	    return false;
	}
#else
	if ( next_imu_data.time < next_gps_data.time ) {
	    return false;
	}
#endif

	if ( next_valid ) {
	    memcpy( &gps_data, &next_gps_data, sizeof(struct gps) );
	}
    }

    double lat_rad, lon_rad, alt_neg;
#ifdef FORCE_REAL_TIME
    do {
#endif
	int result = fscanf( gpsfile,"%lf %lf %lf %lf %lf %lf %lf\n",
			     &next_gps_data.time,
			     &lat_rad, &lon_rad, &alt_neg,
			     &next_gps_data.vn,
			     &next_gps_data.ve,
			     &next_gps_data.vd
			     /* , &gps[7], &gps[8], &gps[9] */
			     );

	if ( result != 7 ) {
	    next_valid = false;
	} else {
	    next_valid = true;
	    gpscount++;
	}
#ifdef FORCE_REAL_TIME
    } while ( get_Time() - real_time_offset > next_gps_data.time );
#endif

    next_gps_data.lat = lat_rad * SGD_RADIANS_TO_DEGREES;
    next_gps_data.lon = lon_rad * SGD_RADIANS_TO_DEGREES;
    next_gps_data.alt = -alt_neg;
    next_gps_data.status = ValidData;

    if ( first_time && next_valid ) {
	first_time = false;
	memcpy( &gps_data, &next_gps_data, sizeof(struct gps) );
    }

    // printf("read gps = %.3f %.8f %.8f %.8f\n", gps_data.time, gps_data.lat,
    // 	   gps_data.lon, gps_data.alt);

    return true;
}


// initialize gpsd input property nodes
static void bind_input( SGPropertyNode *config ) {
    file_base_node = config->getChild("name");
    if ( file_base_node != NULL ) {
	file_base_name = file_base_node->getStringValue();
    }
    configroot = config;
}


/// initialize imu output property nodes 
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


// initialize gps output property nodes 
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


// function prototypes
bool ugfile_imu_init( string rootname, SGPropertyNode *config ) {
    static bool inited = false;

    if ( inited ) {
	// init has already been run
	return true;
    } else {
	inited = true;
    }

    bind_input( config );
    bind_imu_output( rootname );

    string file_name = "";

    /* open initial state file */
    file_name = file_base_name + ".istate";
    istatefile = fopen( file_name.c_str(), "r" );
    if ( istatefile == NULL ) {
	printf( "ugfile_init(): unable to open initial state file = %s\n",
		file_name.c_str() );
	return false;
    }

    int result  = fscanf( istatefile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			  &state.pos[0], &state.pos[1], &state.pos[2],
			  &state.vel[0], &state.vel[1], &state.vel[2],
			  &state.eul[0], &state.eul[1], &state.eul[2] );
    if( result != 9 ) {
	printf("ugfile_init(): unable to read all initial states\n");
	return false;
    }
    fclose( istatefile );

    /* open imu input file */
    file_name = file_base_name + ".imu";
    imufile = fopen( file_name.c_str(), "r" );
    if ( imufile == NULL ) {
	printf( "ugfile_init(): unable to open imu file = %s\n",
		file_name.c_str() );
	return false;
    }
    // read the first imu record
    next_imu_data.time = 0.0;
    if ( !read_imu() ) {
	return false;
    }

    // set the real time clock versus data file time stamp offset
    real_time_offset = get_Time() - imu_data.time;
    printf("real time offset = %.2f\n", real_time_offset);

    return true;
}


bool ugfile_gps_init( string rootname, SGPropertyNode *config ) {
    static bool inited = false;

    if ( inited ) {
	// init has already been run
	return true;
    } else {
	inited = true;
    }

    bind_gps_output( rootname );

    /* open gps input file */
    string file_name = file_base_name + ".gps";
    gpsfile = fopen( file_name.c_str(), "r" );
    if ( gpsfile == NULL ) {
	printf("ugfile_init(): unable to open gps file = %s\n",
	       file_name.c_str() );
	return false;
    }
    // read the first gps record
    next_gps_data.time = 0.0;
    if ( !read_gps() ) {
	return false;
    }

    printf("ugfile gps init() called\n");

    return true;
}


bool ugfile_read() {
    imu_data_valid = false;
    gps_data_valid = false;

    imu_data_valid = read_imu();
    if ( !imu_data_valid ) {
	return false;
    }

    gps_data_valid = read_gps();
    if ( !gps_data_valid ) {
	return false;
    }

    return true;
}


void ugfile_close() {
    fclose( imufile );
    fclose( gpsfile );
}


bool ugfile_get_imu() {
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


bool ugfile_get_gps() {
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
