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

static struct imu imu_data;
static struct gps gps_data;
static bool imu_data_valid = false;
static bool gps_data_valid = false;

// fixme: this should move over to the UMN INS/GNS module
static NavState state;

static SGPropertyNode *outputroot = NULL;

// output property nodes
static SGPropertyNode *timestamp_node = NULL;
static SGPropertyNode *p_node = NULL;
static SGPropertyNode *q_node = NULL;
static SGPropertyNode *r_node = NULL;
static SGPropertyNode *ax_node = NULL;
static SGPropertyNode *ay_node = NULL;
static SGPropertyNode *az_node = NULL;
static SGPropertyNode *hx_node = NULL;
static SGPropertyNode *hy_node = NULL;
static SGPropertyNode *hz_node = NULL;


static bool read_imu() {
    if( feof(imufile) ) {
	return false;
    }

    int result = fscanf( imufile,"%lf %lf %lf %lf %lf %lf %lf\n",
			 &imu_data.time,
			 &imu_data.p, &imu_data.q, &imu_data.r,
			 &imu_data.ax, &imu_data.ay, &imu_data.az );

    // fixme: we should support magnetometer data here
    if ( result != 7 ) {
	return false;
    }

    imucount++;

    return true;
}


static bool read_gps() {
    if( feof(gpsfile) ) {
	return false;
    }

    double lat_rad, lon_rad, alt_neg;
    int result = fscanf( gpsfile,"%lf %lf %lf %lf %lf %lf %lf\n",
			 &gps_data.time,
			 &lat_rad, &lon_rad, &alt_neg,
			 &gps_data.vn, &gps_data.ve, &gps_data.vd
			 /* , &gps[7], &gps[8], &gps[9] */
			 );

    if ( result != 7 ) {
	return false;
    }

    gps_data.lat = lat_rad * SGD_RADIANS_TO_DEGREES;
    gps_data.lon = lon_rad * SGD_RADIANS_TO_DEGREES;
    gps_data.alt = -alt_neg;
    gps_data.status = ValidData;
    gpscount++;

    // printf("read gps = %.3f %.8f %.8f %.8f\n", gps_data.time, gps_data.lat,
    //	   gps_data.lon, gps_data.alt);

    return true;
}


// function prototypes
bool ugfile_init( string rootname ) {
    outputroot = fgGetNode( rootname.c_str(), true );

    timestamp_node = outputroot->getChild("timestamp", 0, true);
    p_node = outputroot->getChild("p-rad_sec", 0, true);
    q_node = outputroot->getChild("q-rad_sec", 0, true);
    r_node = outputroot->getChild("r-rad_sec", 0, true);
    ax_node = outputroot->getChild("ax-mps_sec", 0, true);
    ay_node = outputroot->getChild("ay-mps_sec", 0, true);
    az_node = outputroot->getChild("az-mps_sec", 0, true);
    hx_node = outputroot->getChild("hx", 0, true);
    hy_node = outputroot->getChild("hy", 0, true);
    hz_node = outputroot->getChild("hz", 0, true);

    SGPropertyNode *file_base_node
	= fgGetNode("/config/sensors/file/base", true);
    string file_name;
    string base_name = file_base_node->getStringValue();

    /* open initial state file */
    file_name = base_name + ".istate";
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
    file_name = base_name + ".imu";
    imufile = fopen( file_name.c_str(), "r" );
    if ( imufile == NULL ) {
	printf( "ugfile_init(): unable to open imu file = %s\n",
		file_name.c_str() );
	return false;
    }
    // read the first imu record
    if ( !read_imu() ) {
	return false;
    }

    /* open gps input file */
    file_name = base_name + ".gps";
    gpsfile = fopen( file_name.c_str(), "r" );
    if ( gpsfile == NULL ) {
	printf("ugfile_init(): unable to open gps file = %s\n",
	       file_name.c_str() );
	return false;
    }
    // read the first gps record
    if ( !read_gps() ) {
	return false;
    }

    return true;
}

bool ugfile_read() {
    imu_data_valid = false;
    gps_data_valid = false;

    imu_data_valid = read_imu();
    if ( !imu_data_valid ) {
	return false;
    }

    if ( gps_data.time < imu_data.time ) {
	gps_data_valid = read_gps();
    }

    return true;
}


void ugfile_close() {
    fclose( imufile );
    fclose( gpsfile );
}


bool ugfile_get_imu( struct imu *data ) {
    if ( imu_data_valid ) {
	timestamp_node->setDoubleValue( imu_data.time );
	p_node->setDoubleValue( imu_data.p );
	q_node->setDoubleValue( imu_data.q );
	r_node->setDoubleValue( imu_data.r );
	ax_node->setDoubleValue( imu_data.ax );
	ay_node->setDoubleValue( imu_data.ay );
	az_node->setDoubleValue( imu_data.az );
	hx_node->setDoubleValue( imu_data.hx );
	hy_node->setDoubleValue( imu_data.hy );
	hz_node->setDoubleValue( imu_data.hz );
    }

    return imu_data_valid;
}


bool ugfile_get_gps( struct gps *data ) {
    if ( gps_data_valid ) {
	memcpy( data, &gps_data, sizeof(struct gps) );
    }

    return gps_data_valid;
}
