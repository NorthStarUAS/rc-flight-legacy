//
// FILE: ugfile.cpp
// DESCRIPTION: aquire saved sensor data from a set of files (rather
// than from a live sensor)
//

#include <stdio.h>
#include <string>
#include <string.h>

#include "props/props.hxx"

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

    int result = fscanf( gpsfile,"%lf %lf %lf %lf %lf %lf %lf\n",
			 &gps_data.time,
			 &gps_data.lat, &gps_data.lon, &gps_data.alt,
			 &gps_data.vn, &gps_data.ve, &gps_data.vd
			 /* , &gps[7], &gps[8], &gps[9] */
			 );

    if ( result != 7 ) {
	return false;
    }

    gpscount++;

    return true;
}


// function prototypes
bool ugfile_init() {
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
	// copy fields individually so we don't overwrite phi, the, psi
	data->time = imu_data.time;
	data->p = imu_data.p;
	data->q = imu_data.q;
	data->r = imu_data.r;
	data->ax = imu_data.ax;
	data->ay = imu_data.ay;
	data->az = imu_data.az;
	data->hx = imu_data.hx;
	data->hy = imu_data.hy;
	data->hz = imu_data.hz;
	data->Ps = imu_data.Ps;
	data->Pt = imu_data.Pt;
	data->status = imu_data.status;
    }

    return imu_data_valid;
}


bool ugfile_get_gps( struct gps *data ) {
    if ( gps_data_valid ) {
	memcpy( data, &gps_data, sizeof(struct gps) );
    }

    return gps_data_valid;
}
