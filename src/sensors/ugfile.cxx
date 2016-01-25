//
// FILE: ugfile.cxx
// DESCRIPTION: aquire saved sensor data from a set of files (rather
// than from a live sensor)
//

#include "python/pyprops.hxx"

#include <stdio.h>
#include <string>
#include <string.h>

#include "include/globaldefs.h"
#include "util/timing.h"

#include "gps_mgr.hxx"

#include "ugfile.hxx"


typedef struct
{
    double pos[3];
    double vel[3];
    double eul[3];

    double gyrobias[3];
    double accelbias[3];
} NavState;


struct gps {
    double time;
    double lat,lon,alt;          /* gps position                */
    double  vn,ve,vd;             /* gps velocity                */
    double date;                 /* unix seconds from gps       */
    int8_t status;		 /* data status flag            */
};

struct imu {
   double time;
   double p,q,r;                /* angular velocities    */
   double ax,ay,az;             /* acceleration          */
   double hx,hy,hz;             /* magnetic field        */
   double Ps,Pt;                /* static/pitot pressure */
   // double Tx,Ty,Tz;          /* temperature           */
   double phi,the,psi;          /* attitudes             */
   uint64_t status;             /* error type            */
};

	
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

// property nodes
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;


static bool read_imu() {
    static bool first_time = true;
    static bool next_valid = false;

    if ( feof(imufile) ) {
	return false;
    }

    if ( !first_time ) {
#ifndef BATCH_MODE
	if ( get_Time() - real_time_offset < next_imu_data.time ) {
	    return false;
	}
#endif

	if ( next_valid ) {
	    memcpy( &imu_data, &next_imu_data, sizeof(struct imu) );
	}
    }

#ifndef BATCH_MODE
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
#ifndef BATCH_MODE
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
#ifndef BATCH_MODE
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
#ifndef BATCH_MODE
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
#ifndef BATCH_MODE
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
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("name") ) {
	file_base_name = config->getString("name");
    }
}


/// initialize imu output property nodes 
static void bind_imu_output( string output_path ) {
    imu_node = pyGetNode(output_path, true);
}


// initialize gps output property nodes 
static void bind_gps_output( string output_path ) {
    gps_node = pyGetNode(output_path, true);
}


// function prototypes
bool ugfile_imu_init( string output_path, pyPropertyNode *config ) {
    static bool inited = false;

    if ( inited ) {
	// init has already been run
	return true;
    } else {
	inited = true;
    }

    bind_input( config );
    bind_imu_output( output_path );

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


bool ugfile_gps_init( string output_path, pyPropertyNode *config ) {
    static bool inited = false;

    if ( inited ) {
	// init has already been run
	return true;
    } else {
	inited = true;
    }

    bind_gps_output( output_path );

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
	imu_node.setDouble( "timestamp", imu_data.time );
	imu_node.setDouble( "p_rad_sec", imu_data.p );
	imu_node.setDouble( "q_rad_sec", imu_data.q );
	imu_node.setDouble( "r_rad_sec", imu_data.r );
	imu_node.setDouble( "ax_mps_sec", imu_data.ax );
	imu_node.setDouble( "ay_mps_sec", imu_data.ay );
	imu_node.setDouble( "az_mps_sec", imu_data.az );
	imu_node.setDouble( "hx", imu_data.hx );
	imu_node.setDouble( "hy", imu_data.hy );
	imu_node.setDouble( "hz", imu_data.hz );
    }

    return imu_data_valid;
}


bool ugfile_get_gps() {
    if ( gps_data_valid ) {
	gps_node.setDouble( "timestamp", gps_data.time );
	gps_node.setDouble( "longitude_deg", gps_data.lat );
	gps_node.setDouble( "latitude_deg", gps_data.lon );
	gps_node.setDouble( "altitude_m", gps_data.alt );
	gps_node.setDouble( "vn_ms", gps_data.vn );
	gps_node.setDouble( "ve_ms", gps_data.ve );
	gps_node.setDouble( "vd_ms", gps_data.vd );
	gps_node.setDouble( "unix_time_sec", gps_data.date );
    }

    return gps_data_valid;
}
