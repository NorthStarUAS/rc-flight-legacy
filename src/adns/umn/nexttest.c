#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <limits.h>
#include <string.h>

#include "include/globaldefs.h"

#include "worldcoord.h"
#include "earth.h"
#include "eulerdcm.h"
#include "kalmanfilter.h"
#include "matrix.h"
#include "strapdown.h"

#include "adns.h"


static  int write_output = 1;

FILE *imufile = NULL;
FILE *istatefile = NULL;
FILE *gpsfile = NULL;

int imucount = 0;
int gpscount = 0;
int writecount = 0;

FILE *result_file = NULL;
FILE *filter_file = NULL;
FILE *cov_file = NULL;


int /*bool*/ read_imu( double *imu )
{
    int rv;

    if( feof(imufile) ) {
	return 0 /* false */;
    }

    rv = fscanf( imufile,"%lf %lf %lf %lf %lf %lf %lf\n",
		 &imu[0],
		 &imu[1], &imu[2], &imu[3],
		 &imu[4], &imu[5], &imu[6] );

    if( rv != 7 ) {
	return 0 /* false */;
    }

    imucount++;

    return 1 /* true */;
}


int /*bool*/ read_gps( double *gps )
{
    int rv;

    if( feof(gpsfile) ) {
	return 0 /* false */;
    }

    rv = fscanf( gpsfile,"%lf %lf %lf %lf %lf %lf %lf\n",
		 &gps[0],
		 &gps[1], &gps[2], &gps[3],
		 &gps[4], &gps[5], &gps[6]
		 /* , &gps[7], &gps[8], &gps[9] */
		 );

    if( rv != 7 ) {
	return 0 /* false */;
    }

    gpscount++;

    return 1 /* true */;
}


void write_state( double t, NavState *state )
{
    double pos_ned[3];

    writecount++;

    lla2ned( state->pos, state->pos, pos_ned );
    fprintf( result_file,
	     "%f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f",
	     t,
	     state->pos[0], state->pos[1], state->pos[2],
	     state->vel[0], state->vel[1], state->vel[2],
	     state->eul[0], state->eul[1], state->eul[2] );

    fprintf( result_file,
	     " %.15e %.15e %.15e %.15e %.15e %.15e\n",
	     state->gyrobias[0],  state->gyrobias[1],  state->gyrobias[2],
	     state->accelbias[0], state->accelbias[1], state->accelbias[2] );
}


void write_filter( KalmanFilter *kf )
{
    int i;

    /* write state */
    for ( i = 0; i < kf->state->nrows; i++ ) {
	fprintf(filter_file,"%.15e", kf->state->vector[i]);
	if( i < kf->state->nrows -1 ) {
	    fprintf(filter_file," ");
	}
    }

    for ( i = 0; i < kf->Inn->nrows; i++ ) {
	fprintf(filter_file," %.15e", kf->Inn->vector[i]);
    }

    fprintf(filter_file,"\n");
}


void write_cov( KalmanFilter *kf )
{
    int i;
    for ( i = 0; i < kf->P->nrows; i++ ) {
	fprintf(cov_file,"%.15e", kf->P->matrix[i][i]);
	if( i < kf->P->nrows -1 ) {
	    fprintf(cov_file," ");
	}
    }	
    fprintf(cov_file,"\n");
}


int main( int argc, char *argv[] )
{
    double imu[7];
    /* imu = [time ,angle_rates, accelerations]
     * angle_rates = [rrx rry rrz] radians/sec (aka p, q, r)
     * accerations = [ax ay az] meters/sec^2
     */ 

    double gps[7], gpsnext[7];
    /* gps=[t_gps,gps_pos_lla,gps_vel_ned]
     * gps_pos = [lat lon -elev] radians, meters
     * gps_vel_ned = [veln vele veld] meters/second
     */
    
    int rv;

    /* state.pos = [latitude longitude height] rad rad meter
     * state.vel = [veln vele veld] meters/second
     * state.eul = [yaw pitch roll] radians
     * state.gyrobias = radians/sec
     * state.accelbias=meter/sec^2
     */

    /*	System *system = NULL; */

    char file[PATH_MAX];	

    if( argc != 2 ) {
	printf("useage: test_open_loop datafile/base/path\n");
	exit(EXIT_FAILURE);
    }

    /* create output blank file*/
    sprintf(file,"%s.navstate", argv[1]);
    result_file = fopen(file,"w");
    if( result_file == NULL ) {
	printf("unable to open results file\n");
	exit(EXIT_FAILURE);
    }

    sprintf(file,"%s.filter", argv[1]);
    filter_file = fopen(file,"w");
    if( filter_file == NULL ) {
	printf("unable to open filter file\n");
	exit(EXIT_FAILURE);
    }

    sprintf(file,"%s.cov", argv[1]);
    cov_file = fopen(file,"w");
    if( cov_file == NULL ) {
	printf("unable to open cov file\n");
	exit(EXIT_FAILURE);
    }

    /* initial state */
    NavState istate;
    memset( &istate, 0, sizeof(istate) );
    sprintf(file,"%s.istate", argv[1] );
    istatefile = fopen(file,"r");
    if( istatefile == NULL ) {
	printf("unable to open initial state file\n");
	exit(EXIT_FAILURE);
    }

    rv = fscanf( istatefile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
		 &istate.pos[0], &istate.pos[1], &istate.pos[2],
		 &istate.vel[0], &istate.vel[1], &istate.vel[2],
		 &istate.eul[0], &istate.eul[1], &istate.eul[2] );
    if( rv != 9 ) {
	printf("unable to read all initial states\n");
	exit(EXIT_FAILURE);
    }
    fclose( istatefile );
    umn_adns_set_initial_state( &istate );

    /* Read imu input file*/
    sprintf(file,"%s.imu", argv[1]);
    imufile = fopen(file,"r");
    if( imufile == NULL ) {
	printf("unable to open imu file\n");
	exit(EXIT_FAILURE);
    }

    /* Read gps input data*/
    sprintf(file,"%s.gps", argv[1]);
    gpsfile = fopen(file,"r");
    if( gpsfile == NULL ) {
	printf("unable to open gps file\n");
	exit(EXIT_FAILURE);
    }

    umn_adns_init();

    read_imu(imu);  //imu(0)
    // read_imu(imu);
    read_gps(gpsnext);
    memcpy( gps, gpsnext, sizeof(gps) );

    NavState *state;
    KalmanFilter *kf;

    if ( write_output ) {
	state = umn_adns_get_state();
	write_state(imu[0],state); 
	kf = umn_adns_get_kf();
	write_cov(kf);
    }

    while ( read_imu(imu) ) {
	int fresh_gps = 0;

	if ( imu[0] >= gpsnext[0] ) {
	    // this logic is to avoid using the next gps record too
	    // soon when reading from a file.
	    memcpy( gps, gpsnext, sizeof(gps) );
	    fresh_gps = 1;
	    read_gps(gpsnext);
	    // umn_adns_print_gps(gpsnext);
	}

	umn_adns_update( imu, gps );

	if ( fresh_gps ) {
	    if ( write_output ) {
		write_filter(kf);
	    }
	    // umn_adns_print_filter(kf);
	}

	// umn_adns_print_cov(kf);
	// umn_adns_print_state(&state);
 	if ( write_output ) {
	    kf = umn_adns_get_kf();
	    write_cov(kf);
	    state = umn_adns_get_state();
	    write_state(imu[0],state);
	}
    }

    //	system_free(&s);

    fclose( imufile );
    fclose( gpsfile );
    fclose( result_file );
    fclose( filter_file );
    fclose( cov_file);

    return EXIT_SUCCESS;
}
