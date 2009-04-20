#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <limits.h>
#include <string.h>

#include "worldcoord.h"
#include "earth.h"
#include "eulerdcm.h"
#include "matrix.h"
#include "strapdown.h"

#include "kalmanfilter.h"


typedef struct
{
    double pos[3];
    double vel[3];
    double eul[3];

    double gyrobias[3];
    double accelbias[3];
} NavState;

	
FILE *imufile = NULL;
FILE *istatefile = NULL;
FILE *gpsfile = NULL;

int imucount = 0;
int gpscount = 0;
int writecount = 0;

double pos_ref_lla[3];
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


void print_imu( double *imu )
{
    printf( "imu %.15e,%.15e,%.15e,%.15e,%.15e,%.15e,%.15e\n",
	    imu[0],
	    imu[1], imu[2], imu[3],
	    imu[4], imu[5], imu[6] );
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

void print_gps( double *gps )
{
    printf( "gps %.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f\n",
	    gps[0],
	    gps[1], gps[2], gps[3],
	    gps[4], gps[5], gps[6],
	    gps[7], gps[8], gps[9] );
}


void write_state( double t, NavState *state )
{
    double pos_ned[3];

    writecount++;

    lla2ned( state->pos, pos_ref_lla, pos_ned );
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


void print_state( NavState *state )
{
    printf( "state %.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f\n",
	    state->pos[0], state->pos[1], state->pos[2],
	    state->vel[0], state->vel[1], state->vel[2],
	    state->eul[0], state->eul[1], state->eul[2] );
}


void print_filter(KalmanFilter *kf)
{
    int i;

    /* print state */
    printf("dXhat: ");
    for(i = 0; i < kf->state->nrows; i++) {
	printf("%.15f", kf->state->vector[i]);
	if( i < kf->state->nrows -1 ) {
	    printf(",");
	}
    }
    printf("\n");

    printf("innovation: ");
    for(i = 0; i < kf->Inn->nrows; i++)	{
	printf(",%.15f", kf->Inn->vector[i]);
    }
    printf("\n");
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


void print_cov( KalmanFilter *kf )
{
    int i;
    printf("Cov:");
    for(i = 0; i <=9; i++) {
	printf("%.15f", kf->P->matrix[i][i]);
	if( i < kf->P->nrows -1 ) {
	    printf(",");
	}
    }	
    printf("\n");
    for(i = 9; i <=14; i++) {
	printf("%.15f", kf->P->matrix[i][i]);
	if( i < kf->P->nrows -1 ) {
	    printf(",");
	}
    }
    printf("\n");	
}


int main( int argc, char *argv[] )
{
    int write_output = 0;

    double imu[7];
    double imunext[7];
    /* imu = [time ,angle_rates, accelerations]
     * angle_rates = [rrx rry rrz] rad/sec
     * accerations = [ax ay az] meter/sec^2
     */ 

    double gps[7];
    //gps=[t_gps,gps_pos_lla,gps_vel_ned]
    double dt = 0;
    int rv;
    double d2r = 3.1415926 / 180.0;

    double COV_GPS_POS = 3;  //m,NED
    double COV_GPS_VEL = 0.2;//m/sec

    double SIGMA_W_F = 0.02;
    double SIGMA_C_F = 0.002;
    double SIGMA_W_G = 0.5*d2r;
    double SIGMA_C_G = 0.1*d2r;
    double TAU_F     = 300.0;
    double TAU_G     = 300.0;

    NavState state;
    NavState *pstate = &state;

    /* state.pos = [latitude longitude height] rad rad meter
     * state.vel = [veln vele veld] meter/second
     * state.eul = [yaw pitch roll] radians
     * state.gyrobias = radians/sec
     * state.accelbias=meter/sec^2
     */

    /*	System *system = NULL; */

    char file[PATH_MAX];	
    int count = 0;

    memset( &state, 0, sizeof(state) );

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
    sprintf(file,"%s.istate", argv[1] );
    istatefile = fopen(file,"r");
    if( istatefile == NULL ) {
	printf("unable to open initial state file\n");
	exit(EXIT_FAILURE);
    }

    rv = fscanf( istatefile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
		 &state.pos[0], &state.pos[1], &state.pos[2],
		 &state.vel[0], &state.vel[1], &state.vel[2],
		 &state.eul[0], &state.eul[1], &state.eul[2] );
    if( rv != 9 ) {
	printf("unable to read all initial states\n");
	exit(EXIT_FAILURE);
    }
    fclose( istatefile );

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

    /**********************************************************/
    /********** intialize a system starts here ****************/
    //	system = system_create();
    //  System *s = NULL;
    //	s = (System*)ivl_malloc(sizeof(System));

    ivl_matrix *F = ivl_matrix_new(15,15);
    ivl_matrix *G = ivl_matrix_new(15,12);
    ivl_matrix *Rw = ivl_matrix_new(12,12);
    ivl_matrix *Cb2n = ivl_matrix_new(3,3);
    // ivl_matrix *Cb2n_rot = ivl_matrix_new(3,3);

    KalmanFilter *kf = kalmanfilter_new(15,6); 
    kf->statefeedback = 0 /* false */;

    /* system_computeH(s); */
    ivl_matrix_zeros(kf->H);
    ivl_matrix_identity(kf->H);
    //     ivl_matrix_scalar_mult(kf->H,-1,NULL);

    /* print H */
    int i;
    printf("H:");
    for(i = 0; i < 6*15; i++) {
	printf("%.15f", kf->H->vector[i]);
	if( i < 6*15 - 1 ) {
	    printf(",");
	}
    }

    /*system_computeRv(s);*/
    double v;
    ivl_matrix_zeros( kf->R );
    v = COV_GPS_POS * COV_GPS_POS;
    kf->R->matrix[0][0] = v;
    kf->R->matrix[1][1] = v;
    kf->R->matrix[2][2] = v;
    v = COV_GPS_VEL * COV_GPS_VEL;
    kf->R->matrix[3][3] = v;
    kf->R->matrix[4][4] = v;
    kf->R->matrix[5][5] = v;

    printf("Rv:");
    for(i = 0; i <kf->m; i++) {
	printf("   %.15f", kf->R->matrix[i][i]);
    }
    printf("\n");

    /*system_computeG(s,NULL); *///15*12        
    /*G =  [  Z3   Z3   Z3  Z3;...*/
    /*        Cb2n Z3   Z3  Z3;...*/
    /*        Z3   -Cb2n Z3  Z3;...*/
    /*        Z3   Z3   I3  Z3;...*/
    /*        Z3   Z3   Z3  I3];*/
    //Part of G will be constructed online
    //G(4:6,1:3) = Cb2n;
    //G(7:9,4:6) = -Cb2n; 

    ivl_matrix_zeros( G );
    G->matrix[9][6]  = 1.0;  G->matrix[10][7]  = 1.0;  G->matrix[11][8] = 1.0;
    G->matrix[12][9] = 1.0;  G->matrix[13][10] = 1.0;  G->matrix[14][11] = 1.0;

    /*system_computeRw(s);*/ //12*12
    double w;
    ivl_matrix_zeros(Rw);

    w = SIGMA_W_F * SIGMA_W_F;//accel white
    Rw->matrix[0][0] = w;  Rw->matrix[1][1] = w;  Rw->matrix[2][2] = w;

    w = SIGMA_W_G * SIGMA_W_G;//gyro white
    Rw->matrix[3][3] = w;  Rw->matrix[4][4] = w;  Rw->matrix[5][5] = w;

    w = 2.0 * SIGMA_C_F * SIGMA_C_F / TAU_F;// accel markov
    Rw->matrix[6][6] = w;  Rw->matrix[7][7] = w;  Rw->matrix[8][8] = w;

    w = 2.0 * SIGMA_C_G * SIGMA_C_G / TAU_G;// gyro  markov
    Rw->matrix[9][9] = w;  Rw->matrix[10][10] = w;  Rw->matrix[11][11] = w;

    /*system_computeIntialP(s);*/ //15*15

    double sdp = COV_GPS_POS ;	 // Position Errors (NED Coordinates) 
    double sdv = COV_GPS_VEL;	 // Velocity Errors (NED Coordinates) 

#define IVL_PI    3.1415926535897932384626433832795028841971693993751
#define IVL_DEG2RAD(d)  ((d) * (IVL_PI/180.0))

    double sde = IVL_DEG2RAD(1);	 // Platform Tilt/Attitude Errors 
    double sdf = SIGMA_W_F;	 // Accelerometer Bias Estimation Erros
    double sdw = SIGMA_W_G;	 // Rate Gyro Bias Estimation Errors 

    kf->P->matrix[0][0] = sdp*sdp;  kf->P->matrix[1][1] = sdp*sdp;  kf->P->matrix[2][2] = sdp*sdp;

    kf->P->matrix[3][3] = sdv*sdv;  kf->P->matrix[4][4] = sdv*sdv;  kf->P->matrix[5][5] = sdv*sdv;

    kf->P->matrix[6][6] = sde*sde;  kf->P->matrix[7][7] = sde*sde;  kf->P->matrix[8][8] = sde*sde;

    kf->P->matrix[9][9] = sdf*sdf;  kf->P->matrix[10][10] = sdf*sdf;  kf->P->matrix[11][11] = sdf*sdf;

    kf->P->matrix[12][12] = sdw*sdw;  kf->P->matrix[13][13] = sdw*sdw;  kf->P->matrix[14][14] = sdw*sdw;

    /******************** intialize a system  ends ************************/

    read_imu(imu);  //imu(0)
    read_gps(gps);
    read_gps(gps);  //gps(1)

    if ( write_output ) {
	write_state(imu[0],&state); 
	write_cov(kf);
    }

    while( read_imu(imunext) ) {
	// print_imu(imunext);

	if ( ++count % 500 == 0 ) {
	    printf("count = %d\n", count++);
	}

	dt = imunext[0] - imu[0];

	/******************* INS update *************************/


	/***********************************/
	//system_imu_update(system,&state,imu,dt);
	/***********************************/

	//ins_apply_bias(imu,&state);
	imu[1] -= pstate->gyrobias[0];
	imu[2] -= pstate->gyrobias[1];
	imu[3] -= pstate->gyrobias[2];
	imu[4] -= pstate->accelbias[0];
	imu[5] -= pstate->accelbias[1];
	imu[6] -= pstate->accelbias[2];

	strapdown_er( pstate->pos, pstate->vel, pstate->eul, Cb2n,
	              &imu[1], dt );

	/****** Propagate Navigation State Covariance Forward in time ******/	
	//system_computeF(s, pstate->pos,pstate->vel,Cb2n,imu);

	/*
	  dp2dp = -sk(transRate); dv2dp=I3;
	  dp2dv = diag((cgo/rE)*[-1 -1 2]); dv2dv = -sk(earthRate + totalRate);
	  de2dv = sk(Cb2n*[imu(k-1,5:7)-accelBias(k-1,:)]'); 		df2dv = Cb2n ; 
	  de2de = -sk(totalRate);  		dw2de = -Cb2n; 
	  df2df = -I3/tau_f; 
	  dw2dw = -I3/tau_g; 
	  %       dp      dv      de      df      dw
	  %      1-3     4-6     7-9     10-12   13-15
	  F = [  Z3      I3      Z3      Z3      Z3      ;... % 1 - 3   dp
	  dp2dv     Z3    de2dv    df2dv    Z3      ;... % 4 - 6   dv
	  Z3      Z3      Z3      Z3    dw2de     ;... % 7 - 9   de
	  Z3      Z3      Z3     df2df    Z3      ;... % 10 - 12 df
	  Z3      Z3      Z3      Z3    dw2dw  ]  ;    % 13 -15  dw
	*/

	double sk[9];
	double tmp[3];
	tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
	ivl_matrix_zeros(F);
	/*********** rows 0 - 2 *************/
	F->matrix[0][3] = 1.0;
	F->matrix[1][4] = 1.0;
	F->matrix[2][5] = 1.0;

	/*********** rows 3 - 5 *************/
	/* [3:5][0:2] = dp2dv */
	F->matrix[5][2] = 2.0 * EarthG0 / EarthRad;
	/* [3:5][3:5] = Z3 */
	/* [3:5][6:8] = de2dv = sk(Cb2n*accel) */
	ivl_matrix_dgemv( 0, 1.0, Cb2n, &imu[4], 1.0, tmp );
	skew_symmetric_3x3(tmp,sk);
	F->matrix[3][6] = sk[0]; F->matrix[3][7] = sk[1]; F->matrix[3][8] = sk[2];
	F->matrix[4][6] = sk[3]; F->matrix[4][7] = sk[4]; F->matrix[4][8] = sk[5];
	F->matrix[5][6] = sk[6]; F->matrix[5][7] = sk[7]; F->matrix[5][8] = sk[8];
	/* [3:5][9:11] = df2dv = Cb2n */
	F->matrix[3][9]  = Cb2n->matrix[0][0];
	F->matrix[3][10] = Cb2n->matrix[0][1]; 
	F->matrix[3][11] = Cb2n->matrix[0][2];
	F->matrix[4][9]  = Cb2n->matrix[1][0]; 
	F->matrix[4][10] = Cb2n->matrix[1][1]; 
	F->matrix[4][11] = Cb2n->matrix[1][2];
	F->matrix[5][9]  = Cb2n->matrix[2][0]; 
	F->matrix[5][10] = Cb2n->matrix[2][1]; 
	F->matrix[5][11] = Cb2n->matrix[2][2];
	/* [3:5][12:14] = dw2dv = Z3 */

	/*********** rows 6 - 8 *************/
	/* [6:8][12:14] = dw2de = -Cb2n */
	F->matrix[6][12] = -1.0 * Cb2n->matrix[0][0]; 
	F->matrix[6][13] = -1.0 * Cb2n->matrix[0][1]; 
	F->matrix[6][14] = -1.0 * Cb2n->matrix[0][2];
	F->matrix[7][12] = -1.0 * Cb2n->matrix[1][0]; 
	F->matrix[7][13] = -1.0 * Cb2n->matrix[1][1]; 
	F->matrix[7][14] = -1.0 * Cb2n->matrix[1][2];
	F->matrix[8][12] = -1.0 * Cb2n->matrix[2][0]; 
	F->matrix[8][13] = -1.0 * Cb2n->matrix[2][1]; 
	F->matrix[8][14] = -1.0 * Cb2n->matrix[2][2];

	/*********** rows 9 - 11 ************/
	/* [9:11][9-11] =  df2df = -I3/tau_f */
	F->matrix[9][9] = -1.0 / TAU_F;
	F->matrix[10][10] = -1.0 / TAU_F;
	F->matrix[11][11] = -1.0 / TAU_F;

	/*********** rows 12 - 14 ***********/
	F->matrix[12][12] = -1.0 / TAU_G;
	F->matrix[13][13] = -1.0 / TAU_G;
	F->matrix[14][14] = -1.0 / TAU_G;

	//////////////////////////
	//system_computeG(s,Cb2n);
	//////////////////////////
	//15*12
	/*G =  [  Z3   Z3   Z3  Z3;...*/
	/*        Cb2n Z3   Z3  Z3;...*/
	/*        Z3   -Cb2n Z3  Z3;...*/
	/*        Z3   Z3   I3  Z3;...*/
	/*        Z3   Z3   Z3  I3];*/
	//Part of G will be constructed online
	//G(4:6,1:3) = Cb2n;
	//G(7:9,4:6) = -Cb2n; 

	G->matrix[3][0] = Cb2n->matrix[0][0];
	G->matrix[3][1] = Cb2n->matrix[0][1];
	G->matrix[3][2] = Cb2n->matrix[0][2];
	G->matrix[4][0] = Cb2n->matrix[1][0];
	G->matrix[4][1] = Cb2n->matrix[1][1]; 
	G->matrix[4][2] = Cb2n->matrix[1][2];
	G->matrix[5][0] = Cb2n->matrix[2][0]; 
	G->matrix[5][1] = Cb2n->matrix[2][1]; 
	G->matrix[5][2] = Cb2n->matrix[2][2];
	G->matrix[6][3] = -1.0 * Cb2n->matrix[0][0]; 
	G->matrix[6][4] = -1.0 * Cb2n->matrix[0][1]; 
	G->matrix[6][5] = -1.0 * Cb2n->matrix[0][2];
	G->matrix[7][3] = -1.0 * Cb2n->matrix[1][0]; 
	G->matrix[7][4] = -1.0 * Cb2n->matrix[1][1]; 
	G->matrix[7][5] = -1.0 * Cb2n->matrix[1][2];
	G->matrix[8][3] = -1.0 * Cb2n->matrix[2][0]; 
	G->matrix[8][4] = -1.0 * Cb2n->matrix[2][1]; 
	G->matrix[8][5] = -1.0 * Cb2n->matrix[2][2];

	/* kalman filter project */
	//   disrw computes the discrete equivalent of continous noise for the
	//  dynamic system described by
	//    x_dot = Fx + Gw
	//  w is the driving noise.  Ts is the sampling time and Rwpsd
	//is the power spectral density of the driving noise.

	//compute PHI and Q
	kalmanfilter_eval_Qphi(kf,F,G,dt,Rw);
 
	/* \brief Project error corvariance ahead to next time step
	   P = phi P phi' + Q  */
	kalmanfilter_project_cov(kf);
	/**************** INS update ends here *******************/

	/*********** GPS measurement update starts here **************/
	if ( imu[0] <gps[0] && gps[0] <= imunext[0] ) {
	    // print_gps(gps);

	    /*use pos_ref_lla here*/
	    /* NED reference frame*/
	    memcpy(pos_ref_lla,state.pos, sizeof(pos_ref_lla));//zhiqiang 20070425

	    //system_gps_update(s,pstate,gps);
	    //ins_apply_kalmanfilter(pstate,s->kf,gps, &gps[1]);
		
	    double ins_pos_ned[3];
	    double gps_pos_ned[3];

	    lla2ned( pstate->pos, pos_ref_lla, ins_pos_ned);
	    lla2ned( &(gps[1]), pos_ref_lla, gps_pos_ned);

	    /* form innovation */
	    /* position */
	    kf->Inn->vector[0] = -gps_pos_ned[0]+ins_pos_ned[0];
	    kf->Inn->vector[1] = -gps_pos_ned[1]+ins_pos_ned[1];
	    kf->Inn->vector[2] = -gps_pos_ned[2]+ins_pos_ned[2];

	    /* vel */
	    kf->Inn->vector[3] = -gps[4]+pstate->vel[0];
	    kf->Inn->vector[4] = -gps[5]+pstate->vel[1];
	    kf->Inn->vector[5] = -gps[6]+pstate->vel[2];

	    /* */
	    //Compute Kalmin gain
	    // K = PH' * inv(HPH' + R)
	    kalmanfilter_compute_gain(kf);

	    // brief Update error covariance for updated estimate at current time step
	    // P = (I - KH)P
	    kalmanfilter_update_cov(kf);

	    // dXhat = dXhat_minus(==0) + K ( Innovation )
	    kalmanfilter_update_state(kf);

	    /* apply feedback */
	    /* position (lla)*/
	    pstate->pos[0] -= kf->state->vector[0] /(earth_radius_north(pstate->pos[0]) - pstate->pos[2]);
	    pstate->pos[1] -= kf->state->vector[1]/(earth_radius_east(pstate->pos[0])-pstate->pos[2])*cos(pstate->pos[0]);
	    pstate->pos[2] -= kf->state->vector[2];

	    /* velocity(NED) */
	    pstate->vel[0] -= kf->state->vector[3];
	    pstate->vel[1] -= kf->state->vector[4];
	    pstate->vel[2] -= kf->state->vector[5];

	    /* angles */
	    /*      
	       Ccomp = eul2dcm([eul_ins(k,:)]')';
	       Cerr = I3+sk([attFeedBack(k,:)]);
	       eul_ins(k,:) = dcm2eul((Cerr*Ccomp)')';
	    */    
	    
	    //euleradd(pstate->eul, &(kf->state->vector[6]) );  
	    //int euleradd(double *eul1, double *eul2)
	    double *eul2=&(kf->state->vector[6]);

	    double Ceul1[9];
	    double Ceul2[9];
	    double Cadd[9];
	    double v;

	    euler2dcm(pstate->eul, Ceul1);

	    /* Ceul2 = I3 +sk(eul2) */ //zhiqiangxing  20070425
	    Ceul2[0] = 1.0;	  Ceul2[1] = -eul2[2];	Ceul2[2] = eul2[1];
	    Ceul2[3] = eul2[2];	  Ceul2[4] = 1.0;	Ceul2[5] = -eul2[0];
	    Ceul2[6] = -eul2[1];  Ceul2[7] = eul2[0];	Ceul2[8] = 1.0;

            matrix_mult3x3(MatrixNoTrans, MatrixNoTrans, Ceul2, Ceul1, Cadd);

	    /* transpose result */
	    v = Cadd[1];  Cadd[1] = Cadd[3];  Cadd[3] = v;
	    v = Cadd[2];  Cadd[2] = Cadd[6];  Cadd[6] = v;
	    v = Cadd[5];  Cadd[5] = Cadd[7];  Cadd[7] = v;
	    dcm2euler( Cadd,pstate->eul);

	    /* bias */
	    pstate->accelbias[0] += kf->state->vector[9];
	    pstate->accelbias[1] += kf->state->vector[10];
	    pstate->accelbias[2] += kf->state->vector[11];

	    pstate->gyrobias[0] += kf->state->vector[12];
	    pstate->gyrobias[1] += kf->state->vector[13];
	    pstate->gyrobias[2] += kf->state->vector[14];
			
	    if ( write_output ) {
		write_filter(kf);
	    }
	    // print_filter(kf);

	    // let dXhat_minus=0
	    int ii;
	    for ( ii = 0; ii <= 14; ii++ ) {
		kf->state->vector[ii] = 0;
	    }
                 
	    /* read next gps position */
	    read_gps(gps);
	}
	/************** GPS measurement update ends here ******************/

	if ( write_output ) {
	    write_cov(kf);
	    write_state(imunext[0],&state);
	}
	// print_cov(kf);
	// print_state(&state);

	memcpy( imu,imunext, sizeof( imu ) );
    }

    //	system_free(&s);

    fclose( imufile );
    fclose( gpsfile );
    fclose( result_file );
    fclose( filter_file );
    fclose( cov_file);

    return EXIT_SUCCESS;
}



