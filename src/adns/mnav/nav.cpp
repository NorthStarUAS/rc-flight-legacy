/******************************************************************************
 * FILE: navigation.cpp
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 8/31/06 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

#include "comms/console_link.h"
#include "comms/logging.h"
#include "include/globaldefs.h"
#include "props/props.hxx"
#include "util/matrix.h"
#include "util/myprof.h"
#include "util/navfunc.h"
#include "util/timing.h"

#include "nav.h"


//
// prototypes
//
void nav_algorithm(struct imu *imudta,struct gps *gpsdta);

//
// error characteristics of navigation parameters
//
#define	    Vcov    		0.04                        // (0.2 m/s)^2 
#define     Pcov    		(5.0e-5*D2R)*(5.0e-5*D2R)   // (5.0e-5*d2r)
#define     Rew     		6.359058719353925e+006      //earth radius
#define     Rns     		6.386034030458164e+006      //earth radius
#define     DEFAULT_USECS	100000			    //10 Hz

//
// global matrix variables
//
MATRIX nxs,nF,nG,nGd,nu;
MATRIX nPn,nQn,nRn,nKn,nRinv;
MATRIX dcm;
MATRIX euler,nIden;
MATRIX ntmp99,ntmpr,ntmp91,ntmpr91,ntmprr;
MATRIX ntmp66,ntmp96,ntmp33;

short  gps_init_count = 0;

// nav (cooked gps/accelerometer) property nodes
// static SGPropertyNode *nav_lat_node = NULL;
// static SGPropertyNode *nav_lon_node = NULL;
static SGPropertyNode *nav_alt_feet = NULL;
// static SGPropertyNode *nav_track_node = NULL;
// static SGPropertyNode *nav_vel_node = NULL;
static SGPropertyNode *nav_vert_speed_fps = NULL;


void timer_intr( int sig )
{
    return;
}


// initialize nav structures and matrices
void nav_init()
{
    // matrix creation for navigation computation
    nxs   = mat_creat(9,1,ZERO_MATRIX);   //state x=[lat lon alt ve vn vup bax bay baz]'
    nF    = mat_creat(9,9,ZERO_MATRIX);   //system matrix
    nG    = mat_creat(9,6,ZERO_MATRIX);	 //input matrix
    nGd   = mat_creat(9,6,ZERO_MATRIX);	 //input matrix in discrete time
    nKn   = mat_creat(9,6,ZERO_MATRIX);   //gain matrix
    nu    = mat_creat(6,1,ZERO_MATRIX); nu[5][0] = GRAVITY_NOM;
    dcm   = mat_creat(3,3,ZERO_MATRIX);   //directional cosine matrix
    ntmp33= mat_creat(3,3,ZERO_MATRIX);    
    euler = mat_creat(3,1,UNDEFINED);     //euler angles
    nIden = mat_creat(9,9,UNIT_MATRIX);   //identity matrix
    nRinv = mat_creat(6,6,ZERO_MATRIX);   
    ntmp99= mat_creat(9,9,ZERO_MATRIX);
    ntmpr = mat_creat(9,9,ZERO_MATRIX);
    ntmp91= mat_creat(9,1,ZERO_MATRIX);
    ntmpr91=mat_creat(9,1,ZERO_MATRIX);
    ntmp66 =mat_creat(6,6,ZERO_MATRIX);
    ntmp96 =mat_creat(9,6,ZERO_MATRIX);

    nPn    = mat_creat(9,9,UNIT_MATRIX); //error covariance matrix
    nQn    = mat_creat(9,9,ZERO_MATRIX); //process noise error matrix

    nQn[0][0] = 0;       
    nQn[1][1] = nQn[0][0];
    nQn[2][2] = nQn[0][0];
    nQn[3][3] = 0.18*0.18;     
    nQn[4][4] = nQn[3][3];     
    nQn[5][5] = nQn[3][3];     
    nQn[6][6] = 1.0e-8;
    nQn[7][7] = nQn[6][6]; 
    nQn[8][8] = nQn[6][6]; 
   
    nRn = mat_creat(6,6,ZERO_MATRIX); //measurement noise error matrix
    nRn[0][0] = Pcov; nRn[1][1] = nRn[0][0]; nRn[2][2] = 2.0*2.0;  
    nRn[3][3] = 0.01; nRn[4][4] = nRn[3][3]; nRn[5][5] = 0.02; 
  
    navpacket.err_type = no_gps_update;

    // initialize nav property nodes
    // nav_lat_node = fgGetNode("/position/latitude-deg", true);
    // nav_lon_node = fgGetNode("/position/longitude-deg", true);
    nav_alt_feet = fgGetNode("/position/altitude-nav-ft", true);
    // nav_track_node = fgGetNode("/orientation/groundtrack-deg", true);
    // nav_vel_node = fgGetNode("/velocities/groundspeed-ms", true);
    nav_vert_speed_fps = fgGetNode("/velocities/vertical-speed-fps", true);

    if ( display_on ) {
        printf("[nav] initialized.\n");
    }
}


void nav_update()
{
    struct imu	     imulocal;
    struct gps	     gpslocal;
    static int       gps_state = 0;
    static double    acq_start = 0; // time that gps first acquired

    double cur_time = get_Time();

    // on boot up, wait until 20 seconds after gps acquired before
    // using data in order to let the solution [hopefully] get more
    // accurate.

    if ( gpspacket.err_type == no_error ) {
        if ( gps_state == 0 ) {
            gps_state = 1;      // gps first acquired
            acq_start = cur_time;
            if ( display_on ) printf("[nav] gps first aquired.\n");
        } else if ( gps_state == 1 ) {
            if ( cur_time - acq_start >= 20.0 ) {
                gps_state = 2;  // gps solid for 20 sec
                navpacket.err_type = no_error;

                // initialize navigation state variables with GPS (Lat,Lon,Alt)
                nxs[0][0] = gpspacket.lat*D2R;
                nxs[1][0] = gpspacket.lon*D2R;
                nxs[2][0] = gpspacket.alt;

                if ( display_on ) printf("[nav] navigation is enabled\n");
            } else {
                static int gps_counter = 0;
                gps_counter++;
                if ( gps_counter >= 10 ) {
                    if ( display_on ) {
                        printf( "[nav] gps ready in %.1f seconds.\n",
                                20.0 - (cur_time - acq_start) );
                    }
                    gps_counter = 0;
                }
            }
        }
    }

    if ( gps_state == 2 ) {
        // copy information to local variables
        gpslocal = gpspacket;
        imulocal = imupacket;
                   
        // run navigation algorithm
	nav_alg_prof.start();
        nav_algorithm( &imulocal, &gpslocal );
	nav_alg_prof.stop();
           
        navpacket.lat = nxs[0][0]*R2D;
        navpacket.lon = nxs[1][0]*R2D;
        navpacket.alt = nxs[2][0];
        navpacket.vn  = nxs[3][0];
        navpacket.ve  = nxs[4][0];
        navpacket.vd  = nxs[5][0];
        navpacket.time= get_Time();

        // publish values to property tree
	// nav_lat_node->setDoubleValue( navpacket.lat );
	// nav_lon_node->setDoubleValue( navpacket.lon );
	nav_alt_feet->setDoubleValue( navpacket.alt * SG_METER_TO_FEET );
	// nav_track_node->setDoubleValue( atan2(navpacket.vn, navpacket.ve)
	// 				* SG_RADIANS_TO_DEGREES );
	// nav_vel_node->setDoubleValue( sqrt( navpacket.vn * navpacket.vn
	// 				    + navpacket.ve * navpacket.ve ) );
        nav_vert_speed_fps->setDoubleValue( -navpacket.vd * SG_METER_TO_FEET );

        if ( console_link_on ) {
            console_link_nav( &navpacket );
        }

        if ( log_to_file ) {
            log_nav( &navpacket );
        }

        if ( display_on ) {
            snap_time_interval("nav", 20, 1);
        }
    }
}


void nav_close()
{
    // free memory space
    mat_free(nxs);
    mat_free(nF);
    mat_free(nG);
    mat_free(nPn);
    mat_free(nQn);
    mat_free(nRn);
    mat_free(nRinv);
    mat_free(nKn);
    mat_free(euler);   
    mat_free(nIden);
    mat_free(ntmp99);
    mat_free(ntmp91);
    mat_free(ntmpr91);
    mat_free(ntmpr);
    mat_free(nGd);
    mat_free(nu);
    mat_free(ntmp66);
    mat_free(ntmp96);
    mat_free(ntmp33);
}


//
// navigation algorithm
//
void nav_algorithm(struct imu *imudta,struct gps *gpsdta)
{
    double dt;       //sampling rate of navigation
    short  i = 0;
    double yd[6];  
    static double tnow, tprev = 0; 

    tnow = get_Time();
    dt   = tnow - tprev; tprev = tnow;
    if ( dt == 0 ) dt = 0.100;

    // matrix initialization
    euler[0][0] = imudta->psi;
    euler[1][0] = imudta->the;
    euler[2][0] = imudta->phi;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //fill out F and G
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    EulerToDcm(euler,MAG_DEC,ntmp33);
    mat_scalMul(ntmp33,dt,dcm);
   
    for(i=0;i<9;i++) nF[i][i]=0;
    nF[0][3] = dt/(Rns + nxs[2][0]);
    nF[1][4] = dt/((Rew + nxs[2][0])*cos(nxs[0][0]));
    nF[2][5] =-dt;
    nF[3][6] = -dcm[0][0]; nF[3][7] = -dcm[0][1]; nF[3][8] = -dcm[0][2];
    nF[4][6] = -dcm[1][0]; nF[4][7] = -dcm[1][1]; nF[4][8] = -dcm[1][2];
    nF[5][6] = -dcm[2][0]; nF[5][7] = -dcm[2][1]; nF[5][8] = -dcm[2][2];

    nG[3][0] =  dcm[0][0]; nG[3][1] =  dcm[0][1]; nG[3][2] =  dcm[0][2]; nG[3][3] = dt;
    nG[4][0] =  dcm[1][0]; nG[4][1] =  dcm[1][1]; nG[4][2] =  dcm[1][2]; nG[4][4] = dt;
    nG[5][0] =  dcm[2][0]; nG[5][1] =  dcm[2][1]; nG[5][2] =  dcm[2][2]; nG[5][5] = dt;
   
    //discretization of G
    mat_scalMul(nF,0.5,ntmp99); for(i=0;i<9;i++) ntmp99[i][i]+=1;
    mat_mul(ntmp99,nG,nGd);
    //discretization of F
    for(i=0;i<9;i++) nF[i][i]+=1;
   
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //propagation of navigation equation
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    nu[0][0] = (imudta->ax);
    nu[1][0] = (imudta->ay);
    nu[2][0] = (imudta->az);

    //nxs = nF*nxs + nGd*nu
    mat_mul(nF,nxs,ntmp91);
    mat_mul(nGd,nu,ntmpr91);
    mat_add(ntmp91,ntmpr91,nxs); 

    //error covriance propagation: P = Fd*P*Fd' + Q
    mat_mul(nF,nPn,ntmp99);
    mat_tran(nF,ntmpr);
    mat_mul(ntmp99,ntmpr,nPn);
    for(i=0;i<9;i++) nPn[i][i] += nQn[i][i];

    // update using GPS
    if ( gpsdta->err_type == no_error ) {
        gpspacket.err_type = no_error;
       
        //gain matrix Kn = P*H'*(H*P*H' + R)^-1
        mat_subcopy(nPn, 6, 6, ntmp66);
        for(i=0;i<6;i++) ntmp66[i][i] += nRn[i][i];
        mat_inv(ntmp66,nRinv);
        mat_subcopy(nPn, 9, 6, ntmp96);
        mat_mul(ntmp96,nRinv,nKn);
       
        // error covariance matrix update
        // P = (I - K*H)*P
        mat_subcopy(nKn, 9, 6, ntmp99);
        for(i=1;i<9;i++) { ntmp99[i][6]=ntmp99[i][7]=ntmp99[i][8]=0; }
       
        mat_sub(nIden,ntmp99,ntmpr);
        mat_mul(ntmpr,nPn, ntmp99);
        mat_copy(ntmp99,nPn);
       
        // state update
        yd[0] = (gpsdta->lat*D2R - nxs[0][0]);
        yd[1] = (gpsdta->lon*D2R - nxs[1][0]);
        yd[2] = (gpsdta->alt     - nxs[2][0]);
        yd[3] = (gpsdta->vn      - nxs[3][0]);
        yd[4] = (gpsdta->ve      - nxs[4][0]);
        yd[5] = (gpsdta->vd      - nxs[5][0]);

        for ( i = 0; i < 9; i++ ) {
            nxs[i][0] += nKn[i][0]*yd[0] + nKn[i][1]*yd[1] + nKn[i][2]*yd[2]
                + nKn[i][3]*yd[3] + nKn[i][4]*yd[4] + nKn[i][5]*yd[5];
        }
    } // gps update
}
