/******************************************************************************
 * FILE: navigation.cpp
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 8/31/06 Jung Soon Jang
 ******************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "globaldefs.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "props/props.hxx"
#include "sensors/GPS.h"
#include "sensors/IMU.h"
#include "util/matrix.h"
#include "util/myprof.h"
#include "util/navfunc.h"
#include "util/timing.h"

#include "ahrs.h"
#include "coremag.h"
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
struct nav navpacket;
short  gps_init_count = 0;
static double magvar_rad = 0.0;

MATRIX nxs,nF,nG,nGd,nu;
MATRIX nPn,nQn,nRn,nKn,nRinv;
MATRIX dcm;
MATRIX euler,nIden;
MATRIX ntmp99,ntmpr,ntmp91,ntmpr91,ntmprr;
MATRIX ntmp66,ntmp96,ntmp33;

// nav estimate property nodes
static SGPropertyNode *magvar_deg_node = NULL;;
static SGPropertyNode *nav_lat_node = NULL;
static SGPropertyNode *nav_lon_node = NULL;
static SGPropertyNode *nav_alt_feet_node = NULL;
static SGPropertyNode *nav_track_node = NULL;
// static SGPropertyNode *nav_vel_node = NULL;
static SGPropertyNode *nav_vert_speed_fps_node = NULL;
static SGPropertyNode *pressure_error_m_node = NULL;


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
  
    navpacket.status = NotValid;

    // initialize nav property nodes
    magvar_deg_node = fgGetNode("/config/nav-filter/magvar-deg", true);
    if ( strcmp(magvar_deg_node->getStringValue(), "auto") == 0 ||
	 strlen(magvar_deg_node->getStringValue()) == 0 )
    {
	magvar_rad = 0.0;
    } else {
	magvar_rad = magvar_deg_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
    }
    nav_lat_node = fgGetNode("/position/latitude-deg", true);
    nav_lon_node = fgGetNode("/position/longitude-deg", true);
    nav_alt_feet_node = fgGetNode("/position/altitude-nav-ft", true);
    nav_track_node = fgGetNode("/orientation/groundtrack-deg", true);
    // nav_vel_node = fgGetNode("/velocities/groundspeed-ms", true);
    nav_vert_speed_fps_node
        = fgGetNode("/velocities/vertical-speed-fps",true);
    pressure_error_m_node = fgGetNode("/position/pressure-error-m", true);

    if ( display_on ) {
        printf("[nav] initialized, magvar = %s (deg).\n",
	       magvar_deg_node->getStringValue());
    }
}


void nav_update()
{
    nav_prof.start();

    struct imu	     imulocal;
    struct gps	     gpslocal;
    static int       gps_state = 0;
    static double    acq_start = 0; // time that gps first acquired

    static float Ps_filt_err = 0.0;
    static float Ps_count  = 0.0;
    const float Ps_span = 2500.0;   // counts @ 10hz

    double cur_time = get_Time();

    // on boot up, wait until 20 seconds after gps acquired before
    // using data in order to let the solution [hopefully] get more
    // accurate.

    if ( gpspacket.status == ValidData ) {
        if ( gps_state == 0 ) {
            gps_state = 1;      // gps first acquired
            acq_start = cur_time;
            if ( display_on ) printf("[nav] gps first aquired.\n");
        } else if ( gps_state == 1 ) {
            if ( cur_time - acq_start >= 20.0 ) {
                gps_state = 2;  // gps solid for 20 sec

                // initialize navigation state variables with GPS (Lat,Lon,Alt)
                nxs[0][0] = gpspacket.lat*D2R;
                nxs[1][0] = gpspacket.lon*D2R;
                nxs[2][0] = gpspacket.alt;

		// initialize magnetic variation
		if ( strcmp(magvar_deg_node->getStringValue(), "auto") == 0 ) {
		    long int jd = unixdate_to_julian_days( gpspacket.date );
		    double field[6];
		    magvar_rad
			= calc_magvar( gpspacket.lat*SGD_DEGREES_TO_RADIANS,
				       gpspacket.lon*SGD_DEGREES_TO_RADIANS,
				       gpspacket.alt / 1000.0,
				       jd, field );
		}
                if ( display_on ) {
		    printf("[nav] navigation is enabled, magvar = %.1f (deg)\n",
			   magvar_rad * SGD_RADIANS_TO_DEGREES );
		}
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
           
        navpacket.lat  = nxs[0][0]*R2D;
        navpacket.lon  = nxs[1][0]*R2D;
        navpacket.alt  = nxs[2][0];
        navpacket.vn   = nxs[3][0];
        navpacket.ve   = nxs[4][0];
        navpacket.vd   = nxs[5][0];
        navpacket.time = get_Time();
	navpacket.status = ValidData;

	// compute a filtered error difference between gps altitude
	// and pressure altitude.  (at 4hz update rate this averages
	// the error over about 40 minutes)
        Ps_count += 1.0; if (Ps_count > (Ps_span - 1.0)) {
            Ps_count = (Ps_span - 1.0);
        }
	float alt_err = navpacket.alt - imupacket.Ps;
        Ps_filt_err = (Ps_count / Ps_span) * Ps_filt_err
            + ((Ps_span - Ps_count) / Ps_span) * alt_err;
        // printf("cnt = %.0f err = %.2f\n", Ps_count, Ps_filt_err);

        // publish values to property tree
	nav_lat_node->setDoubleValue( navpacket.lat );
	nav_lon_node->setDoubleValue( navpacket.lon );
	nav_alt_feet_node->setDoubleValue( navpacket.alt * SG_METER_TO_FEET );
	nav_track_node->setDoubleValue( 90 - atan2(navpacket.vn, navpacket.ve)
	 				* SG_RADIANS_TO_DEGREES );
	// nav_vel_node->setDoubleValue( sqrt( navpacket.vn * navpacket.vn
	// 				    + navpacket.ve * navpacket.ve ) );
        nav_vert_speed_fps_node
            ->setDoubleValue( -navpacket.vd * SG_METER_TO_FEET );
        pressure_error_m_node->setFloatValue( Ps_filt_err );

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

    nav_prof.stop();
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
    static double last_gps_time = 0;

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
    EulerToDcm( euler, magvar_rad, ntmp33 );
    mat_scalMul( ntmp33, dt, dcm );
   
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
    // propagation of navigation equation (prediction step)
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

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // update when new GPS data is available (correction step)
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ( gpsdta->status == ValidData && gpsdta->time > last_gps_time )
    {
	last_gps_time = gpsdta->time;
       
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
