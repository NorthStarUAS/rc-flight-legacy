/******************************************************************************
 * FILE: control.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 10/11/05 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "comms/logging.h"
#include "comms/uplink.h"
#include "navigation/mnav.h"
#include "include/globaldefs.h"

#include "util.h"
#include "xmlauto.hxx"

#include "control.h"


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined constant
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define	cdT	      0.04   	     // 25 Hz synchronized with ahrs
#define deg2servo     819    	     // 65536/80deg.
#define servo_mid_pos 32768          // middle position of the servos
#define sign(arg)    (arg>=0 ? 1:-1)
#define MAG_DEC      -0.270944862    /*magnetic declination of Stanford (rad): -15.15 */


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static double sum[5]={0.,};
static struct servo    servopos;
static struct imu      imuval;
static struct gps      gpsval;
static struct nav      navval;
enum   	      modedefs {pitch_mode,roll_mode,heading_mode,altitude_mode,speed_mode,waypoint_mode};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//control code
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static  short anti_windup[4]={1,};
static short k = 0; 
static double Ps_f_p=0;

// the "FlightGear" autopilot
static FGXMLAutopilot ap;


void control_init() {
    ap.init();
    ap.build();
}


void control_reset() {
  // initialization:
  if ( display_on ) { printf("Initializing autopilot\n"); }

  servopos = servopacket;  	         // save the last servo positions
  imuval   = imupacket;                    // save the last attitude
  gpsval   = gpspacket;                    // save the last gps
  navval   = navpacket;                    // save the last nav
  Ps_f_p   = 0.0;			
  k        = 0;

}


void control_update(short flight_mode)
{
    uint16_t cnt_cmd[9]={0,};		 	 //elevator,aileron,throttle command
    double  de = 0, da = 0 /*, dthr = 0*/;        //temp. variables
    double  dthe, dphi, /* dpsi, */ dh;           //perturbed variables
    double  dthe_ref,dphi_ref,dpsi_ref=0,dh_ref=0;//perturbed reference variable
    double  tmpr=0,tmpr1=0,nav_psi=0;
    double  Ps_f=0;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //pass through the first order low pass filter to remove noise
    //G(s)=1/(tau s + 1), tau =0.4;
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    Ps_f   = 0.9048*Ps_f_p + 0.09516*imupacket.Ps;
    Ps_f_p = Ps_f;
    dh     = Ps_f  - imuval.Ps;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // simple pass through
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //elevator
    cnt_cmd[1] = servopacket.chn[1] + (imupacket.the * 1600.0 * 57.3);
    printf("%d\n", cnt_cmd[1]);
    //aileron
    cnt_cmd[0] = servopacket.chn[0] + (imupacket.phi * 16000 * 57.3);
    //throttle
    cnt_cmd[2] = servopacket.chn[2];
    // rudder
    cnt_cmd[3] = servopacket.chn[3];

    ap.update( 0.04 );	// dt = 1/25


    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //send commands
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    send_servo_cmd(cnt_cmd);
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//control code
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void control_uav_old(bool init_done, short flight_mode)
{
    uint16_t cnt_cmd[9]={0,};		 	 //elevator,aileron,throttle command
    double  de = 0, da = 0 /*, dthr = 0*/;        //temp. variables
    double  dthe, dphi, /* dpsi, */ dh;           //perturbed variables
    double  dthe_ref,dphi_ref,dpsi_ref=0,dh_ref=0;//perturbed reference variable
    double  tmpr=0,tmpr1=0,nav_psi=0;
    double  Ps_f=0;
    static  short anti_windup[4]={1,};
    static short k = 0; 
    static double Ps_f_p=0;

    if ( !init_done ) {
        // initialization:
        servopos = servopacket;  	         // save the last servo positions
        imuval   = imupacket;                    // save the last attitude
        gpsval   = gpspacket;                    // save the last gps
        navval   = navpacket;                    // save the last nav
        Ps_f_p   = 0.0;			
        sum[0]=sum[1]=sum[2]=sum[3]=sum[4]= 0.;  // initialize integral sums
        anti_windup[0]=anti_windup[1]=0;
        anti_windup[2]=anti_windup[3]=0;
        k        = 0;
        //printf("\n[control]::control is initialized..!\n");
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //obtain the purturbed states
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    dthe   = imupacket.the; 
    dphi   = imupacket.phi;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //pass through the first order low pass filter to remove noise
    //G(s)=1/(tau s + 1), tau =0.4;
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    Ps_f   = 0.9048*Ps_f_p + 0.09516*imupacket.Ps;
    Ps_f_p = Ps_f;
    dh     = Ps_f  - imuval.Ps;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //outer-loop control: position control
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(numofwaypoints > 0 && whichmode[4] == 1) {
        dpsi_ref = atan2(waypoints[k][1]-gpspacket.lon, waypoints[k][0]-gpspacket.lat);
        //measure error
        tmpr = sqrt( (waypoints[k][1]-gpspacket.lon)*(waypoints[k][1]-gpspacket.lon)
                     +(waypoints[k][0]-gpspacket.lat)*(waypoints[k][0]-gpspacket.lat) );

        //if error is in 15 meters
        if(tmpr <= 2.0e-4) {
            if (k == (numofwaypoints-1))
                k = 0; 
            else
                k = k + 1;
            
            sum[3] = 0;
        }
        whichmode[2] = 1; //enable heading control
    } else {
        dpsi_ref = 0;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //outer-loop control: heading and altitude control
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //check whether the outer-loop controls are enabled
    //heading
    if ( whichmode[2] == 1 ) {

        if (pos_gain[1] == 0)
            nav_psi  = atan2(navpacket.ve, navpacket.vn);
        else
            nav_psi  = imupacket.psi;

        //tmpr1    = sin(dpsi_ref - nav_psi);  
        //wrap around
        tmpr1    = wraparound(dpsi_ref - nav_psi);
      
        sum[3]  += tmpr1*cdT*anti_windup[2];

        dphi_ref = heading_gain[0]*tmpr1
            - heading_gain[2]*imupacket.r;
        // FIXME: no affect in original code
        /* + heading_gain[1]*sum[3]; */

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //bound the command input
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if(fabs(dphi_ref) > 0.61082) {
            // 35 degrees
            dphi_ref = sign(dphi_ref)*0.61082;
            anti_windup[2] = 0;
        } else {
            anti_windup[2] = 1;
        }
    } else {
        dphi_ref = 0;
    }

    //altitude
    if (whichmode[3] == 1) {
        dh_ref   = 0.0;
        sum[2]  += (dh_ref - dh)*cdT*anti_windup[3];
        dthe_ref = alt_gain[0]*(dh_ref - dh) 
            + alt_gain[2]*gpspacket.vd;
        // FIXME: no affect in original code
        /* + alt_gain[1]*sum[2]; */
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //bound the command input
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if(fabs(dthe_ref) > 0.43630) {
            // 25 degrees
            dthe_ref = sign(dthe_ref)*0.43630;
            anti_windup[3] = 0;
        } else {
            anti_windup[3] = 1;
        }
    } else {
        dthe_ref = 0;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //inner-loop control: pitch and roll attitude control
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    sum[0]+= (dthe_ref - dthe)*cdT*anti_windup[0]; //pitch integral
    sum[1]+= (dphi_ref - dphi)*cdT*anti_windup[1]; //roll integral

   
    de  = pitch_gain[0]*(dthe_ref - dthe)	 // radian
        + pitch_gain[2]*(-imupacket.q)
        + pitch_gain[1]*sum[0]
        + pos_gain[0]*fabs(dphi);		 // compensate for the altitude loss
    da  = roll_gain[0]*(dphi_ref - dphi)          // radian
        + roll_gain[2]*(-imupacket.p)
        + roll_gain[1]*sum[1];

    da = -dphi;

    // maximum deflection of control servos limited to 20 degrees
    if (fabs(de) > 0.34904) {
        de = sign(de)*0.34904;
        anti_windup[0] = 0;
    } else {
        anti_windup[0] = 1;
    }
    //maximum deflection of control servos limited to 15 degrees
    if (fabs(da) > 0.26178) {
        da = sign(da)*0.26178;
        anti_windup[1] = 0;
    } else {
        anti_windup[1] = 1;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //elervons:elevator and aileron mixing
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //elevator
    cnt_cmd[1] = servopos.chn[1] + (uint16_t)(deg2servo*(de+da)*57.3);
    //aileron
    cnt_cmd[0] = servopos.chn[0] + (uint16_t)(deg2servo*(da-de)*57.3);
    //throttle
    cnt_cmd[2] = servopos.chn[2];
    // printf("cnt_cmd[2] = %d %d %d\n", cnt_cmd[2], servopos.chn[2], servopacket.chn[2]);

    // for ( i = 0; i < 9; ++i ) {
    //     cnt_cmd[i] = servopos.chn[2];
    // }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //send commands
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    send_servo_cmd(cnt_cmd);
}

