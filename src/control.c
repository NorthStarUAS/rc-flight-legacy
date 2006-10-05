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

#include "globaldefs.h"

void send_servo_cmd(word cnt_cmd[3]);
extern double wraparound(double dta);

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
//external global variables
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern int    sPort0;
extern short  screen_on;
extern short  whichmode[6];
extern double pitch_gain[3],roll_gain[3];
extern double heading_gain[3],alt_gain[3],pos_gain[3];
extern int    numofwaypoints;     
extern double waypoints[8][2];


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//control code
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void control_uav(short init_done, short flight_mode)
{
   word    cnt_cmd[3]={0,};		 	 //elevator,aileron,throttle command
   double  de = 0, da = 0 /*, dthr = 0*/;        //temp. variables
   double  dthe, dphi, /* dpsi, */ dh;           //perturbed variables
   double  dthe_ref,dphi_ref,dpsi_ref=0,dh_ref=0;//perturbed reference variable
   double  tmpr=0,tmpr1=0,nav_psi=0;
   double  Ps_f=0;
   static  short anti_windup[4]={1,};
   /* short   i=0; */
   static short /* count = 0, */ k = 0; 
   static double Ps_f_p=0;

   if (init_done == FALSE) 		         //initialization:
   {
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
   if(numofwaypoints > 0 && whichmode[4] == 1)
   {
       dpsi_ref = atan2(waypoints[k][1]-gpspacket.lon, waypoints[k][0]-gpspacket.lat);
       //measure error
       tmpr = sqrt( (waypoints[k][1]-gpspacket.lon)*(waypoints[k][1]-gpspacket.lon)
                   +(waypoints[k][0]-gpspacket.lat)*(waypoints[k][0]-gpspacket.lat) );

       //if error is in 15 meters
       if(tmpr <= 2.0e-4) {
           if(k == (numofwaypoints-1)) k = 0; 
 	   else                        k = k + 1;

           sum[3] = 0;
       } 
       whichmode[2] = 1; //enable heading control
   }
   else dpsi_ref = 0;

   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //outer-loop control: heading and altitude control
   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //check whether the outer-loop controls are enabled
   //heading
   if (whichmode[2] == 1) {

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
      if(fabs(dphi_ref) > 0.61082) // 35 degrees
      {
	  dphi_ref = sign(dphi_ref)*0.61082;
          anti_windup[2] = 0;
      }
      else anti_windup[2] = 1;
   }
   else dphi_ref = 0;

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
      if(fabs(dthe_ref) > 0.43630) // 25 degrees
      {
	  dthe_ref = sign(dthe_ref)*0.43630;
          anti_windup[3] = 0;
      }
      else anti_windup[3] = 1;
    }
   else dthe_ref = 0;	

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

   //maximum deflection of control servos limited to 20 degrees
   if (fabs(de) > 0.34904) { de = sign(de)*0.34904; anti_windup[0] = 0; }
   else anti_windup[0] = 1;
   //maximum deflection of control servos limited to 15 degrees
   if (fabs(da) > 0.26178) { da = sign(da)*0.26178; anti_windup[1] = 0; }
   else anti_windup[1] = 1;


   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //elervons:elevator and aileron mixing
   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //elevator
   cnt_cmd[1] = servopos.chn[1] + (word)(deg2servo*(de+da)*57.3);
   //aileron
   cnt_cmd[0] = servopos.chn[0] + (word)(deg2servo*(da-de)*57.3);
   //throttle
   cnt_cmd[2] = servopos.chn[2];
   // printf("cnt_cmd[2] = %d %d %d\n", cnt_cmd[2], servopos.chn[2], servopacket.chn[2]);


   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //send commands
   //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   send_servo_cmd(cnt_cmd);
   
}

void send_servo_cmd(word cnt_cmd[3])
{
   //cnt_cmd[1] = ch1:elevator, cnt_cmd[0] = ch0:aileron, cnt_cmd[2] = ch2:throttle
   byte  data[24]={0,};
   short i = 0, nbytes = 0;
   word  sum=0;

   data[0] = 0x55; 
   data[1] = 0x55;
   data[2] = 0x53;
   data[3] = 0x53;

   //elevator
   data[6] = (byte)(cnt_cmd[1] >> 8);
   data[7] = (byte)cnt_cmd[1];
   //throttle
   data[8] = (byte)(cnt_cmd[2] >> 8);
   data[9] = (byte)cnt_cmd[2];
   //aileron
   data[4] = (byte)(cnt_cmd[0] >> 8); 
   data[5] = (byte)cnt_cmd[0];
   
   //checksum:need to be verified
   sum = 0xa6;
   for(i=4;i<22;i++) sum += data[i];
  
   data[22] = (byte)(sum >> 8);
   data[23] = (byte)sum;

   //sendout the command packet
   while (nbytes != 24) nbytes = write(sPort0,(char*)data, 24); 
}

