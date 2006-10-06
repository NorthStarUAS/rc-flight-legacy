//
// Global defintions used in the avionics program
//

#ifndef _UNAV_GLOBALDEFS_H
#define _UNAV_GLOBALDEFS_H


#define TRUE			1
#define FALSE                   0
#define	byte			unsigned char
#define word			unsigned short
#define NSECS_PER_SEC		1000000000

// ncurses
// #define NCURSE_DISPLAY_OPTION

//Stargate I or II
//#define SG2

enum errdefs { neveruse, neveruse1, no_error, got_invalid, checksum_err,
               gps_update, no_gps_update };

struct imu {
   double p,q,r;		/* angular velocities    */
   double ax,ay,az;		/* acceleration          */
   double hx,hy,hz;             /* magnetic field     	 */
   double Ps,Pt;                /* static/pitot pressure */
// double Tx,Ty,Tz;             /* temperature           */
   double phi,the,psi;          /* attitudes             */
   short  err_type;		/* error type		 */
   double time;
};

struct gps {
   double lat,lon,alt;          /* gps position          */
   double ve,vn,vd;             /* gps velocity          */
   word   ITOW;
   short  err_type;             /* error type            */
   double time;
};

struct nav {
   double lat,lon,alt;
   double ve,vn,vd;
// float  t;
   short  err_type;
   double time;
};

struct servo {
   unsigned short chn[8];
   unsigned char status;
};

struct imu imupacket;
struct gps gpspacket;
struct nav navpacket;
// extern struct servo servopacket; /* moved to imugps.h */

//mutex and conditional variables
pthread_mutex_t	mutex_imu;
pthread_mutex_t	mutex_gps;
pthread_mutex_t mutex_nav;
pthread_cond_t  trigger_ahrs;
pthread_cond_t  trigger_nav;


#endif // _UNAV_GLOBALDEFS_H
