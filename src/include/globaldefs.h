//
// Global defintions used in the avionics program
//

#ifndef _UGEAR_GLOBALDEFS_H
#define _UGEAR_GLOBALDEFS_H


#include <stdint.h>

#define NSECS_PER_SEC		1000000000

//Stargate I or II
//#define SG2

enum errdefs { neveruse, neveruse1, no_error, got_invalid, checksum_err,
               gps_update, no_gps_update };

struct imu {
   double time;
   double p,q,r;		/* angular velocities    */
   double ax,ay,az;		/* acceleration          */
   double hx,hy,hz;             /* magnetic field     	 */
   double Ps,Pt;                /* static/pitot pressure */
   // double Tx,Ty,Tz;          /* temperature           */
   double phi,the,psi;          /* attitudes             */
   uint64_t  err_type;		/* error type		 */
};

struct gps {
   double time;
   double lat,lon,alt;          /* gps position          */
   double ve,vn,vd;             /* gps velocity          */
   double ITOW;                 /* seconds since start of week */
   uint64_t err_type;            /* error type            */
};

struct nav {
   double time;
   double lat,lon,alt;
   double ve,vn,vd;
   // float  t;
   uint64_t  err_type;
};

struct servo {
   double time;
   uint16_t chn[8];
   uint64_t status;
};

struct health {
    double time;
    float volts_raw;            /* raw volt reading */
    float volts;                /* filtered volts */
    uint64_t est_seconds;       /* estimated useful seconds remaining */
    uint64_t loadavg;            /* system "1 minute" load average */
    uint64_t ahrs_hz;            /* actual ahrs loop hz */
    uint64_t nav_hz;             /* actual nav loop hz */
};

extern struct imu imupacket;
extern struct gps gpspacket;
extern struct nav navpacket;
extern struct servo servopacket;
extern struct health healthpacket;

// constants
#define M_2PI 6.28318530717958647692

#ifndef M_PI
#define SG_PI  3.1415926535f
#else
#define SG_PI  ((float) M_PI)
#endif

#define SG_180   180.0f

#define SG_DEGREES_TO_RADIANS  (SG_PI/SG_180)
#define SG_RADIANS_TO_DEGREES  (SG_180/SG_PI)


#endif // _UGEAR_GLOBALDEFS_H
