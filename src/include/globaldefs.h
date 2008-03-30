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
   uint64_t err_type;           /* error type            */
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
    uint64_t command_sequence;  /* highest received command sequence num */
    uint64_t loadavg;           /* system "1 minute" load average */
    uint64_t ahrs_hz;           /* actual ahrs loop hz */
    uint64_t nav_hz;            /* actual nav loop hz */
};

extern struct imu imupacket;
extern struct gps gpspacket;
extern struct nav navpacket;
extern struct servo servo_in;
extern struct servo servo_out;
extern struct health healthpacket;

// A varienty of constants

/** For divide by zero avoidance, this will be close enough to zero */
#define SG_EPSILON 0.0000001

#define M_2PI 6.28318530717958647692

#ifndef M_PI
#define SG_PI  3.1415926535f
#else
#define SG_PI  ((float) M_PI)
#endif

#ifdef M_PI_2
#  define  SGD_PI_2  M_PI_2
#else
#  define  SGD_PI_2  1.57079632679489661923
#endif

#ifndef M_PI
#define SGD_PI 3.14159265358979323846   /* From M_PI under Linux/X86 */
#else
#define SGD_PI M_PI
#endif

/** 2 * PI */
#define SGD_2PI      6.28318530717958647692

#define SG_180   180.0f
#define SGD_180  180.0
#define SG_DEGREES_TO_RADIANS  (SG_PI/SG_180)
#define SG_RADIANS_TO_DEGREES  (SG_180/SG_PI)
#define SGD_DEGREES_TO_RADIANS  (SGD_PI/SGD_180)
#define SGD_RADIANS_TO_DEGREES  (SGD_180/SGD_PI)

#define SG_FEET_TO_METER    0.3048
#define SG_METER_TO_FEET    3.28083989501312335958

/** Nautical Miles to Meters */
#define SG_NM_TO_METER      1852.0000

/** Meters to Nautical Miles.  1 nm = 6076.11549 feet */
#define SG_METER_TO_NM      0.0005399568034557235

/** Statute Miles to Meters. */
#define SG_SM_TO_METER      1609.3412196

/** Radians to Nautical Miles.  1 nm = 1/60 of a degree */
#define SG_NM_TO_RAD        0.00029088820866572159

/** Nautical Miles to Radians */
#define SG_RAD_TO_NM        3437.7467707849392526

#endif // _UGEAR_GLOBALDEFS_H
