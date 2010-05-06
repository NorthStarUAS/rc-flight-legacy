//
// Global defintions used in the avionics program
//

#ifndef _UGEAR_GLOBALDEFS_H
#define _UGEAR_GLOBALDEFS_H


#include <stdint.h>


enum errdefs {
    NotValid,			/* data not valid */
    ChecksumError,		/* check sum error (and data invalid) */
    ValidData,			/* data recently valid */
};


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

/** Meters per second to Knots */
#define SG_MPS_TO_KT        1.9438444924406046432

/** Knots to meters per second */
#define SG_KT_TO_MPS        0.5144444444444444444

#endif // _UGEAR_GLOBALDEFS_H
