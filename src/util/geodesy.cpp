#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

// These are hard numbers from the WGS84 standard.  DON'T MODIFY
// unless you want to change the datum.
#define EQURAD 6378137.0
#define _FLATTENING 298.257223563

#define _SQUASH (1 - 1/_FLATTENING)
#define _STRETCH (1/_SQUASH)
#define _POLRAD (EQURAD * _SQUASH)

#define E2 fabs(1 - _SQUASH*_SQUASH)
// static double a = EQURAD;
static double ra2 = 1/(EQURAD*EQURAD);
// static double e = sqrt(E2);
static double e2 = E2;
static double e4 = E2*E2;

Vector3d ecef2lla_for_ublox6(const Vector3d cart) {
    // according to
    // H. Vermeille,
    // Direct transformation from geocentric to geodetic ccordinates,
    // Journal of Geodesy (2002) 76:451-454
    Vector3d geod;
    double X = cart(0);
    double Y = cart(1);
    double Z = cart(2);
    double XXpYY = X*X+Y*Y;
    if( XXpYY + Z*Z < 25 ) {
        // This function fails near the geocenter region, so catch that special case here.
        // Define the innermost sphere of small radius as earth center and return the 
        // coordinates 0/0/-EQURAD. It may be any other place on geoide's surface,
        // the Northpole, Hawaii or Wentorf. This one was easy to code ;-)
        geod << 0.0, 0.0, -EQURAD;
        return geod;
    }
    
    double sqrtXXpYY = sqrt(XXpYY);
    double p = XXpYY*ra2;
    double q = Z*Z*(1-e2)*ra2;
    double r = 1/6.0*(p+q-e4);
    double s = e4*p*q/(4*r*r*r);
    /* 
       s*(2+s) is negative for s = [-2..0]
       slightly negative values for s due to floating point rounding errors
       cause nan for sqrt(s*(2+s))
       We can probably clamp the resulting parable to positive numbers
    */
    if( s >= -2.0 && s <= 0.0 )
        s = 0.0;
    double t = pow(1+s+sqrt(s*(2+s)), 1/3.0);
    double u = r*(1+t+1/t);
    double v = sqrt(u*u+e4*q);
    double w = e2*(u+v-q)/(2*v);
    double k = sqrt(u+v+w*w)-w;
    double D = k*sqrtXXpYY/(k+e2);
    double sqrtDDpZZ = sqrt(D*D+Z*Z);
    geod <<
        2*atan2(Z, D+sqrtDDpZZ), // latitude(rad)
        2*atan2(Y, X+sqrtXXpYY), // longitude(rad)
        (k+e2-1)*sqrtDDpZZ/k;    // elevation(m)
    return geod;
}

/// Return a quaternion rotation from the earth centered to the
/// simulation usual horizontal local frame from given
/// longitude and latitude.
/// The horizontal local frame used in simulations is the frame with x-axis
/// pointing north, the y-axis pointing eastwards and the z axis
/// pointing downwards.
Quaterniond fromLonLatRad(double lon_rad, double lat_rad) {
    Quaterniond q;
    double zd2 = 0.5 * lon_rad;
    double yd2 = -0.25 * M_PI - 0.5 * lat_rad;
    double Szd2 = sin(zd2);
    double Syd2 = sin(yd2);
    double Czd2 = cos(zd2);
    double Cyd2 = cos(yd2);
    q = Quaterniond( Czd2*Cyd2, -Szd2*Syd2, Czd2*Syd2, Szd2*Cyd2 );
    return q;
}

Vector3d quat_backtransform( Quaterniond q, Vector3d v ) {
    q = q.inverse();
    q.normalize();

    Quaterniond p;
    p.w() = 0;
    p.vec() = v;
    Quaterniond rotatedP = q * p * q.inverse(); 
    Vector3d rotatedV = rotatedP.vec();
    return rotatedV;
}
