#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "matrix.h"
#include "earth.h"



/*! 
 * \brief Compute local gravity vector
 *
 *   glocal(lat,h) computes the magnitude of the local
 *   gravitational acceleration.  lat is lattitude in
 *   radians and h is altitude in meter.Note: Because a 
 *   North, East, Down coordinate system is assumed, h is
 *   negative for altitude above the reference ellipsoid.
 * 
 * \param lla 
 * \param g 
 * 
 * \return 
 */
int local_gravity(double *lla, double *g)
{
    double mg;
    double sinlat = sin( lla[0] );
    double sin2lat = sin( 2.0 * lla[0] );
    double t;

    mg = EarthG0 * (1.0 + EarthG1 * sinlat*sinlat - EarthG2 * sin2lat*sin2lat);

    t = 1.0 + (lla[2] / EarthRad);
    g[0] = 0.0;
    g[1] = 0.0;
    g[2] = mg / (t*t);

    return 0;
}

double earth_radius_north(double latitude)
{
    double sinlat = sin( latitude );

    return EarthRad * (1.0 + EarthF * (3.0*sinlat*sinlat - 2.0));
}

double earth_radius_east(double latitude)
{
    double sinlat = sin( latitude );

    return EarthRad * (1.0 + EarthF * sinlat*sinlat );
}


/*! 
 * \brief compute rates of NED coordinate sytem
 *
 *   navrate(v,lat): returns the rotation rate of a 
 *   locally level North, East, Down coordinate system
 *   with respect to an earth fixed coordinate system.
 *   The inputs are velocity (v) in m/s, altitude h in meters
 *   and lattitude(lat) in radians.  In a NED coordinate system
 *   h is negative for heights above the reference ellipsoid.
 * 
 * \param lla 
 * \param vel 
 * \param navrate 
 * 
 * \return 
 */
int transport_rate(double *lla, double *vel, double *navrate)
{
    double Rn = earth_radius_north(lla[0]);
    double Re = earth_radius_east(lla[0]);

    navrate[0] =  vel[1] / (Re - lla[2]);
    navrate[1] = -vel[0] / (Rn - lla[2]);
    navrate[2] = -vel[1] * tan(lla[0])/(Re - lla[2]);

    return 0;
}


/*! 
 * \brief Compute earth rate
 *
 * earthrate(lat) computes the earth rate vector in North, East 
 * Down coordinates as a function of latttitude (lat) given in 
 * radians.
 *
 * \param latitude 
 * \param earthrate 
 * 
 * \return 
 */
int earth_rate(double latitude, double *earthrate)
{
    earthrate[0] = EarthRate * cos( latitude );
    earthrate[1] = 0;
    earthrate[2] = EarthRate * (-sin(latitude));

    return 0;
}


/*! 
 * \brief Compute coriolis acceleration
 *
 * coriolis(v,navrate,lat) computes the coriolis acceleration
 * The inputs are velocity (v) in m/s, the navigation frame
 * rotation rate (tranport rate, computed using transport_rate) in rad/s and 
 * lattitude (lat) in radians.
 * 
 * \param latitude 
 * \param vel 
 * \param navrate 
 * \param ca 
 * 
 * \return 
 */
int coriolis_accel(double latitude, double *vel, double *transrate, double *ca)
{
    double rate[3];
    ivl_matrix *skrate = ivl_matrix_new(3,3);
    
    earth_rate(latitude, rate);

    rate[0] = 2.0 * rate[0] + transrate[0];
    rate[1] = 2.0 * rate[1] + transrate[1];
    rate[2] = 2.0 * rate[2] + transrate[2];

    skew_symmetric_3x3( rate, skrate->vector );

    ivl_matrix_dgemv( 0, 1.0, skrate, vel, 0.0, ca );

    ivl_matrix_free( &skrate );

    return 0;
}
