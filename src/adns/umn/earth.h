#ifndef EARTH_H
#define EARTH_H


/* length of semi major axis, metter */
#define EarthRad 6378137.0
/* major eccentricity of the ellipsoid */
#define EarthEcc 0.0818191908426
/* flattening of the ellipsoid */
#define EarthF (1.0/298.257223563)
/* earth rate in radius = 15.041 deg/hr */
#define EarthRate 7.292115E-5


/* Earth gravity constants */
#define EarthG0	9.780373
#define EarthG1	0.0052891
#define EarthG2	0.0000059

int local_gravity(double *lla, double *g);

double earth_radius_north(double latitude);
double earth_radius_east(double latitude);

int transport_rate(double *lla, double *vel, double *navrate);
int earth_rate(double latitude, double *earthrate);

int coriolis_accel(double latitude, double *vel, double *transrate, double *ca);


#endif /* EARTH_H */
