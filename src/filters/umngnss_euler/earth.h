#ifndef EARTH_H
#define EARTH_H


/* major eccentricity of the ellipsoid */
#define EarthEcc 0.0818191908426
/* flattening of the ellipsoid */
#define EarthF (1.0/298.257223563)
/* length of semi major axis, metter */
#define EarthRad 6378137.0
#define R_0 6378137.0
#define R_P (R_0*(1 - EarthF))
/* earth rate in radius = 15.041 deg/hr */
#define EarthRate 7.292115e-5
#define OmegaIE 7.292115e-5
#define mu_E 3.986004418e14

/* Earth gravity constants */
#define EarthG0	9.780373
#define EarthG1	0.0052891
#define EarthG2	0.0000059

void local_gravity(double lat_rad, double alt_m, double *g);
int local_gravity_old(double *lla, double *g);

double earth_radius_north(double latitude);
double earth_radius_east(double latitude);

int transport_rate(double *lla, double *vel, double *navrate);
int earth_rate(double latitude, double *earthrate);

int coriolis_accel(double latitude, double *vel, double *transrate, double *ca);


#endif /* EARTH_H */
