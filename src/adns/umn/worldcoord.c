#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "earth.h"
#include "worldcoord.h"


int lla2ecef( double *lla, double *ecef )
{
    double ecc2 = EarthEcc * EarthEcc;

    double sinlat = sin( lla[0] );
    double coslat = cos( lla[0] );

    double Rn = EarthRad / sqrt( fabs(1.0 - (ecc2 * sinlat * sinlat)) );

    ecef[0] = (Rn - lla[2]) * coslat * cos(lla[1]);
    ecef[1] = (Rn - lla[2]) * coslat * sin(lla[1]);
    ecef[2] = (Rn * (1.0 - ecc2) - lla[2]) * sinlat;

    return 0;
}


int ecef2ned( double *ecef, double *ref_lla, double *ned )
{
    double sinlat = sin( ref_lla[0] );
    double coslat = cos( ref_lla[0] );

    double sinlong = sin( ref_lla[1] );
    double coslong = cos( ref_lla[1] );

    ned[0] = -ecef[0] * sinlat * coslong 
	- ecef[1] * sinlat * sinlong 
	+ ecef[2] * coslat;
				 
    ned[1] = -ecef[0] * sinlong 
	+ ecef[1]  * coslong;
				 
    ned[2] = -ecef[0] * coslat * coslong 
	- ecef[1] * coslat * sinlong 
	- ecef[2] * sinlat;             

    return 0;
}


int lla2ned( double *lla, double *ref_lla, double *ned )
{
    double ecef[3];

    lla2ecef( lla, ecef );
    ecef2ned( ecef, ref_lla, ned );

    return 0;
}

