#ifndef COORDTRANS_H
#define COORDTRANS_H


/*
 * Global Coordinats (NAD83)
 * lla = [latitude longitude, altitude]
 * latitude is + north in radians
 * longitude is + west in radians
 * altitude is + down, - up
 *
 * ECEF Coordinates
 * Earth Centered earth fixed
 *
 * NED Coordinats
 * North is X axis
 * East is Y axis
 * Z is down
 *
 * ned  = [north east up]
 * north is meters
 * east is meters
 *
 */

int lla2ecef( double *lla, double *ecef );
int ecef2ned( double *ecef, double *ref_lla, double *ned );
int lla2ned( double *lla, double *ref_lla, double *ned );


#endif /* COORDTRANS_H */
