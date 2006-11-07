/*-------------------------------------------------------------------
 *
 *     Module:          Navfuncs.c
 *     Modified:        Gokhan Inalhan (remaining) 
 *                      Demoz Gebre (first three functions)
 *                      Jung Soon Jang
 *    
 *     Description:     navfunc.c contains the listing for all the
 *                      real-time inertial navigation software.
 *
 *		Note: all the functions here do not create memory without
 *			  clearing it.
 *
 *--------------------------------------------------------------------*/

/*     Include Pertinent Header Files */

#include <math.h>
#include "matrix.h"
#include "navfunc.h"

/*=================================================================*/

// dcm better be 3 x 3
MATRIX EulerToDcm(MATRIX euler, double decA,MATRIX dcm)
{

    MATRIX A,B;
    double cPHI,sPHI,cTHE,sTHE,cPSI,sPSI;
  
    cPHI = cos(euler[2][0]); sPHI = sin(euler[2][0]);
    cTHE = cos(euler[1][0]); sTHE = sin(euler[1][0]);
    cPSI = cos(euler[0][0]); sPSI = sin(euler[0][0]);

    A = mat_creat(3,3,ZERO_MATRIX);
    B = mat_creat(3,3,UNDEFINED);
  
    A[0][0] = cos(decA); A[0][1] =-sin(decA);
    A[1][0] = sin(decA); A[1][1] = cos(decA);
    A[2][2] = 1;
    
    B[0][0] = cTHE*cPSI; B[0][1] = sPHI*sTHE*cPSI-cPHI*sPSI; B[0][2] = cPHI*sTHE*cPSI+sPHI*sPSI;
    B[1][0] = cTHE*sPSI; B[1][1] = sPHI*sTHE*sPSI+cPHI*cPSI; B[1][2] = cPHI*sTHE*sPSI-sPHI*cPSI;
    B[2][0] =-sTHE;      B[2][1] = sPHI*cTHE;                B[2][2] = cPHI*cTHE;
  
    mat_mul(A,B,dcm);
  
    mat_free(A);
    mat_free(B);
  
 
    return(dcm);

}

/*=================================================================*/

void EcefToLatLonAlt(MATRIX vector)
{

    int i;
    double x, y, z, q, p, sinlat, sinlat2;
    double a, b, d, radius, lat, alt, dummy;
    double E_WGS84, E2_WGS84, ONE_MIN_E2, A_WGS84;
    MATRIX lla;

    lla = mat_creat(3,1,ZERO_MATRIX);

    E_WGS84 = ECCENTRICITY;   /* Earth ellipse ecc - unitless */
    E2_WGS84 = E_WGS84*E_WGS84;  /* Earth's ellipse ecc^2 - unitless */
    ONE_MIN_E2 = 1.0 - E2_WGS84;
    A_WGS84 = EARTH_RADIUS;         /* Earth's ellipse semi-major axis - meters */

    x = vector[0][0];
    y = vector[1][0];
    z = vector[2][0];

    lla[1][0] = atan2(y, x);           /*  Longitude  */

    p = sqrt((x * x) + (y * y));      /*  Latitude and Altitude  */

    if (p < 0.1) {  
        p = 0.1;
    }

    q = z / p;
    alt = 0.0;
    lat = atan(q * (1.0 / ONE_MIN_E2));
    a = 1.0;
    i = 0;

    while ((a > 0.2) && (i < 20)) {
        sinlat = sin(lat);
        sinlat2 = sinlat * sinlat;
        dummy =sqrt((1.0 - (E2_WGS84 * sinlat2))*(1.0 - (E2_WGS84 * sinlat2)));
        radius = A_WGS84 / sqrt(dummy);
        d = alt;
        alt = (p / cos(lat)) - radius;
        a = q * (radius + alt);
        b = (ONE_MIN_E2 * radius) + alt;
        lat = atan2(a, b);
        a = sqrt((alt - d)*(alt - d));
        i = i + 1;
    }

    lla[0][0] = lat;
    lla[2][0] = alt;

    for (i = 0; i < 3; i++) {
        vector[i][0] = lla[i][0];
    }
      
    mat_free(lla);

}


/*=================================================================*/

void EcefToEnu(MATRIX outputVector, MATRIX inputVector, MATRIX position)
{

    int i;
    double lat, lon;
    MATRIX C, ned, ref_position;
    MATRIX position_copy, delta_pos;

    C = mat_creat(3,3,ZERO_MATRIX);
    ref_position = mat_creat(3,1,ZERO_MATRIX);
    delta_pos = mat_creat(3,1,ZERO_MATRIX);
    position_copy = mat_creat(MatRow(position),MatCol(position),ZERO_MATRIX);
    mat_copy(position, position_copy);

    lat = position[0][0];
    lon = position[1][0];

    LatLonAltToEcef(ref_position,position_copy);

    mat_sub(inputVector,ref_position,delta_pos);

    C[0][0] = -sin(lon);
    C[0][1] = cos(lon);
    C[0][2] = 0;
  
    C[1][0] = -sin(lat)*cos(lon);
    C[1][1] = -sin(lat)*sin(lon);
    C[1][2] = cos(lat);
  
    C[2][0] = cos(lat)*cos(lon);
    C[2][1] = cos(lat)*sin(lon);
    C[2][2] = sin(lat);

    ned = mat_creat(MatRow(C),MatCol(delta_pos),ZERO_MATRIX);
    mat_mul(C,delta_pos,ned);

    for (i = 0; i < 3; i++) {
        outputVector[i][0] = ned[i][0];
    }

    mat_free(ned);
    mat_free(C);
    mat_free(delta_pos);
    mat_free(ref_position);
    mat_free(position_copy);
  
}

/*=================================================================*/

void LatLonAltToEcef(MATRIX vector, MATRIX position)
{
    double Rn, ecc2, alt, denom;
    double sinlat, coslat, coslon, sinlon;


    ecc2 = ECCENTRICITY*ECCENTRICITY; 
    sinlat = sin(position[0][0]);
    coslat = cos(position[0][0]);
    coslon = cos(position[1][0]);
    sinlon = sin(position[1][0]);
    alt = position[2][0];

    denom = (1.0 - (ecc2 * sinlat * sinlat));
    denom = sqrt(denom*denom);

    Rn = EARTH_RADIUS / sqrt(denom);

    //  vector[0][0] = (Rn - alt) * coslat * coslon;
    // vector[1][0] = (Rn - alt) * coslat * sinlon;
    //  vector[2][0] = (Rn * (1.0 - ecc2) - alt) * sinlat;
    vector[0][0] = (Rn + alt) * coslat * coslon;
    vector[1][0] = (Rn + alt) * coslat * sinlon;
    vector[2][0] = (Rn * (1.0 - ecc2) + alt) * sinlat;
}

/*=================================================================*/

void nCltrans(MATRIX n_C_l, double magdec)
{
    double gamma;

    gamma=magdec;
    n_C_l[0][0]=-sin(gamma);
    n_C_l[0][1]=cos(gamma);
    n_C_l[0][2]=0;
    n_C_l[1][0]=cos(gamma);
    n_C_l[1][1]=sin(gamma);
    n_C_l[1][2]=0;
    n_C_l[2][0]=0;
    n_C_l[2][1]=0;
    n_C_l[2][2]=-1;

}


/*=================================================================*/

void eCntrans(MATRIX e_C_n, MATRIX LatLon)
{
    double lat,lon;
    
    lat=LatLon[0][0];
    lon=LatLon[1][0];
    
    e_C_n[0][0]=-sin(lon);
    e_C_n[0][1]=-cos(lon)*sin(lat);
    e_C_n[0][2]=cos(lon)*sin(lat);
 
    e_C_n[1][0]=cos(lon);
    e_C_n[1][1]=-sin(lat)*sin(lon);
    e_C_n[1][2]=sin(lon)*cos(lat);

    e_C_n[2][0]=0;
    e_C_n[2][1]=cos(lat);
    e_C_n[2][2]=sin(lat);
}


/*=====================================================================*/

void lCbtrans(MATRIX l_C_b, MATRIX YawPitchRoll)
{
    double psi,theta,phi;
    
    psi=YawPitchRoll[0][0];
    theta=YawPitchRoll[1][0];
    phi=YawPitchRoll[2][0];
    
    l_C_b[0][0]=cos(theta)*cos(psi);
    l_C_b[0][1]=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
    l_C_b[0][2]=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
 
    l_C_b[1][0]=cos(theta)*sin(psi);
    l_C_b[1][1]=cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
    l_C_b[1][2]=-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);

    l_C_b[2][0]=-sin(theta);
    l_C_b[2][1]=sin(phi)*cos(theta);
    l_C_b[2][2]=cos(phi)*cos(theta);
}

