#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "matrix.h"

#include "eulerdcm.h"


/*! 
 * \brief Convert euler angles to Direction Cosine matrix
 *
 * Convert euler angles (yaw/psi, pitch/theda, roll/phi) into a direction cosine
 * matrix (dcm). 
 *
 * C1=[1 0			0;...
 * 0 cos(phi)	sin(phi);...
 * 0 -sin(phi)	cos(phi)];
 *
 * theda, rotate about y axis
 * C2=[cos(theda)	0 -sin(theda);...
 * 0		1 0;...
 * sin(theda) 0 cos(theda)];
 *
 * %psi, rotate about z axis
 * C3=[cos(psi)  sin(psi) 0;...
 * -sin(psi) cos(psi) 0;...
 *  0		 0		 1];
 *  dcm = C3*C2*C1
 * 
 * \param eul 
 * \param dcm 
 * 
 * \return 
 */
int euler2dcm(double *eul, double *dcm)
{
    double c1 = cos( eul[2] );   
    double c2 = cos( eul[1] ); 
    double c3 = cos( eul[0] );
	
    double s1 = sin( eul[2] );   
    double s2 = sin( eul[1] ); 
    double s3 = sin( eul[0] );

    dcm[0] = c3*c2;  dcm[1] = -c1*s3+c3*s1*s2;  dcm[2] = s1*s3+c3*c1*s2;
    dcm[3] = s3*c2;  dcm[4] = c1*c3+s3*s1*s2;	dcm[5] = -s1*c3+s3*c1*s2;
    dcm[6] = -s2;    dcm[7] = s1*c2;		dcm[8] = c1*c2;

    return 0;
}

/*! 
 * \brief convert direction cosine matrix to euler angles
 *
 * dcm2eul computes the euler angles (in radians) 
 * given the direction cosine matrix C.
 *
 * \param dcm 
 * \param eul 
 * 
 * \return 
 */
int dcm2euler(double *dcm, double *eul)
{
    //eul(1,1) = atan2(C(1,2),C(1,1));  %     yaw
    eul[0] = atan2( dcm[1], dcm[0] );

    //eul(2,1) = -asin(C(1,3));         %     pitch
    eul[1] = - asin( dcm[2] );
	
    //eul(3,1) = atan2(C(2,3),C(3,3));  %     roll
    eul[2] = atan2( dcm[5], dcm[8] );

    return 0;
}



int eulerdiff(double *eul1, double *eul2, double *diff)
{
    double Ceul1[9];
    double Ceul2[9];
    double Cdiff[9];

    /* convert euler angles to dcm */
    euler2dcm( eul1, Ceul1 );
    euler2dcm( eul2, Ceul2);

    /* Cdiff = Ceul2 * Ceul1' */
    matrix_mult3x3( MatrixNoTrans, MatrixTrans, Ceul2, Ceul1, Cdiff );

    /* small angle approx. */
    diff[0] =  Cdiff[5]; /* Cdiff(2,3) */
    diff[1] = -Cdiff[2]; /* Cdiff(1,3) */
    diff[2] =  Cdiff[1]; /* Cdiff(1,2) */

    return 0;
}


/*! 
 * \brief 
 *
 * eul1 <- eul1 + eul2
 * 
 * \param eul1 
 * \param eul2 
 * 
 * \return 
 */
int euleradd(double *eul1, double *eul2)
{
    double Ceul1[9];
    double Ceul2[9];
    double Cadd[9];
    double v;

    euler2dcm( eul1, Ceul1);

    /* Ceul2 = I3 - sk(eul2) */
    Ceul2[0] = 1.0;	  Ceul2[1] = eul2[2];	Ceul2[2] = -eul2[1];
    Ceul2[3] = -eul2[2];  Ceul2[4] = 1.0;	Ceul2[5] = eul2[0];
    Ceul2[6] = eul2[1];	  Ceul2[7] = -eul2[0];	Ceul2[8] = 1.0;

    matrix_mult3x3( MatrixNoTrans, MatrixNoTrans, Ceul2, Ceul1, Cadd );

    /* transpose result */
    v = Cadd[1];
    Cadd[1] = Cadd[3];
    Cadd[3] = v;

    v = Cadd[2];
    Cadd[2] = Cadd[6];
    Cadd[6] = v;

    v = Cadd[5];
    Cadd[5] = Cadd[7];
    Cadd[7] = v;

    dcm2euler( Cadd, eul1);

    return 0;
}
