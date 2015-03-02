#ifndef STRAPDOWN_H
#define STRAPDOWN_H


#include "ivl_matrix.h"


int strapdown_er( double *pos, double *vel, double *eul, ivl_matrix *Cb2n,
		  double *imu, double dt );


#endif /* STRAPDOWN_H */
