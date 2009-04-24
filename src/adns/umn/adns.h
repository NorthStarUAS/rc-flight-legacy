/**
 * \file: adns.h
 *
 * Front end interface for UMN ADNS algorithm
 *
 * Copyright (C) 2009 - Aerospace Engineering and Mechanics Dept, University of Minnesota
 *
 * $Id: adns.h,v 1.1 2009/04/24 17:47:11 curt Exp $
 */


#ifndef _UGEAR_ADNS_UMN_H
#define _UGEAR_ADNS_UMN_H


#include "kalmanfilter.h"


typedef struct
{
    double pos[3];
    double vel[3];
    double eul[3];

    double gyrobias[3];
    double accelbias[3];
} NavState;


int umn_adns_init();
void umn_adns_set_initial_state( NavState *s );
int umn_adns_update( double *imu, double *gps );
NavState *umn_adns_get_state();
KalmanFilter *umn_adns_get_kf();
int umn_adns_close();


#endif // _UGEAR_ADNS_UMN_H
