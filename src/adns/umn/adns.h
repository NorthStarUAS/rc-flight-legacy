/**
 * \file: adns.h
 *
 * Front end interface for UMN ADNS algorithm
 *
 * Copyright (C) 2009 - Aerospace Engineering and Mechanics Dept, University of Minnesota
 *
 * $Id: adns.h,v 1.2 2009/04/27 01:29:09 curt Exp $
 */


#ifndef _UGEAR_ADNS_UMN_H
#define _UGEAR_ADNS_UMN_H

#ifdef __cplusplus
extern "C" {
#endif

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

void umn_adns_print_state( NavState *state );
void umn_adns_print_gps( double *gps );

#ifdef __cplusplus
}
#endif

#endif // _UGEAR_ADNS_UMN_H
