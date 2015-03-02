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
void umn_adns_init_R( double cov_gps_hpos, double cov_gps_vpos,
		      double cov_gps_hvel, double cov_gps_vvel );
void umn_adns_init_Rw( double sigma_w_f, double sigma_w_g,
		       double sigma_c_f, double sigma_c_g,
		       double tau_f, double tau_g );
void umn_adns_init_P( double cov_gps_hpos, double cov_gps_vpos,
		      double cov_gps_hvel, double cov_gps_vvel,
		      double sigma_w_f, double sigma_w_g );
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
