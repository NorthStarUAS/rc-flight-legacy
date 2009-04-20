#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "ivl_matrix.h"

#include "earth.h"
#include "eulerdcm.h"


/*! 
 * \brief Compute strapdown math to update state by dt
 *
 * pos = [latitude longitude height] rad rad meter
 * vel = [veln vele veld] meter/second
 * eul = [yaw pitch roll] radians
 *
 * imu = [angle_rates accelerations]
 * angle_rates = [rrx rry rrz] rad/sec
 * accerations = [ax ay az] meter/sec^2
 * 
 */
int strapdown_er( double *pos, double *vel, double *eul, ivl_matrix *Cb2n,
		  double *imu, double dt )
{
    double g[3];
    double Rns, Rew;
    double transrate[3];
    double totalrate[3];
    double coriolis[3];
    double dp[3], dv[3], de[3];
    double tmp[3];
    double s1, s2, c1, c2;
    ivl_matrix *Cb2n_rot = ivl_matrix_new(3,3);

    euler2dcm( eul, Cb2n->vector );

    local_gravity( pos, g );
    Rns = earth_radius_north( pos[0] );
    Rew = earth_radius_east( pos[0] );

    transport_rate( pos, vel, transrate );
    earth_rate( pos[0], totalrate ); //totalrate=earthRate now

    totalrate[0] += transrate[0];
    totalrate[1] += transrate[1];
    totalrate[2] += transrate[2];

    coriolis_accel( pos[0], vel, transrate, coriolis );

    /* postion increment */
    //  dp1 = [v0(1)/(Rns-p0(3));  v0(2)/((Rew-p0(3))*cos(p0(1)));   v0(3)];
    dp[0] = vel[0] / ( Rns - pos[2]);
    dp[1] = vel[1] / (( Rew - pos[2]) * cos( pos[0] ));
    dp[2] = vel[2];

    /* velocity increment */
    //   dv1 = Cb2n*[imu(k-1,5:7)-accelBias(k-1,:)]'  -sk(2*earthRate + transRate)*v0    +gE;
    /* dv = Cb2n * imu[4:6] */

    // FIXME: this function uses imu[3], but test_closedloop uses imu[4]
    // imu[3] seems to make more sense?!?
    ivl_matrix_dgemv( 0, 1.0, Cb2n, &imu[3], 0.0, dv );
    //ivl_matrix_dgemv( 0, 1.0, Cb2n, &imu[4], 0.0, dv );

    /* dv = imu - cor + g */
    dv[0] = dv[0] - coriolis[0] + g[0];
    dv[1] = dv[1] - coriolis[1] + g[1];
    dv[2] = dv[2] - coriolis[2] + g[2];

    /* angle increment */
    //   de1 = Cb2n_rot*[imu(k-1,2:4)-gyroBias(k-1,:)-(Cb2n'*totalRate)']';
    /* euler angle rates */
    /*
      %euler angle rates = M_b2n_rot * (imu angle rates)
      % assuming nav frame is stationary (no angle rate)
      % R = [rrx rry rrz]'
      % (yaw)psi rate   = [0 sec(theda)*sin(phi) sec(theda)*cos(phi)] * R
      % (pitch)theda rate = [0 cos(phi) -sin(phi)] * R
      % (roll)phi rate   = [1 tan(theda)*sin(phi) tan(theda)*cos(phi)] * R
    */
    s1 = sin( eul[2] );
    c1 = cos( eul[2] );

    s2 = sin( eul[1] );
    c2 = cos( eul[1] );

    Cb2n_rot->matrix[0][0] = 0.0;
    Cb2n_rot->matrix[0][1] = s1/c2;
    Cb2n_rot->matrix[0][2] = c1/c2;
    Cb2n_rot->matrix[1][0] = 0.0;
    Cb2n_rot->matrix[1][1] = c1;
    Cb2n_rot->matrix[1][2] = -s1;
    Cb2n_rot->matrix[2][0] = 1.0;
    Cb2n_rot->matrix[2][1] = (s2*s1)/c2;
    Cb2n_rot->matrix[2][2] = (s2*c1)/c2;

    /* imu - Cb2n' * totalRate */
    ivl_matrix_dgemv( 1, 1.0, Cb2n, totalrate, 0.0, tmp );

    // FIXME!!! another off by one discrepancy?!?
    tmp[0] = imu[0] - tmp[0]; // from strapdown.c
    tmp[1] = imu[1] - tmp[1]; 
    tmp[2] = imu[2] - tmp[2];
    /*tmp[0] = imu[1] - tmp[0]; // from test_closedloop
    tmp[1] = imu[2] - tmp[1];
    tmp[2] = imu[3] - tmp[2];*/

    /* de = Cb2n_rot * (imu[0:3] - Cb2n*totalrate)*/
    ivl_matrix_dgemv( 0, 1.0, Cb2n_rot, tmp, 0.0, de );

    // propagate navigation states
    pos[0] += dp[0] * dt;
    pos[1] += dp[1] * dt;
    pos[2] += dp[2] * dt;

    vel[0] += dv[0] * dt;
    vel[1] += dv[1] * dt;
    vel[2] += dv[2] * dt;

    eul[0] += de[0] * dt;
    eul[1] += de[1] * dt;
    eul[2] += de[2] * dt;

    ivl_matrix_free( &Cb2n_rot );

    return 0;
}


/*
  % body to nav
  Cb2n = eul2dcm(eul0)';

  [gE,cg0] = glocal( pos0(1), pos0(3) );
  [Rns,Rew,Rearth] = earthrad( pos0(1) );
  transRate = navrate( vel0, pos0(3), pos0(1) );
  earthRate = earthrate( pos0(1) );
  totalRate = earthRate+transRate;

  % position
  dp1 = [...
  vel0(1)/(Rns-pos0(3));...
  vel0(2)/((Rew-pos0(3))*cos(pos0(1)));...
  vel0(3)...
  ];
  pos1 = pos0 + dt*dp1;

  % velocity
  dv1 = Cb2n * [imu(4:6)]'-coriolis(vel0,transRate,pos0(1))+gE;
  vel1 = vel0 + dt*dv1;

  % attitude
  % compute euler angle rates
  Cb2n_rot = eul2dcm2(eul0);

  de1 = Cb2n_rot*[imu(1:3)-(Cb2n'*totalRate)']';
  eul1 = eul0 + dt*de1;
*/
