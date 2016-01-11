/**
 * \file: curt_adns.cpp
 *
 * Test bed for ADNS experimentation
 *
 * Copyright (C) 2009 - Curtis L. Olson
 *
 * $Id: umn_interface.cpp,v 1.1 2009/05/15 17:04:56 curt Exp $
 */


#include "python/pyprops.hxx"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // temp: exit()

#include "include/globaldefs.h"
#include "sensors/gps_mgr.hxx"

#include "math/SGMath.hxx"
#include "glocal.hxx"
#include "adns_curt.hxx"


// property nodes
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode filter_node;

//static SGPropertyNode *imu_timestamp_node = NULL;
//static SGPropertyNode *imu_p_node = NULL;
//static SGPropertyNode *imu_q_node = NULL;
//static SGPropertyNode *imu_r_node = NULL;
//static SGPropertyNode *imu_ax_node = NULL;
//static SGPropertyNode *imu_ay_node = NULL;
//static SGPropertyNode *imu_az_node = NULL;
//static SGPropertyNode *imu_hx_node = NULL;
//static SGPropertyNode *imu_hy_node = NULL;
//static SGPropertyNode *imu_hz_node = NULL;

// gps property nodes
//static SGPropertyNode *gps_time_stamp_node = NULL;
//static SGPropertyNode *gps_lat_node = NULL;
//static SGPropertyNode *gps_lon_node = NULL;
//static SGPropertyNode *gps_alt_node = NULL;
//static SGPropertyNode *gps_ve_node = NULL;
//static SGPropertyNode *gps_vn_node = NULL;
//static SGPropertyNode *gps_vd_node = NULL;

// adns output nodes
//static SGPropertyNode *filter_theta_node = NULL;
//static SGPropertyNode *filter_phi_node = NULL;
//static SGPropertyNode *filter_psi_node = NULL;
//static SGPropertyNode *filter_lat_node = NULL;
//static SGPropertyNode *filter_lon_node = NULL;
//static SGPropertyNode *filter_alt_node = NULL;
//static SGPropertyNode *filter_vn_node = NULL;
//static SGPropertyNode *filter_ve_node = NULL;
//static SGPropertyNode *filter_vd_node = NULL;
//static SGPropertyNode *filter_status_node = NULL;

//static SGPropertyNode *filter_alt_feet_node = NULL;
//static SGPropertyNode *filter_track_node = NULL;
//static SGPropertyNode *filter_vel_node = NULL;
//static SGPropertyNode *filter_vert_speed_fps_node = NULL;

//
static bool init_pos = false;

//
static SGGeod pos_geod;		// Geodetic position
static SGVec3d pos_ecef;	// Position in the ECEF frame
static SGVec3d vel_ned;		// Velocity in the NED frame
static SGVec3d glocal_ned;      // Local Gravity Vector in the NED frame
static SGVec3d total_accel_sum_body;
static SGVec3d total_accel_sum_ned;
static SGVec3d accel_sum_ned;

//
static SGQuatd ned2body;
static SGQuatd ecef2ned;

static SGVec3d gyro_bias;


// bind to property tree
static bool bind_properties( string rootname ) {
    // initialize property nodes
    imu_node = pyGetNode("/sensors/imu");
    //imu_timestamp_node = pyGetNode("/sensors/imu/timestamp");
    //imu_p_node = pyGetNode("/sensors/imu/p-rad_sec", true);
    //imu_q_node = pyGetNode("/sensors/imu/q-rad_sec", true);
    //imu_r_node = pyGetNode("/sensors/imu/r-rad_sec", true);
    //imu_ax_node = pyGetNode("/sensors/imu/ax-mps_sec", true);
    //imu_ay_node = pyGetNode("/sensors/imu/ay-mps_sec", true);
    //imu_az_node = pyGetNode("/sensors/imu/az-mps_sec", true);
    //imu_hx_node = pyGetNode("/sensors/imu/hx", true);
    //imu_hy_node = pyGetNode("/sensors/imu/hy", true);
    //imu_hz_node = pyGetNode("/sensors/imu/hz", true);

    // initialize gps property nodes
    gps_time_stamp_node = pyGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = pyGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = pyGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = pyGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = pyGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = pyGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = pyGetNode("/sensors/gps/vd-ms", true);

    // initialize ahrs output nodes 
    SGPropertyNode *outputroot = pyGetNode( rootname.c_str(), true );
    filter_theta_node = outputroot->getChild("pitch-deg", 0, true);
    filter_phi_node = outputroot->getChild("roll-deg", 0, true);
    filter_psi_node = outputroot->getChild("heading-deg", 0, true);
    filter_lat_node = outputroot->getChild("latitude-deg", 0, true);
    filter_lon_node = outputroot->getChild("longitude-deg", 0, true);
    filter_alt_node = outputroot->getChild("altitude-m", 0, true);
    filter_vn_node = outputroot->getChild("vn-ms", 0, true);
    filter_ve_node = outputroot->getChild("ve-ms", 0, true);
    filter_vd_node = outputroot->getChild("vd-ms", 0, true);
    filter_status_node = outputroot->getChild("navigation",0, true);
    filter_status_node->setStringValue("invalid");

    filter_alt_feet_node = outputroot->getChild("altitude-ft", 0, true);
    filter_track_node = outputroot->getChild("groundtrack-deg", 0, true);
    filter_vel_node = outputroot->getChild("groundspeed-ms", 0, true);
    filter_vert_speed_fps_node
        = outputroot->getChild("vertical-speed-fps", 0, true);

    return true;
}


int curt_adns_init( string rootname ) {
    bind_properties( rootname );
    init_pos = false;

    return 1;
}


static int set_initial_conditions( SGGeod pos, SGVec3d vel ) {

    pos_geod = pos;
    pos_ecef = SGVec3d::fromGeod(pos_geod);
    ecef2ned = SGQuatd::fromLonLat(pos_geod);

    vel_ned = vel;

    SGVec3d euler_ned( 0.0, 0.0, 0.0 );
    ned2body = SGQuatd::fromYawPitchRollDeg( euler_ned[2],
					     euler_ned[1],
					     euler_ned[0] );

    glocal_ned = local_gravity( pos_geod.getLatitudeRad(),
				pos_geod.getElevationM() );

    total_accel_sum_body = SGVec3d(0.0, 0.0, 0.0);
    total_accel_sum_ned = SGVec3d(0.0, 0.0, 0.0);
    accel_sum_ned = SGVec3d(0.0, 0.0, 0.0);

    gyro_bias = SGVec3d(0.0, 0.0, 0.0);

    return true;
}


/* +X axis = forward out the nose
 * +Y axis = out right wing
 * +Z axis = down
 *
 * +roll = right (right wing down, left wing up)
 * +pitch = nose up
 * +yaw = nose right
 *
 * +x accel = forward out the nose
 * +y accel = out right wing
 * +z accel = down
 */

static int propagate_ins( double dt,
			  SGVec3d gyro,
			  SGVec3d accel_body,
			  SGVec3d mag )
{
    static SGVec3d xaxis = SGVec3d(1.0, 0.0, 0.0);
    static SGVec3d yaxis = SGVec3d(0.0, 1.0, 0.0);
    static SGVec3d zaxis = SGVec3d(0.0, 0.0, 1.0);

    // compute a quaternion representing the sensed rotation of all
    // three combined axes
    SGQuatd xrot = SGQuatd::fromAngleAxis( (gyro[0] + gyro_bias[0])*dt, xaxis );
    SGQuatd yrot = SGQuatd::fromAngleAxis( (gyro[1] + gyro_bias[1])*dt, yaxis );
    SGQuatd zrot = SGQuatd::fromAngleAxis( gyro[2]*dt, zaxis );
    SGQuatd rot_body = zrot * yrot * xrot;

    // update the ned2body transform (body orientation estimate)
    ned2body = ned2body * rot_body;

    // transform the body acceleration vector into the ned frame
    SGVec3d accel_ned = ned2body.backTransform(accel_body);

    /* printf("accel body=%.8f %.8f %.8f  ned=%.8f %.8f %.8f\n",
	   accel_body[0], accel_body[1], accel_body[2],
	   accel_ned[0], accel_ned[1], accel_ned[2] ); */

    // sum the total raw inertial accelerations before gravity is added in
    // (because if our attitude estimate is off, the gravity vector
    // correction will skew everything.)
    total_accel_sum_body += accel_body * dt;
    total_accel_sum_ned += accel_ned * dt;

    // FIXME: Add correction for Corriolis forces here (maybe? somewhere?!

    // add in the local gravity vector (danger, this could introduce a
    // big error if the attitude estimate is off) (FIXME: THINK HERE)
    accel_ned += glocal_ned;

    // sum the gravity corrected raw inertial accelerations, note that
    // if there is an attitude estimate error, this can be detected in
    // the correction phase by comparing against gps derived data
    accel_sum_ned += accel_ned * dt;

    // update the inertial velocity in the ned frame
    vel_ned += accel_ned * dt;

    // transform the ned velocity vector into the ecef frame
    SGVec3d vel_ecef = ecef2ned.backTransform(vel_ned);

    // update the position in the ecef frame
    pos_ecef += vel_ecef * dt;

    // compute new geodetic position
    pos_geod = SGGeod::fromCart(pos_ecef);

    // compute new ecef2ned transform
    ecef2ned = SGQuatd::fromLonLat(pos_geod);

    // compute new local gravity vector
    glocal_ned = local_gravity( pos_geod.getLatitudeRad(),
				pos_geod.getElevationM() );

    return 1;
}


// compute an estimate of roll and pitch based on our local
// "percieved" gravity vector in body coordinates
static int g_correction( double dt, SGVec3d gps_vel_ned ) {
    static bool inited = false;
    static SGVec3d last_gps_vel_ned;

    static double the_sum = 0.0;
    static double phi_sum = 0.0;

    if ( !inited ) {
	inited = true;
	last_gps_vel_ned = gps_vel_ned;
    }

    double time_factor;
    if ( dt > 0 ) {
	time_factor = 1.0 / dt;
    } else {
	return 0;
    }
    // this will be close to zero if the velocity vector hasn't
    // changed much
    SGVec3d gps_accel = (gps_vel_ned - last_gps_vel_ned) * time_factor;

    // evaluate the length of the total acceleration vector relative
    // to gravity
    double glen = glocal_ned[2] * dt;
    double accel_len = length(total_accel_sum_body);
    double gps_accel_len = length(gps_accel);
    double dlen = fabs(1.0 - accel_len/glen);

    double y1 = sqrt( total_accel_sum_body[1] * total_accel_sum_body[1] +
		      total_accel_sum_body[2] * total_accel_sum_body[2] );
    double x1 = total_accel_sum_body[0];
    double y2 = sqrt( total_accel_sum_body[0] * total_accel_sum_body[0] +
		      total_accel_sum_body[2] * total_accel_sum_body[2] );
    double x2 = total_accel_sum_body[1];
    if ( total_accel_sum_body[2] < 0.0 ) {
	y1 *= -1.0;
	y2 *= -1.0;
    }
    double the_gest = SGD_PI_2 + atan2( y1, x1 );
    double phi_gest = -SGD_PI_2	- atan2( y2, x2 );
    printf("correction dt=%.2f\n", dt);
    printf("  total_accel_sum_body=%.3f %.3f %.3f (%.3f) (%.3f g)\n",
	   total_accel_sum_body[0],
	   total_accel_sum_body[1],
	   total_accel_sum_body[2],
	   length(total_accel_sum_body),
	   accel_len/glen );
    printf( "pitch est = %.2f\n", the_gest * SGD_RADIANS_TO_DEGREES);
    printf( "roll est = %.2f\n", phi_gest * SGD_RADIANS_TO_DEGREES);

    // avoid any correction work unless the total accel vector is very
    // close to 'g'
    if ( fabs( 1.0 - accel_len / glen ) < 0.01 ) {
	// retrieve our current roll and pitch estimates
	double psi_rad, the_rad, phi_rad;
	ned2body.getEulerRad( psi_rad, the_rad, phi_rad );

	double phi_err = phi_gest - phi_rad;
	double the_err = the_gest - the_rad;

	phi_sum += phi_err;
	the_sum += the_err;

	static SGVec3d xaxis = SGVec3d(1.0, 0.0, 0.0);
	static SGVec3d yaxis = SGVec3d(0.0, 1.0, 0.0);
	printf("(the) gest = %.2f  curr = %.2f  diff = %.2f\n",
	       the_gest, the_rad, the_err);
	printf("(phi) gest = %.2f  curr = %.2f  diff = %.2f\n",
	       phi_gest, phi_rad, phi_err);

	// scale gain according to how close the total accel vector is
	// to gravity.
	double pgain = 0.5;
	double igain = 0.000;

	double phi_correction = ( phi_err * pgain + phi_sum * igain ) * dt;
	double the_correction = ( the_err * pgain + the_sum * igain ) * dt;

	SGQuatd xrot = SGQuatd::fromAngleAxis( phi_correction, xaxis );
	SGQuatd yrot = SGQuatd::fromAngleAxis( the_correction, yaxis );
	SGQuatd rot_body = yrot * xrot;

	// update the ned2body transform (body orientation estimate)
	ned2body = ned2body * rot_body;

	double phi_bias = phi_err * pgain * dt /*phi_correction*/;
	double the_bias = the_err * pgain * dt /*the_correction*/;

	gyro_bias[0] = 0.999 * gyro_bias[0] + 0.001 * phi_bias;
	gyro_bias[1] = 0.999 * gyro_bias[1] + 0.001 * the_bias;
	printf("bias\t%.4f\t%.6f\t%.6f\t%.6f\t%.6f\n",
	       dt, phi_bias, the_bias, gyro_bias[0], gyro_bias[1]);

    }

    last_gps_vel_ned = gps_vel_ned;

    return 1;
}


// compute an estimate of yaw based on our local "percieved" gravity
// vector in body coordinates transformed to ned coordinates (using
// our attitude estimate) and then compared against the total gps
// acceleration vector.  These two acceleration vectors should align
// if our attitude estimate is correct, however, there's no way to
// know which combinations of roll, pitch, and yaw transforms we
// should use to correct the situation.  We are already correcting
// roll/pitch when our sensed acceleration vector is close to g, so we
// only adjust yaw here.
static int v_correction( double dt, SGVec3d gps_vel_ned ) {
    static bool inited = false;
    static SGVec3d last_gps_vel_ned;
    static SGVec3d last_vel_ned;

    if ( !inited ) {
	inited = true;
	last_gps_vel_ned = gps_vel_ned;
	last_vel_ned = vel_ned;
    }

    double time_factor;
    if ( dt > 0 ) {
	time_factor = 1.0 / dt;
    } else {
	return 0;
    }
    SGVec3d gps_accel = (gps_vel_ned - last_gps_vel_ned) * time_factor;

    // subtract gravity
    SGVec3d total_gps_accel = gps_accel - glocal_ned * dt;

    // compute our gravity estimate (subject to attitude estimate error)
    SGVec3d g_est = accel_sum_ned - total_accel_sum_ned;

    printf("correction dt=%.2f\n", dt);
    printf("  gps_accel=%.3f %.3f %.3f (%.3f)\n",
	   gps_accel[0], gps_accel[1], gps_accel[2], length(gps_accel));
    printf("  total_gps_accel=%.3f %.3f %.3f (%.3f)\n",
	   total_gps_accel[0], total_gps_accel[1], total_gps_accel[2],
	   length(total_gps_accel));
    printf("  accel_sum_ned=%.3f %.3f %.3f (%.3f)\n",
	   accel_sum_ned[0], accel_sum_ned[1], accel_sum_ned[2],
	   length(accel_sum_ned));
    printf("  total_accel_sum_ned=%.3f %.3f %.3f (%.3f)\n",
	   total_accel_sum_ned[0],
	   total_accel_sum_ned[1],
	   total_accel_sum_ned[2],
	   length(total_accel_sum_ned));
    printf("  g_est=%.3f %.3f %.3f (%.3f)\n",
	   g_est[0], g_est[1], g_est[2], length(g_est) );

    // FIXME: ins_angle computation?
    double gps_angle = atan2( total_gps_accel[0], total_gps_accel[1] );
    double ins_angle = atan2( total_accel_sum_ned[0], total_accel_sum_ned[1] );

    double angle = gps_angle - ins_angle;
    while ( angle < -SGD_PI ) { angle += SGD_2PI; }
    while ( angle > SGD_PI ) { angle -= SGD_2PI; }
    printf("hdg:\t%.4f\t%.4f\t%.4f\t", gps_angle, ins_angle, angle);
    double max_angle = 8.0 * 0.0174; // radians
    if ( angle > max_angle ) { angle = max_angle; }
    if ( angle < -max_angle ) { angle = -max_angle; }


#ifdef OLD_STUFF
    // SGQuatd correction = SGQuatd::fromRotateTo( accel_sum_ned, gps_accel );
    SGQuatd direct_to = SGQuatd::fromRotateTo( total_gps_accel,
					       total_accel_sum_ned );
    double angle;
    SGVec3d axis;
    direct_to.getAngleAxis( angle, axis );
    printf("estimate off by %.2f degrees\n",
	   angle * SGD_RADIANS_TO_DEGREES );
#endif

    // scale gain according to how far different the total accel
    // vector length is to gravity vector length.
    double glen = glocal_ned[2] * dt;
    double accel_len = length(total_accel_sum_body);
    double dlen = accel_len/glen - 1.0;
    double gain = 0.0;
    if ( dlen > 0.0 ) {
	gain = 10.0 * dlen;
    }
    printf("%.3f %.3f ", accel_len/glen, gain);
    angle *= gain * dt;
    printf("%.3f\n", angle);

    SGVec3d zaxis;
    zaxis = SGVec3d(0.0, 0.0, -1.0);
    SGQuatd correction = SGQuatd::fromAngleAxis( angle, zaxis ); 

    // correction * ned2body means do rotation in ned space (not body space)
    ned2body = correction * ned2body;

    last_gps_vel_ned = gps_vel_ned;
    last_vel_ned = vel_ned;

    return 1;
}


static int update_ins( double gps_dt, SGGeod gps_pos, SGVec3d gps_vel_ned )
{
    // converge towards the gravity vector
    g_correction( gps_dt, gps_vel_ned );

    // converge towards correct yaw 
    v_correction( gps_dt, gps_vel_ned );

    pos_ecef = SGVec3d::fromGeod(gps_pos);
    ecef2ned = SGQuatd::fromLonLat(gps_pos);

    vel_ned = gps_vel_ned;

    total_accel_sum_body = SGVec3d(0.0, 0.0, 0.0);
    total_accel_sum_ned = SGVec3d(0.0, 0.0, 0.0);
    accel_sum_ned = SGVec3d(0.0, 0.0, 0.0);

    return 1;
}


int curt_adns_update( double imu_dt ) {
    // printf("imu dt = %.12f\n", imu_dt);

    static double last_gps_time = 0.0;

    if ( GPS_age() < 1 && !init_pos ) {
	last_gps_time = gps_time_stamp_node->getDouble();
	SGGeod pos = SGGeod::fromDegM( gps_lon_node->getDouble(),
				       gps_lat_node->getDouble(),
				       gps_alt_node->getDouble() );
	SGVec3d vel = SGVec3d( gps_vn_node->getDouble(),
			       gps_ve_node->getDouble(),
			       gps_vd_node->getDouble() );
	// vel[0] = 0.0; vel[1] = 0.0; vel[2] = 0.0; // test values
	set_initial_conditions( pos, vel );
	init_pos = true;
    }	    
    if ( init_pos ) {
	double imu_time = imu_timestamp_node->getDouble();
	SGVec3d gyro = SGVec3d( imu_p_node->getDouble(),
				imu_q_node->getDouble(),
				imu_r_node->getDouble() );
	SGVec3d accel = SGVec3d( imu_ax_node->getDouble(),
				 imu_ay_node->getDouble(),
				 imu_az_node->getDouble() );
	SGVec3d mag = SGVec3d( imu_hx_node->getDouble(),
			       imu_hy_node->getDouble(),
			       imu_hz_node->getDouble() );

	// imu_dt = 0.02;
	// gyro = SGVec3d(0.0, 0.0, 0.01745); // 0.01745 = 1 deg/sec
	// accel = SGVec3d(1.0, 0.0, 0.0);    // m/s
	propagate_ins( imu_dt, gyro, accel, mag );

	double gps_time = gps_time_stamp_node->getDouble();
	if ( gps_time > last_gps_time ) {
	    double gps_dt = gps_time - last_gps_time;
	    last_gps_time = gps_time;

	    SGGeod gps_pos = SGGeod::fromDegM( gps_lon_node->getDouble(),
					       gps_lat_node->getDouble(),
					       gps_alt_node->getDouble() );
	    SGVec3d gps_vel = SGVec3d( gps_vn_node->getDouble(),
				       gps_ve_node->getDouble(),
				       gps_vd_node->getDouble() );

	    update_ins( gps_dt, gps_pos, gps_vel );
	}


	double phi_deg, theta_deg, psi_deg;
	// SGQuatd ned2body_est = ned2body * ned2body_correction;
	// ned2body_est.getEulerDeg( psi_deg, theta_deg, phi_deg );
	ned2body.getEulerDeg( psi_deg, theta_deg, phi_deg );
	/*
	printf( "CF: t=%.2f roll=%.2f pitch=%.2f yaw=%.2f vel=%.2f %.2f %.2f\n",
		imu_time, phi_deg, theta_deg, psi_deg,
		vel_ned[0], vel_ned[1], vel_ned[2] );
	printf( "    pos=%.12f %.12f %.3f\n",
		pos_geod.getLongitudeDeg(), pos_geod.getLatitudeDeg(),
		pos_geod.getElevationM() );
	*/

	// publish values to property tree
	filter_phi_node->setDoubleValue( phi_deg );
	filter_theta_node->setDoubleValue( theta_deg );
	filter_psi_node->setDoubleValue( psi_deg );
	filter_lat_node->setDoubleValue( pos_geod.getLatitudeDeg() );
	filter_lon_node->setDoubleValue( pos_geod.getLongitudeDeg() );
	filter_alt_node->setDoubleValue( pos_geod.getElevationM() );
	filter_vn_node->setDoubleValue( vel_ned[0] );
	filter_ve_node->setDoubleValue( vel_ned[1] );
	filter_vd_node->setDoubleValue( vel_ned[2] );
	filter_status_node->setStringValue("valid");

	filter_alt_feet_node->setDoubleValue( pos_geod.getElevationFt() );
	filter_track_node->setDoubleValue( 90 - atan2(vel_ned[0], vel_ned[1])
	 				* SG_RADIANS_TO_DEGREES );
	filter_vel_node->setDoubleValue( sqrt( vel_ned[0] * vel_ned[0]
	 				    + vel_ned[1] * vel_ned[1] ) );
        filter_vert_speed_fps_node
	->setDoubleValue( -vel_ned[2] * SG_METER_TO_FEET );
    }

    return 1;
}


int curt_adns_close() {
    return true;
}
