/**
 * \file: umn_interface.h
 *
 * C++/Property aware interface for UMN ADNS algorithm
 *
 * Copyright (C) 2009 - Curtis L. Olson
 *
 * $Id: umn_interface.cpp,v 1.1 2009/05/15 17:04:56 curt Exp $
 */


#include <math.h>
#include <string.h>

#include <umngnss/adns.h>

#include "include/globaldefs.h"
#include "props/props.hxx"
#include "sensors/gps_mgr.h"

#include "umn_interface.h"


// imu property nodes
static SGPropertyNode *imu_timestamp_node = NULL;
static SGPropertyNode *imu_p_node = NULL;
static SGPropertyNode *imu_q_node = NULL;
static SGPropertyNode *imu_r_node = NULL;
static SGPropertyNode *imu_ax_node = NULL;
static SGPropertyNode *imu_ay_node = NULL;
static SGPropertyNode *imu_az_node = NULL;
static SGPropertyNode *imu_hx_node = NULL;
static SGPropertyNode *imu_hy_node = NULL;
static SGPropertyNode *imu_hz_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;

// filter property nodes
static SGPropertyNode *filter_timestamp_node = NULL;
static SGPropertyNode *filter_theta_node = NULL;
static SGPropertyNode *filter_phi_node = NULL;
static SGPropertyNode *filter_psi_node = NULL;
static SGPropertyNode *filter_lat_node = NULL;
static SGPropertyNode *filter_lon_node = NULL;
static SGPropertyNode *filter_alt_node = NULL;
static SGPropertyNode *filter_vn_node = NULL;
static SGPropertyNode *filter_ve_node = NULL;
static SGPropertyNode *filter_vd_node = NULL;
static SGPropertyNode *filter_status_node = NULL;

static SGPropertyNode *filter_alt_feet_node = NULL;
static SGPropertyNode *filter_track_node = NULL;
static SGPropertyNode *filter_vel_node = NULL;
static SGPropertyNode *filter_vert_speed_fps_node = NULL;


int ugumn_adns_init( string rootname ) {
    // initialize imu property nodes
    imu_timestamp_node = fgGetNode("/sensors/imu/time-stamp");
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);

    // initialize gps property nodes
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);

    // initialize ahrs property nodes 
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    filter_timestamp_node = outputroot->getChild("time-stamp", 0, true);
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

    int result = umn_adns_init();

    return result;
}


bool ugumn_adns_update() {
    static bool umn_init_pos = false;

    bool fresh_data = false;

    if ( GPS_age() < 1 && !umn_init_pos ) {
	umn_init_pos = true;
	NavState s;
	memset( &s, 0, sizeof(NavState) );
	s.pos[0] = gps_lat_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
	s.pos[1] = gps_lon_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
	s.pos[2] = -gps_alt_node->getDoubleValue();
	s.eul[0] = SGD_PI_2 - atan2( gps_vn_node->getDoubleValue(),
				     gps_ve_node->getDoubleValue() );
	umn_adns_set_initial_state( &s );
	umn_adns_print_state( &s );
    }	    

    if ( umn_init_pos ) {
	double imu[7], gps[7];
	imu[0] = imu_timestamp_node->getDoubleValue();
	imu[1] = imu_p_node->getDoubleValue();
	imu[2] = imu_q_node->getDoubleValue();
	imu[3] = imu_r_node->getDoubleValue();
	imu[4] = imu_ax_node->getDoubleValue();
	imu[5] = imu_ay_node->getDoubleValue();
	imu[6] = imu_az_node->getDoubleValue();
	gps[0] = gps_timestamp_node->getDoubleValue();
	gps[1] = gps_lat_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
	gps[2] = gps_lon_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
	gps[3] = -gps_alt_node->getDoubleValue();
	gps[4] = gps_vn_node->getDoubleValue();
	gps[5] = gps_ve_node->getDoubleValue();
	gps[6] = gps_vd_node->getDoubleValue();
	// umn_adns_print_gps( gps );
	umn_adns_update( imu, gps );

	NavState *s = umn_adns_get_state();

	// publish values to property tree
	double psi = s->eul[0];
	if ( psi < 0 ) { psi += SGD_2PI; }
	if ( psi > SGD_2PI ) { psi -= SGD_2PI; }
	filter_timestamp_node->setDoubleValue( imu[0] );
	filter_phi_node->setDoubleValue( s->eul[2] * SG_RADIANS_TO_DEGREES );
	filter_theta_node->setDoubleValue( s->eul[1] * SG_RADIANS_TO_DEGREES );
	filter_psi_node->setDoubleValue( psi * SG_RADIANS_TO_DEGREES );
	filter_lat_node->setDoubleValue( s->pos[0] * SG_RADIANS_TO_DEGREES );
	filter_lon_node->setDoubleValue( s->pos[1] * SG_RADIANS_TO_DEGREES );
	filter_alt_node->setDoubleValue( -s->pos[2] );
	filter_vn_node->setDoubleValue( s->vel[0] );
	filter_ve_node->setDoubleValue( s->vel[1] );
	filter_vd_node->setDoubleValue( s->vel[2] );
	filter_status_node->setStringValue("valid");

	filter_alt_feet_node->setDoubleValue( -s->pos[2] * SG_METER_TO_FEET );
	filter_track_node->setDoubleValue( 90 - atan2(s->vel[0], s->vel[1])
	 				* SG_RADIANS_TO_DEGREES );
	filter_vel_node->setDoubleValue( sqrt( s->vel[0] * s->vel[0]
	 				    + s->vel[1] * s->vel[1] ) );
        filter_vert_speed_fps_node
            ->setDoubleValue( -s->vel[2] * SG_METER_TO_FEET );

	fresh_data = true;
    }

    return fresh_data;
}


int ugumn_adns_close() {
    return umn_adns_close();
}

