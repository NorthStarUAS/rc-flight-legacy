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

#include "globaldefs.h"
#include "adns/mnav/ahrs.h"	// temporary until imupacket dependency is removed?
#include "adns/umn/adns.h"
#include "props/props.hxx"
#include "sensors/gps_mgr.h"
#include "umn_interface.h"


// gps property nodes
static SGPropertyNode *gps_time_stamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;

// adns output nodes
static SGPropertyNode *theta_node = NULL;
static SGPropertyNode *phi_node = NULL;
static SGPropertyNode *psi_node = NULL;

static SGPropertyNode *nav_status_node = NULL;
static SGPropertyNode *nav_lat_node = NULL;
static SGPropertyNode *nav_lon_node = NULL;
static SGPropertyNode *nav_alt_node = NULL;
static SGPropertyNode *nav_alt_feet_node = NULL;
static SGPropertyNode *nav_vn_node = NULL;
static SGPropertyNode *nav_ve_node = NULL;
static SGPropertyNode *nav_vd_node = NULL;
static SGPropertyNode *nav_track_node = NULL;
static SGPropertyNode *nav_vel_node = NULL;
static SGPropertyNode *nav_vert_speed_fps_node = NULL;


int ugumn_adns_init( string rootname ) {
    // initialize ahrs property nodes 
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    theta_node = outputroot->getChild("pitch-deg", 0, true);
    phi_node = outputroot->getChild("roll-deg", 0, true);
    psi_node = outputroot->getChild("heading-deg", 0, true);

    nav_status_node = outputroot->getChild("navigation",0, true);
    nav_status_node->setStringValue("invalid");
    nav_lat_node = outputroot->getChild("latitude-deg", 0, true);
    nav_lon_node = outputroot->getChild("longitude-deg", 0, true);
    nav_alt_node = outputroot->getChild("altitude-m", 0, true);
    nav_alt_feet_node = outputroot->getChild("altitude-ft", 0, true);
    nav_vn_node = outputroot->getChild("vn-ms", 0, true);
    nav_ve_node = outputroot->getChild("ve-ms", 0, true);
    nav_vd_node = outputroot->getChild("vd-ms", 0, true);
    nav_track_node = outputroot->getChild("groundtrack-deg", 0, true);
    nav_vel_node = outputroot->getChild("groundspeed-ms", 0, true);
    nav_vert_speed_fps_node
        = outputroot->getChild("vertical-speed-fps", 0, true);

    // initialize gps property nodes
    gps_time_stamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);

    int result = umn_adns_init();

    return result;
}


int ugumn_adns_update( struct imu *imupacket ) {
    static bool umn_init_pos = false;
    if ( GPS_age() < 1 && !umn_init_pos ) {
	umn_init_pos = true;
	NavState s;
	memset( &s, 0, sizeof(NavState) );
	s.pos[0] =  gps_lat_node->getDoubleValue()
	    * SGD_DEGREES_TO_RADIANS;
	s.pos[1] =  gps_lon_node->getDoubleValue()
	    * SGD_DEGREES_TO_RADIANS;
	s.pos[2] = -gps_alt_node->getDoubleValue();
	umn_adns_set_initial_state( &s );
	umn_adns_print_state( &s );
    }	    
    if ( umn_init_pos ) {
	double imu[7], gps[7];
	imu[0] = imupacket->time;
	imu[1] = imupacket->p;
	imu[2] = imupacket->q;
	imu[3] = imupacket->r;
	imu[4] = imupacket->ax;
	imu[5] = imupacket->ay;
	imu[6] = imupacket->az;
	gps[0] = gps_time_stamp_node->getDoubleValue();
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
	phi_node->setDoubleValue( s->eul[2] * SG_RADIANS_TO_DEGREES );
	theta_node->setDoubleValue( s->eul[1] * SG_RADIANS_TO_DEGREES );
	psi_node->setDoubleValue( s->eul[0] * SG_RADIANS_TO_DEGREES );

	nav_status_node->setStringValue("valid");
	nav_lat_node->setDoubleValue( s->pos[0] * SG_RADIANS_TO_DEGREES );
	nav_lon_node->setDoubleValue( s->pos[1] * SG_RADIANS_TO_DEGREES );
	nav_alt_node->setDoubleValue( -s->pos[2] );
	nav_alt_feet_node->setDoubleValue( -s->pos[2] * SG_METER_TO_FEET );
	nav_vn_node->setDoubleValue( s->vel[0] );
	nav_ve_node->setDoubleValue( s->vel[1] );
	nav_vd_node->setDoubleValue( s->vel[2] );
	nav_track_node->setDoubleValue( 90 - atan2(s->vel[0], s->vel[1])
	 				* SG_RADIANS_TO_DEGREES );
	nav_vel_node->setDoubleValue( sqrt( s->vel[0] * s->vel[0]
	 				    + s->vel[1] * s->vel[1] ) );
        nav_vert_speed_fps_node
            ->setDoubleValue( -s->vel[2] * SG_METER_TO_FEET );
    }

    return 1;
}


int ugumn_adns_close() {
    return umn_adns_close();
}

