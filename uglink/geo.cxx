#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "geo.hxx"


#if 0
static void print_vec3( const char *pref, SGVec3d v ) {
    printf("%s %.3f %.3f %.3f\n", pref, v[0], v[1], v[2]);
}
#endif


static SGQuatd compute_ned2body( double roll_deg, double pitch_deg, double yaw_deg ) {
    double yaw_rad = yaw_deg * SGD_DEGREES_TO_RADIANS;
    double pitch_rad = pitch_deg * SGD_DEGREES_TO_RADIANS;
    double roll_rad = roll_deg * SGD_DEGREES_TO_RADIANS;

    SGQuatd ned2body
	= SGQuatd::fromYawPitchRoll( yaw_rad, pitch_rad, roll_rad );

    return ned2body;
}


// compute diagonal field of view of the camera
static double compute_diag_fov( double h_mm, double v_mm, double focal_len_mm )
{
    double diag_mm = sqrt(h_mm*h_mm + v_mm*v_mm);
    //printf("hmm=%.2f vmm=%.2f dmm=%.2f\n", h_mm, v_mm, diag_mm);

    //double hfov = 2.0 * atan( h_mm / (2.0*focal_len_mm) );
    //double vfov = 2.0 * atan( v_mm / (2.0*focal_len_mm) );
    double dfov = 2.0 * atan( diag_mm / (2.0*focal_len_mm) );
    /*printf("hfov=%.2f vfov=%.2f dfov=%.2f\n",
	   hfov * SGD_RADIANS_TO_DEGREES,
	   vfov * SGD_RADIANS_TO_DEGREES,
	   dfov * SGD_RADIANS_TO_DEGREES);*/

    return dfov;
}

// compute a base unit vector representing the 'look down' ned give a
// 'half' camera diagonal and the angle to sweep it.  Gives us the
// ability to compute a ned vector for the corner of the camera image.
static SGVec3d compute_base_ned_vector( double half_diag, double rotation ) {
    SGVec3d vec;
    vec.z() = cos( half_diag );
    double hdist = sin( half_diag );
    vec.x() = cos( rotation ) * hdist;
    vec.y() = sin( rotation ) * hdist;

    return vec;
}


#if 0
// Compute lookat NED vector from camera angles and body attitude.
static SGVec3d ned_from_camera_angles1( double pan_deg, double tilt_deg,
				       SGQuatd ned2body )
{
    SGVec3d dir_body;

    double pan_rad = pan_deg * SGD_DEGREES_TO_RADIANS;
    double tilt_rad = tilt_deg * SGD_DEGREES_TO_RADIANS;

    dir_body.z() = sin( tilt_rad );
    double hdist = cos( tilt_rad );
    dir_body.x() = cos( pan_rad ) * hdist;
    dir_body.y() = sin( pan_rad ) * hdist;

    SGVec3d dir_ned = ned2body.backTransform(dir_body);
    return dir_ned;
}
#endif


// Estimate wgs84 target point from current ned lookat vector.
// (Estimate is based on current lookat ground altitude which needs to
// be updated regularly from external source for best accuracy.)
static SGGeod wgs84_from_ned( double ground_elev_m,
			      SGGeod pos_geod,
			      SGVec3d dir_ned )
{
    double agl_m = pos_geod.getElevationM() - ground_elev_m;
    printf("agl_m = %.2f\n", agl_m);
    double a = atan2(dir_ned.x(), dir_ned.y());
    double proj_deg = (M_PI/2.0 - a) * SGD_RADIANS_TO_DEGREES;
    //printf("proj_deg = %.8f\n", proj_deg);
    double d_factor = sqrt( dir_ned.x()*dir_ned.x() + dir_ned.y()*dir_ned.y() );
    //printf("d_factor = %.2f\n", d_factor);
    double proj_dist = 0.0;
    if ( dir_ned.z() > 0.0 ) {
	proj_dist = (agl_m / dir_ned.z()) * d_factor;
	// sanity check: don't look more than 100km beyond
	// current location and don't look backwards
	if ( proj_dist < 0.0 ) {
	    proj_dist = 0.0;
	}
	if ( proj_dist > 100000.0 ) {
	    proj_dist = 100000.0;
	}
    } else {
	// if lookat vector is at or above the horizon, put
	// the wgs84 point just below the horizon
	proj_dist = 100000.0;
    }
    
    SGGeod ground_geod = pos_geod;
    ground_geod.setElevationM( ground_elev_m );
    SGGeod new_target;
    double final_course;
    SGGeodesy::direct( ground_geod, proj_deg, proj_dist,
		       new_target, final_course);
    new_target.setElevationM( ground_elev_m );

    return new_target;
}


#if 0
static SGVec3d offset_from_reference( SGGeod p, SGGeod ref ) {
    SGVec3d result;
    double course1, course2, dist;
    SGGeodesy::inverse( ref, p, course1, course2, dist );
    //printf("c1 = %.2f  c2 = %.2f  dist = %.2f\n", course1, course2, dist);
    double angle = course1 * SGD_DEGREES_TO_RADIANS;
    double x = sin(angle) * dist;
    double y = cos(angle) * dist;
    //printf(" x=%.2f y=%.2f\n", x, y);
    result[0] = x;
    result[1] = y;
    result[2] = p.getElevationM() - ref.getElevationM();
    return result;
}
#endif


// cam_pos: SGGeod representing the lon, lat, alt of camera
// roll/pitch/yaw: aircraft orientation in euler angles (degrees)
// h/v: horizontal/vertical size of camera sensor in mm
// focal_len: focal length of sensor in mm
// lookat/ll/lr/ul/ur: SGGeod represention wgs84 location of center of view and corners of the image
void geolocate_image( SGGeod cam_pos, double ground_m,
		      double roll_deg, double pitch_deg, double yaw_deg,
		      double h_mm, double v_mm, double focal_len_mm,
		      SGGeod *lookat_wgs84, 
		      SGGeod *ll_wgs84, 
		      SGGeod *lr_wgs84, 
		      SGGeod *ul_wgs84, 
		      SGGeod *ur_wgs84 )
{
    // Samsung NX210
    // h_mm = 23.5;
    // v_mm = 15.7;
    //double focal_len_mm = 30.0;

    // compute camera parameters
    double diag_fov = compute_diag_fov( h_mm, v_mm, focal_len_mm );
    double half_diag_fov = 0.5 * diag_fov;
    double diag_angle = atan2(v_mm, h_mm);
    //printf("diag_fov = %.2f\n", diag_fov * SGD_RADIANS_TO_DEGREES);
    //printf("diag_angle = %.2f\n", diag_angle * SGD_RADIANS_TO_DEGREES);

    // aircraft position (as a geod)
    // SGGeod pos_geod = SGGeod::fromDegM( lon_deg, lat_deg, alt_m );

    // aircraft orientation (as a quaternion)
    SGQuatd ned2body = compute_ned2body( roll_deg, pitch_deg, yaw_deg );

    SGVec3d vec;

    vec = compute_base_ned_vector( 0.0, 0.0 );
    SGVec3d lookat_ned = ned2body.backTransform(vec);
    *lookat_wgs84 = wgs84_from_ned( ground_m, cam_pos, lookat_ned );

    // ll = lower left, ur = upper right, etc.
    vec = compute_base_ned_vector( half_diag_fov, -SGD_PI_2 - diag_angle );
    SGVec3d ll_ned = ned2body.backTransform(vec);
    *ll_wgs84 = wgs84_from_ned( ground_m, cam_pos, ll_ned );

    vec = compute_base_ned_vector( half_diag_fov, SGD_PI_2 + diag_angle );
    SGVec3d lr_ned = ned2body.backTransform(vec);
    *lr_wgs84 = wgs84_from_ned( ground_m, cam_pos, lr_ned );

    vec = compute_base_ned_vector( half_diag_fov, SGD_PI_2 - diag_angle );
    SGVec3d ur_ned = ned2body.backTransform(vec);
    *ur_wgs84 = wgs84_from_ned( ground_m, cam_pos, ur_ned );

    vec = compute_base_ned_vector( half_diag_fov, -SGD_PI_2 + diag_angle );
    SGVec3d ul_ned = ned2body.backTransform(vec);
    *ul_wgs84 = wgs84_from_ned( ground_m, cam_pos, ul_ned );

    /*
    printf("lookat: %.10f %.10f %.3f\n",
	   lookat_wgs84.getLongitudeDeg(),
	   lookat_wgs84.getLatitudeDeg(),
	   lookat_wgs84.getElevationM());

    printf("lowerleft: %.10f %.10f %.3f\n",
	   ll_wgs84.getLongitudeDeg(),
	   ll_wgs84.getLatitudeDeg(),
	   ll_wgs84.getElevationM());

    printf("lowerright: %.10f %.10f %.3f\n",
	   lr_wgs84.getLongitudeDeg(),
	   lr_wgs84.getLatitudeDeg(),
	   lr_wgs84.getElevationM());

    printf("upperright: %.10f %.10f %.3f\n",
	   ur_wgs84.getLongitudeDeg(),
	   ur_wgs84.getLatitudeDeg(),
	   ur_wgs84.getElevationM());

    printf("upperleft: %.10f %.10f %.3f\n",
	   ul_wgs84.getLongitudeDeg(),
	   ul_wgs84.getLatitudeDeg(),
	   ul_wgs84.getElevationM());
    */
}


