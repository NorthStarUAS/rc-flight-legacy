#include "python/pyprops.hxx"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "include/globaldefs.h"
#include "filters/nav_common/nav_functions_float.hxx"

#include "airshow.hxx"

static pyPropertyNode orient_node;
static pyPropertyNode targets_node;
static pyPropertyNode locks_node;


void airshow_init() {
    orient_node = pyGetNode("/orientation", true);
    targets_node = pyGetNode("/autopilot/targets", true);
    locks_node = pyGetNode("/autopilot/locks", true);
    locks_node.setBool("traditional", true);
}

void airshow_update() {
    // quick mode management hack
    locks_node.setBool("airshow", !locks_node.getBool("traditional") );
    
    // aircraft attitude
    float phi = orient_node.getDouble("roll_deg")*SGD_DEGREES_TO_RADIANS;
    float the = orient_node.getDouble("pitch_deg")*SGD_DEGREES_TO_RADIANS;
    float psi = orient_node.getDouble("heading_deg")*SGD_DEGREES_TO_RADIANS;
    Quaternionf q_N2B = eul2quat(phi, the, psi);

    // target attitude
    float target_the = targets_node.getDouble("pitch_deg")*SGD_DEGREES_TO_RADIANS;
    float heading_error = targets_node.getDouble("course_error_deg")*SGD_DEGREES_TO_RADIANS;
    if ( heading_error < -0.7 ) { heading_error = -0.7; }
    if ( heading_error >  0.7 ) { heading_error =  0.7; }
    printf("hdg err = %.2f\n", heading_error);
    float target_psi = psi + heading_error;

    // target ned vector
    Quaternionf q_tgt = eul2quat(0, target_the, target_psi);
    Vector3f v(1.0, 0.0, 0.0);
    Vector3f target_ned = q_tgt * v;
    printf("target ned: %.1f %.1f %.1f\n", target_ned[0], target_ned[1], target_ned[2]);

    // target in body frame
    Vector3f target_body = q_N2B.inverse() * target_ned;
    // printf("target body: %.1f %.1f %.1f\n", target_body[0], target_body[1], target_body[2]);
    float psi_error = atan2(target_body[1], target_body[0]);
    float psi_mag = sqrt(target_body[0]*target_body[0]
                         + target_body[1]*target_body[1]);
    float the_error = atan2(-target_body[2], psi_mag);
    
    printf("body error: %.1f %.1f\n",
           psi_error*SGD_RADIANS_TO_DEGREES,
           the_error*SGD_RADIANS_TO_DEGREES);
    targets_node.setDouble("body_pitch_error_deg", the_error*SGD_RADIANS_TO_DEGREES);
    targets_node.setDouble("body_yaw_error_deg", psi_error*SGD_RADIANS_TO_DEGREES);
}
