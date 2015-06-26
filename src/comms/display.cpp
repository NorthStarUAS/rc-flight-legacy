#include <sys/types.h>		// opendir() mkdir()
#include <dirent.h>		// opendir()
#include <stdio.h>		// sscanf()
#include <stdlib.h>		// random()
#include <string.h>		// strncmp()
#include <sys/stat.h>		// mkdir()
#include <sys/time.h>
#include <zlib.h>

#include "include/globaldefs.h"

#include "props/props.hxx"
#include "sensors/gps_mgr.hxx"
#include "util/timing.h"

#include "display.h"

bool display_on = false;   // dump summary to display periodically

// imu property nodes
static bool props_inited = false;

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

// air data nodes
static SGPropertyNode *airdata_altitude_node = NULL;
static SGPropertyNode *airdata_airspeed_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

// filter output nodes
static SGPropertyNode *filter_theta_node = NULL;
static SGPropertyNode *filter_phi_node = NULL;
static SGPropertyNode *filter_psi_node = NULL;
static SGPropertyNode *filter_status_node = NULL;
static SGPropertyNode *filter_lat_node = NULL;
static SGPropertyNode *filter_lon_node = NULL;
static SGPropertyNode *filter_alt_node = NULL;
static SGPropertyNode *filter_vn_node = NULL;
static SGPropertyNode *filter_ve_node = NULL;
static SGPropertyNode *filter_vd_node = NULL;

// actuator property nodes
static SGPropertyNode *act_aileron_node = NULL;
static SGPropertyNode *act_elevator_node = NULL;
static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *act_rudder_node = NULL;
static SGPropertyNode *act_channel5_node = NULL;

// pilot property nodes
static SGPropertyNode *pilot_aileron_node = NULL;
static SGPropertyNode *pilot_elevator_node = NULL;
static SGPropertyNode *pilot_throttle_node = NULL;
static SGPropertyNode *pilot_rudder_node = NULL;
static SGPropertyNode *pilot_channel5_node = NULL;

// health/status nodes
static SGPropertyNode *link_seq_num = NULL;
static SGPropertyNode *target_waypoint = NULL;
static SGPropertyNode *system_loadavg_node = NULL;
static SGPropertyNode *input_vcc_node = NULL;


static void init_props() {
    props_inited = true;

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

    // initialize air data nodes
    airdata_altitude_node = fgGetNode("/position/pressure/altitude-m", true);
    airdata_airspeed_node = fgGetNode("/sensors/airdata/airspeed-kt", true);

    // initialize gps property nodes
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);

    // initialize filter property nodes 
    filter_theta_node = fgGetNode("/orientation/pitch-deg", true);
    filter_phi_node = fgGetNode("/orientation/roll-deg", true);
    filter_psi_node = fgGetNode("/orientation/heading-deg", true);
    filter_lat_node = fgGetNode("/position/latitude-deg", true);
    filter_lon_node = fgGetNode("/position/longitude-deg", true);
    filter_alt_node = fgGetNode("/position/altitude-m", true);
    filter_vn_node = fgGetNode("/velocity/vn-ms", true);
    filter_ve_node = fgGetNode("/velocity/ve-ms", true);
    filter_vd_node = fgGetNode("/velocity/vd-ms", true);
    filter_status_node = fgGetNode("/health/navigation", true);

    // initialize actuator property nodes
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);

    // initialize pilot property nodes
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = fgGetNode("/sensors/pilot/manual", true);

    // initialize health/status property nodes
    link_seq_num = fgGetNode("/comms/remote-link/sequence-num", true);
    target_waypoint = fgGetNode( "/task/route/target-waypoint-idx",
				 true );
    system_loadavg_node = fgGetNode("/status/system-load-avg", true);
    input_vcc_node = fgGetNode("/sensors/APM2/board-vcc", true);
}


// periodic console summary of attitude/location estimate
void display_message()
{
    if ( !props_inited ) {
	init_props();
    }

    printf("[m/s^2]:ax  = %6.3f ay  = %6.3f az  = %6.3f \n",
	   imu_ax_node->getDoubleValue(),
	   imu_ay_node->getDoubleValue(),
	   imu_az_node->getDoubleValue());
    printf("[deg/s]:p   = %6.3f q   = %6.3f r   = %6.3f \n",
	   imu_p_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES,
	   imu_q_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES,
	   imu_r_node->getDoubleValue() * SGD_RADIANS_TO_DEGREES);
    printf("[Gauss]:hx  = %6.3f hy  = %6.3f hz  = %6.3f \n",
	   imu_hx_node->getDoubleValue(),
	   imu_hy_node->getDoubleValue(),
	   imu_hz_node->getDoubleValue());
    printf("[deg  ]:phi = %6.2f the = %6.2f psi = %6.2f \n",
	   filter_phi_node->getDoubleValue(),
	   filter_theta_node->getDoubleValue(),
	   filter_psi_node->getDoubleValue());
    printf("[     ]:Palt  = %6.3f Pspd  = %6.3f             \n",
	   airdata_altitude_node->getDoubleValue(),
	   airdata_airspeed_node->getDoubleValue());
#if 0
    // gyro bias from mnav filter
    printf("[deg/s]:bp  = %6.3f,bq  = %6.3f,br  = %6.3f \n",
	   xs[4] * SGD_RADIANS_TO_DEGREES,
	   xs[5] * SGD_RADIANS_TO_DEGREES,
	   xs[6] * SGD_RADIANS_TO_DEGREES);
#endif

    if ( GPS_age() < 10.0 ) {
	time_t current_time = gps_unix_sec_node->getIntValue();
	double remainder = gps_unix_sec_node->getDoubleValue() - current_time;
	struct tm *date = gmtime(&current_time);
        printf("[GPS  ]:date = %04d/%02d/%02d %02d:%02d:%05.2f\n",
	       date->tm_year + 1900, date->tm_mon + 1, date->tm_mday,
	       date->tm_hour, date->tm_min, date->tm_sec + remainder);
        printf("[GPS  ]:lon = %f[deg], lat = %f[deg], alt = %f[m], age = %.2f\n",
	       gps_lon_node->getDoubleValue(), gps_lat_node->getDoubleValue(),
	       gps_alt_node->getDoubleValue(), GPS_age());
    } else {
	printf("[GPS  ]:[%0f seconds old]\n", GPS_age());
    }

    if ( strcmp( filter_status_node->getStringValue(), "valid" ) == 0 ) {
        printf("[filter]:lon = %f[deg], lat = %f[deg], alt = %f[m]\n",
	       filter_lon_node->getDoubleValue(),
	       filter_lat_node->getDoubleValue(),
	       filter_alt_node->getDoubleValue());	
    } else {
	printf("[filter]:[No Valid Data]\n");
    }

    printf("[act  ]: %.2f %.2f %.2f %.2f %.2f\n",
	   act_aileron_node->getDoubleValue(),
	   act_elevator_node->getDoubleValue(),
	   act_throttle_node->getDoubleValue(),
	   act_rudder_node->getDoubleValue(),
	   act_channel5_node->getDoubleValue());
    printf("[health]: cmdseq = %d  tgtwp = %d  loadavg = %.2f  vcc = %.2f\n",
           link_seq_num->getIntValue(), target_waypoint->getIntValue(),
           system_loadavg_node->getFloatValue(),
	   input_vcc_node->getFloatValue());
    printf("\n");

    // printf("imu size = %d\n", sizeof( struct imu ) );
}
