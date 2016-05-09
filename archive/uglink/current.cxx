#include <math.h>
#include <stdio.h>

#include "python/pyprops.hxx"
#include "include/globaldefs.h"

#include "current.hxx"
#include "geo.hxx"


#define HAVE_AIRDATA_SPEED 1
#define HAVE_AIRDATA_ALTITUDE 1
#define HAVE_AIRDATA_CLIMB 1
#define HAVE_AIRDATA_WIND 1


static bool props_inited = false;
static SGPropertyNode *use_ground_speed_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;
static SGPropertyNode *gps_satellites_node = NULL;
static SGPropertyNode *gps_status_node = NULL;

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
static SGPropertyNode *imu_temp_node = NULL;
static SGPropertyNode *imu_status_node = NULL;

// air data property nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_pressure_node = NULL;
static SGPropertyNode *airdata_temperature_node = NULL;
static SGPropertyNode *airdata_altitude_node = NULL;
static SGPropertyNode *airdata_altitude_true_node = NULL;
static SGPropertyNode *airdata_airspeed_node = NULL;
static SGPropertyNode *airdata_climb_fpm_node = NULL;
static SGPropertyNode *airdata_accel_ktps_node = NULL;
static SGPropertyNode *airdata_wind_dir_node = NULL;
static SGPropertyNode *airdata_wind_speed_node = NULL;
static SGPropertyNode *airdata_pitot_scale_node = NULL;
static SGPropertyNode *airdata_status_node = NULL;

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

// actuator property nodes
static SGPropertyNode *act_timestamp_node = NULL;
static SGPropertyNode *act_aileron_node = NULL;
static SGPropertyNode *act_elevator_node = NULL;
static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *act_rudder_node = NULL;
static SGPropertyNode *act_channel5_node = NULL;
static SGPropertyNode *act_channel6_node = NULL;
static SGPropertyNode *act_channel7_node = NULL;
static SGPropertyNode *act_channel8_node = NULL;
static SGPropertyNode *act_status_node = NULL;

// pilot input property nodes
static SGPropertyNode *pilot_timestamp_node = NULL;
static SGPropertyNode *pilot_aileron_node = NULL;
static SGPropertyNode *pilot_elevator_node = NULL;
static SGPropertyNode *pilot_throttle_node = NULL;
static SGPropertyNode *pilot_rudder_node = NULL;
static SGPropertyNode *pilot_channel5_node = NULL;
static SGPropertyNode *pilot_channel6_node = NULL;
static SGPropertyNode *pilot_channel7_node = NULL;
static SGPropertyNode *pilot_channel8_node = NULL;
static SGPropertyNode *pilot_status_node = NULL;

// autopilot status nodes
static SGPropertyNode *ap_timestamp_node = NULL;
static SGPropertyNode *ap_hdg_node = NULL;
static SGPropertyNode *ap_roll_node = NULL;
static SGPropertyNode *ap_altitude_node = NULL;
static SGPropertyNode *ap_climb_node = NULL;
static SGPropertyNode *ap_pitch_node = NULL;
static SGPropertyNode *ap_theta_dot_node = NULL;
static SGPropertyNode *ap_speed_node = NULL;
static SGPropertyNode *ap_waypoint_target_node = NULL;
static SGPropertyNode *ap_route_size_node = NULL;

// system health nodes
static SGPropertyNode *avionics_vcc_node = NULL;
static SGPropertyNode *system_load_avg_node = NULL;
static SGPropertyNode *extern_volts_node = NULL;
static SGPropertyNode *extern_cell_volts_node = NULL;
static SGPropertyNode *extern_amps_node = NULL;
static SGPropertyNode *extern_mah_node = NULL;

// payload nodes
static SGPropertyNode *payload_trigger_num_node = NULL;
static SGPropertyNode *payload_lookat_lon_node = NULL;
static SGPropertyNode *payload_lookat_lat_node = NULL;
static SGPropertyNode *payload_ll_lon_node = NULL;
static SGPropertyNode *payload_ll_lat_node = NULL;
static SGPropertyNode *payload_lr_lon_node = NULL;
static SGPropertyNode *payload_lr_lat_node = NULL;
static SGPropertyNode *payload_ul_lon_node = NULL;
static SGPropertyNode *payload_ul_lat_node = NULL;
static SGPropertyNode *payload_ur_lon_node = NULL;
static SGPropertyNode *payload_ur_lat_node = NULL;

// console link nodes
static SGPropertyNode *link_seq_num = NULL;

// derived
static SGPropertyNode *gps_track_node = NULL;
static SGPropertyNode *gps_speed_node = NULL;
static SGPropertyNode *filter_track_node = NULL;
static SGPropertyNode *filter_speed_node = NULL;
static SGPropertyNode *wind_deg_node = NULL;
static SGPropertyNode *wind_speed_node = NULL;
static SGPropertyNode *pitot_scale_node = NULL;
static SGPropertyNode *filter_climb_node = NULL;
static SGPropertyNode *flight_flying_status = NULL;
static SGPropertyNode *flight_total_timer = NULL;
static SGPropertyNode *flight_auto_timer = NULL;
static SGPropertyNode *flight_odometer = NULL;
static SGPropertyNode *ground_alt_node = NULL;


// bind gps property nodes 
static void bind_gps_nodes() {
    gps_timestamp_node = pyGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = pyGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = pyGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = pyGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = pyGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = pyGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = pyGetNode("/sensors/gps/vd-ms", true);
    gps_unix_sec_node = pyGetNode("/sensors/gps/unix-time-sec", true);
    gps_satellites_node = pyGetNode("/sensors/gps/satellites", true);
    gps_status_node = NULL;
}

// bind imu property nodes 
static void bind_imu_nodes() {
    imu_timestamp_node = pyGetNode("/sensors/imu/time-stamp", true);
    imu_p_node = pyGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = pyGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = pyGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = pyGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = pyGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = pyGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = pyGetNode("/sensors/imu/hx", true);
    imu_hy_node = pyGetNode("/sensors/imu/hy", true);
    imu_hz_node = pyGetNode("/sensors/imu/hz", true);
    imu_temp_node = pyGetNode("/sensors/imu/temp_C", true);
    imu_status_node = pyGetNode("/sensors/imu/status", true);
}

// bind air data property nodes 
static void bind_airdata_nodes() {
    airdata_timestamp_node = pyGetNode("/sensors/airdata/time-stamp", true);
    airdata_pressure_node = pyGetNode("/sensors/airdata/pressure-mbar", true);
    airdata_temperature_node = pyGetNode("/sensors/airdata/temperature-degC", true);
    airdata_altitude_node = pyGetNode("/sensors/airdata/altitude-pressure-m", true);
    airdata_altitude_true_node = pyGetNode("/position/combined/altitude-true-m", true);
    airdata_airspeed_node = pyGetNode("/sensors/airdata/airspeed-kt", true);
    airdata_climb_fpm_node
	= pyGetNode("/sensors/airdata/vertical-speed-fpm", true);
    airdata_accel_ktps_node
	= pyGetNode("/sensors/airdata/acceleration-ktps", true);
    airdata_wind_dir_node = pyGetNode("/sensors/airdata/wind-deg",true);
    airdata_wind_speed_node = pyGetNode("/sensors/airdata/wind-kts",true);
    airdata_pitot_scale_node = pyGetNode("/sensors/airdata/pitot-scale-factor",true);
    airdata_status_node = pyGetNode("/sensors/airdata/status", true);
}

// bind filter property nodes
static void bind_filter_nodes() {
    filter_timestamp_node = pyGetNode("/filters/filter/time-stamp", true);
    filter_theta_node = pyGetNode("/filters/filter/pitch-deg", true);
    filter_phi_node = pyGetNode("/filters/filter/roll-deg", true);
    filter_psi_node = pyGetNode("/filters/filter/heading-deg", true);
    filter_lat_node = pyGetNode("/filters/filter/latitude-deg", true);
    filter_lon_node = pyGetNode("/filters/filter/longitude-deg", true);
    filter_alt_node = pyGetNode("/filters/filter/altitude-m", true);
    filter_vn_node = pyGetNode("/filters/filter/vn-ms", true);
    filter_ve_node = pyGetNode("/filters/filter/ve-ms", true);
    filter_vd_node = pyGetNode("/filters/filter/vd-ms", true);
    filter_status_node = pyGetNode("/filters/filter/status", true);
}

// bind actuator property nodes
static void bind_actuator_nodes() {
    act_timestamp_node = pyGetNode("/actuators/actuator/time-stamp", true);
    act_aileron_node = pyGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = pyGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = pyGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = pyGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = pyGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = pyGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = pyGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = pyGetNode("/actuators/actuator/channel", 7, true);
    act_status_node = pyGetNode("/actuators/actuator/status", true);
}


// bind pilot input property nodes
static void bind_pilot_nodes() {
    pilot_timestamp_node = pyGetNode("/sensors/pilot/time-stamp", true);
    pilot_aileron_node = pyGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = pyGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = pyGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = pyGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = pyGetNode("/sensors/pilot/manual", true);
    pilot_channel6_node = pyGetNode("/sensors/pilot/channel", 5, true);
    pilot_channel7_node = pyGetNode("/sensors/pilot/channel", 6, true);
    pilot_channel8_node = pyGetNode("/sensors/pilot/channel", 7, true);
    pilot_status_node = pyGetNode("/sensors/pilot/status", true);
}

// bind autopilot status property nodes
static void bind_ap_nodes() {
    ap_timestamp_node = pyGetNode("/autopilot/time-stamp", true);
    ap_hdg_node = pyGetNode( "/autopilot/settings/target-heading-deg",
			     true );
    ap_roll_node = pyGetNode("/autopilot/settings/target-roll-deg", true);
    ap_altitude_node = pyGetNode( "/autopilot/settings/target-msl-ft", true );
    ap_climb_node = pyGetNode("/autopilot/internal/target-climb-rate-fps",
			      true);
    ap_pitch_node = pyGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_theta_dot_node = pyGetNode( "/autopilot/settings/target-the-dot", true );
    ap_speed_node = pyGetNode( "/autopilot/settings/target-speed-kt", true );
    ap_waypoint_target_node
	= pyGetNode( "/autopilot/route/target-waypoint-index", true );
    ap_route_size_node = pyGetNode( "/autopilot/route/size", true );
}


// derived properties
static void bind_derived_nodes() {
    gps_track_node = pyGetNode("/sensors/gps/track-deg", true);
    gps_speed_node = pyGetNode("/sensors/gps/speed-kt", true);
    filter_track_node = pyGetNode("/filters/filter/track-deg", true);
    filter_speed_node = pyGetNode("/filters/filter/speed-kt", true);
    wind_deg_node = pyGetNode("/filters/wind-deg", true);
    wind_speed_node = pyGetNode("/filters/wind-speed-kt", true);
    pitot_scale_node = pyGetNode("/filters/pitot-scale-factor", true);
    filter_climb_node = pyGetNode("/filters/climb-rate-fpm", true);

    flight_flying_status = pyGetNode("/status/in-flight", true);
    flight_total_timer = pyGetNode("/status/flight-timer-secs", true);
    flight_auto_timer = pyGetNode("/status/autopilot-timer-secs", true);
    flight_odometer = pyGetNode("/status/flight-odometer", true);
    ground_alt_node = pyGetNode("/filters/ground-alt-m", true);
}


// bind system health property nodes
static void bind_health_nodes() {
    system_load_avg_node = pyGetNode("/status/system-load-avg", true);
    avionics_vcc_node = pyGetNode("/status/input-vcc", true);
    extern_volts_node = pyGetNode("/status/extern-volts", true);
    extern_cell_volts_node = pyGetNode("/status/extern-cell-volts", true);
    extern_amps_node = pyGetNode("/status/extern-amps", true);
    extern_mah_node = pyGetNode("/status/extern-mah", true);
}


// bind payload property nodes
static void bind_payload_nodes() {
    payload_trigger_num_node = pyGetNode("/payload/camera/trigger-num", true);
    payload_lookat_lon_node = pyGetNode("/payload/camera/lookat-lon-deg", true);
    payload_lookat_lat_node = pyGetNode("/payload/camera/lookat-lat-deg", true);
    payload_ll_lon_node = pyGetNode("/payload/camera/lower-left-lon-deg", true);
    payload_ll_lat_node = pyGetNode("/payload/camera/lower-left-lat-deg", true);
    payload_lr_lon_node = pyGetNode("/payload/camera/lower-right-lon-deg", true);
    payload_lr_lat_node = pyGetNode("/payload/camera/lower-right-lat-deg", true);
    payload_ul_lon_node = pyGetNode("/payload/camera/upper-left-lon-deg", true);
    payload_ul_lat_node = pyGetNode("/payload/camera/upper-left-lat-deg", true);
    payload_ur_lon_node = pyGetNode("/payload/camera/upper-right-lon-deg", true);
    payload_ur_lat_node = pyGetNode("/payload/camera/upper-right-lat-deg", true);
}


static void bind_props() {
    bind_gps_nodes();
    bind_imu_nodes();
    bind_airdata_nodes();
    bind_filter_nodes();
    bind_actuator_nodes();
    bind_pilot_nodes();
    bind_ap_nodes();
    bind_health_nodes();
    bind_payload_nodes();
    bind_derived_nodes();

    // command sequence node
    link_seq_num = pyGetNode("/comms/remote-link/sequence-num", true);
    use_ground_speed_node = pyGetNode("/config/use-ground-speed", true);
}


// -----------------------


// update gps property nodes 
static void update_gps_nodes( struct gps *gpspacket ) {
    gps_timestamp_node->setDouble( gpspacket->timestamp );
    gps_lat_node->setDouble( gpspacket->lat );
    gps_lon_node->setDouble( gpspacket->lon );
    gps_alt_node->setDouble( gpspacket->alt );
    gps_ve_node->setDouble( gpspacket->vn );
    gps_vn_node->setDouble( gpspacket->ve );
    gps_vd_node->setDouble( gpspacket->vd );
    gps_unix_sec_node->setDouble( gpspacket->gps_time );
    gps_satellites_node->setIntValue( gpspacket->satellites );
}

// update imu property nodes 
static void update_imu_nodes( struct imu *imupacket ) {
    imu_timestamp_node->setDouble( imupacket->timestamp );
    imu_p_node->setDouble( imupacket->p );
    imu_q_node->setDouble( imupacket->q );
    imu_r_node->setDouble( imupacket->r );
    imu_ax_node->setDouble( imupacket->ax );
    imu_ay_node->setDouble( imupacket->ay );
    imu_az_node->setDouble( imupacket->az );
    imu_hx_node->setDouble( imupacket->hx );
    imu_hy_node->setDouble( imupacket->hy );
    imu_hz_node->setDouble( imupacket->hz );
    imu_temp_node->setDouble( imupacket->temp );
    imu_status_node->setIntValue( imupacket->status );
}

// update air data property nodes 
static void update_airdata_nodes( struct airdata *airpacket ) {
    airdata_timestamp_node->setDouble( airpacket->timestamp );
    airdata_pressure_node->setFloatValue( airpacket->pressure );
    airdata_temperature_node->setFloatValue( airpacket->temperature );
    airdata_altitude_node->setFloatValue( airpacket->altitude );
    airdata_altitude_true_node->setFloatValue( airpacket->altitude_true );
    airdata_airspeed_node->setFloatValue( airpacket->airspeed );
    airdata_climb_fpm_node->setFloatValue( airpacket->climb_fpm );
    airdata_accel_ktps_node->setFloatValue( airpacket->acceleration );
    airdata_wind_dir_node->setFloatValue( airpacket->wind_dir );
    airdata_wind_speed_node->setFloatValue( airpacket->wind_speed );
    airdata_pitot_scale_node->setFloatValue( airpacket->pitot_scale );
    airdata_status_node->setIntValue( airpacket->status );
}

// update filter property nodes
static void update_filter_nodes( struct filter *filterpacket ) {
    filter_timestamp_node->setDouble( filterpacket->timestamp );
    filter_lat_node->setDouble( filterpacket->lat );
    filter_lon_node->setDouble( filterpacket->lon );
    filter_alt_node->setDouble( filterpacket->alt );
    filter_vn_node->setDouble( filterpacket->vn );
    filter_ve_node->setDouble( filterpacket->ve );
    filter_vd_node->setDouble( filterpacket->vd );
    filter_phi_node->setDouble( filterpacket->phi );
    filter_theta_node->setDouble( filterpacket->theta );
    filter_psi_node->setDouble( filterpacket->psi );
    filter_status_node->setIntValue( filterpacket->status );
}

// update actuator property nodes
static void update_actuator_nodes( struct actuator *actpacket ) {
    act_timestamp_node->setDouble( actpacket->timestamp );
    act_aileron_node->setFloatValue( actpacket->ail );
    act_elevator_node->setFloatValue( actpacket->ele );
    act_throttle_node->setFloatValue( actpacket->thr );
    act_rudder_node->setFloatValue( actpacket->rud );
    act_channel5_node->setFloatValue( actpacket->ch5 );
    act_channel6_node->setFloatValue( actpacket->ch6 );
    act_channel7_node->setFloatValue( actpacket->ch7 );
    act_channel8_node->setFloatValue( actpacket->ch8 );
    act_status_node->setIntValue( actpacket->status );
}


// update pilot input property nodes
static void update_pilot_nodes( struct pilot *pilotpacket ) {
    pilot_timestamp_node->setDouble( pilotpacket->timestamp );
    pilot_aileron_node->setFloatValue( pilotpacket->ail );
    pilot_elevator_node->setFloatValue( pilotpacket->ele );
    pilot_throttle_node->setFloatValue( pilotpacket->thr );
    pilot_rudder_node->setFloatValue( pilotpacket->rud );
    pilot_channel5_node->setFloatValue( pilotpacket->ch5 );
    pilot_channel6_node->setFloatValue( pilotpacket->ch6 );
    pilot_channel7_node->setFloatValue( pilotpacket->ch7 );
    pilot_channel8_node->setFloatValue( pilotpacket->ch8 );
    pilot_status_node->setIntValue( pilotpacket->status );
}


// update autopilot status property nodes
static void update_ap_nodes( struct apstatus *appacket ) {
    ap_timestamp_node->setDouble( appacket->timestamp );
    ap_hdg_node->setFloatValue( appacket->target_heading_deg );
    ap_roll_node->setFloatValue( appacket->target_roll_deg );
    ap_altitude_node->setFloatValue( appacket->target_altitude_msl_ft );
    ap_climb_node->setFloatValue( appacket->target_climb_fps );
    ap_pitch_node->setFloatValue( appacket->target_pitch_deg );
    ap_theta_dot_node->setFloatValue( appacket->target_theta_dot );
    ap_speed_node->setFloatValue( appacket->target_speed_kt );
    ap_waypoint_target_node->setIntValue( appacket->target_wp );
    ap_route_size_node->setIntValue( appacket->route_size );
 
    // build/update the route
    SGPropertyNode *route_node = pyGetNode("/autopilot/route", true);
    if ( appacket->wp_index < 65535 ) {
	SGPropertyNode *wpt_node
	    = route_node->getChild("wpt", appacket->wp_index, true);
	SGPropertyNode *lon_node = wpt_node->getChild("lon-deg", 0, true);
	SGPropertyNode *lat_node = wpt_node->getChild("lat-deg", 0, true);
	lon_node->setDouble( appacket->wp_lon );
	lat_node->setDouble( appacket->wp_lat );
    } else {
	SGPropertyNode *wpt_node
	    = route_node->getChild("home", 0, true);
	SGPropertyNode *lon_node = wpt_node->getChild("lon-deg", 0, true);
	SGPropertyNode *lat_node = wpt_node->getChild("lat-deg", 0, true);
	lon_node->setDouble( appacket->wp_lon );
	lat_node->setDouble( appacket->wp_lat );
     }
}


// update system health property nodes
static void update_health_nodes( struct health *healthpacket ) {
    system_load_avg_node->setFloatValue( healthpacket->load_avg );
    avionics_vcc_node->setFloatValue( healthpacket->avionics_vcc );
    extern_volts_node->setFloatValue( healthpacket->extern_volts );
    extern_cell_volts_node->setFloatValue( healthpacket->extern_cell_volts );
    extern_amps_node->setFloatValue( healthpacket->extern_amps );
    extern_mah_node->setFloatValue( healthpacket->extern_mah );
}


// update payload property nodes
static void update_payload_nodes( struct payload *payloadpacket ) {
    payload_trigger_num_node->setIntValue( payloadpacket->trigger_num );
}


void update_props( struct gps *gpspacket,
		   struct imu *imupacket,
		   struct airdata *airpacket,
		   struct filter *filterpacket,
		   struct actuator *actpacket,
		   struct pilot *pilotpacket,
		   struct apstatus *appacket,
		   struct health *healthpacket,
		   struct payload *payloadpacket )
{
    if ( ! props_inited ) {
	props_inited = true;
	bind_props();
    }

    update_gps_nodes( gpspacket );
    update_imu_nodes( imupacket );
    update_airdata_nodes( airpacket );
    update_filter_nodes( filterpacket );
    update_actuator_nodes( actpacket );
    update_pilot_nodes( pilotpacket );
    update_ap_nodes( appacket );
    update_health_nodes( healthpacket );
    update_payload_nodes( payloadpacket );
}


void compute_derived_data( struct gps *gpspacket,
			   struct imu *imupacket,
			   struct airdata *airpacket,
			   struct filter *filterpacket,
			   struct actuator *actpacket,
			   struct pilot *pilotpacket,
			   struct apstatus *appacket,
			   struct health *healthpacket,
			   struct payload *payloadpacket )
{
    // static double last_time = 0.0;

    // compute filtered speed, pressure altitude, and pressure
    // difference based rate of climb.
    // double dt = airpacket->timestamp - last_time;

    if ( ! props_inited ) {
	props_inited = true;
	bind_props();
    }

    // compute in-flight status
    bool in_flight = false;
#ifdef HAVE_AIRDATA_SPEED
    if ( airpacket->airspeed > 15 ) { 
	in_flight = true;
    }
#else
    if ( filter_speed_node->getDouble() > 15 ) {
	in_flight = true;
    }
#endif
    flight_flying_status->setDouble( in_flight );

    // update timers
    static double last_time_sec = filterpacket->timestamp;
    double dt = filterpacket->timestamp - last_time_sec;
    last_time_sec = filterpacket->timestamp;

    // flight timer
    if ( in_flight ) {
	double timer = flight_total_timer->getDouble();
	timer += dt;
	flight_total_timer->setDouble( timer );
    }

    // autopilot timer
    if ( in_flight && pilot_channel5_node->getDouble() < 0.5 ) {
	double timer = flight_auto_timer->getDouble();
	timer += dt;
	flight_auto_timer->setDouble( timer );

    }

    // estimate distance traveled from filter velocity and dt
    if ( in_flight ) {
	double vel_ms = filter_speed_node->getDouble() * SG_KT_TO_MPS;
	double od = flight_odometer->getDouble();
	od += vel_ms * dt;
	flight_odometer->setDouble( od );
    }

    // estimate ground altitude (average altitude when not flying
    static double ground_alt_filter = -9999.9;
    if ( ! in_flight && filter_timestamp_node->getDouble() > 0.0 ) {
	double alt = filter_alt_node->getDouble();
	if ( ground_alt_filter < -1000 ) {
	    ground_alt_filter = alt;
	} else {
	    ground_alt_filter = 0.999 * ground_alt_filter + 0.001 * alt;
	}
	ground_alt_node->setDouble(ground_alt_filter);
	// printf("(%.4f)ground alt = %.2f\n", filter_timestamp_node->getDouble(), ground_alt_filter);
    }

    // compute ground track heading/speed
    double vn, ve, vd, hdg, speed;
    vn = filterpacket->vn;
    ve = filterpacket->ve;
    vd = filterpacket->vd;
    hdg = (SGD_PI * 0.5 - atan2(vn, ve)) * SG_RADIANS_TO_DEGREES;
    speed = sqrt( vn*vn + ve*ve + vd*vd ) * SG_MPS_TO_KT;
    filter_track_node->setDouble( hdg );
    filter_speed_node->setDouble( speed );

    // smooth vertical speed
    static double filter_climb = 0.0;
    double climb = 0.0;
# ifdef HAVE_AIRDATA_CLIMB
    climb = airpacket->climb_fpm;
# else
    climb = -vd * SG_METER_TO_FEET * 60.0;
# endif
    filter_climb = 0.99 * filter_climb + 0.01 * climb;
    filter_climb_node->setDouble( filter_climb );

    // Estimate wind direction and speed based on ground track speed
    // versus aircraft heading and indicated airspeed.
#ifdef HAVE_AIRDATA_WIND
    wind_deg_node->setDouble( airpacket->wind_dir );
    wind_speed_node->setDouble( airpacket->wind_speed );
    pitot_scale_node->setDouble( airpacket->pitot_scale );
#else
    // only estimate wind/pitot scale if in flight (that's the only
    // time the assumptions that go into this estimator are valid.)
    if ( in_flight ) {
	static double pitot_scale_filt = 1.0;
	double psi = SGD_PI_2 - filterpacket->psi * SG_DEGREES_TO_RADIANS;
	double ue = cos(psi) * (airpacket->airspeed * pitot_scale_filt * SG_KT_TO_MPS);
	double un = sin(psi) * (airpacket->airspeed * pitot_scale_filt * SG_KT_TO_MPS);
	double we = ue - ve;
	double wn = un - vn;

	static double filt_we = 0.0, filt_wn = 0.0;
	filt_we = 0.9995 * filt_we + 0.0005 * we;
	filt_wn = 0.9995 * filt_wn + 0.0005 * wn;
	    
	double wind_deg = 90 - atan2( filt_wn, filt_we ) * SGD_RADIANS_TO_DEGREES;
	if ( wind_deg < 0 ) { wind_deg += 360.0; }
	wind_deg_node->setDouble( wind_deg );
	wind_speed_node->setDouble(
	        sqrt( filt_we*filt_we + filt_wn*filt_wn ) * SG_MPS_TO_KT
	);

	// estimate pitot tube scale error
	double true_e = filt_we + ve;
	double true_n = filt_wn + vn;

	double true_deg = 90 - atan2( true_n, true_e ) * SGD_RADIANS_TO_DEGREES;
	if ( true_deg < 0 ) { true_deg += 360.0; }
	double true_speed_kt = sqrt( true_e*true_e + true_n*true_n )
	    * SG_MPS_TO_KT;

	double pitot_scale = 1.0;
	if ( airpacket->airspeed > 1.0 ) {
	    pitot_scale = true_speed_kt / airpacket->airspeed;
	    if ( pitot_scale < 0.25 ) { pitot_scale = 0.25;	}
	    if ( pitot_scale > 4.00 ) { pitot_scale = 4.00; }
	}

	pitot_scale_filt = 0.999 * pitot_scale_filt + 0.001 * pitot_scale;
	pitot_scale_node->setDouble( pitot_scale_filt );
	//printf("pitot_scale_filt = %.2f\n", pitot_scale_filt);
	printf("%.2f %.1f %.1f\n", filterpacket->timestamp, airpacket->airspeed, true_speed_kt);
    }
#endif

    // temporary experiment: open loop compute heading from yaw gyro
    // only, starting with filter heading as soon as auto-mode
    // (vs. manual) mode is set.  Then see how close/well this tracks.

    bool auto_mode = (pilot_channel8_node->getIntValue() < 0);
    static bool last_auto_mode = auto_mode;

    bool throttle = (pilot_throttle_node->getDouble() > 0.5);
    static bool last_throttle = throttle;

    //double timestamp =  filter_timestamp_node->getDouble();
    //static double last_timestamp = timestamp;
    //double dt = timestamp - last_timestamp;
    static double ol_roll = filter_psi_node->getDouble();
    static double ol_pitch = filter_psi_node->getDouble();
    static double ol_yaw = filter_psi_node->getDouble();

    if ( !last_auto_mode && auto_mode ) {
	ol_roll = filter_phi_node->getDouble();
	ol_pitch = filter_theta_node->getDouble();
	ol_yaw = filter_psi_node->getDouble();
    }
    last_auto_mode = auto_mode;

    if ( !last_throttle && throttle ) {
	ol_roll = filter_phi_node->getDouble();
	ol_pitch = filter_theta_node->getDouble();
	ol_yaw = filter_psi_node->getDouble();
    }
    last_throttle = throttle;

    if ( dt > 0.0 ) {
	double dp
	    = (imu_p_node->getDouble() * dt) * SGD_RADIANS_TO_DEGREES;
	double dq
	    = (imu_q_node->getDouble() * dt) * SGD_RADIANS_TO_DEGREES;
	double dr
	    = (imu_r_node->getDouble() * dt) * SGD_RADIANS_TO_DEGREES;
	//printf("dr=%.3f imu_r=%.3f dt=%.3f\n
	ol_roll += dp;
	if ( ol_roll < -180.0 ) { ol_roll += 360.0; }
	if ( ol_roll > 180.0 ) { ol_roll -= 360.0; }
	ol_pitch += dq;
	if ( ol_pitch < -180.0 ) { ol_pitch += 360.0; }
	if ( ol_pitch > 180.0 ) { ol_pitch -= 360.0; }
	ol_yaw += dr;
	if ( ol_yaw < 0.0 ) { ol_yaw += 360.0; }
	if ( ol_yaw > 360.0 ) { ol_yaw -= 360.0; }
    }
    /*if ( throttle ) {
	printf("OL %.3f %.3f %.3f %.3f\n",
	       filter_timestamp_node->getDouble(),
	       filter_theta_node->getDouble(),
	       ol_pitch,
	       imu_q_node->getDouble() * SGD_RADIANS_TO_DEGREES);
    }*/
    /*if ( in_flight ) {
	// dump out throttle position vs airspeed vs climb
	printf("SI %.2f %.1f %.0f\n", 
	       act_throttle_node->getDouble(),
	       airdata_airspeed_node->getDouble(),
	       airdata_climb_fpm_node->getDouble());
    }*/

    // Compute camera target/coverage for each new shot

    // hard coded values for samsung NX210 camera
    const double h_mm = 23.7;
    const double v_mm = 15.7;
    const double focal_len_mm = 30.0;

    static int last_trigger_num = 0;

    if ( payloadpacket->trigger_num != last_trigger_num ) {
	last_trigger_num = payloadpacket->trigger_num;

	// printf("alt = %.2f\n", filter_alt_node->getDouble());
	SGGeod cam_pos = SGGeod::fromDegM(filter_lon_node->getDouble(),
					  filter_lat_node->getDouble(),
					  filter_alt_node->getDouble());
	SGGeod lookat_wgs84, ll_wgs84, lr_wgs84, ul_wgs84, ur_wgs84;
 
	geolocate_image( cam_pos, ground_alt_node->getDouble(),
			 filter_phi_node->getDouble(),
			 filter_theta_node->getDouble(),
			 filter_psi_node->getDouble(),
			 h_mm, v_mm, focal_len_mm,
			 &lookat_wgs84,
			 &ll_wgs84, &lr_wgs84, &ul_wgs84, &ur_wgs84 );
	// printf("Camera shot @ %.8f %.8f\n", lookat_wgs84.getLongitudeDeg(),
	//        lookat_wgs84.getLatitudeDeg());
	payload_lookat_lon_node->setDouble(lookat_wgs84.getLongitudeDeg());
	payload_lookat_lat_node->setDouble(lookat_wgs84.getLatitudeDeg());
	payload_ll_lon_node->setDouble( ll_wgs84.getLongitudeDeg() );
	payload_ll_lat_node->setDouble( ll_wgs84.getLatitudeDeg() );
	payload_lr_lon_node->setDouble( lr_wgs84.getLongitudeDeg() );
	payload_lr_lat_node->setDouble( lr_wgs84.getLatitudeDeg() );
	payload_ul_lon_node->setDouble( ul_wgs84.getLongitudeDeg() );
	payload_ul_lat_node->setDouble( ul_wgs84.getLatitudeDeg() );
	payload_ur_lon_node->setDouble( ur_wgs84.getLongitudeDeg() );
	payload_ur_lat_node->setDouble( ur_wgs84.getLatitudeDeg() );
    }
}


string current_get_fcs_nav_string() {
    static int max_buf = 256;
    char buf[max_buf];

    double filter_hdg = (SGD_PI * 0.5 - atan2(filter_vn_node->getDouble(), filter_ve_node->getDouble())) * SG_RADIANS_TO_DEGREES;
    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.1f,%.1f,%.2f",
	     imu_timestamp_node->getDouble(),
	     ap_hdg_node->getDouble(),
	     ap_roll_node->getDouble(),
	     filter_hdg,
	     filter_phi_node->getDouble(),
	     act_aileron_node->getDouble());

    return buf;
}

string current_get_fcs_speed_string() {
    static int max_buf = 256;
    char buf[max_buf];

    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.1f,%.1f,%.2f",
	     imu_timestamp_node->getDouble(),
	     ap_speed_node->getDouble(),
	     ap_pitch_node->getDouble(),
	     airdata_airspeed_node->getDouble(),
	     filter_theta_node->getDouble(),
	     act_elevator_node->getDouble());

    return buf;
}

string current_get_fcs_altitude_string() {
    static int max_buf = 256;
    char buf[max_buf];

    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.2f",
	     imu_timestamp_node->getDouble(),
	     ap_altitude_node->getDouble(),
	     // filter_alt_node->getDouble(),
	     airdata_altitude_true_node->getDouble()*SG_METER_TO_FEET,
	     act_throttle_node->getDouble() );

    return buf;
}
