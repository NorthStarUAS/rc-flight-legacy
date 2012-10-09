#include <math.h>
#include <stdio.h>

#include "props/props.hxx"
#include "include/globaldefs.h"

#include "current.hxx"


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
static SGPropertyNode *imu_status_node = NULL;

// air data property nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_altitude_node = NULL;
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
static SGPropertyNode *ap_speed_node = NULL;
static SGPropertyNode *ap_waypoint_target_node = NULL;
static SGPropertyNode *ap_route_size_node = NULL;

// system health nodes
static SGPropertyNode *system_loadavg_node = NULL;
static SGPropertyNode *input_vcc_node = NULL;

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
static SGPropertyNode *flight_motor_timer = NULL;
static SGPropertyNode *flight_odometer = NULL;


// bind gps property nodes 
static void bind_gps_nodes() {
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);
    gps_satellites_node = fgGetNode("/sensors/gps/satellites", true);
    gps_status_node = NULL;
}

// bind imu property nodes 
static void bind_imu_nodes() {
    imu_timestamp_node = fgGetNode("/sensors/imu/time-stamp", true);
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);
    imu_status_node = fgGetNode("/sensors/imu/status", true);
}

// bind air data property nodes 
static void bind_airdata_nodes() {
    airdata_timestamp_node = fgGetNode("/sensors/air-data/time-stamp", true);
    airdata_altitude_node = fgGetNode("/sensors/air-data/altitude-pressure-m", true);
    airdata_airspeed_node = fgGetNode("/sensors/air-data/airspeed-kt", true);
    airdata_climb_fpm_node
	= fgGetNode("/sensors/air-data/vertical-speed-fpm", true);
    airdata_accel_ktps_node
	= fgGetNode("/sensors/air-data/acceleration-ktps", true);
    airdata_wind_dir_node = fgGetNode("/sensors/air-data/wind-deg",true);
    airdata_wind_speed_node = fgGetNode("/sensors/air-data/wind-kts",true);
    airdata_pitot_scale_node = fgGetNode("/sensors/air-data/pitot-scale-factor",true);
    airdata_status_node = fgGetNode("/sensors/air-data/status", true);
}

// bind filter property nodes
static void bind_filter_nodes() {
    filter_timestamp_node = fgGetNode("/filters/filter/time-stamp", true);
    filter_theta_node = fgGetNode("/filters/filter/pitch-deg", true);
    filter_phi_node = fgGetNode("/filters/filter/roll-deg", true);
    filter_psi_node = fgGetNode("/filters/filter/heading-deg", true);
    filter_lat_node = fgGetNode("/filters/filter/latitude-deg", true);
    filter_lon_node = fgGetNode("/filters/filter/longitude-deg", true);
    filter_alt_node = fgGetNode("/filters/filter/altitude-m", true);
    filter_vn_node = fgGetNode("/filters/filter/vn-ms", true);
    filter_ve_node = fgGetNode("/filters/filter/ve-ms", true);
    filter_vd_node = fgGetNode("/filters/filter/vd-ms", true);
    filter_status_node = fgGetNode("/filters/filter/status", true);
}

// bind actuator property nodes
static void bind_actuator_nodes() {
    act_timestamp_node = fgGetNode("/actuators/actuator/time-stamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);
    act_status_node = fgGetNode("/actuators/actuator/status", true);
}


// bind pilot input property nodes
static void bind_pilot_nodes() {
    pilot_timestamp_node = fgGetNode("/sensors/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = fgGetNode("/sensors/pilot/manual", true);
    pilot_channel6_node = fgGetNode("/sensors/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/sensors/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/sensors/pilot/channel", 7, true);
    pilot_status_node = fgGetNode("/sensors/pilot/status", true);
}

// bind autopilot status property nodes
static void bind_ap_nodes() {
    ap_timestamp_node = fgGetNode("/autopilot/time-stamp", true);
    ap_hdg_node = fgGetNode( "/autopilot/settings/target-heading-deg",
			     true );
    ap_roll_node = fgGetNode("/autopilot/settings/target-roll-deg", true);
    ap_altitude_node = fgGetNode( "/autopilot/settings/target-msl-ft", true );
    ap_climb_node = fgGetNode("/autopilot/internal/target-climb-rate-fps",
			      true);
    ap_pitch_node = fgGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_speed_node = fgGetNode( "/autopilot/settings/target-speed-kt", true );
    ap_waypoint_target_node
	= fgGetNode( "/autopilot/route/target-waypoint-index", true );
    ap_route_size_node = fgGetNode( "/autopilot/route/size", true );
}


// derived properties
static void bind_derived_nodes() {
    gps_track_node = fgGetNode("/sensors/gps/track-deg", true);
    gps_speed_node = fgGetNode("/sensors/gps/speed-kt", true);
    filter_track_node = fgGetNode("/filters/filter/track-deg", true);
    filter_speed_node = fgGetNode("/filters/filter/speed-kt", true);
    wind_deg_node = fgGetNode("/filters/wind-deg", true);
    wind_speed_node = fgGetNode("/filters/wind-speed-kt", true);
    pitot_scale_node = fgGetNode("/filters/pitot-scale-factor", true);
    filter_climb_node = fgGetNode("/filters/climb-rate-fpm", true);

    flight_flying_status = fgGetNode("/status/in-flight", true);
    flight_total_timer = fgGetNode("/status/flight-timer-secs", true);
    flight_auto_timer = fgGetNode("/status/autopilot-timer-secs", true);
    flight_motor_timer = fgGetNode("/status/motor-run-secs", true);
    flight_odometer = fgGetNode("/status/flight-odometer", true);
}


// bind system health property nodes
static void bind_health_nodes() {
    system_loadavg_node = fgGetNode("/status/system-load-avg", true);
    input_vcc_node = fgGetNode("/status/input-vcc", true);
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
    bind_derived_nodes();

    // command sequence node
    link_seq_num = fgGetNode("/comms/remote-link/sequence-num", true);
    use_ground_speed_node = fgGetNode("/config/use-ground-speed", true);
}


// -----------------------


// update gps property nodes 
static void update_gps_nodes( struct gps *gpspacket ) {
    gps_timestamp_node->setDoubleValue( gpspacket->timestamp );
    gps_lat_node->setDoubleValue( gpspacket->lat );
    gps_lon_node->setDoubleValue( gpspacket->lon );
    gps_alt_node->setDoubleValue( gpspacket->alt );
    gps_ve_node->setDoubleValue( gpspacket->vn );
    gps_vn_node->setDoubleValue( gpspacket->ve );
    gps_vd_node->setDoubleValue( gpspacket->vd );
    gps_unix_sec_node->setDoubleValue( gpspacket->gps_time );
    gps_satellites_node->setIntValue( gpspacket->satellites );
}

// update imu property nodes 
static void update_imu_nodes( struct imu *imupacket ) {
    imu_timestamp_node->setDoubleValue( imupacket->timestamp );
    imu_p_node->setDoubleValue( imupacket->p );
    imu_q_node->setDoubleValue( imupacket->q );
    imu_r_node->setDoubleValue( imupacket->r );
    imu_ax_node->setDoubleValue( imupacket->ax );
    imu_ay_node->setDoubleValue( imupacket->ay );
    imu_az_node->setDoubleValue( imupacket->az );
    imu_hx_node->setDoubleValue( imupacket->hx );
    imu_hy_node->setDoubleValue( imupacket->hy );
    imu_hz_node->setDoubleValue( imupacket->hz );
    imu_status_node->setIntValue( imupacket->status );
}

// update air data property nodes 
static void update_airdata_nodes( struct airdata *airpacket ) {
    airdata_timestamp_node->setDoubleValue( airpacket->timestamp );
    airdata_altitude_node->setFloatValue( airpacket->altitude );
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
    filter_timestamp_node->setDoubleValue( filterpacket->timestamp );
    filter_lat_node->setDoubleValue( filterpacket->lat );
    filter_lon_node->setDoubleValue( filterpacket->lon );
    filter_alt_node->setDoubleValue( filterpacket->alt );
    filter_vn_node->setDoubleValue( filterpacket->vn );
    filter_ve_node->setDoubleValue( filterpacket->ve );
    filter_vd_node->setDoubleValue( filterpacket->vd );
    filter_phi_node->setDoubleValue( filterpacket->phi );
    filter_theta_node->setDoubleValue( filterpacket->theta );
    filter_psi_node->setDoubleValue( filterpacket->psi );
    filter_status_node->setIntValue( filterpacket->status );
}

// update actuator property nodes
static void update_actuator_nodes( struct actuator *actpacket ) {
    act_timestamp_node->setDoubleValue( actpacket->timestamp );
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
    pilot_timestamp_node->setDoubleValue( pilotpacket->timestamp );
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
    ap_timestamp_node->setDoubleValue( appacket->timestamp );
    ap_hdg_node->setFloatValue( appacket->target_heading_deg );
    ap_roll_node->setFloatValue( appacket->target_roll_deg );
    ap_altitude_node->setFloatValue( appacket->target_altitude_msl_ft );
    ap_climb_node->setFloatValue( appacket->target_climb_fps );
    ap_pitch_node->setFloatValue( appacket->target_pitch_deg );
    ap_speed_node->setFloatValue( appacket->target_speed_kt );
    ap_waypoint_target_node->setIntValue( appacket->target_wp );
    ap_route_size_node->setIntValue( appacket->route_size );
 
    // build/update the route
    SGPropertyNode *route_node = fgGetNode("/autopilot/route", true);
    if ( appacket->wp_index < 65535 ) {
	SGPropertyNode *wpt_node
	    = route_node->getChild("wpt", appacket->wp_index, true);
	SGPropertyNode *lon_node = wpt_node->getChild("lon-deg", 0, true);
	SGPropertyNode *lat_node = wpt_node->getChild("lat-deg", 0, true);
	lon_node->setDoubleValue( appacket->wp_lon );
	lat_node->setDoubleValue( appacket->wp_lat );
    } else {
	SGPropertyNode *wpt_node
	    = route_node->getChild("home", 0, true);
	SGPropertyNode *lon_node = wpt_node->getChild("lon-deg", 0, true);
	SGPropertyNode *lat_node = wpt_node->getChild("lat-deg", 0, true);
	lon_node->setDoubleValue( appacket->wp_lon );
	lat_node->setDoubleValue( appacket->wp_lat );
     }
}


// update system health property nodes
static void update_health_nodes( struct health *healthpacket ) {
  input_vcc_node->setFloatValue( healthpacket->input_vcc );
  system_loadavg_node->setFloatValue( healthpacket->loadavg );
}


void update_props( struct gps *gpspacket,
		   struct imu *imupacket,
		   struct airdata *airpacket,
		   struct filter *filterpacket,
		   struct actuator *actpacket,
		   struct pilot *pilotpacket,
		   struct apstatus *appacket,
		   struct health *healthpacket )
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
}


void compute_derived_data( struct gps *gpspacket,
			   struct imu *imupacket,
			   struct airdata *airpacket,
			   struct filter *filterpacket,
			   struct actuator *actpacket,
			   struct pilot *pilotpacket,
			   struct apstatus *appacket,
			   struct health *healthpacket )
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
    if ( airpacket->airspeed > 20 ) { 
	in_flight = true;
    }
#else
    if ( filter_speed_node->getDoubleValue() > 15 ) {
	in_flight = true;
    }
#endif
    flight_flying_status->setDoubleValue( in_flight );

    // update timers
    static double last_time_sec = filterpacket->timestamp;
    double dt = filterpacket->timestamp - last_time_sec;
    last_time_sec = filterpacket->timestamp;

    // flight timer
    if ( in_flight ) {
	double timer = flight_total_timer->getDoubleValue();
	timer += dt;
	flight_total_timer->setDoubleValue( timer );
    }

    // autopilot timer
    if ( in_flight && pilot_channel5_node->getDoubleValue() < 0.5 ) {
	double timer = flight_auto_timer->getDoubleValue();
	timer += dt;
	flight_auto_timer->setDoubleValue( timer );

    }

    // motor timer
    double motor_timer = flight_motor_timer->getDoubleValue();
    double throttle_norm = 0.0;
    if ( pilot_channel5_node->getDoubleValue() < 0.5 ) {
	// get throttle from actuator output in auto flight mode
	throttle_norm = act_throttle_node->getDoubleValue();
    } else {
	// get throttle from pilot command in manual flight mode
	throttle_norm = pilot_throttle_node->getDoubleValue();
    }
    motor_timer += ( dt * throttle_norm );
    flight_motor_timer->setDoubleValue( motor_timer );

    // estimate distance traveled from filter velocity and dt
    if ( in_flight ) {
	double vel_ms = filter_speed_node->getDoubleValue() * SG_KT_TO_MPS;
	double od = flight_odometer->getDoubleValue();
	od += vel_ms * dt;
	flight_odometer->setDoubleValue( od );
    }

    // compute ground track heading/speed
    double vn, ve, vd, hdg, speed;
    vn = filterpacket->vn;
    ve = filterpacket->ve;
    vd = filterpacket->vd;
    hdg = (SGD_PI * 0.5 - atan2(vn, ve)) * SG_RADIANS_TO_DEGREES;
    speed = sqrt( vn*vn + ve*ve + vd*vd ) * SG_MPS_TO_KT;
    filter_track_node->setDoubleValue( hdg );
    filter_speed_node->setDoubleValue( speed );

    // smooth vertical speed
    static double filter_climb = 0.0;
    double climb = 0.0;
# ifdef HAVE_AIRDATA_CLIMB
    climb = airpacket->climb_fpm;
# else
    climb = -vd * SG_METER_TO_FEET * 60.0;
# endif
    filter_climb = 0.99 * filter_climb + 0.01 * climb;
    filter_climb_node->setDoubleValue( filter_climb );

    // Estimate wind direction and speed based on ground track speed
    // versus aircraft heading and indicated airspeed.
#ifdef HAVE_AIRDATA_WIND
    wind_deg_node->setDoubleValue( airpacket->wind_dir );
    wind_speed_node->setDoubleValue( airpacket->wind_speed );
    pitot_scale_node->setDoubleValue( airpacket->pitot_scale );
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
	wind_deg_node->setDoubleValue( wind_deg );
	wind_speed_node->setDoubleValue(
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
	pitot_scale_node->setDoubleValue( pitot_scale_filt );
	//printf("pitot_scale_filt = %.2f\n", pitot_scale_filt);
    }
#endif

    // temporary experiment: open loop compute heading from yaw gyro
    // only, starting with filter heading as soon as auto-mode
    // (vs. manual) mode is set.  Then see how close/well this tracks.
    bool auto_mode = (pilot_channel8_node->getIntValue() < 0);
    static bool last_auto_mode = auto_mode;
    //double timestamp =  filter_timestamp_node->getDoubleValue();
    //static double last_timestamp = timestamp;
    //double dt = timestamp - last_timestamp;
    static double ol_hdg = filter_psi_node->getDoubleValue();
    if ( !last_auto_mode && auto_mode ) {
	ol_hdg = filter_psi_node->getDoubleValue();
    }
    last_auto_mode = auto_mode;
    if ( dt > 0.0 ) {
	double dr
	    = (imu_r_node->getDoubleValue() * dt) * SGD_RADIANS_TO_DEGREES;
	//printf("dr=%.3f imu_r=%.3f dt=%.3f\n
	ol_hdg += dr;
	if ( ol_hdg < 0.0 ) { ol_hdg += 360.0; }
	if ( ol_hdg > 360.0 ) { ol_hdg -= 360.0; }
    }
    printf("ol_hdg %.3f %.3f %.3f %.3f\n",
	   filter_timestamp_node->getDoubleValue(),
	   ol_hdg, filter_psi_node->getDoubleValue(),
	   filter_psi_node->getDoubleValue() - ol_hdg);
}
