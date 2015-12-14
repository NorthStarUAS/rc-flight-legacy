#ifndef _AURA_CURRENT_HXX
#define _AURA_CURRENT_HXX


#include <string>

#include "globals.hxx"


void compute_derived_data( struct gps *gpspacket,
			   struct imu *imupacket,
			   struct airdata *airpacket,
			   struct filter *filterpacket,
			   struct actuator *actpacket,
			   struct pilot *pilotpacket,
			   struct apstatus *appacket,
			   struct health *healthpacket,
			   struct payload *payloadpacket );

void update_props( struct gps *gpspacket,
		   struct imu *imupacket,
		   struct airdata *airpacket,
		   struct filter *filterpacket,
		   struct actuator *actpacket,
		   struct pilot *pilotpacket,
		   struct apstatus *appacket,
		   struct health *healthpacket,
		   struct payload *payloadpacket );

string current_get_fcs_nav_string();
string current_get_fcs_speed_string();
string current_get_fcs_altitude_string();


#endif // _AURA_CURRENT_HXX
