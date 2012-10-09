#ifndef _UGLINK_PUSH_UDP_HXX
#define _UGLINK_PUSH_UDP_HXX

#include <string>
using std::string;

#include "globals.hxx"


// config values
extern string out_host;
extern bool do_broadcast;
extern int ctrls_port;
extern int fdm_port;
extern int gui_port;
extern int openiris_port;

int udp_open_sockets();

void udp_send_data( struct gps *gpspacket,
		    struct imu *imupacket,
		    struct airdata *airpacket,
		    struct filter *filterpacket,
		    struct actuator *actpacket,
		    struct pilot *pilotpacket,
		    struct apstatus *appacket,
		    struct health *healthpacket );


#endif // _UGLINK_PUSH_UDP_HXX
