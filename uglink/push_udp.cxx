#include <iostream>
#include <math.h>
#include <arpa/inet.h>

//#include <plib/net.h>

#include <include/globaldefs.h>
#include <comms/netSocket.h>

#include "net_ctrls.hxx"
#include "net_fdm.hxx"
#include "net_gui.hxx"

#include "push_udp.hxx"
#include "wp_mgr.hxx"

using std::cout;
using std::endl;


// config values
string out_host = "localhost";
bool do_broadcast = false;
int ctrls_port;
int fdm_port;
int gui_port;
int openiris_port;

// Network channels
static netSocket ctrls_sock, fdm_sock, gui_sock, /*opengc_sock,*/ openiris_sock;

static bool props_inited = false;
static SGPropertyNode *alt_offset_node = NULL;
static SGPropertyNode *est_controls_node = NULL;
static SGPropertyNode *use_groundtrack_hdg_node = NULL;
static SGPropertyNode *use_ground_speed_node = NULL;
static SGPropertyNode *flying_wing_node = NULL;
static SGPropertyNode *filter_speed_node = NULL;
static SGPropertyNode *wind_deg_node = NULL;
static SGPropertyNode *wind_speed_node = NULL;
static SGPropertyNode *pitot_scale_node = NULL;
static SGPropertyNode *filter_climb_node = NULL;

int udp_open_sockets()
{
    if ( ! props_inited ) {
	props_inited = true;
	alt_offset_node = pyGetNode("/config/alt-offset-m", true);
	est_controls_node = pyGetNode("/config/estimate-controls", true);
	use_groundtrack_hdg_node = pyGetNode("/config/use-groundtrack-heading",
					     true);
	use_ground_speed_node = pyGetNode("/config/use-ground-speed", true);
	flying_wing_node = pyGetNode("/config/flying-wing-mode", true);
	filter_speed_node = pyGetNode("/filters/filter/speed-kt", true);
	wind_deg_node = pyGetNode("/filters/wind-deg", true);
	wind_speed_node = pyGetNode("/filters/wind-speed-kt", true);
	pitot_scale_node = pyGetNode("/filters/pitot-scale-factor", true);
	filter_climb_node = pyGetNode("/filters/climb-rate-fpm", true);
    }

    if ( ! ctrls_sock.open( false ) ) {  // open a UDP socket
        cout << "error opening ctrls output socket" << endl;
        return -1;
    }
    if ( ! fdm_sock.open( false ) ) {  // open a UDP socket
        cout << "error opening fdm output socket" << endl;
        return -1;
    }
    if ( ! gui_sock.open( false ) ) {  // open a UDP socket
        cout << "error opening gui output socket" << endl;
        return -1;
    }
    if ( ! openiris_sock.open( false ) ) {  // open a UDP socket
        cout << "error opening openiris output socket" << endl;
        return -1;
    }
    cout << "open net channels" << endl;

    ctrls_sock.setBlocking( false );
    fdm_sock.setBlocking( false );
    gui_sock.setBlocking( false );
    openiris_sock.setBlocking( false );
    cout << "blocking false" << endl;

    if ( do_broadcast ) {
        ctrls_sock.setBroadcast( true );
        fdm_sock.setBroadcast( true );
        gui_sock.setBroadcast( true );
        openiris_sock.setBroadcast( true );
    }

    if ( ctrls_sock.connect( out_host.c_str(), ctrls_port ) == -1 ) {
        perror("connect");
        cout << "error connecting to outgoing ctrls port: " << out_host
	     << ":" << ctrls_port << endl;
        return -1;
    }
    cout << "connected outgoing ctrls socket" << endl;

    if ( fdm_sock.connect( out_host.c_str(), fdm_port ) == -1 ) {
        perror("connect");
        cout << "error connecting to outgoing fdm port: " << out_host
	     << ":" << fdm_port << endl;
        return -1;
    }
    cout << "connected outgoing fdm socket" << endl;

    if ( gui_sock.connect( out_host.c_str(), gui_port ) == -1 ) {
        perror("connect");
        cout << "error connecting to outgoing gui port: " << out_host
	     << ":" << gui_port << endl;
        return -1;
    }
    cout << "connected outgoing gui socket" << endl;

    if ( openiris_sock.connect( out_host.c_str(), openiris_port ) == -1 ) {
        perror("connect");
        cout << "error connecting to outgoing openiris port: " << out_host
	     << ":" << openiris_port << endl;
        return -1;
    }
    cout << "connected outgoing openiris socket" << endl;

    return 0;
}


static bool sgIsLittleEndian() {
    static const int sgEndianTest = 1;
    return (*((char *) &sgEndianTest ) != 0);
}

static bool sgIsBigEndian() {
    static const int sgEndianTest = 1;
    return (*((char *) &sgEndianTest ) == 0);
}


// The function htond is defined this way due to the way some
// processors and OSes treat floating point values.  Some will raise
// an exception whenever a "bad" floating point value is loaded into a
// floating point register.  Solaris is notorious for this, but then
// so is LynxOS on the PowerPC.  By translating the data in place,
// there is no need to load a FP register with the "corruped" floating
// point value.  By doing the BIG_ENDIAN test, I can optimize the
// routine for big-endian processors so it can be as efficient as
// possible
static void htond (double &x)	
{
    if ( sgIsLittleEndian() ) {
        int    *Double_Overlay;
        int     Holding_Buffer;
    
        Double_Overlay = (int *) &x;
        Holding_Buffer = Double_Overlay [0];
    
        Double_Overlay [0] = htonl (Double_Overlay [1]);
        Double_Overlay [1] = htonl (Holding_Buffer);
    } else {
        return;
    }
}

// Float version
static void htonf (float &x)	
{
    if ( sgIsLittleEndian() ) {
        int    *Float_Overlay;
        int     Holding_Buffer;
    
        Float_Overlay = (int *) &x;
        Holding_Buffer = Float_Overlay [0];
    
        Float_Overlay [0] = htonl (Holding_Buffer);
    } else {
        return;
    }
}


static void ugear2fg( struct gps *gpspacket,
		      struct imu *imupacket,
		      struct airdata *airpacket,
		      struct filter *filterpacket,
		      struct actuator *actpacket,
		      struct pilot *pilotpacket,
		      struct apstatus *appacket,
		      struct health *healthpacket,
		      FGNetFDM *fdm, FGNetCtrls *ctrls )
{
    unsigned int i;

    // Version sanity checking
    fdm->version = FG_NET_FDM_VERSION;

    // Aero parameters
    fdm->longitude = filterpacket->lon * SG_DEGREES_TO_RADIANS;
    fdm->latitude = filterpacket->lat * SG_DEGREES_TO_RADIANS;
    fdm->altitude = filterpacket->alt + alt_offset_node->getDouble();
    fdm->agl = -9999.0;
    fdm->phi = filterpacket->phi * SG_DEGREES_TO_RADIANS; // roll
    fdm->theta = filterpacket->theta * SG_DEGREES_TO_RADIANS; // pitch;
    fdm->psi = filterpacket->psi * SG_DEGREES_TO_RADIANS; // heading 

    fdm->phidot = 0.0;
    fdm->thetadot = 0.0;
    fdm->psidot = 0.0;

    double vn = filterpacket->vn;
    double ve = filterpacket->ve;
    double vd = filterpacket->vd;

    if ( use_groundtrack_hdg_node->getBoolValue() ) {
        fdm->psi = SGD_PI * 0.5 - atan2(vn, ve); // heading
    }

    if ( use_ground_speed_node->getBoolValue() ) {
	fdm->vcas = filter_speed_node->getDouble();
    } else {
	fdm->vcas = airpacket->airspeed;
    }
    fdm->climb_rate = filter_climb_node->getDouble() / 60.0;
    // fdm->altitude = est_altitude_m;

    /* printf("%.3f, %.3f, %.3f, %.3f, %.8f, %.8f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
           imupacket->time, imupacket->the, -filterpacket->vd, climbf,
           filterpacket->lat, filterpacket->lon, gpspacket->alt, filterpacket->alt,
           imupacket->Ps, Ps, Ps + Ps_error); */

    // cout << "climb rate = " << aero->hdota << endl;
    fdm->v_north = vn * SG_METER_TO_FEET;
    fdm->v_east = ve * SG_METER_TO_FEET;
    fdm->v_down = vd * SG_METER_TO_FEET;
    fdm->v_wind_body_north = 0.0;
    fdm->v_wind_body_east = 0.0;
    fdm->v_wind_body_down = 0.0;
    fdm->stall_warning = 0.0;

    fdm->A_X_pilot = 0.0;
    fdm->A_Y_pilot = 0.0;
    fdm->A_Z_pilot = 0.0 /* (should be -G) */;

    // Engine parameters
    fdm->num_engines = 1;
    fdm->eng_state[0] = 2;
    // cout << "state = " << fdm->eng_state[0] << endl;
    double rpm = 5000.0 - ((double)actpacket->thr / 65536.0)*3500.0;
    if ( rpm < 0.0 ) { rpm = 0.0; }
    if ( rpm > 5000.0 ) { rpm = 5000.0; }
    fdm->rpm[0] = rpm;

    fdm->fuel_flow[0] = 0.0;
    fdm->egt[0] = 0.0;
    // cout << "egt = " << aero->EGT << endl;
    fdm->oil_temp[0] = 0.0;
    fdm->oil_px[0] = 0.0;

    // Consumables
    fdm->num_tanks = 2;
    fdm->fuel_quantity[0] = 0.0;
    fdm->fuel_quantity[1] = 0.0;

    // Gear and flaps
    fdm->num_wheels = 3;
    fdm->wow[0] = 0;
    fdm->wow[1] = 0;
    fdm->wow[2] = 0;

    // the following really aren't used in this context
    fdm->cur_time = 0;
    fdm->warp = 0;
    fdm->visibility = 0;

    // cout << "Flap deflection = " << aero->dflap << endl;
    fdm->left_flap = 0.0;
    fdm->right_flap = 0.0;

    if ( est_controls_node->getBoolValue() ) {
        static float est_elev = 0.0;
        static float est_aileron = 0.0;
        static float est_rudder = 0.0;
        est_elev = 0.99 * est_elev + 0.01 * (imupacket->q * 4);
        est_aileron = 0.95 * est_aileron + 0.05 * (imupacket->p * 5);
        est_rudder = 0.95 * est_rudder + 0.05 * (imupacket->r * 2);
        ctrls->elevator = fdm->elevator = -est_elev;
        ctrls->aileron = fdm->left_aileron = est_aileron;
        fdm->right_aileron = -est_aileron;
        ctrls->rudder = fdm->rudder = est_rudder;
    } else {
	if ( pilotpacket->ch5 > 0.5 ) {
	    // manual override is on, display pilot inputs
	    if ( !flying_wing_node->getBoolValue() ) {
		ctrls->elevator = fdm->elevator = pilotpacket->ele * -1.0;
		ctrls->aileron = fdm->left_aileron = pilotpacket->ail;
		fdm->right_aileron = pilotpacket->ail * -1.0;
		ctrls->rudder = fdm->rudder = pilotpacket->rud * -1.0;
	    } else {
		ctrls->elevator = fdm->elevator = 0.0;
		ctrls->aileron = fdm->left_aileron = pilotpacket->ail;
		fdm->right_aileron = pilotpacket->ele;
		ctrls->rudder = fdm->rudder = pilotpacket->rud;
	    }
	    ctrls->throttle[0] = pilotpacket->thr;
	} else {
	    // autopilot is active, display actuator commands
	    if ( !flying_wing_node->getBoolValue() ) {
		ctrls->elevator = fdm->elevator = actpacket->ele * -1.0;
		ctrls->aileron = fdm->left_aileron = actpacket->ail * -1.0;
		fdm->right_aileron = actpacket->ail;
		ctrls->rudder = fdm->rudder = actpacket->rud * -1.0;
	    } else {
		double ch1 = actpacket->ail;
		double ch2 = actpacket->ele;
		double e = (ch1 - ch2) / 2.0;
		double a = ch1 - e;
		ctrls->elevator = fdm->elevator = e;
		ctrls->aileron = fdm->left_aileron = a;
		fdm->right_aileron = actpacket->ele;
		ctrls->rudder = fdm->rudder = actpacket->rud;
	    }
	    ctrls->throttle[0] = actpacket->thr;
	}
	ctrls->elevator *= 1.0;
	ctrls->aileron *= 1.0;
    }
    fdm->elevator_trim_tab = 0.0;
    fdm->left_flap = 0.0;
    fdm->right_flap = 0.0;
    fdm->nose_wheel = 0.0;
    fdm->speedbrake = 0.0;
    fdm->spoilers = 0.0;

    // Convert the net buffer to network format
    fdm->version = htonl(fdm->version);

    htond(fdm->longitude);
    htond(fdm->latitude);
    htond(fdm->altitude);
    htonf(fdm->agl);
    htonf(fdm->phi);
    htonf(fdm->theta);
    htonf(fdm->psi);
    htonf(fdm->alpha);
    htonf(fdm->beta);

    htonf(fdm->phidot);
    htonf(fdm->thetadot);
    htonf(fdm->psidot);
    htonf(fdm->vcas);
    htonf(fdm->climb_rate);
    htonf(fdm->v_north);
    htonf(fdm->v_east);
    htonf(fdm->v_down);
    htonf(fdm->v_wind_body_north);
    htonf(fdm->v_wind_body_east);
    htonf(fdm->v_wind_body_down);

    htonf(fdm->A_X_pilot);
    htonf(fdm->A_Y_pilot);
    htonf(fdm->A_Z_pilot);

    htonf(fdm->stall_warning);
    htonf(fdm->slip_deg);

    for ( i = 0; i < fdm->num_engines; ++i ) {
        fdm->eng_state[i] = htonl(fdm->eng_state[i]);
        htonf(fdm->rpm[i]);
        htonf(fdm->fuel_flow[i]);
        htonf(fdm->egt[i]);
        htonf(fdm->cht[i]);
        htonf(fdm->mp_osi[i]);
        htonf(fdm->tit[i]);
        htonf(fdm->oil_temp[i]);
        htonf(fdm->oil_px[i]);
    }
    fdm->num_engines = htonl(fdm->num_engines);

    for ( i = 0; i < fdm->num_tanks; ++i ) {
        htonf(fdm->fuel_quantity[i]);
    }
    fdm->num_tanks = htonl(fdm->num_tanks);

    for ( i = 0; i < fdm->num_wheels; ++i ) {
        fdm->wow[i] = htonl(fdm->wow[i]);
        htonf(fdm->gear_pos[i]);
        htonf(fdm->gear_steer[i]);
        htonf(fdm->gear_compression[i]);
    }
    fdm->num_wheels = htonl(fdm->num_wheels);

    fdm->cur_time = htonl( fdm->cur_time );
    fdm->warp = htonl( fdm->warp );
    htonf(fdm->visibility);

    htonf(fdm->elevator);
    htonf(fdm->elevator_trim_tab);
    htonf(fdm->left_flap);
    htonf(fdm->right_flap);
    htonf(fdm->left_aileron);
    htonf(fdm->right_aileron);
    htonf(fdm->rudder);
    htonf(fdm->nose_wheel);
    htonf(fdm->speedbrake);
    htonf(fdm->spoilers);

    ctrls->version = FG_NET_CTRLS_VERSION;
    ctrls->elevator_trim = 0.0;
    ctrls->flaps = 0.0;
    ctrls->num_engines = 4;
    ctrls->num_tanks = 1;

    ctrls->version = htonl(ctrls->version);
    ctrls->num_engines = htonl(ctrls->num_engines);
    ctrls->num_tanks = htonl(ctrls->num_tanks);
    htond(ctrls->aileron);
    htond(ctrls->rudder);
    htond(ctrls->elevator);
    htond(ctrls->elevator_trim);
    htond(ctrls->flaps);
    htond(ctrls->throttle[0]);

    ctrls->speedup = 1;
    ctrls->freeze = 0;
    ctrls->speedup = htonl(ctrls->speedup);
    ctrls->freeze = htonl(ctrls->freeze);

    // additional "abused" data fields
    ctrls->reserved[0] = appacket->target_roll_deg * 100.0 + 18000.0; // flight director target roll
    ctrls->reserved[1] = appacket->target_pitch_deg * 100.0 + 9000.0; // flight director target pitch
    double hdg_bug_offset = appacket->target_heading_deg - filterpacket->psi;
    if ( hdg_bug_offset < -180.0 ) { hdg_bug_offset += 360.0; }
    if ( hdg_bug_offset > 180.0 ) { hdg_bug_offset -= 360.0; }
    ctrls->reserved[2] = hdg_bug_offset * 100.0 + 36000.0; // target heading bug
    ctrls->reserved[3] = appacket->target_climb_fps * 1000.0 + 100000.0;   // target VVI bug
    ctrls->reserved[4] = appacket->target_altitude_msl_ft * 100.0; // target altitude bug
    ctrls->reserved[5] = appacket->target_speed_kt * 100.0; // target speed bug
    static double vn_filter = 0.0;
    static double ve_filter = 0.0;
    vn_filter = 0.95*vn_filter + 0.05*vn;
    ve_filter = 0.95*ve_filter + 0.05*ve;
    double ground_track = (SGD_PI * 0.5 - atan2(vn_filter, ve_filter))*SG_RADIANS_TO_DEGREES;
    double ground_track_offset = ground_track - filterpacket->psi;
    while ( ground_track_offset < -180.0 ) { ground_track_offset += 360.0; }
    while ( ground_track_offset > 180.0 ) { ground_track_offset -= 360.0; }
    // printf("gt = %.0f hdg = %.0f offset = %.0f\n", ground_track, filterpacket->psi, ground_track_offset);
    ctrls->reserved[6] = ground_track_offset * 100.0 + 36000.0;
    ctrls->reserved[7] = gpspacket->satellites;   // gps status box

    wp_mgr.update_wp( appacket->target_wp, appacket->wp_index,
		      appacket->wp_lon, appacket->wp_lat );
    wp_mgr.update_pos_vel( filterpacket->timestamp,
			   filterpacket->lon, filterpacket->lat,
			   filterpacket->vn, filterpacket->ve );
    ctrls->reserved[8] = (int)(wp_mgr.get_dist_m() / 10.0);
    ctrls->reserved[9] = wp_mgr.get_eta_sec();
    /* printf("%s  %s\n",
       wp_mgr.get_dist_str().c_str(), wp_mgr.get_eta_str().c_str() ); */

    for ( int i = 0; i < RESERVED_SPACE; i++ ) {
	ctrls->reserved[i] = htonl(ctrls->reserved[i]);
    }
}


static void ugear2gui( struct gps *gpspacket,
		       struct imu *imupacket,
		       struct airdata *airpacket,
		       struct filter *filterpacket,
                       struct actuator *actpacket,
		       struct pilot *pilotpacket,
		       struct apstatus *appacket,
		       struct health *healthpacket,
                       FGNetGUI *gui )
{
    // Version sanity checking
    gui->version = FG_NET_GUI_VERSION;

    // Aero parameters
    gui->longitude = filterpacket->lon * SG_DEGREES_TO_RADIANS;
    gui->latitude = filterpacket->lat * SG_DEGREES_TO_RADIANS;
    gui->altitude = filterpacket->alt + alt_offset_node->getDouble();
    gui->agl = -9999.0;
    gui->phi = filterpacket->phi * SG_DEGREES_TO_RADIANS;     // roll
    gui->theta = filterpacket->theta * SG_DEGREES_TO_RADIANS; // pitch;
    gui->psi = filterpacket->psi * SG_DEGREES_TO_RADIANS;     // heading

    double vn = filterpacket->vn;
    double ve = filterpacket->ve;
    // double vd = filterpacket->vd;

    if ( use_groundtrack_hdg_node->getBoolValue() ) {
        gui->psi = SGD_PI * 0.5 - atan2(vn, ve); // heading
    }

    // printf("%.1f kts %.2f fps\n", speed_kts, climb_fps);

    if ( use_ground_speed_node->getBoolValue() ) {
	gui->vcas = filter_speed_node->getDouble();
    } else {
	gui->vcas = airpacket->airspeed;
    }
    gui->climb_rate = (filter_climb_node->getDouble()/60.0);
    // gui->altitude = est_altitude_m;

    /* printf("%.3f, %.3f, %.3f, %.3f, %.8f, %.8f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
           imupacket->time, imupacket->the, -filterpacket->vd, climbf,
           filterpacket->lat, filterpacket->lon, gpspacket->alt, filterpacket->alt,
           imupacket->Ps, Ps, Ps + Ps_error); */

    // cout << "climb rate = " << aero->hdota << endl;

    // Consumables
    gui->num_tanks = 2;
    gui->fuel_quantity[0] = 0.0;
    gui->fuel_quantity[1] = 0.0;

    // the following really aren't used in this context
    gui->cur_time = 0;
    gui->warp = 0;
    gui->ground_elev = 0;
    gui->wind_deg = wind_deg_node->getDouble();
    gui->wind_kts = wind_speed_node->getDouble();

    gui->wp_index = appacket->wp_index;
    gui->wp_lon = appacket->wp_lon;
    gui->wp_lat = appacket->wp_lat;
    gui->target_wp = appacket->target_wp;
    gui->num_tanks = appacket->route_size;

#if 0
    // Convert the net buffer to network format
    gui->version = htonl(gui->version);

    htond(gui->longitude);
    htond(gui->latitude);
    htonf(gui->altitude);
    htonf(gui->agl);
    htonf(gui->phi);
    htonf(gui->theta);
    htonf(gui->psi);

    htonf(gui->vcas);
    htonf(gui->climb_rate);

    for ( unsigned int i = 0; i < gui->num_tanks; ++i ) {
        htonf(gui->fuel_quantity[i]);
    }
    gui->num_tanks = htonl(gui->num_tanks);

    gui->cur_time = htonl( gui->cur_time );
    gui->warp = htonl( gui->warp );
#endif
}


void udp_send_data( struct gps *gpspacket,
		    struct imu *imupacket,
		    struct airdata *airpacket,
		    struct filter *filterpacket,
		    struct actuator *actpacket,
		    struct pilot *pilotpacket,
		    struct apstatus *appacket,
		    struct health *healthpacket )
{
    int len;
    int ctrlsize = sizeof( FGNetCtrls );
    int fdmsize = sizeof( FGNetFDM );
    int guisize = sizeof( FGNetGUI );
    //int ogcsize = sizeof( ogcFGData );

    //ogcFGData fgogc; 
    FGNetCtrls fgctrls;
    FGNetFDM fgfdm;
    FGNetGUI fggui;

    ugear2fg( gpspacket, imupacket, airpacket, filterpacket, actpacket,
	      pilotpacket, appacket, healthpacket, &fgfdm, &fgctrls );
    ugear2gui( gpspacket, imupacket, airpacket, filterpacket, actpacket,
	       pilotpacket, appacket, healthpacket, &fggui );

    len = ctrls_sock.send(&fgctrls, ctrlsize, 0);
    len = fdm_sock.send(&fgfdm, fdmsize, 0);
    len = gui_sock.send(&fggui, guisize, 0);
    len = openiris_sock.send(&fggui, guisize, 0);
}
