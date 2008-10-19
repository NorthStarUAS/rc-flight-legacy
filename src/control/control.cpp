/******************************************************************************
 * FILE: control.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 10/11/05 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "comms/logging.h"
#include "comms/uplink.h"
#include "include/globaldefs.h"
#include "navigation/mnav.h"
#include "props/props.hxx"
#include "util/timing.h"

#include "util.h"
#include "xmlauto.hxx"

#include "control.h"


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//pre-defined constant
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define	cdT	      0.04   	     // 25 Hz synchronized with ahrs
#define deg2servo     819    	     // 65536/80deg.
#define servo_mid_pos 32768          // middle position of the servos
#define sign(arg)    (arg>=0 ? 1:-1)
#define MAG_DEC      -0.270944862    /*magnetic declination of Stanford (rad): -15.15 */


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct servo servo_out;
// static double sum[5]={0.,};
//static struct servo    servopos;
//static struct imu      imuval;
//static struct gps      gpsval;
//static struct nav      navval;
//enum   	      modedefs {pitch_mode,roll_mode,heading_mode,altitude_mode,speed_mode,waypoint_mode};

static SGPropertyNode *agl_alt_ft_node = NULL;
static SGPropertyNode *aileron_out_node = NULL;
static SGPropertyNode *elevator_out_node = NULL;
static SGPropertyNode *elevator_damp_node = NULL;
static SGPropertyNode *throttle_out_node = NULL;
static SGPropertyNode *rudder_out_node = NULL;
static SGPropertyNode *ap_target = NULL;
static SGPropertyNode *elevon_mix = NULL;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//control code
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// the "FlightGear" autopilot
static FGXMLAutopilot ap;


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values
    ap.init();
    ap.build();

    // initialize the flight control output property nodes
    agl_alt_ft_node = fgGetNode("/position/altitude-agl-ft", true);
    aileron_out_node = fgGetNode("/controls/flight/aileron", true);
    elevator_out_node = fgGetNode("/controls/flight/elevator", true);
    elevator_damp_node = fgGetNode("/controls/flight/elevator-damp", true);
    throttle_out_node = fgGetNode("/controls/engine/throttle", true);
    rudder_out_node = fgGetNode("/controls/flight/rudder", true);

    ap_target = fgGetNode("/autopilot/settings/target-roll-deg", true);

    elevon_mix = fgGetNode("/config/autopilot/elevon-mixing", true);
}


void control_reset() {
  // initialization:
  if ( display_on ) { printf("Initializing autopilot\n"); }

  //servopos = servo_in;	// save the last servo positions
  //imuval   = imupacket;		// save the last attitude
  //gpsval   = gpspacket;		// save the last gps
  //navval   = navpacket;		// save the last nav
}


void control_update(short flight_mode)
{
    // make a quick exit if we are disabled
    if ( !autopilot_active ) {
      return;
    }

    // reset the autopilot if requested
    if ( autopilot_reinit ) {
      control_reset();
      autopilot_reinit = false;
    }

    // optional: use channel #6 to change the autopilot target value
    // double min_value = -35.0;
    // double max_value = 35.0;
    // double tgt_value = (max_value - min_value) *
    //   ((double)servo_in.chn[5] / 65535.0) + min_value;
    // ap_target->setFloatValue( tgt_value );

    // update the autopilot stages
    ap.update( 0.04 );	// dt = 1/25

    /* printf("%.2f %.2f\n", aileron_out_node->getFloatValue(),
              elevator_out_node->getFloatValue()); */
    /* static SGPropertyNode *vert_speed_fps
       = fgGetNode("/velocities/vertical-speed-fps", true); */
    /* static SGPropertyNode *true_alt
       = fgGetNode("/position/altitude-ft", true); */
    /* printf("%.1f %.2f %.2f\n",
           true_alt->getFloatValue(),
           vert_speed_fps->getFloatValue(),
           elevator_out_node->getFloatValue()); */

    // initialize the servo command array to central values so we don't
    // inherit junk
    for ( int i = 0; i < 8; ++i ) {
        servo_out.chn[i] = 32768;
    }

    float elevator = elevator_out_node->getFloatValue()
	+ elevator_damp_node->getFloatValue();

    if ( elevon_mix->getBoolValue() ) {
        // elevon mixing mode

        //aileron
        servo_out.chn[0] = 32768
            + (int16_t)(aileron_out_node->getFloatValue() * 32768)
            + (int16_t)(elevator * 32768);

        //elevator
        servo_out.chn[1] = 32768
            + (int16_t)(aileron_out_node->getFloatValue() * 32768)
            - (int16_t)(elevator * 32768);
    } else {
        // conventional airframe mode

        //aileron
        servo_out.chn[0] = 32768
            + (int16_t)(aileron_out_node->getFloatValue() * 32768);

        //elevator
        servo_out.chn[1] = 32768
            + (int16_t)(elevator * 32768);
    }

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!

    // Placing the engine throttle under autopilot control requires
    // EXTREME care!!!!

    // Propellers are constructed of sharpened knife material.
    // Electric motors don't quit and give up if they encounter intial
    // resistance.  Severe injuries to hand or face or any other body
    // part in the vicinity of the motor or prop can occur at any
    // time.

    // Care must be taken during initial setup, and then from that
    // point on during all operational, testing, and ground handling
    // phases.  Extreme vigilance must always be maintianed if the
    // autopilot has control of the throttle.

    // I cannot stress this point enough!!!  One nanosecond of
    // distraction or loss of focus can result in severe lifelong
    // injury or death!  Do not take your fingers or face for granted.
    // Always maintain utmost caution and correct safety procedures to
    // ensure safe operation with a throttle enabled UAS.

    // As an internal safety measure, the throttle will be completely
    // turned off (value of 12000 on a 0-65535 scale) when the
    // pressure altitude is < 50m AGL.

    // None of the built in safety measures are sufficient for a safe
    // system!  Pressure sensor readings can glitch, bugs can creep
    // into the code over time, anything can happen.  Be extremely
    // distrustful of the propellor and always make sure your body
    // parts are never in the path of the propellor or where the
    // propellor and aircraft could go if the engine came alive
    // unexpectedly.

    // throttle

    // limit throttle change to 128 units per cycle ... which means it
    // takes about 10 seconds to traverse a range of 32768 (about full
    // range) assuming 25 cycles per second.
    static int16_t last_throttle = 12000;
    int16_t target_throttle = 32768
	+ (int16_t)(throttle_out_node->getFloatValue() * 32768);
    int16_t diff = target_throttle - last_throttle;
    if ( diff > 128 ) diff = 128;
    if ( diff < -128 ) diff = -128;
    servo_out.chn[2] = last_throttle + diff;

    // override and disable throttle output if within 100' of the
    // ground (assuming ground elevation is the pressure altitude we
    // recorded with the system started up.
    if ( agl_alt_ft_node->getFloatValue() < 100.0 ) {
        servo_out.chn[2] = 12000;
    }

    // printf("throttle = %.2f %d\n", throttle_out_node->getFloatValue(),
    //        servo_out.chn[2]);

    last_throttle = servo_out.chn[2];

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!

    // rudder
    servo_out.chn[3] = 32768
        + (int16_t)(rudder_out_node->getFloatValue() * 32768);

    // time stamp the packet for logging
    servo_out.time = get_Time();

    // send commanded servo positions to the MNAV
    send_short_servo_cmd();
}


void control_close() {
  // nothing to see here, move along ...
}
