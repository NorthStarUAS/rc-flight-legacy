//
// FILE: act_goldy2.cxx
// DESCRIPTION: send actuator commands to Goldy2
//

#include "python/pyprops.hxx"

#include <stdio.h>
#include <string>
#include <string.h>

#include "comms/netSocket.h"
#include "init/globals.hxx"
#include "util/timing.h"

#include "sensors/util_goldy2.hxx"
#include "act_goldy2.hxx"

#define PWM_CENTER 1500
#define PWM_HALF_RANGE 513
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)

static netSocket sock;
static int port = 0;
static string hostname = "";
static string output_mode = "radians";
static string controls_mapping = "vireo";

// property nodes
static pyPropertyNode act_node;
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;


// initialize goldy2 config property nodes
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("host") ) {
	hostname = config->getString("host");
    }
    if ( config->hasChild("port") ) {
	port = config->getLong("port");
    }
    if ( config->hasChild("controls_mapping") ) {
	controls_mapping = config->getString("controls_mapping");
    }
    if ( config->hasChild("output_mode") ) {
	output_mode = config->getString("output_mode");
    }
}


/// initialize actuator property nodes 
static void bind_act_nodes( string output_path ) {
    act_node = pyGetNode(output_path, true);
#define NUM_ACTUATORS 8
    act_node.setLen("channel", NUM_ACTUATORS, 0.0);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
}


// function prototypes
bool goldy2_act_init( string output_path, pyPropertyNode *config ) {
    printf("actuator_init()\n");

    bind_input( config );
    bind_act_nodes( output_path );

    // open a UDP socket
    if ( ! sock.open( false ) ) {
	printf("Error opening imu input socket\n");
	return false;
    }

    // connect ...
    if ( sock.connect( hostname.c_str(), port ) == -1 ) {
	printf("error connecting to %s:%d\n", hostname.c_str(), port);
	return false;
    }

    // don't block
    sock.setBlocking( false );

    return true;
}


#if 0
// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count )
{
    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
}
#endif


// generate a pwm pulse length from a normalized [-1 to 1] or [0 to 1] range
static int gen_pulse( double val, bool symmetrical ) {
    int pulse = 0;

    if ( symmetrical ) {
	// i.e. aileron, rudder, elevator
	if ( val < -1.5 ) { val = -1.5; }
	if ( val > 1.5 ) { val = 1.5; }
	pulse = PWM_CENTER + (int)(PWM_HALF_RANGE * val);
    } else {
	// i.e. throttle, flaps
	if ( val < 0.0 ) { val = 0.0; }
	if ( val > 1.0 ) { val = 1.0; }
	pulse = PWM_MIN + (int)(PWM_RANGE * val);
    }

    return pulse;
}


bool goldy2_act_update() {
    // static uint16_t pos = 1000;
    const int goldy2_act_size = 68;
    const int packet_size = 60;	// 6*n, n=10
    const int max_act = 10;
    uint8_t packet_buf[goldy2_act_size];
    uint8_t *start_buf = packet_buf;
    uint8_t *buf = packet_buf;

    *(char *)buf = 'U'; buf++;
    *(char *)buf = 'M'; buf++;
    *(char *)buf = 'N'; buf++;
    *(uint8_t *)buf = 0x01; buf++;	  // payload type
    *(uint8_t *)buf = packet_size; buf++; // LSB
    *(uint8_t *)buf = 0; buf++;		  // MSB

    double aileron = act_node.getDouble("channel", 0);
    double elevator = act_node.getDouble("channel", 1);
    double throttle = act_node.getDouble("channel", 2);
    double rudder = act_node.getDouble("channel", 3);
    // double gear = act_node.getDouble("channel", 4);
    double flaps = act_node.getDouble("channel", 5);
    
    // printf("ail=%.2f ele=%.2f\n", aileron, elevator);

    int thr_pwm = gen_pulse( throttle, false );
    // printf("thr pwm = %d\n", thr_pwm);
    int zero_act = 0;
    int units = 0; // 0 = PWM, 1 = angle control (radians)
    
    if ( output_mode == "pwm" ) {
	zero_act = PWM_CENTER;
	units = 0;
    } else if ( output_mode == "radians" ) {
	zero_act = 0.0;
	units = 1;
    }
    int act_out[max_act] = { zero_act };

    if ( controls_mapping == "vireo" ) {
	int right_act = 0;
	int left_act = 0;
	if ( output_mode == "pwm" ) {
	    double right_cmd = -elevator*0.5 - aileron*0.5;
	    double left_cmd = elevator*0.5 - aileron*0.5;
	    right_act = gen_pulse( right_cmd, true );
	    left_act = gen_pulse( left_cmd, true );
	} else if ( output_mode == "radians" ) {
	    right_act = (elevator + aileron) * 1000.0;
	    if ( right_act < -1570 ) { right_act = -1570; }
	    if ( right_act > 1570 ) { right_act = 1570; }
	    left_act = (elevator - aileron) * 1000.0;
	    if ( left_act < -1570 ) { left_act = -1570; }
	    if ( left_act > 1570 ) { left_act = 1570; }
	}
	act_out[2] = thr_pwm;
	act_out[3] = right_act;
	act_out[4] = left_act;
    } else if ( controls_mapping == "tyr" ) {
	int ele_act = 0;
	int rud_act = 0;
	int right_ail_act = 0;
	int left_ail_act = 0;
	int right_flap_act = 0;
	int left_flap_act = 0;
	if ( output_mode == "pwm" ) {
	    ele_act = gen_pulse( -elevator, true );
	    rud_act = gen_pulse( -rudder, true );
	    right_ail_act = gen_pulse( aileron, true );
	    left_ail_act = gen_pulse( -aileron, true );
	    right_flap_act = gen_pulse( flaps, true );
	    left_flap_act = gen_pulse( flaps, true );
	} else if ( output_mode == "radians" ) {
	    ele_act = -elevator * 1000.0;
	    if ( ele_act < -1570 ) { ele_act = -1570; }
	    if ( ele_act > 1570 ) { ele_act = 1570; }
	    rud_act = -rudder * 1000.0;
	    if ( rud_act < -1570 ) { rud_act = -1570; }
	    if ( rud_act > 1570 ) { rud_act = 1570; }
	    right_ail_act = aileron * 1000.0;
	    if ( right_ail_act < -1570 ) { right_ail_act = -1570; }
	    if ( right_ail_act > 1570 ) { right_ail_act = 1570; }
	    left_ail_act = -aileron * 1000.0;
	    if ( left_ail_act < -1570 ) { left_ail_act = -1570; }
	    if ( left_ail_act > 1570 ) { left_ail_act = 1570; }
	    right_flap_act = flaps * 1000.0;
	    if ( right_flap_act < -1570 ) { right_flap_act = -1570; }
	    if ( right_flap_act > 1570 ) { right_flap_act = 1570; }
	    left_flap_act = flaps * 1000.0;
	    if ( left_flap_act < -1570 ) { left_flap_act = -1570; }
	    if ( left_flap_act > 1570 ) { left_flap_act = 1570; }
	}
	act_out[2] = thr_pwm;
	act_out[3] = rud_act;
	act_out[4] = ele_act;
	act_out[5] = left_ail_act;
	act_out[6] = left_flap_act;
	act_out[7] = right_flap_act;
	act_out[8] = right_ail_act;
    }
    
    for ( uint8_t i = 0; i < 10; i++ ) {
	int ch_units = units;
	int val = act_out[i];
	if ( i == 2 ) {
            // channel 2 is always throttle, always pwm units
	    ch_units = 0;
	}
	*(uint8_t *)buf = i; buf++;
	*(uint8_t *)buf = ch_units; buf++;
        if ( ch_units == 0 ) {
	    *(uint16_t *)buf = val; buf += 2;
	    *(int16_t *)buf = 0; buf += 2;
        } else {
	    *(uint16_t *)buf = 0; buf += 2;
	    *(int16_t *)buf = val; buf += 2;
        }
    }

    uint16_t CRC = utilCRC16(start_buf+3, packet_size+3, 0);
    uint16_t CRC_msb = CRC / 256;
    uint16_t CRC_lsb = CRC - (CRC_msb * 256);
    *(uint8_t *)buf = CRC_lsb; buf++;
    *(uint8_t *)buf = CRC_msb; buf++;
    /* printf("actual size = %d (we hoped for %d)\n", (int)(buf - start_buf), goldy2_act_size); */

    int result = sock.send( packet_buf, goldy2_act_size, 0 );
    if ( result != goldy2_act_size ) {
	return false;
    }

    return true;
}


void goldy2_act_close() {
    sock.close();
}
