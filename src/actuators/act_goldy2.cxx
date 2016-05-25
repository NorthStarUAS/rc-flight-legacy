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

#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)

static netSocket sock;
static int port = 0;
static string hostname = "";
static string output_mode = "angles";

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
    uint8_t packet_buf[goldy2_act_size];
    uint8_t *start_buf = packet_buf;
    uint8_t *buf = packet_buf;

    *(char *)buf = 'U'; buf++;
    *(char *)buf = 'M'; buf++;
    *(char *)buf = 'N'; buf++;
    *(uint8_t *)buf = 0x01; buf++;	  // payload type
    *(uint8_t *)buf = packet_size; buf++; // LSB
    *(uint8_t *)buf = 0; buf++;		  // MSB

    // if (pos++ > 2000) { pos = 1000; }

    double aileron = act_node.getDouble("channel", 0);
    double elevator = act_node.getDouble("channel", 1);
    double throttle = act_node.getDouble("channel", 2);
    
    int thr_pwm = gen_pulse( throttle, false );
    int left_act = 0;
    int right_act = 0;
    int zero_act = 0;
    int units = 0; // 0 = PWM, 1 = angle control (radians)
    
    if ( output_mode == "pwm" ) {
	double left_cmd = elevator*0.5 - aileron*0.5;
	double right_cmd = -elevator*0.5 - aileron*0.5;
	left_act = gen_pulse( left_cmd, true );
	right_act = gen_pulse( right_cmd, true );
	zero_act = PWM_CENTER;
	units = 0;
    } else if ( output_mode == "radians" ) {
	left_act = (elevator - aileron) * 1000.0;
	if ( left_act < -1570 ) { left_act = -1570; }
	if ( left_act > 1570 ) { left_act = 1570; }
	right_act = (elevator + aileron) * 1000.0;
	if ( right_act < -1570 ) { right_act = -1570; }
	if ( right_act > 1570 ) { right_act = 1570; }
	zero_act = 0.0;
	units = 1;
    }
    
    int val;
    for ( uint8_t i = 0; i < 10; i++ ) {
	int ch_units = units;
	if ( i == 2 ) {
            val = thr_pwm;
	    ch_units = 0;
        } else if ( i == 3 ) {
	    val = right_act;
	} else if ( i == 4 ) {
	    val = left_act;
	} else {
	    val = zero_act;
	}
        // val = pos;
	*(uint8_t *)buf = i; buf++;
	*(uint8_t *)buf = ch_units; buf++;
	*(uint16_t *)buf = val; buf += 2;
	*(int16_t *)buf = 0; buf += 2;
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
