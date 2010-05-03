#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <string>

#include "checksum.h"
#include "include/globaldefs.h"

#include "control/route_mgr.hxx"
#include "health/health.h"
#include "props/props.hxx"
#include "sensors/gps_mgr.h"
#include "util/strutils.hxx"

#include "serial.hxx"

#include "console_link.h"

using std::string;

// global variables

static SGPropertyNode *console_dev = NULL;
bool console_link_on = false;    // link to ground station via console port
static SGSerialPort console;

// open up the console port
void console_link_init() {
    console_dev = fgGetNode("/config/console/device", true);
    console.open_port( console_dev->getStringValue(), true );
    console.set_baud( 115200 );
}


static short console_write( const uint8_t *buf, const short size ) {
    int result = console.write_port( (const char *)buf, size );
    return result;
}


static void console_link_packet( const uint8_t packet_id,
				 const uint8_t *packet_buf,
				 const int packet_size )
{
    // printf(" begin console_link_packet()\n");
    uint8_t buf[3];
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = packet_id; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    buf[0] = packet_size; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( packet_buf, packet_size );

    // check sum (2 bytes)
    ugear_cksum( packet_id, packet_size, packet_buf, packet_size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
    // printf(" end console_link_packet()\n");
}


void console_link_gps( uint8_t *gps_buf, int gps_size, int skip_count ) {
    // printf("Console link gps()\n");
    if ( skip_count < 1 ) { skip_count = 1; }
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    console_link_packet( GPS_PACKET_V1, gps_buf, gps_size );
}


void console_link_imu( uint8_t *imu_buf, int imu_size, int skip_count  ) {
    // printf("Console link imu()\n");
    if ( skip_count < 1 ) { skip_count = 1; }
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    console_link_packet( IMU_PACKET_V1, imu_buf, imu_size );
}


void console_link_filter( uint8_t *filter_buf, int filter_size, int skip_count ) {
    // printf("Console link filter()\n");
    if ( skip_count < 1 ) { skip_count = 1; }
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    console_link_packet( FILTER_PACKET_V1, filter_buf, filter_size );
    // printf("end Console link filter()\n");
}


void console_link_actuator( uint8_t *actuator_buf, int actuator_size,
			    int skip_count  )
{
    // printf("Console link actuator()\n");
    if ( skip_count < 1 ) { skip_count = 1; }
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    console_link_packet( ACTUATOR_PACKET_V1, actuator_buf, actuator_size );
}


void console_link_pilot( uint8_t *pilot_buf, int pilot_size, int skip_count  )
{
    // printf("Console link pilot()\n");
    if ( skip_count < 1 ) { skip_count = 1; }
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    console_link_packet( PILOT_INPUT_PACKET_V1, pilot_buf, pilot_size );
}


void console_link_health( struct health *healthpacket, int skip_count  ) {
    // printf("Console link health()\n");
    if ( skip_count < 1 ) { skip_count = 1; }
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

    uint8_t buf[3];
    uint8_t size;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = HEALTH_PACKET_V1; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct health);
    buf[0] = size; buf[1] = 0;
    // printf("servo size = %d\n", size);
    console_write( buf, 1 );

    // packet data
    uint8_t bytes = console_write( (uint8_t *)healthpacket, size );
    // uint8_t *tmp = (uint8_t *)healthpacket;
    // printf("%d %d %d %d\n", tmp[0], tmp[1], tmp[2], tmp[3] );

    if ( bytes != size ) {
      printf("Only wrote %d health bytes out of %d\n", bytes, size);
    }

    // check sum (2 bytes)
    ugear_cksum( HEALTH_PACKET_V1, size, (uint8_t *)healthpacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );

    /* 
       FILE *debug = fopen("/mnt/mmc/debug.txt", "a");
       fprintf(debug, "wpt %d: %.8f %.8f\n", healthpacket->wp_index,
       healthpacket->wp_lon, healthpacket->wp_lat);
       fclose(debug);
    */

}


static void console_link_execute_command( const string command ) {
    vector <string> token = split( command, "," );

    if ( token.size() < 1 ) {
        // no valid tokens
        return;
    }

    // command to fly a new altitude

    // command to interrupt route and come back to some new point and
    // keep passing over it.

    if ( token[0] == "hb" && token.size() == 1 ) {
        // heart beat, ignore

    } else if ( token[0] == "home" && token.size() == 5 ) {
        // specify new home location
        double lon = atof( token[1].c_str() );
        double lat = atof( token[2].c_str() );
        // double alt_ft = atof( token[3].c_str() );
        double course_deg = atof( token[4].c_str() );
        SGWayPoint wp( lon, lat );
        route_mgr.update_home( wp, course_deg, true );
    } else if ( token[0] == "go" && token.size() == 2 ) {
        // specify router mode
        if ( token[1] == "home" ) {
            route_mgr.set_home_mode();
        } else if ( token[1] == "route" ) {
            route_mgr.set_route_mode();
        }

    } else if ( token[0] == "ap" && token.size() == 3 ) {
        // specify an autopilot target
        if ( token[1] == "agl-ft" ) {
            double agl_ft = atof( token[2].c_str() );
            SGPropertyNode_ptr override_agl_node
                = fgGetNode( "/autopilot/settings/override-agl-ft", true );
            override_agl_node->setDoubleValue( agl_ft );
        } else if ( token[1] == "msl-ft" ) {
            double msl_ft = atof( token[2].c_str() );
            SGPropertyNode_ptr override_msl_node
                = fgGetNode( "/autopilot/settings/override-msl-ft", true );
            override_msl_node->setDoubleValue( msl_ft );
        } else if ( token[1] == "speed-kt" ) {
            double speed_kt = atof( token[2].c_str() );
            SGPropertyNode_ptr speed_kt_node
                = fgGetNode( "/autopilot/settings/target-speed-kt", true );
            speed_kt_node->setDoubleValue( speed_kt );
        }
    } else if ( token[0] == "wp" && token.size() == 5 ) {
        // specify new waypoint coordinates for a waypoint
        int index = atoi( token[1].c_str() );
        double lon = atof( token[2].c_str() );
        double lat = atof( token[3].c_str() );
        double alt_ft = atof( token[4].c_str() );
        SGWayPoint wp( lon, lat, alt_ft * SG_FEET_TO_METER );
        route_mgr.replace_waypoint( wp, index );
    }
}


static int console_read_command( char result_buf[256] ) {
    // read character by character until we run out of data or find a '\n'
    // if we run out of data, save what we have so far and start with that for
    // the next call.

    // persistant data because we may not get the whole command in one call
    static char command_buf[256];
    static int command_counter = 0;

    char buf[2]; buf[0] = 0;

    int result = console.read_port( buf, 1 );
    while ( result == 1 && buf[0] != '\n' ) {
        command_buf[command_counter] = buf[0];
        command_counter++;
        result = console.read_port( buf, 1 );
    }

    if ( result == 1 && buf[0] == '\n' ) {
        command_buf[command_counter] = 0; // terminate string
        int size = command_counter + 1;
        strncpy( result_buf, command_buf, size );
        command_counter = 0;
        return size;
    }

    return 0;
}


// calculate the nmea check sum
static char calc_nmea_cksum(const char *sentence) {
    unsigned char sum = 0;
    int i, len;

    // cout << sentence << endl;

    len = strlen(sentence);
    sum = sentence[0];
    for ( i = 1; i < len; i++ ) {
        // cout << sentence[i];
        sum ^= sentence[i];
    }
    // cout << endl;

    // printf("sum = %02x\n", sum);
    return sum;
}


// read, parse, and execute incomming commands, return true if a valid
// command received, false otherwise.
bool console_link_command() {
    char command_buf[256];
    int result = console_read_command( command_buf );

    if ( result == 0 ) {
        return false;
    }
    
    /* printf("read command '%s'\n", command_buf); */

    /*
      FILE *debug;
      debug = fopen("/tmp/debug.txt", "a");
      fprintf(debug, "Received command: '%s'\n", command_buf);
      fclose(debug);
    */

    string cmd = command_buf;

    // validate check sum
    if ( cmd.length() < 4 ) {
        // bogus command
        return false;
    }
    string nmea_sum = cmd.substr(cmd.length() - 2);
    cmd = cmd.substr(0, cmd.length() - 3);
    char cmd_sum[10];
    snprintf( cmd_sum, 3, "%02X", calc_nmea_cksum(cmd.c_str()) );

    /*
      debug = fopen("/tmp/debug.txt", "a");
      fprintf(debug, " cmd: '%s' nmea: '%s' '%s'\n", cmd.c_str(),
              nmea_sum.c_str(), cmd_sum);
      fclose(debug);
    */

    if ( nmea_sum.c_str()[0] != cmd_sum[0]
         || nmea_sum.c_str()[1] != cmd_sum[1])
    {
        // checksum failure
        /*
	  debug = fopen("/tmp/debug.txt", "a");
	  fprintf(debug, "check sum failure\n");
	  fclose(debug);
	*/
        return false;
    }

    // parse the command
    string::size_type pos = cmd.find_first_of(",");
    if ( pos == string::npos ) {
        // bogus command
        return false;
    }

    /*
      FILE *debug = fopen("/mnt/mmc/debug.txt", "a");
      fprintf(debug, "command: %s\n", cmd.c_str());
      fclose(debug);
    */

    // extract command sequence number
    string num = cmd.substr(0, pos);
    int sequence = atoi( num.c_str() );

    // remainder
    cmd = cmd.substr(pos + 1);

    // execute command
    /*
      debug = fopen("/tmp/debug.txt", "a");
      fprintf(debug, "Sequence: %d  Execute: '%s'\n", sequence, cmd.c_str());
      fclose(debug);
    */
    console_link_execute_command( cmd );

    // register that we've received this message correctly
    health_update_command_sequence(sequence);

    return true;
}
