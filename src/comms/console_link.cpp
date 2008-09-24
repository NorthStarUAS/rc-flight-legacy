#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <string>

#include "checksum.h"
#include "globaldefs.h"
#include "serial.h"

#include <control/route_mgr.hxx>
#include <health/health.h>
#include <util/strutils.hxx>

#include "console_link.h"

using std::string;

// global variables

bool console_link_on = false;    // link to ground station via console port
char console_dev[64] = "/dev/ttyS0";
static int confd;


// open up the console port
void console_link_init() {
    confd = open_serial( console_dev, BAUDRATE_115200, true, true );
}


static short console_write( uint8_t *buf, short size ) {
    for ( int i = 0; i < size; ++i ) {
        // printf("%d ", (uint8_t)buf[i]);
        write( confd, buf+i, 1 );
    }
    // printf("\n");
    return size;
}


void console_link_gps( struct gps *gpspacket ) {
    static const uint8_t skip_count = 2;
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
    buf[0] = GPS_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct gps);
    buf[0] = size; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( (uint8_t *)gpspacket, size );

    // check sum (2 bytes)
    ugear_cksum( GPS_PACKET, size, (uint8_t *)gpspacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_imu( struct imu *imupacket ) {
    static const uint8_t skip_count = 5;
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
    buf[0] = IMU_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct imu);
    buf[0] = size; buf[1] = 0;
    // printf("imu size = %d\n", size);
    console_write( buf, 1 );

    // packet data
    uint8_t bytes = console_write( (uint8_t *)imupacket, size );
    if ( bytes != size ) {
      printf("Only wrote %d imu bytes out of %d\n", bytes, size);
    }

    // check sum (2 bytes)
    ugear_cksum( IMU_PACKET, size, (uint8_t *)imupacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_nav( struct nav *navpacket ) {
    static const uint8_t skip_count = 2;
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
    buf[0] = NAV_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct nav);
    buf[0] = size; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( (uint8_t *)navpacket, size );

    // check sum (2 bytes)
    ugear_cksum( NAV_PACKET, size, (uint8_t *)navpacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_servo( struct servo *servopacket ) {
    static const uint8_t skip_count = 5;
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
    buf[0] = SERVO_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct servo);
    buf[0] = size; buf[1] = 0;
    // printf("servo size = %d\n", size);
    console_write( buf, 1 );

    // packet data
    uint8_t bytes = console_write( (uint8_t *)servopacket, size );
    // uint8_t *tmp = (uint8_t *)servopacket;
    // printf("%d %d %d %d\n", tmp[0], tmp[1], tmp[2], tmp[3] );

    if ( bytes != size ) {
      printf("Only wrote %d servo bytes out of %d\n", bytes, size);
    }

    // check sum (2 bytes)
    ugear_cksum( SERVO_PACKET, size, (uint8_t *)servopacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_health( struct health *healthpacket ) {
    uint8_t buf[3];
    uint8_t size;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = HEALTH_PACKET; buf[1] = 0;
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
    ugear_cksum( HEALTH_PACKET, size, (uint8_t *)healthpacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


static void console_link_execute_command( const string command ) {
    vector <string> token = split( command, "," );

    if ( token.size() < 1 ) {
        // no valide tokens
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
        double alt_ft = atof( token[3].c_str() );
        double course_deg = atof( token[4].c_str() );
        SGWayPoint wp( lon, lat, alt_ft * SG_FEET_TO_METER );
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
        if ( token[1] == "alt" ) {
            double alt = atof( token[2].c_str() );
            SGPropertyNode_ptr target_altitude_ft
                = fgGetNode( "/autopilot/settings/target-altitude-ft", true );
            target_altitude_ft->setDoubleValue( alt );
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

    int result = read( confd, buf, 1 );
    while ( result == 1 && buf[0] != '\n' ) {
        command_buf[command_counter] = buf[0];
        command_counter++;
        result = read( confd, buf, 1 );
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
    
    // FILE *debug;
    // debug = fopen("/tmp/debug.txt", "a");
    // fprintf(debug, "Received command: '%s'\n", command_buf);
    // fclose(debug);

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

    // debug = fopen("/tmp/debug.txt", "a");
    // fprintf(debug, " cmd: '%s' nmea: '%s' '%s'\n", cmd.c_str(),
    //         nmea_sum.c_str(), cmd_sum);
    // fclose(debug);

    if ( nmea_sum.c_str()[0] != cmd_sum[0]
         || nmea_sum.c_str()[1] != cmd_sum[1])
    {
        // checksum failure
        // debug = fopen("/tmp/debug.txt", "a");
        // fprintf(debug, "check sum failure\n");
        // fclose(debug);
        return false;
    }

    // parse the command
    string::size_type pos = cmd.find_first_of(",");
    if ( pos == string::npos ) {
        // bogus command
        return false;
    }

    FILE *debug = fopen("/mnt/mmc/debug.txt", "a");
    fprintf(debug, "command: %s\n", cmd.c_str());
    fclose(debug);

    // extract command sequence number
    string num = cmd.substr(0, pos);
    int sequence = atoi( num.c_str() );

    // remainder
    cmd = cmd.substr(pos + 1);

    // execute command
    // debug = fopen("/tmp/debug.txt", "a");
    // fprintf(debug, "Sequence: %d  Execute: '%s'\n", sequence, cmd.c_str());
    // fclose(debug);
    console_link_execute_command( cmd );

    // register that we've received this message correctly
    health_update_command_sequence(sequence);

    return true;
}
