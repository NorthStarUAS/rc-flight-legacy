/**
 * \file: gps_mediatek.cxx
 *
 * MediaTek 3329 MTX protocol driver
 *
 * Copyright (C) 2012 - Curtis L. Olson colson@atiak.com
 *
 */

// FIXME: VALIDATE NMEA CHECKSUMS!!!!

#include <errno.h>		// errno
#include <math.h>		// sin() cos()
#include <sys/types.h>		// open()
#include <sys/stat.h>		// open()
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset()
#include <sys/time.h>		// gettimeofday()
#include <time.h>		// mktime()
#include <string>

using std::string;

#include "include/globaldefs.h"

#include "comms/logging.h"
#include "math/SGMath.hxx"
#include "math/SGGeodesy.hxx"
#include "props/props.hxx"
#include "util/strutils.hxx"
#include "util/timing.h"
#include "gps_mgr.hxx"

#include "gps_mediatek.hxx"


// gpsd property nodes
static SGPropertyNode *configroot = NULL;

static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;
static SGPropertyNode *gps_satellites_node = NULL;
static SGPropertyNode *gps_fix_type_node = NULL;
static SGPropertyNode *gps_firmware_node = NULL;
static SGPropertyNode *gps_device_name_node = NULL;

static int fd = -1;
static string device_name = "/dev/ttyS0";


// initialize gpsd input property nodes
static void bind_input( SGPropertyNode *config ) {
    gps_device_name_node = config->getChild("device");
    if ( gps_device_name_node != NULL ) {
	device_name = gps_device_name_node->getStringValue();
    }
    configroot = config;
}


// initialize gpsd output property nodes 
static void bind_output( string rootname ) {
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_satellites_node = outputroot->getChild("satellites", 0, true);
    gps_fix_type_node = outputroot->getChild("fix-type", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);
    gps_firmware_node = outputroot->getChild("firmware", 0, true);
}


// send our configured init strings to configure gpsd the way we prefer
static bool gps_mediatek3329_open() {
    if ( display_on ) {
	printf("mediatek 3329 on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// Serial port settings
    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    config.c_cflag     = B57600 |  // bps rate
	// CRTSCTS | // output flow ctnl
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    // Enable non-blocking IO (one more time for good measure)
    fcntl(fd, F_SETFL, O_NONBLOCK);

    return true;
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


static int gps_send_cmd( string msg ) {
    char msg_sum[10];
    snprintf( msg_sum, 3, "%02X", calc_nmea_cksum(msg.c_str()) );
    string command = "$";
    command += msg;
    command += "*";
    command += msg_sum;
    if ( display_on ) {
	printf("sending '%s'\n", command.c_str());
    }
    command += "\r\n";
    
    int len = write( fd, command.c_str(), command.length() );

    return len;
}


void gps_mediatek3329_init( string rootname, SGPropertyNode *config ) {
    bind_input( config );
    bind_output( rootname );
    gps_mediatek3329_open();

    // send setup strings (reference command set from datasheets in
    // the documentation)

    unsigned int len = 0;

    // GGA and RMC at full rate:
    string cmd314 = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0";
    // GGA and RMC at full rate, GSV every 10th position report:
    // string cmd314 = "PMTK314,0,1,0,1,0,10,0,0,0,0,0,0,0,0,0,0,0";
    len = gps_send_cmd( cmd314 );
    if ( len != cmd314.length() + 6 ) {
	printf("MediaTek: error sending command: %s\n", cmd314.c_str());
    }

    // 200ms update rate (5hz)
    string cmd220 = "PMTK220,200";
    len = gps_send_cmd( cmd220 );
    if ( len != cmd220.length() + 6 ) {
	printf("MediaTek: error sending command: %s\n", cmd220.c_str());
    }

    // Enable WAAS $PMTK301,2
    string cmd301 = "PMTK301,2";
    len = gps_send_cmd( cmd301 );
    if ( len != cmd301.length() + 6 ) {
	printf("MediaTek: error sending command: %s\n", cmd301.c_str());
    }

    // Query firmware version $PMTK605
    string cmd604 = "PMTK605";
    len = gps_send_cmd( cmd604 );
    if ( len != cmd604.length() + 6 ) {
	printf("MediaTek: error sending command: %s\n", cmd604.c_str());
    }

    // Command to set baud should we ever want to change from the
    // default of 57,600: $PMTK251,38400*27 (up to 115200 should be
    // supported)
}


static double gtime_to_gsec( string gtime ) {
    string hour = gtime.substr(0, 2);
    string min = gtime.substr(2, 2);
    string sec = gtime.substr(4);

    double gsec	=
	atoi(hour.c_str()) * 3600.0
	+ atoi(min.c_str()) * 60.0
	+ atof(sec.c_str());

    return gsec;
}


static double date_time_to_unix_sec( string gdate, string gtime ) {
   string hour = gtime.substr(0, 2);
   string min = gtime.substr(2, 2);
   string isec = gtime.substr(4, 2);
   string fsec = gtime.substr(6);

   string day = gdate.substr(0, 2);
   string mon = gdate.substr(2, 2);
   string year = gdate.substr(4, 2);

   // printf("%s : %s : %s + 0%s  %s / %s / %s\n", hour.c_str(), min.c_str(),
   //        isec.c_str(), fsec.c_str(), day.c_str(), mon.c_str(),
   //        year.c_str() );

   struct tm t;
   t.tm_sec = atoi(isec.c_str());
   t.tm_min = atoi(min.c_str());
   t.tm_hour = atoi(hour.c_str());
   t.tm_mday = atoi(day.c_str());
   t.tm_mon = atoi(mon.c_str()) - 1;
   t.tm_year = atoi(year.c_str()) + 100;
   t.tm_gmtoff = 0;

   // force timezone to GMT/UTC so mktime() does the proper conversion
   tzname[0] = tzname[1] = "GMT";
   timezone = 0;
   daylight = 0;
   setenv("TZ", "UTC", 1);

   // printf("%d\n", mktime(&t));
   // printf("tzname[0]=%s, tzname[1]=%s, timezone=%d, daylight=%d\n",
   //        tzname[0], tzname[1], timezone, daylight);

   double result = (double)mktime(&t);
   result += atof( fsec.c_str() );

   return result;
}


static bool parse_nmea_msg( char *payload, int size )
{
    bool new_position = false;
    static bool set_system_time = false;
    static double last_gsec = 0.0;
    static double last_alt_m = 0.0;

    string msg = (string)payload;
    // printf("orig = %s\n", msg.c_str());

    // validate message
    if ( msg.length() < 9 ) {
        // bogus command
	if ( display_on ) { printf("mediatek: short command '%s'\n", msg.c_str()); }
        return false;
    }
 
    // save sender check sum
    string nmea_sum = msg.substr(msg.length() - 2);
    // printf("nmea_sum = %s\n", nmea_sum.c_str() );

    // trim down to core message
    msg = msg.substr(0, msg.length() - 3);
    // printf("nmea_msg = %s\n", msg.c_str());
    char msg_sum[10];
    snprintf( msg_sum, 3, "%02X", calc_nmea_cksum(msg.c_str()) );

    if ( nmea_sum.c_str()[0] != msg_sum[0]
         || nmea_sum.c_str()[1] != msg_sum[1])
    {
        // checksum failure
	if ( event_log_on ) {
	    event_log( "mediatek", "checksum failed for message:" );
	    event_log( "  mediatek", payload );
	}
	if ( display_on ) {
	    printf("mediatek: checksum failure %s (msg) != %02X (calc)\n", nmea_sum.c_str(), calc_nmea_cksum(msg.c_str()) );
	    printf("mediatek: failed message: %s\n", payload);
	}
        return false;
    }

    vector <string> token = split( msg, "," );
    if ( token.size() < 1 ) {
        // no valid tokens
        return false;
    }

    double dd = 0.0;
    double mm = 0.0;

    if ( token[0] == "GPGGA" && token.size() == 15 ) {
	// ex: GPGGA,163227,3321.173,N,11039.855,W,1,,,3333,F,,,,*0F

	double alt_m = 0.0;
	double vspeed_mps = 0.0;

	double gsec = gtime_to_gsec( token[1].c_str() );

	// double lat_deg = atof( token[2].c_str() );
	// if ( token[3] == "S" ) {
	//     lat_deg *= -1.0;
	// }

	// double lon_deg = atof( token[4].c_str() );
	// if ( token[5] == "W" ) {
	//     lon_deg *= -1.0;
	// }

	int fix_ind = atoi( token[6].c_str() );
	gps_fix_type_node->setIntValue( fix_ind );

	int num_sats = atoi( token[7].c_str() );
	gps_satellites_node->setIntValue( num_sats );

	float hdop = atof( token[8].c_str() );

	if ( fix_ind > 0 ) {
	    alt_m = atof( token[9].c_str() );
	    if ( token[10] == "F" ) {
		alt_m *= SG_FEET_TO_METER;
	    }
	    gps_alt_node->setDoubleValue( alt_m );

	    float geoid_sep_m = atof( token[11].c_str() );
	    if ( token[12] == "F" ) {
		geoid_sep_m *= SG_FEET_TO_METER;
	    }

	    // compute vertical speed
	    double dt = gsec - last_gsec;
	    if ( dt < 0.0 ) { dt += 86400; }
	    double da = alt_m - last_alt_m;
	    if ( dt > 0.001 ) {
		vspeed_mps = da / dt;
	    }
	    gps_vd_node->setDoubleValue( -vspeed_mps );

	    last_gsec = gsec;
	    last_alt_m = alt_m;

#if 0
	    if ( display_on ) {
		printf("gga (%.3f) %.2f %.3f %d\n",
		       gsec, alt_m, vspeed_mps, num_sats);
	    }
#endif
	}
    } else if ( token[0] == "GPRMC" && token.size() == 13 ) {
	// ex: $GPRMC,053740.000,A,2503.6319,N,12136.0099,E,2.69,79.65,100106,,,A*53
	double gsec = gtime_to_gsec( token[1].c_str() );

	if ( token[2] == "A" ) {
	    // for the mediatek, the gga string is sent, followed by
	    // the rmc string.  So we notify the system of new valid
	    // data whenever a valid rmc string is read.

	    new_position = true;
	    gps_timestamp_node->setDoubleValue( get_Time() );

	    // compute unix time (time_t)
	    double unix_sec = date_time_to_unix_sec( token[9], token[1] );
	    gps_unix_sec_node->setDoubleValue( unix_sec );
	    if ( ! set_system_time ) {
		set_system_time = true;
		time_t sec = (time_t)unix_sec;
		suseconds_t usec = (unix_sec - sec) * 1000000;
		struct timeval fulltime;
		fulltime.tv_sec = sec;
		fulltime.tv_usec = usec;
		settimeofday( &fulltime, NULL );
		get_Time( true ); 	// reset precise clock timer to zero
	    }

	    dd = atof( token[3].substr(0, 2).c_str() );
	    mm = atof( token[3].substr(2).c_str() );
	    double lat_deg = dd + (mm / 60.0);
	    if ( token[4] == "S" ) {
		lat_deg *= -1.0;
	    }
	    gps_lat_node->setDoubleValue( lat_deg );

	    dd = atof( token[5].substr(0, 3).c_str() );
	    mm = atof( token[5].substr(3).c_str() );
	    double lon_deg = dd + (mm / 60.0);
	    if ( token[6] == "W" ) {
		lon_deg *= -1.0;
	    }
	    gps_lon_node->setDoubleValue( lon_deg );

	    float speed_kts = atof( token[7].c_str() );
	    float course_deg = atof( token[8].c_str() );

	    // compute speed/course to vel NED
	    double speed_mps = speed_kts * SG_KT_TO_MPS;
	    double angle_rad = (90.0 - course_deg) * SGD_DEGREES_TO_RADIANS;
	    gps_vn_node->setDoubleValue( sin(angle_rad) * speed_mps );
	    gps_ve_node->setDoubleValue( cos(angle_rad) * speed_mps );

#if 0
	    if ( display_on ) {
		printf("rmc (%.3f) mps=%.1f deg=%.1f rad=%.3f vn=%.1f ve=%.1f\n",
		       unix_sec,
		       speed_mps, course_deg, angle_rad,
		       gps_vn_node->getDoubleValue(),
		       gps_ve_node->getDoubleValue());
	    }
#endif
	}
    } else if ( token[0] == "PMTK001" && token.size() == 3 ) {
	int cmd_id = atoi( token[1].c_str() );
	int status_id = atoi( token[2].c_str() );
	if ( status_id == 1 ) {
	    printf("MediaTek: received unsupported command: %d\n", cmd_id);
	} else if ( status_id == 2 ) {
	    printf("MediaTek: valid command, but action failed: %d\n", cmd_id);
	} else if ( status_id == 3 ) {
	    printf("MediaTek: command succeeded: %d\n", cmd_id);
	}
    } else if ( token[0] == "PMTK705" && token.size() == 5 ) {
	string firmware = token[1] + "," + token[2] + "," + token[3];
	gps_firmware_node->setStringValue( firmware.c_str() );
	printf("MediaTek: firmware rev: %s\n", firmware.c_str() );
    } else {
	if ( display_on ) {
	    printf("MediaTek Unknown NMEA Message = \"%s\"\n", payload);
	}
    }

    return new_position;
}


static bool read_mediatek3329() {
    static int state = 0;
    static int counter = 0;
    int len;
    char input[8];
    static char payload[257];	// 256+1 for a null string terminator

    // printf("read mediatek, entry state = %d\n", state);

    bool new_position = false;

    if ( state == 0 ) {
	counter = 0;
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    //printf("mt0 read: %d\n", (unsigned int)input[0]);
	    if ( input[0] == '$' ) {
		state = 1;
		break;
	    }
	    len = read( fd, input, 1 );
	    //printf("mt0 read: %d\n", (unsigned int)input[0]);
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	//printf("mt1 read: %d (%d)\n", (unsigned int)input[0], counter);
	while ( len > 0 && counter < 256 ) {
	    if ( input[0] == '\r' ) {
		payload[counter] = 0;
		state = 2;
		break;
	    } else {
		payload[counter++] = input[0];
	    }
	    len = read( fd, input, 1 );
	    //printf("mt1 read: %d (%d)\n", (unsigned int)input[0], counter);
	}
	if ( counter >= 256 ) {
	    state = 0;
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	//printf("mt2 read: %d\n", (unsigned int)input[0]);
	if ( len > 0 ) {
	    if ( input[0] == '\n' ) {
		// printf("mta: calling parser with '%s'\n", payload);
		new_position = parse_nmea_msg( payload, counter );
	    }
	    state = 0;
	}
	if ( counter >= 256 ) {
	    state = 0;
	}
    }

    return new_position;
}


bool gps_mediatek3329_update() {
    // run an iteration of the mediatek scanner/parser
    bool gps_data_valid = read_mediatek3329();

    return gps_data_valid;
 }


void gps_mediatek3329_close() {
}
