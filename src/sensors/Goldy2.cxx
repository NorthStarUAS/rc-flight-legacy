//
// FILE: Goldy2.cxx
// DESCRIPTION: aquire sensor data from Goldy2 FMU (via ethernet)
//

#include "python/pyprops.hxx"

#include <stdint.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>  // settimeofday()

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "init/globals.hxx"
//#include "math/SGMath.hxx"
//#include "math/SGGeodesy.hxx"
#include "util/geodesy.hxx"
#include "sensors/cal_temp.hxx"
#include "util/linearfit.hxx"
#include "util/netSocket.h"
//#include "util/poly1d.hxx"
#include "util/timing.h"

#include "util_goldy2.hxx"
#include "Goldy2.hxx"


static netSocket sock;
static int port = 0;
static int gps_fix_value = 0;
static const int rcin_channels = 16;
static uint16_t rcin[rcin_channels];
static string pilot_mapping[rcin_channels]; // channel->name mapping
static bool pilot_symmetric[rcin_channels]; // normalization symmetry flag
static bool pilot_invert[rcin_channels];    // invert input flag
static bool pilot_wing_mixing = false;	    // for now this is a bit of a hack

#define SBUS_CENTER 992
#define SBUS_HALF_RANGE 820
#define SBUS_RANGE (SBUS_HALF_RANGE * 2)
#define SBUS_MIN (SBUS_CENTER - SBUS_HALF_RANGE)
#define SBUS_MAX (SBUS_CENTER + SBUS_HALF_RANGE)

static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode airdata_node;
static pyPropertyNode pilot_node;

static bool master_init = false;
static bool imu_inited = false;
static bool airdata_inited = false;
static bool gps_inited = false;
static bool pilot_input_inited = false;
static string imu_orientation = "normal";

static double imu_timestamp = 0.0;
static LinearFitFilter imu_offset(200.0, 0.01);

static AuraCalTemp p_cal;
static AuraCalTemp q_cal;
static AuraCalTemp r_cal;
static AuraCalTemp ax_cal;
static AuraCalTemp ay_cal;
static AuraCalTemp az_cal;
static Matrix4d mag_cal;

struct imu_sensors_t {
    uint64_t time;
    uint16_t type;
    uint16_t valid;
    uint32_t imu_time_sync_in;
    float magX;
    float magY;
    float magZ;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
    float pressure;
    uint32_t att_time_sync_in;
    float yaw;
    float pitch;
    float roll;    
} imu_sensors;


// initialize goldy2_imu input property nodes
static void bind_input() {
    pyPropertyNode config = pyGetNode("/config/sensors/Goldy2", true);
    if ( config.hasChild("port") ) {
	port = config.getLong("port");
    }
    printf("Goldy2 port = %d\n", port);
}


// initialize imu output property nodes 
static void bind_imu_output( string output_path ) {
    if ( imu_inited ) {
        return;
    }
    imu_node = pyGetNode(output_path, true);

    imu_inited = true;
}


// initialize airdata output property nodes 
static void bind_airdata_output( string output_path ) {
    if ( airdata_inited ) {
	return;
    }
    airdata_node = pyGetNode(output_path, true);

    airdata_inited = true;
}


// initialize gps output property nodes
static void bind_gps_output( string output_path ) {
    if ( gps_inited ) {
        return;
    }
    gps_node = pyGetNode(output_path, true);

    gps_inited = true;
}

// initialize pilot output property nodes
static void bind_pilot_controls( string output_path ) {
    if ( pilot_input_inited ) {
        return;
    }
    pilot_node = pyGetNode(output_path, true);
    pilot_node.setLen("channel", rcin_channels, 0);

    pilot_input_inited = true;
}


bool goldy2_init() {
    if ( master_init ) {
        return true;
    }

    bind_input();

    // open a UDP socket
    if ( ! sock.open( false ) ) {
	printf("Error opening goldy2 input socket\n");
	return false;
    }

    // bind ...
    if ( sock.bind( "", port ) == -1 ) {
	printf("error binding to port %d\n", port );
	return false;
    }

#if 0 // we are using blocking for main loop sync now
    // don't block waiting for input
    sock.setBlocking( false );
#endif

    master_init = true;

    return true;
}


// function prototypes
bool goldy2_imu_init( string output_path, pyPropertyNode *config ) {
    if ( ! goldy2_init() ) {
        return false;
    }

    bind_imu_output( output_path );
    
    if ( config->hasChild("imu_orientation") ) {
	imu_orientation = config->getString("imu_orientation");
    }

    if ( config->hasChild("calibration") ) {
	pyPropertyNode cal = config->getChild("calibration");
	double min_temp = 27.0;
	double max_temp = 27.0;
	if ( cal.hasChild("min_temp_C") ) {
	    min_temp = cal.getDouble("min_temp_C");
	}
	if ( cal.hasChild("max_temp_C") ) {
	    max_temp = cal.getDouble("max_temp_C");
	}
	
	pyPropertyNode p_node = cal.getChild("p");
	p_cal.init( &p_node, min_temp, max_temp );
	pyPropertyNode q_node = cal.getChild("q");
	q_cal.init( &q_node, min_temp, max_temp );
	pyPropertyNode r_node = cal.getChild("r");
	r_cal.init( &r_node, min_temp, max_temp );

	pyPropertyNode ax_node = cal.getChild("ax");
	ax_cal.init( &ax_node, min_temp, max_temp );
	pyPropertyNode ay_node = cal.getChild("ay");
	ay_cal.init( &ay_node, min_temp, max_temp );
	pyPropertyNode az_node = cal.getChild("az");
	az_cal.init( &az_node, min_temp, max_temp );

	if ( cal.hasChild("mag_affine") ) {
	    string tokens_str = cal.getString("mag_affine");
	    vector<string> tokens = split(tokens_str);
	    if ( tokens.size() == 16 ) {
		int r = 0, c = 0;
		for ( unsigned int i = 0; i < 16; i++ ) {
		    mag_cal(r,c) = atof(tokens[i].c_str());
		    c++;
		    if ( c > 3 ) {
			c = 0;
			r++;
		    }
		}
	    } else {
		printf("ERROR: wrong number of elements for mag_cal affine matrix!\n");
		mag_cal.setIdentity();
	    }
	} else {
	    mag_cal.setIdentity();
	}

	// save the imu calibration parameters with the data file so that
	// later the original raw sensor values can be derived.
        write_imu_calibration( &cal );
    }

    return true;
}


// function prototypes
bool goldy2_airdata_init( string output_path ) {
    if ( ! goldy2_init() ) {
        return false;
    }

    bind_airdata_output( output_path );

    return true;
}

bool goldy2_gps_init( string output_path  ) {
    if ( ! goldy2_init() ) {
        return false;
    }

    bind_gps_output( output_path );

    return true;
}

bool goldy2_pilot_init( string output_path, pyPropertyNode *config ) {
    if ( ! goldy2_init() ) {
        return false;
    }

    bind_pilot_controls( output_path );
    
    if ( config->hasChild("channel") ) {
	for ( int i = 0; i < rcin_channels; i++ ) {
	    pilot_mapping[i] = config->getString("channel", i);
	    printf("pilot input: channel %d maps to %s\n", i, pilot_mapping[i].c_str());
	}
    }
    if ( config->hasChild("symmetric") ) {
	for ( int i = 0; i < rcin_channels; i++ ) {
	    pilot_symmetric[i] = config->getBool("symmetric", i);
	    printf("pilot input: channel %d symmetry %d\n", i, pilot_symmetric[i]);
	}
    }
    if ( config->hasChild("invert") ) {
	for ( int i = 0; i < rcin_channels; i++ ) {
	    pilot_invert[i] = config->getBool("invert", i);
	    printf("pilot input: channel %d invert %d\n", i, pilot_invert[i]);
	}
    }

    if ( config->hasChild("wing_mixing") ) {
	pilot_wing_mixing = config->getBool("wing_mixing");
    }
    
    return true;
}

static bool goldy2_imu_update_internal() {
    static uint64_t last_imu_internal_time = 0;

    double p_raw = 0.0, q_raw = 0.0, r_raw = 0.0;
    double ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0;
    double hx_raw = 0.0, hy_raw = 0.0, hz_raw = 0.0;
    if ( imu_orientation == "" || imu_orientation == "normal" ) {
	p_raw = imu_sensors.gyroX;
	q_raw = imu_sensors.gyroY;
	r_raw = imu_sensors.gyroZ;
	ax_raw = imu_sensors.accelX;
	ay_raw = imu_sensors.accelY;
	az_raw = imu_sensors.accelZ;
	hx_raw = imu_sensors.magX;
	hy_raw = imu_sensors.magY;
	hz_raw = imu_sensors.magZ;
    } else if ( imu_orientation == "edgewise" ) {
	p_raw = imu_sensors.gyroX;
	q_raw = -imu_sensors.gyroZ;
	r_raw = imu_sensors.gyroY;
	ax_raw = imu_sensors.accelX;
	ay_raw = -imu_sensors.accelZ;
	az_raw = imu_sensors.accelY;
	hx_raw = imu_sensors.magX;
	hy_raw = -imu_sensors.magZ;
	hz_raw = imu_sensors.magY;
    } else if ( imu_orientation == "reverse" ) {
	p_raw = -imu_sensors.gyroX;
	q_raw = -imu_sensors.gyroY;
	r_raw = imu_sensors.gyroZ;
	ax_raw = -imu_sensors.accelX;
	ay_raw = -imu_sensors.accelY;
	az_raw = imu_sensors.accelZ;
	hx_raw = -imu_sensors.magX;
	hy_raw = -imu_sensors.magY;
	hz_raw = imu_sensors.magZ;
    } else {
	printf("unknown imu orientation: %s\n", imu_orientation.c_str());
    }
    double temp_C = imu_sensors.temp;
	
    imu_node.setDouble( "p_rad_sec", p_cal.calibrate(p_raw, temp_C) );
    imu_node.setDouble( "q_rad_sec", q_cal.calibrate(q_raw, temp_C) );
    imu_node.setDouble( "r_rad_sec", r_cal.calibrate(r_raw, temp_C) );
    imu_node.setDouble( "ax_mps_sec", ax_cal.calibrate(ax_raw, temp_C) );
    imu_node.setDouble( "ay_mps_sec", ay_cal.calibrate(ay_raw, temp_C) );
    imu_node.setDouble( "az_mps_sec", az_cal.calibrate(az_raw, temp_C) );

    imu_node.setDouble( "hx_raw", hx_raw );
    imu_node.setDouble( "hy_raw", hy_raw );
    imu_node.setDouble( "hz_raw", hz_raw );
	
    Vector4d hs((double)hx_raw, (double)hy_raw, (double)hz_raw, 1.0);
    Vector4d hc = mag_cal * hs;
    imu_node.setDouble( "hx", hc(0) );
    imu_node.setDouble( "hy", hc(1) );
    imu_node.setDouble( "hz", hc(2) );

    imu_node.setDouble( "temp_C", imu_sensors.temp );
    imu_node.setDouble( "pressure", imu_sensors.pressure );
    imu_node.setDouble( "roll_deg", imu_sensors.roll );
    imu_node.setDouble( "pitch_deg", imu_sensors.pitch );
    imu_node.setDouble( "yaw_deg", imu_sensors.yaw );

    // timestamp dance: this is a little jig that I do to make a
    // more consistent time stamp that still is in the host
    // reference frame.  Assumes the FMU clock drifts relative to
    // host clock.  Assumes the FMU imu stamp dt is very stable.
    // Assumes the host system is not-real time and there may be
    // momentary external disruptions to execution. The code
    // estimates the error (difference) between FMU clock and
    // host clock.  Then builds a real time linear fit of FMU
    // clock versus difference with the host.  This linear fit is
    // used to estimate the current error (smoothly), add that to
    // the FMU clock and derive a more regular/stable IMU time
    // stamp (versus just sampling current host time.)
	
    double imu_remote_sec = (double)imu_sensors.time / 1000000.0;
    double diff = imu_timestamp - imu_remote_sec;
    if ( last_imu_internal_time > imu_sensors.time ) {
	events->log("FMU", "micros() rolled over\n");
	imu_offset.reset();
    }
    imu_offset.update(imu_remote_sec, diff);
    double fit_diff = imu_offset.get_value(imu_remote_sec);
    // printf("imu = %.6f fit_diff = %.6f  diff = %.6f  ts = %.6f\n",
    //        imu_remote_sec, fit_diff, diff, imu_remote_sec + fit_diff );
    imu_node.setDouble( "timestamp", imu_remote_sec + fit_diff );

    last_imu_internal_time = imu_sensors.time;

    return true;
}


// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count )
{
    return; // beaglebone is little endian and so is the ublox ...

    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
}


static bool parse_ublox_msg( uint8_t msg_class, uint8_t msg_id,
			     uint16_t payload_length, uint8_t *payload )
{
    bool new_position = false;
    static bool set_system_time = false;

    if ( msg_class == 0x01 && msg_id == 0x02 ) {
	// NAV-POSLLH
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 20, 4);
	my_swap( payload, 24, 4);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	int32_t lon = *((int32_t *)(p+4));
	int32_t lat = *((int32_t *)(p+8));
	int32_t height = *((int32_t *)(p+12));
	int32_t hMSL = *((int32_t *)(p+16));
	// uint32_t hAcc = *((uint32_t *)(p+20));
	// uint32_t vAcc = *((uint32_t *)(p+24));
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-posllh (%d) %d %d %d %d\n",
		       iTOW, lon, lat, height, hMSL);
	    }
	}
    } else if ( msg_class == 0x01 && msg_id == 0x06 ) {
	// NAV-SOL
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 2);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 20, 4);
	my_swap( payload, 24, 4);
	my_swap( payload, 28, 4);
	my_swap( payload, 32, 4);
	my_swap( payload, 36, 4);
	my_swap( payload, 40, 4);
	my_swap( payload, 44, 2);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)(p+0));
	int32_t fTOW = *((int32_t *)(p+4));
	int16_t week = *((int16_t *)(p+8));
	uint8_t gpsFix = p[10];
	// uint8_t flags = p[11];
	int32_t ecefX = *((int32_t *)(p+12));
	int32_t ecefY = *((int32_t *)(p+16));
	int32_t ecefZ = *((int32_t *)(p+20));
	// uint32_t pAcc = *((uint32_t *)(p+24));
	int32_t ecefVX = *((int32_t *)(p+28));
	int32_t ecefVY = *((int32_t *)(p+32));
	int32_t ecefVZ = *((int32_t *)(p+36));
	// uint32_t sAcc = *((uint32_t *)(p+40));
	// uint16_t pDOP = *((uint16_t *)(p+44));
	uint8_t numSV = p[47];
	if ( display_on && 0 ) {
	    printf("nav-sol (%d) %d %d %d %d %d [ %d %d %d ]\n",
		   gpsFix, iTOW, fTOW, ecefX, ecefY, ecefZ,
		   ecefVX, ecefVY, ecefVZ);
	}
	Vector3d ecef( ecefX / 100.0, ecefY / 100.0, ecefZ / 100.0 );
	Vector3d wgs84 = ecef2lla(ecef);
	Quaterniond ecef2ned = fromLonLatRad(wgs84[1], wgs84[0]);
	Vector3d vel_ecef( ecefVX / 100.0, ecefVY / 100.0, ecefVZ / 100.0 );
	Vector3d vel_ned = quat_backtransform(ecef2ned, vel_ecef);
	if ( display_on && 0 ) {
	    printf("my vel ned = %.2f %.2f %.2f\n", vel_ned.x(), vel_ned.y(), vel_ned.z());
	    printf("wgs84 = %.10f, %.10f, %.2f\n",
                   wgs84[0] * 180.0 / M_PI, wgs84[1] * 180.0 / M_PI, wgs84[2] );
	}

 	gps_node.setLong( "satellites", numSV );
 	gps_fix_value = gpsFix;
	if ( gps_fix_value == 0 ) {
	    gps_node.setLong( "status", 0 );
	} else if ( gps_fix_value == 1 || gps_fix_value == 2 ) {
	    gps_node.setLong( "status", 1 );
	} else if ( gps_fix_value == 3 ) {
	    gps_node.setLong( "status", 2 );
	}

	if ( fabs(ecefX) > 650000000
	     || fabs(ecefY) > 650000000
	     || fabs(ecefZ) > 650000000 ) {
	    // earth radius is about 6371km (637,100,000 cm).  If one
	    // of the ecef coordinates is beyond this radius we know
	    // we have bad data.  This means we won't toss data until
	    // above about 423,000' MSL
	    events->log( "ublox", "received bogus ecef data" );
	} else if ( wgs84[2] > 60000 ) {
	    // sanity check: assume altitude > 60k meters (200k feet) is bad
	} else if ( wgs84[2] < -1000 ) {
	    // sanity check: assume altitude < -1000 meters (-3000 feet) is bad
	} else if ( gpsFix == 3 ) {
	    // passed basic sanity checks and gps is reporting a 3d fix
	    new_position = true;
	    gps_node.setDouble( "timestamp", get_Time() );
	    gps_node.setDouble( "latitude_deg", wgs84[0] * 180.0 / M_PI );
	    gps_node.setDouble( "longitude_deg", wgs84[1] * 180.0 / M_PI );
	    gps_node.setDouble( "altitude_m", wgs84[2] );
	    gps_node.setDouble( "vn_ms", vel_ned.x() );
	    gps_node.setDouble( "ve_ms", vel_ned.y() );
	    gps_node.setDouble( "vd_ms", vel_ned.z() );
	    // printf("        %.10f %.10f %.2f - %.2f %.2f %.2f\n",
	    //        wgs84.getLatitudeDeg(),
	    //        wgs84.getLongitudeDeg(),
	    //        wgs84.getElevationM(),
	    //        vel_ned.x(), vel_ned.y(), vel_ned.z() );

	    double julianDate = (week * 7.0) + 
		(0.001 * iTOW) / 86400.0 +  //86400 = seconds in 1 day
		2444244.5; // 2444244.5 Julian date of GPS epoch (Jan 5 1980 at midnight)
	    julianDate = julianDate - 2440587.5; // Subtract Julian Date of Unix Epoch (Jan 1 1970)

	    double unixSecs = julianDate * 86400.0;
	    // double unixFract = unixSecs - floor(unixSecs);
	    // struct timeval time;
	    gps_node.setDouble( "unix_time_sec", unixSecs );
#if 0
	    if ( unixSecs > 1263154775 && !set_system_time) {
		printf("Setting system time to %.3f\n", unixSecs);
		set_system_time = true;
		time.tv_sec = floor(unixSecs);
		time.tv_usec = floor(unixFract * 1000000.);
		settimeofday(&time, NULL);
	    }
#endif
	}
    } else if ( msg_class == 0x01 && msg_id == 0x07 ) {
	// NAV-PVT
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 2);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 24, 4);
	my_swap( payload, 28, 4);
	my_swap( payload, 32, 4);
	my_swap( payload, 36, 4);
	my_swap( payload, 40, 4);
	my_swap( payload, 44, 4);
	my_swap( payload, 48, 4);
	my_swap( payload, 52, 4);
	my_swap( payload, 56, 4);
	my_swap( payload, 60, 4);
	my_swap( payload, 64, 4);
	my_swap( payload, 68, 4);
	my_swap( payload, 72, 4);
	my_swap( payload, 76, 2);
	my_swap( payload, 78, 2);
	my_swap( payload, 80, 4);

	uint8_t *p = payload;
	//uint32_t iTOW = *((uint32_t *)p+0);
	int16_t year = *((uint16_t *)(p+4));
	uint8_t month = p[6];
	uint8_t day = p[7];
	uint8_t hour = p[8];
	uint8_t min = p[9];
	uint8_t sec = p[10];
	//uint8_t valid = p[11];
	uint32_t tAcc = *((uint32_t *)(p+12));
	int32_t nano = *((int32_t *)(p+16));
	uint8_t gpsFix = p[20];
	//uint8_t flags = p[21];
	uint8_t numSV = p[23];
	int32_t lon = *((int32_t *)(p+24));
	int32_t lat = *((int32_t *)(p+28));
	//int32_t height = *((int32_t *)(p+32));
	int32_t hMSL = *((int32_t *)(p+36));
	uint32_t hAcc = *((uint32_t *)(p+40));
	uint32_t vAcc = *((uint32_t *)(p+44));
	int32_t velN = *((int32_t *)(p+48));
	int32_t velE = *((int32_t *)(p+52));
	int32_t velD = *((int32_t *)(p+56));
	uint32_t gSpeed = *((uint32_t *)(p+60));
	int32_t heading = *((int32_t *)(p+64));
	//uint32_t sAcc = *((uint32_t *)(p+68));
	//uint32_t headingAcc = *((uint32_t *)(p+72));
	uint16_t pDOP = *((uint16_t *)(p+76));

 	gps_fix_value = gpsFix;
	if ( gps_fix_value == 0 ) {
	    gps_node.setLong( "status", 0 );
	} else if ( gps_fix_value == 1 || gps_fix_value == 2 ) {
	    gps_node.setLong( "status", 1 );
	} else if ( gps_fix_value == 3 ) {
	    gps_node.setLong( "status", 2 );
	}
	// printf("fix: %d lon: %.8f lat: %.8f\n", gpsFix, (double)lon, (double)lat);

	if ( gpsFix == 3 ) {
	    // gps thinks we have a good position
 	    new_position = true;

	    gps_node.setDouble( "timestamp", get_Time() );

	    struct tm gps_time;
	    gps_time.tm_sec = sec;
	    gps_time.tm_min = min;
	    gps_time.tm_hour = hour;
	    gps_time.tm_mday = day;
	    gps_time.tm_mon = month - 1;
	    gps_time.tm_year = year - 1900;
	    double unix_sec = (double)mktime( &gps_time ) - timezone;
	    unix_sec += nano / 1000000000.0;
	    gps_node.setDouble( "unix_time_sec", unix_sec );
	    gps_node.setDouble( "time_accuracy_ns", tAcc );
	    
	    gps_node.setLong( "satellites", numSV );
	    
	    gps_node.setDouble( "latitude_deg", (double)lat / 10000000.0);
	    gps_node.setDouble( "longitude_deg", (double)lon / 10000000.0);
	    gps_node.setDouble( "altitude_m", (float)hMSL / 1000.0 );
	    gps_node.setDouble( "vn_ms", (float)velN / 1000.0 );
	    gps_node.setDouble( "ve_ms", (float)velE / 1000.0 );
	    gps_node.setDouble( "vd_ms", (float)velD / 1000.0 );
	    gps_node.setDouble( "horiz_accuracy_m", hAcc );
	    gps_node.setDouble( "vert_accuracy_m", vAcc );
	    gps_node.setDouble( "groundspeed_ms", gSpeed / 1000.0 );
	    gps_node.setDouble( "groundtrack_deg", heading / 100000.0 );
	    gps_node.setDouble( "heading_accuracy_deg", hAcc / 100000.0 );
	    gps_node.setDouble( "pdop", pDOP / 100.0 );
	}
   } else if ( msg_class == 0x01 && msg_id == 0x12 ) {
	// NAV-VELNED
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 20, 4);
	my_swap( payload, 24, 4);
	my_swap( payload, 28, 4);
	my_swap( payload, 32, 4);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	int32_t velN = *((int32_t *)(p+4));
	int32_t velE = *((int32_t *)(p+8));
	int32_t velD = *((int32_t *)(p+12));
	uint32_t speed = *((uint32_t *)(p+16));
	// uint32_t gspeed = *((uint32_t *)(p+20));
	int32_t heading = *((int32_t *)(p+24));
	// uint32_t sAcc = *((uint32_t *)(p+28));
	// uint32_t cAcc = *((uint32_t *)(p+32));
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-velned (%d) %.2f %.2f %.2f s = %.2f h = %.2f\n",
		       iTOW, velN / 100.0, velE / 100.0, velD / 100.0,
		       speed / 100.0, heading / 100000.0);
	    }
	}
    } else if ( msg_class == 0x01 && msg_id == 0x21 ) {
	// NAV-TIMEUTC
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 4);
	my_swap( payload, 8, 4);
	my_swap( payload, 12, 2);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)(p+0));
	// uint32_t tAcc = *((uint32_t *)(p+4));
	int32_t nano = *((int32_t *)(p+8));
	int16_t year = *((int16_t *)(p+12));
	uint8_t month = p[14];
	uint8_t day = p[15];
	uint8_t hour = p[16];
	uint8_t min = p[17];
	uint8_t sec = p[18];
	uint8_t valid = p[19];
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("nav-timeutc (%d) %02x %04d/%02d/%02d %02d:%02d:%02d\n",
		       iTOW, valid, year, month, day, hour, min, sec);
	    }
	}
	if ( !set_system_time && year > 2009 ) {
	    set_system_time = true;
	    printf("set system clock: nav-timeutc (%d) %02x %04d/%02d/%02d %02d:%02d:%02d\n",
		   iTOW, valid, year, month, day, hour, min, sec);
	    struct tm gps_time;
	    gps_time.tm_sec = sec;
	    gps_time.tm_min = min;
	    gps_time.tm_hour = hour;
	    gps_time.tm_mday = day;
	    gps_time.tm_mon = month - 1;
	    gps_time.tm_year = year - 1900;
	    time_t unix_sec = mktime( &gps_time ) - timezone;
	    printf("gps->unix time = %d\n", (int)unix_sec);
	    struct timeval fulltime;
	    fulltime.tv_sec = unix_sec;
	    fulltime.tv_usec = nano / 1000;
	    settimeofday( &fulltime, NULL );
	}
    } else if ( msg_class == 0x01 && msg_id == 0x30 ) {
	// NAV-SVINFO (partial parse)
	my_swap( payload, 0, 4);

	uint8_t *p = payload;
	// uint32_t iTOW = *((uint32_t *)(p+0));
	uint8_t numCh = p[4];
	// uint8_t globalFlags = p[5];
	int satUsed = 0;
	for ( int i = 0; i < numCh; i++ ) {
	    // uint8_t satid = p[9 + 12*i];
	    // uint8_t flags = p[10 + 12*i];
	    uint8_t quality = p[11 + 12*i];
	    // printf(" chn=%d satid=%d flags=%d quality=%d\n", i, satid, flags, quality);
	    if ( quality > 3 ) {
		satUsed++;
	    }
	}
 	// gps_satellites_node.setLong( satUsed );
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("Satellite count = %d/%d\n", satUsed, numCh);
	    }
	}
    } else {
	if ( display_on ) {
	    printf("UBLOX msg class = %d  msg id = %d\n",
		   msg_class, msg_id);
	}
    }

    return new_position;
}


static bool scan_ublox(uint8_t *packet, int packet_len) {
    static int state = 0;
    static int msg_class = 0, msg_id = 0;
    static int length_lo = 0, length_hi = 0, payload_length = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    static uint8_t payload[500];
    int pos = 0;

    // printf("read ublox, entry state = %d\n", state);

    bool new_position = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	while ( pos < packet_len && packet[pos] != 0xB5 ) {
	    // printf( "state0: len = %d val = %2X\n", len, packet[pos] );
	    pos += 1;
	}
	if ( pos < packet_len && packet[pos] == 0xB5 ) {
	    // printf( "read 0xB5\n");
	    state++;
	}
    }
    if ( state == 1 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    if ( packet[pos] == 0x62 ) {
		// printf( "read 0x62\n");
		state++;
	    } else if ( packet[pos] == 0xB5 ) {
		// printf( "read 0xB5\n");
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    msg_class = packet[pos];
	    cksum_A += packet[pos];
	    cksum_B += cksum_A;
	    // printf( "msg class = %d\n", msg_class );
	    state++;
	}
    }
    if ( state == 3 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    msg_id = packet[pos];
	    cksum_A += packet[pos];
	    cksum_B += cksum_A;
	    // printf( "msg id = %d\n", msg_id );
	    state++;
	}
    }
    if ( state == 4 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    length_lo = packet[pos];
	    cksum_A += packet[pos];
	    cksum_B += cksum_A;
	    state++;
	}
    }
    if ( state == 5 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    length_hi = packet[pos];
	    cksum_A += packet[pos];
	    cksum_B += cksum_A;
	    payload_length = length_hi*256 + length_lo;
	    // printf( "payload len = %d\n", payload_length );
	    if ( payload_length > 400 ) {
		state = 0;
	    } else {
		state++;
	    }
	}
    }
    if ( state == 6 ) {
	pos += 1;
	while ( pos < packet_len ) {
	    payload[counter++] = packet[pos];
	    // printf( "%02X ", packet[pos] );
	    cksum_A += packet[pos];
	    cksum_B += cksum_A;
	    if ( counter >= payload_length ) {
		break;
	    }
	    pos += 1;
	}

	if ( counter >= payload_length ) {
	    state++;
	    // printf( "\n" );
	}
    }
    if ( state == 7 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    cksum_lo = packet[pos];
	    state++;
	}
    }
    if ( state == 8 ) {
	pos += 1;
	if ( pos < packet_len ) {
	    cksum_hi = packet[pos];
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
		// printf( "checksum passes (%d)!\n", msg_id );
		new_position = parse_ublox_msg( msg_class, msg_id,
						payload_length, payload );
		state++;
	    } else {
		if ( display_on && 0 ) {
		    printf("checksum failed %d %d (computed) != %d %d (message)\n",
			   cksum_A, cksum_B, cksum_lo, cksum_hi );
		}
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    return new_position;
}

// parse packets
static int goldy2_parse( uint8_t *buf, int size ) {
    if ( size < 8 ) {
        printf("goldy packet corruption!\n");
	return 0;
    }
    // printf("  header = %c %c %c\n", buf[0], buf[1], buf[2]);
    // printf("  type id = 0x%02x size = %d\n", buf[3], size);
    int len = buf[4] + 256*buf[5];
    // printf("  package len = %d\n", len);
    uint16_t CRC = buf[6+len] + 256*buf[7+len];
    // printf("  CRC = %d\n", CRC);
    // printf("  Computed CRC = %d\n", utilCRC16(buf+3, len+3, 0));
    if ( CRC != utilCRC16(buf+3, len+3, 0) ) {
	printf("goldy packet CRC mismatch!\n");
	return 0;
    }

    // check header
    if ( buf[0] != 'U' || buf[1] != 'M' || buf[2] != 'N' ) {
	printf("goldy packet header invalid\n");
	return 0;
    }
    if ( buf[3] == 0x81 && len == 76 ) {
	// IMU packet
	imu_timestamp = get_Time();
        uint8_t *payload = buf + 6;
	uint64_t time_ls = *(uint32_t *)payload; payload += 4;
	uint64_t time_ms = *(uint32_t *)payload; payload += 4;
	// printf("time1 = %llu time2 = %llu\n", time_ls, time_ms);
	imu_sensors.time = time_ls + 4294967295U * time_ms;
        // for ( int i = 0; i < 8; ++i ) {
	//     printf("%02x ", *(buf + 6 + i));
	// }
	// printf("\n");
	// printf("time = %llu\n", imu_sensors.time);
	imu_sensors.type = *(uint16_t *)payload; payload += 2;
	// printf("type = %d\n", imu_sensors.type);
	imu_sensors.valid = *(uint16_t *)payload; payload += 2;
	// printf("valid = %d\n", imu_sensors.valid);
	imu_sensors.imu_time_sync_in = *(uint32_t *)payload; payload += 4;
	// printf("imu sync time = %d\n", imu_sensors.imu_time_sync_in);
	imu_sensors.magX = *(float *)payload; payload += 4;
	imu_sensors.magY = *(float *)payload; payload += 4;
	imu_sensors.magZ = *(float *)payload; payload += 4;
	imu_sensors.accelX = *(float *)payload; payload += 4;
	imu_sensors.accelY = *(float *)payload; payload += 4;
	imu_sensors.accelZ = *(float *)payload; payload += 4;
	// printf("          accel = %.3f %.3f %.3f\n", imu_sensors.accelX, imu_sensors.accelY, imu_sensors.accelZ);
	imu_sensors.gyroX = *(float *)payload; payload += 4;
	imu_sensors.gyroY = *(float *)payload; payload += 4;
	imu_sensors.gyroZ = *(float *)payload; payload += 4;
	// printf("          gyro = %.3f %.3f %.3f\n", imu_sensors.gyroX, imu_sensors.gyroY, imu_sensors.gyroZ);
	imu_sensors.temp = *(float *)payload; payload += 4;
	imu_sensors.pressure = *(float *)payload; payload += 4;
	imu_sensors.att_time_sync_in = *(uint32_t *)payload; payload += 4;
	imu_sensors.yaw = *(float *)payload; payload += 4;
	imu_sensors.pitch = *(float *)payload; payload += 4;
	imu_sensors.roll = *(float *)payload; payload += 4;
	
	// update the propery tree and timestamps
	goldy2_imu_update_internal();
    } else if ( buf[3] == 0x82 ) {
	// printf("GPS Packet len = %d\n", len);
        uint8_t *payload = buf + 6;
	scan_ublox(payload, len);
    } else if ( buf[3] == 0x85 && len == 32 ) {
        uint8_t *payload = buf + 6;
        for ( int i = 0; i < rcin_channels; i++ ) {
	    rcin[i] = *(uint16_t *)payload; payload += 2;
	    // printf(" %d", rcin[i]);
	}
        // printf("\n");

    } else {
	// printf("goldy unknown packet or wrong length.\n");
	return 0;
    }

    return buf[3];
}


// read and parse the next incoming packet
static int goldy2_read() {
    const int goldy2_max_size = 2048;
    uint8_t packet_buf[goldy2_max_size];
    
    int result = sock.recv(packet_buf, goldy2_max_size, 0);
    if ( result > 0 ) {
	int pkt_id = goldy2_parse(packet_buf, result);
	return pkt_id;
    }

    return 0;
}

// Read goldy2 packets using IMU packet as the main timing reference.
// Returns the dt from the IMU perspective, not the localhost
// perspective.  This should generally be far more accurate and
// consistent.
double goldy2_update() {
    // printf("checking for packet ...\n");
    double last_time = imu_node.getDouble( "timestamp" );
    while ( true ) {
	int pkt_id = goldy2_read();
	if ( pkt_id == 0x81 /* IMU */ ) {
	    int bytes_available = 0;
            ioctl(sock.getHandle(), FIONREAD, &bytes_available);
            if ( bytes_available < 512 /* IMU packet len */ ) {
                // bigger values == break out and process data even though we
                // are slightly backed up (play more catchup).
                // Smaller values == throw more frames away, even if we get
                // a bit behind.
                // printf("%d ", bytes_available);
		break;
            }
	    // printf("looping: %d bytes available in imu sock buffer\n", bytes_available);
	}
    }
    double cur_time = imu_node.getDouble( "timestamp" );

    return cur_time - last_time;
}


// this keeps the imu_mgr happy, but the real work to update the
// property tree is performed right away when we receive and parse the
// packet.
bool goldy2_imu_update() {
    return true;
}

bool goldy2_airdata_update() {
    bool fresh_data = false;

    static double last_time = 0.0;
    double cur_time = 0.0;
    if ( airdata_node.hasChild("timestamp") ) {
        cur_time = airdata_node.getDouble("timestamp");
    }

    if ( cur_time > last_time ) {
	fresh_data = true;
    }

    last_time = cur_time;

    return fresh_data;
}


bool goldy2_gps_update() {
    static double last_timestamp = 0.0;
    double current_timestamp = 0.0;
    if ( gps_node.hasChild("timestamp") ) {
	current_timestamp = gps_node.getDouble("timestamp");
    }
    if ( current_timestamp > last_timestamp ) {
        last_timestamp = current_timestamp;
        return true;
    } else {
        return false;
    }
}

// convert a sbus pulse length to a normalize [-1 to 1] or [0 to 1] range
static float normalize_pulse( int pulse, bool symmetrical ) {
    float result = 0.0;

    if ( symmetrical ) {
	// i.e. aileron, rudder, elevator
	result = (pulse - SBUS_CENTER) / (float)SBUS_HALF_RANGE;
	if ( result < -1.0 ) { result = -1.0; }
	if ( result > 1.0 ) { result = 1.0; }
    } else {
	// i.e. throttle
	result = (pulse - SBUS_MIN) / (float)SBUS_RANGE;
	if ( result < 0.0 ) { result = 0.0; }
	if ( result > 1.0 ) { result = 1.0; }
    }

    return result;
}

bool goldy2_pilot_update() {
    // quick sanity check Goldy2 can spit out garbage on random
    // individual channels periodically.  Only look at first 7
    // channels.
    bool ok = true;
    for ( int i = 0; i < 7; i++ ) {
        if ( rcin[i] < SBUS_MIN ) { ok = false; }
        if ( rcin[i] > SBUS_MAX ) { ok = false; }
        // printf("%d ", rcin[i]);
    }
    // printf("\n");
    if ( ! ok ) {
	events->log("Goldy2", "detected bad sbus packet (ignoring)");
        return false;
    }
    
    pilot_node.setDouble( "timestamp", get_Time() );
    float default_val = SBUS_CENTER;
    for ( int i = 0; i < rcin_channels; i++ ) {
	if ( i == 0 ) {
	    // special case autopilot master switch on ch1
	    default_val = SBUS_MAX;
	} else if ( pilot_symmetric[i] ) {
	    default_val = SBUS_CENTER;
	} else {
	    default_val = SBUS_MIN;
	}
        if ( rcin[i] < SBUS_MIN ) { rcin[i] = default_val; }
        if ( rcin[i] > SBUS_MAX ) { rcin[i] = default_val; }
	float val = normalize_pulse( rcin[i], pilot_symmetric[i] );
	if ( pilot_invert[i] ) {
	    if ( pilot_symmetric[i] ) {
		val *= -1.0;
	    } else {
		val = 1.0 - val;
	    }
	}
	pilot_node.setDouble( pilot_mapping[i].c_str(), val );
	pilot_node.setDouble( "channel", i, val );
    }
    if ( pilot_wing_mixing ) {
	double l = pilot_node.getDouble("left_surface");
	double r = pilot_node.getDouble("right_surface");
	double ail = (-l + r) * 0.5;
	double ele = (l + r) * 0.5;
	pilot_node.setDouble("aileron", ail);
	pilot_node.setDouble("elevator", ele);
    }

    return true;
}

void goldy2_imu_close() {
    sock.close();
}

void goldy2_airdata_close() {
}

void goldy2_gps_close() {
}

void goldy2_pilot_close() {
}
