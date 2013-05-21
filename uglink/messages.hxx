#ifndef _UGEAR_MESSAGES_HXX
#define _UGEAR_MESSAGES_HXX


//#include <simgear/compiler.h>

#include <iostream>
#include <string>
#include <vector>
#include <zlib.h>

// #include "iochannel.hxx"

#include "globals.hxx"
#include "serial.hxx"

using std::cout;
using std::endl;
using std::string;
using std::vector;


enum ugPacketType {
    GPS_PACKET_V1 = 0,
    IMU_PACKET_V1 = 1,
    FILTER_PACKET_V1 = 2,
    ACTUATOR_PACKET_V1 = 3,
    PILOT_INPUT_PACKET_V1 = 4,
    AP_STATUS_PACKET_V1 = 5,
    AIR_DATA_PACKET_V1 = 6,
    SYSTEM_HEALTH_PACKET_V1 = 7,
    AIR_DATA_PACKET_V2 = 8,
    AIR_DATA_PACKET_V3 = 9,
    AP_STATUS_PACKET_V2 = 10,
    SYSTEM_HEALTH_PACKET_V2 = 11,
    PAYLOAD_PACKET_V1 = 12,
};

// these constants are depricated and remain to support older code
// that knows how to load and decode older data streams.  At some
// point when the sentimental value of old old data is lost, this code
// could get stripped out.
const uint8_t GPS_PACKET_V1_SIZE = 44;
const uint8_t IMU_PACKET_V1_SIZE = 45;
const uint8_t FILTER_PACKET_V1_SIZE = 42;
const uint8_t ACTUATOR_PACKET_V1_SIZE = 25;
const uint8_t PILOT_INPUT_PACKET_V1_SIZE = 25;
const uint8_t AP_STATUS_PACKET_V1_SIZE = 43;
const uint8_t AIR_DATA_PACKET_V1_SIZE = 19;
const uint8_t AIR_DATA_PACKET_V2_SIZE = 23;
const uint8_t SYSTEM_HEALTH_PACKET_V1_SIZE = 12;

// Manage a saved ugear log (track file)
class UGTrack {

private:

    vector <gps> gps_data;
    vector <imu> imu_data;
    vector <airdata> air_data;
    vector <filter> filter_data;
    vector <actuator> act_data;
    vector <pilot> pilot_data;
    vector <apstatus> ap_data;
    vector <health> health_data;
    vector <payload> payload_data;

    // parse message and put current data into vector if message has a
    // newer time stamp than existing data.
    void parse_msg( const int id, char *buf,
		    struct gps *gpspacket,
		    struct imu *imupacket,
		    struct airdata *airpacket,
		    struct filter *filterpacket,
		    struct actuator *actpacket,
		    struct pilot *pilotpacket,
		    struct apstatus *appacket,
		    struct health *healthpacket,
		    struct payload *payloadpacket );

    // activate special double swap logic for non-standard stargate
    // double format
    bool sg_swap;

public:

    UGTrack();
    ~UGTrack();

    // read/parse the next message from the specified data stream,
    // returns id # if a valid message found.
    int next_message( gzFile fd, SGIOChannel *log,
                      struct gps *gpspacket,
		      struct imu *imupacket,
		      struct airdata *airpacket,
		      struct filter *filterpacket,
		      struct actuator *actpacket,
		      struct pilot *pilotpacket,
		      struct apstatus *appacket,
		      struct health *healthpacket,
		      struct payload *payloadpacket,
		      bool ignore_checksum );
    int next_message( SGSerialPort *serial, SGIOChannel *log,
                      struct gps *gpspacket,
		      struct imu *imupacket,
		      struct airdata *airpacket,
		      struct filter *filterpacket,
		      struct actuator *actpacket,
		      struct pilot *pilotpacket,
		      struct apstatus *appacket,
		      struct health *healthpacket,
		      struct payload *payloadpacket,
		      bool ignore_checksum );

    // load the named stream log file into internal buffers
    bool load_stream( const string &file, bool ignore_checksum );

    // load the named flight files into internal buffers
    bool load_flight( const string &path );

    // export the raw imu/gps data for offline libumngnss processing
    bool export_raw_umn( const string &path );

    // export the whole flight data set as tab delimited text data
    bool export_text_tab( const string &path );

    inline int gps_size() const { return gps_data.size(); }
    inline int imu_size() const { return imu_data.size(); }
    inline int airdata_size() const { return air_data.size(); }
    inline int filter_size() const { return filter_data.size(); }
    inline int act_size() const { return act_data.size(); }
    inline int pilot_size() const { return pilot_data.size(); }
    inline int ap_size() const { return ap_data.size(); }
    inline int health_size() const { return health_data.size(); }
    inline int payload_size() const { return payload_data.size(); }

    inline gps get_gpspt( const unsigned int i )
    {
        if ( i < gps_data.size() ) {
            return gps_data[i];
        } else {
            return gps();
        }
    }
    inline imu get_imupt( const unsigned int i )
    {
        if ( i < imu_data.size() ) {
            return imu_data[i];
        } else {
            return imu();
        }
    }
    inline airdata get_airdatapt( const unsigned int i )
    {
        if ( i < air_data.size() ) {
            return air_data[i];
        } else {
            return airdata();
        }
    }
    inline filter get_filterpt( const unsigned int i )
    {
        if ( i < filter_data.size() ) {
            return filter_data[i];
        } else {
            return filter();
        }
    }
    inline actuator get_actpt( const unsigned int i )
    {
        if ( i < act_data.size() ) {
            return act_data[i];
        } else {
            return actuator();
        }
    }
    inline pilot get_pilotpt( const unsigned int i )
    {
        if ( i < pilot_data.size() ) {
            return pilot_data[i];
        } else {
            return pilot();
        }
    }
    inline apstatus get_appt( const unsigned int i )
    {
        if ( i < ap_data.size() ) {
            return ap_data[i];
        } else {
            return apstatus();
        }
    }
    inline health get_healthpt( const unsigned int i )
    {
        if ( i < health_data.size() ) {
            return health_data[i];
        } else {
            return health();
        }
    }
    inline payload get_payloadpt( const unsigned int i )
    {
        if ( i < payload_data.size() ) {
            return payload_data[i];
        } else {
            return payload();
        }
    }
      

    // set stargate mode where we have to do an odd swapping of doubles to
    // account for their non-standard formate
    inline void set_stargate_swap_mode() {
        sg_swap = true;
    }
};


gps UGEARInterpGPS( const gps A, const gps B, const double percent );
imu UGEARInterpIMU( const imu A, const imu B, const double percent );
airdata UGEARInterpAIR( const airdata A, const airdata B, const double percent );
filter UGEARInterpFILTER( const filter A, const filter B,
			  const double percent );
actuator UGEARInterpACT( const actuator A, const actuator B,
			 const double percent );
pilot UGEARInterpPILOT( const pilot A, const pilot B, const double percent );
apstatus UGEARInterpAP( const apstatus A, const apstatus B,
			  const double percent );
health UGEARInterpHEALTH( const health A, const health B,
			  const double percent );


#endif // _UGEAR_MESSAGES_HXX
