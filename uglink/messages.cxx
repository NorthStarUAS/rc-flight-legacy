#include <stdio.h>
#include <zlib.h>
#include <stdlib.h>

#include "sg_file.hxx"
#include "include/globaldefs.h"
#include "util/sg_path.hxx"
#include <util/timing.h>

#include "command.hxx"
#include "messages.hxx"

using std::cout;
using std::endl;


#define START_OF_MSG0 147
#define START_OF_MSG1 224


UGTrack::UGTrack():
    sg_swap(false)
{
};

UGTrack::~UGTrack() {};


static bool validate_cksum( uint8_t id, uint8_t size, char *buf,
                            uint8_t cksum0, uint8_t cksum1,
			    bool ignore_checksum )
{
    if ( ignore_checksum ) {
        return true;
    }

    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += id;
    c1 += c0;
    // cout << "c0 = " << (unsigned int)c0 << " c1 = " << (unsigned int)c1 << endl;

    c0 += size;
    c1 += c0;
    // cout << "c0 = " << (unsigned int)c0 << " c1 = " << (unsigned int)c1 << endl;

    for ( uint8_t i = 0; i < size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
        // cout << "c0 = " << (unsigned int)c0 << " c1 = " << (unsigned int)c1
        //      << " [" << (unsigned int)(uint8_t)buf[i] << "]" << endl;
    }

    // cout << "c0 = " << (unsigned int)c0 << " (" << (unsigned int)cksum0
    //      << ") c1 = " << (unsigned int)c1 << " (" << (unsigned int)cksum1
    //      << ")" << endl;

    if ( c0 == cksum0 && c1 == cksum1 ) {
        return true;
    } else {
        return false;
    }
}


void UGTrack::parse_msg( const int id, char *buf,
			 struct gps *gpspacket, struct imu *imupacket,
			 struct airdata *airpacket,
			 struct filter *filterpacket,
			 struct actuator *actpacket,
			 struct pilot *pilotpacket,
			 struct apstatus *appacket,
			 struct health *healthpacket,
			 struct payload *payloadpacket)
{
    if ( id == GPS_PACKET_V1 ) {
	gpspacket->timestamp = *(double *)buf; buf += 8;
	gpspacket->lat = *(double *)buf; buf += 8;
	gpspacket->lon = *(double *)buf; buf += 8;
	gpspacket->alt = *(float *)buf; buf += 4;
	gpspacket->vn = (*(int16_t *)buf) / 100.0; buf += 2;
	gpspacket->ve = (*(int16_t *)buf) / 100.0; buf += 2;
	gpspacket->vd = (*(int16_t *)buf) / 100.0; buf += 2;
	gpspacket->gps_time = *(double *)buf; buf += 8;
	gpspacket->satellites = *(uint8_t *)buf; buf += 1;
	gpspacket->status = *(uint8_t *)buf; buf += 1;

	/*printf("gps %.2f %.8f %.8f %.2f %.2f %.2f %.2f %.2f %d %d\n",
	       gpspacket->timestamp, gpspacket->lat, gpspacket->lon,
	       gpspacket->alt, gpspacket->vn, gpspacket->ve,
	       gpspacket->vd, gpspacket->gps_time, gpspacket->satellites,
	       gpspacket->status);*/
    } else if ( id == IMU_PACKET_V1 ) {
	imupacket->timestamp = *(double *)buf; buf += 8;
	imupacket->p = *(float *)buf; buf += 4;
	imupacket->q = *(float *)buf; buf += 4;
	imupacket->r = *(float *)buf; buf += 4;
	imupacket->ax = *(float *)buf; buf += 4;
	imupacket->ay = *(float *)buf; buf += 4;
	imupacket->az = *(float *)buf; buf += 4;
	imupacket->hx = *(float *)buf; buf += 4;
	imupacket->hy = *(float *)buf; buf += 4;
	imupacket->hz = *(float *)buf; buf += 4;
	imupacket->status = *(uint8_t *)buf; buf += 1;
	
	/*printf("imu = %.2f (%.3f %.3f %.3f) (%.3f %.3f %.f) (%.3f %.3f %.3f) %d\n",
	       imupacket->timestamp, imupacket->p, imupacket->q, imupacket->r,
	       imupacket->ax, imupacket->ay, imupacket->az, imupacket->hx,
	       imupacket->hy, imupacket->hz, imupacket->status );*/
	/*printf("imu %.3f %.4f %.4f %.4f %.4f %.4f %.4f\n",
	       imupacket->timestamp, imupacket->p, imupacket->q, imupacket->r,
	       imupacket->ax, imupacket->ay, imupacket->az);*/
    } else if ( id == IMU_PACKET_V2 ) {
	imupacket->timestamp = *(double *)buf; buf += 8;
	imupacket->p = *(float *)buf; buf += 4;
	imupacket->q = *(float *)buf; buf += 4;
	imupacket->r = *(float *)buf; buf += 4;
	imupacket->ax = *(float *)buf; buf += 4;
	imupacket->ay = *(float *)buf; buf += 4;
	imupacket->az = *(float *)buf; buf += 4;
	imupacket->hx = *(float *)buf; buf += 4;
	imupacket->hy = *(float *)buf; buf += 4;
	imupacket->hz = *(float *)buf; buf += 4;
	imupacket->temp = *(int16_t *)buf / 10.0; buf += 2;
	imupacket->status = *(uint8_t *)buf; buf += 1;
	
	/*printf("imu = %.2f (%.3f %.3f %.3f) (%.3f %.3f %.f) (%.3f %.3f %.3f) %d\n",
	       imupacket->timestamp, imupacket->p, imupacket->q, imupacket->r,
	       imupacket->ax, imupacket->ay, imupacket->az, imupacket->hx,
	       imupacket->hy, imupacket->hz, imupacket->status );*/
	/*printf("imu %.3f %.4f %.4f %.4f %.4f %.4f %.4f\n",
	       imupacket->timestamp, imupacket->p, imupacket->q, imupacket->r,
	       imupacket->ax, imupacket->ay, imupacket->az);*/
    } else if ( id == AIR_DATA_PACKET_V1 ) {
	airpacket->timestamp = *(double *)buf; buf += 8;
	airpacket->airspeed = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->altitude = *(float *)buf; buf += 4;
	airpacket->climb_fpm = *(int16_t *)buf / 10.0; buf += 2;
	airpacket->acceleration = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->status = *(uint8_t *)buf; buf += 1;
	
	/* printf("air1 %.2f %.1f %.1f %.2f %.2f %.1f %.1f %.2f %d\n",
	       airpacket->timestamp, airpacket->airspeed, airpacket->altitude,
	       airpacket->climb_fpm, airpacket->acceleration,
	       airpacket->status ); */
    } else if ( id == AIR_DATA_PACKET_V2 ) {
	airpacket->timestamp = *(double *)buf; buf += 8;
	airpacket->airspeed = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->altitude = *(float *)buf; buf += 4;
	airpacket->climb_fpm = *(int16_t *)buf / 10.0; buf += 2;
	airpacket->acceleration = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->wind_dir = *(uint16_t *)buf / 100.0; buf += 2;
	airpacket->wind_speed = *(uint8_t *)buf / 4.0; buf += 1;
	airpacket->pitot_scale = *(uint8_t *)buf / 100.0; buf += 1;
	airpacket->status = *(uint8_t *)buf; buf += 1;
	
	/* printf("air2 %.2f %.1f %.1f %.2f %.2f %.1f %.1f %.2f %d\n",
	       airpacket->timestamp, airpacket->airspeed, airpacket->altitude,
	       airpacket->climb_fpm, airpacket->acceleration,
	       airpacket->wind_dir, airpacket->wind_speed,
	       airpacket->pitot_scale, airpacket->status ); */
    } else if ( id == AIR_DATA_PACKET_V3 ) {
	airpacket->timestamp = *(double *)buf; buf += 8;
	airpacket->pressure = *(uint16_t *)buf / 10.0; buf += 2;
	airpacket->temperature = *(int16_t *)buf / 10.0; buf += 2;
	airpacket->airspeed = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->altitude = *(float *)buf; buf += 4;
	airpacket->climb_fpm = *(int16_t *)buf / 10.0; buf += 2;
	airpacket->acceleration = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->wind_dir = *(uint16_t *)buf / 100.0; buf += 2;
	airpacket->wind_speed = *(uint8_t *)buf / 4.0; buf += 1;
	airpacket->pitot_scale = *(uint8_t *)buf / 100.0; buf += 1;
	airpacket->status = *(uint8_t *)buf; buf += 1;
	
	/*printf("air3 %.2f %.1f %.1f %.1f %.1f %.2f %.2f %.1f %.1f %.2f %d\n",
	       airpacket->timestamp, airpacket->pressure,
	       airpacket->temperature, airpacket->airspeed, airpacket->altitude,
	       airpacket->climb_fpm, airpacket->acceleration,
	       airpacket->wind_dir, airpacket->wind_speed,
	       airpacket->pitot_scale, airpacket->status );*/
    } else if ( id == AIR_DATA_PACKET_V4 ) {
	airpacket->timestamp = *(double *)buf; buf += 8;
	airpacket->pressure = *(uint16_t *)buf / 10.0; buf += 2;
	airpacket->temperature = *(int16_t *)buf / 10.0; buf += 2;
	airpacket->airspeed = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->altitude = *(float *)buf; buf += 4;
	airpacket->altitude_true = *(float *)buf; buf += 4;
	airpacket->climb_fpm = *(int16_t *)buf / 10.0; buf += 2;
	airpacket->acceleration = *(int16_t *)buf / 100.0; buf += 2;
	airpacket->wind_dir = *(uint16_t *)buf / 100.0; buf += 2;
	airpacket->wind_speed = *(uint8_t *)buf / 4.0; buf += 1;
	airpacket->pitot_scale = *(uint8_t *)buf / 100.0; buf += 1;
	airpacket->status = *(uint8_t *)buf; buf += 1;
	
	/* printf("air4 %.2f %.1f %.1f %.1f %.1f %.1f %.2f %.2f %.1f %.1f %.2f %d\n",
	       airpacket->timestamp, airpacket->pressure,
	       airpacket->temperature, airpacket->airspeed,
	       airpacket->altitude, airpacket->altitude_true,
	       airpacket->climb_fpm, airpacket->acceleration,
	       airpacket->wind_dir, airpacket->wind_speed,
	       airpacket->pitot_scale, airpacket->status ); */
    } else if ( id == FILTER_PACKET_V1 ) {
	filterpacket->timestamp = *(double *)buf; buf += 8;
	filterpacket->lat = *(double *)buf; buf += 8;
	filterpacket->lon = *(double *)buf; buf += 8;
	filterpacket->alt = *(float *)buf; buf += 4;
	filterpacket->vn = (*(int16_t *)buf) / 100.0; buf += 2;
	filterpacket->ve = (*(int16_t *)buf) / 100.0; buf += 2;
	filterpacket->vd = (*(int16_t *)buf) / 100.0; buf += 2;
	filterpacket->phi = (*(int16_t *)buf) / 10.0; buf += 2;
	filterpacket->theta = (*(int16_t *)buf) / 10.0; buf += 2;
	filterpacket->psi = (*(int16_t *)buf) / 10.0; buf += 2;
	if ( filterpacket->psi < 0 ) { filterpacket->psi += 360.0; }
	if ( filterpacket->psi > 360 ) { filterpacket->psi -= 360.0; }
	filterpacket->command_seq = *(uint8_t *)buf; buf += 1;
	filterpacket->status = *(uint8_t *)buf; buf += 1;

	/*command_mgr.update_cmd_sequence( filterpacket->command_seq,
					 filterpacket->timestamp );*/

	/*printf("filter %.2f %.8f %.8f %.2f %.2f %.2f %.2f %.1f %.1f %.1f %d %d\n",
	       filterpacket->timestamp, filterpacket->lat, filterpacket->lon,
	       filterpacket->alt, filterpacket->vn, filterpacket->ve,
	       filterpacket->vd, filterpacket->phi, filterpacket->theta,
	       filterpacket->psi, filterpacket->status, command_seq);*/
    } else if ( id == ACTUATOR_PACKET_V1 ) {
	actpacket->timestamp = *(double *)buf; buf += 8;
	actpacket->ail = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->ele = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->thr = (*(uint16_t *)buf) / 60000.0; buf += 2;
	actpacket->rud = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->ch5 = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->ch6 = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->ch7 = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->ch8 = (*(int16_t *)buf) / 30000.0; buf += 2;
	actpacket->status = *(uint8_t *)buf; buf += 1;

	/*printf("act %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d\n",
	       actpacket->timestamp, actpacket->ail, actpacket->ele,
	       actpacket->thr, actpacket->rud, actpacket->ch5, actpacket->ch6,
	       actpacket->ch7, actpacket->ch8, actpacket->status);*/
    } else if ( id == PILOT_INPUT_PACKET_V1 ) {
	pilotpacket->timestamp = *(double *)buf; buf += 8;
	pilotpacket->ail = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->ele = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->thr = (*(uint16_t *)buf) / 60000.0; buf += 2;
	pilotpacket->rud = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->ch5 = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->ch6 = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->ch7 = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->ch8 = (*(int16_t *)buf) / 30000.0; buf += 2;
	pilotpacket->status = *(uint8_t *)buf; buf += 1;

 	/*printf("pilot %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d\n",
	       pilotpacket->timestamp, pilotpacket->ail, pilotpacket->ele,
	       pilotpacket->thr, pilotpacket->rud, pilotpacket->ch5,
	       pilotpacket->ch6, pilotpacket->ch7, pilotpacket->ch8,
	       pilotpacket->status);*/
    } else if ( id == AP_STATUS_PACKET_V1 ) {
	appacket->timestamp = *(double *)buf; buf += 8;
	appacket->target_heading_deg = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_roll_deg = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_altitude_msl_ft = *(uint16_t *)buf; buf += 2;
	appacket->target_climb_fps = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_pitch_deg = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_speed_kt = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_wp = *(uint16_t *)buf; buf += 2;
	appacket->wp_lon = *(double *)buf; buf += 8;
	appacket->wp_lat = *(double *)buf; buf += 8;
	appacket->wp_index = *(uint16_t *)buf; buf += 2;
	appacket->route_size = *(uint16_t *)buf; buf += 2;
	uint8_t command_seq = *(uint8_t *)buf; buf += 1;

	/*command_mgr.update_cmd_sequence( command_seq, appacket->timestamp );*/

	/* printf("ap = %.2f (%.1f %.1f) (%04.0f %.1f %.1f) %.1f [%d %.10f %.10f %d %d]\n",
	       appacket->timestamp, appacket->target_heading_deg,
	       appacket->target_roll_deg, appacket->target_altitude_msl_ft,
	       appacket->target_climb_fps*60.0, appacket->target_pitch_deg,
	       appacket->target_speed_kt, appacket->target_wp,
	       appacket->wp_lon, appacket->wp_lat, appacket->wp_index,
	       appacket->route_size); */
    } else if ( id == AP_STATUS_PACKET_V2 ) {
	appacket->timestamp = *(double *)buf; buf += 8;
	appacket->target_heading_deg = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_roll_deg = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_altitude_msl_ft = *(uint16_t *)buf; buf += 2;
	appacket->target_climb_fps = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_pitch_deg = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_theta_dot = (*(int16_t *)buf) / 1000.0; buf += 2;
	appacket->target_speed_kt = (*(int16_t *)buf) / 10.0; buf += 2;
	appacket->target_wp = *(uint16_t *)buf; buf += 2;
	appacket->wp_lon = *(double *)buf; buf += 8;
	appacket->wp_lat = *(double *)buf; buf += 8;
	appacket->wp_index = *(uint16_t *)buf; buf += 2;
	appacket->route_size = *(uint16_t *)buf; buf += 2;
	uint8_t command_seq = *(uint8_t *)buf; buf += 1;

	/*command_mgr.update_cmd_sequence( command_seq, appacket->timestamp );*/

	/* printf("ap = %.2f (%.1f %.1f) (%04.0f %.1f %.1f %.2f) %.1f [%d %.10f %.10f %d %d]\n",
	       appacket->timestamp, appacket->target_heading_deg,
	       appacket->target_roll_deg, appacket->target_altitude_msl_ft,
	       appacket->target_climb_fps*60.0, appacket->target_pitch_deg,
	       appacket->target_theta_dot, appacket->target_speed_kt,
	       appacket->target_wp,
	       appacket->wp_lon, appacket->wp_lat, appacket->wp_index,
	       appacket->route_size); */
    } else if ( id == SYSTEM_HEALTH_PACKET_V1 ) {
        healthpacket->timestamp = *(double *)buf; buf += 8;
	healthpacket->avionics_vcc = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->load_avg = (*(uint16_t *)buf) / 100.0; buf += 2;
	/* printf("health = %.2f vcc=%.2f loadavg=%.2f\n",
	       healthpacket->timestamp, healthpacket->input_vcc,
	       healthpacket->loadavg); */
    } else if ( id == SYSTEM_HEALTH_PACKET_V2 ) {
        healthpacket->timestamp = *(double *)buf; buf += 8;
	healthpacket->load_avg = (*(uint16_t *)buf) / 100.0; buf += 2;
	healthpacket->avionics_vcc = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_volts = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_amps = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_mah = (*(uint16_t *)buf); buf += 2;
	/* printf("health = %.2f vcc=%.2f loadavg=%.2f\n",
	       healthpacket->timestamp, healthpacket->input_vcc,
	       healthpacket->loadavg); */
    } else if ( id == SYSTEM_HEALTH_PACKET_V3 ) {
        healthpacket->timestamp = *(double *)buf; buf += 8;
	healthpacket->load_avg = (*(uint16_t *)buf) / 100.0; buf += 2;
	healthpacket->avionics_vcc = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_volts = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_cell_volts = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_amps = (*(uint16_t *)buf) / 1000.0; buf += 2;
	healthpacket->extern_mah = (*(uint16_t *)buf); buf += 2;
	/* printf("health = %.2f vcc=%.2f loadavg=%.2f\n",
	       healthpacket->timestamp, healthpacket->input_vcc,
	       healthpacket->loadavg); */
    } else if ( id == PAYLOAD_PACKET_V1 ) {
        payloadpacket->timestamp = *(double *)buf; buf += 8;
	payloadpacket->trigger_num = *(uint16_t *)buf; buf += 2;
	/* printf("payload = %.2f trigger_num=%d\n",
	   payloadpacket->timestamp, payloadpacket->trigger_num); */
    } else {
        cout << "unknown id = " << id << endl;
    }
}


// load the named stream log file into internal buffers
bool UGTrack::load_stream( const string &file, bool ignore_checksum ) {
    int count = 0;

    gps gpspacket;
    imu imupacket;
    airdata airpacket;
    filter filterpacket;
    actuator actpacket;
    pilot pilotpacket;
    apstatus appacket;
    health healthpacket;
    payload payloadpacket;

    double gps_time = 0.0;
    double imu_time = 0.0;
    double air_time = 0.0;
    double filter_time = 0.0;
    double act_time = 0.0;
    double pilot_time = 0.0;
    double ap_time = 0.0;
    double health_time = 0.0;
    double payload_time = 0.0;

    gps_data.clear();
    imu_data.clear();
    air_data.clear();
    filter_data.clear();
    act_data.clear();
    pilot_data.clear();
    ap_data.clear();
    health_data.clear();
    payload_data.clear();

    static double alt_max = 0.0;
    static double alt_min = 999999999.0;

    // open the file
    gzFile input;
    if ( (input = gzopen( file.c_str(), "rb" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    while ( ! gzeof(input) ) {
        // cout << "looking for next message ..." << endl;
        int id = next_message( input, NULL, &gpspacket, &imupacket,
			       &airpacket, &filterpacket, &actpacket,
			       &pilotpacket, &appacket, &healthpacket,
			       &payloadpacket, ignore_checksum );
        count++;

        if ( id == GPS_PACKET_V1 ) {
            if ( gpspacket.gps_time > gps_time &&
		 gpspacket.gps_time < 4600000000)
	    {
		// double interval = gpspacket.gps_time - gps_time;
		// if ( gps_time < 0.00001 || interval < 1000000 ) {
		    /* printf("gps interval %.3f (last=%.3f new=%.3f)\n",
		       interval, gps_time, gpspacket.gps_time); */
		    gps_data.push_back( gpspacket );
		    if ( gpspacket.alt < alt_min ) {
			alt_min = gpspacket.alt;
		    }
		    if ( gpspacket.alt > alt_max ) {
			alt_max = gpspacket.alt;
		    }
		    // } else {
		    //   cout << "oops gps time too far in future: "
		    //     << gpspacket.gps_time << " > " << gps_time << endl;
		    // }
            } else {
		cout << "oops gps back in time: " << gpspacket.gps_time << " " << gps_time << endl;
            }
	    gps_time = gpspacket.gps_time;
        } else if ( id == IMU_PACKET_V1 || id == IMU_PACKET_V2 ) {
            if ( imupacket.timestamp > imu_time ) {
                imu_data.push_back( imupacket );
            } else {
                cout << "oops imu back in time" << endl;
            }
	    imu_time = imupacket.timestamp;
        } else if ( id == AIR_DATA_PACKET_V1 || id == AIR_DATA_PACKET_V2 || id == AIR_DATA_PACKET_V3 || id == AIR_DATA_PACKET_V4 ) {
            if ( airpacket.timestamp > air_time ) {
                air_data.push_back( airpacket );
            } else {
                cout << "oops airdata back in time" << endl;
            }
	    air_time = airpacket.timestamp;
        } else if ( id == FILTER_PACKET_V1 ) {
            if ( filterpacket.timestamp > filter_time ) {
                filter_data.push_back( filterpacket );
            } else {
                cout << "oops filter back in time" << endl;
		// cout << "  " << filterpacket.timestamp << " < " << filter_time << endl;
            }
	    filter_time = filterpacket.timestamp;
        } else if ( id == ACTUATOR_PACKET_V1 ) {
            if ( actpacket.timestamp > act_time ) {
                act_data.push_back( actpacket );
            } else {
                cout << "oops actuator back in time" << endl;
            }
	    act_time = actpacket.timestamp;
        } else if ( id == PILOT_INPUT_PACKET_V1 ) {
            if ( pilotpacket.timestamp > pilot_time ) {
                pilot_data.push_back( pilotpacket );
            } else {
                cout << "oops pilot back in time" << endl;
            }
	    pilot_time = pilotpacket.timestamp;
        } else if ( id == AP_STATUS_PACKET_V1 || id == AP_STATUS_PACKET_V2 ) {
            if ( appacket.timestamp > ap_time ) {
                ap_data.push_back( appacket );
            } else {
                cout << "oops ap status back in time" << endl;
            }
	    ap_time = appacket.timestamp;
        } else if ( id == SYSTEM_HEALTH_PACKET_V1 || id == SYSTEM_HEALTH_PACKET_V2 || id == SYSTEM_HEALTH_PACKET_V3 ) {
            if ( healthpacket.timestamp > health_time ) {
                health_data.push_back( healthpacket );
            } else {
                cout << "oops health status back in time" << endl;
            }
	    health_time = healthpacket.timestamp;
        } else if ( id == PAYLOAD_PACKET_V1 ) {
            if ( payloadpacket.timestamp > payload_time ) {
                payload_data.push_back( payloadpacket );
            } else {
                cout << "oops payload status back in time" << endl;
            }
	    payload_time = payloadpacket.timestamp;
        } else {
	    cout << "Unknown packet id: " << id << endl;
	}
    }

    cout << "processed " << count << " messages" << endl;
    if ( alt_max > alt_min + 50 ) {
	printf("Flight altitude range: %.0f - %.0f (meters) MSL\n",
	       alt_min, alt_max);
    } else {
	printf("Ground test around %.0f (meters) MSL\n",
	       (alt_min + alt_max) / 2.0);
    }
    return true;
}


// load the named stream log file into internal buffers
bool UGTrack::load_flight( const string &path ) {
    gps gpspacket;
    imu imupacket;
    airdata airpacket;
    filter filterpacket;
    actuator actpacket;
    pilot pilotpacket;
    apstatus appacket;
    health healthpacket;
    payload payloadpacket;

    gps_data.clear();
    imu_data.clear();
    filter_data.clear();
    act_data.clear();
    ap_data.clear();
    health_data.clear();
    payload_data.clear();

    gzFile fgps = NULL;
    gzFile fimu = NULL;
    gzFile fair = NULL;
    gzFile ffilter = NULL;
    gzFile fact = NULL;
    gzFile fpilot = NULL;
    gzFile fap = NULL;
    gzFile fhealth = NULL;

    SGPath file;
    int size;

    double dt;
    double t_last;
    double maxt;

    char buf[256];

    // open the gps file
    file = path; file.append( "gps.dat.gz" );
    if ( (fgps = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = GPS_PACKET_V1_SIZE;
    printf("gps size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fgps, buf, size ) == size ) {
	parse_msg( GPS_PACKET_V1, buf, &gpspacket, &imupacket, &airpacket,
		   &filterpacket, &actpacket, &pilotpacket, &appacket,
		   &healthpacket, &payloadpacket );
	dt = gpspacket.gps_time - t_last;
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f t2 = %.3f  count = %d\n",
		    maxt, t_last, gpspacket.gps_time, (int)gps_data.size() );
	}
        gps_data.push_back( gpspacket );
	t_last = gpspacket.gps_time;
    }

    // open the imu file
    file = path; file.append( "imu.dat.gz" );
    if ( (fimu = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = IMU_PACKET_V1_SIZE;
    printf("imu size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fimu, buf, size ) == size ) {
	parse_msg( IMU_PACKET_V1, buf, &gpspacket, &imupacket, &airpacket,
		   &filterpacket, &actpacket, &pilotpacket, &appacket,
		   &healthpacket, &payloadpacket );
        /* printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
               imupacket.Ps, imupacket.Pt,
               imupacket.p, imupacket.q, imupacket.r,
               imupacket.ax, imupacket.ay, imupacket.az); */
	dt = imupacket.timestamp - t_last;
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, imupacket.timestamp, (int)imu_data.size() );
	}
        imu_data.push_back( imupacket );
	t_last = imupacket.timestamp;
    }

    // open the airdata file
    file = path; file.append( "air.dat.gz" );
    if ( (fair = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = AIR_DATA_PACKET_V2_SIZE;
    printf("air data size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fair, buf, size ) == size ) {
	parse_msg( AIR_DATA_PACKET_V2, buf, &gpspacket, &imupacket, &airpacket,
		   &filterpacket, &actpacket, &pilotpacket, &appacket,
		   &healthpacket, &payloadpacket );
        printf("airdata %.3f %.1f %.1f %.1f %.3f %.1f %.1f %.2f\n",
               airpacket.timestamp, airpacket.airspeed, airpacket.altitude,
               airpacket.climb_fpm, airpacket.acceleration,
	       airpacket.wind_dir, airpacket.wind_speed,
	       airpacket.pitot_scale);
	dt = airpacket.timestamp - t_last;
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, airpacket.timestamp, (int)air_data.size() );
	}
        air_data.push_back( airpacket );
	t_last = airpacket.timestamp;
    }

    // open the filter file
    file = path; file.append( "filter.dat.gz" );
    if ( (ffilter = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = FILTER_PACKET_V1_SIZE;
    printf("filter size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( ffilter, buf, size ) == size ) {
	parse_msg( FILTER_PACKET_V1, buf, &gpspacket, &imupacket, &airpacket,
		   &filterpacket, &actpacket, &pilotpacket, &appacket,
		   &healthpacket, &payloadpacket );
        printf("filter %.3f %.4f %.4f %.2f %.2f %.2f\n", filterpacket.timestamp,
	       filterpacket.lat, filterpacket.lon,
	       filterpacket.phi, filterpacket.theta, filterpacket.psi );
	dt = filterpacket.timestamp - t_last;
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, filterpacket.timestamp, (int)filter_data.size() );
	}
        filter_data.push_back( filterpacket );
	t_last = filterpacket.timestamp;
    }

    // open the actuator file
    file = path; file.append( "actuator.dat.gz" );
    if ( (fact = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = ACTUATOR_PACKET_V1_SIZE;
    printf("actuator size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fact, buf, size ) == size ) {
	parse_msg( ACTUATOR_PACKET_V1, buf, &gpspacket, &imupacket, &airpacket,
		   &filterpacket, &actpacket, &pilotpacket, &appacket,
		   &healthpacket, &payloadpacket );
	printf("act %.3f %.2f %.2f %.2f %.2f\n",
	       actpacket.timestamp, actpacket.ail, actpacket.ele,
	       actpacket.thr, actpacket.rud);
	dt = actpacket.timestamp - t_last;
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, actpacket.timestamp, (int)act_data.size() );
	}
        act_data.push_back( actpacket );
	t_last = actpacket.timestamp;
    }

    // open the pilot file
    file = path; file.append( "pilot.dat.gz" );
    if ( (fpilot = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = PILOT_INPUT_PACKET_V1_SIZE;
    printf("pilot size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fpilot, buf, size ) == size ) {
	parse_msg( PILOT_INPUT_PACKET_V1, buf, &gpspacket, &imupacket,
		   &airpacket, &filterpacket, &actpacket, &pilotpacket,
		   &appacket, &healthpacket, &payloadpacket );
	printf("pilot %.3f %.2f %.2f %.2f %.2f\n",
	       pilotpacket.timestamp, pilotpacket.ail, pilotpacket.ele,
	       pilotpacket.thr, pilotpacket.rud);
	dt = pilotpacket.timestamp - t_last;
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, pilotpacket.timestamp, (int)pilot_data.size() );
	}
        pilot_data.push_back( pilotpacket );
	t_last = pilotpacket.timestamp;
    }

    // open the ap status file
    file = path; file.append( "ap.dat.gz" );
    if ( (fap = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = AP_STATUS_PACKET_V1_SIZE;
    printf("ap status size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fap, buf, size ) == size ) {
	parse_msg( AP_STATUS_PACKET_V1, buf, &gpspacket, &imupacket, &airpacket,
		   &filterpacket, &actpacket, &pilotpacket, &appacket,
		   &healthpacket, &payloadpacket );
	dt = appacket.timestamp - t_last;
	/* printf("%.3f\n", appacket.timestamp); */
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, appacket.timestamp, (int)ap_data.size() );
	}
        ap_data.push_back( appacket );
	t_last = appacket.timestamp;
    }

    // open the health status file
    file = path; file.append( "health.dat.gz" );
    if ( (fhealth = gzopen( file.c_str(), "r" )) == NULL ) {
        printf("Cannot open %s\n", file.c_str());
        return false;
    }

    size = SYSTEM_HEALTH_PACKET_V1_SIZE;
    printf("health status size = %d\n", size);
    t_last = 0.0;
    maxt = 0.0;
    while ( gzread( fhealth, buf, size ) == size ) {
	parse_msg( SYSTEM_HEALTH_PACKET_V1, buf, &gpspacket, &imupacket,
		   &airpacket, &filterpacket, &actpacket, &pilotpacket,
		   &appacket, &healthpacket, &payloadpacket );
	dt = healthpacket.timestamp - t_last;
	/* printf("%.3f\n", healthpacket.timestamp); */
	if ( dt > maxt ) {
	    maxt = dt;
	    printf( "maxt = %.3f  t1 = %.3f  t2 = %.3f  count = %d\n",
		    maxt, t_last, healthpacket.timestamp, (int)health_data.size() );
	}
        health_data.push_back( healthpacket );
	t_last = healthpacket.timestamp;
    }

    return true;
}


// export the raw imu/gps data for offline libumngnss processing
bool UGTrack::export_raw_umn( const string &path ) {
    FILE *gps_fd = NULL;
    string gps_file = path + "/";
    gps_file += "flight_umn.gps";
    gps_fd = fopen( gps_file.c_str(), "w" );
    if ( gps_fd == NULL ) {
	perror("");
	exit(-1);
    }
    gps gpspacket;
    for ( int i = 0; i < gps_size(); i++ ) {
	gpspacket = get_gpspt(i);
	fprintf( gps_fd, "%.4f %.10f %.10f %.4f %.8f %.8f %.8f\n",
		 gpspacket.timestamp,
		 gpspacket.lat * SG_DEGREES_TO_RADIANS,
		 gpspacket.lon * SG_DEGREES_TO_RADIANS,
		 -gpspacket.alt,
		 gpspacket.vn, gpspacket.ve, gpspacket.vd );
    }
    fclose(gps_fd);

    FILE *imu_fd = NULL;
    string imu_file = path + "/";
    imu_file += "flight_umn.imu";
    imu_fd = fopen( imu_file.c_str(), "w" );
    if ( imu_fd == NULL ) {
	perror("");
	exit(-1);
    }
    imu imupacket;
    for ( int i = 0; i < imu_size(); i++ ) {
	imupacket = get_imupt(i);
	fprintf( imu_fd, "%.4f %.5f %.5f %.5f %.5f %.5f %.5f %.3f %.3f %.3f %.1f\n",
		 imupacket.timestamp,
		 imupacket.p, imupacket.q, imupacket.r,
		 imupacket.ax, imupacket.ay, imupacket.az,
		 imupacket.hx, imupacket.hy, imupacket.hz,
		 imupacket.temp
		 );
    }
    fclose(imu_fd);

    FILE *istate_fd = NULL;
    string istate_file = path + "/";
    istate_file += "flight_umn.istate";
    istate_fd = fopen( istate_file.c_str(), "w" );
    if ( istate_fd == NULL ) {
	perror("");
	exit(-1);
    }
    gpspacket = get_gpspt(0);
    fprintf( istate_fd, "%.10f %.10f %.4f %.8f %.8f %.8f %.4f %.4f %.4f\n",
	     gpspacket.lat * SG_DEGREES_TO_RADIANS,
	     gpspacket.lon * SG_DEGREES_TO_RADIANS,
	     -gpspacket.alt,
	     gpspacket.vn, gpspacket.ve, gpspacket.vd,
	     0.0, 0.0, 0.0 );
    fclose(istate_fd);

    return true;
}


// export the whole flight data set as tab delimited files
bool UGTrack::export_text_tab( const string &path ) {
    SGPropertyNode *flying_wing_node = pyGetNode("/config/flying-wing-mode", true);
    FILE *gps_fd = NULL;
    string gps_file = path + "/";
    gps_file += "gps.txt";
    gps_fd = fopen( gps_file.c_str(), "w" );
    if ( gps_fd == NULL ) {
	perror("");
	exit(-1);
    }
    gps gpspacket;
    for ( int i = 0; i < gps_size(); i++ ) {
	gpspacket = get_gpspt(i);
	fprintf( gps_fd,
		 "%.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%f\t%d\t%d\n",
		 gpspacket.timestamp,
		 gpspacket.lat, gpspacket.lon, gpspacket.alt,
		 gpspacket.vn, gpspacket.ve, gpspacket.vd,
		 gpspacket.gps_time, gpspacket.satellites, gpspacket.status );
    }
    fclose(gps_fd);

    FILE *imu_fd = NULL;
    string imu_file = path + "/";
    imu_file += "imu.txt";
    imu_fd = fopen( imu_file.c_str(), "w" );
    if ( imu_fd == NULL ) {
	perror("");
	exit(-1);
    }
    imu imupacket;
    for ( int i = 0; i < imu_size(); i++ ) {
	imupacket = get_imupt(i);
	fprintf( imu_fd, "%.3f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.3f\t%.3f\t%.3f\t%.1f\t%d\n",
		 imupacket.timestamp,
		 imupacket.p, imupacket.q, imupacket.r,
		 imupacket.ax, imupacket.ay, imupacket.az,
		 imupacket.hx, imupacket.hy, imupacket.hz,
		 imupacket.temp, imupacket.status
		 );
    }
    fclose(imu_fd);

    FILE *air_fd = NULL;
    string air_file = path + "/";
    air_file += "air.txt";
    air_fd = fopen( air_file.c_str(), "w" );
    if ( air_fd == NULL ) {
	perror("");
	exit(-1);
    }
    airdata airpacket;
    for ( int i = 0; i < airdata_size(); i++ ) {
	airpacket = get_airdatapt(i);
	fprintf( air_fd, "%.3f\t%.1f\t%.1f\t%.1f\t%.2f\t%.2f\t%.2f\t%.2f\t%.1f\t%.1f\t%.2f\t%d\n",
		 airpacket.timestamp,
		 airpacket.pressure, airpacket.temperature,
		 airpacket.airspeed,
		 airpacket.altitude, airpacket.altitude_true,
		 airpacket.climb_fpm,
		 airpacket.acceleration, airpacket.wind_dir,
		 airpacket.wind_speed, airpacket.pitot_scale, airpacket.status
		 );
    }
    fclose(air_fd);

    FILE *filter_fd = NULL;
    string filter_file = path + "/";
    filter_file += "filter.txt";
    filter_fd = fopen( filter_file.c_str(), "w" );
    if ( filter_fd == NULL ) {
	perror("");
	exit(-1);
    }
    filter filterpacket;
    for ( int i = 0; i < filter_size(); i++ ) {
	filterpacket = get_filterpt(i);
	fprintf( filter_fd,
		 "%.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\t%d\n",

		 filterpacket.timestamp,
		 filterpacket.lat, filterpacket.lon, filterpacket.alt,
		 filterpacket.vn, filterpacket.ve, filterpacket.vd,
		 filterpacket.phi, filterpacket.theta, filterpacket.psi,
		 filterpacket.status
		 );
    }
    fclose(filter_fd);

    FILE *act_fd = NULL;
    string act_file = path + "/";
    act_file += "act.txt";
    act_fd = fopen( act_file.c_str(), "w" );
    if ( act_fd == NULL ) {
	perror("");
	exit(-1);
    }
    actuator actpacket;
    for ( int i = 0; i < act_size(); i++ ) {
	actpacket = get_actpt(i);
	double ail = actpacket.ail;
	double ele = actpacket.ele;
	if ( flying_wing_node->getBool() ) {
	    double ch1 = actpacket.ail;
	    double ch2 = actpacket.ele;
	    double e = (ch1 - ch2) / 2.0;
	    double a = ch1 - e;
	    ele = e;
	    ail = a;
	}
	fprintf( act_fd,
		 "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\n",
		 actpacket.timestamp,
		 ail, ele, actpacket.thr, actpacket.rud,
		 actpacket.ch5, actpacket.ch6, actpacket.ch7, actpacket.ch8,
		 actpacket.status
		 );
    }
    fclose(act_fd);

    FILE *pilot_fd = NULL;
    string pilot_file = path + "/";
    pilot_file += "pilot.txt";
    pilot_fd = fopen( pilot_file.c_str(), "w" );
    if ( pilot_fd == NULL ) {
	perror("");
	exit(-1);
    }
    pilot pilotpacket;
    for ( int i = 0; i < pilot_size(); i++ ) {
	pilotpacket = get_pilotpt(i);
	double ail = pilotpacket.ail;
	double ele = pilotpacket.ele;
	if ( flying_wing_node->getBool() ) {
	    double ch1 = pilotpacket.ail;
	    double ch2 = pilotpacket.ele;
	    double e = (ch1 - ch2) / 2.0;
	    double a = ch1 - e;
	    ele = e;
	    ail = a;
	}
	fprintf( pilot_fd,
		 "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\n",
		 pilotpacket.timestamp,
		 ail, ele,
		 pilotpacket.thr, pilotpacket.rud,
		 pilotpacket.ch5, pilotpacket.ch6,
		 pilotpacket.ch7, pilotpacket.ch8,
		 pilotpacket.status
		 );
    }
    fclose(pilot_fd);

    FILE *ap_fd = NULL;
    string ap_file = path + "/";
    ap_file += "ap.txt";
    ap_fd = fopen( ap_file.c_str(), "w" );
    if ( ap_fd == NULL ) {
	perror("");
	exit(-1);
    }
    apstatus appacket;
    for ( int i = 0; i < ap_size(); i++ ) {
	appacket = get_appt(i);
	fprintf( ap_fd,
		 "%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%.2f\t%d\t%.10f\t%.10f\t%d\t%d\n",
		 appacket.timestamp,
		 appacket.target_heading_deg, appacket.target_roll_deg,
		 appacket.target_altitude_msl_ft, appacket.target_climb_fps,
		 appacket.target_pitch_deg, appacket.target_theta_dot,
		 appacket.target_speed_kt,
		 appacket.target_wp, appacket.wp_lon, appacket.wp_lat,
		 appacket.wp_index, appacket.route_size
		 );
    }
    fclose(ap_fd);

    FILE *health_fd = NULL;
    string health_file = path + "/";
    health_file += "health.txt";
    health_fd = fopen( health_file.c_str(), "w" );
    if ( health_fd == NULL ) {
	perror("");
	exit(-1);
    }
    health healthpacket;
    for ( int i = 0; i < health_size(); i++ ) {
	healthpacket = get_healthpt(i);
	fprintf( health_fd,
		 "%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.0f\n",
		 healthpacket.timestamp,
		 healthpacket.load_avg, healthpacket.avionics_vcc,
		 healthpacket.extern_volts, healthpacket.extern_cell_volts,
		 healthpacket.extern_amps, healthpacket.extern_mah );
    }
    fclose(health_fd);

    FILE *payload_fd = NULL;
    string payload_file = path + "/";
    payload_file += "payload.txt";
    payload_fd = fopen( payload_file.c_str(), "w" );
    if ( payload_fd == NULL ) {
	perror("");
	exit(-1);
    }
    payload payloadpacket;
    for ( int i = 0; i < payload_size(); i++ ) {
	payloadpacket = get_payloadpt(i);
	fprintf( payload_fd,
		 "%.3f\t%d\n",
		 payloadpacket.timestamp,
		 payloadpacket.trigger_num );
    }
    fclose(payload_fd);

    return true;
}


// attempt to work around some system dependent issues.  Our read can
// return < data than we want.
int serial_read( SGSerialPort *serial, SGIOChannel *log,
		 uint8_t *buf, int length )
{
    int bytes_read = 0;
    static unsigned int total_bytes = 0;
    static SGPropertyNode *telemetry_rate_node
	= pyGetNode("/comms/telemetry-input-bytes-per-sec", true);

    bytes_read = serial->read_port( (char *)buf, length );

    if ( bytes_read > 0 && log != NULL ) {
	log->write( (char *)buf, bytes_read );
    }

    if ( bytes_read > 0 ) {
	static double first_read_sec = get_Time();
	double current_read_sec = get_Time();
	total_bytes += bytes_read;
	double elapsed_sec = current_read_sec - first_read_sec;
	if ( elapsed_sec > 0.0 ) {
	    double byte_rate = total_bytes / elapsed_sec;
	    telemetry_rate_node->setDoubleValue(byte_rate);
	}
    }

    return bytes_read;
}


static void glean_ascii_msgs( const char c ) {
    static string line = "";

    // if alphanumeric or punctuation, add to the line
    if ( isalnum(c) || ispunct(c) || isblank(c) ) {
	line.push_back(c);
    }
    if ( c == '\n' || line.length() > 80 ) {
	cout << "Remote: " << line << endl;
	line.clear();
    }	
}


// load the next message of a real time data stream
int UGTrack::next_message( gzFile fd, SGIOChannel *log,
			   struct gps *gpspacket,
			   struct imu *imupacket,
			   struct airdata *airpacket,
			   struct filter *filterpacket,
			   struct actuator *actpacket,
			   struct pilot *pilotpacket,
			   struct apstatus *appacket,
			   struct health *healthpacket,
			   struct payload *payloadpacket,
			   bool ignore_checksum )
{
    char tmpbuf[256];
    char savebuf[256];

    // cout << "in next_message()" << endl;

    bool myeof = false;

    // scan for sync characters
    uint8_t sync0, sync1;
    gzread( fd, tmpbuf, 2 );
    sync0 = (unsigned char)tmpbuf[0];
    sync1 = (unsigned char)tmpbuf[1];
    while ( (sync0 != START_OF_MSG0 || sync1 != START_OF_MSG1) && !myeof ) {
        sync0 = sync1;
        gzread( fd, tmpbuf, 1 ); sync1 = (unsigned char)tmpbuf[0];
        cout << "scanning for start of message "
	     << (unsigned int)sync0 << " " << (unsigned int)sync1
	     << ", eof = " << gzeof(fd) << endl;
	myeof = gzeof(fd);
    }

    // cout << "found start of message ..." << endl;

    // read message id and size
    gzread( fd, tmpbuf, 2 );
    uint8_t id = (unsigned char)tmpbuf[0];
    uint8_t size = (unsigned char)tmpbuf[1];
    // cout << "message = " << (int)id << " size = " << (int)size << endl;

    // load message
    int count = gzread( fd, savebuf, size );
    if ( count != size ) {
	cout << "ERROR: didn't read enough bytes!" << endl;
    }

    // read checksum
    gzread( fd, tmpbuf, 2 );
    uint8_t cksum0 = (unsigned char)tmpbuf[0];
    uint8_t cksum1 = (unsigned char)tmpbuf[1];
    
    if ( validate_cksum( id, size, savebuf, cksum0, cksum1, ignore_checksum ) )
    {
        parse_msg( id, savebuf, gpspacket, imupacket, airpacket, filterpacket,
		   actpacket, pilotpacket, appacket, healthpacket,
		   payloadpacket );
        return id;
    }

    cout << "Check sum failure!" << endl;
    return -1;
}


// load the next message of a real time data stream
int UGTrack::next_message( SGSerialPort *serial, SGIOChannel *log,
                           struct gps *gpspacket,
			   struct imu *imupacket,
			   struct airdata *airpacket,
			   struct filter *filterpacket,
			   struct actuator *actpacket,
			   struct pilot *pilotpacket,
			   struct apstatus *appacket,
			   struct health *healthpacket,
			   struct payload *payloadpacket,
                           bool ignore_checksum )
{
    static int state = 0;
    static int pkt_id = 0;
    static int pkt_len = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    int msg_id = -1;

    // printf("enter routine, state = %d\n", state);

#if 0
    // setup select
    int fd = serial->get_fd() + 1;
    fd_set input_fds;
    FD_ZERO(&input_fds);
    FD_SET(fd, &input_fds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    int n = select( fd + 1, &input_fds, NULL, NULL, &timeout );
    printf("select n = %d\n", n);

    if ( n < 0 ) {
	// error
    } else if ( n == 0 ) {
	// timeout with no input
	printf("input timeout ...\n");
    } else {
	if ( FD_ISSET(fd, &input_fds) ) {
#endif
	    if ( state == 0 ) {
		counter = 0;
		cksum_A = cksum_B = 0;
		len = serial_read( serial, log, input, 1 );
		while ( len > 0 && input[0] != START_OF_MSG0 ) {
		    // printf( " state0: len = %d val = %2X\n", len, input[0] );
	            glean_ascii_msgs( input[0] );
		    len = serial_read( serial, log, input, 1 );
		}
		if ( len > 0 && input[0] == START_OF_MSG0 ) {
		    // printf( " read START_OF_MSG0\n");
		    state++;
		}
	    }
	    if ( state == 1 ) {
		len = serial_read( serial, log, input, 1 );
		if ( len > 0 ) {
		    if ( input[0] == START_OF_MSG1 ) {
			// printf( " read START_OF_MSG1\n");
			state++;
		    } else if ( input[0] == START_OF_MSG0 ) {
			// printf( " read START_OF_MSG0\n");
		    } else {
			state = 0;
		    }
		}
	    }
	    if ( state == 2 ) {
		len = serial_read( serial, log, input, 1 );
		if ( len > 0 ) {
		    pkt_id = input[0];
		    cksum_A += input[0];
		    cksum_B += cksum_A;
		    // printf( " pkt_id = %d\n", pkt_id );
		    state++;
		}
	    }
	    if ( state == 3 ) {
		len = serial_read( serial, log, input, 1 );
		if ( len > 0 ) {
		    pkt_len = input[0];
		    if ( pkt_len < 255 ) {
			// printf( " pkt_len = %d\n", pkt_len );
			// printf( " payload = ");
			cksum_A += input[0];
			cksum_B += cksum_A;
			state++;
		    } else {
			state = 0;
		    }
		}
	    }
	    if ( state == 4 ) {
		len = serial_read( serial, log, input, 1 );
		while ( len > 0 ) {
		    payload[counter++] = input[0];
		    // printf( "%02X ", input[0] );
		    cksum_A += input[0];
		    cksum_B += cksum_A;
		    if ( counter >= pkt_len ) {
			state++;
			// printf( "\n" );
			break;
		    }
		    len = serial_read( serial, log, input, 1 );
		}
	    }
	    if ( state == 5 ) {
		len = serial_read( serial, log, input, 1 );
		if ( len > 0 ) {
		    cksum_lo = input[0];
		    // printf( " cksum_lo = %d\n", cksum_lo );
		    state++;
		}
	    }
	    if ( state == 6 ) {
		len = serial_read( serial, log, input, 1 );
		if ( len > 0 ) {
		    cksum_hi = input[0];
		    // printf( " cksum_hi = %d\n", cksum_hi );
		    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
			// printf( "checksum passes (%d)!\n", pkt_id );
			parse_msg( pkt_id, (char *)payload, gpspacket,
				   imupacket, airpacket, filterpacket,
				   actpacket, pilotpacket, appacket,
				   healthpacket, payloadpacket );
			msg_id = pkt_id;
		    } else {
			printf("pkt=%d checksum failed %d %d (computed) != %d %d (message)\n",
			       pkt_id, cksum_A, cksum_B, cksum_lo, cksum_hi );
		    }

		    // this is the end of a record, reset state to 0 to start
		    // looking for next record
		    state = 0;
		}
	    }
#if 0 // select
	}
    }
#endif


    //if ( msg_id == -1 ) {
    //  perror("read fail");
    // }
    //printf("exit routine... (%d)\n", msg_id);

    return msg_id;
}


static double interp( double a, double b, double p, bool rotational = false, bool deg = false ) {
    double diff = b - a;
    if ( rotational ) {
        // special handling of rotational data
	if ( !deg ) {
	    if ( diff > SGD_PI ) {
		diff -= SGD_2PI;
	    } else if ( diff < -SGD_PI ) {
		diff += SGD_2PI;
	    }
	} else {
	    if ( diff > 180.0 ) {
		diff -= 360.0;
	    } else if ( diff < -180.0 ) {
		diff += 360.0;
	    }
	}
    }
    return a + diff * p;
}


gps UGEARInterpGPS( const gps A, const gps B, const double percent )
{
    gps p;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.lat = interp(A.lat, B.lat, percent);
    p.lon = interp(A.lon, B.lon, percent);
    p.alt = interp(A.alt, B.alt, percent);
    p.ve = interp(A.ve, B.ve, percent);
    p.vn = interp(A.vn, B.vn, percent);
    p.vd = interp(A.vd, B.vd, percent);
    p.gps_time = interp(A.gps_time, B.gps_time, percent);
    p.satellites = A.satellites;
    p.status = A.status;

    return p;
}

imu UGEARInterpIMU( const imu A, const imu B, const double percent )
{
    imu p;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.p = interp(A.p, B.p, percent);
    p.q = interp(A.q, B.q, percent);
    p.r = interp(A.r, B.r, percent);
    p.ax = interp(A.ax, B.ax, percent);
    p.ay = interp(A.ay, B.ay, percent);
    p.az = interp(A.az, B.az, percent);
    p.hx = interp(A.hx, B.hx, percent);
    p.hy = interp(A.hy, B.hy, percent);
    p.hz = interp(A.hz, B.hz, percent);
    p.temp = interp(A.temp, B.temp, percent);
    p.status = A.status;

    return p;
}

airdata UGEARInterpAIR( const airdata A, const airdata B, const double percent )
{
    airdata p;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.pressure = interp(A.pressure, B.pressure, percent);
    p.temperature = interp(A.temperature, B.temperature, percent);
    p.airspeed = interp(A.airspeed, B.airspeed, percent);
    p.altitude = interp(A.altitude, B.altitude, percent);
    p.altitude_true = interp(A.altitude_true, B.altitude_true, percent);
    p.climb_fpm = interp(A.climb_fpm, B.climb_fpm, percent);
    p.acceleration = interp(A.acceleration, B.acceleration, percent);
    p.wind_dir = interp(A.wind_dir, B.wind_dir, percent);
    p.wind_speed = interp(A.wind_speed, B.wind_speed, percent);
    p.pitot_scale = interp(A.pitot_scale, B.pitot_scale, percent);
    p.status = A.status;

    return p;
}

filter UGEARInterpFILTER( const filter A, const filter B, const double percent )
{
    filter p;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.lat = interp(A.lat, B.lat, percent);
    p.lon = interp(A.lon, B.lon, percent);
    p.alt = interp(A.alt, B.alt, percent);
    p.ve = interp(A.ve, B.ve, percent);
    p.vn = interp(A.vn, B.vn, percent);
    p.vd = interp(A.vd, B.vd, percent);
    p.phi = interp(A.phi, B.phi, percent, true, true);
    p.theta = interp(A.theta, B.theta, percent, true, true);
    p.psi = interp(A.psi, B.psi, percent, true, true);
    p.status = A.status;
    p.command_seq = A.command_seq;

    return p;
}


actuator UGEARInterpACT( const actuator A, const actuator B, const double percent )
{
    actuator p;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.ail = interp(A.ail, B.ail, percent);
    p.ele = interp(A.ele, B.ele, percent);
    p.thr = interp(A.thr, B.thr, percent);
    p.rud = interp(A.rud, B.rud, percent);
    p.ch5 = interp(A.ch5, B.ch5, percent);
    p.ch6 = interp(A.ch6, B.ch6, percent);
    p.ch7 = interp(A.ch7, B.ch7, percent);
    p.ch8 = interp(A.ch8, B.ch8, percent);
    p.status = A.status;

    return p;
}


pilot UGEARInterpPILOT( const pilot A, const pilot B, const double percent )
{
    pilot p;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.ail = interp(A.ail, B.ail, percent);
    p.ele = interp(A.ele, B.ele, percent);
    p.thr = interp(A.thr, B.thr, percent);
    p.rud = interp(A.rud, B.rud, percent);
    p.ch5 = interp(A.ch5, B.ch5, percent);
    p.ch6 = interp(A.ch6, B.ch6, percent);
    p.ch7 = interp(A.ch7, B.ch7, percent);
    p.ch8 = interp(A.ch8, B.ch8, percent);
    p.status = A.status;

    return p;
}


apstatus UGEARInterpAP( const apstatus A, const apstatus B, const double percent )
{
    apstatus p = B;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.target_heading_deg = interp(A.target_heading_deg, B.target_heading_deg, percent, true, true);
    p.target_roll_deg = interp(A.target_roll_deg, B.target_roll_deg, percent);
    p.target_altitude_msl_ft = interp(A.target_altitude_msl_ft, B.target_altitude_msl_ft, percent);
    p.target_climb_fps = interp(A.target_climb_fps, B.target_climb_fps, percent);
    p.target_pitch_deg = interp(A.target_pitch_deg, B.target_pitch_deg, percent);
    p.target_theta_dot = interp(A.target_theta_dot, B.target_theta_dot, percent);
    p.target_speed_kt = interp(A.target_speed_kt, B.target_speed_kt, percent);
    p.target_wp = A.target_wp;
    p.wp_lon = A.wp_lon;
    p.wp_lat = A.wp_lat;
    p.wp_index = A.wp_index;
    p.route_size = A.route_size;

    return p;
}

health UGEARInterpHEALTH( const health A, const health B, const double percent )
{
    health p = B;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.load_avg = interp(A.load_avg, B.load_avg, percent);
    p.avionics_vcc = interp(A.avionics_vcc, B.avionics_vcc, percent);
    p.extern_volts = interp(A.extern_volts, B.extern_volts, percent);
    p.extern_cell_volts = interp(A.extern_cell_volts, B.extern_cell_volts, percent);
    p.extern_amps = interp(A.extern_amps, B.extern_amps, percent);
    p.extern_mah = interp(A.extern_mah, B.extern_mah, percent);

    return p;
}

payload UGEARInterpPAYLOAD( const payload A, const payload B,
			    const double percent )
{
    payload p = B;
    p.timestamp = interp(A.timestamp, B.timestamp, percent);
    p.trigger_num = B.trigger_num;

    return p;
}

