//
// FILE: Aura4.h
// DESCRIPTION: interact with Aura4 (Teensy/Pika) sensor head
//

#pragma once

#include <pyprops.h>

#include "drivers/driver.h"

#include "include/globaldefs.h" /* fixme, get rid of? */

#include "aura4_messages.h"

class Aura4_t: public driver_t {
    
public:
    Aura4_t() {}
    ~Aura4_t() {}
    void init( pyPropertyNode *config );
    void read() {}
    void process() {}
    void write() {}
    void close() {}

private:
    string device_name = "/dev/ttyS4";
    int baud = 500000;
    
    void info( const char* format, ... );
    void hard_error( const char*format, ... );
    
    bool open( pyPropertyNode *config );
    void init_airdata( pyPropertyNode *config );
    void init_gps( pyPropertyNode *config );
    void init_imu( pyPropertyNode *config );
    void init_pilot( pyPropertyNode *config );
    void init_actuators( pyPropertyNode *config );

    bool send_config();
    bool write_config_master();
    bool write_config_imu();
    bool write_config_mixer();
    bool write_config_pwm();
    bool write_config_airdata();
    bool write_config_led();
    bool write_config_power();
    bool write_config_stab();
    bool write_command_zero_gyros();
    bool write_command_cycle_inceptors();
    bool write_command_reset_ekf();
    bool wait_for_ack(uint8_t id);

    double update();
    bool parse( uint8_t pkt_id, uint8_t pkt_len, uint8_t *payload );
    bool update_imu( message::imu_raw_t *imu );

    bool update_actuators();
};

// function prototypes

double Aura4_update();
void Aura4_close();
bool Aura4_request_baud( uint32_t baud );

bool Aura4_gps_init( string output_path, pyPropertyNode *config  );
bool Aura4_gps_update();
void Aura4_gps_close();

bool Aura4_airdata_init( string output_path );
bool Aura4_airdata_update();
// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void Aura4_airdata_zero_airspeed();
void Aura4_airdata_close();

bool Aura4_pilot_init( string output_path, pyPropertyNode *config );
bool Aura4_pilot_update();
void Aura4_pilot_close();

bool Aura4_act_init( pyPropertyNode *config );
bool Aura4_act_update();
void Aura4_act_close();

extern bool Aura4_actuator_configured;
