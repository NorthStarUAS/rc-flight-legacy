//
// FILE: Aura4.h
// DESCRIPTION: interact with Aura4 (Teensy/Pika) sensor head
//

#pragma once

#include <pyprops.h>

#include "comms/serial_link.h"
#include "drivers/driver.h"
#include "include/globaldefs.h" /* fixme, get rid of? */
#include "util/butter.h"
#include "util/lowpass.h"

#include "aura4_messages.h"

class Aura4_t: public driver_t {
    
public:
    Aura4_t() {}
    ~Aura4_t() {}
    void init( pyPropertyNode *config );
    void read();
    void process() {}
    void write() {}
    void close();

private:
    pyPropertyNode aura4_node;
    pyPropertyNode power_node;
    pyPropertyNode imu_node;
    pyPropertyNode gps_node;
    pyPropertyNode pilot_node;
    pyPropertyNode act_node;
    pyPropertyNode airdata_node;
    pyPropertyNode aura4_config;
    
    string device_name = "/dev/ttyS4";
    int baud = 500000;
    SerialLink serial;
    bool configuration_sent = false;
    int last_ack_id = 0;
    int last_ack_subid = 0;

    float pitot_calibrate = 1.0;
    ButterworthFilter pitot_filter = ButterworthFilter(2, 100, 0.8);
    int battery_cells = 4;
    
    message::aura_nav_pvt_t nav_pvt;
    message::airdata_t airdata;
    message::config_master_t config_master;
    message::config_imu_t config_imu;
    message::config_mixer_t config_mixer;
    message::config_pwm_t config_pwm;
    message::config_airdata_t config_airdata;
    message::config_power_t config_power;
    message::config_led_t config_led;
    message::config_stab_damping_t config_stab;
    
    LowPassFilter avionics_vcc_filt = LowPassFilter(2.0);
    LowPassFilter int_main_vcc_filt = LowPassFilter(2.0);
    LowPassFilter ext_main_vcc_filt = LowPassFilter(2.0);


    void info( const char* format, ... );
    void hard_error( const char*format, ... );
    
    bool open( pyPropertyNode *config );
    void init_airdata( pyPropertyNode *config );
    void init_gps( pyPropertyNode *config );
    void init_imu( pyPropertyNode *config );
    void init_pilot( pyPropertyNode *config );
    void init_actuators( pyPropertyNode *config );

    void master_defaults();
    void airdata_defaults();
    void imu_setup_defaults();
    void led_defaults();
    void mixer_defaults();
    void power_defaults();
    void pwm_defaults();
    void stability_defaults();
    
    bool parse( uint8_t pkt_id, uint8_t pkt_len, uint8_t *payload );
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

    bool update_airdata();
    bool update_gps();
    bool update_imu( message::imu_raw_t *imu );
    bool update_pilot( message::pilot_t *pilot );
    
    bool update_actuators();
};
