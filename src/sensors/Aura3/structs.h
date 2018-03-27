#ifndef AURA_STRUCTURES_H_INCLUDED
#define AURA_STRUCTURES_H_INCLUDED

#include "config_pwm.h"
#include "config_sbus.h"


#pragma pack(push, 1)           // set alignment to 1 byte boundary

//////////////////////////////////////////////////////
// master config (for messages and saving in eeprom)
//////////////////////////////////////////////////////

typedef struct {
    int version;

    float imu_orient[9];        // IMU orientation matrix
    
    // pwm output signal hz, 50hz default for analog servos, maximum
    // rate is servo dependent: digital servos can usually do
    // 200-250hz, analog servos and ESC's typically require 50hz
    uint16_t pwm_hz[PWM_CHANNELS];
    
    float act_gain[PWM_CHANNELS]; // actuator gain (reversing/scaling)
    
    // mixing modes
    bool mix_autocoord;
    bool mix_throttle_trim;
    bool mix_flap_trim;
    bool mix_elevon;
    bool mix_flaperon;
    bool mix_vtail;
    bool mix_diff_thrust;

    // mixing gains
    float mix_Gac;              // aileron gain for autocoordination
    float mix_Get;              // elevator trim w/ throttle gain
    float mix_Gef;              // elevator trim w/ flap gain
    float mix_Gea;              // aileron gain for elevons
    float mix_Gee;              // elevator gain for elevons
    float mix_Gfa;              // aileron gain for flaperons
    float mix_Gff;              // flaps gain for flaperons
    float mix_Gve;              // elevator gain for vtail
    float mix_Gvr;              // rudder gain for vtail
    float mix_Gtt;              // throttle gain for diff thrust
    float mix_Gtr;              // rudder gain for diff thrust
    
    // axis damping modes
    bool sas_rollaxis;
    bool sas_pitchaxis;
    bool sas_yawaxis;
    bool sas_tune;

    // axis damping gains
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_max_gain;
} config_t;


//////////////////////////////////////////////////////
// communication structures (note: some of these
// structures may have altered types or units to
// minimize packet lengths.)
//////////////////////////////////////////////////////

// pilot input (from RC system)
typedef struct {
    int16_t channel[SBUS_CHANNELS];
    uint8_t flags;
} pilot_packet_t;

// imu
typedef struct {
    uint32_t micros;
    int16_t channel[10];
} imu_packet_t;

#pragma pack(pop)


#endif // AURA_STRUCTURES_H_INCLUDED
