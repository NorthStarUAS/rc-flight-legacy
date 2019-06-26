#pragma once

#pragma pack(push,1)            // set alignment to 1 byte boundary

// Master Board Configuration
typedef struct {
    uint8_t board;              // 0 = Marmot v1, 1 = Aura v2
} config_master_t;

// IMU Configuration
typedef struct {
    uint8_t interface;          // 0 = SPI, 1 = I2C
    uint8_t pin_or_address;     // SPI CS pin or I2C address
    float orientation[9];       // IMU orientation matrix
} config_imu_t;

// Actuators Configuration
typedef struct {
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
} config_act_t;

// Air Data Configuration
typedef struct {
    uint8_t barometer;          // 0 = BME280/SPI, 1 = BMP280/I2C, 2 = BFS Swift
    uint8_t pitot;              // 0 = MS4525, 1 = MS5525, 2 = BFS Swift
    uint8_t swift_baro_addr;
    uint8_t swift_pitot_addr;
} config_airdata_t;

// Power Configuration
typedef struct {
    bool have_attopilot;
} config_power_t;

// LED Configuration
typedef struct {
    uint8_t pin;                // 0 = no LED
} config_led_t;

// master config (for messages and saving in eeprom)
typedef struct {
    config_master_t master;
    config_imu_t imu;
    config_act_t actuators;
    config_airdata_t airdata;
    config_power_t power;
    config_led_t led;
} config_t;

#pragma pack(pop)
