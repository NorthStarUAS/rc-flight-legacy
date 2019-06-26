#pragma once

#include <stdint.h>  // uint8_t, et. al.
#include <string.h>  // memcpy()

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t message_command_ack_id = 20;
const uint8_t message_config_master_id = 21;
const uint8_t message_config_imu_id = 22;
const uint8_t message_config_actuators_id = 23;
const uint8_t message_config_airdata_id = 24;
const uint8_t message_config_power_id = 25;
const uint8_t message_config_led_id = 26;
const uint8_t message_command_inceptors_id = 40;
const uint8_t message_command_zero_gyros_id = 41;
const uint8_t message_command_cycle_inceptors_id = 42;

// Message: command_ack (id: 20)
struct message_command_ack_t {
    // public fields
    uint8_t command_id;
    uint8_t subcommand_id;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t command_id;
        uint8_t subcommand_id;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 20;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        _buf.command_id = command_id;
        _buf.subcommand_id = subcommand_id;
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        command_id = _buf.command_id;
        subcommand_id = _buf.subcommand_id;
    }
};

// Message: config_master (id: 21)
struct message_config_master_t {
    // public fields
    uint8_t board;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t board;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 21;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        _buf.board = board;
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        board = _buf.board;
    }
};

// Message: config_imu (id: 22)
struct message_config_imu_t {
    // public fields
    uint8_t interface;
    uint8_t pin_or_address;
    float orientation[9];

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t interface;
        uint8_t pin_or_address;
        float orientation[9];
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 22;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        _buf.interface = interface;
        _buf.pin_or_address = pin_or_address;
        for (int _i=0; _i<9; _i++) _buf.orientation[_i] = orientation[_i];
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        interface = _buf.interface;
        pin_or_address = _buf.pin_or_address;
        for (int _i=0; _i<9; _i++) orientation[_i] = _buf.orientation[_i];
    }
};

// Message: config_actuators (id: 23)
struct message_config_actuators_t {
    // public fields
    uint16_t pwm_hz[8];
    float act_gain[8];
    bool mix_autocoord;
    bool mix_throttle_trim;
    bool mix_flap_trim;
    bool mix_elevon;
    bool mix_flaperon;
    bool mix_vtail;
    bool mix_diff_thrust;
    float mix_Gac;
    float mix_Get;
    float mix_Gef;
    float mix_Gea;
    float mix_Gee;
    float mix_Gfa;
    float mix_Gff;
    float mix_Gve;
    float mix_Gvr;
    float mix_Gtt;
    float mix_Gtr;
    bool sas_rollaxis;
    bool sas_pitchaxis;
    bool sas_yawaxis;
    bool sas_tune;
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_max_gain;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint16_t pwm_hz[8];
        float act_gain[8];
        bool mix_autocoord;
        bool mix_throttle_trim;
        bool mix_flap_trim;
        bool mix_elevon;
        bool mix_flaperon;
        bool mix_vtail;
        bool mix_diff_thrust;
        float mix_Gac;
        float mix_Get;
        float mix_Gef;
        float mix_Gea;
        float mix_Gee;
        float mix_Gfa;
        float mix_Gff;
        float mix_Gve;
        float mix_Gvr;
        float mix_Gtt;
        float mix_Gtr;
        bool sas_rollaxis;
        bool sas_pitchaxis;
        bool sas_yawaxis;
        bool sas_tune;
        float sas_rollgain;
        float sas_pitchgain;
        float sas_yawgain;
        float sas_max_gain;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 23;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        for (int _i=0; _i<8; _i++) _buf.pwm_hz[_i] = pwm_hz[_i];
        for (int _i=0; _i<8; _i++) _buf.act_gain[_i] = act_gain[_i];
        _buf.mix_autocoord = mix_autocoord;
        _buf.mix_throttle_trim = mix_throttle_trim;
        _buf.mix_flap_trim = mix_flap_trim;
        _buf.mix_elevon = mix_elevon;
        _buf.mix_flaperon = mix_flaperon;
        _buf.mix_vtail = mix_vtail;
        _buf.mix_diff_thrust = mix_diff_thrust;
        _buf.mix_Gac = mix_Gac;
        _buf.mix_Get = mix_Get;
        _buf.mix_Gef = mix_Gef;
        _buf.mix_Gea = mix_Gea;
        _buf.mix_Gee = mix_Gee;
        _buf.mix_Gfa = mix_Gfa;
        _buf.mix_Gff = mix_Gff;
        _buf.mix_Gve = mix_Gve;
        _buf.mix_Gvr = mix_Gvr;
        _buf.mix_Gtt = mix_Gtt;
        _buf.mix_Gtr = mix_Gtr;
        _buf.sas_rollaxis = sas_rollaxis;
        _buf.sas_pitchaxis = sas_pitchaxis;
        _buf.sas_yawaxis = sas_yawaxis;
        _buf.sas_tune = sas_tune;
        _buf.sas_rollgain = sas_rollgain;
        _buf.sas_pitchgain = sas_pitchgain;
        _buf.sas_yawgain = sas_yawgain;
        _buf.sas_max_gain = sas_max_gain;
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        for (int _i=0; _i<8; _i++) pwm_hz[_i] = _buf.pwm_hz[_i];
        for (int _i=0; _i<8; _i++) act_gain[_i] = _buf.act_gain[_i];
        mix_autocoord = _buf.mix_autocoord;
        mix_throttle_trim = _buf.mix_throttle_trim;
        mix_flap_trim = _buf.mix_flap_trim;
        mix_elevon = _buf.mix_elevon;
        mix_flaperon = _buf.mix_flaperon;
        mix_vtail = _buf.mix_vtail;
        mix_diff_thrust = _buf.mix_diff_thrust;
        mix_Gac = _buf.mix_Gac;
        mix_Get = _buf.mix_Get;
        mix_Gef = _buf.mix_Gef;
        mix_Gea = _buf.mix_Gea;
        mix_Gee = _buf.mix_Gee;
        mix_Gfa = _buf.mix_Gfa;
        mix_Gff = _buf.mix_Gff;
        mix_Gve = _buf.mix_Gve;
        mix_Gvr = _buf.mix_Gvr;
        mix_Gtt = _buf.mix_Gtt;
        mix_Gtr = _buf.mix_Gtr;
        sas_rollaxis = _buf.sas_rollaxis;
        sas_pitchaxis = _buf.sas_pitchaxis;
        sas_yawaxis = _buf.sas_yawaxis;
        sas_tune = _buf.sas_tune;
        sas_rollgain = _buf.sas_rollgain;
        sas_pitchgain = _buf.sas_pitchgain;
        sas_yawgain = _buf.sas_yawgain;
        sas_max_gain = _buf.sas_max_gain;
    }
};

// Message: config_airdata (id: 24)
struct message_config_airdata_t {
    // public fields
    uint8_t barometer;
    uint8_t pitot;
    uint8_t swift_baro_addr;
    uint8_t swift_pitot_addr;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t barometer;
        uint8_t pitot;
        uint8_t swift_baro_addr;
        uint8_t swift_pitot_addr;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 24;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        _buf.barometer = barometer;
        _buf.pitot = pitot;
        _buf.swift_baro_addr = swift_baro_addr;
        _buf.swift_pitot_addr = swift_pitot_addr;
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        barometer = _buf.barometer;
        pitot = _buf.pitot;
        swift_baro_addr = _buf.swift_baro_addr;
        swift_pitot_addr = _buf.swift_pitot_addr;
    }
};

// Message: config_power (id: 25)
struct message_config_power_t {
    // public fields
    bool have_attopilot;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        bool have_attopilot;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 25;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        _buf.have_attopilot = have_attopilot;
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        have_attopilot = _buf.have_attopilot;
    }
};

// Message: config_led (id: 26)
struct message_config_led_t {
    // public fields
    uint8_t pin;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t pin;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 26;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        _buf.pin = pin;
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        pin = _buf.pin;
    }
};

// Message: command_inceptors (id: 40)
struct message_command_inceptors_t {
    // public fields
    float channel[6];

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        int16_t channel[6];
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 40;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        for (int _i=0; _i<6; _i++) _buf.channel[_i] = intround(channel[_i] * 16384);
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        for (int _i=0; _i<6; _i++) channel[_i] = _buf.channel[_i] / (float)16384;
    }
};

// Message: command_zero_gyros (id: 41)
struct message_command_zero_gyros_t {
    // public fields

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 41;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
    }
};

// Message: command_cycle_inceptors (id: 42)
struct message_command_cycle_inceptors_t {
    // public fields

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 42;
    static const uint16_t len = sizeof(_buf);

    uint8_t *pack() {
        return (uint8_t *)(&_buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
    }
};

