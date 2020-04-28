#pragma once

#include <stdint.h>  // uint8_t, et. al.
#include <string.h>  // memcpy()

namespace message {

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t command_ack_id = 10;
const uint8_t config_airdata_id = 11;
const uint8_t config_board_id = 12;
const uint8_t config_ekf_id = 13;
const uint8_t config_imu_id = 14;
const uint8_t config_mixer_id = 15;
const uint8_t config_mixer_matrix_id = 16;
const uint8_t config_power_id = 17;
const uint8_t config_pwm_id = 18;
const uint8_t config_stability_damping_id = 19;
const uint8_t command_inceptors_id = 20;
const uint8_t command_zero_gyros_id = 21;
const uint8_t command_reset_ekf_id = 22;
const uint8_t command_cycle_inceptors_id = 23;
const uint8_t pilot_id = 24;
const uint8_t imu_id = 25;
const uint8_t aura_nav_pvt_id = 26;
const uint8_t airdata_id = 27;
const uint8_t power_id = 28;
const uint8_t status_id = 29;
const uint8_t ekf_id = 30;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

// Constants
static const uint8_t pwm_channels = 8;  // number of pwm output channels
static const uint8_t sbus_channels = 16;  // number of sbus channels
static const uint8_t ap_channels = 6;  // number of sbus channels
static const uint8_t mix_matrix_size = 64;  // 8 x 8 mix matrix

// Enums
enum class enum_nav {
    none = 0,  // disable nav filter
    nav15 = 1,  // 15-state ins/gps filter
    nav15_mag = 2  // 15-state ins/gps/mag filter
};

// Message: command_ack (id: 10)
struct command_ack_t {
    // public fields
    uint8_t command_id;
    uint8_t subcommand_id;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t command_id;
        uint8_t subcommand_id;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 10;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->command_id = command_id;
        _buf->subcommand_id = subcommand_id;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        command_id = _buf->command_id;
        subcommand_id = _buf->subcommand_id;
        return true;
    }
};

// Message: config_airdata (id: 11)
struct config_airdata_t {
    // public fields
    uint8_t barometer;
    uint8_t pitot;
    uint8_t swift_baro_addr;
    uint8_t swift_pitot_addr;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t barometer;
        uint8_t pitot;
        uint8_t swift_baro_addr;
        uint8_t swift_pitot_addr;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 11;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->barometer = barometer;
        _buf->pitot = pitot;
        _buf->swift_baro_addr = swift_baro_addr;
        _buf->swift_pitot_addr = swift_pitot_addr;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        barometer = _buf->barometer;
        pitot = _buf->pitot;
        swift_baro_addr = _buf->swift_baro_addr;
        swift_pitot_addr = _buf->swift_pitot_addr;
        return true;
    }
};

// Message: config_board (id: 12)
struct config_board_t {
    // public fields
    uint8_t board;
    uint8_t led_pin;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t board;
        uint8_t led_pin;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 12;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->board = board;
        _buf->led_pin = led_pin;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        board = _buf->board;
        led_pin = _buf->led_pin;
        return true;
    }
};

// Message: config_ekf (id: 13)
struct config_ekf_t {
    // public fields
    enum_nav select;
    float sig_w_accel;
    float sig_w_gyro;
    float sig_a_d;
    float tau_a;
    float sig_g_d;
    float tau_g;
    float sig_gps_p_ne;
    float sig_gps_p_d;
    float sig_gps_v_ne;
    float sig_gps_v_d;
    float sig_mag;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t select;
        float sig_w_accel;
        float sig_w_gyro;
        float sig_a_d;
        float tau_a;
        float sig_g_d;
        float tau_g;
        float sig_gps_p_ne;
        float sig_gps_p_d;
        float sig_gps_v_ne;
        float sig_gps_v_d;
        float sig_mag;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 13;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->select = (uint8_t)select;
        _buf->sig_w_accel = sig_w_accel;
        _buf->sig_w_gyro = sig_w_gyro;
        _buf->sig_a_d = sig_a_d;
        _buf->tau_a = tau_a;
        _buf->sig_g_d = sig_g_d;
        _buf->tau_g = tau_g;
        _buf->sig_gps_p_ne = sig_gps_p_ne;
        _buf->sig_gps_p_d = sig_gps_p_d;
        _buf->sig_gps_v_ne = sig_gps_v_ne;
        _buf->sig_gps_v_d = sig_gps_v_d;
        _buf->sig_mag = sig_mag;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        select = (enum_nav)_buf->select;
        sig_w_accel = _buf->sig_w_accel;
        sig_w_gyro = _buf->sig_w_gyro;
        sig_a_d = _buf->sig_a_d;
        tau_a = _buf->tau_a;
        sig_g_d = _buf->sig_g_d;
        tau_g = _buf->tau_g;
        sig_gps_p_ne = _buf->sig_gps_p_ne;
        sig_gps_p_d = _buf->sig_gps_p_d;
        sig_gps_v_ne = _buf->sig_gps_v_ne;
        sig_gps_v_d = _buf->sig_gps_v_d;
        sig_mag = _buf->sig_mag;
        return true;
    }
};

// Message: config_imu (id: 14)
struct config_imu_t {
    // public fields
    uint8_t interface;
    uint8_t pin_or_address;
    float strapdown_calib[9];
    float accel_scale[3];
    float accel_translate[3];
    float mag_affine[16];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t interface;
        uint8_t pin_or_address;
        float strapdown_calib[9];
        float accel_scale[3];
        float accel_translate[3];
        float mag_affine[16];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 14;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->interface = interface;
        _buf->pin_or_address = pin_or_address;
        for (int _i=0; _i<9; _i++) _buf->strapdown_calib[_i] = strapdown_calib[_i];
        for (int _i=0; _i<3; _i++) _buf->accel_scale[_i] = accel_scale[_i];
        for (int _i=0; _i<3; _i++) _buf->accel_translate[_i] = accel_translate[_i];
        for (int _i=0; _i<16; _i++) _buf->mag_affine[_i] = mag_affine[_i];
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        interface = _buf->interface;
        pin_or_address = _buf->pin_or_address;
        for (int _i=0; _i<9; _i++) strapdown_calib[_i] = _buf->strapdown_calib[_i];
        for (int _i=0; _i<3; _i++) accel_scale[_i] = _buf->accel_scale[_i];
        for (int _i=0; _i<3; _i++) accel_translate[_i] = _buf->accel_translate[_i];
        for (int _i=0; _i<16; _i++) mag_affine[_i] = _buf->mag_affine[_i];
        return true;
    }
};

// Message: config_mixer (id: 15)
struct config_mixer_t {
    // public fields
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

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
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
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 15;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->mix_autocoord = mix_autocoord;
        _buf->mix_throttle_trim = mix_throttle_trim;
        _buf->mix_flap_trim = mix_flap_trim;
        _buf->mix_elevon = mix_elevon;
        _buf->mix_flaperon = mix_flaperon;
        _buf->mix_vtail = mix_vtail;
        _buf->mix_diff_thrust = mix_diff_thrust;
        _buf->mix_Gac = mix_Gac;
        _buf->mix_Get = mix_Get;
        _buf->mix_Gef = mix_Gef;
        _buf->mix_Gea = mix_Gea;
        _buf->mix_Gee = mix_Gee;
        _buf->mix_Gfa = mix_Gfa;
        _buf->mix_Gff = mix_Gff;
        _buf->mix_Gve = mix_Gve;
        _buf->mix_Gvr = mix_Gvr;
        _buf->mix_Gtt = mix_Gtt;
        _buf->mix_Gtr = mix_Gtr;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        mix_autocoord = _buf->mix_autocoord;
        mix_throttle_trim = _buf->mix_throttle_trim;
        mix_flap_trim = _buf->mix_flap_trim;
        mix_elevon = _buf->mix_elevon;
        mix_flaperon = _buf->mix_flaperon;
        mix_vtail = _buf->mix_vtail;
        mix_diff_thrust = _buf->mix_diff_thrust;
        mix_Gac = _buf->mix_Gac;
        mix_Get = _buf->mix_Get;
        mix_Gef = _buf->mix_Gef;
        mix_Gea = _buf->mix_Gea;
        mix_Gee = _buf->mix_Gee;
        mix_Gfa = _buf->mix_Gfa;
        mix_Gff = _buf->mix_Gff;
        mix_Gve = _buf->mix_Gve;
        mix_Gvr = _buf->mix_Gvr;
        mix_Gtt = _buf->mix_Gtt;
        mix_Gtr = _buf->mix_Gtr;
        return true;
    }
};

// Message: config_mixer_matrix (id: 16)
struct config_mixer_matrix_t {
    // public fields
    float matrix[mix_matrix_size];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t matrix[mix_matrix_size];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 16;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        for (int _i=0; _i<mix_matrix_size; _i++) _buf->matrix[_i] = intround(matrix[_i] * 16384);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        for (int _i=0; _i<mix_matrix_size; _i++) matrix[_i] = _buf->matrix[_i] / (float)16384;
        return true;
    }
};

// Message: config_power (id: 17)
struct config_power_t {
    // public fields
    bool have_attopilot;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        bool have_attopilot;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 17;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->have_attopilot = have_attopilot;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        have_attopilot = _buf->have_attopilot;
        return true;
    }
};

// Message: config_pwm (id: 18)
struct config_pwm_t {
    // public fields
    uint16_t pwm_hz;
    float act_gain[pwm_channels];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t pwm_hz;
        float act_gain[pwm_channels];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 18;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->pwm_hz = pwm_hz;
        for (int _i=0; _i<pwm_channels; _i++) _buf->act_gain[_i] = act_gain[_i];
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        pwm_hz = _buf->pwm_hz;
        for (int _i=0; _i<pwm_channels; _i++) act_gain[_i] = _buf->act_gain[_i];
        return true;
    }
};

// Message: config_stability_damping (id: 19)
struct config_stability_damping_t {
    // public fields
    bool sas_rollaxis;
    bool sas_pitchaxis;
    bool sas_yawaxis;
    bool sas_tune;
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_max_gain;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        bool sas_rollaxis;
        bool sas_pitchaxis;
        bool sas_yawaxis;
        bool sas_tune;
        float sas_rollgain;
        float sas_pitchgain;
        float sas_yawgain;
        float sas_max_gain;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 19;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->sas_rollaxis = sas_rollaxis;
        _buf->sas_pitchaxis = sas_pitchaxis;
        _buf->sas_yawaxis = sas_yawaxis;
        _buf->sas_tune = sas_tune;
        _buf->sas_rollgain = sas_rollgain;
        _buf->sas_pitchgain = sas_pitchgain;
        _buf->sas_yawgain = sas_yawgain;
        _buf->sas_max_gain = sas_max_gain;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        sas_rollaxis = _buf->sas_rollaxis;
        sas_pitchaxis = _buf->sas_pitchaxis;
        sas_yawaxis = _buf->sas_yawaxis;
        sas_tune = _buf->sas_tune;
        sas_rollgain = _buf->sas_rollgain;
        sas_pitchgain = _buf->sas_pitchgain;
        sas_yawgain = _buf->sas_yawgain;
        sas_max_gain = _buf->sas_max_gain;
        return true;
    }
};

// Message: command_inceptors (id: 20)
struct command_inceptors_t {
    // public fields
    float channel[ap_channels];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t channel[ap_channels];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 20;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        for (int _i=0; _i<ap_channels; _i++) _buf->channel[_i] = intround(channel[_i] * 16384);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        for (int _i=0; _i<ap_channels; _i++) channel[_i] = _buf->channel[_i] / (float)16384;
        return true;
    }
};

// Message: command_zero_gyros (id: 21)
struct command_zero_gyros_t {
    // public fields

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 21;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        len = sizeof(_compact_t);
        return true;
    }
};

// Message: command_reset_ekf (id: 22)
struct command_reset_ekf_t {
    // public fields

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 22;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        len = sizeof(_compact_t);
        return true;
    }
};

// Message: command_cycle_inceptors (id: 23)
struct command_cycle_inceptors_t {
    // public fields

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 23;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        len = sizeof(_compact_t);
        return true;
    }
};

// Message: pilot (id: 24)
struct pilot_t {
    // public fields
    float channel[sbus_channels];
    uint8_t flags;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t channel[sbus_channels];
        uint8_t flags;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 24;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        for (int _i=0; _i<sbus_channels; _i++) _buf->channel[_i] = intround(channel[_i] * 16384);
        _buf->flags = flags;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        for (int _i=0; _i<sbus_channels; _i++) channel[_i] = _buf->channel[_i] / (float)16384;
        flags = _buf->flags;
        return true;
    }
};

// Message: imu (id: 25)
struct imu_t {
    // public fields
    uint32_t millis;
    int16_t raw[6];
    int16_t cal[10];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t raw[6];
        int16_t cal[10];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 25;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        for (int _i=0; _i<6; _i++) _buf->raw[_i] = raw[_i];
        for (int _i=0; _i<10; _i++) _buf->cal[_i] = cal[_i];
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        for (int _i=0; _i<6; _i++) raw[_i] = _buf->raw[_i];
        for (int _i=0; _i<10; _i++) cal[_i] = _buf->cal[_i];
        return true;
    }
};

// Message: aura_nav_pvt (id: 26)
struct aura_nav_pvt_t {
    // public fields
    uint32_t iTOW;
    int16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
    uint8_t reserved[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t iTOW;
        int16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint8_t valid;
        uint32_t tAcc;
        int32_t nano;
        uint8_t fixType;
        uint8_t flags;
        uint8_t flags2;
        uint8_t numSV;
        int32_t lon;
        int32_t lat;
        int32_t height;
        int32_t hMSL;
        uint32_t hAcc;
        uint32_t vAcc;
        int32_t velN;
        int32_t velE;
        int32_t velD;
        uint32_t gSpeed;
        int32_t heading;
        uint32_t sAcc;
        uint32_t headingAcc;
        uint16_t pDOP;
        uint8_t reserved[6];
        int32_t headVeh;
        int16_t magDec;
        uint16_t magAcc;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 26;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->iTOW = iTOW;
        _buf->year = year;
        _buf->month = month;
        _buf->day = day;
        _buf->hour = hour;
        _buf->min = min;
        _buf->sec = sec;
        _buf->valid = valid;
        _buf->tAcc = tAcc;
        _buf->nano = nano;
        _buf->fixType = fixType;
        _buf->flags = flags;
        _buf->flags2 = flags2;
        _buf->numSV = numSV;
        _buf->lon = lon;
        _buf->lat = lat;
        _buf->height = height;
        _buf->hMSL = hMSL;
        _buf->hAcc = hAcc;
        _buf->vAcc = vAcc;
        _buf->velN = velN;
        _buf->velE = velE;
        _buf->velD = velD;
        _buf->gSpeed = gSpeed;
        _buf->heading = heading;
        _buf->sAcc = sAcc;
        _buf->headingAcc = headingAcc;
        _buf->pDOP = pDOP;
        for (int _i=0; _i<6; _i++) _buf->reserved[_i] = reserved[_i];
        _buf->headVeh = headVeh;
        _buf->magDec = magDec;
        _buf->magAcc = magAcc;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        iTOW = _buf->iTOW;
        year = _buf->year;
        month = _buf->month;
        day = _buf->day;
        hour = _buf->hour;
        min = _buf->min;
        sec = _buf->sec;
        valid = _buf->valid;
        tAcc = _buf->tAcc;
        nano = _buf->nano;
        fixType = _buf->fixType;
        flags = _buf->flags;
        flags2 = _buf->flags2;
        numSV = _buf->numSV;
        lon = _buf->lon;
        lat = _buf->lat;
        height = _buf->height;
        hMSL = _buf->hMSL;
        hAcc = _buf->hAcc;
        vAcc = _buf->vAcc;
        velN = _buf->velN;
        velE = _buf->velE;
        velD = _buf->velD;
        gSpeed = _buf->gSpeed;
        heading = _buf->heading;
        sAcc = _buf->sAcc;
        headingAcc = _buf->headingAcc;
        pDOP = _buf->pDOP;
        for (int _i=0; _i<6; _i++) reserved[_i] = _buf->reserved[_i];
        headVeh = _buf->headVeh;
        magDec = _buf->magDec;
        magAcc = _buf->magAcc;
        return true;
    }
};

// Message: airdata (id: 27)
struct airdata_t {
    // public fields
    float baro_press_pa;
    float baro_temp_C;
    float baro_hum;
    float ext_diff_press_pa;
    float ext_static_press_pa;
    float ext_temp_C;
    uint16_t error_count;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        float baro_press_pa;
        float baro_temp_C;
        float baro_hum;
        float ext_diff_press_pa;
        float ext_static_press_pa;
        float ext_temp_C;
        uint16_t error_count;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 27;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->baro_press_pa = baro_press_pa;
        _buf->baro_temp_C = baro_temp_C;
        _buf->baro_hum = baro_hum;
        _buf->ext_diff_press_pa = ext_diff_press_pa;
        _buf->ext_static_press_pa = ext_static_press_pa;
        _buf->ext_temp_C = ext_temp_C;
        _buf->error_count = error_count;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        baro_press_pa = _buf->baro_press_pa;
        baro_temp_C = _buf->baro_temp_C;
        baro_hum = _buf->baro_hum;
        ext_diff_press_pa = _buf->ext_diff_press_pa;
        ext_static_press_pa = _buf->ext_static_press_pa;
        ext_temp_C = _buf->ext_temp_C;
        error_count = _buf->error_count;
        return true;
    }
};

// Message: power (id: 28)
struct power_t {
    // public fields
    float int_main_v;
    float avionics_v;
    float ext_main_v;
    float ext_main_amp;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t int_main_v;
        uint16_t avionics_v;
        uint16_t ext_main_v;
        uint16_t ext_main_amp;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 28;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->int_main_v = uintround(int_main_v * 100);
        _buf->avionics_v = uintround(avionics_v * 100);
        _buf->ext_main_v = uintround(ext_main_v * 100);
        _buf->ext_main_amp = uintround(ext_main_amp * 100);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        int_main_v = _buf->int_main_v / (float)100;
        avionics_v = _buf->avionics_v / (float)100;
        ext_main_v = _buf->ext_main_v / (float)100;
        ext_main_amp = _buf->ext_main_amp / (float)100;
        return true;
    }
};

// Message: status (id: 29)
struct status_t {
    // public fields
    uint16_t serial_number;
    uint16_t firmware_rev;
    uint16_t master_hz;
    uint32_t baud;
    uint16_t byte_rate;
    uint16_t timer_misses;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t serial_number;
        uint16_t firmware_rev;
        uint16_t master_hz;
        uint32_t baud;
        uint16_t byte_rate;
        uint16_t timer_misses;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 29;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->serial_number = serial_number;
        _buf->firmware_rev = firmware_rev;
        _buf->master_hz = master_hz;
        _buf->baud = baud;
        _buf->byte_rate = byte_rate;
        _buf->timer_misses = timer_misses;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        serial_number = _buf->serial_number;
        firmware_rev = _buf->firmware_rev;
        master_hz = _buf->master_hz;
        baud = _buf->baud;
        byte_rate = _buf->byte_rate;
        timer_misses = _buf->timer_misses;
        return true;
    }
};

// Message: ekf (id: 30)
struct ekf_t {
    // public fields
    uint32_t millis;
    double lat_rad;
    double lon_rad;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float phi_rad;
    float the_rad;
    float psi_rad;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    float max_pos_cov;
    float max_vel_cov;
    float max_att_cov;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        double lat_rad;
        double lon_rad;
        float altitude_m;
        float vn_ms;
        float ve_ms;
        float vd_ms;
        float phi_rad;
        float the_rad;
        float psi_rad;
        float p_bias;
        float q_bias;
        float r_bias;
        float ax_bias;
        float ay_bias;
        float az_bias;
        uint16_t max_pos_cov;
        uint16_t max_vel_cov;
        uint16_t max_att_cov;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 30;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->lat_rad = lat_rad;
        _buf->lon_rad = lon_rad;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = vn_ms;
        _buf->ve_ms = ve_ms;
        _buf->vd_ms = vd_ms;
        _buf->phi_rad = phi_rad;
        _buf->the_rad = the_rad;
        _buf->psi_rad = psi_rad;
        _buf->p_bias = p_bias;
        _buf->q_bias = q_bias;
        _buf->r_bias = r_bias;
        _buf->ax_bias = ax_bias;
        _buf->ay_bias = ay_bias;
        _buf->az_bias = az_bias;
        _buf->max_pos_cov = uintround(max_pos_cov * 100);
        _buf->max_vel_cov = uintround(max_vel_cov * 1000);
        _buf->max_att_cov = uintround(max_att_cov * 10000);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        lat_rad = _buf->lat_rad;
        lon_rad = _buf->lon_rad;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms;
        ve_ms = _buf->ve_ms;
        vd_ms = _buf->vd_ms;
        phi_rad = _buf->phi_rad;
        the_rad = _buf->the_rad;
        psi_rad = _buf->psi_rad;
        p_bias = _buf->p_bias;
        q_bias = _buf->q_bias;
        r_bias = _buf->r_bias;
        ax_bias = _buf->ax_bias;
        ay_bias = _buf->ay_bias;
        az_bias = _buf->az_bias;
        max_pos_cov = _buf->max_pos_cov / (float)100;
        max_vel_cov = _buf->max_vel_cov / (float)1000;
        max_att_cov = _buf->max_att_cov / (float)10000;
        status = _buf->status;
        return true;
    }
};

} // namespace message
