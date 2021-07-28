#pragma once

// Ardupilot realloc() support
#if defined(ARDUPILOT_BUILD)
  #include <AP_HAL/AP_HAL.h>
  extern const AP_HAL::HAL& hal;
  #define REALLOC(X, Y) hal.util->std_realloc( (X), (Y) )
#else
  #define REALLOC(X, Y) std::realloc( (X), (Y) )
#endif

#include <stdint.h>  // uint8_t, et. al.
#include <stdlib.h>  // malloc() / free()
#include <string.h>  // memcpy()

#include <string>
using std::string;

#include "props2.h"  // github.com/RiceCreekUAS/props/v2

namespace rcfmu_message {

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t command_ack_id = 10;
const uint8_t config_json_id = 11;
const uint8_t command_inceptors_id = 12;
const uint8_t command_zero_gyros_id = 13;
const uint8_t command_reset_ekf_id = 14;
const uint8_t command_cycle_inceptors_id = 15;
const uint8_t pilot_id = 16;
const uint8_t imu_id = 17;
const uint8_t gps_id = 18;
const uint8_t airdata_id = 19;
const uint8_t power_id = 20;
const uint8_t status_id = 21;
const uint8_t ekf_id = 22;

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
class command_ack_t {
public:

    uint8_t command_id;
    uint8_t subcommand_id;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t command_id;
        uint8_t subcommand_id;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 10;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_ack_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->command_id = command_id;
        _buf->subcommand_id = subcommand_id;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        command_id = _buf->command_id;
        subcommand_id = _buf->subcommand_id;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setUInt("command_id", command_id);
        node.setUInt("subcommand_id", subcommand_id);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        command_id = node.getUInt("command_id");
        subcommand_id = node.getUInt("subcommand_id");
    }
};

// Message: config_json (id: 11)
class config_json_t {
public:

    string path;
    string json;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t path_len;
        uint16_t json_len;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 11;
    uint8_t *payload = nullptr;
    int len = 0;

    ~config_json_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += path.length();
        size += json.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->path_len = path.length();
        _buf->json_len = json.length();
        memcpy(&(payload[len]), path.c_str(), path.length());
        len += path.length();
        memcpy(&(payload[len]), json.c_str(), json.length());
        len += json.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        path = string((char *)&(external_message[len]), _buf->path_len);
        len += _buf->path_len;
        json = string((char *)&(external_message[len]), _buf->json_len);
        len += _buf->json_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setString("path", path);
        node.setString("json", json);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        path = node.getString("path");
        json = node.getString("json");
    }
};

// Message: command_inceptors (id: 12)
class command_inceptors_t {
public:

    float channel[ap_channels];

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t channel[ap_channels];
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 12;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_inceptors_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        for (int _i=0; _i<ap_channels; _i++) _buf->channel[_i] = intround(channel[_i] * 16384);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        for (int _i=0; _i<ap_channels; _i++) channel[_i] = _buf->channel[_i] / (float)16384;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        for (int _i=0; _i<ap_channels; _i++) node.setDouble("channel", _i, channel[_i]);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        for (int _i=0; _i<ap_channels; _i++) channel[_i] = node.getDouble("channel", _i);
    }
};

// Message: command_zero_gyros (id: 13)
class command_zero_gyros_t {
public:


    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 13;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_zero_gyros_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        len = sizeof(_compact_t);
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
    }
};

// Message: command_reset_ekf (id: 14)
class command_reset_ekf_t {
public:


    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 14;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_reset_ekf_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        len = sizeof(_compact_t);
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
    }
};

// Message: command_cycle_inceptors (id: 15)
class command_cycle_inceptors_t {
public:


    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 15;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_cycle_inceptors_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        len = sizeof(_compact_t);
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
    }
};

// Message: pilot (id: 16)
class pilot_t {
public:

    float channel[sbus_channels];
    uint8_t flags;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t channel[sbus_channels];
        uint8_t flags;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 16;
    uint8_t *payload = nullptr;
    int len = 0;

    ~pilot_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        for (int _i=0; _i<sbus_channels; _i++) _buf->channel[_i] = intround(channel[_i] * 16384);
        _buf->flags = flags;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        for (int _i=0; _i<sbus_channels; _i++) channel[_i] = _buf->channel[_i] / (float)16384;
        flags = _buf->flags;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        for (int _i=0; _i<sbus_channels; _i++) node.setDouble("channel", _i, channel[_i]);
        node.setUInt("flags", flags);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        for (int _i=0; _i<sbus_channels; _i++) channel[_i] = node.getDouble("channel", _i);
        flags = node.getUInt("flags");
    }
};

// Message: imu (id: 17)
class imu_t {
public:

    uint32_t millis;
    float ax_raw;
    float ay_raw;
    float az_raw;
    float hx_raw;
    float hy_raw;
    float hz_raw;
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float p_rps;
    float q_rps;
    float r_rps;
    float hx;
    float hy;
    float hz;
    float temp_C;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t ax_raw;
        int16_t ay_raw;
        int16_t az_raw;
        int16_t hx_raw;
        int16_t hy_raw;
        int16_t hz_raw;
        int16_t ax_mps2;
        int16_t ay_mps2;
        int16_t az_mps2;
        int16_t p_rps;
        int16_t q_rps;
        int16_t r_rps;
        int16_t hx;
        int16_t hy;
        int16_t hz;
        int16_t temp_C;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 17;
    uint8_t *payload = nullptr;
    int len = 0;

    ~imu_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->ax_raw = intround(ax_raw * 835.296217);
        _buf->ay_raw = intround(ay_raw * 835.296217);
        _buf->az_raw = intround(az_raw * 835.296217);
        _buf->hx_raw = intround(hx_raw * 30000);
        _buf->hy_raw = intround(hy_raw * 30000);
        _buf->hz_raw = intround(hz_raw * 30000);
        _buf->ax_mps2 = intround(ax_mps2 * 835.296217);
        _buf->ay_mps2 = intround(ay_mps2 * 835.296217);
        _buf->az_mps2 = intround(az_mps2 * 835.296217);
        _buf->p_rps = intround(p_rps * 3754.82165);
        _buf->q_rps = intround(q_rps * 3754.82165);
        _buf->r_rps = intround(r_rps * 3754.82165);
        _buf->hx = intround(hx * 30000);
        _buf->hy = intround(hy * 30000);
        _buf->hz = intround(hz * 30000);
        _buf->temp_C = intround(temp_C * 250);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        ax_raw = _buf->ax_raw / (float)835.296217;
        ay_raw = _buf->ay_raw / (float)835.296217;
        az_raw = _buf->az_raw / (float)835.296217;
        hx_raw = _buf->hx_raw / (float)30000;
        hy_raw = _buf->hy_raw / (float)30000;
        hz_raw = _buf->hz_raw / (float)30000;
        ax_mps2 = _buf->ax_mps2 / (float)835.296217;
        ay_mps2 = _buf->ay_mps2 / (float)835.296217;
        az_mps2 = _buf->az_mps2 / (float)835.296217;
        p_rps = _buf->p_rps / (float)3754.82165;
        q_rps = _buf->q_rps / (float)3754.82165;
        r_rps = _buf->r_rps / (float)3754.82165;
        hx = _buf->hx / (float)30000;
        hy = _buf->hy / (float)30000;
        hz = _buf->hz / (float)30000;
        temp_C = _buf->temp_C / (float)250;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setUInt("millis", millis);
        node.setDouble("ax_raw", ax_raw);
        node.setDouble("ay_raw", ay_raw);
        node.setDouble("az_raw", az_raw);
        node.setDouble("hx_raw", hx_raw);
        node.setDouble("hy_raw", hy_raw);
        node.setDouble("hz_raw", hz_raw);
        node.setDouble("ax_mps2", ax_mps2);
        node.setDouble("ay_mps2", ay_mps2);
        node.setDouble("az_mps2", az_mps2);
        node.setDouble("p_rps", p_rps);
        node.setDouble("q_rps", q_rps);
        node.setDouble("r_rps", r_rps);
        node.setDouble("hx", hx);
        node.setDouble("hy", hy);
        node.setDouble("hz", hz);
        node.setDouble("temp_C", temp_C);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        millis = node.getUInt("millis");
        ax_raw = node.getDouble("ax_raw");
        ay_raw = node.getDouble("ay_raw");
        az_raw = node.getDouble("az_raw");
        hx_raw = node.getDouble("hx_raw");
        hy_raw = node.getDouble("hy_raw");
        hz_raw = node.getDouble("hz_raw");
        ax_mps2 = node.getDouble("ax_mps2");
        ay_mps2 = node.getDouble("ay_mps2");
        az_mps2 = node.getDouble("az_mps2");
        p_rps = node.getDouble("p_rps");
        q_rps = node.getDouble("q_rps");
        r_rps = node.getDouble("r_rps");
        hx = node.getDouble("hx");
        hy = node.getDouble("hy");
        hz = node.getDouble("hz");
        temp_C = node.getDouble("temp_C");
    }
};

// Message: gps (id: 18)
class gps_t {
public:

    uint32_t millis;
    uint64_t unix_usec;
    uint8_t num_sats;
    uint8_t status;
    int32_t longitude_raw;
    int32_t latitude_raw;
    float altitude_m;
    float vn_mps;
    float ve_mps;
    float vd_mps;
    float hAcc;
    float vAcc;
    float hdop;
    float vdop;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint64_t unix_usec;
        uint8_t num_sats;
        uint8_t status;
        int32_t longitude_raw;
        int32_t latitude_raw;
        float altitude_m;
        float vn_mps;
        float ve_mps;
        float vd_mps;
        float hAcc;
        float vAcc;
        float hdop;
        float vdop;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 18;
    uint8_t *payload = nullptr;
    int len = 0;

    ~gps_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->unix_usec = unix_usec;
        _buf->num_sats = num_sats;
        _buf->status = status;
        _buf->longitude_raw = longitude_raw;
        _buf->latitude_raw = latitude_raw;
        _buf->altitude_m = altitude_m;
        _buf->vn_mps = vn_mps;
        _buf->ve_mps = ve_mps;
        _buf->vd_mps = vd_mps;
        _buf->hAcc = hAcc;
        _buf->vAcc = vAcc;
        _buf->hdop = hdop;
        _buf->vdop = vdop;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        unix_usec = _buf->unix_usec;
        num_sats = _buf->num_sats;
        status = _buf->status;
        longitude_raw = _buf->longitude_raw;
        latitude_raw = _buf->latitude_raw;
        altitude_m = _buf->altitude_m;
        vn_mps = _buf->vn_mps;
        ve_mps = _buf->ve_mps;
        vd_mps = _buf->vd_mps;
        hAcc = _buf->hAcc;
        vAcc = _buf->vAcc;
        hdop = _buf->hdop;
        vdop = _buf->vdop;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setUInt("millis", millis);
        node.setUInt64("unix_usec", unix_usec);
        node.setUInt("num_sats", num_sats);
        node.setUInt("status", status);
        node.setInt("longitude_raw", longitude_raw);
        node.setInt("latitude_raw", latitude_raw);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_mps", vn_mps);
        node.setDouble("ve_mps", ve_mps);
        node.setDouble("vd_mps", vd_mps);
        node.setDouble("hAcc", hAcc);
        node.setDouble("vAcc", vAcc);
        node.setDouble("hdop", hdop);
        node.setDouble("vdop", vdop);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        millis = node.getUInt("millis");
        unix_usec = node.getUInt64("unix_usec");
        num_sats = node.getUInt("num_sats");
        status = node.getUInt("status");
        longitude_raw = node.getInt("longitude_raw");
        latitude_raw = node.getInt("latitude_raw");
        altitude_m = node.getDouble("altitude_m");
        vn_mps = node.getDouble("vn_mps");
        ve_mps = node.getDouble("ve_mps");
        vd_mps = node.getDouble("vd_mps");
        hAcc = node.getDouble("hAcc");
        vAcc = node.getDouble("vAcc");
        hdop = node.getDouble("hdop");
        vdop = node.getDouble("vdop");
    }
};

// Message: airdata (id: 19)
class airdata_t {
public:

    float baro_press_pa;
    float baro_temp_C;
    float baro_hum;
    float ext_diff_press_pa;
    float ext_static_press_pa;
    float ext_temp_C;
    uint16_t error_count;

    // internal structure for packing
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

    // id, ptr to payload and len
    static const uint8_t id = 19;
    uint8_t *payload = nullptr;
    int len = 0;

    ~airdata_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
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
        _compact_t *_buf = (_compact_t *)external_message;
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

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setDouble("baro_press_pa", baro_press_pa);
        node.setDouble("baro_temp_C", baro_temp_C);
        node.setDouble("baro_hum", baro_hum);
        node.setDouble("ext_diff_press_pa", ext_diff_press_pa);
        node.setDouble("ext_static_press_pa", ext_static_press_pa);
        node.setDouble("ext_temp_C", ext_temp_C);
        node.setUInt("error_count", error_count);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        baro_press_pa = node.getDouble("baro_press_pa");
        baro_temp_C = node.getDouble("baro_temp_C");
        baro_hum = node.getDouble("baro_hum");
        ext_diff_press_pa = node.getDouble("ext_diff_press_pa");
        ext_static_press_pa = node.getDouble("ext_static_press_pa");
        ext_temp_C = node.getDouble("ext_temp_C");
        error_count = node.getUInt("error_count");
    }
};

// Message: power (id: 20)
class power_t {
public:

    float int_main_v;
    float avionics_v;
    float ext_main_v;
    float ext_main_amp;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t int_main_v;
        uint16_t avionics_v;
        uint16_t ext_main_v;
        uint16_t ext_main_amp;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 20;
    uint8_t *payload = nullptr;
    int len = 0;

    ~power_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->int_main_v = uintround(int_main_v * 100);
        _buf->avionics_v = uintround(avionics_v * 100);
        _buf->ext_main_v = uintround(ext_main_v * 100);
        _buf->ext_main_amp = uintround(ext_main_amp * 100);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        int_main_v = _buf->int_main_v / (float)100;
        avionics_v = _buf->avionics_v / (float)100;
        ext_main_v = _buf->ext_main_v / (float)100;
        ext_main_amp = _buf->ext_main_amp / (float)100;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setDouble("int_main_v", int_main_v);
        node.setDouble("avionics_v", avionics_v);
        node.setDouble("ext_main_v", ext_main_v);
        node.setDouble("ext_main_amp", ext_main_amp);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        int_main_v = node.getDouble("int_main_v");
        avionics_v = node.getDouble("avionics_v");
        ext_main_v = node.getDouble("ext_main_v");
        ext_main_amp = node.getDouble("ext_main_amp");
    }
};

// Message: status (id: 21)
class status_t {
public:

    uint16_t serial_number;
    uint16_t firmware_rev;
    uint16_t master_hz;
    uint32_t baud;
    uint16_t byte_rate;
    uint16_t timer_misses;

    // internal structure for packing
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

    // id, ptr to payload and len
    static const uint8_t id = 21;
    uint8_t *payload = nullptr;
    int len = 0;

    ~status_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
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
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        serial_number = _buf->serial_number;
        firmware_rev = _buf->firmware_rev;
        master_hz = _buf->master_hz;
        baud = _buf->baud;
        byte_rate = _buf->byte_rate;
        timer_misses = _buf->timer_misses;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setUInt("serial_number", serial_number);
        node.setUInt("firmware_rev", firmware_rev);
        node.setUInt("master_hz", master_hz);
        node.setUInt("baud", baud);
        node.setUInt("byte_rate", byte_rate);
        node.setUInt("timer_misses", timer_misses);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        serial_number = node.getUInt("serial_number");
        firmware_rev = node.getUInt("firmware_rev");
        master_hz = node.getUInt("master_hz");
        baud = node.getUInt("baud");
        byte_rate = node.getUInt("byte_rate");
        timer_misses = node.getUInt("timer_misses");
    }
};

// Message: ekf (id: 22)
class ekf_t {
public:

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

    // id, ptr to payload and len
    static const uint8_t id = 22;
    uint8_t *payload = nullptr;
    int len = 0;

    ~ekf_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
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
        _compact_t *_buf = (_compact_t *)external_message;
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

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode node) {
        node.setUInt("millis", millis);
        node.setDouble("lat_rad", lat_rad);
        node.setDouble("lon_rad", lon_rad);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_ms", vn_ms);
        node.setDouble("ve_ms", ve_ms);
        node.setDouble("vd_ms", vd_ms);
        node.setDouble("phi_rad", phi_rad);
        node.setDouble("the_rad", the_rad);
        node.setDouble("psi_rad", psi_rad);
        node.setDouble("p_bias", p_bias);
        node.setDouble("q_bias", q_bias);
        node.setDouble("r_bias", r_bias);
        node.setDouble("ax_bias", ax_bias);
        node.setDouble("ay_bias", ay_bias);
        node.setDouble("az_bias", az_bias);
        node.setDouble("max_pos_cov", max_pos_cov);
        node.setDouble("max_vel_cov", max_vel_cov);
        node.setDouble("max_att_cov", max_att_cov);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode node) {
        millis = node.getUInt("millis");
        lat_rad = node.getDouble("lat_rad");
        lon_rad = node.getDouble("lon_rad");
        altitude_m = node.getDouble("altitude_m");
        vn_ms = node.getDouble("vn_ms");
        ve_ms = node.getDouble("ve_ms");
        vd_ms = node.getDouble("vd_ms");
        phi_rad = node.getDouble("phi_rad");
        the_rad = node.getDouble("the_rad");
        psi_rad = node.getDouble("psi_rad");
        p_bias = node.getDouble("p_bias");
        q_bias = node.getDouble("q_bias");
        r_bias = node.getDouble("r_bias");
        ax_bias = node.getDouble("ax_bias");
        ay_bias = node.getDouble("ay_bias");
        az_bias = node.getDouble("az_bias");
        max_pos_cov = node.getDouble("max_pos_cov");
        max_vel_cov = node.getDouble("max_vel_cov");
        max_att_cov = node.getDouble("max_att_cov");
        status = node.getUInt("status");
    }
};

} // namespace rcfmu_message
