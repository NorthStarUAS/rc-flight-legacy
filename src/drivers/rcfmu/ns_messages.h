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

namespace ns_message {

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t gps_v3_id = 26;
const uint8_t gps_v4_id = 34;
const uint8_t gps_v5_id = 49;
const uint8_t imu_v4_id = 35;
const uint8_t imu_v5_id = 45;
const uint8_t imu_v6_id = 50;
const uint8_t airdata_v6_id = 40;
const uint8_t airdata_v7_id = 43;
const uint8_t airdata_v8_id = 54;
const uint8_t filter_v4_id = 36;
const uint8_t filter_v5_id = 47;
const uint8_t nav_v6_id = 52;
const uint8_t nav_metrics_v6_id = 53;
const uint8_t actuator_v2_id = 21;
const uint8_t actuator_v3_id = 37;
const uint8_t effectors_v1_id = 61;
const uint8_t pilot_v3_id = 38;
const uint8_t pilot_v4_id = 51;
const uint8_t inceptors_v1_id = 62;
const uint8_t power_v1_id = 55;
const uint8_t ap_status_v6_id = 33;
const uint8_t ap_status_v7_id = 39;
const uint8_t ap_targets_v1_id = 59;
const uint8_t mission_v1_id = 60;
const uint8_t system_health_v5_id = 41;
const uint8_t system_health_v6_id = 46;
const uint8_t status_v7_id = 56;
const uint8_t event_v1_id = 27;
const uint8_t event_v2_id = 44;
const uint8_t command_v1_id = 28;
const uint8_t ack_v1_id = 57;

// Constants
static const uint8_t sbus_channels = 16;  // number of sbus channels
static const uint8_t ap_channels = 6;  // number of sbus channels

// Message: gps_v3 (id: 26)
class gps_v3_t {
public:

    uint8_t index;
    double timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    double unixtime_sec;
    uint8_t satellites;
    float horiz_accuracy_m;
    float vert_accuracy_m;
    float pdop;
    uint8_t fix_type;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        double unixtime_sec;
        uint8_t satellites;
        uint16_t horiz_accuracy_m;
        uint16_t vert_accuracy_m;
        uint16_t pdop;
        uint8_t fix_type;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 26;
    uint8_t *payload = nullptr;
    int len = 0;

    ~gps_v3_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100.0);
        _buf->ve_ms = intround(ve_ms * 100.0);
        _buf->vd_ms = intround(vd_ms * 100.0);
        _buf->unixtime_sec = unixtime_sec;
        _buf->satellites = satellites;
        _buf->horiz_accuracy_m = uintround(horiz_accuracy_m * 100.0);
        _buf->vert_accuracy_m = uintround(vert_accuracy_m * 100.0);
        _buf->pdop = uintround(pdop * 100.0);
        _buf->fix_type = fix_type;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100.0;
        ve_ms = _buf->ve_ms / (float)100.0;
        vd_ms = _buf->vd_ms / (float)100.0;
        unixtime_sec = _buf->unixtime_sec;
        satellites = _buf->satellites;
        horiz_accuracy_m = _buf->horiz_accuracy_m / (float)100.0;
        vert_accuracy_m = _buf->vert_accuracy_m / (float)100.0;
        pdop = _buf->pdop / (float)100.0;
        fix_type = _buf->fix_type;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("latitude_deg", latitude_deg);
        node.setDouble("longitude_deg", longitude_deg);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_ms", vn_ms);
        node.setDouble("ve_ms", ve_ms);
        node.setDouble("vd_ms", vd_ms);
        node.setDouble("unixtime_sec", unixtime_sec);
        node.setUInt("satellites", satellites);
        node.setDouble("horiz_accuracy_m", horiz_accuracy_m);
        node.setDouble("vert_accuracy_m", vert_accuracy_m);
        node.setDouble("pdop", pdop);
        node.setUInt("fix_type", fix_type);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        latitude_deg = node.getDouble("latitude_deg");
        longitude_deg = node.getDouble("longitude_deg");
        altitude_m = node.getDouble("altitude_m");
        vn_ms = node.getDouble("vn_ms");
        ve_ms = node.getDouble("ve_ms");
        vd_ms = node.getDouble("vd_ms");
        unixtime_sec = node.getDouble("unixtime_sec");
        satellites = node.getUInt("satellites");
        horiz_accuracy_m = node.getDouble("horiz_accuracy_m");
        vert_accuracy_m = node.getDouble("vert_accuracy_m");
        pdop = node.getDouble("pdop");
        fix_type = node.getUInt("fix_type");
    }
};

// Message: gps_v4 (id: 34)
class gps_v4_t {
public:

    uint8_t index;
    float timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    double unixtime_sec;
    uint8_t satellites;
    float horiz_accuracy_m;
    float vert_accuracy_m;
    float pdop;
    uint8_t fix_type;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        double unixtime_sec;
        uint8_t satellites;
        uint16_t horiz_accuracy_m;
        uint16_t vert_accuracy_m;
        uint16_t pdop;
        uint8_t fix_type;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 34;
    uint8_t *payload = nullptr;
    int len = 0;

    ~gps_v4_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100.0);
        _buf->ve_ms = intround(ve_ms * 100.0);
        _buf->vd_ms = intround(vd_ms * 100.0);
        _buf->unixtime_sec = unixtime_sec;
        _buf->satellites = satellites;
        _buf->horiz_accuracy_m = uintround(horiz_accuracy_m * 100.0);
        _buf->vert_accuracy_m = uintround(vert_accuracy_m * 100.0);
        _buf->pdop = uintround(pdop * 100.0);
        _buf->fix_type = fix_type;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100.0;
        ve_ms = _buf->ve_ms / (float)100.0;
        vd_ms = _buf->vd_ms / (float)100.0;
        unixtime_sec = _buf->unixtime_sec;
        satellites = _buf->satellites;
        horiz_accuracy_m = _buf->horiz_accuracy_m / (float)100.0;
        vert_accuracy_m = _buf->vert_accuracy_m / (float)100.0;
        pdop = _buf->pdop / (float)100.0;
        fix_type = _buf->fix_type;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("latitude_deg", latitude_deg);
        node.setDouble("longitude_deg", longitude_deg);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_ms", vn_ms);
        node.setDouble("ve_ms", ve_ms);
        node.setDouble("vd_ms", vd_ms);
        node.setDouble("unixtime_sec", unixtime_sec);
        node.setUInt("satellites", satellites);
        node.setDouble("horiz_accuracy_m", horiz_accuracy_m);
        node.setDouble("vert_accuracy_m", vert_accuracy_m);
        node.setDouble("pdop", pdop);
        node.setUInt("fix_type", fix_type);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        latitude_deg = node.getDouble("latitude_deg");
        longitude_deg = node.getDouble("longitude_deg");
        altitude_m = node.getDouble("altitude_m");
        vn_ms = node.getDouble("vn_ms");
        ve_ms = node.getDouble("ve_ms");
        vd_ms = node.getDouble("vd_ms");
        unixtime_sec = node.getDouble("unixtime_sec");
        satellites = node.getUInt("satellites");
        horiz_accuracy_m = node.getDouble("horiz_accuracy_m");
        vert_accuracy_m = node.getDouble("vert_accuracy_m");
        pdop = node.getDouble("pdop");
        fix_type = node.getUInt("fix_type");
    }
};

// Message: gps_v5 (id: 49)
class gps_v5_t {
public:

    uint8_t index;
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
    float hAcc_m;
    float vAcc_m;
    float hdop;
    float vdop;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        uint64_t unix_usec;
        uint8_t num_sats;
        uint8_t status;
        int32_t longitude_raw;
        int32_t latitude_raw;
        float altitude_m;
        int16_t vn_mps;
        int16_t ve_mps;
        int16_t vd_mps;
        int16_t hAcc_m;
        int16_t vAcc_m;
        int16_t hdop;
        int16_t vdop;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 49;
    uint8_t *payload = nullptr;
    int len = 0;

    ~gps_v5_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->unix_usec = unix_usec;
        _buf->num_sats = num_sats;
        _buf->status = status;
        _buf->longitude_raw = longitude_raw;
        _buf->latitude_raw = latitude_raw;
        _buf->altitude_m = altitude_m;
        _buf->vn_mps = intround(vn_mps * 100.0);
        _buf->ve_mps = intround(ve_mps * 100.0);
        _buf->vd_mps = intround(vd_mps * 100.0);
        _buf->hAcc_m = intround(hAcc_m * 100.0);
        _buf->vAcc_m = intround(vAcc_m * 100.0);
        _buf->hdop = intround(hdop * 100.0);
        _buf->vdop = intround(vdop * 100.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        unix_usec = _buf->unix_usec;
        num_sats = _buf->num_sats;
        status = _buf->status;
        longitude_raw = _buf->longitude_raw;
        latitude_raw = _buf->latitude_raw;
        altitude_m = _buf->altitude_m;
        vn_mps = _buf->vn_mps / (float)100.0;
        ve_mps = _buf->ve_mps / (float)100.0;
        vd_mps = _buf->vd_mps / (float)100.0;
        hAcc_m = _buf->hAcc_m / (float)100.0;
        vAcc_m = _buf->vAcc_m / (float)100.0;
        hdop = _buf->hdop / (float)100.0;
        vdop = _buf->vdop / (float)100.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
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
        node.setDouble("hAcc_m", hAcc_m);
        node.setDouble("vAcc_m", vAcc_m);
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

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
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
        hAcc_m = node.getDouble("hAcc_m");
        vAcc_m = node.getDouble("vAcc_m");
        hdop = node.getDouble("hdop");
        vdop = node.getDouble("vdop");
    }
};

// Message: imu_v4 (id: 35)
class imu_v4_t {
public:

    uint8_t index;
    float timestamp_sec;
    float p_rad_sec;
    float q_rad_sec;
    float r_rad_sec;
    float ax_mps_sec;
    float ay_mps_sec;
    float az_mps_sec;
    float hx;
    float hy;
    float hz;
    float temp_C;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        float p_rad_sec;
        float q_rad_sec;
        float r_rad_sec;
        float ax_mps_sec;
        float ay_mps_sec;
        float az_mps_sec;
        float hx;
        float hy;
        float hz;
        int16_t temp_C;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 35;
    uint8_t *payload = nullptr;
    int len = 0;

    ~imu_v4_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->p_rad_sec = p_rad_sec;
        _buf->q_rad_sec = q_rad_sec;
        _buf->r_rad_sec = r_rad_sec;
        _buf->ax_mps_sec = ax_mps_sec;
        _buf->ay_mps_sec = ay_mps_sec;
        _buf->az_mps_sec = az_mps_sec;
        _buf->hx = hx;
        _buf->hy = hy;
        _buf->hz = hz;
        _buf->temp_C = intround(temp_C * 10.0);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        p_rad_sec = _buf->p_rad_sec;
        q_rad_sec = _buf->q_rad_sec;
        r_rad_sec = _buf->r_rad_sec;
        ax_mps_sec = _buf->ax_mps_sec;
        ay_mps_sec = _buf->ay_mps_sec;
        az_mps_sec = _buf->az_mps_sec;
        hx = _buf->hx;
        hy = _buf->hy;
        hz = _buf->hz;
        temp_C = _buf->temp_C / (float)10.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("p_rad_sec", p_rad_sec);
        node.setDouble("q_rad_sec", q_rad_sec);
        node.setDouble("r_rad_sec", r_rad_sec);
        node.setDouble("ax_mps_sec", ax_mps_sec);
        node.setDouble("ay_mps_sec", ay_mps_sec);
        node.setDouble("az_mps_sec", az_mps_sec);
        node.setDouble("hx", hx);
        node.setDouble("hy", hy);
        node.setDouble("hz", hz);
        node.setDouble("temp_C", temp_C);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        p_rad_sec = node.getDouble("p_rad_sec");
        q_rad_sec = node.getDouble("q_rad_sec");
        r_rad_sec = node.getDouble("r_rad_sec");
        ax_mps_sec = node.getDouble("ax_mps_sec");
        ay_mps_sec = node.getDouble("ay_mps_sec");
        az_mps_sec = node.getDouble("az_mps_sec");
        hx = node.getDouble("hx");
        hy = node.getDouble("hy");
        hz = node.getDouble("hz");
        temp_C = node.getDouble("temp_C");
        status = node.getUInt("status");
    }
};

// Message: imu_v5 (id: 45)
class imu_v5_t {
public:

    uint8_t index;
    float timestamp_sec;
    float p_rad_sec;
    float q_rad_sec;
    float r_rad_sec;
    float ax_mps_sec;
    float ay_mps_sec;
    float az_mps_sec;
    float hx;
    float hy;
    float hz;
    float ax_raw;
    float ay_raw;
    float az_raw;
    float hx_raw;
    float hy_raw;
    float hz_raw;
    float temp_C;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        float p_rad_sec;
        float q_rad_sec;
        float r_rad_sec;
        float ax_mps_sec;
        float ay_mps_sec;
        float az_mps_sec;
        float hx;
        float hy;
        float hz;
        float ax_raw;
        float ay_raw;
        float az_raw;
        float hx_raw;
        float hy_raw;
        float hz_raw;
        int16_t temp_C;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 45;
    uint8_t *payload = nullptr;
    int len = 0;

    ~imu_v5_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->p_rad_sec = p_rad_sec;
        _buf->q_rad_sec = q_rad_sec;
        _buf->r_rad_sec = r_rad_sec;
        _buf->ax_mps_sec = ax_mps_sec;
        _buf->ay_mps_sec = ay_mps_sec;
        _buf->az_mps_sec = az_mps_sec;
        _buf->hx = hx;
        _buf->hy = hy;
        _buf->hz = hz;
        _buf->ax_raw = ax_raw;
        _buf->ay_raw = ay_raw;
        _buf->az_raw = az_raw;
        _buf->hx_raw = hx_raw;
        _buf->hy_raw = hy_raw;
        _buf->hz_raw = hz_raw;
        _buf->temp_C = intround(temp_C * 10.0);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        p_rad_sec = _buf->p_rad_sec;
        q_rad_sec = _buf->q_rad_sec;
        r_rad_sec = _buf->r_rad_sec;
        ax_mps_sec = _buf->ax_mps_sec;
        ay_mps_sec = _buf->ay_mps_sec;
        az_mps_sec = _buf->az_mps_sec;
        hx = _buf->hx;
        hy = _buf->hy;
        hz = _buf->hz;
        ax_raw = _buf->ax_raw;
        ay_raw = _buf->ay_raw;
        az_raw = _buf->az_raw;
        hx_raw = _buf->hx_raw;
        hy_raw = _buf->hy_raw;
        hz_raw = _buf->hz_raw;
        temp_C = _buf->temp_C / (float)10.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("p_rad_sec", p_rad_sec);
        node.setDouble("q_rad_sec", q_rad_sec);
        node.setDouble("r_rad_sec", r_rad_sec);
        node.setDouble("ax_mps_sec", ax_mps_sec);
        node.setDouble("ay_mps_sec", ay_mps_sec);
        node.setDouble("az_mps_sec", az_mps_sec);
        node.setDouble("hx", hx);
        node.setDouble("hy", hy);
        node.setDouble("hz", hz);
        node.setDouble("ax_raw", ax_raw);
        node.setDouble("ay_raw", ay_raw);
        node.setDouble("az_raw", az_raw);
        node.setDouble("hx_raw", hx_raw);
        node.setDouble("hy_raw", hy_raw);
        node.setDouble("hz_raw", hz_raw);
        node.setDouble("temp_C", temp_C);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        p_rad_sec = node.getDouble("p_rad_sec");
        q_rad_sec = node.getDouble("q_rad_sec");
        r_rad_sec = node.getDouble("r_rad_sec");
        ax_mps_sec = node.getDouble("ax_mps_sec");
        ay_mps_sec = node.getDouble("ay_mps_sec");
        az_mps_sec = node.getDouble("az_mps_sec");
        hx = node.getDouble("hx");
        hy = node.getDouble("hy");
        hz = node.getDouble("hz");
        ax_raw = node.getDouble("ax_raw");
        ay_raw = node.getDouble("ay_raw");
        az_raw = node.getDouble("az_raw");
        hx_raw = node.getDouble("hx_raw");
        hy_raw = node.getDouble("hy_raw");
        hz_raw = node.getDouble("hz_raw");
        temp_C = node.getDouble("temp_C");
        status = node.getUInt("status");
    }
};

// Message: imu_v6 (id: 50)
class imu_v6_t {
public:

    uint8_t index;
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
        uint8_t index;
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
    static const uint8_t id = 50;
    uint8_t *payload = nullptr;
    int len = 0;

    ~imu_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->ax_raw = intround(ax_raw * 835.296217);
        _buf->ay_raw = intround(ay_raw * 835.296217);
        _buf->az_raw = intround(az_raw * 835.296217);
        _buf->hx_raw = intround(hx_raw * 30000.0);
        _buf->hy_raw = intround(hy_raw * 30000.0);
        _buf->hz_raw = intround(hz_raw * 30000.0);
        _buf->ax_mps2 = intround(ax_mps2 * 835.296217);
        _buf->ay_mps2 = intround(ay_mps2 * 835.296217);
        _buf->az_mps2 = intround(az_mps2 * 835.296217);
        _buf->p_rps = intround(p_rps * 3754.82165);
        _buf->q_rps = intround(q_rps * 3754.82165);
        _buf->r_rps = intround(r_rps * 3754.82165);
        _buf->hx = intround(hx * 30000.0);
        _buf->hy = intround(hy * 30000.0);
        _buf->hz = intround(hz * 30000.0);
        _buf->temp_C = intround(temp_C * 250.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        ax_raw = _buf->ax_raw / (float)835.296217;
        ay_raw = _buf->ay_raw / (float)835.296217;
        az_raw = _buf->az_raw / (float)835.296217;
        hx_raw = _buf->hx_raw / (float)30000.0;
        hy_raw = _buf->hy_raw / (float)30000.0;
        hz_raw = _buf->hz_raw / (float)30000.0;
        ax_mps2 = _buf->ax_mps2 / (float)835.296217;
        ay_mps2 = _buf->ay_mps2 / (float)835.296217;
        az_mps2 = _buf->az_mps2 / (float)835.296217;
        p_rps = _buf->p_rps / (float)3754.82165;
        q_rps = _buf->q_rps / (float)3754.82165;
        r_rps = _buf->r_rps / (float)3754.82165;
        hx = _buf->hx / (float)30000.0;
        hy = _buf->hy / (float)30000.0;
        hz = _buf->hz / (float)30000.0;
        temp_C = _buf->temp_C / (float)250.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
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

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
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

// Message: airdata_v6 (id: 40)
class airdata_v6_t {
public:

    uint8_t index;
    float timestamp_sec;
    float pressure_mbar;
    float temp_C;
    float airspeed_smoothed_kt;
    float altitude_smoothed_m;
    float altitude_true_m;
    float pressure_vertical_speed_fps;
    float wind_dir_deg;
    float wind_speed_kt;
    float pitot_scale_factor;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t pressure_mbar;
        int16_t temp_C;
        int16_t airspeed_smoothed_kt;
        float altitude_smoothed_m;
        float altitude_true_m;
        int16_t pressure_vertical_speed_fps;
        uint16_t wind_dir_deg;
        uint8_t wind_speed_kt;
        uint8_t pitot_scale_factor;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 40;
    uint8_t *payload = nullptr;
    int len = 0;

    ~airdata_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->pressure_mbar = uintround(pressure_mbar * 10.0);
        _buf->temp_C = intround(temp_C * 100.0);
        _buf->airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100.0);
        _buf->altitude_smoothed_m = altitude_smoothed_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600.0);
        _buf->wind_dir_deg = uintround(wind_dir_deg * 100.0);
        _buf->wind_speed_kt = uintround(wind_speed_kt * 4.0);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100.0);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        pressure_mbar = _buf->pressure_mbar / (float)10.0;
        temp_C = _buf->temp_C / (float)100.0;
        airspeed_smoothed_kt = _buf->airspeed_smoothed_kt / (float)100.0;
        altitude_smoothed_m = _buf->altitude_smoothed_m;
        altitude_true_m = _buf->altitude_true_m;
        pressure_vertical_speed_fps = _buf->pressure_vertical_speed_fps / (float)600.0;
        wind_dir_deg = _buf->wind_dir_deg / (float)100.0;
        wind_speed_kt = _buf->wind_speed_kt / (float)4.0;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("pressure_mbar", pressure_mbar);
        node.setDouble("temp_C", temp_C);
        node.setDouble("airspeed_smoothed_kt", airspeed_smoothed_kt);
        node.setDouble("altitude_smoothed_m", altitude_smoothed_m);
        node.setDouble("altitude_true_m", altitude_true_m);
        node.setDouble("pressure_vertical_speed_fps", pressure_vertical_speed_fps);
        node.setDouble("wind_dir_deg", wind_dir_deg);
        node.setDouble("wind_speed_kt", wind_speed_kt);
        node.setDouble("pitot_scale_factor", pitot_scale_factor);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        pressure_mbar = node.getDouble("pressure_mbar");
        temp_C = node.getDouble("temp_C");
        airspeed_smoothed_kt = node.getDouble("airspeed_smoothed_kt");
        altitude_smoothed_m = node.getDouble("altitude_smoothed_m");
        altitude_true_m = node.getDouble("altitude_true_m");
        pressure_vertical_speed_fps = node.getDouble("pressure_vertical_speed_fps");
        wind_dir_deg = node.getDouble("wind_dir_deg");
        wind_speed_kt = node.getDouble("wind_speed_kt");
        pitot_scale_factor = node.getDouble("pitot_scale_factor");
        status = node.getUInt("status");
    }
};

// Message: airdata_v7 (id: 43)
class airdata_v7_t {
public:

    uint8_t index;
    float timestamp_sec;
    float pressure_mbar;
    float temp_C;
    float airspeed_smoothed_kt;
    float altitude_smoothed_m;
    float altitude_true_m;
    float pressure_vertical_speed_fps;
    float wind_dir_deg;
    float wind_speed_kt;
    float pitot_scale_factor;
    uint16_t error_count;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t pressure_mbar;
        int16_t temp_C;
        int16_t airspeed_smoothed_kt;
        float altitude_smoothed_m;
        float altitude_true_m;
        int16_t pressure_vertical_speed_fps;
        uint16_t wind_dir_deg;
        uint8_t wind_speed_kt;
        uint8_t pitot_scale_factor;
        uint16_t error_count;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 43;
    uint8_t *payload = nullptr;
    int len = 0;

    ~airdata_v7_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->pressure_mbar = uintround(pressure_mbar * 10.0);
        _buf->temp_C = intround(temp_C * 100.0);
        _buf->airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100.0);
        _buf->altitude_smoothed_m = altitude_smoothed_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600.0);
        _buf->wind_dir_deg = uintround(wind_dir_deg * 100.0);
        _buf->wind_speed_kt = uintround(wind_speed_kt * 4.0);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100.0);
        _buf->error_count = error_count;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        pressure_mbar = _buf->pressure_mbar / (float)10.0;
        temp_C = _buf->temp_C / (float)100.0;
        airspeed_smoothed_kt = _buf->airspeed_smoothed_kt / (float)100.0;
        altitude_smoothed_m = _buf->altitude_smoothed_m;
        altitude_true_m = _buf->altitude_true_m;
        pressure_vertical_speed_fps = _buf->pressure_vertical_speed_fps / (float)600.0;
        wind_dir_deg = _buf->wind_dir_deg / (float)100.0;
        wind_speed_kt = _buf->wind_speed_kt / (float)4.0;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100.0;
        error_count = _buf->error_count;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("pressure_mbar", pressure_mbar);
        node.setDouble("temp_C", temp_C);
        node.setDouble("airspeed_smoothed_kt", airspeed_smoothed_kt);
        node.setDouble("altitude_smoothed_m", altitude_smoothed_m);
        node.setDouble("altitude_true_m", altitude_true_m);
        node.setDouble("pressure_vertical_speed_fps", pressure_vertical_speed_fps);
        node.setDouble("wind_dir_deg", wind_dir_deg);
        node.setDouble("wind_speed_kt", wind_speed_kt);
        node.setDouble("pitot_scale_factor", pitot_scale_factor);
        node.setUInt("error_count", error_count);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        pressure_mbar = node.getDouble("pressure_mbar");
        temp_C = node.getDouble("temp_C");
        airspeed_smoothed_kt = node.getDouble("airspeed_smoothed_kt");
        altitude_smoothed_m = node.getDouble("altitude_smoothed_m");
        altitude_true_m = node.getDouble("altitude_true_m");
        pressure_vertical_speed_fps = node.getDouble("pressure_vertical_speed_fps");
        wind_dir_deg = node.getDouble("wind_dir_deg");
        wind_speed_kt = node.getDouble("wind_speed_kt");
        pitot_scale_factor = node.getDouble("pitot_scale_factor");
        error_count = node.getUInt("error_count");
        status = node.getUInt("status");
    }
};

// Message: airdata_v8 (id: 54)
class airdata_v8_t {
public:

    uint8_t index;
    uint32_t millis;
    float baro_press_pa;
    float diff_press_pa;
    float air_temp_C;
    float airspeed_mps;
    float altitude_agl_m;
    float altitude_true_m;
    float altitude_ground_m;
    uint8_t is_airborne;
    uint32_t flight_timer_millis;
    float wind_dir_deg;
    float wind_speed_mps;
    float pitot_scale_factor;
    uint16_t error_count;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        uint16_t baro_press_pa;
        uint16_t diff_press_pa;
        int16_t air_temp_C;
        int16_t airspeed_mps;
        float altitude_agl_m;
        float altitude_true_m;
        float altitude_ground_m;
        uint8_t is_airborne;
        uint32_t flight_timer_millis;
        uint16_t wind_dir_deg;
        uint8_t wind_speed_mps;
        uint8_t pitot_scale_factor;
        uint16_t error_count;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 54;
    uint8_t *payload = nullptr;
    int len = 0;

    ~airdata_v8_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->baro_press_pa = uintround(baro_press_pa * 0.5);
        _buf->diff_press_pa = uintround(diff_press_pa * 10.0);
        _buf->air_temp_C = intround(air_temp_C * 250.0);
        _buf->airspeed_mps = intround(airspeed_mps * 100.0);
        _buf->altitude_agl_m = altitude_agl_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->is_airborne = is_airborne;
        _buf->flight_timer_millis = flight_timer_millis;
        _buf->wind_dir_deg = uintround(wind_dir_deg * 100.0);
        _buf->wind_speed_mps = uintround(wind_speed_mps * 10.0);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100.0);
        _buf->error_count = error_count;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        baro_press_pa = _buf->baro_press_pa / (float)0.5;
        diff_press_pa = _buf->diff_press_pa / (float)10.0;
        air_temp_C = _buf->air_temp_C / (float)250.0;
        airspeed_mps = _buf->airspeed_mps / (float)100.0;
        altitude_agl_m = _buf->altitude_agl_m;
        altitude_true_m = _buf->altitude_true_m;
        altitude_ground_m = _buf->altitude_ground_m;
        is_airborne = _buf->is_airborne;
        flight_timer_millis = _buf->flight_timer_millis;
        wind_dir_deg = _buf->wind_dir_deg / (float)100.0;
        wind_speed_mps = _buf->wind_speed_mps / (float)10.0;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        node.setDouble("baro_press_pa", baro_press_pa);
        node.setDouble("diff_press_pa", diff_press_pa);
        node.setDouble("air_temp_C", air_temp_C);
        node.setDouble("airspeed_mps", airspeed_mps);
        node.setDouble("altitude_agl_m", altitude_agl_m);
        node.setDouble("altitude_true_m", altitude_true_m);
        node.setDouble("altitude_ground_m", altitude_ground_m);
        node.setUInt("is_airborne", is_airborne);
        node.setUInt("flight_timer_millis", flight_timer_millis);
        node.setDouble("wind_dir_deg", wind_dir_deg);
        node.setDouble("wind_speed_mps", wind_speed_mps);
        node.setDouble("pitot_scale_factor", pitot_scale_factor);
        node.setUInt("error_count", error_count);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        baro_press_pa = node.getDouble("baro_press_pa");
        diff_press_pa = node.getDouble("diff_press_pa");
        air_temp_C = node.getDouble("air_temp_C");
        airspeed_mps = node.getDouble("airspeed_mps");
        altitude_agl_m = node.getDouble("altitude_agl_m");
        altitude_true_m = node.getDouble("altitude_true_m");
        altitude_ground_m = node.getDouble("altitude_ground_m");
        is_airborne = node.getUInt("is_airborne");
        flight_timer_millis = node.getUInt("flight_timer_millis");
        wind_dir_deg = node.getDouble("wind_dir_deg");
        wind_speed_mps = node.getDouble("wind_speed_mps");
        pitot_scale_factor = node.getDouble("pitot_scale_factor");
        error_count = node.getUInt("error_count");
    }
};

// Message: filter_v4 (id: 36)
class filter_v4_t {
public:

    uint8_t index;
    float timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint8_t sequence_num;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 36;
    uint8_t *payload = nullptr;
    int len = 0;

    ~filter_v4_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100.0);
        _buf->ve_ms = intround(ve_ms * 100.0);
        _buf->vd_ms = intround(vd_ms * 100.0);
        _buf->roll_deg = intround(roll_deg * 10.0);
        _buf->pitch_deg = intround(pitch_deg * 10.0);
        _buf->yaw_deg = intround(yaw_deg * 10.0);
        _buf->p_bias = intround(p_bias * 10000.0);
        _buf->q_bias = intround(q_bias * 10000.0);
        _buf->r_bias = intround(r_bias * 10000.0);
        _buf->ax_bias = intround(ax_bias * 1000.0);
        _buf->ay_bias = intround(ay_bias * 1000.0);
        _buf->az_bias = intround(az_bias * 1000.0);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100.0;
        ve_ms = _buf->ve_ms / (float)100.0;
        vd_ms = _buf->vd_ms / (float)100.0;
        roll_deg = _buf->roll_deg / (float)10.0;
        pitch_deg = _buf->pitch_deg / (float)10.0;
        yaw_deg = _buf->yaw_deg / (float)10.0;
        p_bias = _buf->p_bias / (float)10000.0;
        q_bias = _buf->q_bias / (float)10000.0;
        r_bias = _buf->r_bias / (float)10000.0;
        ax_bias = _buf->ax_bias / (float)1000.0;
        ay_bias = _buf->ay_bias / (float)1000.0;
        az_bias = _buf->az_bias / (float)1000.0;
        sequence_num = _buf->sequence_num;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("latitude_deg", latitude_deg);
        node.setDouble("longitude_deg", longitude_deg);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_ms", vn_ms);
        node.setDouble("ve_ms", ve_ms);
        node.setDouble("vd_ms", vd_ms);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("pitch_deg", pitch_deg);
        node.setDouble("yaw_deg", yaw_deg);
        node.setDouble("p_bias", p_bias);
        node.setDouble("q_bias", q_bias);
        node.setDouble("r_bias", r_bias);
        node.setDouble("ax_bias", ax_bias);
        node.setDouble("ay_bias", ay_bias);
        node.setDouble("az_bias", az_bias);
        node.setUInt("sequence_num", sequence_num);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        latitude_deg = node.getDouble("latitude_deg");
        longitude_deg = node.getDouble("longitude_deg");
        altitude_m = node.getDouble("altitude_m");
        vn_ms = node.getDouble("vn_ms");
        ve_ms = node.getDouble("ve_ms");
        vd_ms = node.getDouble("vd_ms");
        roll_deg = node.getDouble("roll_deg");
        pitch_deg = node.getDouble("pitch_deg");
        yaw_deg = node.getDouble("yaw_deg");
        p_bias = node.getDouble("p_bias");
        q_bias = node.getDouble("q_bias");
        r_bias = node.getDouble("r_bias");
        ax_bias = node.getDouble("ax_bias");
        ay_bias = node.getDouble("ay_bias");
        az_bias = node.getDouble("az_bias");
        sequence_num = node.getUInt("sequence_num");
        status = node.getUInt("status");
    }
};

// Message: filter_v5 (id: 47)
class filter_v5_t {
public:

    uint8_t index;
    float timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    float max_pos_cov;
    float max_vel_cov;
    float max_att_cov;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint16_t max_pos_cov;
        uint16_t max_vel_cov;
        uint16_t max_att_cov;
        uint8_t sequence_num;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 47;
    uint8_t *payload = nullptr;
    int len = 0;

    ~filter_v5_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100.0);
        _buf->ve_ms = intround(ve_ms * 100.0);
        _buf->vd_ms = intround(vd_ms * 100.0);
        _buf->roll_deg = intround(roll_deg * 10.0);
        _buf->pitch_deg = intround(pitch_deg * 10.0);
        _buf->yaw_deg = intround(yaw_deg * 10.0);
        _buf->p_bias = intround(p_bias * 10000.0);
        _buf->q_bias = intround(q_bias * 10000.0);
        _buf->r_bias = intround(r_bias * 10000.0);
        _buf->ax_bias = intround(ax_bias * 1000.0);
        _buf->ay_bias = intround(ay_bias * 1000.0);
        _buf->az_bias = intround(az_bias * 1000.0);
        _buf->max_pos_cov = uintround(max_pos_cov * 100.0);
        _buf->max_vel_cov = uintround(max_vel_cov * 1000.0);
        _buf->max_att_cov = uintround(max_att_cov * 10000.0);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100.0;
        ve_ms = _buf->ve_ms / (float)100.0;
        vd_ms = _buf->vd_ms / (float)100.0;
        roll_deg = _buf->roll_deg / (float)10.0;
        pitch_deg = _buf->pitch_deg / (float)10.0;
        yaw_deg = _buf->yaw_deg / (float)10.0;
        p_bias = _buf->p_bias / (float)10000.0;
        q_bias = _buf->q_bias / (float)10000.0;
        r_bias = _buf->r_bias / (float)10000.0;
        ax_bias = _buf->ax_bias / (float)1000.0;
        ay_bias = _buf->ay_bias / (float)1000.0;
        az_bias = _buf->az_bias / (float)1000.0;
        max_pos_cov = _buf->max_pos_cov / (float)100.0;
        max_vel_cov = _buf->max_vel_cov / (float)1000.0;
        max_att_cov = _buf->max_att_cov / (float)10000.0;
        sequence_num = _buf->sequence_num;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("latitude_deg", latitude_deg);
        node.setDouble("longitude_deg", longitude_deg);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_ms", vn_ms);
        node.setDouble("ve_ms", ve_ms);
        node.setDouble("vd_ms", vd_ms);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("pitch_deg", pitch_deg);
        node.setDouble("yaw_deg", yaw_deg);
        node.setDouble("p_bias", p_bias);
        node.setDouble("q_bias", q_bias);
        node.setDouble("r_bias", r_bias);
        node.setDouble("ax_bias", ax_bias);
        node.setDouble("ay_bias", ay_bias);
        node.setDouble("az_bias", az_bias);
        node.setDouble("max_pos_cov", max_pos_cov);
        node.setDouble("max_vel_cov", max_vel_cov);
        node.setDouble("max_att_cov", max_att_cov);
        node.setUInt("sequence_num", sequence_num);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        latitude_deg = node.getDouble("latitude_deg");
        longitude_deg = node.getDouble("longitude_deg");
        altitude_m = node.getDouble("altitude_m");
        vn_ms = node.getDouble("vn_ms");
        ve_ms = node.getDouble("ve_ms");
        vd_ms = node.getDouble("vd_ms");
        roll_deg = node.getDouble("roll_deg");
        pitch_deg = node.getDouble("pitch_deg");
        yaw_deg = node.getDouble("yaw_deg");
        p_bias = node.getDouble("p_bias");
        q_bias = node.getDouble("q_bias");
        r_bias = node.getDouble("r_bias");
        ax_bias = node.getDouble("ax_bias");
        ay_bias = node.getDouble("ay_bias");
        az_bias = node.getDouble("az_bias");
        max_pos_cov = node.getDouble("max_pos_cov");
        max_vel_cov = node.getDouble("max_vel_cov");
        max_att_cov = node.getDouble("max_att_cov");
        sequence_num = node.getUInt("sequence_num");
        status = node.getUInt("status");
    }
};

// Message: nav_v6 (id: 52)
class nav_v6_t {
public:

    uint8_t index;
    uint32_t millis;
    int32_t latitude_raw;
    int32_t longitude_raw;
    float altitude_m;
    float vn_mps;
    float ve_mps;
    float vd_mps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        int32_t latitude_raw;
        int32_t longitude_raw;
        float altitude_m;
        int16_t vn_mps;
        int16_t ve_mps;
        int16_t vd_mps;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        uint8_t sequence_num;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 52;
    uint8_t *payload = nullptr;
    int len = 0;

    ~nav_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->latitude_raw = latitude_raw;
        _buf->longitude_raw = longitude_raw;
        _buf->altitude_m = altitude_m;
        _buf->vn_mps = intround(vn_mps * 100.0);
        _buf->ve_mps = intround(ve_mps * 100.0);
        _buf->vd_mps = intround(vd_mps * 100.0);
        _buf->roll_deg = intround(roll_deg * 50.0);
        _buf->pitch_deg = intround(pitch_deg * 50.0);
        _buf->yaw_deg = intround(yaw_deg * 50.0);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        latitude_raw = _buf->latitude_raw;
        longitude_raw = _buf->longitude_raw;
        altitude_m = _buf->altitude_m;
        vn_mps = _buf->vn_mps / (float)100.0;
        ve_mps = _buf->ve_mps / (float)100.0;
        vd_mps = _buf->vd_mps / (float)100.0;
        roll_deg = _buf->roll_deg / (float)50.0;
        pitch_deg = _buf->pitch_deg / (float)50.0;
        yaw_deg = _buf->yaw_deg / (float)50.0;
        sequence_num = _buf->sequence_num;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        node.setInt("latitude_raw", latitude_raw);
        node.setInt("longitude_raw", longitude_raw);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_mps", vn_mps);
        node.setDouble("ve_mps", ve_mps);
        node.setDouble("vd_mps", vd_mps);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("pitch_deg", pitch_deg);
        node.setDouble("yaw_deg", yaw_deg);
        node.setUInt("sequence_num", sequence_num);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        latitude_raw = node.getInt("latitude_raw");
        longitude_raw = node.getInt("longitude_raw");
        altitude_m = node.getDouble("altitude_m");
        vn_mps = node.getDouble("vn_mps");
        ve_mps = node.getDouble("ve_mps");
        vd_mps = node.getDouble("vd_mps");
        roll_deg = node.getDouble("roll_deg");
        pitch_deg = node.getDouble("pitch_deg");
        yaw_deg = node.getDouble("yaw_deg");
        sequence_num = node.getUInt("sequence_num");
        status = node.getUInt("status");
    }
};

// Message: nav_metrics_v6 (id: 53)
class nav_metrics_v6_t {
public:

    uint8_t index;
    uint32_t metrics_millis;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    float Pp0;
    float Pp1;
    float Pp2;
    float Pv0;
    float Pv1;
    float Pv2;
    float Pa0;
    float Pa1;
    float Pa2;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t metrics_millis;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint16_t Pp0;
        uint16_t Pp1;
        uint16_t Pp2;
        uint16_t Pv0;
        uint16_t Pv1;
        uint16_t Pv2;
        uint16_t Pa0;
        uint16_t Pa1;
        uint16_t Pa2;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 53;
    uint8_t *payload = nullptr;
    int len = 0;

    ~nav_metrics_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->metrics_millis = metrics_millis;
        _buf->p_bias = intround(p_bias * 10000.0);
        _buf->q_bias = intround(q_bias * 10000.0);
        _buf->r_bias = intround(r_bias * 10000.0);
        _buf->ax_bias = intround(ax_bias * 1000.0);
        _buf->ay_bias = intround(ay_bias * 1000.0);
        _buf->az_bias = intround(az_bias * 1000.0);
        _buf->Pp0 = uintround(Pp0 * 100.0);
        _buf->Pp1 = uintround(Pp1 * 100.0);
        _buf->Pp2 = uintround(Pp2 * 100.0);
        _buf->Pv0 = uintround(Pv0 * 1000.0);
        _buf->Pv1 = uintround(Pv1 * 1000.0);
        _buf->Pv2 = uintround(Pv2 * 1000.0);
        _buf->Pa0 = uintround(Pa0 * 10000.0);
        _buf->Pa1 = uintround(Pa1 * 10000.0);
        _buf->Pa2 = uintround(Pa2 * 10000.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        metrics_millis = _buf->metrics_millis;
        p_bias = _buf->p_bias / (float)10000.0;
        q_bias = _buf->q_bias / (float)10000.0;
        r_bias = _buf->r_bias / (float)10000.0;
        ax_bias = _buf->ax_bias / (float)1000.0;
        ay_bias = _buf->ay_bias / (float)1000.0;
        az_bias = _buf->az_bias / (float)1000.0;
        Pp0 = _buf->Pp0 / (float)100.0;
        Pp1 = _buf->Pp1 / (float)100.0;
        Pp2 = _buf->Pp2 / (float)100.0;
        Pv0 = _buf->Pv0 / (float)1000.0;
        Pv1 = _buf->Pv1 / (float)1000.0;
        Pv2 = _buf->Pv2 / (float)1000.0;
        Pa0 = _buf->Pa0 / (float)10000.0;
        Pa1 = _buf->Pa1 / (float)10000.0;
        Pa2 = _buf->Pa2 / (float)10000.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("metrics_millis", metrics_millis);
        node.setDouble("p_bias", p_bias);
        node.setDouble("q_bias", q_bias);
        node.setDouble("r_bias", r_bias);
        node.setDouble("ax_bias", ax_bias);
        node.setDouble("ay_bias", ay_bias);
        node.setDouble("az_bias", az_bias);
        node.setDouble("Pp0", Pp0);
        node.setDouble("Pp1", Pp1);
        node.setDouble("Pp2", Pp2);
        node.setDouble("Pv0", Pv0);
        node.setDouble("Pv1", Pv1);
        node.setDouble("Pv2", Pv2);
        node.setDouble("Pa0", Pa0);
        node.setDouble("Pa1", Pa1);
        node.setDouble("Pa2", Pa2);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        metrics_millis = node.getUInt("metrics_millis");
        p_bias = node.getDouble("p_bias");
        q_bias = node.getDouble("q_bias");
        r_bias = node.getDouble("r_bias");
        ax_bias = node.getDouble("ax_bias");
        ay_bias = node.getDouble("ay_bias");
        az_bias = node.getDouble("az_bias");
        Pp0 = node.getDouble("Pp0");
        Pp1 = node.getDouble("Pp1");
        Pp2 = node.getDouble("Pp2");
        Pv0 = node.getDouble("Pv0");
        Pv1 = node.getDouble("Pv1");
        Pv2 = node.getDouble("Pv2");
        Pa0 = node.getDouble("Pa0");
        Pa1 = node.getDouble("Pa1");
        Pa2 = node.getDouble("Pa2");
    }
};

// Message: actuator_v2 (id: 21)
class actuator_v2_t {
public:

    uint8_t index;
    double timestamp_sec;
    float aileron;
    float elevator;
    float throttle;
    float rudder;
    float channel5;
    float flaps;
    float channel7;
    float channel8;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        int16_t aileron;
        int16_t elevator;
        uint16_t throttle;
        int16_t rudder;
        int16_t channel5;
        int16_t flaps;
        int16_t channel7;
        int16_t channel8;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 21;
    uint8_t *payload = nullptr;
    int len = 0;

    ~actuator_v2_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->aileron = intround(aileron * 20000.0);
        _buf->elevator = intround(elevator * 20000.0);
        _buf->throttle = uintround(throttle * 60000.0);
        _buf->rudder = intround(rudder * 20000.0);
        _buf->channel5 = intround(channel5 * 20000.0);
        _buf->flaps = intround(flaps * 20000.0);
        _buf->channel7 = intround(channel7 * 20000.0);
        _buf->channel8 = intround(channel8 * 20000.0);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        aileron = _buf->aileron / (float)20000.0;
        elevator = _buf->elevator / (float)20000.0;
        throttle = _buf->throttle / (float)60000.0;
        rudder = _buf->rudder / (float)20000.0;
        channel5 = _buf->channel5 / (float)20000.0;
        flaps = _buf->flaps / (float)20000.0;
        channel7 = _buf->channel7 / (float)20000.0;
        channel8 = _buf->channel8 / (float)20000.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("aileron", aileron);
        node.setDouble("elevator", elevator);
        node.setDouble("throttle", throttle);
        node.setDouble("rudder", rudder);
        node.setDouble("channel5", channel5);
        node.setDouble("flaps", flaps);
        node.setDouble("channel7", channel7);
        node.setDouble("channel8", channel8);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        aileron = node.getDouble("aileron");
        elevator = node.getDouble("elevator");
        throttle = node.getDouble("throttle");
        rudder = node.getDouble("rudder");
        channel5 = node.getDouble("channel5");
        flaps = node.getDouble("flaps");
        channel7 = node.getDouble("channel7");
        channel8 = node.getDouble("channel8");
        status = node.getUInt("status");
    }
};

// Message: actuator_v3 (id: 37)
class actuator_v3_t {
public:

    uint8_t index;
    float timestamp_sec;
    float aileron;
    float elevator;
    float throttle;
    float rudder;
    float channel5;
    float flaps;
    float channel7;
    float channel8;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        int16_t aileron;
        int16_t elevator;
        uint16_t throttle;
        int16_t rudder;
        int16_t channel5;
        int16_t flaps;
        int16_t channel7;
        int16_t channel8;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 37;
    uint8_t *payload = nullptr;
    int len = 0;

    ~actuator_v3_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->aileron = intround(aileron * 20000.0);
        _buf->elevator = intround(elevator * 20000.0);
        _buf->throttle = uintround(throttle * 60000.0);
        _buf->rudder = intround(rudder * 20000.0);
        _buf->channel5 = intround(channel5 * 20000.0);
        _buf->flaps = intround(flaps * 20000.0);
        _buf->channel7 = intround(channel7 * 20000.0);
        _buf->channel8 = intround(channel8 * 20000.0);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        aileron = _buf->aileron / (float)20000.0;
        elevator = _buf->elevator / (float)20000.0;
        throttle = _buf->throttle / (float)60000.0;
        rudder = _buf->rudder / (float)20000.0;
        channel5 = _buf->channel5 / (float)20000.0;
        flaps = _buf->flaps / (float)20000.0;
        channel7 = _buf->channel7 / (float)20000.0;
        channel8 = _buf->channel8 / (float)20000.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("aileron", aileron);
        node.setDouble("elevator", elevator);
        node.setDouble("throttle", throttle);
        node.setDouble("rudder", rudder);
        node.setDouble("channel5", channel5);
        node.setDouble("flaps", flaps);
        node.setDouble("channel7", channel7);
        node.setDouble("channel8", channel8);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        aileron = node.getDouble("aileron");
        elevator = node.getDouble("elevator");
        throttle = node.getDouble("throttle");
        rudder = node.getDouble("rudder");
        channel5 = node.getDouble("channel5");
        flaps = node.getDouble("flaps");
        channel7 = node.getDouble("channel7");
        channel8 = node.getDouble("channel8");
        status = node.getUInt("status");
    }
};

// Message: effectors_v1 (id: 61)
class effectors_v1_t {
public:

    uint8_t index;
    uint32_t millis;
    float channel[8];

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        int16_t channel[8];
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 61;
    uint8_t *payload = nullptr;
    int len = 0;

    ~effectors_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        for (int _i=0; _i<8; _i++) _buf->channel[_i] = intround(channel[_i] * 20000.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf->channel[_i] / (float)20000.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        for (int _i=0; _i<8; _i++) node.setDouble("channel", channel[_i], _i);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        for (int _i=0; _i<8; _i++) channel[_i] = node.getDouble("channel", _i);
    }
};

// Message: pilot_v3 (id: 38)
class pilot_v3_t {
public:

    uint8_t index;
    float timestamp_sec;
    float channel[8];
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        int16_t channel[8];
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 38;
    uint8_t *payload = nullptr;
    int len = 0;

    ~pilot_v3_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        for (int _i=0; _i<8; _i++) _buf->channel[_i] = intround(channel[_i] * 20000.0);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf->channel[_i] / (float)20000.0;
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

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        for (int _i=0; _i<8; _i++) node.setDouble("channel", channel[_i], _i);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        for (int _i=0; _i<8; _i++) channel[_i] = node.getDouble("channel", _i);
        status = node.getUInt("status");
    }
};

// Message: pilot_v4 (id: 51)
class pilot_v4_t {
public:

    uint8_t index;
    uint32_t millis;
    float channel[sbus_channels];
    uint8_t failsafe;
    uint8_t master_switch;
    uint8_t throttle_safety;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        int16_t channel[sbus_channels];
        uint8_t failsafe;
        uint8_t master_switch;
        uint8_t throttle_safety;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 51;
    uint8_t *payload = nullptr;
    int len = 0;

    ~pilot_v4_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        for (int _i=0; _i<sbus_channels; _i++) _buf->channel[_i] = intround(channel[_i] * 20000.0);
        _buf->failsafe = failsafe;
        _buf->master_switch = master_switch;
        _buf->throttle_safety = throttle_safety;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        for (int _i=0; _i<sbus_channels; _i++) channel[_i] = _buf->channel[_i] / (float)20000.0;
        failsafe = _buf->failsafe;
        master_switch = _buf->master_switch;
        throttle_safety = _buf->throttle_safety;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        for (int _i=0; _i<sbus_channels; _i++) node.setDouble("channel", channel[_i], _i);
        node.setUInt("failsafe", failsafe);
        node.setUInt("master_switch", master_switch);
        node.setUInt("throttle_safety", throttle_safety);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        for (int _i=0; _i<sbus_channels; _i++) channel[_i] = node.getDouble("channel", _i);
        failsafe = node.getUInt("failsafe");
        master_switch = node.getUInt("master_switch");
        throttle_safety = node.getUInt("throttle_safety");
    }
};

// Message: inceptors_v1 (id: 62)
class inceptors_v1_t {
public:

    uint8_t index;
    uint32_t millis;
    float channel[ap_channels];

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        int16_t channel[ap_channels];
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 62;
    uint8_t *payload = nullptr;
    int len = 0;

    ~inceptors_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        for (int _i=0; _i<ap_channels; _i++) _buf->channel[_i] = intround(channel[_i] * 2000.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        for (int _i=0; _i<ap_channels; _i++) channel[_i] = _buf->channel[_i] / (float)2000.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        for (int _i=0; _i<ap_channels; _i++) node.setDouble("channel", channel[_i], _i);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        for (int _i=0; _i<ap_channels; _i++) channel[_i] = node.getDouble("channel", _i);
    }
};

// Message: power_v1 (id: 55)
class power_v1_t {
public:

    uint8_t index;
    uint32_t millis;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 55;
    uint8_t *payload = nullptr;
    int len = 0;

    ~power_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->avionics_vcc = uintround(avionics_vcc * 1000.0);
        _buf->main_vcc = uintround(main_vcc * 1000.0);
        _buf->cell_vcc = uintround(cell_vcc * 1000.0);
        _buf->main_amps = uintround(main_amps * 1000.0);
        _buf->total_mah = uintround(total_mah * 0.5);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        avionics_vcc = _buf->avionics_vcc / (float)1000.0;
        main_vcc = _buf->main_vcc / (float)1000.0;
        cell_vcc = _buf->cell_vcc / (float)1000.0;
        main_amps = _buf->main_amps / (float)1000.0;
        total_mah = _buf->total_mah / (float)0.5;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        node.setDouble("avionics_vcc", avionics_vcc);
        node.setDouble("main_vcc", main_vcc);
        node.setDouble("cell_vcc", cell_vcc);
        node.setDouble("main_amps", main_amps);
        node.setDouble("total_mah", total_mah);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        avionics_vcc = node.getDouble("avionics_vcc");
        main_vcc = node.getDouble("main_vcc");
        cell_vcc = node.getDouble("cell_vcc");
        main_amps = node.getDouble("main_amps");
        total_mah = node.getDouble("total_mah");
    }
};

// Message: ap_status_v6 (id: 33)
class ap_status_v6_t {
public:

    uint8_t index;
    double timestamp_sec;
    uint8_t flags;
    float groundtrack_deg;
    float roll_deg;
    uint16_t altitude_msl_ft;
    uint16_t altitude_ground_m;
    float pitch_deg;
    float airspeed_kt;
    uint16_t flight_timer;
    uint16_t target_waypoint_idx;
    double wp_longitude_deg;
    double wp_latitude_deg;
    uint16_t wp_index;
    uint16_t route_size;
    uint8_t task_id;
    uint16_t task_attribute;
    uint8_t sequence_num;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint8_t flags;
        int16_t groundtrack_deg;
        int16_t roll_deg;
        uint16_t altitude_msl_ft;
        uint16_t altitude_ground_m;
        int16_t pitch_deg;
        int16_t airspeed_kt;
        uint16_t flight_timer;
        uint16_t target_waypoint_idx;
        double wp_longitude_deg;
        double wp_latitude_deg;
        uint16_t wp_index;
        uint16_t route_size;
        uint8_t task_id;
        uint16_t task_attribute;
        uint8_t sequence_num;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 33;
    uint8_t *payload = nullptr;
    int len = 0;

    ~ap_status_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->flags = flags;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10.0);
        _buf->roll_deg = intround(roll_deg * 10.0);
        _buf->altitude_msl_ft = altitude_msl_ft;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->pitch_deg = intround(pitch_deg * 10.0);
        _buf->airspeed_kt = intround(airspeed_kt * 10.0);
        _buf->flight_timer = flight_timer;
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_longitude_deg = wp_longitude_deg;
        _buf->wp_latitude_deg = wp_latitude_deg;
        _buf->wp_index = wp_index;
        _buf->route_size = route_size;
        _buf->task_id = task_id;
        _buf->task_attribute = task_attribute;
        _buf->sequence_num = sequence_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        flags = _buf->flags;
        groundtrack_deg = _buf->groundtrack_deg / (float)10.0;
        roll_deg = _buf->roll_deg / (float)10.0;
        altitude_msl_ft = _buf->altitude_msl_ft;
        altitude_ground_m = _buf->altitude_ground_m;
        pitch_deg = _buf->pitch_deg / (float)10.0;
        airspeed_kt = _buf->airspeed_kt / (float)10.0;
        flight_timer = _buf->flight_timer;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_longitude_deg = _buf->wp_longitude_deg;
        wp_latitude_deg = _buf->wp_latitude_deg;
        wp_index = _buf->wp_index;
        route_size = _buf->route_size;
        task_id = _buf->task_id;
        task_attribute = _buf->task_attribute;
        sequence_num = _buf->sequence_num;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setUInt("flags", flags);
        node.setDouble("groundtrack_deg", groundtrack_deg);
        node.setDouble("roll_deg", roll_deg);
        node.setUInt("altitude_msl_ft", altitude_msl_ft);
        node.setUInt("altitude_ground_m", altitude_ground_m);
        node.setDouble("pitch_deg", pitch_deg);
        node.setDouble("airspeed_kt", airspeed_kt);
        node.setUInt("flight_timer", flight_timer);
        node.setUInt("target_waypoint_idx", target_waypoint_idx);
        node.setDouble("wp_longitude_deg", wp_longitude_deg);
        node.setDouble("wp_latitude_deg", wp_latitude_deg);
        node.setUInt("wp_index", wp_index);
        node.setUInt("route_size", route_size);
        node.setUInt("task_id", task_id);
        node.setUInt("task_attribute", task_attribute);
        node.setUInt("sequence_num", sequence_num);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        flags = node.getUInt("flags");
        groundtrack_deg = node.getDouble("groundtrack_deg");
        roll_deg = node.getDouble("roll_deg");
        altitude_msl_ft = node.getUInt("altitude_msl_ft");
        altitude_ground_m = node.getUInt("altitude_ground_m");
        pitch_deg = node.getDouble("pitch_deg");
        airspeed_kt = node.getDouble("airspeed_kt");
        flight_timer = node.getUInt("flight_timer");
        target_waypoint_idx = node.getUInt("target_waypoint_idx");
        wp_longitude_deg = node.getDouble("wp_longitude_deg");
        wp_latitude_deg = node.getDouble("wp_latitude_deg");
        wp_index = node.getUInt("wp_index");
        route_size = node.getUInt("route_size");
        task_id = node.getUInt("task_id");
        task_attribute = node.getUInt("task_attribute");
        sequence_num = node.getUInt("sequence_num");
    }
};

// Message: ap_status_v7 (id: 39)
class ap_status_v7_t {
public:

    uint8_t index;
    float timestamp_sec;
    uint8_t flags;
    float groundtrack_deg;
    float roll_deg;
    float altitude_msl_ft;
    float altitude_ground_m;
    float pitch_deg;
    float airspeed_kt;
    float flight_timer;
    uint16_t target_waypoint_idx;
    double wp_longitude_deg;
    double wp_latitude_deg;
    uint16_t wp_index;
    uint16_t route_size;
    uint8_t task_id;
    uint16_t task_attribute;
    uint8_t sequence_num;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint8_t flags;
        int16_t groundtrack_deg;
        int16_t roll_deg;
        uint16_t altitude_msl_ft;
        uint16_t altitude_ground_m;
        int16_t pitch_deg;
        int16_t airspeed_kt;
        uint16_t flight_timer;
        uint16_t target_waypoint_idx;
        double wp_longitude_deg;
        double wp_latitude_deg;
        uint16_t wp_index;
        uint16_t route_size;
        uint8_t task_id;
        uint16_t task_attribute;
        uint8_t sequence_num;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 39;
    uint8_t *payload = nullptr;
    int len = 0;

    ~ap_status_v7_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->flags = flags;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10.0);
        _buf->roll_deg = intround(roll_deg * 10.0);
        _buf->altitude_msl_ft = uintround(altitude_msl_ft * 1.0);
        _buf->altitude_ground_m = uintround(altitude_ground_m * 1.0);
        _buf->pitch_deg = intround(pitch_deg * 10.0);
        _buf->airspeed_kt = intround(airspeed_kt * 10.0);
        _buf->flight_timer = uintround(flight_timer * 1.0);
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_longitude_deg = wp_longitude_deg;
        _buf->wp_latitude_deg = wp_latitude_deg;
        _buf->wp_index = wp_index;
        _buf->route_size = route_size;
        _buf->task_id = task_id;
        _buf->task_attribute = task_attribute;
        _buf->sequence_num = sequence_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        flags = _buf->flags;
        groundtrack_deg = _buf->groundtrack_deg / (float)10.0;
        roll_deg = _buf->roll_deg / (float)10.0;
        altitude_msl_ft = _buf->altitude_msl_ft / (float)1.0;
        altitude_ground_m = _buf->altitude_ground_m / (float)1.0;
        pitch_deg = _buf->pitch_deg / (float)10.0;
        airspeed_kt = _buf->airspeed_kt / (float)10.0;
        flight_timer = _buf->flight_timer / (float)1.0;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_longitude_deg = _buf->wp_longitude_deg;
        wp_latitude_deg = _buf->wp_latitude_deg;
        wp_index = _buf->wp_index;
        route_size = _buf->route_size;
        task_id = _buf->task_id;
        task_attribute = _buf->task_attribute;
        sequence_num = _buf->sequence_num;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setUInt("flags", flags);
        node.setDouble("groundtrack_deg", groundtrack_deg);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("altitude_msl_ft", altitude_msl_ft);
        node.setDouble("altitude_ground_m", altitude_ground_m);
        node.setDouble("pitch_deg", pitch_deg);
        node.setDouble("airspeed_kt", airspeed_kt);
        node.setDouble("flight_timer", flight_timer);
        node.setUInt("target_waypoint_idx", target_waypoint_idx);
        node.setDouble("wp_longitude_deg", wp_longitude_deg);
        node.setDouble("wp_latitude_deg", wp_latitude_deg);
        node.setUInt("wp_index", wp_index);
        node.setUInt("route_size", route_size);
        node.setUInt("task_id", task_id);
        node.setUInt("task_attribute", task_attribute);
        node.setUInt("sequence_num", sequence_num);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        flags = node.getUInt("flags");
        groundtrack_deg = node.getDouble("groundtrack_deg");
        roll_deg = node.getDouble("roll_deg");
        altitude_msl_ft = node.getDouble("altitude_msl_ft");
        altitude_ground_m = node.getDouble("altitude_ground_m");
        pitch_deg = node.getDouble("pitch_deg");
        airspeed_kt = node.getDouble("airspeed_kt");
        flight_timer = node.getDouble("flight_timer");
        target_waypoint_idx = node.getUInt("target_waypoint_idx");
        wp_longitude_deg = node.getDouble("wp_longitude_deg");
        wp_latitude_deg = node.getDouble("wp_latitude_deg");
        wp_index = node.getUInt("wp_index");
        route_size = node.getUInt("route_size");
        task_id = node.getUInt("task_id");
        task_attribute = node.getUInt("task_attribute");
        sequence_num = node.getUInt("sequence_num");
    }
};

// Message: ap_targets_v1 (id: 59)
class ap_targets_v1_t {
public:

    uint8_t index;
    uint32_t millis;
    float groundtrack_deg;
    float altitude_agl_ft;
    float airspeed_kt;
    float roll_deg;
    float pitch_deg;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        int16_t groundtrack_deg;
        uint16_t altitude_agl_ft;
        int16_t airspeed_kt;
        int16_t roll_deg;
        int16_t pitch_deg;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 59;
    uint8_t *payload = nullptr;
    int len = 0;

    ~ap_targets_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10.0);
        _buf->altitude_agl_ft = uintround(altitude_agl_ft * 10.0);
        _buf->airspeed_kt = intround(airspeed_kt * 10.0);
        _buf->roll_deg = intround(roll_deg * 10.0);
        _buf->pitch_deg = intround(pitch_deg * 10.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        groundtrack_deg = _buf->groundtrack_deg / (float)10.0;
        altitude_agl_ft = _buf->altitude_agl_ft / (float)10.0;
        airspeed_kt = _buf->airspeed_kt / (float)10.0;
        roll_deg = _buf->roll_deg / (float)10.0;
        pitch_deg = _buf->pitch_deg / (float)10.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        node.setDouble("groundtrack_deg", groundtrack_deg);
        node.setDouble("altitude_agl_ft", altitude_agl_ft);
        node.setDouble("airspeed_kt", airspeed_kt);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("pitch_deg", pitch_deg);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        groundtrack_deg = node.getDouble("groundtrack_deg");
        altitude_agl_ft = node.getDouble("altitude_agl_ft");
        airspeed_kt = node.getDouble("airspeed_kt");
        roll_deg = node.getDouble("roll_deg");
        pitch_deg = node.getDouble("pitch_deg");
    }
};

// Message: mission_v1 (id: 60)
class mission_v1_t {
public:

    uint8_t index;
    uint32_t millis;
    uint8_t is_airborne;
    float flight_timer;
    string task_name;
    uint16_t task_attribute;
    uint16_t route_size;
    uint16_t target_waypoint_idx;
    uint16_t wp_index;
    int32_t wp_longitude_raw;
    int32_t wp_latitude_raw;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        uint8_t is_airborne;
        uint16_t flight_timer;
        uint16_t task_name_len;
        uint16_t task_attribute;
        uint16_t route_size;
        uint16_t target_waypoint_idx;
        uint16_t wp_index;
        int32_t wp_longitude_raw;
        int32_t wp_latitude_raw;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 60;
    uint8_t *payload = nullptr;
    int len = 0;

    ~mission_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += task_name.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->is_airborne = is_airborne;
        _buf->flight_timer = uintround(flight_timer * 1.0);
        _buf->task_name_len = task_name.length();
        _buf->task_attribute = task_attribute;
        _buf->route_size = route_size;
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_index = wp_index;
        _buf->wp_longitude_raw = wp_longitude_raw;
        _buf->wp_latitude_raw = wp_latitude_raw;
        memcpy(&(payload[len]), task_name.c_str(), task_name.length());
        len += task_name.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        is_airborne = _buf->is_airborne;
        flight_timer = _buf->flight_timer / (float)1.0;
        task_attribute = _buf->task_attribute;
        route_size = _buf->route_size;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_index = _buf->wp_index;
        wp_longitude_raw = _buf->wp_longitude_raw;
        wp_latitude_raw = _buf->wp_latitude_raw;
        task_name = string((char *)&(external_message[len]), _buf->task_name_len);
        len += _buf->task_name_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        node.setUInt("is_airborne", is_airborne);
        node.setDouble("flight_timer", flight_timer);
        node.setString("task_name", task_name);
        node.setUInt("task_attribute", task_attribute);
        node.setUInt("route_size", route_size);
        node.setUInt("target_waypoint_idx", target_waypoint_idx);
        node.setUInt("wp_index", wp_index);
        node.setInt("wp_longitude_raw", wp_longitude_raw);
        node.setInt("wp_latitude_raw", wp_latitude_raw);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        is_airborne = node.getUInt("is_airborne");
        flight_timer = node.getDouble("flight_timer");
        task_name = node.getString("task_name");
        task_attribute = node.getUInt("task_attribute");
        route_size = node.getUInt("route_size");
        target_waypoint_idx = node.getUInt("target_waypoint_idx");
        wp_index = node.getUInt("wp_index");
        wp_longitude_raw = node.getInt("wp_longitude_raw");
        wp_latitude_raw = node.getInt("wp_latitude_raw");
    }
};

// Message: system_health_v5 (id: 41)
class system_health_v5_t {
public:

    uint8_t index;
    float timestamp_sec;
    float system_load_avg;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t system_load_avg;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 41;
    uint8_t *payload = nullptr;
    int len = 0;

    ~system_health_v5_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->system_load_avg = uintround(system_load_avg * 100.0);
        _buf->avionics_vcc = uintround(avionics_vcc * 1000.0);
        _buf->main_vcc = uintround(main_vcc * 1000.0);
        _buf->cell_vcc = uintround(cell_vcc * 1000.0);
        _buf->main_amps = uintround(main_amps * 1000.0);
        _buf->total_mah = uintround(total_mah * 0.1);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        system_load_avg = _buf->system_load_avg / (float)100.0;
        avionics_vcc = _buf->avionics_vcc / (float)1000.0;
        main_vcc = _buf->main_vcc / (float)1000.0;
        cell_vcc = _buf->cell_vcc / (float)1000.0;
        main_amps = _buf->main_amps / (float)1000.0;
        total_mah = _buf->total_mah / (float)0.1;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("system_load_avg", system_load_avg);
        node.setDouble("avionics_vcc", avionics_vcc);
        node.setDouble("main_vcc", main_vcc);
        node.setDouble("cell_vcc", cell_vcc);
        node.setDouble("main_amps", main_amps);
        node.setDouble("total_mah", total_mah);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        system_load_avg = node.getDouble("system_load_avg");
        avionics_vcc = node.getDouble("avionics_vcc");
        main_vcc = node.getDouble("main_vcc");
        cell_vcc = node.getDouble("cell_vcc");
        main_amps = node.getDouble("main_amps");
        total_mah = node.getDouble("total_mah");
    }
};

// Message: system_health_v6 (id: 46)
class system_health_v6_t {
public:

    uint8_t index;
    float timestamp_sec;
    float system_load_avg;
    uint16_t fmu_timer_misses;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t system_load_avg;
        uint16_t fmu_timer_misses;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 46;
    uint8_t *payload = nullptr;
    int len = 0;

    ~system_health_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->system_load_avg = uintround(system_load_avg * 100.0);
        _buf->fmu_timer_misses = fmu_timer_misses;
        _buf->avionics_vcc = uintround(avionics_vcc * 1000.0);
        _buf->main_vcc = uintround(main_vcc * 1000.0);
        _buf->cell_vcc = uintround(cell_vcc * 1000.0);
        _buf->main_amps = uintround(main_amps * 1000.0);
        _buf->total_mah = uintround(total_mah * 0.1);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        system_load_avg = _buf->system_load_avg / (float)100.0;
        fmu_timer_misses = _buf->fmu_timer_misses;
        avionics_vcc = _buf->avionics_vcc / (float)1000.0;
        main_vcc = _buf->main_vcc / (float)1000.0;
        cell_vcc = _buf->cell_vcc / (float)1000.0;
        main_amps = _buf->main_amps / (float)1000.0;
        total_mah = _buf->total_mah / (float)0.1;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setDouble("system_load_avg", system_load_avg);
        node.setUInt("fmu_timer_misses", fmu_timer_misses);
        node.setDouble("avionics_vcc", avionics_vcc);
        node.setDouble("main_vcc", main_vcc);
        node.setDouble("cell_vcc", cell_vcc);
        node.setDouble("main_amps", main_amps);
        node.setDouble("total_mah", total_mah);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        system_load_avg = node.getDouble("system_load_avg");
        fmu_timer_misses = node.getUInt("fmu_timer_misses");
        avionics_vcc = node.getDouble("avionics_vcc");
        main_vcc = node.getDouble("main_vcc");
        cell_vcc = node.getDouble("cell_vcc");
        main_amps = node.getDouble("main_amps");
        total_mah = node.getDouble("total_mah");
    }
};

// Message: status_v7 (id: 56)
class status_v7_t {
public:

    uint8_t index;
    uint32_t millis;
    uint16_t serial_number;
    uint16_t firmware_rev;
    uint8_t master_hz;
    uint32_t baud;
    uint32_t available_memory;
    uint16_t byte_rate;
    uint16_t main_loop_timer_misses;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        uint32_t millis;
        uint16_t serial_number;
        uint16_t firmware_rev;
        uint8_t master_hz;
        uint32_t baud;
        uint32_t available_memory;
        uint16_t byte_rate;
        uint16_t main_loop_timer_misses;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 56;
    uint8_t *payload = nullptr;
    int len = 0;

    ~status_v7_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->millis = millis;
        _buf->serial_number = serial_number;
        _buf->firmware_rev = firmware_rev;
        _buf->master_hz = master_hz;
        _buf->baud = baud;
        _buf->available_memory = available_memory;
        _buf->byte_rate = byte_rate;
        _buf->main_loop_timer_misses = main_loop_timer_misses;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        millis = _buf->millis;
        serial_number = _buf->serial_number;
        firmware_rev = _buf->firmware_rev;
        master_hz = _buf->master_hz;
        baud = _buf->baud;
        available_memory = _buf->available_memory;
        byte_rate = _buf->byte_rate;
        main_loop_timer_misses = _buf->main_loop_timer_misses;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setUInt("millis", millis);
        node.setUInt("serial_number", serial_number);
        node.setUInt("firmware_rev", firmware_rev);
        node.setUInt("master_hz", master_hz);
        node.setUInt("baud", baud);
        node.setUInt("available_memory", available_memory);
        node.setUInt("byte_rate", byte_rate);
        node.setUInt("main_loop_timer_misses", main_loop_timer_misses);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        millis = node.getUInt("millis");
        serial_number = node.getUInt("serial_number");
        firmware_rev = node.getUInt("firmware_rev");
        master_hz = node.getUInt("master_hz");
        baud = node.getUInt("baud");
        available_memory = node.getUInt("available_memory");
        byte_rate = node.getUInt("byte_rate");
        main_loop_timer_misses = node.getUInt("main_loop_timer_misses");
    }
};

// Message: event_v1 (id: 27)
class event_v1_t {
public:

    uint8_t index;
    double timestamp_sec;
    string message;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint16_t message_len;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 27;
    uint8_t *payload = nullptr;
    int len = 0;

    ~event_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += message.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        message = string((char *)&(external_message[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("index", index);
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setString("message", message);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        index = node.getUInt("index");
        timestamp_sec = node.getDouble("timestamp_sec");
        message = node.getString("message");
    }
};

// Message: event_v2 (id: 44)
class event_v2_t {
public:

    float timestamp_sec;
    uint8_t sequence_num;
    string message;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        float timestamp_sec;
        uint8_t sequence_num;
        uint16_t message_len;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 44;
    uint8_t *payload = nullptr;
    int len = 0;

    ~event_v2_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += message.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->timestamp_sec = timestamp_sec;
        _buf->sequence_num = sequence_num;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        timestamp_sec = _buf->timestamp_sec;
        sequence_num = _buf->sequence_num;
        message = string((char *)&(external_message[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setDouble("timestamp_sec", timestamp_sec);
        node.setUInt("sequence_num", sequence_num);
        node.setString("message", message);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        timestamp_sec = node.getDouble("timestamp_sec");
        sequence_num = node.getUInt("sequence_num");
        message = node.getString("message");
    }
};

// Message: command_v1 (id: 28)
class command_v1_t {
public:

    uint16_t sequence_num;
    string message;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t sequence_num;
        uint16_t message_len;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 28;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += message.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->sequence_num = sequence_num;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        sequence_num = _buf->sequence_num;
        message = string((char *)&(external_message[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("sequence_num", sequence_num);
        node.setString("message", message);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        sequence_num = node.getUInt("sequence_num");
        message = node.getString("message");
    }
};

// Message: ack_v1 (id: 57)
class ack_v1_t {
public:

    uint16_t sequence_num;
    uint8_t result;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t sequence_num;
        uint8_t result;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 57;
    uint8_t *payload = nullptr;
    int len = 0;

    ~ack_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->sequence_num = sequence_num;
        _buf->result = result;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        sequence_num = _buf->sequence_num;
        result = _buf->result;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("sequence_num", sequence_num);
        node.setUInt("result", result);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        sequence_num = node.getUInt("sequence_num");
        result = node.getUInt("result");
    }
};

} // namespace ns_message
