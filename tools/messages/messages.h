#pragma once

#include <stdint.h>  // uint8_t, et. al.
#include <string.h>  // memcpy()

#include <string>
using std::string;

namespace message {

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t simple_test_id = 0;
const uint8_t array_test_id = 1;
const uint8_t dynamic_string_test_id = 2;
const uint8_t enum_test_id = 3;
const uint8_t gps_v4_id = 34;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

// Constants
static const uint8_t max_flags = 4;  // flags
static const uint8_t max_args = 4;  // args

// Enums
enum class enum_sequence1 {
    enum1 = 0,
    enum2 = 1,  // None
    enum3 = 2,
    enum4 = 3,
    enum5 = 4
};
enum class enum_sequence2 {
    enum1a = 0,
    enum2a = 1,
    enum3a = 2,
    enum4a = 3,
    enum5a = 4
};

// Message: simple_test (id: 0)
struct simple_test_t {
    // public fields
    int16_t a;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t a;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 0;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->a = a;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        a = _buf->a;
        return true;
    }
};

// Message: array_test (id: 1)
struct array_test_t {
    // public fields
    double time;
    int8_t flags[max_flags];
    float orientation[9];
    uint16_t something;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        double time;
        int8_t flags[max_flags];
        int16_t orientation[9];
        uint16_t something;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 1;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->time = time;
        for (int _i=0; _i<max_flags; _i++) _buf->flags[_i] = flags[_i];
        for (int _i=0; _i<9; _i++) _buf->orientation[_i] = intround(orientation[_i] * 53.3);
        _buf->something = something;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        time = _buf->time;
        for (int _i=0; _i<max_flags; _i++) flags[_i] = _buf->flags[_i];
        for (int _i=0; _i<9; _i++) orientation[_i] = _buf->orientation[_i] / (float)53.3;
        something = _buf->something;
        return true;
    }
};

// Message: dynamic_string_test (id: 2)
struct dynamic_string_test_t {
    // public fields
    double time;
    string event;
    uint16_t counter;
    string args[max_args];
    bool status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        double time;
        uint8_t event_len;
        uint16_t counter;
        uint8_t args_len[max_args];
        bool status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 2;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += event.length();
        for (int _i=0; _i<max_args; _i++) size += args[_i].length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->time = time;
        _buf->event_len = event.length();
        _buf->counter = counter;
        for (int _i=0; _i<max_args; _i++) _buf->args_len[_i] = args[_i].length();
        _buf->status = status;
        memcpy(&(payload[len]), event.c_str(), event.length());
        len += event.length();
        for (int _i=0; _i<max_args; _i++) {
            memcpy(&(payload[len]), args[_i].c_str(), args[_i].length());
            len += args[_i].length();
        }
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        time = _buf->time;
        counter = _buf->counter;
        status = _buf->status;
        event = string((char *)&(payload[len]), _buf->event_len);
        len += _buf->event_len;
        for (int _i=0; _i<max_args; _i++) {
            args[_i] = string((char *)&(payload[len]), _buf->args_len[_i]);
            len += _buf->args_len[_i];
        }
        return true;
    }
};

// Message: enum_test (id: 3)
struct enum_test_t {
    // public fields
    enum_sequence1 time;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t time;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 3;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->time = (uint8_t)time;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        time = (enum_sequence1)_buf->time;
        return true;
    }
};

// Message: gps_v4 (id: 34)
struct gps_v4_t {
    // public fields
    uint8_t index;
    float time_sec;
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
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float time_sec;
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

    // public info fields
    static const uint8_t id = 34;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->time_sec = time_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->unixtime_sec = unixtime_sec;
        _buf->satellites = satellites;
        _buf->horiz_accuracy_m = uintround(horiz_accuracy_m * 100);
        _buf->vert_accuracy_m = uintround(vert_accuracy_m * 100);
        _buf->pdop = uintround(pdop * 100);
        _buf->fix_type = fix_type;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        time_sec = _buf->time_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        unixtime_sec = _buf->unixtime_sec;
        satellites = _buf->satellites;
        horiz_accuracy_m = _buf->horiz_accuracy_m / (float)100;
        vert_accuracy_m = _buf->vert_accuracy_m / (float)100;
        pdop = _buf->pdop / (float)100;
        fix_type = _buf->fix_type;
        return true;
    }
};

} // namespace message
