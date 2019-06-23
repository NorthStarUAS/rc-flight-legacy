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
const uint8_t message_id_simple_test = 0;
const uint8_t message_id_array_test = 1;
const uint8_t message_id_gps_v4 = 34;

// Message: simple_test
// Id: 0
struct message_simple_test_t {
    int16_t a;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        int16_t a;
    } buf;
    #pragma pack(pop)

    const uint8_t id = 0;
    const uint16_t len = sizeof(buf);

    uint8_t *pack() {
        buf.a = a;
        return (uint8_t *)(&buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&buf, message, len);
        a = buf.a;
    }
};

// Message: array_test
// Id: 1
struct message_array_test_t {
    float orientation[9];

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        int16_t orientation[9];
    } buf;
    #pragma pack(pop)

    const uint8_t id = 1;
    const uint16_t len = sizeof(buf);

    uint8_t *pack() {
        for (int i=0; i<9; i++) buf.orientation[i] = intround(orientation[i] * 53.3);
        return (uint8_t *)(&buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&buf, message, len);
        for (int i=0; i<9; i++) orientation[i] = buf.orientation[i] / (float)53.3;
    }
};

// Message: gps_v4
// Id: 34
struct message_gps_v4_t {
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
    #pragma pack(push, 1)
    struct {
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
    } buf;
    #pragma pack(pop)

    const uint8_t id = 34;
    const uint16_t len = sizeof(buf);

    uint8_t *pack() {
        buf.index = index;
        buf.time_sec = time_sec;
        buf.latitude_deg = latitude_deg;
        buf.longitude_deg = longitude_deg;
        buf.altitude_m = altitude_m;
        buf.vn_ms = intround(vn_ms * 100);
        buf.ve_ms = intround(ve_ms * 100);
        buf.vd_ms = intround(vd_ms * 100);
        buf.unixtime_sec = unixtime_sec;
        buf.satellites = satellites;
        buf.horiz_accuracy_m = uintround(horiz_accuracy_m * 100);
        buf.vert_accuracy_m = uintround(vert_accuracy_m * 100);
        buf.pdop = uintround(pdop * 100);
        buf.fix_type = fix_type;
        return (uint8_t *)(&buf);
    }

    void unpack(uint8_t *message) {
        memcpy(&buf, message, len);
        index = buf.index;
        time_sec = buf.time_sec;
        latitude_deg = buf.latitude_deg;
        longitude_deg = buf.longitude_deg;
        altitude_m = buf.altitude_m;
        vn_ms = buf.vn_ms / (float)100;
        ve_ms = buf.ve_ms / (float)100;
        vd_ms = buf.vd_ms / (float)100;
        unixtime_sec = buf.unixtime_sec;
        satellites = buf.satellites;
        horiz_accuracy_m = buf.horiz_accuracy_m / (float)100;
        vert_accuracy_m = buf.vert_accuracy_m / (float)100;
        pdop = buf.pdop / (float)100;
        fix_type = buf.fix_type;
    }
};

