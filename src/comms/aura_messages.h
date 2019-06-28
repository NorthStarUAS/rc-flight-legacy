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
const uint8_t message_gps_v2_id = 16;
const uint8_t message_gps_v3_id = 26;
const uint8_t message_gps_v4_id = 34;
const uint8_t message_imu_v3_id = 17;
const uint8_t message_imu_v4_id = 35;
const uint8_t message_airdata_v5_id = 18;
const uint8_t message_airdata_v6_id = 40;
const uint8_t message_airdata_v7_id = 43;
const uint8_t message_filter_v2_id = 22;
const uint8_t message_filter_v3_id = 31;
const uint8_t message_filter_v4_id = 36;
const uint8_t message_actuator_v2_id = 21;
const uint8_t message_actuator_v3_id = 37;
const uint8_t message_pilot_v2_id = 20;
const uint8_t message_pilot_v3_id = 38;
const uint8_t message_ap_status_v4_id = 30;
const uint8_t message_ap_status_v5_id = 32;
const uint8_t message_ap_status_v6_id = 33;
const uint8_t message_ap_status_v7_id = 39;
const uint8_t message_system_health_v4_id = 19;
const uint8_t message_system_health_v5_id = 41;
const uint8_t message_payload_v2_id = 23;
const uint8_t message_payload_v3_id = 42;

// Message: gps_v2 (id: 16)
struct message_gps_v2_t {
    // public fields
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
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
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
        uint8_t status;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 16;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.latitude_deg = latitude_deg;
        _buf.longitude_deg = longitude_deg;
        _buf.altitude_m = altitude_m;
        _buf.vn_ms = intround(vn_ms * 100);
        _buf.ve_ms = intround(ve_ms * 100);
        _buf.vd_ms = intround(vd_ms * 100);
        _buf.unixtime_sec = unixtime_sec;
        _buf.satellites = satellites;
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        latitude_deg = _buf.latitude_deg;
        longitude_deg = _buf.longitude_deg;
        altitude_m = _buf.altitude_m;
        vn_ms = _buf.vn_ms / (float)100;
        ve_ms = _buf.ve_ms / (float)100;
        vd_ms = _buf.vd_ms / (float)100;
        unixtime_sec = _buf.unixtime_sec;
        satellites = _buf.satellites;
        status = _buf.status;
    }
};

// Message: gps_v3 (id: 26)
struct message_gps_v3_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 26;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.latitude_deg = latitude_deg;
        _buf.longitude_deg = longitude_deg;
        _buf.altitude_m = altitude_m;
        _buf.vn_ms = intround(vn_ms * 100);
        _buf.ve_ms = intround(ve_ms * 100);
        _buf.vd_ms = intround(vd_ms * 100);
        _buf.unixtime_sec = unixtime_sec;
        _buf.satellites = satellites;
        _buf.horiz_accuracy_m = uintround(horiz_accuracy_m * 100);
        _buf.vert_accuracy_m = uintround(vert_accuracy_m * 100);
        _buf.pdop = uintround(pdop * 100);
        _buf.fix_type = fix_type;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        latitude_deg = _buf.latitude_deg;
        longitude_deg = _buf.longitude_deg;
        altitude_m = _buf.altitude_m;
        vn_ms = _buf.vn_ms / (float)100;
        ve_ms = _buf.ve_ms / (float)100;
        vd_ms = _buf.vd_ms / (float)100;
        unixtime_sec = _buf.unixtime_sec;
        satellites = _buf.satellites;
        horiz_accuracy_m = _buf.horiz_accuracy_m / (float)100;
        vert_accuracy_m = _buf.vert_accuracy_m / (float)100;
        pdop = _buf.pdop / (float)100;
        fix_type = _buf.fix_type;
    }
};

// Message: gps_v4 (id: 34)
struct message_gps_v4_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 34;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.latitude_deg = latitude_deg;
        _buf.longitude_deg = longitude_deg;
        _buf.altitude_m = altitude_m;
        _buf.vn_ms = intround(vn_ms * 100);
        _buf.ve_ms = intround(ve_ms * 100);
        _buf.vd_ms = intround(vd_ms * 100);
        _buf.unixtime_sec = unixtime_sec;
        _buf.satellites = satellites;
        _buf.horiz_accuracy_m = uintround(horiz_accuracy_m * 100);
        _buf.vert_accuracy_m = uintround(vert_accuracy_m * 100);
        _buf.pdop = uintround(pdop * 100);
        _buf.fix_type = fix_type;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        latitude_deg = _buf.latitude_deg;
        longitude_deg = _buf.longitude_deg;
        altitude_m = _buf.altitude_m;
        vn_ms = _buf.vn_ms / (float)100;
        ve_ms = _buf.ve_ms / (float)100;
        vd_ms = _buf.vd_ms / (float)100;
        unixtime_sec = _buf.unixtime_sec;
        satellites = _buf.satellites;
        horiz_accuracy_m = _buf.horiz_accuracy_m / (float)100;
        vert_accuracy_m = _buf.vert_accuracy_m / (float)100;
        pdop = _buf.pdop / (float)100;
        fix_type = _buf.fix_type;
    }
};

// Message: imu_v3 (id: 17)
struct message_imu_v3_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
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
    struct {
        uint8_t index;
        double timestamp_sec;
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 17;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.p_rad_sec = p_rad_sec;
        _buf.q_rad_sec = q_rad_sec;
        _buf.r_rad_sec = r_rad_sec;
        _buf.ax_mps_sec = ax_mps_sec;
        _buf.ay_mps_sec = ay_mps_sec;
        _buf.az_mps_sec = az_mps_sec;
        _buf.hx = hx;
        _buf.hy = hy;
        _buf.hz = hz;
        _buf.temp_C = intround(temp_C * 10);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        p_rad_sec = _buf.p_rad_sec;
        q_rad_sec = _buf.q_rad_sec;
        r_rad_sec = _buf.r_rad_sec;
        ax_mps_sec = _buf.ax_mps_sec;
        ay_mps_sec = _buf.ay_mps_sec;
        az_mps_sec = _buf.az_mps_sec;
        hx = _buf.hx;
        hy = _buf.hy;
        hz = _buf.hz;
        temp_C = _buf.temp_C / (float)10;
        status = _buf.status;
    }
};

// Message: imu_v4 (id: 35)
struct message_imu_v4_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 35;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.p_rad_sec = p_rad_sec;
        _buf.q_rad_sec = q_rad_sec;
        _buf.r_rad_sec = r_rad_sec;
        _buf.ax_mps_sec = ax_mps_sec;
        _buf.ay_mps_sec = ay_mps_sec;
        _buf.az_mps_sec = az_mps_sec;
        _buf.hx = hx;
        _buf.hy = hy;
        _buf.hz = hz;
        _buf.temp_C = intround(temp_C * 10);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        p_rad_sec = _buf.p_rad_sec;
        q_rad_sec = _buf.q_rad_sec;
        r_rad_sec = _buf.r_rad_sec;
        ax_mps_sec = _buf.ax_mps_sec;
        ay_mps_sec = _buf.ay_mps_sec;
        az_mps_sec = _buf.az_mps_sec;
        hx = _buf.hx;
        hy = _buf.hy;
        hz = _buf.hz;
        temp_C = _buf.temp_C / (float)10;
        status = _buf.status;
    }
};

// Message: airdata_v5 (id: 18)
struct message_airdata_v5_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
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
    struct {
        uint8_t index;
        double timestamp_sec;
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 18;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.pressure_mbar = uintround(pressure_mbar * 10);
        _buf.temp_C = intround(temp_C * 100);
        _buf.airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100);
        _buf.altitude_smoothed_m = altitude_smoothed_m;
        _buf.altitude_true_m = altitude_true_m;
        _buf.pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600);
        _buf.wind_dir_deg = uintround(wind_dir_deg * 100);
        _buf.wind_speed_kt = uintround(wind_speed_kt * 4);
        _buf.pitot_scale_factor = uintround(pitot_scale_factor * 100);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        pressure_mbar = _buf.pressure_mbar / (float)10;
        temp_C = _buf.temp_C / (float)100;
        airspeed_smoothed_kt = _buf.airspeed_smoothed_kt / (float)100;
        altitude_smoothed_m = _buf.altitude_smoothed_m;
        altitude_true_m = _buf.altitude_true_m;
        pressure_vertical_speed_fps = _buf.pressure_vertical_speed_fps / (float)600;
        wind_dir_deg = _buf.wind_dir_deg / (float)100;
        wind_speed_kt = _buf.wind_speed_kt / (float)4;
        pitot_scale_factor = _buf.pitot_scale_factor / (float)100;
        status = _buf.status;
    }
};

// Message: airdata_v6 (id: 40)
struct message_airdata_v6_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 40;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.pressure_mbar = uintround(pressure_mbar * 10);
        _buf.temp_C = intround(temp_C * 100);
        _buf.airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100);
        _buf.altitude_smoothed_m = altitude_smoothed_m;
        _buf.altitude_true_m = altitude_true_m;
        _buf.pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600);
        _buf.wind_dir_deg = uintround(wind_dir_deg * 100);
        _buf.wind_speed_kt = uintround(wind_speed_kt * 4);
        _buf.pitot_scale_factor = uintround(pitot_scale_factor * 100);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        pressure_mbar = _buf.pressure_mbar / (float)10;
        temp_C = _buf.temp_C / (float)100;
        airspeed_smoothed_kt = _buf.airspeed_smoothed_kt / (float)100;
        altitude_smoothed_m = _buf.altitude_smoothed_m;
        altitude_true_m = _buf.altitude_true_m;
        pressure_vertical_speed_fps = _buf.pressure_vertical_speed_fps / (float)600;
        wind_dir_deg = _buf.wind_dir_deg / (float)100;
        wind_speed_kt = _buf.wind_speed_kt / (float)4;
        pitot_scale_factor = _buf.pitot_scale_factor / (float)100;
        status = _buf.status;
    }
};

// Message: airdata_v7 (id: 43)
struct message_airdata_v7_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 43;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.pressure_mbar = uintround(pressure_mbar * 10);
        _buf.temp_C = intround(temp_C * 100);
        _buf.airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100);
        _buf.altitude_smoothed_m = altitude_smoothed_m;
        _buf.altitude_true_m = altitude_true_m;
        _buf.pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600);
        _buf.wind_dir_deg = uintround(wind_dir_deg * 100);
        _buf.wind_speed_kt = uintround(wind_speed_kt * 4);
        _buf.pitot_scale_factor = uintround(pitot_scale_factor * 100);
        _buf.error_count = error_count;
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        pressure_mbar = _buf.pressure_mbar / (float)10;
        temp_C = _buf.temp_C / (float)100;
        airspeed_smoothed_kt = _buf.airspeed_smoothed_kt / (float)100;
        altitude_smoothed_m = _buf.altitude_smoothed_m;
        altitude_true_m = _buf.altitude_true_m;
        pressure_vertical_speed_fps = _buf.pressure_vertical_speed_fps / (float)600;
        wind_dir_deg = _buf.wind_dir_deg / (float)100;
        wind_speed_kt = _buf.wind_speed_kt / (float)4;
        pitot_scale_factor = _buf.pitot_scale_factor / (float)100;
        error_count = _buf.error_count;
        status = _buf.status;
    }
};

// Message: filter_v2 (id: 22)
struct message_filter_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        double timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        uint8_t sequence_num;
        uint8_t status;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 22;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.latitude_deg = latitude_deg;
        _buf.longitude_deg = longitude_deg;
        _buf.altitude_m = altitude_m;
        _buf.vn_ms = intround(vn_ms * 100);
        _buf.ve_ms = intround(ve_ms * 100);
        _buf.vd_ms = intround(vd_ms * 100);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.yaw_deg = intround(yaw_deg * 10);
        _buf.sequence_num = sequence_num;
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        latitude_deg = _buf.latitude_deg;
        longitude_deg = _buf.longitude_deg;
        altitude_m = _buf.altitude_m;
        vn_ms = _buf.vn_ms / (float)100;
        ve_ms = _buf.ve_ms / (float)100;
        vd_ms = _buf.vd_ms / (float)100;
        roll_deg = _buf.roll_deg / (float)10;
        pitch_deg = _buf.pitch_deg / (float)10;
        yaw_deg = _buf.yaw_deg / (float)10;
        sequence_num = _buf.sequence_num;
        status = _buf.status;
    }
};

// Message: filter_v3 (id: 31)
struct message_filter_v3_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
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
    struct {
        uint8_t index;
        double timestamp_sec;
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 31;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.latitude_deg = latitude_deg;
        _buf.longitude_deg = longitude_deg;
        _buf.altitude_m = altitude_m;
        _buf.vn_ms = intround(vn_ms * 100);
        _buf.ve_ms = intround(ve_ms * 100);
        _buf.vd_ms = intround(vd_ms * 100);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.yaw_deg = intround(yaw_deg * 10);
        _buf.p_bias = intround(p_bias * 10000);
        _buf.q_bias = intround(q_bias * 10000);
        _buf.r_bias = intround(r_bias * 10000);
        _buf.ax_bias = intround(ax_bias * 1000);
        _buf.ay_bias = intround(ay_bias * 1000);
        _buf.az_bias = intround(az_bias * 1000);
        _buf.sequence_num = sequence_num;
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        latitude_deg = _buf.latitude_deg;
        longitude_deg = _buf.longitude_deg;
        altitude_m = _buf.altitude_m;
        vn_ms = _buf.vn_ms / (float)100;
        ve_ms = _buf.ve_ms / (float)100;
        vd_ms = _buf.vd_ms / (float)100;
        roll_deg = _buf.roll_deg / (float)10;
        pitch_deg = _buf.pitch_deg / (float)10;
        yaw_deg = _buf.yaw_deg / (float)10;
        p_bias = _buf.p_bias / (float)10000;
        q_bias = _buf.q_bias / (float)10000;
        r_bias = _buf.r_bias / (float)10000;
        ax_bias = _buf.ax_bias / (float)1000;
        ay_bias = _buf.ay_bias / (float)1000;
        az_bias = _buf.az_bias / (float)1000;
        sequence_num = _buf.sequence_num;
        status = _buf.status;
    }
};

// Message: filter_v4 (id: 36)
struct message_filter_v4_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 36;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.latitude_deg = latitude_deg;
        _buf.longitude_deg = longitude_deg;
        _buf.altitude_m = altitude_m;
        _buf.vn_ms = intround(vn_ms * 100);
        _buf.ve_ms = intround(ve_ms * 100);
        _buf.vd_ms = intround(vd_ms * 100);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.yaw_deg = intround(yaw_deg * 10);
        _buf.p_bias = intround(p_bias * 10000);
        _buf.q_bias = intround(q_bias * 10000);
        _buf.r_bias = intround(r_bias * 10000);
        _buf.ax_bias = intround(ax_bias * 1000);
        _buf.ay_bias = intround(ay_bias * 1000);
        _buf.az_bias = intround(az_bias * 1000);
        _buf.sequence_num = sequence_num;
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        latitude_deg = _buf.latitude_deg;
        longitude_deg = _buf.longitude_deg;
        altitude_m = _buf.altitude_m;
        vn_ms = _buf.vn_ms / (float)100;
        ve_ms = _buf.ve_ms / (float)100;
        vd_ms = _buf.vd_ms / (float)100;
        roll_deg = _buf.roll_deg / (float)10;
        pitch_deg = _buf.pitch_deg / (float)10;
        yaw_deg = _buf.yaw_deg / (float)10;
        p_bias = _buf.p_bias / (float)10000;
        q_bias = _buf.q_bias / (float)10000;
        r_bias = _buf.r_bias / (float)10000;
        ax_bias = _buf.ax_bias / (float)1000;
        ay_bias = _buf.ay_bias / (float)1000;
        az_bias = _buf.az_bias / (float)1000;
        sequence_num = _buf.sequence_num;
        status = _buf.status;
    }
};

// Message: actuator_v2 (id: 21)
struct message_actuator_v2_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 21;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.aileron = intround(aileron * 20000);
        _buf.elevator = intround(elevator * 20000);
        _buf.throttle = uintround(throttle * 60000);
        _buf.rudder = intround(rudder * 20000);
        _buf.channel5 = intround(channel5 * 20000);
        _buf.flaps = intround(flaps * 20000);
        _buf.channel7 = intround(channel7 * 20000);
        _buf.channel8 = intround(channel8 * 20000);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        aileron = _buf.aileron / (float)20000;
        elevator = _buf.elevator / (float)20000;
        throttle = _buf.throttle / (float)60000;
        rudder = _buf.rudder / (float)20000;
        channel5 = _buf.channel5 / (float)20000;
        flaps = _buf.flaps / (float)20000;
        channel7 = _buf.channel7 / (float)20000;
        channel8 = _buf.channel8 / (float)20000;
        status = _buf.status;
    }
};

// Message: actuator_v3 (id: 37)
struct message_actuator_v3_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 37;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.aileron = intround(aileron * 20000);
        _buf.elevator = intround(elevator * 20000);
        _buf.throttle = uintround(throttle * 60000);
        _buf.rudder = intround(rudder * 20000);
        _buf.channel5 = intround(channel5 * 20000);
        _buf.flaps = intround(flaps * 20000);
        _buf.channel7 = intround(channel7 * 20000);
        _buf.channel8 = intround(channel8 * 20000);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        aileron = _buf.aileron / (float)20000;
        elevator = _buf.elevator / (float)20000;
        throttle = _buf.throttle / (float)60000;
        rudder = _buf.rudder / (float)20000;
        channel5 = _buf.channel5 / (float)20000;
        flaps = _buf.flaps / (float)20000;
        channel7 = _buf.channel7 / (float)20000;
        channel8 = _buf.channel8 / (float)20000;
        status = _buf.status;
    }
};

// Message: pilot_v2 (id: 20)
struct message_pilot_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float channel[8];
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        double timestamp_sec;
        int16_t channel[8];
        uint8_t status;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 20;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        for (int _i=0; _i<8; _i++) _buf.channel[_i] = intround(channel[_i] * 20000);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf.channel[_i] / (float)20000;
        status = _buf.status;
    }
};

// Message: pilot_v3 (id: 38)
struct message_pilot_v3_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float channel[8];
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        float timestamp_sec;
        int16_t channel[8];
        uint8_t status;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 38;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        for (int _i=0; _i<8; _i++) _buf.channel[_i] = intround(channel[_i] * 20000);
        _buf.status = status;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf.channel[_i] / (float)20000;
        status = _buf.status;
    }
};

// Message: ap_status_v4 (id: 30)
struct message_ap_status_v4_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
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
    uint8_t sequence_number;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        double timestamp_sec;
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
        uint8_t sequence_number;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 30;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.groundtrack_deg = intround(groundtrack_deg * 10);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.altitude_msl_ft = altitude_msl_ft;
        _buf.altitude_ground_m = altitude_ground_m;
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.airspeed_kt = intround(airspeed_kt * 10);
        _buf.flight_timer = flight_timer;
        _buf.target_waypoint_idx = target_waypoint_idx;
        _buf.wp_longitude_deg = wp_longitude_deg;
        _buf.wp_latitude_deg = wp_latitude_deg;
        _buf.wp_index = wp_index;
        _buf.route_size = route_size;
        _buf.sequence_number = sequence_number;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        groundtrack_deg = _buf.groundtrack_deg / (float)10;
        roll_deg = _buf.roll_deg / (float)10;
        altitude_msl_ft = _buf.altitude_msl_ft;
        altitude_ground_m = _buf.altitude_ground_m;
        pitch_deg = _buf.pitch_deg / (float)10;
        airspeed_kt = _buf.airspeed_kt / (float)10;
        flight_timer = _buf.flight_timer;
        target_waypoint_idx = _buf.target_waypoint_idx;
        wp_longitude_deg = _buf.wp_longitude_deg;
        wp_latitude_deg = _buf.wp_latitude_deg;
        wp_index = _buf.wp_index;
        route_size = _buf.route_size;
        sequence_number = _buf.sequence_number;
    }
};

// Message: ap_status_v5 (id: 32)
struct message_ap_status_v5_t {
    // public fields
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
    uint8_t sequence_number;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
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
        uint8_t sequence_number;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 32;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.flags = flags;
        _buf.groundtrack_deg = intround(groundtrack_deg * 10);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.altitude_msl_ft = altitude_msl_ft;
        _buf.altitude_ground_m = altitude_ground_m;
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.airspeed_kt = intround(airspeed_kt * 10);
        _buf.flight_timer = flight_timer;
        _buf.target_waypoint_idx = target_waypoint_idx;
        _buf.wp_longitude_deg = wp_longitude_deg;
        _buf.wp_latitude_deg = wp_latitude_deg;
        _buf.wp_index = wp_index;
        _buf.route_size = route_size;
        _buf.sequence_number = sequence_number;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        flags = _buf.flags;
        groundtrack_deg = _buf.groundtrack_deg / (float)10;
        roll_deg = _buf.roll_deg / (float)10;
        altitude_msl_ft = _buf.altitude_msl_ft;
        altitude_ground_m = _buf.altitude_ground_m;
        pitch_deg = _buf.pitch_deg / (float)10;
        airspeed_kt = _buf.airspeed_kt / (float)10;
        flight_timer = _buf.flight_timer;
        target_waypoint_idx = _buf.target_waypoint_idx;
        wp_longitude_deg = _buf.wp_longitude_deg;
        wp_latitude_deg = _buf.wp_latitude_deg;
        wp_index = _buf.wp_index;
        route_size = _buf.route_size;
        sequence_number = _buf.sequence_number;
    }
};

// Message: ap_status_v6 (id: 33)
struct message_ap_status_v6_t {
    // public fields
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
    uint8_t sequence_number;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
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
        uint8_t sequence_number;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 33;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.flags = flags;
        _buf.groundtrack_deg = intround(groundtrack_deg * 10);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.altitude_msl_ft = altitude_msl_ft;
        _buf.altitude_ground_m = altitude_ground_m;
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.airspeed_kt = intround(airspeed_kt * 10);
        _buf.flight_timer = flight_timer;
        _buf.target_waypoint_idx = target_waypoint_idx;
        _buf.wp_longitude_deg = wp_longitude_deg;
        _buf.wp_latitude_deg = wp_latitude_deg;
        _buf.wp_index = wp_index;
        _buf.route_size = route_size;
        _buf.task_id = task_id;
        _buf.task_attribute = task_attribute;
        _buf.sequence_number = sequence_number;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        flags = _buf.flags;
        groundtrack_deg = _buf.groundtrack_deg / (float)10;
        roll_deg = _buf.roll_deg / (float)10;
        altitude_msl_ft = _buf.altitude_msl_ft;
        altitude_ground_m = _buf.altitude_ground_m;
        pitch_deg = _buf.pitch_deg / (float)10;
        airspeed_kt = _buf.airspeed_kt / (float)10;
        flight_timer = _buf.flight_timer;
        target_waypoint_idx = _buf.target_waypoint_idx;
        wp_longitude_deg = _buf.wp_longitude_deg;
        wp_latitude_deg = _buf.wp_latitude_deg;
        wp_index = _buf.wp_index;
        route_size = _buf.route_size;
        task_id = _buf.task_id;
        task_attribute = _buf.task_attribute;
        sequence_number = _buf.sequence_number;
    }
};

// Message: ap_status_v7 (id: 39)
struct message_ap_status_v7_t {
    // public fields
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
    struct {
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
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 39;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.flags = flags;
        _buf.groundtrack_deg = intround(groundtrack_deg * 10);
        _buf.roll_deg = intround(roll_deg * 10);
        _buf.altitude_msl_ft = uintround(altitude_msl_ft * 1);
        _buf.altitude_ground_m = uintround(altitude_ground_m * 1);
        _buf.pitch_deg = intround(pitch_deg * 10);
        _buf.airspeed_kt = intround(airspeed_kt * 10);
        _buf.flight_timer = uintround(flight_timer * 1);
        _buf.target_waypoint_idx = target_waypoint_idx;
        _buf.wp_longitude_deg = wp_longitude_deg;
        _buf.wp_latitude_deg = wp_latitude_deg;
        _buf.wp_index = wp_index;
        _buf.route_size = route_size;
        _buf.task_id = task_id;
        _buf.task_attribute = task_attribute;
        _buf.sequence_num = sequence_num;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        flags = _buf.flags;
        groundtrack_deg = _buf.groundtrack_deg / (float)10;
        roll_deg = _buf.roll_deg / (float)10;
        altitude_msl_ft = _buf.altitude_msl_ft / (float)1;
        altitude_ground_m = _buf.altitude_ground_m / (float)1;
        pitch_deg = _buf.pitch_deg / (float)10;
        airspeed_kt = _buf.airspeed_kt / (float)10;
        flight_timer = _buf.flight_timer / (float)1;
        target_waypoint_idx = _buf.target_waypoint_idx;
        wp_longitude_deg = _buf.wp_longitude_deg;
        wp_latitude_deg = _buf.wp_latitude_deg;
        wp_index = _buf.wp_index;
        route_size = _buf.route_size;
        task_id = _buf.task_id;
        task_attribute = _buf.task_attribute;
        sequence_num = _buf.sequence_num;
    }
};

// Message: system_health_v4 (id: 19)
struct message_system_health_v4_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float system_load_avg;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        double timestamp_sec;
        uint16_t system_load_avg;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 19;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.system_load_avg = uintround(system_load_avg * 100);
        _buf.avionics_vcc = uintround(avionics_vcc * 1000);
        _buf.main_vcc = uintround(main_vcc * 1000);
        _buf.cell_vcc = uintround(cell_vcc * 1000);
        _buf.main_amps = uintround(main_amps * 1000);
        _buf.total_mah = uintround(total_mah * 10);
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        system_load_avg = _buf.system_load_avg / (float)100;
        avionics_vcc = _buf.avionics_vcc / (float)1000;
        main_vcc = _buf.main_vcc / (float)1000;
        cell_vcc = _buf.cell_vcc / (float)1000;
        main_amps = _buf.main_amps / (float)1000;
        total_mah = _buf.total_mah / (float)10;
    }
};

// Message: system_health_v5 (id: 41)
struct message_system_health_v5_t {
    // public fields
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
    struct {
        uint8_t index;
        float timestamp_sec;
        uint16_t system_load_avg;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 41;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.system_load_avg = uintround(system_load_avg * 100);
        _buf.avionics_vcc = uintround(avionics_vcc * 1000);
        _buf.main_vcc = uintround(main_vcc * 1000);
        _buf.cell_vcc = uintround(cell_vcc * 1000);
        _buf.main_amps = uintround(main_amps * 1000);
        _buf.total_mah = uintround(total_mah * 10);
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        system_load_avg = _buf.system_load_avg / (float)100;
        avionics_vcc = _buf.avionics_vcc / (float)1000;
        main_vcc = _buf.main_vcc / (float)1000;
        cell_vcc = _buf.cell_vcc / (float)1000;
        main_amps = _buf.main_amps / (float)1000;
        total_mah = _buf.total_mah / (float)10;
    }
};

// Message: payload_v2 (id: 23)
struct message_payload_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    uint16_t trigger_num;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        double timestamp_sec;
        uint16_t trigger_num;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 23;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.trigger_num = trigger_num;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        trigger_num = _buf.trigger_num;
    }
};

// Message: payload_v3 (id: 42)
struct message_payload_v3_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    uint16_t trigger_num;

    // internal structure for packing
    #pragma pack(push, 1)
    struct {
        uint8_t index;
        float timestamp_sec;
        uint16_t trigger_num;
    } _buf;
    #pragma pack(pop)

    static const uint8_t id = 42;
    static const uint16_t len = sizeof(_buf);
    uint8_t *payload = NULL;

    uint8_t *pack() {
        _buf.index = index;
        _buf.timestamp_sec = timestamp_sec;
        _buf.trigger_num = trigger_num;
        payload = (uint8_t *)(&_buf);
        return payload;
    }

    void unpack(uint8_t *message) {
        memcpy(&_buf, message, len);
        index = _buf.index;
        timestamp_sec = _buf.timestamp_sec;
        trigger_num = _buf.trigger_num;
    }
};

