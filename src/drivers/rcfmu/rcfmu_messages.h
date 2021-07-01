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
const uint8_t aura_nav_pvt_id = 18;
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
};

// Message: imu (id: 17)
class imu_t {
public:

    uint32_t millis;
    int16_t raw[6];
    int16_t cal[10];

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t raw[6];
        int16_t cal[10];
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
        for (int _i=0; _i<6; _i++) _buf->raw[_i] = raw[_i];
        for (int _i=0; _i<10; _i++) _buf->cal[_i] = cal[_i];
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        for (int _i=0; _i<6; _i++) raw[_i] = _buf->raw[_i];
        for (int _i=0; _i<10; _i++) cal[_i] = _buf->cal[_i];
        return true;
    }
};

// Message: aura_nav_pvt (id: 18)
class aura_nav_pvt_t {
public:

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

    // id, ptr to payload and len
    static const uint8_t id = 18;
    uint8_t *payload = nullptr;
    int len = 0;

    ~aura_nav_pvt_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
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
        _compact_t *_buf = (_compact_t *)external_message;
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
};

} // namespace rcfmu_message
