#pragma once

#include "setup_pwm.h"
#include "setup_sbus.h"

#include "structs_config.h"

#pragma pack(push, 1)           // set alignment to 1 byte boundary

// ack
typedef struct {
    uint8_t command_id;
    uint8_t subcommand_id;
} ack_packet_t;

// ublox8 nav_pvt structure (copied from aura-sensors/src/UBLOX8/UBLOX8.h)
typedef struct {
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
} aura_nav_pvt_t;

// pilot input (from RC system)
typedef struct {
    int16_t channel[SBUS_CHANNELS]; // normalized: [-16384, 16384]
    uint8_t flags;
} pilot_packet_t;

// imu
typedef struct {
    uint32_t micros;
    int16_t channel[10];
} imu_packet_t;

// airdata
typedef struct {
    float baro_press_pa;
    float baro_temp_C;
    float baro_hum;
    float ext_diff_press_pa;
    float ext_static_press_pa;
    float ext_temp_C;
} airdata_packet_t;

// power
typedef struct {
    uint16_t int_main_v;
    uint16_t avionics_v;
    uint16_t ext_main_v;
    uint16_t ext_main_amp;
} power_packet_t;

// status
typedef struct {
    uint16_t serial_number;
    uint16_t firmware_rev;
    uint16_t master_hz;
    uint32_t baud;
    uint16_t byte_rate;
} status_packet_t;

#pragma pack(pop)
