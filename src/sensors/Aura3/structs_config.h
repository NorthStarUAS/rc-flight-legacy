#pragma once

#pragma pack(push,1)            // set alignment to 1 byte boundary

typedef struct {
    uint8_t interface;          // 0 = SPI, 1 = I2C
    uint8_t pin_or_address;     // SPI CS pin or I2C address
    float orientation[9];       // IMU orientation matrix
} config_imu_t;

#pragma pack(pop)
