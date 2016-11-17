/*
sensorData.h
Brian R Taylor
brian.taylor@bolderflight.com
2016-11-02
Copyright (c) 2016 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

// sensor data to send to SOC
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */
struct sensors
{
    uint64_t time_us;           // time, us
    float pwr_v;                // input power, v

    float imu_accel_mss[3];     // accelerometer [x,y,z], m/s/s
    float imu_gyro_rads[3];     // gyroscope [x,y,z], rad/s
    float imu_mag_uTesla[3];    // magnetometer [x,y,z], uTesla
    float imu_temp_c;           // imu die temperature, C
    
    uint32_t gps_iTOW;          // GPS time of the navigation epoch, ms
    uint16_t gps_utcYear;       // Year (UTC), year
    uint8_t gps_utcMonth;       // Month, range 1..12 (UTC), month
    uint8_t gps_utcDay;         // Day of month, range 1..31 (UTC), day
    uint8_t gps_utcHour;        // Hour of day, range 0..23 (UTC), hour
    uint8_t gps_utcMin;         // Minute of hour, range 0..59 (UTC), min 
    uint8_t gps_utcSec;         // Seconds of minute, range 0..60 (UTC), s
    uint8_t gps_valid;          // Validity flags
    uint32_t gps_tAcc;          // Time accuracy estimate (UTC), ns
    int32_t gps_utcNano;        // Fraction of second, range -1e9 .. 1e9 (UTC), ns
    uint8_t gps_fixType;        // GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
    uint8_t gps_flags;          // Fix status flags
    uint8_t gps_flags2;         // Additional flags
    uint8_t gps_numSV;          // Number of satellites used in Nav Solution
    double gps_lon_rad;         // Longitude, rad
    double gps_lat_rad;         // Latitude, rad
    double gps_height_m;        // Height above ellipsoid, m
    double gps_hMSL_m;          // Heigh above mean sea level, m
    double gps_hAcc_m;          // Horizontal accuracy estimate, m
    double gps_vAcc_m;          // Vertical accuracy estimate, m
    double gps_velN_mps;        // NED north velocity, m/s
    double gps_velE_mps;        // NED east velocity, m/s
    double gps_velD_mps;        // NED down velocity, m/s
    double gps_sAcc_mps;        // speed accuracy estimate, m/s
    uint8_t gps_pDOP;           // position DOP

    float sbus_channels[14];    // normalized sbus channels
    uint8_t sbus_failSafe;      // bool sbus failsafe
    uint16_t sbus_lostFrames;   // counter number of lost frames
    uint8_t sbus_autoMode;      // bool autopilot enabled (sbus channel 0)
    uint8_t sbus_thrEnable;     // bool throttle enabled (sbus channel 1)
};
#pragma pack(pop)   /* restore original alignment from stack */

#endif
