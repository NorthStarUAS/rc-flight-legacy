#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
using namespace Eigen;

struct GNSS_raw_measurement
{
    double AODO;          // the age of data offset, in seconds.
    double Cic;           // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    double Cis;           // Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination
    double Crc;           // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    double Crs;           // Amplitude of the Sine Harmonic Correction Term to the Orbit Radius
    double Cuc;           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    double Cus;           // Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude
    bool FIT;             // ?
    double IDOT;          // Rate of Inclination Angle
    int IODC;             // Issue of Data, Clock (10 Bits) [Units: N/A]
    int IODE;             // Issue of Data (8 Bits) [Units: N/A]
    int L2;               // Code on L2 (2 Bits) [Units: N/A]
    int L2P;              // L2 P Data Flag (1 Bit) [Units: Discrete]
    double M0;            // Mean Anomaly at Reference Time
    double Omega0;        // Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    double Omegad;        // Rate of Right Ascension
    int TOW17;            // Time-of-week
    double Tgd;           // (8 Bits / Two's Complement with sign on MSB) [Units: Seconds]
    double WN;            // GPS Week Number (10 Bits) [units: Week]
    double af0;           // (22 Bits / Two's Complement with sign on MSB) [Units: Seconds]
    double af1;           // (16 Bits / Two's Complement with sign on MSB) [Units: Sec/Sec]
    double af2;           // (8 Bits / Two's Complement with sign on MSB) [Units: Sec/Sec^2]
    int constellation;    // type of constellation, e.g., 0=GPS, GLONASS, BEIDOU
    double deltan;        // Mean Motion Difference From Computed Value
    double doppler;       // Doppler shift measurement, note: by multiplying frequency, we get psedu-range rate
    double e;             // Eccentricity
    bool frame1;          // Validity of subframe 1
    bool frame2;          // Validity of subframe 2
    bool frame3;          // Validity of subframe 3
    int gnssid;           // Satellite ID number
    int hlth;             // Satellite Vehicle Health (6 Bits) [Units: Discretes]
    double i0;            // Inclination at Reference Time
    double omega;         // Argument of Perigee
    double pseudorange;   // pseudorange (m)
    double sqrtA;         // Square Root off the Semi-Major Axis
    double timestamp;     // current seconds
    double toc;           // (16 Bits) [Units: Seconds]
    double toe;           // Reference Time Ephemeris
    int ura;              // Satellite Vehicle Accuracy (4 Bits) [Units: N/A], binary
};

// Calculate the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d E2D( Vector3d p_E );

// EphemerisData (subframe1,2,3) to Satellite ecef x, y, z in meter, vx, vy, vz in m/s
VectorXd EphemerisData2Satecef(float t,
                               uint32_t TOW, uint8_t L2, uint16_t week_No, uint8_t L2_Flag, uint8_t SV_Acc, uint8_t SV_Hlth,
                               double T_GD, uint16_t IODC, double t_OC, int8_t a_f2, double a_f1, double a_f0,
                               uint8_t IODE, double C_rs, double delta_n, double M_0, double C_uc, double ecc, double C_us,
                               double sqrt_A, double t_OE, double C_ic, double Omega_0, double C_is, double i_0, double C_rc,
                               double omega, double Omega_dot, double IDOT);

// process raw measurement to give a 8 x 1 matrix (range, range rate, x,y,z,vx,vy,vz)
VectorXd EphemerisData2PosVelClock(GNSS_raw_measurement gnss_raw_measurement);

// compute vehicle's postion, velocity in E frame and clock offset (m) and drift (m/s) 
void GNSS_LS_pos_vel(MatrixXd &gnss_measurement,
                     Vector3d &pEst_E_m_, Vector3d &vEst_E_mps_, double &clockBias_m_, double &clockRateBias_mps_);

// if we know our position, try to find a clock bias that minimizes the difference between pseudorange and geometric range
void GNSS_clock_bias(MatrixXd &gnss_measurement, Vector3d &pos_true);
