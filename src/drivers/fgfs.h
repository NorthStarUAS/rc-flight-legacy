//
// FILE: FGFS.h
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "drivers/driver.h"
#include "util/netSocket.h"

class fgfs_t: public driver_t {
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    fgfs_t() {}
    ~fgfs_t() {}
    void init( pyPropertyNode *config );
    float read();
    void process() {}
    void write();
    void close();
    void command( const char *cmd ) {}
    
private:
    pyPropertyNode act_node;
    pyPropertyNode airdata_node;
    pyPropertyNode gps_node;
    pyPropertyNode imu_node;
    pyPropertyNode orient_node;
    pyPropertyNode pos_node;
    pyPropertyNode power_node;
    pyPropertyNode route_node;
    pyPropertyNode targets_node;

    netSocket sock_act;
    netSocket sock_imu;
    netSocket sock_gps;

    Vector3f mag_ned;
    Quaternionf q_N2B;
    Matrix3f C_N2B;
    
    int battery_cells = 4;
    
    void info( const char* format, ... );
    void hard_error( const char*format, ... );
    
    void init_act( pyPropertyNode *config );
    void init_airdata( pyPropertyNode *config );
    void init_gps( pyPropertyNode *config );
    void init_imu( pyPropertyNode *config );
    bool update_gps();
    bool update_imu();
};
