//
// FILE: FGFS.h
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "props2.h"

#include "drivers/driver.h"
#include "util/netSocket.h"

class fgfs_t: public driver_t {
    
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    fgfs_t() {}
    ~fgfs_t() {}
    void init( PropertyNode *config );
    float read();
    void process() {}
    void write();
    void close();
    void command( const char *cmd ) {}
    
private:
    PropertyNode act_node;
    PropertyNode airdata_node;
    PropertyNode gps_node;
    PropertyNode imu_node;
    PropertyNode orient_node;
    PropertyNode pos_node;
    PropertyNode power_node;
    PropertyNode route_node;
    PropertyNode targets_node;

    netSocket sock_act;
    netSocket sock_imu;
    netSocket sock_gps;

    Vector3f mag_ned;
    Quaternionf q_N2B;
    Matrix3f C_N2B;
    
    int battery_cells = 4;
    
    void info( const char* format, ... );
    void hard_error( const char*format, ... );
    
    void init_act( PropertyNode *config );
    void init_airdata( PropertyNode *config );
    void init_gps( PropertyNode *config );
    void init_imu( PropertyNode *config );
    bool update_gps();
    bool update_imu();
};
