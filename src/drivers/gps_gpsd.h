//
// gpsd driver (via generic socket interface to minimize build dependencies)
//

#pragma once

#include <string>
using std::string;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include <props2.h>

#include "util/netSocket.h"
#include "util/props_helper.h"
#include "util/timing.h"

#include "drivers/driver.h"

class gpsd_t: public driver_t {
    
public:
    gpsd_t() {
        pEst_E_m = Vector3d(0, 0, 0);
        vEst_E_mps = Vector3d(0, 0, 0);
    }
    ~gpsd_t() {}
    void init( PropertyNode *config );
    float read();
    void process() {}
    void write() {}
    void close();
    void command( const char *cmd ) {}

private:
    PropertyNode gps_node;
    PropertyNode raw_node;
    PropertyNode ephem_node;
    int port = 2947;
    string host = "localhost";
    string init_string = "?WATCH={\"enable\":true,\"json\":true,\"scaled\":true}";
    Vector3d pEst_E_m, vEst_E_mps;
    double clockBiasEst_m=0;

    netSocket gpsd_sock;
    bool socket_connected = false;
    double last_init_time = 0.0;
    string json_buffer = "";
    double ephem_write_time = 0;
    double leapseconds = 0;
    void connect();
    void send_init();
    bool process_buffer();
    bool parse_message(const string message);
    VectorXd dump_sat_pos(int svid, double tow, double pr, double doppler,
                          PropertyNode ephem);
};
