//
// gpsd driver (via generic socket interface to minimize build dependencies)
//

#pragma once

#include <string>
using std::string;

#include <pyprops.h>

#include "comms/display.h"
#include "util/netSocket.h"
#include "util/props_helper.h"
#include "util/timing.h"

#include "drivers/driver.h"

class gpsd_t: public driver_t {
    
public:
    gpsd_t() {}
    ~gpsd_t() {}
    void init( pyPropertyNode *config );
    float read();
    void process() {}
    void write() {}
    void close();
    void command( const char *cmd ) {}

private:
    pyPropertyNode gps_node;
    pyPropertyNode raw_node;
    int port = 2947;
    string host = "localhost";
    string init_string = "?WATCH={\"enable\":true,\"json\":true,\"scaled\":true}";
    netSocket gpsd_sock;
    bool socket_connected = false;
    double last_init_time = 0.0;
    string json_buffer = "";
    void connect();
    void send_init();
    bool process_buffer();
    bool parse_message(const string message);
};
