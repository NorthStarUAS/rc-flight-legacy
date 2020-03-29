#pragma once

#include <pyprops.h>

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include <pymodule.h>

class pyModuleLogging: public pyModuleBase {

public:

    // constructor / destructor
    pyModuleLogging();
    ~pyModuleLogging() {}

    bool open(const char *path);
    void update();
    bool close();

    void log_message( int id, uint8_t *buf, int len );

    void write_configs();
};

// sort of a hack for now, but doing this in pure C let's me pass in a
// property node pointer without having to figure out how to propagate
// that to the python system.
bool write_imu_calibration( pyPropertyNode *config );
