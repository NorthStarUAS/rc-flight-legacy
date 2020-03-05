#pragma once

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include <pymodule.h>

class pyModuleEventLog: public pyModuleBase {

public:

    // constructor / destructor
    pyModuleEventLog();
    ~pyModuleEventLog() {}

    bool log(const char *header, const char *message);
};
