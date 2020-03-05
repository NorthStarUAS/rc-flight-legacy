#pragma once

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include <pymodule.h>

extern bool display_on;

class pyModuleDisplay: public pyModuleBase {

public:

    // constructor / destructor
    pyModuleDisplay();
    ~pyModuleDisplay() {}

    bool show(const char *message);
    void status_summary();
};
