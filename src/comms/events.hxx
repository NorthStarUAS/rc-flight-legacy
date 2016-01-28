#ifndef _AURA_EVENTS_HXX
#define _AURA_EVENTS_HXX

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include "python/pymodule.hxx"

class pyModuleEventLog: public pyModuleBase {

public:

    // constructor / destructor
    pyModuleEventLog();
    ~pyModuleEventLog() {}

    bool open(const char *path);
    bool log(const char *header, const char *message);
    bool close();
    
};

#endif // _AURA_EVENTS_HXX