#ifndef _AURA_REMOTE_LINK_HXX
#define _AURA_REMOTE_LINK_HXX

#include "python/pyprops.hxx"

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include "python/pymodule.hxx"

#include <stdint.h>
#include <string>
#include <vector>

class pyModuleRemoteLink: public pyModuleBase {

public:
    
    // constructor / destructor
    pyModuleRemoteLink();
    ~pyModuleRemoteLink() {}

    // bool open();
    void send_message( uint8_t *buf, int size );
    bool command();
    bool flush_serial();
    bool decode_fcs_update( const char *buf );

private:

    bool remote_link_on;
};

#endif // _AURA_REMOTE_LINK_HXX
