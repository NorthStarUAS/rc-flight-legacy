#ifndef _AURA_PACKER_HXX
#define _AURA_PACKER_HXX

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include "python/pymodule.hxx"
#include "python/pyprops.hxx"

class pyModulePacker: public pyModuleBase {

public:

    // constructor / destructor
    pyModulePacker();
    ~pyModulePacker() {}

    int pack_gps_v1(int index, uint8_t *buf);
    
};

#endif // _AURA_PACKER_HXX
