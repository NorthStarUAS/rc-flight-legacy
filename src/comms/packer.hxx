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

    int pack(int index, const char *pack_function, uint8_t *buf);
    int pack_gps(int index, uint8_t *buf);
    int pack_imu(int index, uint8_t *buf);
    int pack_airdata(int index, uint8_t *buf);
    int pack_health(int index, uint8_t *buf);
    int pack_pilot(int index, uint8_t *buf);
    int pack_actuator(int index, uint8_t *buf);
    int pack_filter(int index, uint8_t *buf);
    int pack_payload(int index, uint8_t *buf);
    int pack_ap(int index, uint8_t *buf);
};

#endif // _AURA_PACKER_HXX
