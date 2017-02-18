#ifndef _AURA_LOGGING_HXX
#define _AURA_LOGGING_HXX

#include "python/pyprops.hxx"

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include "python/pymodule.hxx"

class pyModuleLogging: public pyModuleBase {

public:

    // constructor / destructor
    pyModuleLogging();
    ~pyModuleLogging() {}

    bool open(const char *path);
    void update();
    bool close();

    void log_actuator( uint8_t *buf, int size );
    void log_airdata( uint8_t *buf, int size );
    void log_ap( uint8_t *buf, int size );
    void log_filter( uint8_t *buf, int size );
    void log_gps( uint8_t *buf, int size );
    void log_health( uint8_t *buf, int size );
    void log_imu( uint8_t *buf, int size );
    void log_payload( uint8_t *buf, int size );
    void log_pilot( uint8_t *buf, int size );
    void log_raven( uint8_t *buf, int size );

    void write_configs();
};

// sort of a hack for now, but pure C let's me pass in a property node
// pointer without having to figure out how to propagate that to the
// python system.
bool write_imu_calibration( pyPropertyNode *config );

#endif // _AURA_LOGGING_HXX
