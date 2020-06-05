#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "util/myprof.h"
#include "drivers/Aura4/Aura4.h"
#include "drivers/fgfs.h"
#include "drivers/lightware.h"
#include "drivers/maestro.h"
#include "drivers/gps_gpsd.h"
#include "drivers/ublox8.h"
#include "drivers/ublox9.h"
#include "driver_mgr.h"

driver_mgr_t::driver_mgr_t() {
    drivers.clear();
}

void driver_mgr_t::init() {
    sensors_node = pyGetNode("/sensors", true);
    pyPropertyNode config_node = pyGetNode("/config", true);
    unsigned int len = config_node.getLen("drivers");
    printf("Found %d driver sections\n", len);
    for ( unsigned int i = 0; i < len; i++ ) {
        ostringstream child;
        child << "drivers[" << i << "]";
        printf("Initializing device: %s\n", child.str().c_str());
	pyPropertyNode driver_node = config_node.getChild(child.str().c_str());
        if ( driver_node.hasChild("Aura4") ) {
            pyPropertyNode section_node = driver_node.getChild("Aura4");
            driver_t *d = new Aura4_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("fgfs") ) {
            pyPropertyNode section_node = driver_node.getChild("fgfs");
            driver_t *d = new fgfs_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("lightware") ) {
            pyPropertyNode section_node = driver_node.getChild("lightware");
            driver_t *d = new lightware_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("maestro") ) {
            pyPropertyNode section_node = driver_node.getChild("maestro");
            driver_t *d = new maestro_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("ublox8") ) {
            pyPropertyNode section_node = driver_node.getChild("ublox8");
            driver_t *d = new ublox8_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("gpsd") ) {
            pyPropertyNode section_node = driver_node.getChild("gpsd");
            driver_t *d = new gpsd_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("ublox9") ) {
            pyPropertyNode section_node = driver_node.getChild("ublox9");
            driver_t *d = new ublox9_t();
            d->init(&section_node);
            drivers.push_back(d);
        }
    }
}

float driver_mgr_t::read() {
    driver_prof.start();
    float master_dt = 0.0;
    for ( unsigned int i = 0; i < drivers.size(); i++ ) {
        float dt = drivers[i]->read();
        if (i == 0) {
            master_dt = dt;
        }
    }
    driver_prof.stop();
    return master_dt;
}

void driver_mgr_t::process() {
    for ( unsigned int i = 0; i < drivers.size(); i++ ) {
        drivers[i]->process();
    }
}

void driver_mgr_t::write() {
    for ( unsigned int i = 0; i < drivers.size(); i++ ) {
        drivers[i]->write();
    }
}

void driver_mgr_t::close() {
    for ( unsigned int i = 0; i < drivers.size(); i++ ) {
        drivers[i]->close();
    }
}

void driver_mgr_t::send_commands() {
    // check for and respond to an airdata calibrate request
    if (sensors_node.getBool("airdata_calibrate") ) {
	sensors_node.setBool("airdata_calibrate", false);
        for ( unsigned int i = 0; i < drivers.size(); i++ ) {
            drivers[i]->command("airdata_calibrate");
        }
    }
}

// global shared instance
driver_mgr_t driver_mgr;
