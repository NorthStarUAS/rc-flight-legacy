#include <string>
using std::string;

#include "util/myprof.h"
#include "drivers/Aura4/Aura4.h"
#include "drivers/fgfs.h"
#include "driver_mgr.h"

driver_mgr_t::driver_mgr_t() {
    drivers.clear();
}

void driver_mgr_t::init() {
    sensors_node = pyGetNode("/sensors", true);
    drivers_node = pyGetNode("/config/drivers", true);
    vector<string> children = drivers_node.getChildren();
    printf("Found %d driver sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
        printf("Initializing device: %s\n", children[i].c_str());
	pyPropertyNode section_node = drivers_node.getChild(children[i].c_str());
        if ( children[i] == "Aura4" ) {
            driver_t *d = new Aura4_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( children[i] == "fgfs" ) {
            driver_t *d = new fgfs_t();
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
