#include <string>
using std::string;

#include "drivers/Aura4/Aura4.h"
#include "driver_mgr.h"

driver_mgr_t::driver_mgr_t() {
    drivers.clear();
}

void driver_mgr_t::init() {
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
        }
    }
}

void driver_mgr_t::read() {
    for ( unsigned int i = 0; i < drivers.size(); i++ ) {
        drivers[i]->read();
    }
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

// global shared instance
driver_mgr_t driver_mgr;