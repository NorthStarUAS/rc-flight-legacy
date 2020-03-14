#include <string>
using std::string;

#include "driver_mgr.h"

driver_mgr_t::driver_mgr_t() {
    drivers.clear();
}

void driver_mgr_t::init() {
    config_node = pyGetNode("/config/drivers", true);
    vector<string> children = config_node.getChildren();
    printf("Found %d driver sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode driver_node = config_node.getChild(children[i].c_str());
    }
}

void driver_mgr_t::update() {
}

// global shared instance
driver_mgr_t driver_mgr;
