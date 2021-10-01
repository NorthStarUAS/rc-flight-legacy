#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <string>
using std::string;

#include <python_sys.h>

#include "drivers/Aura4/Aura4.h"
#include "drivers/rcfmu/rcfmu.h"
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

void driver_mgr_t::init(SharedStateWrapper d) {
    // Initialize Document for props2
    printf("set document: %p\n", d.doc);
    PropertyNode("/").set_shared_state(d);
    
    sensors_node = PropertyNode( "/sensors" );
    PropertyNode config_node( "/config" );
    unsigned int len = config_node.getLen("drivers");
    printf("Found %d driver sections\n", len);
    for ( unsigned int i = 0; i < len; i++ ) {
        string child = "drivers/" + std::to_string(i);
        printf("Initializing device: %s\n", child.c_str());
	PropertyNode driver_node = config_node.getChild( child.c_str() );
        if ( driver_node.hasChild("Aura4") ) {
            PropertyNode section_node = driver_node.getChild( "Aura4" );
            driver_t *d = new Aura4_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("rcfmu") ) {
            PropertyNode section_node = driver_node.getChild( "rcfmu" );
            driver_t *d = new rcfmu_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("fgfs") ) {
            PropertyNode section_node = driver_node.getChild( "fgfs" );
            driver_t *d = new fgfs_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("lightware") ) {
            PropertyNode section_node = driver_node.getChild( "lightware" );
            driver_t *d = new lightware_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("maestro") ) {
            PropertyNode section_node = driver_node.getChild( "maestro" );
            driver_t *d = new maestro_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("ublox8") ) {
            PropertyNode section_node = driver_node.getChild( "ublox8" );
            driver_t *d = new ublox8_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("gpsd") ) {
            printf("initializing gpsd\n");
            PropertyNode section_node = driver_node.getChild( "gpsd" );
            driver_t *d = new gpsd_t();
            d->init(&section_node);
            drivers.push_back(d);
        } else if ( driver_node.hasChild("ublox9") ) {
            PropertyNode section_node = driver_node.getChild( "ublox9" );
            driver_t *d = new ublox9_t();
            d->init(&section_node);
            drivers.push_back(d);
        }
    }
}

float driver_mgr_t::read() {
    float master_dt = 0.0;
    for ( unsigned int i = 0; i < drivers.size(); i++ ) {
        float dt = drivers[i]->read();
        if (i == 0) {
            master_dt = dt;
        }
    }
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

PYBIND11_MODULE(driver_mgr, m) {
    py::class_<driver_mgr_t>(m, "driver_mgr")
        .def(py::init<>())
        .def("init", &driver_mgr_t::init)
        .def("read", &driver_mgr_t::read)
        .def("process", &driver_mgr_t::process)
        .def("write", &driver_mgr_t::write)
        .def("close", &driver_mgr_t::close)
        .def("send_commands", &driver_mgr_t::send_commands)
    ;
}
