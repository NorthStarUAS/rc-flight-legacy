//
// globals.cxx - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#include "globals.hxx"


pyModuleEventLog *events = NULL;
pyModuleLogging *logging = NULL;
pyModulePacker *packer = NULL;
UGTelnet *telnet = NULL;
pyModuleBase *mission_mgr = NULL;


bool AuraCoreInit() {
    // create instances
    events = new pyModuleEventLog;
    logging = new pyModuleLogging;
    packer = new pyModulePacker;
    mission_mgr = new pyModuleBase;

    // import and init the python modules
    events->init("comms.events");
    logging->init("comms.logging");
    packer->init("comms.packer");
    mission_mgr->init("mission.mission_mgr");
    
    return true;
}
