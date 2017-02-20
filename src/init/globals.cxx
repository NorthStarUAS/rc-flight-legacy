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
pyModuleRemoteLink *remote_link = NULL;
UGTelnet *telnet = NULL;
pyModuleBase *mission_mgr = NULL;


bool AuraCoreInit() {
    // create instances
    events = new pyModuleEventLog;
    logging = new pyModuleLogging;
    packer = new pyModulePacker;
    remote_link = new pyModuleRemoteLink;
    mission_mgr = new pyModuleBase;

    // import and init the python modules
    packer->init("comms.packer");
    logging->init("comms.logging");
    remote_link->init("comms.remote_link");
    events->init("comms.events");
    mission_mgr->init("mission.mission_mgr");
    
    return true;
}
