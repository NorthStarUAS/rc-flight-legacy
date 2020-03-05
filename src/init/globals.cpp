//
// globals.cpp - global modules
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#include "globals.h"


pyModuleDisplay *display = NULL;
pyModuleEventLog *events = NULL;
pyModuleLogging *logging = NULL;
pyModuleRemoteLink *remote_link = NULL;
pyModuleBase *mission_mgr = NULL;
pyModuleBase *telnet = NULL;


bool AuraCoreInit() {
    // create instances
    display = new pyModuleDisplay;
    events = new pyModuleEventLog;
    logging = new pyModuleLogging;
    remote_link = new pyModuleRemoteLink;
    mission_mgr = new pyModuleBase;
    telnet = new pyModuleBase;
    
    // import and init the python modules
    display->init("comms.display");
    logging->init("comms.logging");
    remote_link->init("comms.remote_link");
    events->init("comms.events");
    mission_mgr->init("mission.mission_mgr");
    telnet->init("comms.telnet");
    
    return true;
}
