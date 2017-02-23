//
// globals.hxx - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#ifndef AURA_GLOBALS_HXX
#define AURA_GLOBALS_HXX


#include "comms/display.hxx"
#include "comms/events.hxx"
#include "comms/logging.hxx"
#include "comms/packer.hxx"
#include "comms/remote_link.hxx"
#include "python/pymodule.hxx"


extern pyModuleDisplay *display;
extern pyModuleEventLog *events;
extern pyModuleLogging *logging;
extern pyModulePacker *packer;
extern pyModuleRemoteLink *remote_link;
extern pyModuleBase *mission_mgr;
extern pyModuleBase *telnet;


bool AuraCoreInit();


#endif // AURA_GLOBALS_HXX
