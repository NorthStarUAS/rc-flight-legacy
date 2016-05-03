//
// globals.hxx - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#ifndef AURA_GLOBALS_HXX
#define AURA_GLOBALS_HXX


#include "comms/events.hxx"
#include "comms/packer.hxx"
#include "comms/telnet.hxx"
#include "control/circle_mgr.hxx"
#include "control/route_mgr.hxx"
#include "python/pymodule.hxx"


extern pyModuleEventLog *events;
extern pyModulePacker *packer;
extern UGTelnet *telnet;
extern AuraCircleMgr *circle_mgr;
extern FGRouteMgr *route_mgr;
extern pyModuleBase *mission_mgr;


bool AuraCoreInit();


#endif // AURA_GLOBALS_HXX
