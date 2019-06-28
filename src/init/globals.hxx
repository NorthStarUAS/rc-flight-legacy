//
// globals.hxx - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#pragma once

#include "comms/display.hxx"
#include "comms/events.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include <pymodule.hxx>

extern pyModuleDisplay *display;
extern pyModuleEventLog *events;
extern pyModuleLogging *logging;
extern pyModuleRemoteLink *remote_link;
extern pyModuleBase *mission_mgr;
extern pyModuleBase *telnet;

bool AuraCoreInit();
