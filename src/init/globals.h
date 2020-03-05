//
// globals.h - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#pragma once

#include "comms/display.h"
#include "comms/events.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include <pymodule.h>

extern pyModuleDisplay *display;
extern pyModuleEventLog *events;
extern pyModuleLogging *logging;
extern pyModuleRemoteLink *remote_link;
extern pyModuleBase *mission_mgr;
extern pyModuleBase *telnet;

bool AuraCoreInit();
