//
// globals.hxx - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#ifndef UGEAR_GLOBALS_HXX
#define UGEAR_GLOBALS_HXX


#include "comms/packetizer.hxx"
#include "comms/telnet.hxx"


extern UGPacketizer *packetizer;
extern UGTelnet *telnet;


bool UGGlobals_init();


#endif // UGEAR_GLOBALS_HXX
