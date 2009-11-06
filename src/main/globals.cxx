//
// globals.cxx - global references
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started Fall 2009.
// This code is released into the public domain.
// 

#include "comms/packetizer.hxx"


UGPacketizer *packetizer;


bool UGGlobals_init() {
    packetizer = new UGPacketizer;

    return true;
}
