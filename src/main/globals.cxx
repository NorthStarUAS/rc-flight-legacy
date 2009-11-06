#include "comms/packetizer.hxx"


UGPacketizer *packetizer;


bool UGGlobals_init() {
    packetizer = new UGPacketizer;

    return true;
}
