/**
 * \file: payload_mgr.cpp
 *
 * Payload manager
 *
 * Copyright (C) 2011 - Curtis L. Olson curtolson@flightgear.org
 *
 * $Id: act_mgr.cpp,v 1.3 2009/08/25 15:04:01 curt Exp $
 */

#include "python/pyprops.hxx"

#include <cstdio>

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "init/globals.hxx"

#include "payload_mgr.hxx"


UGPayloadMgr::UGPayloadMgr():
    remote_link_skip(0),
    logging_skip(0),
    remote_link_count(0),
    logging_count(0)
{
}

UGPayloadMgr::~UGPayloadMgr() {
}



void UGPayloadMgr::bind() {
    // pass
}


void UGPayloadMgr::init() {
    bind();
    
    pyPropertyNode remote_link_node = pyGetNode("/config/remote_link", true);
    pyPropertyNode logging_node = pyGetNode("/config/logging", true);
    remote_link_skip = remote_link_node.getDouble("payload_skip");
    logging_skip = logging_node.getDouble("payload_skip");
}


bool UGPayloadMgr::update() {
    remote_link_count = remote_link_random( remote_link_skip );
    logging_count = remote_link_random( logging_skip );

    bool fresh_data = true;
    if ( fresh_data ) {
	bool send_remote_link = false;
	if ( remote_link_on && remote_link_count <= 0 ) {
	    send_remote_link = true;
	    remote_link_count = remote_link_skip;
	}
	
	bool send_logging = false;
	if ( log_to_file && logging_count <= 0 ) {
	    send_logging = true;
	    logging_count = logging_skip;
	}
	
	if ( send_remote_link || send_logging ) {
	    uint8_t buf[256];
	    int size = packer->pack_payload( 0, buf );
	    if ( send_remote_link ) {
		remote_link_payload( buf, size );
	    }
	    if ( send_logging ) {
		log_payload( buf, size );
	    }
	}
	
	if ( remote_link_on ) {
	    remote_link_count--;
	}
	
	if ( log_to_file ) {
	    logging_count--;
	}
    }

    return true;
}


void UGPayloadMgr::close() {
}


// global instance of the payload manager
UGPayloadMgr payload_mgr;
