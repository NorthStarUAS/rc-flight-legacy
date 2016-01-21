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


// comm property nodes
static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;


UGPayloadMgr::UGPayloadMgr()
{
}

UGPayloadMgr::~UGPayloadMgr() {
}



void UGPayloadMgr::bind() {
    // initialize comm nodes
    remote_link_node = pyGetNode("/config/remote_link", true);
    logging_node = pyGetNode("/config/logging", true);
}


void UGPayloadMgr::init() {
    bind();
}


bool UGPayloadMgr::update() {
    if ( remote_link_on || log_to_file ) {
	uint8_t buf[256];
	int size = packetizer->packetize_payload( buf );

	if ( remote_link_on ) {
	    // printf("sending filter packet\n");
	    remote_link_payload( buf, size,
				 remote_link_node.getLong("payload_skip") );
	}

	if ( log_to_file ) {
	    log_payload( buf, size, logging_node.getLong("payload_skip") );
	}
    }
    return true;
}


void UGPayloadMgr::close() {
}


// global instance of the payload manager
UGPayloadMgr payload_mgr;
