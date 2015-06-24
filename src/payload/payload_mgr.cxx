/**
 * \file: payload_mgr.cpp
 *
 * Payload manager
 *
 * Copyright (C) 2011 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: act_mgr.cpp,v 1.3 2009/08/25 15:04:01 curt Exp $
 */

#include <cstdio>

#include "comms/display.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "init/globals.hxx"
#include "props/props.hxx"

#include "payload_mgr.hxx"


// comm property nodes
static SGPropertyNode *payload_console_skip = NULL;
static SGPropertyNode *payload_logging_skip = NULL;


UGPayloadMgr::UGPayloadMgr()
{
}

UGPayloadMgr::~UGPayloadMgr() {
}



void UGPayloadMgr::bind() {
    // config_props = fgGetNode( "/config/payload", true );

    // initialize comm nodes
    payload_console_skip = fgGetNode("/config/remote-link/payload-skip", true);
    payload_logging_skip = fgGetNode("/config/logging/payload-skip", true);
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
				 payload_console_skip->getIntValue() );
	}

	if ( log_to_file ) {
	    log_payload( buf, size, payload_logging_skip->getIntValue() );
	}
    }
    return true;
}


void UGPayloadMgr::close() {
}


// global instance of the payload manager
UGPayloadMgr payload_mgr;
