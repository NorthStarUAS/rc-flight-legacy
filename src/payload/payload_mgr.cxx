/**
 * \file: payload_mgr.cpp
 *
 * Payload manager
 *
 * Copyright (C) 2011 - Curtis L. Olson curtolson@flightgear.org
 *
 * $Id: act_mgr.cpp,v 1.3 2009/08/25 15:04:01 curt Exp $
 */

#include <pyprops.hxx>

#include <cstdio>

#include "comms/aura_messages.h"
#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "init/globals.hxx"

#include "payload_mgr.hxx"

// fixme: these shouldn't really be here in static/global space
static pyPropertyNode status_node;
static pyPropertyNode payload_node;

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
    status_node = pyGetNode( "/status", true );
    payload_node = pyGetNode("/payload", true);
}


bool UGPayloadMgr::update() {
    remote_link_count = 0;
    logging_count = 0;

    bool fresh_data = true;
    if ( fresh_data ) {
	bool send_remote_link = false;
	if ( remote_link_count < 0 ) {
	    send_remote_link = true;
	    remote_link_count = remote_link_skip;
	}
	
	bool send_logging = false;
	if ( logging_count < 0 ) {
	    send_logging = true;
	    logging_count = logging_skip;
	}
	
	if ( send_remote_link || send_logging ) {
            message_payload_v3_t payload;
            payload.index =  0;
            payload.timestamp_sec = status_node.getDouble("frame_time");
            payload.trigger_num = payload_node.getDouble("trigger_num");
            payload.pack();
	    if ( send_remote_link ) {
		remote_link->send_message( payload.id, payload.payload, payload.len );
	    }
	    if ( send_logging ) {
		logging->log_message( payload.id, payload.payload, payload.len );
	    }
	}
	
        remote_link_count--;
        logging_count--;
    }

    return true;
}


void UGPayloadMgr::close() {
}


// global instance of the payload manager
UGPayloadMgr payload_mgr;
