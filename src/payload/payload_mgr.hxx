/**
 * \file: payload_mgr.h
 *
 * Top level payload manager
 *
 * Copyright (C) 2013 - Curtis L. Olson curtolson@flightgear.org
 */


#pragma once

#include "python/pyprops.hxx"

class UGPayloadMgr {

public:

    UGPayloadMgr();
    ~UGPayloadMgr();

    void bind();
    void init();
    bool update();
    void close();

private:
    
    int remote_link_skip;
    int logging_skip;
    int remote_link_count;
    int logging_count;
};


// global reference to payload mgr (move to globals?)
extern UGPayloadMgr payload_mgr;
