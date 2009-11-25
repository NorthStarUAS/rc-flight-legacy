// \file telnet.hxx
// telnet oroperty server class.
//
// Written by Curtis Olson, started September 2000.
// Modified by Bernie Bright, May 2002.
// Adapted from FlightGear props.hxx/cxx code November 2009.
//
// Copyright (C) 2000  Curtis L. Olson - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU LGPL
//

#ifndef _UGEAR_TELNET_HXX
#define _UGEAR_TELNET_HXX


#include "netChannel.h"


/**
 * Telent server class.
 * This class provides a telnet-like server for remote access to
 * FlightGear properties.
 */
class UGTelnet: netChannel
{

private:

    /**
     * Server port to listen on.
     */
    int port;
    bool enabled;

public:

    /**
     * Create a new TCP server.
     * 
     * @param tokens Tokenized configuration parameters
     */
    UGTelnet( const int port_num );

    /**
     * Destructor.
     */
    ~UGTelnet();

    /**
     * Start the telnet server.
     */
    bool open();

    /**
     * Process network activity.
     */
    bool process();

    /**
     * 
     */
    bool close();

    /**
     * Accept a new client connection.
     */
    void handleAccept();

};

#endif // _UGEAR_TELNET_HXX

