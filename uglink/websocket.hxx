// \file websocket.hxx
// websocket (html5) server class.
//
// Copyright (C) 2012  Curtis L. Olson - colson@atiak.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//


#ifndef _UG_WEBSOCKET_HXX
#define _UG_WEBSOCKET_HXX

#include <string>
#include <vector>

using std::string;
using std::vector;

#include <plib/netChannel.h>


/**
 * WebSocket server class.
 * This class provides a websocket v13 server for remote javascript access to
 * program data.
 */
class UGWebSocket: netChannel
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
    UGWebSocket( const int port_num );

    /**
     * Destructor.
     */
    ~UGWebSocket();

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

#endif // _UG_WEBSOCKET_HXX

