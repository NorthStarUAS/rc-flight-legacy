// \file telnet.cxx
// telnet property server class.
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

#include <sstream>

#include "props/props.hxx"
#include "props/props_io.hxx"
#include "util/strutils.hxx"

#include "netChat.h"
#include "logging.h"
#include "telnet.hxx"

using std::stringstream;
using std::ends;


/**
 * Props connection class.
 * This class represents a connection to props client.
 */
class PropsChannel : public netChat
{
    netBuffer buffer;

    /**
     * Current property node name.
     */
    string path;

    enum Mode {
	PROMPT,
	DATA
    };
    Mode mode;

public:

    /**
     * Constructor.
     */
    PropsChannel();
    
    /**
     * Append incoming data to our request buffer.
     *
     * @param s Character string to append to buffer
     * @param n Number of characters to append.
     */
    void collectIncomingData( const char* s, int n );

    /**
     * Process a complete request from the props client.
     */
    void foundTerminator();

private:
    /**
     * Return a "Node no found" error message to the client.
     */
    void node_not_found_error( const string& node_name );
};

/**
 * 
 */
PropsChannel::PropsChannel()
    : buffer(512),
      path("/"),
      mode(PROMPT)
{
    // setTerminator( "\r\n" );
    setTerminator( "\n" );
}

/**
 * 
 */
void
PropsChannel::collectIncomingData( const char* s, int n )
{
    buffer.append( s, n );
}

/**
 * 
 */
void
PropsChannel::node_not_found_error( const string& node_name )
{
    string error = "-ERR Node \"";
    error += node_name;
    error += "\" not found.";
    push( error.c_str() );
    push( getTerminator() );
}

// return a human readable form of the value "type"
static string getValueTypeString( const SGPropertyNode *node )
{
    string result;

    if ( node == NULL )
    {
        return "unknown";
    }

    SGPropertyNode::Type type = node->getType();
    if ( type == SGPropertyNode::UNSPECIFIED ) {
        result = "unspecified";
    } else if ( type == SGPropertyNode::NONE ) {
        result = "none";
    } else if ( type == SGPropertyNode::BOOL ) {
        result = "bool";
    } else if ( type == SGPropertyNode::INT ) {
        result = "int";
    } else if ( type == SGPropertyNode::LONG ) {
        result = "long";
    } else if ( type == SGPropertyNode::FLOAT ) {
        result = "float";
    } else if ( type == SGPropertyNode::DOUBLE ) {
        result = "double";
    } else if ( type == SGPropertyNode::STRING ) {
        result = "string";
    }

    return result;
}

/**
 * We have a command.
 * 
 */
void
PropsChannel::foundTerminator()
{
    const char* cmd = buffer.getData();
    if ( display_on ) {
	printf( "processing command = '%s'\n", cmd );
    }

    vector<string> tokens = split( cmd );

    SGPropertyNode* node = fgGetNode( path.c_str() );

    if (!tokens.empty()) {
	string command = tokens[0];

        if ( command == "null" ) {
            // do nothing!
	} else if ( command == "ls" ) {
	    SGPropertyNode* dir = node;
	    if (tokens.size() == 2) {
		if ( tokens[1][0] == '/' ) {
		    dir = fgGetNode( tokens[1].c_str() );
		} else {
		    string s = path;
		    s += "/";
		    s += tokens[1];
		    dir = fgGetNode( s.c_str() );
		}
	    }

	    if ( dir != NULL ) {
		for ( int i = 0; i < dir->nChildren(); i++ ) {
		    SGPropertyNode * child = dir->getChild(i);
		    string line = child->getDisplayName(true);

		    if ( child->nChildren() > 0 ) {
			line += "/";
		    } else {
			if (mode == PROMPT) {
			    string value = child->getStringValue();
			    line += " =\t'" + value + "'\t(";
			line += getValueTypeString( child );
			line += ")";
			}
		    }

		    line += getTerminator();
		    push( line.c_str() );
		}
	    } else {
		node_not_found_error( tokens[1] );
	    }
	} else if ( command == "cd" ) {
	    if (tokens.size() == 2) {
		SGPropertyNode* child = node->getNode( tokens[1].c_str() );
		if ( child ) {
		    node = child;
		    path = node->getPath();
		} else {
		    node_not_found_error( tokens[1] );
		}
	    }
	} else if ( command == "pwd" ) {
	    string pwd = node->getPath();
	    if (pwd.empty()) {
		pwd = "/";
	    }

	    push( pwd.c_str() );
	    push( getTerminator() );
	} else if ( command == "get" || command == "show" ) {
	    if ( tokens.size() == 2 ) {
		string tmp;
		string value = node->getStringValue ( tokens[1].c_str(), "" );
		if ( mode == PROMPT ) {
		    tmp = tokens[1];
		    tmp += " = '";
		    tmp += value;
		    tmp += "' (";
		    tmp += getValueTypeString( node->getNode( tokens[1].c_str() ) );
		    tmp += ")";
		} else {
		    tmp = value;
		}
		push( tmp.c_str() );
		push( getTerminator() );
	    }
	} else if ( command == "set" ) {
	    if ( tokens.size() >= 2 ) {
		string value, tmp;
		for (unsigned int i = 2; i < tokens.size(); i++) {
		    if ( i > 2 ) {
			value += " ";
		    }
		    value += tokens[i];
		}
		node->getNode( tokens[1].c_str(), true )
		    ->setStringValue(value.c_str());

		if ( mode == PROMPT ) {
		    // now fetch and write out the new value as confirmation
		    // of the change
		    value = node->getStringValue ( tokens[1].c_str(), "" );
		    tmp = tokens[1] + " = '" + value + "' (";
		    tmp += getValueTypeString( node->getNode( tokens[1].c_str() ) );
		    tmp += ")";
		    push( tmp.c_str() );
		    push( getTerminator() );
		}
	    }
	} else if ( command == "dump" ) {
	    stringstream buf;
	    if ( tokens.size() <= 1 ) {
		writeProperties( buf, node, true );
		buf << ends; // null terminate the string
		push( buf.str().c_str() );
		push( getTerminator() );
	    } else {
		SGPropertyNode *child = node->getNode( tokens[1].c_str() );
		if ( child ) {
		    writeProperties ( buf, child, true );
		    buf << ends; // null terminate the string
		    push( buf.str().c_str() );
		    push( getTerminator() );
		} else {
		    node_not_found_error( tokens[1] );
		}
	    }
         } else if ( command == "quit" ) {
	    close();
	    shouldDelete();
	    return;
	} else if ( command == "data" ) {
	    mode = DATA;
	} else if ( command == "prompt" ) {
	    mode = PROMPT;
	} else {
	    const char* msg = "\
Valid commands are:\r\n\
\r\n\
cd <dir>           cd to a directory, '..' to move back\r\n\
data               switch to raw data mode\r\n\
dump [<dir>]       dump the current state (in xml)\r\n\
get <var>          show the value of a parameter\r\n\
help               show this help message\r\n\
ls [<dir>]         list directory\r\n\
prompt             switch to interactive mode (default)\r\n\
pwd                display your current path\r\n\
quit               terminate connection\r\n\
# run <command>      run built in command\r\n\
set <var> <val>    set <var> to a new <val>\r\n";
	    push( msg );
	}
    }

    if (mode == PROMPT) {
	string prompt = "> ";
	push( prompt.c_str() );
    }

    buffer.remove();
}

/**
 * 
 */
UGTelnet::UGTelnet( const int port_num ):
    enabled(false)
{
    port = port_num;
}

/**
 * 
 */
UGTelnet::~UGTelnet()
{
}

/**
 * 
 */
bool
UGTelnet::open()
{
    if (enabled ) {
	printf("This shouldn't happen, but the telnet channel is already in use, ignoring\n" );
	return false;
    }

    netChannel::open();
    netChannel::bind( "", port );
    netChannel::listen( 5 );
    printf("Telnet server started on port %d\n", port );

    enabled = true;

    return true;
}

/**
 * 
 */
bool
UGTelnet::close()
{
    if ( display_on ) {
	printf("closing UGTelnet\n" );
    }

    return true;
}

/**
 * 
 */
bool
UGTelnet::process()
{
    netChannel::poll();
    return true;
}

/**
 * 
 */
void
UGTelnet::handleAccept()
{
    netAddress addr;
    int handle = netChannel::accept( &addr );
    printf("Telnet server accepted connection from %s:%d\n",
           addr.getHost(), addr.getPort() );
    PropsChannel* channel = new PropsChannel();
    channel->setHandle( handle );
}
