/**
 * \file props_io.hxx
 * Interface definition for property list io.
 * Started Fall 2000 by David Megginson, david@megginson.com
 * This code is released into the Public Domain.
 *
 * See props.html for documentation [replace with URL when available].
 *
 * $Id: props_io.hxx,v 1.1 2007/02/14 02:43:46 curt Exp $
 */

#ifndef __PROPS_IO_HXX
#define __PROPS_IO_HXX

#include "props/props.hxx"

#include <stdio.h>

#include <string>
#include <vector>
#include <map>
#include <iostream>

using std::string;
using std::vector;
using std::map;
using std::istream;
using std::ostream;

/**
 * Read properties from an XML input stream.
 */
void readProperties (istream &input, SGPropertyNode * start_node,
		     const string &base = "", int default_mode = 0);


/**
 * Read properties from an XML file.
 */
void readProperties (const string &file, SGPropertyNode * start_node,
                     int default_mode = 0);


/**
 * Read properties from an in-memory buffer.
 */
void readProperties (const char *buf, const int size,
                     SGPropertyNode * start_node, int default_mode = 0);


/**
 * Write properties to an XML output stream.
 */
void writeProperties (ostream &output, const SGPropertyNode * start_node,
		      bool write_all = false,
		      SGPropertyNode::Attribute archive_flag = SGPropertyNode::ARCHIVE);


/**
 * Write properties to an XML file.
 */
void writeProperties (const string &file, const SGPropertyNode * start_node,
		      bool write_all = false,
		      SGPropertyNode::Attribute archive_flag = SGPropertyNode::ARCHIVE);


/**
 * Copy properties from one node to another.
 */
bool copyProperties (const SGPropertyNode *in, SGPropertyNode *out);


#endif // __PROPS_IO_HXX

// end of props_io.hxx
