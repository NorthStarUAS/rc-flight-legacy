/**
 * \file strutils.h
 * String utilities.
 */

// Written by Bernie Bright, started 1998
//
// Copyright (C) 1998  Bernie Bright - bbright@bigpond.net.au
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id: strutils.hxx,v 1.1 2008/04/04 06:22:43 curt Exp $


#ifndef UG_STRUTILS_H
#define UG_STRUTILS_H

#include <stdlib.h>

#include <string>
#include <vector>

using std::string;
using std::vector;


/**
 * Split a string into a words using 'sep' as the delimiter string.
 * Produces a result similar to the perl and python functions of the
 * same name.
 * 
 * @param s The string to split into words,
 * @param sep Word delimiters.  If not specified then any whitespace is a separator,
 * @param maxsplit If given, splits at no more than maxsplit places,
 * resulting in at most maxsplit+1 words.
 * @return Array of words.
 */
vector<string>
split( const string& s, const char* sep = 0, int maxsplit = 0 );


#endif // UG_STRUTILS_H
