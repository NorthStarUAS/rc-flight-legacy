// String utilities.
//
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
// $Id: strutils.cxx,v 1.1 2008/04/04 06:22:43 curt Exp $

#include <ctype.h>
#include "strutils.hxx"

/**
 * 
 */
static vector<string>
split_whitespace( const string& str, int maxsplit )
{
    vector<string> result;
    string::size_type len = str.length();
    string::size_type i = 0;
    string::size_type j;
    int countsplit = 0;

    while (i < len) {
        while (i < len && isspace((unsigned char)str[i])) {
            ++i;
        }

        j = i;

        while (i < len && !isspace((unsigned char)str[i])) {
            ++i;
        }

        if (j < i) {
            result.push_back( str.substr(j, i-j) );
            ++countsplit;
            while (i < len && isspace((unsigned char)str[i])) {
                ++i;
            }

            if (maxsplit && (countsplit >= maxsplit) && i < len) {
                result.push_back( str.substr( i, len-i ) );
                i = len;
            }
        }
    }

    return result;
}

/**
 * 
 */
vector<string>
split( const string& str, const char* sep, int maxsplit )
{
    if (sep == 0)
        return split_whitespace( str, maxsplit );

    vector<string> result;
    int n = strlen( sep );
    if (n == 0) {
        // Error: empty separator string
        return result;
    }
    const char* s = str.c_str();
    string::size_type len = str.length();
    string::size_type i = 0;
    string::size_type j = 0;
    int splitcount = 0;

    while (i+n <= len) {
        if (s[i] == sep[0] && (n == 1 || memcmp(s+i, sep, n) == 0)) {
            result.push_back( str.substr(j,i-j) );
            i = j = i + n;
            ++splitcount;
            if (maxsplit && (splitcount >= maxsplit))
                break;
        } else {
            ++i;
        }
    }

    result.push_back( str.substr(j,len-j) );
    return result;
}
