// ap.hxx - a flexible generic way to build autopilots
//
// Written by Curtis Olson, started January 2004.
//
// Copyright (C) 2004-2017  Curtis L. Olson - curtolson@flightgear.org
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
// $Id: xmlauto.hxx,v 1.1 2007/03/20 20:39:49 curt Exp $


#ifndef _AURA_AUTOPILOT_HXX
#define _AURA_AUTOPILOT_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include "python/pyprops.hxx"

#include <vector>
using std::vector;

#include "component.hxx"


/**
 * Model an autopilot system.
 * 
 */

class AuraAutopilot {

public:

    AuraAutopilot() {}
    ~AuraAutopilot() {}

    void init();
    void reset();
    void update( double dt );

    bool build();

protected:

    typedef vector<APComponent *> comp_list;

private:

    bool serviceable;
    comp_list components;
};


#endif // _AURA_AUTOPILOT_HXX
