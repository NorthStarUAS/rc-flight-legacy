// dig_filter.h - a flexible digital filter class
//
// Copyright (C) 2004-2017  Curtis L. Olson  - curtolson@flightgear.org
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

#pragma once

#include <string>
#include <deque>

using std::string;
using std::deque;

#include "component.h"


/**
 * AuraDigitalFilter - a selection of digital filters
 *
 * Exponential filter
 * Double exponential filter
 * Moving average filter
 * Noise spike filter
 *
 * All these filters are low-pass filters.
 *
 */

class AuraDigitalFilter : public APComponent
{
private:
    double Tf;            // Filter time [s]
    unsigned int samples; // Number of input samples to average
    double rateOfChange;  // The maximum allowable rate of change [1/s]
    deque <double> output;
    deque <double> input;
    enum filterTypes { exponential, doubleExponential, movingAverage, noiseSpike };
    filterTypes filterType;

    bool debug;

public:
    AuraDigitalFilter( string config_path );
    ~AuraDigitalFilter() {}

    void reset();
    void update(double dt);
};
