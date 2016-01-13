// xmlauto.hxx - a more flexible, generic way to build autopilots
//
// Written by Curtis Olson, started January 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
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


#ifndef _AURA_XMLAUTO_HXX
#define _AURA_XMLAUTO_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include "python/pyprops.hxx"

#include <string>
#include <vector>
#include <deque>

using std::string;
using std::vector;
using std::deque;


/**
 * Base class for other autopilot components
 */

class FGXMLAutoComponent {

protected:

    pyPropertyNode pid_node;
    
    pyPropertyNode enable_node;
    string enable_attr;
    string enable_value;
    bool honor_passive;
    bool enabled;

    pyPropertyNode input_node;
    string input_attr;
    
    pyPropertyNode ref_node;
    string ref_attr;
    string ref_value;
  
    string r_n_prop;
    string r_n_value;
    // SGPropertyNode *output_node;
    //vector <SGPropertyNode *> output_list;

    pyPropertyNode output_node;
    vector <string> output_list;

    pyPropertyNode config_node;

public:

    FGXMLAutoComponent() :
      //passive_mode( fgGetNode("/autopilot/locks/passive-mode", true) ),
      enable_value( "" ),
      honor_passive( false ),
      enabled( false )
      //input_prop( NULL ),
      //r_n_prop( NULL ),
      //r_n_value( NULL )
    { }

    virtual ~FGXMLAutoComponent() {}

    virtual void update (double dt)=0;
    
    inline const char *get_name() { return pid_node.getString("name").c_str(); }
};


/**
 * Roy Ovesen's PID controller
 */

class FGPIDController : public FGXMLAutoComponent {

private:

    // debug flag
    //SGPropertyNode *debug_node;

    // Input values
    //SGPropertyNode *y_n_node;                 // measured process value
    //SGPropertyNode *r_n_node;                 // reference (set point) value

    // Configuration values
    //SGPropertyNode *Ts_node;	              // time step (optional)
    //SGPropertyNode *Kp_node;                  // proportional gain

    //SGPropertyNode *alpha_node;               // low pass filter weighing factor (usually 0.1)
    //SGPropertyNode *beta_node;                // process value weighing factor for
                                // calculating proportional error
                                // (usually 1.0)
    //SGPropertyNode *gamma_node;               // process value weighing factor for
                                // calculating derivative error
                                // (usually 0.0)

    //SGPropertyNode *Ti_node;                  // Integrator time (sec)
    //SGPropertyNode *Td_node;                  // Derivator time (sec)

    //SGPropertyNode *u_min_node;               // Minimum output clamp
    //SGPropertyNode *u_max_node;               // Maximum output clamp

    // Previous state tracking values
    double ep_n_1;              // ep[n-1]  (prop error)
    double edf_n_1;             // edf[n-1] (derivative error)
    double edf_n_2;             // edf[n-2] (derivative error)
    double u_n_1;               // u[n-1]   (output)
    double desiredTs;            // desired sampling interval (sec)
    double elapsedTime;          // elapsed time (sec)
    
    
    
public:

    FGPIDController( pyPropertyNode *pid_node );
    FGPIDController( pyPropertyNode *pid_node, bool old );
    ~FGPIDController() {}

    void update_old( double dt );
    void update( double dt );
};


/**
 * A simplistic P [ + I ] PID controller
 */

class FGPISimpleController : public FGXMLAutoComponent {

private:

    // proportional component data
    bool proportional;
    //SGPropertyNode *Kp_node;

    // integral component data
    bool integral;
    //SGPropertyNode *Ki_node;
    double int_sum;

    // post functions for output
    bool clamp;

    // debug flag
    //SGPropertyNode *debug_node;

    // Input values
    double y_n;                 // measured process value
    double r_n;                 // reference (set point) value

    //SGPropertyNode *u_min_node;               // Minimum output clamp
    //SGPropertyNode *u_max_node;               // Maximum output clamp

    
public:

    FGPISimpleController( pyPropertyNode *pid_node );
    ~FGPISimpleController() {}

    void update( double dt );
};


/**
 * Predictor - calculates value in x seconds future.
 */

class FGPredictor : public FGXMLAutoComponent {

private:

    // proportional component data
    double last_value;
    double average;
    double seconds;
    double filter_gain;

    // debug flag
    bool debug_node;

    // Input values
    double ivalue;                 // input value
    
public:

    FGPredictor( pyPropertyNode *pid_node );
    ~FGPredictor() {}

    void update( double dt );
};


/**
 * FGDigitalFilter - a selection of digital filters
 *
 * Exponential filter
 * Double exponential filter
 * Moving average filter
 * Noise spike filter
 *
 * All these filters are low-pass filters.
 *
 */

class FGDigitalFilter : public FGXMLAutoComponent
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
    FGDigitalFilter( pyPropertyNode *pid_node );
    ~FGDigitalFilter() {}

    void update(double dt);
};

/**
 * Model an autopilot system.
 * 
 */

class FGXMLAutopilot /* : public SGSubsystem */
{

public:

    FGXMLAutopilot();
    ~FGXMLAutopilot();

    void init();
    void reinit();
    void bind();
    void unbind();
    void update( double dt );

    bool build();

protected:

    typedef vector<FGXMLAutoComponent *> comp_list;

private:

    bool serviceable;
    //SGPropertyNode *config_props;
    comp_list components;
};


#endif // _AURA_XMLAUTO_HXX
