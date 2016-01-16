// xmlauto.cxx - a more flexible, generic way to build autopilots
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


#include "python/pyprops.hxx"

#include <math.h>
#include <stdlib.h>

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "util/exception.hxx"
#include "util/sg_path.hxx"
#include "util/wind.hxx"

#include "xmlauto.hxx"


FGPIDController::FGPIDController( pyPropertyNode *pid_node ):
    ep_n_1( 0.0 ),
    edf_n_1( 0.0 ),
    edf_n_2( 0.0 ),
    u_n_1( 0.0 ),
    desiredTs( 0.00001 ),
    elapsedTime( 0.0 )
{
    size_t pos;
    
    // enable
    pyPropertyNode node = pid_node->getChild("enable", true);
    string enable_prop = node.getString("prop");
    enable_value = node.getString("value");
    honor_passive = node.getBool("honor_passive");
    pos = enable_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = enable_prop.substr(0, pos);
	enable_attr = enable_prop.substr(pos+1);
	enable_node = pyGetNode( path, true );
    }

    // input
    node = pid_node->getChild("input", true);
    string input_prop = node.getString("prop");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = input_prop.substr(0, pos);
	input_attr = input_prop.substr(pos+1);
	input_node = pyGetNode( path, true );
    }

    // reference
    node = pid_node->getChild("reference", true);
    string ref_prop = node.getString("prop");
    ref_value = node.getString("value");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = ref_prop.substr(0, pos);
	ref_attr = ref_prop.substr(pos+1);
	ref_node = pyGetNode( path, true );
    }

    // output
    node = pid_node->getChild( "output", true );
    unsigned int num = node.getLen("prop");
    for ( unsigned int i = 0; i < num; ++i ) {
	ostringstream str;
	str << "prop" << '[' << i << ']';
	string ename = str.str();
	string output_prop = node.getString(ename.c_str());
	pos = output_prop.rfind("/");
	if ( pos != string::npos ) {
	    string path = output_prop.substr(0, pos);
	    string attr = output_prop.substr(pos+1);
	    pyPropertyNode onode = pyGetNode( path, true );
	    output_node.push_back( onode );
	    output_attr.push_back( attr );
	}
    }
 
    // config
    pyPropertyNode config_node = pid_node->getChild( "config", true );
    if ( config_node.hasChild("Ts") ) {
	desiredTs = config_node.getDouble("Ts");
    }
            
    if ( !config_node.hasChild("beta") ) {
	// create with default value
	config_node.setDouble( "beta", 1.0 );
    }
    if ( !config_node.hasChild("alpha") ) {
	// create with default value
	config_node.setDouble( "alpha", 0.1 );
    }
}


/*
 * Roy Vegard Ovesen:
 *
 * Ok! Here is the PID controller algorithm that I would like to see
 * implemented:
 *
 *   delta_u_n = Kp * [ (ep_n - ep_n-1) + ((Ts/Ti)*e_n)
 *               + (Td/Ts)*(edf_n - 2*edf_n-1 + edf_n-2) ]
 *
 *   u_n = u_n-1 + delta_u_n
 *
 * where:
 *
 * delta_u : The incremental output
 * Kp      : Proportional gain
 * ep      : Proportional error with reference weighing
 *           ep = beta * (r - y)
 *           where:
 *           beta : Weighing factor
 *           r    : Reference (setpoint)
 *           y    : Process value, measured
 * e       : Error
 *           e = r - y
 * Ts      : Sampling interval
 * Ti      : Integrator time
 * Td      : Derivator time
 * edf     : Derivate error with reference weighing and filtering
 *           edf_n = edf_n-1 / ((Ts/Tf) + 1) + ed_n * (Ts/Tf) / ((Ts/Tf) + 1)
 *           where:
 *           Tf : Filter time
 *           Tf = alpha * Td , where alpha usually is set to 0.1
 *           ed : Unfiltered derivate error with reference weighing
 *             ed = gamma * r - y
 *             where:
 *             gamma : Weighing factor
 * 
 * u       : absolute output
 * 
 * Index n means the n'th value.
 * 
 * 
 * Inputs:
 * enabled ,
 * y_n , r_n , beta=1 , gamma=0 , alpha=0.1 ,
 * Kp , Ti , Td , Ts (is the sampling time available?)
 * u_min , u_max
 * 
 * Output:
 * u_n
 */

void FGPIDController::update( double dt ) {
    double ep_n;            // proportional error with reference weighing
    double e_n;             // error
    double ed_n;            // derivative error
    double edf_n;           // derivative error filter
    double Tf;              // filter time
    double delta_u_n = 0.0; // incremental output
    double u_n = 0.0;       // absolute output
    double Ts;              // sampling interval (sec)
    
    elapsedTime += dt;
    if ( elapsedTime <= desiredTs ) {
        // do nothing if no time has elapsed
        return;
    }
    Ts = elapsedTime;
    elapsedTime = 0.0;

    if (!enable_node.isNull() && enable_node.getString("enable_attr") == enable_value) {
	enabled = true;
    } else {
	enabled = false;
    }

    bool debug = pid_node.getBool("debug");

    if ( Ts > 0.0) {
        if ( debug ) printf("Updating %s Ts = %.2f", get_name(), Ts );

        double y_n = 0.0;
	y_n = input_node.getDouble(input_attr.c_str());

        double r_n = 0.0;
	if ( ref_value != "" ) {
	    r_n = atof(ref_value.c_str());
	} else {
            r_n = ref_node.getDouble(ref_attr.c_str());
	}
                      
        if ( debug ) printf("  input = %.3f ref = %.3f\n", y_n, r_n );

        // Calculates proportional error:
        ep_n = config_node.getDouble("beta") * (r_n - y_n);
        if ( debug ) {
	    printf( "  ep_n = %.3f", ep_n);
	    printf( "  ep_n_1 = %.3f", ep_n_1);
	}

        // Calculates error:
        e_n = r_n - y_n;
        if ( debug ) printf( " e_n = %.3f", e_n);

        // Calculates derivate error:
        ed_n = config_node.getDouble("gamma") * r_n - y_n;
        if ( debug ) printf(" ed_n = %.3f", ed_n);

	double Td = config_node.getDouble("Td");
        if ( Td > 0.0 ) {
            // Calculates filter time:
            Tf = config_node.getDouble("alpha") * Td;
            if ( debug ) printf(" Tf = %.3f", Tf);

            // Filters the derivate error:
            edf_n = edf_n_1 / (Ts/Tf + 1)
                + ed_n * (Ts/Tf) / (Ts/Tf + 1);
            if ( debug ) printf(" edf_n = %.3f", edf_n);
        } else {
            edf_n = ed_n;
        }

        // Calculates the incremental output:
	double Ti = config_node.getDouble("Ti");
	double Kp = config_node.getDouble("Kp");
        if ( Ti > 0.0 ) {
            delta_u_n = Kp * ( (ep_n - ep_n_1)
                               + ((Ts/Ti) * e_n)
                               + ((Td/Ts) * (edf_n - 2*edf_n_1 + edf_n_2)) );
        }

        if ( debug ) {
	    printf(" delta_u_n = %.3f\n", delta_u_n);
            printf("P: %.3f  I: %.3f  D:%.3f\n",
		   Kp * (ep_n - ep_n_1),
		   Kp * ((Ts/Ti) * e_n),
		   Kp * ((Td/Ts) * (edf_n - 2*edf_n_1 + edf_n_2)));
        }

        // Integrator anti-windup logic:
	double u_min = config_node.getDouble("u_min");
	double u_max = config_node.getDouble("u_max");
        if ( delta_u_n > (u_max - u_n_1) ) {
            delta_u_n = u_max - u_n_1;
            if ( debug ) printf(" max saturation\n");
        } else if ( delta_u_n < (u_min - u_n_1) ) {
            delta_u_n = u_min - u_n_1;
            if ( debug ) printf(" min saturation\n");
        }

        // Calculates absolute output:
        u_n = u_n_1 + delta_u_n;
        if ( debug ) printf("  output = %.3f\n", u_n);

        // Updates indexed values;
        u_n_1   = u_n;
        ep_n_1  = ep_n;
        edf_n_2 = edf_n_1;
        edf_n_1 = edf_n;
    }

    if ( enabled ) {
	// Copy the result to the output node(s)
	for ( unsigned int i = 0; i < output_node.size(); i++ ) {
	    output_node[i].setDouble( output_attr[i].c_str(), u_n );
	}
    } else if ( output_node.size() > 0 ) {
	// Mirror the output value while we are not enabled so there
	// is less of a continuity break when this module is enabled

	// pull output value from the corresponding property tree value
	u_n = output_node[0].getDouble(output_attr[0].c_str());
	// and clip
	double u_min = config_node.getDouble("u_min");
	double u_max = config_node.getDouble("u_max");
 	if ( u_n < u_min ) { u_n = u_min; }
	if ( u_n > u_max ) { u_n = u_max; }
	u_n_1 = u_n;
    }
}


FGPISimpleController::FGPISimpleController( pyPropertyNode *pid_node ):
    proportional( false ),
    integral( false ),
    int_sum( 0.0 ),
    clamp( false ),
    y_n( 0.0 ),
    r_n( 0.0 )
{
    size_t pos;

    // enable
    pyPropertyNode node = pid_node->getChild("enable", true);
    string enable_prop = node.getString("prop");
    enable_value = node.getString("value");
    honor_passive = node.getBool("honor_passive");
    pos = enable_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = enable_prop.substr(0, pos);
	enable_attr = enable_prop.substr(pos+1);
	enable_node = pyGetNode( path, true );
    }

    // input
    node = pid_node->getChild("input", true);
    string input_prop = node.getString("prop");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = input_prop.substr(0, pos);
	input_attr = input_prop.substr(pos+1);
	input_node = pyGetNode( path, true );
    }

    // reference
    node = pid_node->getChild("reference", true);
    string ref_prop = node.getString("prop");
    ref_value = node.getString("value");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = ref_prop.substr(0, pos);
	ref_attr = ref_prop.substr(pos+1);
	ref_node = pyGetNode( path, true );
    }

    // output
    node = pid_node->getChild( "output", true );
    unsigned int num = node.getLen("prop");
    for ( unsigned int i = 0; i < num; ++i ) {
	ostringstream str;
	str << "prop" << '[' << i << ']';
	string ename = str.str();
	string output_prop = node.getString(ename.c_str());
	pos = output_prop.rfind("/");
	if ( pos != string::npos ) {
	    string path = output_prop.substr(0, pos);
	    string attr = output_prop.substr(pos+1);
	    pyPropertyNode onode = pyGetNode( path, true );
	    output_node.push_back( onode );
	    output_attr.push_back( attr );
	}
    }
 
    // config
    pyPropertyNode config_node = pid_node->getChild( "config", true );
}


void FGPISimpleController::update( double dt ) {
    if (!enable_node.isNull() && enable_node.getString("enable_attr") == enable_value) {
	enabled = true;
    } else {
	enabled = false;
    }

    bool debug = pid_node.getBool("debug");
    if ( debug ) printf("Updating %s\n", get_name());
    double input = 0.0;
    input = input_node.getDouble(input_attr.c_str());

    double r_n = 0.0;
    if ( ref_value != "" ) {
	r_n = atof(ref_value.c_str());
    } else {
	r_n = ref_node.getDouble(ref_attr.c_str());
    }
                      
    double error = r_n - input;
    if ( debug ) printf("input = %.3f reference = %.3f error = %.3f\n",
			input, r_n, error);

    double prop_comp = 0.0;

    prop_comp = error * config_node.getDouble("Kp");
    double u_min = config_node.getDouble("u_min");
    double u_max = config_node.getDouble("u_max");
    if ( prop_comp < u_min ) { prop_comp = u_min; }
    if ( prop_comp > u_max ) { prop_comp = u_max; }

    int_sum += error * config_node.getDouble("Ki") * dt;

    double pre_output = prop_comp + int_sum;
    double clamp_output = pre_output;
    if ( clamp_output < u_min ) { clamp_output = u_min; }
    if ( clamp_output > u_max ) { clamp_output = u_max; }
    if ( clamp_output != pre_output && integral ) {
	int_sum = clamp_output - prop_comp;
    }

    if ( debug ) printf("prop_comp = %.3f int_sum = %.3f\n",
			prop_comp, int_sum);
    if ( debug ) printf("clamped output = %.3f\n", clamp_output);

    if ( enabled ) {
	// Copy the result to the output node(s)
	for ( unsigned int i = 0; i < output_node.size(); i++ ) {
	    output_node[i].setDouble( output_attr[i].c_str(), clamp_output );
	}
    }
}


FGPredictor::FGPredictor ( pyPropertyNode *config_node ):
    last_value ( 999999999.9 ),
    average ( 0.0 ),
    seconds( 0.0 ),
    filter_gain( 0.0 ),
    ivalue( 0.0 )
{
    size_t pos;

    // enable
    pyPropertyNode node = config_node->getChild("enable", true);
    string enable_prop = node.getString("prop");
    enable_value = node.getString("value");
    honor_passive = node.getBool("honor_passive");
    pos = enable_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = enable_prop.substr(0, pos);
	enable_attr = enable_prop.substr(pos+1);
	enable_node = pyGetNode( path, true );
    }

    // input
    node = config_node->getChild("input", true);
    string input_prop = node.getString("prop");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = input_prop.substr(0, pos);
	input_attr = input_prop.substr(pos+1);
	input_node = pyGetNode( path, true );
    }

    if ( config_node->hasChild("seconds") ) {
	seconds = config_node->getDouble("seconds");
    }
    if ( config_node->hasChild("filter_gain") ) {
	filter_gain = config_node->getDouble("filter_gain");
    }
    
    // output
    node = config_node->getChild( "output", true );
    unsigned int num = node.getLen("prop");
    for ( unsigned int i = 0; i < num; ++i ) {
	ostringstream str;
	str << "prop" << '[' << i << ']';
	string ename = str.str();
	string output_prop = node.getString(ename.c_str());
	pos = output_prop.rfind("/");
	if ( pos != string::npos ) {
	    string path = output_prop.substr(0, pos);
	    string attr = output_prop.substr(pos+1);
	    pyPropertyNode onode = pyGetNode( path, true );
	    output_node.push_back( onode );
	    output_attr.push_back( attr );
	}
    }
}

void FGPredictor::update( double dt ) {
    /*
       Simple moving average filter converts input value to predicted value "seconds".

       Smoothing as described by Curt Olson:
         gain would be valid in the range of 0 - 1.0
         1.0 would mean no filtering.
         0.0 would mean no input.
         0.5 would mean (1 part past value + 1 part current value) / 2
         0.1 would mean (9 parts past value + 1 part current value) / 10
         0.25 would mean (3 parts past value + 1 part current value) / 4

    */

    if (!enable_node.isNull() && enable_node.getString("enable_attr") == enable_value) {
	enabled = true;
    } else {
	enabled = false;
    }

    ivalue = input_node.getDouble(input_attr.c_str());

    if ( enabled ) {
        // first time initialize average
        if (last_value >= 999999999.0) {
           last_value = ivalue;
        }

        if ( dt > 0.0 ) {
            double current = (ivalue - last_value)/dt; // calculate current error change (per second)
            if ( dt < 1.0 ) {
                average = (1.0 - dt) * average + current * dt;
            } else {
                average = current;
            }

            // calculate output with filter gain adjustment
            double output = ivalue + (1.0 - filter_gain) * (average * seconds) + filter_gain * (current * seconds);

	    // Copy the result to the output node(s)
	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output );
	    }
        }
        last_value = ivalue;
    }
}


FGDigitalFilter::FGDigitalFilter( pyPropertyNode *config_node )
{
    size_t pos;
    samples = 1;

    // enable
    pyPropertyNode node = config_node->getChild("enable", true);
    string enable_prop = node.getString("prop");
    enable_value = node.getString("value");
    honor_passive = node.getBool("honor_passive");
    pos = enable_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = enable_prop.substr(0, pos);
	enable_attr = enable_prop.substr(pos+1);
	enable_node = pyGetNode( path, true );
    }

    // input
    node = config_node->getChild("input", true);
    string input_prop = node.getString("prop");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = input_prop.substr(0, pos);
	input_attr = input_prop.substr(pos+1);
	input_node = pyGetNode( path, true );
    }

    if ( config_node->hasChild("type") ) {
	string cval = config_node->getString("type");
	if ( cval == "exponential" ) {
	    filterType = exponential;
	} else if (cval == "double-exponential") {
	    filterType = doubleExponential;
	} else if (cval == "moving-average") {
	    filterType = movingAverage;
	} else if (cval == "noise-spike") {
	    filterType = noiseSpike;
	}
    }
    if ( config_node->hasChild("filter_time") ) {
	Tf = config_node->getDouble("filter_time");
    }
    if ( config_node->hasChild("samples") ) {
	samples = config_node->getLong("samples");
    }
    if ( config_node->hasChild("max_rate_of_change") ) {
	rateOfChange = config_node->getDouble("max_rate_of_change");
    }

    // output
    node = config_node->getChild( "output", true );
    unsigned int num = node.getLen("prop");
    for ( unsigned int i = 0; i < num; ++i ) {
	ostringstream str;
	str << "prop" << '[' << i << ']';
	string ename = str.str();
	string output_prop = node.getString(ename.c_str());
	pos = output_prop.rfind("/");
	if ( pos != string::npos ) {
	    string path = output_prop.substr(0, pos);
	    string attr = output_prop.substr(pos+1);
	    pyPropertyNode onode = pyGetNode( path, true );
	    output_node.push_back( onode );
	    output_attr.push_back( attr );
	}
    }

    output.resize(2, 0.0);
    input.resize(samples + 1, 0.0);
}

void FGDigitalFilter::update(double dt)
{
    if (!enable_node.isNull() && enable_node.getString("enable_attr") == enable_value) {
	enabled = true;
    } else {
	enabled = false;
    }

    input.push_front( input_node.getDouble(input_attr.c_str()) );
    input.resize(samples + 1, 0.0);

    if ( enabled && dt > 0.0 ) {
        /*
         * Exponential filter
         *
         * Output[n] = alpha*Input[n] + (1-alpha)*Output[n-1]
         *
         */

        if (filterType == exponential)
        {
            double alpha = 1 / ((Tf/dt) + 1);
            output.push_front(alpha * input[0] + 
                              (1 - alpha) * output[0]);
	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
            output.resize(1);
        } 
        else if (filterType == doubleExponential)
        {
            double alpha = 1 / ((Tf/dt) + 1);
            output.push_front(alpha * alpha * input[0] + 
                              2 * (1 - alpha) * output[0] -
                              (1 - alpha) * (1 - alpha) * output[1]);
 	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
            output.resize(2);
        }
        else if (filterType == movingAverage)
        {
            output.push_front(output[0] + 
                              (input[0] - input.back()) / samples);
 	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
            output.resize(1);
        }
        else if (filterType == noiseSpike)
        {
            double maxChange = rateOfChange * dt;

            if ((output[0] - input[0]) > maxChange)
            {
                output.push_front(output[0] - maxChange);
            }
            else if ((output[0] - input[0]) < -maxChange)
            {
                output.push_front(output[0] + maxChange);
            }
            else if (fabs(input[0] - output[0]) <= maxChange)
            {
                output.push_front(input[0]);
            }

 	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
	    output.resize(1);
        }
        if ( pid_node.getBool("debug") ) {
            printf("input: %.3f\toutput: %.3f\n", input[0], output[0]);
        }
    }
}


FGXMLAutopilot::FGXMLAutopilot() {
}


FGXMLAutopilot::~FGXMLAutopilot() {
}

 
void FGXMLAutopilot::init() {
    if ( ! build() ) {
	printf("Detected an internal inconsistency in the autopilot\n");
	printf(" configuration.  See earlier errors for\n" );
	printf(" details.\n");
	exit(-1);
    }        
}


void FGXMLAutopilot::reinit() {
    components.clear();
    init();
    build();
}


void FGXMLAutopilot::bind() {
}

void FGXMLAutopilot::unbind() {
}

bool FGXMLAutopilot::build() {
    pyPropertyNode config_props = pyGetNode( "/config/fcs/autopilot", true );

    // FIXME: we have always depended on the order of children
    // components here to ensure PID stages are run in the correct
    // order, however that is a bad thing to assume ... especially now
    // with pyprops!!!
    vector <string> children = config_props.getChildren();
    unsigned int count = children.size();
    for ( unsigned int i = 0; i < count; ++i ) {
        pyPropertyNode node = config_props.getChild(children[i].c_str());
        string name = children[i];
	size_t pos = name.find("[");
	if ( pos != string::npos ) {
	    name = name.substr(0, pos);
	}
        printf("%s\n", name.c_str());
        if ( name == "pid_controller" ) {
            FGXMLAutoComponent *c = new FGPIDController( &node );
            components.push_back( c );
        } else if ( name == "pi_simple_controller" ) {
            FGXMLAutoComponent *c = new FGPISimpleController( &node );
            components.push_back( c );
        } else if ( name == "predict_simple" ) {
            FGXMLAutoComponent *c = new FGPredictor( &node );
            components.push_back( c );
        } else if ( name == "filter" ) {
            FGXMLAutoComponent *c = new FGDigitalFilter( &node );
            components.push_back( c );
	} else if ( name == "L1_controller" ) {
	    // information placeholder, we don't do anything here.
        } else {
	    printf("Unknown top level section: %s\n", name.c_str() );
            return false;
        }
    }

    return true;
}


// normalize a value to lie between min and max
template <class T>
inline void SG_NORMALIZE_RANGE( T &val, const T min, const T max ) {
    T step = max - min;
    while( val >= max )  val -= step;
    while( val < min ) val += step;
};


/*
 * Update helper values
 */
static void update_helper( double dt ) {
#if 0
    // Estimate speed in 5,10 seconds
    static SGPropertyNode *vel = pyGetNode( "/velocity/airspeed-kt", true );
    static SGPropertyNode *lookahead5
        = pyGetNode( "/autopilot/internal/lookahead-5-sec-airspeed-kt", true );
    static SGPropertyNode *lookahead10
        = pyGetNode( "/autopilot/internal/lookahead-10-sec-airspeed-kt", true );

    static double average = 0.0; // average/filtered prediction
    static double v_last = 0.0;  // last velocity

    if ( dt > 0.0 ) {
        double v = vel.getDouble();
        double a = (v - v_last) / dt;

        if ( dt < 1.0 ) {
            average = (1.0 - dt) * average + dt * a;
        } else {
            average = a;
        }

        lookahead5->setDouble( v + average * 5.0 );
        lookahead10->setDouble( v + average * 10.0 );
        v_last = v;
    }
#endif

#if 0
    // given the current wind estimate, compute the true heading
    // required to fly the current ground course, then compute the
    // true heading required to fly the target nav course.  Steer
    // based on the heading difference in "true orientation" space
    // rather than in ground track space.  This schedules our
    // steering gain based on the wind vector automatically.  Note we
    // do not use our actual true heading because our wind estimate
    // and true heading estimate aren't perfect, winds can shift
    // rapidly, and trusting average or old estimates can throw us way
    // off.

    double diff;

    // real sensor data
    static SGPropertyNode *groundtrack_deg
        = pyGetNode( "/orientation/groundtrack-deg", true );

    // wind estimates
    // autopilot settings
    static SGPropertyNode *target_course_deg
        = pyGetNode( "/autopilot/settings/target-groundtrack-deg", true );

    // compute heading error in aircraft heading space after doing
    // wind triangle math on the current and target ground courses.
    // This gives us a close estimate of how far we have to yaw the
    // aircraft nose to get on the target ground course.
    double hdg_error = wind_heading_diff(groundtrack_deg.getDouble(),
					 target_course_deg.getDouble());

    static SGPropertyNode *wind_heading_error
        = pyGetNode( "/autopilot/settings/wind-heading-error-deg", true );
    wind_heading_error->setDouble( hdg_error );
#endif

#if 0
    // Calculate "wind compensated" groundtrack heading error
    // normalized to +/- 180.0.  The ground track heading error can be
    // misleading if there is wind.  we compute a wind compensated
    // true heading difference so we know what heading we need to roll
    // out at to achieve the target ground track heading.  The actual
    // computations are performed in the route_mgr code, the wind
    // estimation is performed in the filter_mgr code.
    static SGPropertyNode *target_wind_true
        = pyGetNode( "/filters/wind-est/target-heading-deg", true );
    static SGPropertyNode *true_hdg
        = pyGetNode( "/orientation/heading-deg", true );
    static SGPropertyNode *wind_true_error
        = pyGetNode( "/autopilot/internal/wind-true-error-deg", true );

    diff = target_wind_true.getDouble() - true_hdg.getDouble();
    if ( diff < -180.0 ) { diff += 360.0; }
    if ( diff > 180.0 ) { diff -= 360.0; }
    wind_true_error->setDouble( diff );
#endif

#if 0
    // calculate the roll angle squared for the purpose of adding open
    // loop elevator deflection in turns.
    static SGPropertyNode *roll_squared
        = pyGetNode( "/orientation/roll-deg-squared", true );
    static SGPropertyNode *phi_node
	= pyGetNode("/orientation/roll-deg", true);
    double roll = phi_node.getDouble();
    roll_squared->setDouble( roll * roll );
#endif

}


/*
 * Update the list of autopilot components
 */

void FGXMLAutopilot::update( double dt ) {
    update_helper( dt );

    unsigned int i;
    for ( i = 0; i < components.size(); ++i ) {
        components[i]->update( dt );
    }
}

