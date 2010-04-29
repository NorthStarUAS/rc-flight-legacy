//
// FILE: arduservo.hxx
// DESCRIPTION: interact with ardupilot based servo subsystem board
//

#ifndef _UGEAR_ARDUSERVO_HXX
#define _UGEAR_ARDUSERVO_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool arduservo_init( SGPropertyNode *config );
bool arduservo_update();
void arduservo_close();


#endif // _UGEAR_ARDUSERVO_HXX
