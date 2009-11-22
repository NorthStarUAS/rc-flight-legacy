/**
 * \file: act_mgr.h
 *
 * Front end management interface for output actuators
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: act_mgr.h,v 1.1 2009/04/20 18:38:12 curt Exp $
 */


#ifndef _UGEAR_ACT_MGR_H
#define _UGEAR_ACT_MGR_H

#include "include/globaldefs.h"

//
// global variables
//

/* extern struct servo servo_out; */


void Actuator_init();
bool Actuator_update();
void Actuator_close();


#endif // _UGEAR_ACT_MGR_H
