/**
 * \file: act_mgr.h
 *
 * Front end management interface for output actuators
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 * $Id: act_mgr.h,v 1.1 2009/04/20 18:38:12 curt Exp $
 */


#pragma once

void Actuator_init();
bool Actuator_update();
void Actuator_close();
