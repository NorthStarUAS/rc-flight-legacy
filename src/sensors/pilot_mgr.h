/**
 * \file: pilot_mgr.h
 *
 * Front end management interface for reading pilot input.
 *
 * Copyright (C) 2010 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: airdata_mgr.h,v 1.1 2009/05/01 11:40:48 curt Exp $
 */


#ifndef _UGEAR_PILOT_INPUT_MGR_H
#define _UGEAR_PILOT_INPUT_MGR_H


void PilotInput_init();
bool PilotInput_update();
void PilotInput_close();


#endif // _UGEAR_PILOT_INPUT_MGR_H
