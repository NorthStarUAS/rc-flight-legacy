/**
 * \file: pilot_mgr.hxx
 *
 * Front end management interface for reading pilot input.
 *
 * Copyright (C) 2010 - Curtis L. Olson curtolson@gmail.com
 *
 */


#ifndef _AURA_PILOT_INPUT_MGR_HXX
#define _AURA_PILOT_INPUT_MGR_HXX


void PilotInput_init();
bool PilotInput_update();
void PilotInput_close();


#endif // _AURA_PILOT_INPUT_MGR_HXX
