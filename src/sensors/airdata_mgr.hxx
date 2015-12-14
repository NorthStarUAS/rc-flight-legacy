/**
 * \file: airdata_mgr.hxx
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 */


#ifndef _AURA_AIRDATA_MGR_HXX
#define _AURA_AIRDATA_MGR_HXX


void AirData_init();
bool AirData_update();
void AirData_recalibrate();
void AirData_close();


#endif // _AURA_AIRDATA_MGR_HXX
