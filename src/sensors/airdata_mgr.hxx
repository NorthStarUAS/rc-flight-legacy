/**
 * \file: airdata_mgr.hxx
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 */


#ifndef _UGEAR_AIRDATA_MGR_HXX
#define _UGEAR_AIRDATA_MGR_HXX


void AirData_init();
bool AirData_update();
void AirData_close();


#endif // _UGEAR_AIRDATA_MGR_HXX
