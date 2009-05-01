/**
 * \file: airdata_mgr.h
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: airdata_mgr.h,v 1.1 2009/05/01 11:40:48 curt Exp $
 */


#ifndef _UGEAR_AIRDATA_MGR_H
#define _UGEAR_AIRDATA_MGR_H


void AirData_init();
bool AirData_update();
void AirData_close();


#endif // _UGEAR_AIRDATA_MGR_H
